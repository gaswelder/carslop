//! OSM PBF source extraction for the location compiler.
use super::{invalid, BoundingBox, Result};
use quick_xml::{
    events::{BytesEnd, BytesStart, Event},
    Writer,
};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    io::{BufWriter, Write},
    path::Path,
};

#[derive(Clone, Debug)]
struct Object {
    kind: String,
    id: i64,
    version: Option<i32>,
    xml: Vec<u8>,
    refs: Vec<i64>,
    members: Vec<(String, i64)>,
}
fn pbf_object(e: osmpbf::Element<'_>) -> Result<Object> {
    use osmpbf::{Element, RelMemberType};
    let mut o = Object {
        kind: String::new(),
        id: 0,
        version: None,
        xml: vec![],
        refs: vec![],
        members: vec![],
    };
    let mut tags = Vec::new();
    let mut coords = None;
    let mut roles = Vec::new();
    match e {
        Element::Node(n) => {
            o.kind = "node".into();
            o.id = n.id();
            o.version = n.info().version();
            coords = Some((n.lat(), n.lon()));
            tags.extend(n.tags());
        }
        Element::DenseNode(n) => {
            o.kind = "node".into();
            o.id = n.id();
            o.version = n.info().map(|i| i.version());
            coords = Some((n.lat(), n.lon()));
            tags.extend(n.tags());
        }
        Element::Way(w) => {
            o.kind = "way".into();
            o.id = w.id();
            o.version = w.info().version();
            o.refs.extend(w.refs());
            tags.extend(w.tags());
        }
        Element::Relation(r) => {
            o.kind = "relation".into();
            o.id = r.id();
            o.version = r.info().version();
            tags.extend(r.tags());
            for m in r.members() {
                let kind = match m.member_type {
                    RelMemberType::Node => "node",
                    RelMemberType::Way => "way",
                    RelMemberType::Relation => "relation",
                };
                o.members.push((kind.into(), m.member_id));
                roles.push(m.role()?.to_owned());
            }
        }
    }
    let mut w = Writer::new(Vec::new());
    let id = o.id.to_string();
    let version = o.version.map(|v| v.to_string());
    let mut start = BytesStart::new(&o.kind);
    start.push_attribute(("id", id.as_str()));
    if let Some(v) = &version {
        start.push_attribute(("version", v.as_str()));
    }
    let lat = coords.map(|c| format!("{:.7}", c.0));
    let lon = coords.map(|c| format!("{:.7}", c.1));
    if let (Some(lat), Some(lon)) = (&lat, &lon) {
        start.push_attribute(("lat", lat.as_str()));
        start.push_attribute(("lon", lon.as_str()));
    }
    w.write_event(Event::Start(start))?;
    for (k, v) in tags {
        let mut t = BytesStart::new("tag");
        t.push_attribute(("k", k));
        t.push_attribute(("v", v));
        w.write_event(Event::Empty(t))?;
    }
    for id in &o.refs {
        let value = id.to_string();
        let mut t = BytesStart::new("nd");
        t.push_attribute(("ref", value.as_str()));
        w.write_event(Event::Empty(t))?;
    }
    for ((kind, id), role) in o.members.iter().zip(roles.iter()) {
        let value = id.to_string();
        let mut t = BytesStart::new("member");
        t.push_attribute(("type", kind.as_str()));
        t.push_attribute(("ref", value.as_str()));
        t.push_attribute(("role", role.as_str()));
        w.write_event(Event::Empty(t))?;
    }
    w.write_event(Event::End(BytesEnd::new(&o.kind)))?;
    o.xml = w.into_inner();
    Ok(o)
}
fn scan(path: &Path, mut f: impl FnMut(osmpbf::Element<'_>) -> Result<()>) -> Result<()> {
    let mut result = Ok(());
    osmpbf::ElementReader::from_path(path)?.for_each(|e| {
        if result.is_ok() {
            result = f(e);
        }
    })?;
    result
}
pub fn extract_osmpbf(source: &Path, output: &Path, bounds: BoundingBox) -> Result<()> {
    println!("Extracting OSM data...");
    if !bounds.is_valid() {
        return Err(invalid("invalid SOUTH WEST NORTH EAST bounds"));
    }
    if source == output {
        return Err(invalid("output must differ from source"));
    }
    let mut timestamp = None;
    for blob in osmpbf::BlobReader::from_path(source)? {
        let blob = blob?;
        if let osmpbf::BlobDecode::OsmHeader(header) = blob.decode()? {
            timestamp = header
                .osmosis_replication_timestamp()
                .map(jiff::Timestamp::from_second)
                .transpose()?
                .map(|t| t.to_string());
            break;
        }
    }
    let mut inside = BTreeSet::new();
    println!("Selecting nodes...");
    scan(source, |e| {
        let n = match e {
            osmpbf::Element::Node(n) => Some((n.id(), n.lat(), n.lon())),
            osmpbf::Element::DenseNode(n) => Some((n.id(), n.lat(), n.lon())),
            _ => None,
        };
        if let Some((id, lat, lon)) = n {
            if lat >= bounds.south
                && lat <= bounds.north
                && lon >= bounds.west
                && lon <= bounds.east
            {
                inside.insert(id);
            }
        }
        Ok(())
    })?;
    let mut ways = BTreeMap::new();
    let mut relations = BTreeMap::new();
    let mut members = BTreeSet::new();
    scan(source, |e| {
        if let osmpbf::Element::Way(w) = e {
            if w.refs().any(|id| inside.contains(&id)) {
                ways.insert(w.id(), pbf_object(osmpbf::Element::Way(w))?);
            }
        }
        Ok(())
    })?;
    scan(source, |e| {
        if let osmpbf::Element::Relation(r) = e {
            if r.tags().any(|(k, v)| k == "type" && v == "multipolygon") {
                let o = pbf_object(osmpbf::Element::Relation(r))?;
                if o.members.iter().all(|(k, _)| k != "relation")
                    && o.members
                        .iter()
                        .any(|(k, id)| k == "way" && ways.contains_key(id))
                {
                    members.extend(
                        o.members
                            .iter()
                            .filter(|(k, _)| k == "way")
                            .map(|(_, id)| *id),
                    );
                    relations.insert(o.id, o);
                }
            }
        }
        Ok(())
    })?;
    scan(source, |e| {
        if let osmpbf::Element::Way(w) = e {
            if members.contains(&w.id()) && !ways.contains_key(&w.id()) {
                ways.insert(w.id(), pbf_object(osmpbf::Element::Way(w))?);
            }
        }
        Ok(())
    })?;
    if members.iter().any(|id| !ways.contains_key(id)) {
        return Err(invalid("missing multipolygon way member"));
    }
    let mut needed = inside;
    for w in ways.values() {
        needed.extend(&w.refs);
    }
    for r in relations.values() {
        needed.extend(
            r.members
                .iter()
                .filter(|(k, _)| k == "node")
                .map(|(_, id)| *id),
        );
    }
    // Stream nodes into a temporary spool; only selected ways/relations stay in memory.
    if let Some(parent) = output.parent().filter(|p| !p.as_os_str().is_empty()) {
        fs::create_dir_all(parent)?;
    }
    let spool_file = tempfile::NamedTempFile::new()?;
    let spool = spool_file.path();
    let result = (|| -> Result<()> {
        let mut nodes = BufWriter::new(spool_file.reopen()?);
        let mut count = 0;
        scan(source, |e| {
            let id = match &e {
                osmpbf::Element::Node(n) => Some(n.id()),
                osmpbf::Element::DenseNode(n) => Some(n.id()),
                _ => None,
            };
            if id.is_some_and(|id| needed.remove(&id)) {
                let o = pbf_object(e)?;
                nodes.write_all(&o.xml)?;
                nodes.write_all(b"\n")?;
                count += 1;
            }
            Ok(())
        })?;
        nodes.flush()?;
        if !needed.is_empty() {
            return Err(invalid("missing way/member node references"));
        }
        let mut out = BufWriter::new(fs::File::create(output)?);
        writeln!(out,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<osm version=\"0.6\" generator=\"nfs5like regional PBF extraction\">")?;
        if let Some(t) = &timestamp {
            writeln!(out, "<meta osm_base=\"{t}\"/>")?;
        }
        std::io::copy(&mut fs::File::open(spool)?, &mut out)?;
        for o in ways.values().chain(relations.values()) {
            out.write_all(&o.xml)?;
            out.write_all(b"\n")?;
        }
        writeln!(out, "</osm>")?;
        out.flush()?;
        fs::write(
            output.with_extension("json"),
            serde_json::to_vec_pretty(
                &serde_json::json!({"bbox":bounds.to_array(),"source":source.file_name().map(|s|s.to_string_lossy()),"source_timestamp":timestamp,"nodes":count,"ways":ways.len(),"multipolygons":relations.len()}),
            )?,
        )?;
        println!(
            "{count} nodes, {} ways, {} multipolygons",
            ways.len(),
            relations.len()
        );
        Ok(())
    })();
    result
}

#[cfg(test)]
mod pbf_tests {
    use super::*;
    fn var(mut n: u64) -> Vec<u8> {
        let mut b = vec![];
        while n >= 128 {
            b.push(n as u8 | 128);
            n >>= 7;
        }
        b.push(n as u8);
        b
    }
    fn uint(field: u64, n: u64) -> Vec<u8> {
        [var(field << 3), var(n)].concat()
    }
    fn bytes(field: u64, b: &[u8]) -> Vec<u8> {
        [var(field << 3 | 2), var(b.len() as u64), b.to_vec()].concat()
    }
    fn zig(n: i64) -> u64 {
        ((n << 1) ^ (n >> 63)) as u64
    }
    fn fixture(path: &Path) {
        let mut table = vec![];
        for s in ["", "type", "multipolygon", "outer", "name", "A & B"] {
            table.extend(bytes(1, s.as_bytes()));
        }
        let mut group = vec![];
        for (id, lat, lon) in [
            (1, 0, 0),
            (2, 20_000_000, 20_000_000),
            (3, 30_000_000, 30_000_000),
            (4, 40_000_000, 40_000_000),
        ] {
            let node = [
                uint(1, zig(id)),
                bytes(2, &var(4)),
                bytes(3, &var(5)),
                bytes(4, &uint(1, 7)),
                uint(8, zig(lat)),
                uint(9, zig(lon)),
            ]
            .concat();
            group.extend(bytes(1, &node));
        }
        for (id, refs) in [(10, vec![1, 2]), (11, vec![3, 4])] {
            let mut last = 0;
            let mut packed = vec![];
            for n in refs {
                packed.extend(var(zig(n - last)));
                last = n;
            }
            group.extend(bytes(
                3,
                &[uint(1, id), bytes(8, &packed), bytes(4, &uint(1, 3))].concat(),
            ));
        }
        let relation = [
            uint(1, 20),
            bytes(2, &var(1)),
            bytes(3, &var(2)),
            bytes(8, &[var(3), var(3)].concat()),
            bytes(9, &[var(zig(10)), var(zig(1))].concat()),
            bytes(10, &[1, 1]),
        ]
        .concat();
        group.extend(bytes(4, &relation));
        let block = [bytes(1, &table), bytes(2, &group)].concat();
        let blob = bytes(1, &block);
        let header = [bytes(1, b"OSMData"), uint(3, blob.len() as u64)].concat();
        let data = [(header.len() as u32).to_be_bytes().to_vec(), header, blob].concat();
        fs::write(path, data).unwrap();
    }
    #[test]
    fn extraction_completes_outside_ways_and_multipolygons() {
        let dir = tempfile::tempdir().unwrap();
        let pbf = dir.path().join("source.pbf");
        let output = dir.path().join("region.osm");
        fixture(&pbf);
        extract_osmpbf(
            &pbf,
            &output,
            BoundingBox {
                south: -0.1,
                west: -0.1,
                north: 0.1,
                east: 0.1,
            },
        )
        .unwrap();
        let source = crate::location_compiler::osm::read(&output).unwrap();
        assert_eq!(
            source.nodes.keys().copied().collect::<Vec<_>>(),
            vec![1, 2, 3, 4]
        );
        assert_eq!(
            source.ways.keys().copied().collect::<Vec<_>>(),
            vec![10, 11]
        );
        assert_eq!(source.relations[0].id, 20);
        assert_eq!(source.nodes[&1].tag("name"), "A & B");
        let xml = fs::read_to_string(&output).unwrap();
        assert!(xml.contains("version=\"7\""));
        assert!(xml.contains("A &amp; B"));
        let report: serde_json::Value =
            serde_json::from_slice(&fs::read(output.with_extension("json")).unwrap()).unwrap();
        assert_eq!(report["nodes"], 4);
        assert_eq!(report["multipolygons"], 1);
    }
}

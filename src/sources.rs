//! OSM PBF source extraction for the location compiler.
use crate::{
    location::{invalid, Result},
    world::BoundingBox,
};
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
pub fn extract_osm_boundary(source: &Path, output: &Path, bounds: BoundingBox) -> Result<()> {
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

//! Offline source preparation commands, built into the game executable.
use crate::location::{invalid, Result};
use quick_xml::{
    events::{BytesEnd, BytesStart, Event},
    Reader, Writer,
};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    io::{BufReader, BufWriter, Read, Write},
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
fn xml_objects(path: &Path) -> Result<Vec<Object>> {
    let file = fs::File::open(path)?;
    let input: Box<dyn Read> = if path.extension().is_some_and(|e| e == "gz") {
        Box::new(flate2::read::GzDecoder::new(file))
    } else {
        Box::new(file)
    };
    let mut r = Reader::from_reader(BufReader::new(input));
    let mut buf = Vec::new();
    let mut objects = Vec::new();
    let mut current: Option<(Object, Writer<Vec<u8>>)> = None;
    let mut root = false;
    let mut closed = false;
    loop {
        let event = r.read_event_into(&mut buf)?.into_owned();
        match &event {
            Event::Start(e) | Event::Empty(e) => {
                let name = e.name().as_ref().to_owned();
                if name == "remark" {
                    return Err(invalid("incomplete Overpass snapshot"));
                }
                if name == "osm" {
                    if root {
                        return Err(invalid("multiple OSM roots"));
                    }
                    root = true;
                    if matches!(event, Event::Empty(_)) {
                        closed = true;
                    }
                }
                if ["node", "way", "relation"].contains(&name.as_str()) {
                    if current.is_some() {
                        return Err(invalid("nested OSM object"));
                    }
                    let mut attrs = BTreeMap::new();
                    for a in e.attributes() {
                        let a = a?;
                        attrs.insert(
                            a.key.as_ref().to_owned(),
                            a.normalized_value(quick_xml::XmlVersion::Implicit1_0)?
                                .into_owned(),
                        );
                    }
                    let o = Object {
                        kind: name,
                        id: attrs
                            .get("id")
                            .ok_or_else(|| invalid("missing object id"))?
                            .parse()?,
                        version: attrs.get("version").map(|s| s.parse()).transpose()?,
                        xml: vec![],
                        refs: vec![],
                        members: vec![],
                    };
                    current = Some((o, Writer::new(Vec::new())));
                }
            }
            Event::End(e) if e.name().as_ref() == "osm" => closed = true,
            Event::Eof => break,
            _ => {}
        }
        if let Some((o, w)) = &mut current {
            w.write_event(event.clone())?;
            let done = match &event {
                Event::Empty(e) => e.name().as_ref() == o.kind.as_str(),
                Event::End(e) => e.name().as_ref() == o.kind.as_str(),
                _ => false,
            };
            if done {
                let (mut o, w) = current.take().unwrap();
                o.xml = w.into_inner();
                objects.push(o);
            }
        }
        buf.clear();
    }
    if !root || !closed || current.is_some() {
        return Err(invalid("incomplete OSM XML"));
    }
    Ok(objects)
}
fn write_objects(output: &Path, objects: impl IntoIterator<Item = Object>) -> Result<()> {
    let mut out = BufWriter::new(fs::File::create(output)?);
    writeln!(out,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<osm version=\"0.6\" generator=\"nfs5like offline source tools\">\n<note>OpenStreetMap contributors, ODbL. See sources/manifest.json.</note>")?;
    for o in objects {
        out.write_all(&o.xml)?;
        out.write_all(b"\n")?;
    }
    writeln!(out, "</osm>")?;
    out.flush()?;
    Ok(())
}
pub fn merge(original: &Path, extension: &Path, output: &Path) -> Result<()> {
    if output == original || output == extension {
        return Err(invalid("output must differ from inputs"));
    }
    let mut items = BTreeMap::new();
    for o in xml_objects(extension)?
        .into_iter()
        .chain(xml_objects(original)?)
    {
        let key = (
            match o.kind.as_str() {
                "node" => 0,
                "way" => 1,
                _ => 2,
            },
            o.id,
        );
        let replace = items.get(&key).is_none_or(|p: &Object| {
            p.version.is_none() || o.version.is_some_and(|v| Some(v) >= p.version)
        });
        if replace {
            items.insert(key, o);
        }
    }
    write_objects(output, items.into_values())
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
pub fn extract(source: &Path, output: &Path, b: [f64; 4]) -> Result<()> {
    if !b.iter().all(|v| v.is_finite())
        || b[0] >= b[2]
        || b[1] >= b[3]
        || b[0] < -90.
        || b[2] > 90.
        || b[1] < -180.
        || b[3] > 180.
    {
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
            if lat >= b[0] && lat <= b[2] && lon >= b[1] && lon <= b[3] {
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
                &serde_json::json!({"bbox":b,"source":source.file_name().map(|s|s.to_string_lossy()),"source_timestamp":timestamp,"nodes":count,"ways":ways.len(),"multipolygons":relations.len()}),
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

pub fn aerial(image: &Path, export: &Path, output: &Path, size: u32, gutter: u32) -> Result<()> {
    let im = image::ImageReader::open(image)?.decode()?.to_rgb8();
    let (width, height) = im.dimensions();
    if size == 0 || width == 0 || height == 0 || width % size != 0 || height % size != 0 {
        return Err(invalid(
            "image dimensions must be multiples of positive tile size",
        ));
    }
    let side = size
        .checked_add(
            gutter
                .checked_mul(2)
                .ok_or_else(|| invalid("gutter overflow"))?,
        )
        .ok_or_else(|| invalid("tile size overflow"))?;
    let len = (side as usize)
        .checked_mul(side as usize)
        .and_then(|v| v.checked_mul(3))
        .filter(|v| *v <= 256 * 1024 * 1024)
        .ok_or_else(|| invalid("tile exceeds 256 MiB"))?;
    let metadata = fs::read(export)?;
    let v: serde_json::Value = serde_json::from_slice(&metadata)?;
    let e = &v["extent"];
    let get = |k: &str| -> Result<f64> {
        e[k].as_f64()
            .filter(|v| v.is_finite())
            .ok_or_else(|| invalid(format!("invalid extent {k}")))
    };
    let (xmin, ymin, xmax, ymax) = (get("xmin")?, get("ymin")?, get("xmax")?, get("ymax")?);
    if xmin >= xmax || ymin >= ymax {
        return Err(invalid("invalid export extent"));
    }
    let local = |mx: f64, my: f64| {
        let lat = (2. * (my / 6378137.).exp().atan() - std::f64::consts::FRAC_PI_2).to_degrees();
        let lon = (mx / 6378137.).to_degrees();
        [
            (lon - 33.320).to_radians() * 6371000. * 35.162_f64.to_radians().cos(),
            -(lat - 35.162).to_radians() * 6371000.,
        ]
    };
    let nw = local(xmin, ymax);
    let se = local(xmax, ymin);
    let (nx, nz) = (width / size, height / size);
    fs::create_dir_all(output)?;
    let mut index = format!("AERIAL 1\nsize {size} {gutter}\n");
    for z in 0..nz {
        for x in 0..nx {
            let mut raw = Vec::with_capacity(len);
            for py in 0..side {
                for px in 0..side {
                    let sx = (x as i64 * size as i64 + px as i64 - gutter as i64)
                        .clamp(0, width as i64 - 1) as u32;
                    let sy = (z as i64 * size as i64 + py as i64 - gutter as i64)
                        .clamp(0, height as i64 - 1) as u32;
                    raw.extend_from_slice(&im.get_pixel(sx, sy).0);
                }
            }
            let name = format!("{x}_{z}.rgb");
            fs::write(output.join(&name), &raw)?;
            let bounds = [
                nw[0] + (se[0] - nw[0]) * x as f64 / nx as f64,
                nw[1] + (se[1] - nw[1]) * z as f64 / nz as f64,
                nw[0] + (se[0] - nw[0]) * (x + 1) as f64 / nx as f64,
                nw[1] + (se[1] - nw[1]) * (z + 1) as f64 / nz as f64,
            ];
            index.push_str(&format!(
                "tile {:.6} {:.6} {:.6} {:.6} {name} {:08x}\n",
                bounds[0],
                bounds[1],
                bounds[2],
                bounds[3],
                crate::street_surface::crc32_bytes(&raw)
            ));
        }
    }
    fs::write(output.join("source-export.json"), metadata)?;
    fs::write(output.join("tiles.txt"), index)?;
    println!("{} aerial tiles", nx * nz);
    Ok(())
}

pub fn run(args: Vec<std::ffi::OsString>) -> Result<()> {
    use std::path::PathBuf;
    let command = args[0].to_str().ok_or_else(|| invalid("invalid command"))?;
    let mut positional = Vec::new();
    let mut options = BTreeMap::new();
    let mut bbox = [35.10, 33.25, 35.23, 33.47];
    let mut seen_bbox = false;
    let mut values = args[1..].iter();
    while let Some(arg) = values.next() {
        if arg == "--bbox" && command == "extract-region" {
            if seen_bbox {
                return Err(invalid("duplicate --bbox"));
            }
            seen_bbox = true;
            for v in &mut bbox {
                *v = values
                    .next()
                    .and_then(|v| v.to_str())
                    .ok_or_else(|| invalid("--bbox requires SOUTH WEST NORTH EAST"))?
                    .parse()?;
            }
        } else if arg.to_string_lossy().starts_with('-') {
            let key = arg.to_str().ok_or_else(|| invalid("invalid option"))?;
            if command != "build-aerial-tiles"
                || ![
                    "--image",
                    "--export",
                    "--output",
                    "--size",
                    "--gutter",
                    "--preview-output",
                ]
                .contains(&key)
            {
                return Err(invalid(format!("unexpected option: {key}")));
            }
            let value = values
                .next()
                .ok_or_else(|| invalid(format!("{key} requires a value")))?;
            if options.insert(key, value).is_some() {
                return Err(invalid(format!("duplicate {key}")));
            }
        } else {
            positional.push(PathBuf::from(arg));
        }
    }
    match command {
        "merge-osm" if positional.len() == 3 => {
            merge(&positional[0], &positional[1], &positional[2])
        }
        "extract-region" if positional.len() == 2 => extract(&positional[0], &positional[1], bbox),
        "build-aerial-tiles" if positional.is_empty() => {
            let path = |key| -> Result<&Path> {
                options
                    .get(key)
                    .map(Path::new)
                    .ok_or_else(|| invalid(format!("missing {key}")))
            };
            let number = |key, default| -> Result<u32> {
                options
                    .get(key)
                    .map(|v| {
                        v.to_str()
                            .ok_or_else(|| invalid("invalid number"))?
                            .parse()
                            .map_err(Into::into)
                    })
                    .unwrap_or(Ok(default))
            };
            aerial(
                path("--image")?,
                path("--export")?,
                path("--output")?,
                number("--size", 512)?,
                number("--gutter", 8)?,
            )?;
            if let Some(dir) = options.get("--preview-output") {
                let dir = Path::new(dir);
                fs::create_dir_all(dir)?;
                let im = image::ImageReader::open(path("--image")?)?
                    .decode()?
                    .to_rgb8();
                let v: serde_json::Value = serde_json::from_slice(&fs::read(path("--export")?)?)?;
                let e = &v["extent"];
                fs::write(dir.join("image.rgb"), im.as_raw())?;
                fs::write(
                    dir.join("georef.txt"),
                    format!(
                        "{} {} {} {} {} {}\n",
                        im.width(),
                        im.height(),
                        e["xmin"],
                        e["ymin"],
                        e["xmax"],
                        e["ymax"]
                    ),
                )?;
            }
            Ok(())
        }
        _ => Err(invalid("invalid source tool arguments; see --help")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn merge_versions_gzip_tags_and_incomplete_inputs() {
        let dir = tempfile::tempdir().unwrap();
        let old = dir.path().join("old.gz");
        let ext = dir.path().join("extension.osm");
        let out = dir.path().join("out.osm");
        let mut gz = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::default());
        gz.write_all(br#"<osm><node id="1" version="2" lat="35.16" lon="33.32"><tag k="name" v="A &amp; B"/></node><node id="2" version="1"/></osm>"#).unwrap();
        fs::write(&old, gz.finish().unwrap()).unwrap();
        fs::write(&ext,br#"<osm><node id="1" lat="35.17"/><node id="2" version="3"/><way id="10"><nd ref="1"/></way></osm>"#).unwrap();
        merge(&old, &ext, &out).unwrap();
        let objects = xml_objects(&out).unwrap();
        assert_eq!(objects.len(), 3);
        assert_eq!(objects[0].version, Some(2));
        assert_eq!(objects[1].version, Some(3));
        assert!(String::from_utf8(objects[0].xml.clone())
            .unwrap()
            .contains("A &amp; B"));
        let first = fs::read(&out).unwrap();
        merge(&old, &ext, &out).unwrap();
        assert_eq!(first, fs::read(&out).unwrap());
        fs::write(&ext, "<osm><remark>timeout</remark></osm>").unwrap();
        assert!(merge(&old, &ext, &out).is_err());
        assert_eq!(first, fs::read(&out).unwrap());
        fs::write(&ext, "<osm><node id=\"1\"/>").unwrap();
        assert!(xml_objects(&ext).is_err());
    }
    #[test]
    fn aerial_gutters_checksums_and_validation() {
        let dir = tempfile::tempdir().unwrap();
        let image = dir.path().join("image.png");
        let export = dir.path().join("export.json");
        let out = dir.path().join("tiles");
        image::RgbImage::from_fn(128, 128, |x, y| {
            image::Rgb([x as u8, y as u8, (x + y) as u8])
        })
        .save(&image)
        .unwrap();
        fs::write(
            &export,
            r#"{"extent":{"xmin":3709000,"ymin":4185800,"xmax":3709100,"ymax":4185900}}"#,
        )
        .unwrap();
        aerial(&image, &export, &out, 64, 4).unwrap();
        let index = fs::read_to_string(out.join("tiles.txt")).unwrap();
        assert_eq!(index.lines().count(), 6);
        for line in index.lines().skip(2) {
            let f: Vec<_> = line.split_whitespace().collect();
            let raw = fs::read(out.join(f[5])).unwrap();
            assert_eq!(raw.len(), 72 * 72 * 3);
            assert_eq!(
                crate::street_surface::crc32_bytes(&raw),
                u32::from_str_radix(f[6], 16).unwrap()
            );
        }
        let a = fs::read(out.join("0_0.rgb")).unwrap();
        let b = fs::read(out.join("1_0.rgb")).unwrap();
        for y in 0..72 {
            assert_eq!(
                &a[(y * 72 + 64) * 3..(y * 72 + 72) * 3],
                &b[y * 72 * 3..(y * 72 + 8) * 3]
            );
        }
        assert_eq!(&a[..3], &[0, 0, 0]);
        assert!(aerial(&image, &export, &out, 0, 4).is_err());
        assert!(aerial(&image, &export, &out, 63, 4).is_err());
    }
    #[test]
    fn source_command_arguments() {
        let args = |a: &[&str]| run(a.iter().map(Into::into).collect());
        assert!(args(&["extract-region", "a", "b", "--bbox", "NaN", "0", "1", "1"]).is_err());
        assert!(args(&["merge-osm", "a", "b"]).is_err());
        assert!(args(&["build-aerial-tiles", "--image", "a", "--image", "b"]).is_err());
    }
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
        extract(&pbf, &output, [-0.1, -0.1, 0.1, 0.1]).unwrap();
        let o = xml_objects(&output).unwrap();
        assert_eq!(
            o.iter()
                .filter(|o| o.kind == "node")
                .map(|o| o.id)
                .collect::<Vec<_>>(),
            vec![1, 2, 3, 4]
        );
        assert_eq!(
            o.iter()
                .filter(|o| o.kind == "way")
                .map(|o| o.id)
                .collect::<Vec<_>>(),
            vec![10, 11]
        );
        assert_eq!(o.last().unwrap().id, 20);
        assert_eq!(o[0].version, Some(7));
        assert!(String::from_utf8(o[0].xml.clone())
            .unwrap()
            .contains("A &amp; B"));
        let report: serde_json::Value =
            serde_json::from_slice(&fs::read(output.with_extension("json")).unwrap()).unwrap();
        assert_eq!(report["nodes"], 4);
        assert_eq!(report["multipolygons"], 1);
    }
}

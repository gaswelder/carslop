use crate::{
    location::{add, invalid, mul, sub, Result},
    world::P,
};
use quick_xml::{
    events::{BytesStart, Event},
    Reader,
};
use std::collections::BTreeMap;
use std::fs;
use std::io::BufReader;
use std::path::Path;

#[derive(Clone, Default)]
pub struct OSMElement {
    pub id: i64,
    pub tags: BTreeMap<String, String>,
    pub coord: Option<P>,
    pub nodes: Vec<i64>,
    pub members: Vec<(String, i64, String)>,
}
impl OSMElement {
    pub fn tag(&self, key: &str) -> &str {
        self.tags.get(key).map(String::as_str).unwrap_or("")
    }
}

pub struct OSMMap {
    pub nodes: BTreeMap<i64, OSMElement>,
    pub ways: BTreeMap<i64, OSMElement>,
    pub relations: Vec<OSMElement>,
    pub bounds: Option<[f64; 4]>,
}

fn attrs(e: &BytesStart<'_>) -> Result<BTreeMap<String, String>> {
    e.attributes()
        .map(|a| {
            let a = a?;
            Ok((
                a.key.as_ref().to_owned(),
                a.normalized_value(quick_xml::XmlVersion::Implicit1_0)?
                    .into_owned(),
            ))
        })
        .collect()
}

pub fn read(path: &Path) -> Result<OSMMap> {
    let mut reader = Reader::from_reader(BufReader::new(fs::File::open(path)?));
    let mut buffer = Vec::new();
    let mut current: Option<(String, OSMElement)> = None;
    let mut source = OSMMap {
        nodes: BTreeMap::new(),
        ways: BTreeMap::new(),
        relations: vec![],
        bounds: None,
    };
    let mut root = false;
    let mut closed = false;
    loop {
        match reader.read_event_into(&mut buffer)? {
            Event::Start(e) | Event::Empty(e) => {
                let name = e.name().as_ref().to_owned();
                if name == "remark" {
                    return Err(invalid(
                        "OSM export contains an error remark; refusing incomplete data",
                    ));
                }
                if name == "osm" {
                    if root {
                        return Err(invalid("multiple OSM roots"));
                    }
                    root = true;
                } else if name == "bounds" {
                    let a = attrs(&e)?;
                    source.bounds = Some([
                        a.get("minlat")
                            .ok_or_else(|| invalid("missing minlat"))?
                            .parse()?,
                        a.get("minlon")
                            .ok_or_else(|| invalid("missing minlon"))?
                            .parse()?,
                        a.get("maxlat")
                            .ok_or_else(|| invalid("missing maxlat"))?
                            .parse()?,
                        a.get("maxlon")
                            .ok_or_else(|| invalid("missing maxlon"))?
                            .parse()?,
                    ]);
                } else if name == "meta" {
                    //
                } else if matches!(name.as_str(), "node" | "way" | "relation") {
                    // Flush self-closing elements when the next element begins.
                    flush(&mut current, &mut source);
                    let a = attrs(&e)?;
                    let id: i64 = a
                        .get("id")
                        .ok_or_else(|| invalid("missing OSM id"))?
                        .parse()?;
                    if id <= 0 {
                        return Err(invalid(
                            "OSM IDs must be positive; save an export with published object IDs",
                        ));
                    }
                    let coord: Option<P> = if name == "node" {
                        Some([
                            a.get("lat")
                                .ok_or_else(|| invalid("missing latitude"))?
                                .parse()?,
                            a.get("lon")
                                .ok_or_else(|| invalid("missing longitude"))?
                                .parse()?,
                        ])
                    } else {
                        None
                    };
                    if let Some(p) = coord {
                        if !p[0].is_finite()
                            || !p[1].is_finite()
                            || p[0].abs() > 90.0
                            || p[1].abs() > 180.0
                        {
                            return Err(invalid("invalid OSM coordinate"));
                        }
                    }
                    current = Some((
                        name,
                        OSMElement {
                            id,
                            coord,
                            ..Default::default()
                        },
                    ));
                } else if let Some((_, element)) = &mut current {
                    let a = attrs(&e)?;
                    match name.as_str() {
                        "tag" => {
                            element.tags.insert(
                                a.get("k")
                                    .ok_or_else(|| invalid("missing tag key"))?
                                    .clone(),
                                a.get("v")
                                    .ok_or_else(|| invalid("missing tag value"))?
                                    .clone(),
                            );
                        }
                        "nd" => element.nodes.push(
                            a.get("ref")
                                .ok_or_else(|| invalid("missing node reference"))?
                                .parse()?,
                        ),
                        "member" => element.members.push((
                            a.get("type").cloned().unwrap_or_default(),
                            a.get("ref")
                                .ok_or_else(|| invalid("missing member reference"))?
                                .parse()?,
                            a.get("role").cloned().unwrap_or_default(),
                        )),
                        _ => {}
                    }
                }
            }
            Event::End(e) if matches!(e.name().as_ref(), "node" | "way" | "relation") => {
                flush(&mut current, &mut source)
            }
            Event::End(e) if e.name().as_ref() == "osm" => {
                closed = true;
            }
            Event::Eof => break,
            _ => {}
        }
        buffer.clear();
    }
    flush(&mut current, &mut source);
    if !root || !closed || source.nodes.is_empty() {
        return Err(invalid(
            "expected an OSM XML file containing nodes (.pbf is not supported)",
        ));
    }
    Ok(source)
}

fn flush(current: &mut Option<(String, OSMElement)>, source: &mut OSMMap) {
    if let Some((kind, e)) = current.take() {
        match kind.as_str() {
            "node" => {
                source.nodes.insert(e.id, e);
            }
            "way" => {
                source.ways.insert(e.id, e);
            }
            _ => source.relations.push(e),
        }
    }
}
pub fn number(s: &str) -> Option<f64> {
    let s = s.trim().strip_suffix('m').unwrap_or(s.trim()).trim();
    if s.is_empty() || s.chars().any(|c| !c.is_ascii_digit() && c != '.') {
        return None;
    }
    s.parse::<f64>().ok().filter(|n| n.is_finite())
}
pub fn clip(a: P, b: P, bounds: [f64; 4]) -> Option<(P, P)> {
    let d = sub(b, a);
    let (mut lo, mut hi) = (0.0_f64, 1.0_f64);
    for (p, q) in [
        (-d[0], a[0] - bounds[0]),
        (d[0], bounds[2] - a[0]),
        (-d[1], a[1] - bounds[1]),
        (d[1], bounds[3] - a[1]),
    ] {
        if p.abs() < 1e-12 {
            if q < 0.0 {
                return None;
            }
        } else if p < 0.0 {
            lo = lo.max(q / p);
        } else {
            hi = hi.min(q / p);
        }
        if lo > hi {
            return None;
        }
    }
    Some((add(a, mul(d, lo)), add(a, mul(d, hi))))
}

use super::*;
use quick_xml::{
    events::{BytesStart, Event},
    Reader,
};
use std::{
    collections::{BTreeSet, HashMap},
    io::BufReader,
};

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
pub fn read(path: &Path) -> Result<Source> {
    let mut reader = Reader::from_reader(BufReader::new(fs::File::open(path)?));
    let mut buffer = Vec::new();
    let mut current: Option<(String, Element)> = None;
    let mut source = Source {
        nodes: BTreeMap::new(),
        ways: BTreeMap::new(),
        relations: vec![],
        bounds: None,
        timestamp: "not supplied by export API".into(),
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
                    let a = attrs(&e)?;
                    if let Some(t) = a.get("timestamp") {
                        source.timestamp = t.clone();
                    }
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
                    let a = attrs(&e)?;
                    if let Some(t) = a.get("osm_base") {
                        source.timestamp = t.clone();
                    }
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
                        Element {
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
fn flush(current: &mut Option<(String, Element)>, source: &mut Source) {
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
pub fn compile(source: &Source, bbox: Option<[f64; 4]>) -> Result<Map> {
    let bbox = bbox.or(source.bounds).unwrap_or_else(|| {
        let mut b = [90.0_f64, 180.0_f64, -90.0_f64, -180.0_f64];
        for e in source.nodes.values() {
            let p = e.coord.unwrap();
            b[0] = b[0].min(p[0]);
            b[1] = b[1].min(p[1]);
            b[2] = b[2].max(p[0]);
            b[3] = b[3].max(p[1]);
        }
        b
    });
    if bbox.iter().any(|v| !v.is_finite())
        || bbox[0] >= bbox[2]
        || bbox[1] >= bbox[3]
        || bbox[0] < -90.0
        || bbox[2] > 90.0
        || bbox[1] < -180.0
        || bbox[3] > 180.0
    {
        return Err(invalid("invalid bounding box (south west north east)"));
    }
    let origin = [(bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0];
    if origin[0].abs() > 85.0 {
        return Err(invalid(
            "locations within 5 degrees of the poles are unsupported",
        ));
    }
    let mut map = Map {
        origin,
        bounds: [0.0; 4],
        nodes: vec![],
        roads: vec![],
        buildings: vec![],
    };
    let a = map.metres([bbox[0], bbox[1]]);
    let b = map.metres([bbox[2], bbox[3]]);
    map.bounds = [a[0], b[1], b[0], a[1]];
    // Bound geometry and height-field allocations before processing an accidental country extract.
    if b[0] - a[0] > 80000.0 || a[1] - b[1] > 80000.0 {
        return Err(invalid(
            "crop exceeds 80 km; supply a smaller --bbox SOUTH WEST NORTH EAST",
        ));
    }
    let coords: BTreeMap<_, _> = source
        .nodes
        .iter()
        .map(|(id, e)| (*id, map.metres(e.coord.unwrap())))
        .collect();
    let mut positions = HashMap::new();
    let major = [
        "motorway",
        "trunk",
        "motorway_link",
        "trunk_link",
        "primary",
        "secondary",
        "tertiary",
        "primary_link",
        "secondary_link",
        "tertiary_link",
    ];
    for way in source.ways.values() {
        let kind = way.tag("highway");
        if !major.contains(&kind)
            && !["unclassified", "residential", "living_street", "service"].contains(&kind)
        {
            continue;
        }
        if ["access", "vehicle", "motor_vehicle"]
            .iter()
            .any(|k| matches!(way.tag(k), "no" | "private"))
            || way.tag("area") == "yes"
            || ["bridge", "tunnel"]
                .iter()
                .any(|k| !matches!(way.tag(k), "" | "no"))
            || !matches!(way.tag("layer"), "" | "0")
        {
            continue;
        }
        let default = match kind {
            "motorway" => 12.0,
            "trunk" => 11.0,
            "primary" => 10.0,
            "secondary" => 9.0,
            "tertiary" => 8.0,
            "service" | "living_street" => 5.5,
            _ => 6.5,
        };
        let width = number(way.tag("width"))
            .unwrap_or_else(|| {
                let w = number(way.tag("lanes"))
                    .filter(|n| *n > 0.0)
                    .map(|n| n * 3.1)
                    .unwrap_or(default);
                if major.contains(&kind) {
                    (w + 0.9).max(8.0)
                } else {
                    w
                }
            })
            .clamp(5.5, 18.0);
        let direction = if way.tag("oneway") == "-1" {
            -1
        } else if matches!(way.tag("oneway"), "yes" | "1" | "true")
            || way.tag("junction") == "roundabout"
        {
            1
        } else {
            0
        };
        for pair in way.nodes.windows(2) {
            let (Some(a), Some(b)) = (coords.get(&pair[0]), coords.get(&pair[1])) else {
                continue;
            };
            let Some((a, b)) = clip(*a, *b, map.bounds) else {
                continue;
            };
            if dist(a, b) < 0.5 {
                continue;
            }
            let mut ends = [0; 2];
            for (i, p) in [a, b].iter().enumerate() {
                let key = (
                    (p[0] * 1000.0).round() as i64,
                    (p[1] * 1000.0).round() as i64,
                );
                ends[i] = *positions.entry(key).or_insert_with(|| {
                    let id = map.nodes.len();
                    map.nodes
                        .push([key.0 as f64 / 1000.0, key.1 as f64 / 1000.0]);
                    id
                });
            }
            map.roads.push(Road {
                ends,
                width,
                direction,
                way: way.id,
                kind: if major.contains(&kind) {
                    0
                } else if matches!(kind, "service" | "living_street") {
                    2
                } else {
                    1
                },
                lanes: (number(way.tag("lanes")).unwrap_or(if direction == 0 { 2.0 } else { 1.0 })
                    as usize)
                    .clamp(1, 6),
            });
        }
    }
    if map.roads.is_empty() {
        return Err(invalid(
            "no public, ground-level drivable roads in this crop",
        ));
    }
    map.roads.sort_by(|a, b| {
        let key = |r: &Road| {
            let a = map.nodes[r.ends[0]];
            let b = map.nodes[r.ends[1]];
            (
                dist(a, b) < 25.0,
                length(mul(add(a, b), 0.5)),
                r.way,
                r.ends[0],
            )
        };
        let a = key(a);
        let b = key(b);
        a.0.cmp(&b.0)
            .then(a.1.total_cmp(&b.1))
            .then(a.2.cmp(&b.2))
            .then(a.3.cmp(&b.3))
    });
    let complex: BTreeSet<_> = source
        .relations
        .iter()
        .filter(|e| !e.tag("building").is_empty() && e.members.iter().any(|m| m.2 == "inner"))
        .flat_map(|e| e.members.iter().filter(|m| m.0 == "way").map(|m| m.1))
        .collect();
    for way in source.ways.values() {
        if matches!(way.tag("building"), "" | "no")
            || complex.contains(&way.id)
            || way.nodes.len() < 4
            || way.nodes.first() != way.nodes.last()
        {
            continue;
        }
        let Some(mut points) = way.nodes[..way.nodes.len() - 1]
            .iter()
            .map(|id| coords.get(id).copied())
            .collect::<Option<Vec<_>>>()
        else {
            continue;
        };
        let original = points.clone();
        points = original
            .iter()
            .enumerate()
            .filter(|(i, p)| dist(**p, original[(*i + original.len() - 1) % original.len()]) > 0.05)
            .map(|(_, p)| *p)
            .collect();
        if points.len() < 3 || points.iter().any(|p| !map.contains(*p)) {
            continue;
        }
        let area = points
            .iter()
            .zip(points.iter().cycle().skip(1))
            .take(points.len())
            .map(|(a, b)| cross(*a, *b))
            .sum::<f64>()
            / 2.0;
        if area.abs() < 4.0 {
            continue;
        }
        if area < 0.0 {
            points.reverse();
        }
        let height = number(way.tag("height"));
        map.buildings.push(Building {
            id: way.id,
            height: height
                .unwrap_or_else(|| {
                    number(way.tag("building:levels"))
                        .filter(|n| *n > 0.0)
                        .map(|n| n * 3.1)
                        .unwrap_or(
                            if matches!(way.tag("building"), "apartments" | "commercial" | "office")
                            {
                                9.3
                            } else {
                                6.2
                            },
                        )
                })
                .clamp(2.5, 70.0),
            tagged: height.is_some(),
            points,
        });
    }
    Ok(map)
}
pub fn write(map: &Map, output: &Path) -> Result<String> {
    use std::fmt::Write;
    let mut text = format!(
        "CITY 2\norigin {} {}\nbounds {:.3} {:.3} {:.3} {:.3}\n",
        map.origin[0], map.origin[1], map.bounds[0], map.bounds[1], map.bounds[2], map.bounds[3]
    );
    for p in &map.nodes {
        writeln!(text, "n {:.3} {:.3}", p[0], p[1])?;
    }
    for r in &map.roads {
        writeln!(
            text,
            "r {} {} {:.2} {} {} {} {}",
            r.ends[0], r.ends[1], r.width, r.direction, r.way, r.kind, r.lanes
        )?;
    }
    for b in &map.buildings {
        write!(
            text,
            "b {} {:.2} {} {}",
            b.id,
            b.height,
            u8::from(b.tagged),
            b.points.len()
        )?;
        for p in &b.points {
            write!(text, " {:.3} {:.3}", p[0], p[1])?;
        }
        text.push('\n');
    }
    fs::write(output.join("city.txt"), &text)?;
    Ok(text)
}

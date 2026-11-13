use crate::geometry::{add, mul, sub};
use crate::geometry::{BoundingBox, BoundingBoxXZ, LatLon};
use crate::{
    error::{invalid, Result},
    geometry::XZ,
};
use geo::{BoundingRect, Coord, MapCoords, MultiPolygon};
use quick_xml::{
    events::{BytesStart, Event},
    Reader,
};
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::fs;
use std::io::BufReader;
use std::path::Path;

pub(super) fn relation_id(url: &str) -> Result<i64> {
    let rest = url
        .trim()
        .strip_prefix("https://")
        .or_else(|| url.trim().strip_prefix("http://"))
        .ok_or_else(|| invalid("expected an OpenStreetMap relation URL"))?;
    let (host, path) = rest
        .split_once('/')
        .ok_or_else(|| invalid("expected an OpenStreetMap relation URL"))?;
    if host != "www.openstreetmap.org" && host != "openstreetmap.org" {
        return Err(invalid("expected an openstreetmap.org URL"));
    }
    let path = path.split(['?', '#']).next().unwrap().trim_end_matches('/');
    let id = path
        .strip_prefix("relation/")
        .filter(|s| !s.is_empty() && s.bytes().all(|b| b.is_ascii_digit()))
        .and_then(|s| s.parse::<i64>().ok())
        .filter(|id| *id > 0)
        .ok_or_else(|| invalid("expected /relation/<positive ID> in the OSM URL"))?;
    Ok(id)
}

pub(super) fn import_name(relation: &OSMElement) -> String {
    let name = [relation.tag("name:en"), relation.tag("name")]
        .into_iter()
        .find(|name| !name.trim().is_empty())
        .unwrap_or("");
    let slug: String = name
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() {
                c.to_ascii_lowercase()
            } else {
                '-'
            }
        })
        .collect();
    let slug = slug
        .split('-')
        .filter(|s| !s.is_empty())
        .collect::<Vec<_>>()
        .join("-");
    let slug = slug
        .get(..slug.len().min(40))
        .unwrap()
        .trim_end_matches('-');
    // IDs keep equal names distinct and provide a fallback for non-ASCII names.
    format!(
        "{}-{}",
        if slug.is_empty() { "relation" } else { slug },
        relation.id
    )
}

/// Source OSM node, way or relation; applicable fields depend on the element kind.
#[derive(Clone, Default)]
pub struct OSMElement {
    /// OSM ID within the node, way or relation namespace.
    pub id: i64,
    /// Source tag names and values.
    tags: BTreeMap<String, String>,
    /// Node position as [latitude, longitude] degrees; absent for ways and relations.
    pub coord: Option<LatLon>,
    /// Ordered node IDs forming a way; empty for other element kinds.
    pub nodes: Vec<i64>,
    /// Relation members as (element kind, OSM ID, role), in source order.
    pub members: Vec<(String, i64, String)>,
}

impl OSMElement {
    pub fn tag(&self, key: &str) -> &str {
        self.tags.get(key).map(String::as_str).unwrap_or("")
    }
}

/// Parsed OSM extract retaining source elements and optional geographic bounds.
pub struct OSMMap {
    /// Node elements keyed by OSM node ID.
    pub nodes: BTreeMap<i64, OSMElement>,
    /// Way elements keyed by OSM way ID.
    pub ways: BTreeMap<i64, OSMElement>,
    /// Relation elements in source order.
    pub relations: Vec<OSMElement>,
    /// Optional extract bounds as [south, west, north, east] in degrees.
    pub bounds: Option<[f64; 4]>,
}

/// Administrative boundary resolved from an OSM relation, including holes and islands.
pub struct OSMBoundary {
    /// OSM relation ID identifying this boundary.
    pub relation: i64,
    /// Boundary polygons in geographic [longitude, latitude] degrees.
    pub geometry: MultiPolygon<f64>,
}

impl OSMBoundary {
    pub fn bbox(&self) -> BoundingBox {
        let b = self.geometry.bounding_rect().unwrap();
        BoundingBox {
            min_lat: b.min().y,
            min_lon: b.min().x,
            max_lat: b.max().y,
            max_lon: b.max().x,
        }
    }
    pub fn local(&self, origin: LatLon) -> MultiPolygon<f64> {
        self.geometry.map_coords(|c| Coord {
            x: (c.x - origin.lon).to_radians() * 6371000.0 * origin.lat.to_radians().cos(),
            y: -(c.y - origin.lat).to_radians() * 6371000.0,
        })
    }
    pub fn to_geojson(&self) -> Value {
        let coordinates: Vec<Vec<Vec<[f64; 2]>>> = self
            .geometry
            .0
            .iter()
            .map(|p| {
                std::iter::once(p.exterior())
                    .chain(p.interiors())
                    .map(|r| r.0.iter().map(|c| [c.x, c.y]).collect())
                    .collect()
            })
            .collect();
        json!({"type":"Feature","properties":{"osm_relation":self.relation},"geometry":{"type":"MultiPolygon","coordinates":coordinates}})
    }
    // Offline fixture tools read persisted boundaries; live imports pass typed geometry.

    // Returns the boundary's width and height in meters.
    pub fn size(&self) -> (f64, f64) {
        let width = (self.bbox().max_lon - self.bbox().min_lon).to_radians()
            * 6371000.
            * ((self.bbox().min_lat + self.bbox().max_lat) / 2.)
                .to_radians()
                .cos();
        let height = (self.bbox().max_lat - self.bbox().min_lat).to_radians() * 6371000.;

        return (width, height);
    }
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
                    let coord: Option<LatLon> = if name == "node" {
                        Some(LatLon::new(
                            a.get("lat")
                                .ok_or_else(|| invalid("missing latitude"))?
                                .parse()?,
                            a.get("lon")
                                .ok_or_else(|| invalid("missing longitude"))?
                                .parse()?,
                        ))
                    } else {
                        None
                    };
                    if let Some(p) = coord {
                        if !p.lat.is_finite()
                            || !p.lon.is_finite()
                            || p.lat.abs() > 90.0
                            || p.lon.abs() > 180.0
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
pub fn clip(a: XZ, b: XZ, bounds: BoundingBoxXZ) -> Option<(XZ, XZ)> {
    let d = sub(b, a);
    let (mut lo, mut hi) = (0.0_f64, 1.0_f64);
    for (p, q) in [
        (-d.x, a.x - bounds.min.x),
        (d.x, bounds.max.x - a.x),
        (-d.z, a.z - bounds.min.z),
        (d.z, bounds.max.z - a.z),
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn relation_urls_ignore_viewport_and_query() {
        for url in [
            "https://www.openstreetmap.org/relation/2087230#map=11/34.7977/32.9562",
            "http://openstreetmap.org/relation/2087230/?foo=bar#map=1/0/0",
        ] {
            assert_eq!(relation_id(url).unwrap(), 2087230);
        }
        for url in [
            "https://openstreetmap.org.evil.test/relation/1",
            "https://openstreetmap.org@evil.test/relation/1",
            "https://openstreetmap.org/way/1",
            "https://openstreetmap.org/relation/0",
            "https://openstreetmap.org/relation/-1",
            "https://openstreetmap.org/relation/1/full",
            "https://openstreetmap.org/relation/99999999999999999999999",
        ] {
            assert!(relation_id(url).is_err(), "{url}");
        }
    }

    #[test]
    fn names_use_requested_relation_tags_and_have_safe_fallbacks() {
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("boundary.osm");
        fs::write(
            &path,
            r#"<osm version="0.6">
<node id="1" lat="34.7" lon="33.0"/>
<relation id="1"><tag k="name" v="Λεμεσός"/><tag k="name:en" v="Limassol District"/></relation>
<relation id="2"><tag k="name" v="../Some / Place"/></relation>
<relation id="3"><tag k="name" v="Λεμεσός"/></relation>
<relation id="4"/>
<relation id="5"><tag k="name:en" v=" "/><tag k="name" v="Local Name"/></relation>
</osm>"#,
        )
        .unwrap();
        let map = read(&path).unwrap();
        let names: Vec<_> = map.relations.iter().map(import_name).collect();
        assert_eq!(
            names,
            [
                "limassol-district-1",
                "some-place-2",
                "relation-3",
                "relation-4",
                "local-name-5"
            ]
        );
    }
}

use crate::util;
use crate::util::error::invalid;
use crate::util::error::Result;
use crate::util::geometry::add;
use crate::util::geometry::mul;
use crate::util::geometry::sub;
use crate::util::geometry::BoundingBox;
use crate::util::geometry::BoundingBoxXZ;
use crate::util::geometry::LatLon;
use crate::util::geometry::XZ;
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

/// A rectangle centered on the map coordinates, with dimensions in kilometers.
#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct ImportArea {
    pub latitude: f64,
    pub longitude: f64,
    pub width_km: f64,
    pub height_km: f64,
}

pub fn from_url(url: &str, width_km: f64, height_km: f64) -> Result<ImportArea> {
    let fragment = url
        .trim()
        .split_once('#')
        .map(|(_, fragment)| fragment)
        .ok_or_else(|| invalid("expected #map=zoom/latitude/longitude in the URL"))?;
    let map = fragment
        .split('&')
        .find_map(|part| part.strip_prefix("map="))
        .ok_or_else(|| invalid("expected #map=zoom/latitude/longitude in the URL"))?;
    let parts: Vec<_> = map.split('/').collect();
    if parts.len() != 3 {
        return Err(invalid("expected #map=zoom/latitude/longitude in the URL"));
    }
    let area = ImportArea {
        latitude: parts[1].parse()?,
        longitude: parts[2].parse()?,
        width_km,
        height_km,
    };
    geometry(&area)?;
    Ok(area)
}

pub fn name(import_area: &ImportArea) -> String {
    format!(
        "location-{}-{}-{}x{}km",
        import_area.latitude, import_area.longitude, import_area.width_km, import_area.height_km
    )
}

pub fn geometry(import_area: &ImportArea) -> Result<OSMArea> {
    if !import_area.latitude.is_finite()
        || !import_area.longitude.is_finite()
        || import_area.latitude.abs() > 85.0
        || import_area.longitude.abs() > 180.0
    {
        return Err(invalid(
            "invalid center coordinates; latitude must be within 85 degrees of the equator",
        ));
    }
    if !import_area.width_km.is_finite()
        || !import_area.height_km.is_finite()
        || import_area.width_km <= 0.0
        || import_area.height_km <= 0.0
        || import_area.width_km > 80.0
        || import_area.height_km > 80.0
    {
        return Err(invalid(
            "width and height must be positive and at most 80 km",
        ));
    }
    let half_lat = (import_area.height_km * 500.0 / 6371000.0).to_degrees();
    let half_lon = (import_area.width_km * 500.0
        / (6371000.0 * import_area.latitude.to_radians().cos()))
    .to_degrees();
    let bounds = BoundingBox {
        min_lat: import_area.latitude - half_lat,
        max_lat: import_area.latitude + half_lat,
        min_lon: import_area.longitude - half_lon,
        max_lon: import_area.longitude + half_lon,
    };
    if !util::geometry::is_valid_for_imagery(&bounds) {
        return Err(invalid(
            "requested rectangle crosses the antimeridian or the supported latitude limits",
        ));
    }
    Ok(OSMArea {
        geometry: MultiPolygon(vec![geo::Rect::new(
            Coord {
                x: bounds.min_lon,
                y: bounds.min_lat,
            },
            Coord {
                x: bounds.max_lon,
                y: bounds.max_lat,
            },
        )
        .to_polygon()]),
    })
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

pub fn tag<'a>(osm_element: &'a OSMElement, key: &str) -> &'a str {
    osm_element.tags.get(key).map(String::as_str).unwrap_or("")
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

/// Geographic area used to crop imported map data.
pub struct OSMArea {
    /// Boundary polygons in geographic [longitude, latitude] degrees.
    pub geometry: MultiPolygon<f64>,
}

pub fn bbox(osm_area: &OSMArea) -> BoundingBox {
    let b = osm_area.geometry.bounding_rect().unwrap();
    BoundingBox {
        min_lat: b.min().y,
        min_lon: b.min().x,
        max_lat: b.max().y,
        max_lon: b.max().x,
    }
}
pub fn local(osm_area: &OSMArea, origin: LatLon) -> MultiPolygon<f64> {
    osm_area.geometry.map_coords(|c| Coord {
        x: (c.x - origin.lon).to_radians() * 6371000.0 * origin.lat.to_radians().cos(),
        y: -(c.y - origin.lat).to_radians() * 6371000.0,
    })
}
pub fn to_geojson(osm_area: &OSMArea) -> Value {
    let coordinates: Vec<Vec<Vec<[f64; 2]>>> = osm_area
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
    json!({"type":"Feature","properties":{},"geometry":{"type":"MultiPolygon","coordinates":coordinates}})
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
    let map = read_xml(BufReader::new(fs::File::open(path)?))?;
    if map.nodes.is_empty() {
        return Err(invalid(
            "expected an OSM XML file containing nodes (.pbf is not supported)",
        ));
    }
    Ok(map)
}

pub fn read_xml(source: impl std::io::BufRead) -> Result<OSMMap> {
    let mut reader = Reader::from_reader(source);
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
                        Some(util::geometry::new(
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
    if !root || !closed {
        return Err(invalid("expected an OSM XML file (.pbf is not supported)"));
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
pub fn relation_feature_id(id: i64) -> u64 {
    (1_u64 << 62) | id as u64
}
pub fn building_source(source: &OSMMap, id: u64) -> Option<&OSMElement> {
    if id & (1_u64 << 62) != 0 {
        source
            .relations
            .iter()
            .find(|relation| relation_feature_id(relation.id) == id)
    } else {
        source.ways.get(&(id as i64))
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

/// Roof storeys are separate from building:levels. Explicit height already includes them.
pub fn roof_height(element: &OSMElement) -> f64 {
    number(tag(element, "roof:height"))
        .or_else(|| number(tag(element, "roof:levels")).map(|n| n * 3.1))
        .unwrap_or(0.)
        .clamp(0., 30.)
}

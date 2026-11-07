//! Administrative geometry, embedded in the location manifest as GeoJSON.
use super::*;
use geo::{Area, BooleanOps, BoundingRect, Contains, MapCoords, Validation};
use serde_json::{json, Value};

pub(crate) struct Boundary {
    pub relation: i64,
    pub geometry: MultiPolygon<f64>,
}
impl Boundary {
    pub fn resolve(path: &Path, relation: i64) -> Result<Self> {
        let source = super::osm::read(path)?;
        let rel = source
            .relations
            .iter()
            .find(|r| r.id == relation)
            .ok_or_else(|| invalid("boundary relation missing from response"))?;
        if rel.tag("boundary") != "administrative"
            || !matches!(rel.tag("type"), "boundary" | "multipolygon")
        {
            return Err(invalid(
                "requested relation is not an administrative boundary",
            ));
        }
        let mut parts = [vec![], vec![]];
        for (kind, id, role) in &rel.members {
            if !matches!(role.as_str(), "outer" | "inner" | "") {
                continue;
            }
            if kind != "way" {
                return Err(invalid(
                    "nested boundary geometry is unsupported; expected way members",
                ));
            }
            let way = source
                .ways
                .get(id)
                .ok_or_else(|| invalid(format!("missing boundary way {id}")))?;
            if way.nodes.len() < 2 {
                return Err(invalid("empty boundary way"));
            }
            parts[usize::from(role == "inner")].push(way.nodes.clone());
        }
        let make = |parts| -> Result<Vec<Polygon<f64>>> {
            super::scenery::rings(parts)
                .ok_or_else(|| invalid("boundary ways do not form closed rings"))?
                .into_iter()
                .map(|ring| {
                    let points =
                        ring.iter()
                            .map(|id| {
                                let p = source.nodes.get(id).and_then(|n| n.coord).ok_or_else(
                                    || invalid(format!("missing boundary node {id}")),
                                )?;
                                Ok([p[1], p[0]])
                            })
                            .collect::<Result<Vec<_>>>()?;
                    let polygon = polygon(&points);
                    if !polygon.is_valid() || polygon.unsigned_area() == 0.0 {
                        return Err(invalid("invalid boundary ring"));
                    }
                    Ok(polygon)
                })
                .collect()
        };
        let [outer, inner] = parts;
        let outer = make(outer)?;
        let inner = make(inner)?;
        if outer.is_empty() {
            return Err(invalid("boundary has no exterior"));
        }
        let mut holes = vec![vec![]; outer.len()];
        for hole in inner {
            // Attach to the smallest enclosing shell, preserving islands inside holes.
            let owner = outer
                .iter()
                .enumerate()
                .filter(|(_, p)| p.contains(&hole))
                .min_by(|(_, a), (_, b)| a.unsigned_area().total_cmp(&b.unsigned_area()))
                .map(|(i, _)| i)
                .ok_or_else(|| invalid("boundary has an uncontained hole"))?;
            holes[owner].push(hole.exterior().clone());
        }
        let geometry = outer
            .into_iter()
            .zip(holes)
            .try_fold(empty(), |g, (p, holes)| {
                let polygon = Polygon::new(p.exterior().clone(), holes);
                if !polygon.is_valid() {
                    return Err(invalid("invalid boundary shell and holes"));
                }
                Ok(g.union(&MultiPolygon(vec![polygon])))
            })?;
        if geometry.0.is_empty() || !geometry.is_valid() {
            return Err(invalid("invalid administrative geometry"));
        }
        Ok(Self { relation, geometry })
    }
    pub fn bbox(&self) -> BoundingBox {
        let b = self.geometry.bounding_rect().unwrap();
        BoundingBox {
            south: b.min().y,
            west: b.min().x,
            north: b.max().y,
            east: b.max().x,
        }
    }
    pub fn local(&self, origin: P) -> MultiPolygon<f64> {
        self.geometry.map_coords(|c| Coord {
            x: (c.x - origin[1]).to_radians() * 6371000.0 * origin[0].to_radians().cos(),
            y: -(c.y - origin[0]).to_radians() * 6371000.0,
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
    pub fn from_manifest(manifest: &Value) -> Result<Option<Self>> {
        manifest.get("boundary").map(Self::from_geojson).transpose()
    }
    fn from_geojson(v: &Value) -> Result<Self> {
        if v["type"] != "Feature" || v["geometry"]["type"] != "MultiPolygon" {
            return Err(invalid("expected boundary MultiPolygon feature"));
        }
        let coordinates: Vec<Vec<Vec<[f64; 2]>>> =
            serde_json::from_value(v["geometry"]["coordinates"].clone())?;
        let mut polygons = vec![];
        for rings in coordinates {
            if rings.is_empty()
                || rings.iter().any(|r| {
                    r.len() < 4
                        || r.first() != r.last()
                        || r.iter().any(|p| {
                            !p[0].is_finite()
                                || !p[1].is_finite()
                                || p[0].abs() > 180.
                                || p[1].abs() > 90.
                        })
                })
            {
                return Err(invalid("invalid boundary coordinates"));
            }
            polygons.push(Polygon::new(
                line(&rings[0]),
                rings[1..].iter().map(|r| line(r)).collect(),
            ));
        }
        let geometry = MultiPolygon(polygons);
        if geometry.0.is_empty() || !geometry.is_valid() {
            return Err(invalid("invalid boundary geometry"));
        }
        Ok(Self {
            relation: v["properties"]["osm_relation"]
                .as_i64()
                .ok_or_else(|| invalid("missing boundary relation ID"))?,
            geometry,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo::{Intersects, Point};

    fn fixture() -> (tempfile::TempDir, std::path::PathBuf) {
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join("boundary.osm");
        fs::write(&path, r#"<osm version="0.6">
<node id="1" lat="0" lon="0"/><node id="2" lat="0" lon="0.01"/>
<node id="3" lat="0.01" lon="0.01"/><node id="4" lat="0.01" lon="0"/>
<node id="5" lat="0.004" lon="0.004"/><node id="6" lat="0.004" lon="0.006"/>
<node id="7" lat="0.006" lon="0.006"/><node id="8" lat="0.006" lon="0.004"/>
<node id="9" lat="0.005" lon="-0.002"/><node id="10" lat="0.005" lon="0.012"/>
<node id="11" lat="0.02" lon="0.02"/><node id="12" lat="0.02" lon="0.021"/>
<node id="13" lat="0.021" lon="0.021"/><node id="14" lat="0.021" lon="0.02"/>
<way id="20"><nd ref="1"/><nd ref="2"/><nd ref="3"/></way>
<way id="21"><nd ref="1"/><nd ref="4"/><nd ref="3"/></way>
<way id="22"><nd ref="5"/><nd ref="6"/><nd ref="7"/><nd ref="8"/><nd ref="5"/></way>
<way id="23"><nd ref="9"/><nd ref="10"/><tag k="highway" v="residential"/><tag k="oneway" v="yes"/></way>
<way id="24"><nd ref="11"/><nd ref="12"/><nd ref="13"/><nd ref="14"/><nd ref="11"/></way>
<relation id="2087230"><member type="way" ref="21" role="outer"/>
<member type="way" ref="22" role="inner"/><member type="way" ref="20" role="outer"/>
<member type="way" ref="24" role="outer"/>
<tag k="type" v="boundary"/><tag k="boundary" v="administrative"/></relation></osm>"#).unwrap();
        (temp, path)
    }
    #[test]
    fn boundary_assembles_reversed_ways_holes_and_islands() {
        let (temp, path) = fixture();
        let b = Boundary::resolve(&path, 2087230).unwrap();
        assert_eq!(b.geometry.0.len(), 2);
        assert!(!b.geometry.intersects(&Point::new(0.005, 0.005)));
        assert!(b.geometry.intersects(&Point::new(0.002, 0.005)));
        assert!(b.geometry.intersects(&Point::new(0.0205, 0.0205)));
        let manifest_path = temp.path().join("manifest.json");
        fs::write(
            &manifest_path,
            serde_json::to_string_pretty(&json!({"boundary": b.to_geojson()})).unwrap() + "\n",
        )
        .unwrap();
        let manifest: Value = serde_json::from_slice(&fs::read(manifest_path).unwrap()).unwrap();
        let restored = Boundary::from_manifest(&manifest).unwrap().unwrap();
        assert_eq!(b.relation, restored.relation);
        assert_eq!(b.geometry, restored.geometry);
        assert!(Boundary::from_manifest(&json!({})).unwrap().is_none());
        assert!(Boundary::from_manifest(&json!({"boundary": {}})).is_err());
        let source = super::super::osm::read(&path).unwrap();
        let map = super::super::osm::compile_boundary(&source, Some(b.bbox().to_array()), Some(&b))
            .unwrap();
        // Both original endpoints lie outside; clip through the hole into two roads.
        assert_eq!(map.roads.len(), 2);
        for road in &map.roads {
            assert_eq!(road.direction, 1);
            assert!(map.nodes[road.ends[0]][0] < map.nodes[road.ends[1]][0]);
            let midpoint = mul(add(map.nodes[road.ends[0]], map.nodes[road.ends[1]]), 0.5);
            assert!(map.contains(midpoint));
        }
        let output = temp.path().join("compiled");
        fs::create_dir(&output).unwrap();
        let city = super::super::osm::write(&map, &output.join("city.txt")).unwrap();
        let surfaces = super::super::surfaces::build(
            &map,
            &city,
            &output.join("streets.txt"),
            &output.join("streets.json"),
        )
        .unwrap();
        let outside = surfaces.asphalt.difference(map.boundary.as_ref().unwrap());
        assert!(outside.unsigned_area() < 0.001);
    }
    #[test]
    fn boundary_refuses_incomplete_geometry() {
        let (_temp, path) = fixture();
        let text = fs::read_to_string(&path)
            .unwrap()
            .replace("<member type=\"way\" ref=\"20\" role=\"outer\"/>", "");
        fs::write(&path, text).unwrap();
        assert!(Boundary::resolve(&path, 2087230).is_err());
    }
}

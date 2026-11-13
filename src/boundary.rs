use crate::error::{invalid, Result};
use crate::geometry::empty;
use crate::osm::{OSMBoundary, OSMMap};
use geo::{Area, BooleanOps, Contains, Validation};
use geo::{LineString, MultiPolygon, Polygon};

pub fn resolve_boundary(source: &OSMMap, relation: i64) -> Result<OSMBoundary> {
    //
    // Find the relation in the map
    //
    let rel = source
        .relations
        .iter()
        .find(|r| r.id == relation)
        .ok_or_else(|| invalid("boundary relation missing from response"))?;

    //
    // Must be a boundary relation
    //
    if rel.tag("boundary") != "administrative"
        || !matches!(rel.tag("type"), "boundary" | "multipolygon")
    {
        return Err(invalid(
            "requested relation is not an administrative boundary",
        ));
    }

    //
    //
    //
    let mut parts = [vec![], vec![]];

    //
    //
    //
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
        crate::build_landscape::rings(parts)
            .ok_or_else(|| invalid("boundary ways do not form closed rings"))?
            .into_iter()
            .map(|ring| {
                let points =
                    ring.iter()
                        .map(|id| {
                            let p =
                                source.nodes.get(id).and_then(|n| n.coord).ok_or_else(|| {
                                    invalid(format!("missing boundary node {id}"))
                                })?;
                            Ok((p.lon, p.lat))
                        })
                        .collect::<Result<Vec<_>>>()?;
                let polygon = Polygon::new(LineString::from(points), vec![]);
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
    Ok(OSMBoundary { relation, geometry })
}

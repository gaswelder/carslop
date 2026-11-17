use crate::build;
use crate::model;
use crate::model::map_details::RoadSurface;
use crate::model::source_location::SourceLocation;
use crate::model::source_location::SourceRoad;
use crate::util;
use crate::util::error::invalid;
use crate::util::error::Result;
use crate::util::geometry::add;
use crate::util::geometry::cross;
use crate::util::geometry::dist;
use crate::util::geometry::dot;
use crate::util::geometry::empty;
use crate::util::geometry::length;
use crate::util::geometry::line;
use crate::util::geometry::mul;
use crate::util::geometry::path_length;
use crate::util::geometry::polygon;
use crate::util::geometry::sub;
use crate::util::geometry::substring;
use crate::util::geometry::unit;
use crate::util::geometry::XZ;
use geo::{
    algorithm::{
        buffer::{BufferStyle, LineCap, LineJoin},
        orient::Direction,
        unary_union,
    },
    Area, BooleanOps, BoundingRect, Buffer, Intersects, Orient, Point, TriangulateEarcut,
};
use geo::{MultiPolygon, Polygon};
use std::collections::BTreeMap;
use std::path::Path;

/// Compiled road paths and exclusion geometry shared by landscape and furniture generation.
pub struct Surfaces {
    /// Smoothed centrelines in local [X, Z] metres, indexed like SourceLocation::road_lines.
    pub paths: Vec<Vec<XZ>>,
    /// Incident ground-road indices for each SourceLocation node.
    pub adjacency: Vec<Vec<usize>>,
    /// Road, junction and building exclusion polygons in local [X, Z] metres; may overlap.
    pub obstacles: MultiPolygon<f64>,
}

/// Broad-phase queries avoid walking an entire city's polygons for every post or fillet.
pub struct PolygonIndex<'a> {
    polygons: &'a [Polygon<f64>],
    tree: rstar::RTree<rstar::primitives::GeomWithData<rstar::primitives::Rectangle<XZ>, usize>>,
}

pub fn new<'a>(geometry: &'a MultiPolygon<f64>) -> PolygonIndex<'a> {
    let items = geometry
        .0
        .iter()
        .enumerate()
        .filter_map(|(i, p)| {
            p.bounding_rect().map(|r| {
                rstar::primitives::GeomWithData::new(
                    rstar::primitives::Rectangle::from_corners(
                        XZ {
                            x: r.min().x,
                            z: r.min().y,
                        },
                        XZ {
                            x: r.max().x,
                            z: r.max().y,
                        },
                    ),
                    i,
                )
            })
        })
        .collect();
    PolygonIndex::<'a> {
        polygons: &geometry.0,
        tree: rstar::RTree::bulk_load(items),
    }
}
fn candidates<'a, 'index>(
    polygon_index: &'index PolygonIndex<'a>,
    geometry: &MultiPolygon<f64>,
) -> Vec<&'index Polygon<f64>> {
    let Some(r) = geometry.bounding_rect() else {
        return vec![];
    };
    let envelope = rstar::AABB::from_corners(
        XZ {
            x: r.min().x,
            z: r.min().y,
        },
        XZ {
            x: r.max().x,
            z: r.max().y,
        },
    );
    polygon_index
        .tree
        .locate_in_envelope_intersecting(&envelope)
        .map(|item| &polygon_index.polygons[item.data])
        .collect()
}
pub fn intersects<'a>(polygon_index: &PolygonIndex<'a>, geometry: &MultiPolygon<f64>) -> bool {
    candidates(polygon_index, geometry)
        .iter()
        .any(|p| p.intersects(geometry))
}
pub fn nearby<'a>(
    polygon_index: &PolygonIndex<'a>,
    geometry: &MultiPolygon<f64>,
) -> MultiPolygon<f64> {
    MultiPolygon(
        candidates(polygon_index, geometry)
            .into_iter()
            .cloned()
            .collect(),
    )
}

pub fn union(items: &[MultiPolygon<f64>]) -> MultiPolygon<f64> {
    if items.is_empty() {
        empty()
    } else {
        unary_union(items).orient(Direction::Default)
    }
}
fn flat_buffer(path: &[XZ], width: f64) -> MultiPolygon<f64> {
    line(path).buffer_with_style(
        BufferStyle::new(width)
            .line_cap(LineCap::Butt)
            .line_join(LineJoin::Round(std::f64::consts::PI / 16.0)),
    )
}
fn ribbon(
    path: &[XZ],
    road: &SourceRoad<f64>,
    adjacency: &[Vec<usize>],
    extra: f64,
) -> MultiPolygon<f64> {
    let half = road.road_width_start_meters / 2.0 + extra;
    let mut parts = vec![flat_buffer(path, half)];
    for (n, p) in road.ends.iter().zip([path[0], *path.last().unwrap()]) {
        if adjacency[*n].len() < 3 {
            parts.push(Point::new(p.x, p.z).buffer(half));
        }
    }
    union(&parts)
}
// Clip the legacy rounded ribbon to the selected side of the smoothed path.
// Local Z points south, so OSM left is the negative cross-product side.
fn pavement_ribbon(
    path: &[XZ],
    road: &SourceRoad<f64>,
    adjacency: &[Vec<usize>],
    width: f64,
) -> MultiPolygon<f64> {
    let sides = road.sidewalks;
    if !sides.left && !sides.right {
        return empty();
    }
    let full = ribbon(path, road, adjacency, width);
    if sides.left && sides.right {
        return full;
    }
    let reach = road.road_width_start_meters / 2.0 + width;
    let sign = if sides.left { -1.0 } else { 1.0 };
    let first = unit(sub(path[1], path[0]));
    let last = unit(sub(path[path.len() - 1], path[path.len() - 2]));
    let mut centre = path.to_vec();
    centre[0] = sub(centre[0], mul(first, reach));
    let end = centre.len() - 1;
    centre[end] = add(centre[end], mul(last, reach));
    let mut outer = Vec::with_capacity(centre.len());
    for (i, p) in centre.iter().enumerate() {
        let before = if i == 0 {
            first
        } else {
            unit(sub(*p, centre[i - 1]))
        };
        let after = if i == end {
            last
        } else {
            unit(sub(centre[i + 1], *p))
        };
        let normal = |v: XZ| XZ { x: -v.z, z: v.x };
        let bisector = add(normal(before), normal(after));
        let offset = mul(
            bisector,
            sign * reach / (1.0 + dot(before, after)).max(0.25),
        );
        outer.push(add(*p, offset));
    }
    centre.extend(outer.into_iter().rev());
    full.intersection(&MultiPolygon(vec![polygon(&centre)]))
}

fn paths(map: &SourceLocation<f64>, adjacency: &[Vec<usize>]) -> Vec<Vec<XZ>> {
    let end = |node: usize, other: usize| {
        let p = util::geometry::xz(map.road_nodes[node]);
        if adjacency[node].len() != 2 {
            return vec![p];
        }
        let neighbor = adjacency[node]
            .iter()
            .flat_map(|i| map.road_lines[*i].ends)
            .find(|n| *n != node && *n != other);
        let Some(neighbor) = neighbor else {
            return vec![p];
        };
        let a = unit(sub(util::geometry::xz(map.road_nodes[other]), p));
        let b = unit(sub(util::geometry::xz(map.road_nodes[neighbor]), p));
        let d = dot(a, b);
        if d > 0.5 || d < -0.9995 {
            return vec![p];
        }
        let displacement = length(add(a, b)) * 0.25;
        let trim = 2.0_f64
            .min(dist(p, util::geometry::xz(map.road_nodes[other])) * 0.2)
            .min(dist(p, util::geometry::xz(map.road_nodes[neighbor])) * 0.2)
            .min(0.6 / displacement.max(0.001));
        let q0 = add(p, mul(a, trim));
        let q1 = add(p, mul(b, trim));
        (0..5)
            .map(|i| {
                let t = 0.5 - i as f64 * 0.125;
                add(
                    add(mul(q0, (1.0 - t).powi(2)), mul(p, 2.0 * (1.0 - t) * t)),
                    mul(q1, t * t),
                )
            })
            .collect()
    };
    map.road_lines
        .iter()
        .map(|r| {
            let mut path = end(r.ends[0], r.ends[1]);
            let mut tail = end(r.ends[1], r.ends[0]);
            tail.reverse();
            path.extend(tail);
            path.dedup_by(|a, b| dist(*a, *b) < 0.001);
            path
        })
        .collect()
}
/// Emit triangles incrementally after the existing quantization and winding checks.
pub fn triangulate(
    geometry: &MultiPolygon<f64>,
    mut emit: impl FnMut([XZ; 3]) -> Result<()>,
) -> Result<usize> {
    let mut count = 0;
    // Bound triangulation work for city-wide connected polygons with many holes.
    // Split only the emitted mesh; curb rings still follow the original outline.
    let mut pending: Vec<_> = geometry.0.iter().cloned().map(|p| (p, 0)).collect();
    while let Some((poly, depth)) = pending.pop() {
        if poly.unsigned_area() <= 0.000005 {
            continue;
        }
        let vertices =
            poly.exterior().0.len() + poly.interiors().iter().map(|r| r.0.len()).sum::<usize>();
        if vertices > 4096 && depth < 24 {
            let r = poly.bounding_rect().unwrap();
            let (x0, y0, x1, y1) = (r.min().x, r.min().y, r.max().x, r.max().y);
            let boxes = if x1 - x0 >= y1 - y0 {
                let mid = (x0 + x1) / 2.;
                [[x0, y0, mid, y1], [mid, y0, x1, y1]]
            } else {
                let mid = (y0 + y1) / 2.;
                [[x0, y0, x1, mid], [x0, mid, x1, y1]]
            };
            for b in boxes {
                let crop = polygon(&[
                    XZ { x: b[0], z: b[1] },
                    XZ { x: b[2], z: b[1] },
                    XZ { x: b[2], z: b[3] },
                    XZ { x: b[0], z: b[3] },
                ]);
                pending.extend(
                    poly.intersection(&crop)
                        .0
                        .into_iter()
                        .map(|p| (p, depth + 1)),
                );
            }
            continue;
        }
        for tri in poly.earcut_triangles() {
            let mut p = [
                [tri.v1().x, tri.v1().y],
                [tri.v2().x, tri.v2().y],
                [tri.v3().x, tri.v3().y],
            ];
            // Validate winding at the precision actually stored on disk. Thin
            // triangles can collapse or flip when coordinates are rounded.
            for v in p.iter_mut().flatten() {
                *v = (*v * 1_000_000.).round() / 1_000_000.;
            }
            let [a, b, c] = p.map(|[x, z]| XZ { x, z });
            let area = cross(sub(b, a), sub(c, a));
            // Match StreetSurface::load's minimum signed cross product.
            if area.abs() <= 0.00001 {
                continue;
            }
            if area < 0.0 {
                p.reverse();
            }
            emit(p.map(|[x, z]| XZ { x, z }))?;
            count += 1;
        }
    }
    Ok(count)
}

/// Straight piece of a compiled road path, indexed for nearest-road queries.
#[derive(Clone)]
pub struct RoadLineIndexItem {
    /// Start position in local [X, Z] metres.
    start_coords: XZ,
    /// End position in local [X, Z] metres.
    end_coords: XZ,
    /// Index of the owning road and its compiled path.
    pub road_index: usize,
}

impl rstar::RTreeObject for RoadLineIndexItem {
    type Envelope = rstar::AABB<XZ>;
    fn envelope(&self) -> Self::Envelope {
        rstar::AABB::from_corners(
            XZ {
                x: self.start_coords.x.min(self.end_coords.x),
                z: self.start_coords.z.min(self.end_coords.z),
            },
            XZ {
                x: self.start_coords.x.max(self.end_coords.x),
                z: self.start_coords.z.max(self.end_coords.z),
            },
        )
    }
}

impl rstar::PointDistance for RoadLineIndexItem {
    fn distance_2(&self, p: &XZ) -> f64 {
        util::geometry::project(&[self.start_coords, self.end_coords], *p)
            .0
            .powi(2)
    }
}

// Nearest road segments via an R-tree, shared by curbs and roadside controls.
pub fn index(paths: &[Vec<XZ>]) -> rstar::RTree<RoadLineIndexItem> {
    rstar::RTree::bulk_load(
        paths
            .iter()
            .enumerate()
            .flat_map(|(road, path)| {
                path.windows(2).map(move |p| RoadLineIndexItem {
                    start_coords: p[0],
                    end_coords: p[1],
                    road_index: road,
                })
            })
            .collect(),
    )
}

pub struct WriteStreetsResult {
    pub surfaces: Surfaces,
}

pub fn build_streets(
    map: &SourceLocation<f64>,
    directory: &Path,
    source: Option<&build::osm::OSMMap>,
) -> Result<WriteStreetsResult> {
    let elevated: Vec<_> = map
        .road_lines
        .iter()
        .map(|road| build::transport::road_grade(source, road) != 0)
        .collect();
    let mut adjacency = vec![vec![]; map.road_nodes.len()];
    for (i, r) in map.road_lines.iter().enumerate() {
        if elevated[i] {
            continue;
        }
        for n in r.ends {
            adjacency[n].push(i);
        }
    }
    let mut paths = paths(map, &adjacency);
    for (i, path) in paths.iter_mut().enumerate() {
        if elevated[i] {
            *path = map.road_lines[i]
                .ends
                .map(|n| util::geometry::xz(map.road_nodes[n]))
                .to_vec();
        }
    }
    let buildings = union(
        &map.buildings
            .iter()
            .filter(|b| b.min_height_meters < 1.7)
            .map(|b| model::world::footprint(b).buffer(0.1))
            .collect::<Vec<_>>(),
    );
    let building_index = new(&buildings);
    let mut patches = Vec::new();
    let mut patch_kinds = Vec::new();
    let mut patch_nodes = Vec::new();
    let mut patch_pavement = Vec::new();
    for (node, arms) in adjacency.iter().enumerate().filter(|(_, a)| a.len() >= 3) {
        let p = util::geometry::xz(map.road_nodes[node]);
        let other = |i: usize| {
            let r = &map.road_lines[i];
            util::geometry::xz(
                map.road_nodes[if r.ends[0] == node {
                    r.ends[1]
                } else {
                    r.ends[0]
                }],
            )
        };
        let mut arms = arms.clone();
        arms.sort_by(|a, b| {
            let a = sub(other(*a), p);
            let b = sub(other(*b), p);
            a.z.atan2(a.x).total_cmp(&b.z.atan2(b.x))
        });
        for j in 0..arms.len() {
            let ia = arms[j];
            let ib = arms[(j + 1) % arms.len()];
            let a = unit(sub(other(ia), p));
            let b = unit(sub(other(ib), p));
            let angle = (b.z.atan2(b.x) - a.z.atan2(a.x)).rem_euclid(std::f64::consts::TAU);
            if angle <= 40.0_f64.to_radians() || angle >= 165.0_f64.to_radians() {
                continue;
            }
            let qa = add(
                p,
                mul(
                    XZ { x: -a.z, z: a.x },
                    map.road_lines[ia].road_width_start_meters / 2.0,
                ),
            );
            let qb = add(
                p,
                mul(
                    XZ { x: b.z, z: -b.x },
                    map.road_lines[ib].road_width_start_meters / 2.0,
                ),
            );
            let delta = sub(qb, qa);
            let den = cross(a, b);
            let sa = cross(delta, b) / den;
            let sb = cross(delta, a) / den;
            let kind = map.road_lines[ia].kind.min(map.road_lines[ib].kind);
            let radius = [3.5_f64, 2.3, 1.0][kind]
                .min(path_length(&paths[ia]) * 0.4 - sa)
                .min(path_length(&paths[ib]) * 0.4 - sb);
            if radius < 0.35 || sa < 0.0 || sb < 0.0 {
                continue;
            }
            let q = add(qa, mul(a, sa));
            let start = add(q, mul(a, radius));
            let end = add(q, mul(b, radius));
            let mut points = vec![p];
            points.extend((0..13).map(|i| {
                let t = i as f64 / 12.0;
                add(
                    add(mul(start, (1.0 - t).powi(2)), mul(q, 2.0 * t * (1.0 - t))),
                    mul(end, t * t),
                )
            }));
            let poly = MultiPolygon(vec![polygon(&points)]);
            let poly = poly.difference(&nearby(&building_index, &poly));
            if !poly.0.is_empty() {
                let has_pavement = |i: usize, outward: XZ| {
                    let road = &map.road_lines[i];
                    let forward = mul(outward, if road.ends[0] == node { 1.0 } else { -1.0 });
                    if cross(forward, add(a, b)) < 0.0 {
                        road.sidewalks.left
                    } else {
                        road.sidewalks.right
                    }
                };
                patch_pavement.push(has_pavement(ia, a) && has_pavement(ib, b));
                patches.push(poly);
                patch_kinds.push(kind);
                patch_nodes.push(node);
            }
        }
    }
    let buffers: Vec<_> = map
        .road_lines
        .iter()
        .zip(&paths)
        .enumerate()
        .map(|(i, (r, p))| {
            if elevated[i] {
                empty()
            } else {
                ribbon(p, r, &adjacency, 0.)
            }
        })
        .collect();
    let mut cores = vec![];
    let mut core_kinds = vec![];
    for (n, arms) in adjacency.iter().enumerate().filter(|(_, a)| a.len() >= 3) {
        let width = arms
            .iter()
            .map(|i| map.road_lines[*i].road_width_start_meters / 2.0)
            .fold(f64::INFINITY, f64::min);
        let p = util::geometry::xz(map.road_nodes[n]);
        cores.push(Point::new(p.x, p.z).buffer(width));
        core_kinds.push(arms.iter().map(|i| map.road_lines[*i].kind).min().unwrap());
    }
    let asphalt = union(
        &buffers
            .iter()
            .chain(&patches)
            .chain(&cores)
            .cloned()
            .collect::<Vec<_>>(),
    );
    let asphalt = map
        .boundary
        .as_ref()
        .map_or_else(|| asphalt.clone(), |b| asphalt.intersection(b));
    let inner = asphalt.buffer(-0.16);
    let gutter = asphalt.difference(&inner);
    struct MaterialZone {
        material: RoadSurface,
        geometry: MultiPolygon<f64>,
    }
    let mut material_zones = Vec::new();
    if let Some(source) = source {
        // An unpaved route can cross or run beside a paved carriageway. Keep
        // its gravel outside the paved road's footprint instead of painting
        // the gravel buffer across the entire shared road mesh.
        let paved_roads = union(
            &buffers
                .iter()
                .zip(&map.road_lines)
                .filter(|(_, road)| {
                    road.osm_way_id
                        .and_then(|id| source.ways.get(&id))
                        .is_some_and(|way| {
                            build::details::road_surface(build::osm::tag(way, "surface"))
                                != RoadSurface::Unpaved
                        })
                })
                .map(|(geometry, _)| geometry.clone())
                .collect::<Vec<_>>(),
        );
        for material in [
            RoadSurface::Concrete,
            RoadSurface::Paving,
            RoadSurface::Unpaved,
        ] {
            let pieces: Vec<_> = buffers
                .iter()
                .zip(&map.road_lines)
                .filter(|(_, road)| {
                    road.osm_way_id
                        .and_then(|id| source.ways.get(&id))
                        .is_some_and(|way| {
                            build::details::road_surface(build::osm::tag(way, "surface"))
                                == material
                        })
                })
                .map(|(geometry, _)| geometry.clone())
                .collect();
            material_zones.push(MaterialZone {
                material,
                geometry: if material == RoadSurface::Unpaved {
                    union(&pieces).difference(&paved_roads)
                } else {
                    union(&pieces)
                },
            });
        }
    }
    let mut remaining = inner.clone();
    let mut groups = vec![];
    for kind in 0..3 {
        let mut pieces = vec![];
        pieces.extend(
            buffers
                .iter()
                .zip(&map.road_lines)
                .filter(|(_, r)| r.kind == kind)
                .map(|(g, _)| g.clone()),
        );
        pieces.extend(
            patches
                .iter()
                .zip(&patch_kinds)
                .filter(|(_, k)| **k == kind)
                .map(|(g, _)| g.clone()),
        );
        pieces.extend(
            cores
                .iter()
                .zip(&core_kinds)
                .filter(|(_, k)| **k == kind)
                .map(|(g, _)| g.clone()),
        );
        let zone = union(&pieces);
        let mut geometry = remaining.intersection(&zone);
        for MaterialZone {
            material,
            geometry: zone,
        } in &material_zones
        {
            let base = match material {
                RoadSurface::Concrete => 8,
                RoadSurface::Paving => 11,
                RoadSurface::Unpaved => 14,
                RoadSurface::Asphalt => unreachable!(),
            };
            groups.push((base + kind, geometry.intersection(zone)));
            geometry = geometry.difference(zone);
        }
        groups.push((kind, geometry));
        remaining = remaining.difference(&zone);
    }
    groups.push((1, remaining));
    groups.push((3, gutter));
    let mut paved = asphalt.clone();
    for kind in 0..3 {
        let width = [2.2, 1.4, 0.65][kind];
        let mut pieces: Vec<_> = paths
            .iter()
            .zip(&map.road_lines)
            .enumerate()
            .filter(|(i, (_, r))| r.kind == kind && !elevated[*i])
            .map(|(_, entry)| entry)
            .map(|(p, r)| {
                let way =
                    source.and_then(|source| r.osm_way_id.and_then(|id| source.ways.get(&id)));
                let sides = [true, false].map(|left| {
                    let mut road = r.clone();
                    road.sidewalks.left = left && r.sidewalks.left;
                    road.sidewalks.right = !left && r.sidewalks.right;
                    let tagged = way
                        .and_then(|way| {
                            build::osm::number(build::osm::tag(
                                way,
                                if left {
                                    "sidewalk:left:width"
                                } else {
                                    "sidewalk:right:width"
                                },
                            ))
                            .or_else(|| {
                                build::osm::number(build::osm::tag(way, "sidewalk:both:width"))
                            })
                            .or_else(|| build::osm::number(build::osm::tag(way, "sidewalk:width")))
                        })
                        .unwrap_or(width)
                        .clamp(0.2, 10.);
                    pavement_ribbon(p, &road, &adjacency, tagged)
                });
                union(&sides)
            })
            .collect();
        pieces.extend(
            patches
                .iter()
                .zip(&patch_kinds)
                .zip(&patch_pavement)
                .filter(|((_, k), enabled)| **k == kind && **enabled)
                .map(|((g, _), _)| g.buffer(width)),
        );
        let zone = union(&pieces);
        groups.push((5 + kind, zone.difference(&paved).difference(&buildings)));
        paved = paved.union(&zone);
    }
    let junctions = union(
        &adjacency
            .iter()
            .enumerate()
            .filter(|(_, a)| a.len() > 2)
            .map(|(n, a)| {
                let p = util::geometry::xz(map.road_nodes[n]);
                Point::new(p.x, p.z).buffer(
                    a.iter()
                        .map(|i| map.road_lines[*i].road_width_start_meters / 2.0)
                        .fold(0.0, f64::max)
                        + 3.0,
                )
            })
            .collect::<Vec<_>>(),
    );
    let mut phases = vec![0.0; map.road_lines.len()];
    let mut by_way: BTreeMap<i64, Vec<usize>> = BTreeMap::new();
    for (i, r) in map.road_lines.iter().enumerate() {
        let way = r
            .osm_way_id
            .ok_or_else(|| invalid("compiler road is missing a way ID"))?;
        by_way.entry(way).or_default().push(i);
    }
    for ids in by_way.values() {
        let mut pending: std::collections::BTreeSet<_> = ids.iter().copied().collect();
        while !pending.is_empty() {
            let incoming: std::collections::BTreeSet<_> =
                pending.iter().map(|i| map.road_lines[*i].ends[1]).collect();
            let mut current = *pending
                .iter()
                .find(|i| !incoming.contains(&map.road_lines[**i].ends[0]))
                .unwrap_or(pending.first().unwrap());
            let mut distance = 0.0;
            while pending.remove(&current) {
                phases[current] = distance;
                distance += path_length(&paths[current]);
                let Some(next) = pending
                    .iter()
                    .find(|i| map.road_lines[**i].ends[0] == map.road_lines[current].ends[1])
                    .copied()
                else {
                    break;
                };
                current = next;
            }
        }
    }
    let offset = |path: &[XZ], amount: f64| -> Vec<XZ> {
        path.iter()
            .enumerate()
            .map(|(i, p)| {
                let before = path[i.saturating_sub(1)];
                let after = path[(i + 1).min(path.len() - 1)];
                let tangent = unit(sub(after, before));
                add(
                    *p,
                    mul(
                        XZ {
                            x: -tangent.z,
                            z: tangent.x,
                        },
                        amount,
                    ),
                )
            })
            .collect()
    };
    let mut marks = vec![];
    for (i, (path, r)) in paths.iter().zip(&map.road_lines).enumerate() {
        if elevated[i] {
            continue;
        }
        if source
            .and_then(|source| r.osm_way_id.and_then(|id| source.ways.get(&id)))
            .is_some_and(|way| {
                build::details::road_surface(build::osm::tag(way, "surface"))
                    == RoadSurface::Unpaved
            })
        {
            continue;
        }
        if r.kind == 0 {
            for side in [-1.0, 1.0] {
                marks.push(flat_buffer(
                    &offset(path, side * (r.road_width_start_meters / 2.0 - 0.38)),
                    0.045,
                ));
            }
        }
        let mut dashed = vec![];
        let way = source.and_then(|source| r.osm_way_id.and_then(|id| source.ways.get(&id)));
        let forward = way
            .and_then(|w| build::osm::number(build::osm::tag(w, "lanes:forward")))
            .map(|v| v as usize);
        let backward = way
            .and_then(|w| build::osm::number(build::osm::tag(w, "lanes:backward")))
            .map(|v| v as usize);
        if r.direction == 0 && (forward.is_some() || backward.is_some()) {
            let forward = forward
                .unwrap_or_else(|| r.lanes.saturating_sub(backward.unwrap_or(1)))
                .clamp(1, 6);
            let backward = backward
                .unwrap_or_else(|| r.lanes.saturating_sub(forward))
                .clamp(1, 6);
            let width = (r.road_width_start_meters - 0.9) / (forward + backward) as f64;
            for lane in 1..forward + backward {
                dashed.push(offset(
                    path,
                    -(r.road_width_start_meters - 0.9) / 2. + lane as f64 * width,
                ));
            }
        } else if r.direction == 0
            && (r.kind == 0 || (r.kind == 1 && r.road_width_start_meters >= 7.5))
        {
            dashed.push(path.clone());
        } else if r.kind == 0 && r.direction != 0 && r.lanes > 1 {
            for lane in 1..r.lanes {
                dashed.push(offset(
                    path,
                    -(r.road_width_start_meters - 0.9) / 2.0
                        + (r.road_width_start_meters - 0.9) * lane as f64 / r.lanes as f64,
                ));
            }
        }
        for path in dashed {
            let n = path_length(&path);
            let mut d = -(phases[i] % 9.0);
            while d < n {
                let lo = d.max(0.0);
                let hi = n.min(d + 3.0);
                if hi > lo {
                    marks.push(flat_buffer(&substring(&path, lo, hi), 0.065));
                }
                d += 9.0;
            }
        }
    }
    let lane_marks = union(&marks).intersection(&inner).difference(&junctions);
    let paint = if let Some(source) = source {
        let crossings = build::crossings::build_crossings(source, map, &paths, &inner);
        lane_marks
            .union(&build::lane_details::paint(source, map, &paths, &inner).difference(&junctions))
            .difference(&crossings.clearance)
            .union(&crossings.paint)
    } else {
        lane_marks
    };
    groups.push((4, paint));
    let mut file = build::streets::create(
        directory,
        map.road_lines.len(),
        build::streets::Precision::Imported,
    )?;
    if let Some(n) = patch_nodes.iter().min_by(|a, b| {
        length(util::geometry::xz(map.road_nodes[**a]))
            .total_cmp(&length(util::geometry::xz(map.road_nodes[**b])))
    }) {
        build::streets::junction(&mut file, *n)?;
    }
    for (i, path) in paths.iter().enumerate() {
        build::streets::path(&mut file, i, path)?;
    }

    let mut counts = BTreeMap::new();
    for (style, geometry) in groups {
        let geometry = map
            .boundary
            .as_ref()
            .map_or_else(|| geometry.clone(), |b| geometry.intersection(b));
        let n = triangulate(&geometry, |points| {
            build::streets::triangle(&mut file, style, points)?;
            Ok(())
        })?;
        *counts.entry(style).or_insert(0) += n;
    }
    let tree = index(&paths);
    for poly in &asphalt.0 {
        for ring in std::iter::once(poly.exterior()).chain(poly.interiors()) {
            for pair in ring.0.windows(2) {
                let a = XZ {
                    x: pair[0].x,
                    z: pair[0].y,
                };
                let b = XZ {
                    x: pair[1].x,
                    z: pair[1].y,
                };
                if dist(a, b) < 0.001 {
                    continue;
                }
                let tangent = unit(sub(b, a));
                let kind = map.road_lines[tree
                    .nearest_neighbor(&mul(add(a, b), 0.5))
                    .unwrap()
                    .road_index]
                    .kind;
                build::streets::curb(
                    &mut file,
                    kind,
                    [a, b],
                    XZ {
                        x: -tangent.z,
                        z: tangent.x,
                    },
                )?;
            }
        }
    }
    build::streets::flush(&mut file)?;

    // Index the small input pieces, rather than a city-wide connected polygon.
    // A probe intersects their union exactly when it intersects any one piece.
    let original_footprints = MultiPolygon(
        map.buildings
            .iter()
            .map(|b| model::world::footprint(b))
            .collect(),
    );
    let obstacles = MultiPolygon(
        buffers
            .into_iter()
            .chain(patches)
            .chain(cores)
            .chain(std::iter::once(original_footprints))
            .flat_map(|g| g.0)
            .collect(),
    );

    Ok(WriteStreetsResult {
        surfaces: Surfaces {
            paths,
            adjacency,
            obstacles,
        },
    })
}

#[cfg(test)]
#[path = "tests/surfaces_geometry.rs"]
mod geometry_tests;

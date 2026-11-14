use crate::build_crossings;
use crate::geometry;
use crate::geometry::XZ;
use crate::geometry::{
    add, cross, dist, dot, empty, length, line, mul, path_length, polygon, sub, substring, unit,
};
use crate::osm;
use crate::{
    error::{invalid, Result},
    source_location::{SourceLocation, SourceRoad},
};
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
use std::fs;
use std::path::Path;
use std::{fmt, io};

/// Compiled road paths and exclusion geometry shared by landscape and furniture generation.
pub struct Surfaces {
    /// Smoothed centrelines in local [X, Z] metres, indexed like SourceLocation::road_lines.
    pub paths: Vec<Vec<XZ>>,
    /// Incident road indices for each SourceLocation node.
    pub adjacency: Vec<Vec<usize>>,
    /// Exact carriageway footprint, including junctions, for raised road clipping.
    pub asphalt: MultiPolygon<f64>,
    /// Road, junction and building exclusion polygons in local [X, Z] metres; may overlap.
    pub obstacles: MultiPolygon<f64>,
}

/// Broad-phase queries avoid walking an entire city's polygons for every post or fillet.
pub struct PolygonIndex<'a> {
    polygons: &'a [Polygon<f64>],
    tree: rstar::RTree<rstar::primitives::GeomWithData<rstar::primitives::Rectangle<XZ>, usize>>,
}
impl<'a> PolygonIndex<'a> {
    pub fn new(geometry: &'a MultiPolygon<f64>) -> Self {
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
        Self {
            polygons: &geometry.0,
            tree: rstar::RTree::bulk_load(items),
        }
    }
    fn candidates(&self, geometry: &MultiPolygon<f64>) -> Vec<&Polygon<f64>> {
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
        self.tree
            .locate_in_envelope_intersecting(&envelope)
            .map(|item| &self.polygons[item.data])
            .collect()
    }
    pub fn intersects(&self, geometry: &MultiPolygon<f64>) -> bool {
        self.candidates(geometry)
            .iter()
            .any(|p| p.intersects(geometry))
    }
    pub fn nearby(&self, geometry: &MultiPolygon<f64>) -> MultiPolygon<f64> {
        MultiPolygon(self.candidates(geometry).into_iter().cloned().collect())
    }
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
        let p = map.road_nodes[node].xz();
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
        let a = unit(sub(map.road_nodes[other].xz(), p));
        let b = unit(sub(map.road_nodes[neighbor].xz(), p));
        let d = dot(a, b);
        if d > 0.5 || d < -0.9995 {
            return vec![p];
        }
        let displacement = length(add(a, b)) * 0.25;
        let trim = 2.0_f64
            .min(dist(p, map.road_nodes[other].xz()) * 0.2)
            .min(dist(p, map.road_nodes[neighbor].xz()) * 0.2)
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
fn triangles(
    writer: &mut impl io::Write,
    prefix: &str,
    geometry: &MultiPolygon<f64>,
) -> Result<usize> {
    triangulate(geometry, |p| {
        io::Write::write_fmt(
            writer,
            format_args!(
                "{prefix} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6}\n",
                p[0].x, p[0].z, p[1].x, p[1].z, p[2].x, p[2].z
            ),
        )?;
        Ok(())
    })
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
        geometry::project(&[self.start_coords, self.end_coords], *p)
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

pub fn write_streets_txt(
    map: &SourceLocation<f64>,
    path_streets_txt: &Path,
    source: Option<&osm::OSMMap>,
) -> Result<WriteStreetsResult> {
    let mut adjacency = vec![vec![]; map.road_nodes.len()];
    for (i, r) in map.road_lines.iter().enumerate() {
        for n in r.ends {
            adjacency[n].push(i);
        }
    }
    let paths = paths(map, &adjacency);
    let buildings = union(
        &map.buildings
            .iter()
            .map(|b| polygon(&b.ground_outline).buffer(0.1))
            .collect::<Vec<_>>(),
    );
    let building_index = PolygonIndex::new(&buildings);
    let mut patches = Vec::new();
    let mut patch_kinds = Vec::new();
    let mut patch_nodes = Vec::new();
    let mut patch_pavement = Vec::new();
    for (node, arms) in adjacency.iter().enumerate().filter(|(_, a)| a.len() >= 3) {
        let p = map.road_nodes[node].xz();
        let other = |i: usize| {
            let r = &map.road_lines[i];
            map.road_nodes[if r.ends[0] == node {
                r.ends[1]
            } else {
                r.ends[0]
            }]
            .xz()
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
            let poly = poly.difference(&building_index.nearby(&poly));
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
        .map(|(r, p)| ribbon(p, r, &adjacency, 0.0))
        .collect();
    let mut cores = vec![];
    let mut core_kinds = vec![];
    for (n, arms) in adjacency.iter().enumerate().filter(|(_, a)| a.len() >= 3) {
        let width = arms
            .iter()
            .map(|i| map.road_lines[*i].road_width_start_meters / 2.0)
            .fold(f64::INFINITY, f64::min);
        let p = map.road_nodes[n].xz();
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
        groups.push((kind, remaining.intersection(&zone)));
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
            .filter(|(_, r)| r.kind == kind)
            .map(|(p, r)| pavement_ribbon(p, r, &adjacency, width))
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
                let p = map.road_nodes[n].xz();
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
        if r.kind == 0 {
            for side in [-1.0, 1.0] {
                marks.push(flat_buffer(
                    &offset(path, side * (r.road_width_start_meters / 2.0 - 0.38)),
                    0.045,
                ));
            }
        }
        let mut dashed = vec![];
        if r.direction == 0 && (r.kind == 0 || (r.kind == 1 && r.road_width_start_meters >= 7.5)) {
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
        let crossings = build_crossings::build_crossings(source, map, &paths, &inner);
        lane_marks
            .difference(&crossings.clearance)
            .union(&crossings.paint)
    } else {
        lane_marks
    };
    groups.push((4, paint));
    let mut file = io::BufWriter::new(fs::File::create(&path_streets_txt)?);
    io::Write::write_fmt(
        &mut file,
        format_args!("STREETS 1 {}\n", map.road_lines.len()),
    )?;
    if let Some(n) = patch_nodes.iter().min_by(|a, b| {
        length(map.road_nodes[**a].xz()).total_cmp(&length(map.road_nodes[**b].xz()))
    }) {
        io::Write::write_fmt(&mut file, format_args!("j {n}\n"))?;
    }
    for (i, path) in paths.iter().enumerate() {
        let mut row = format!("p {i} {}", path.len());
        for p in path {
            fmt::Write::write_fmt(&mut row, format_args!(" {:.6} {:.6}", p.x, p.z))?;
        }
        io::Write::write_fmt(&mut file, format_args!("{row}\n"))?;
    }

    let mut counts = BTreeMap::new();
    for (style, geometry) in groups {
        let geometry = map
            .boundary
            .as_ref()
            .map_or_else(|| geometry.clone(), |b| geometry.intersection(b));
        let n = triangles(&mut file, &format!("t {style}"), &geometry)?;
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
                io::Write::write_fmt(
                    &mut file,
                    format_args!(
                        "c {kind} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6}\n",
                        a.x, a.z, b.x, b.z, -tangent.z, tangent.x
                    ),
                )?;
            }
        }
    }
    io::Write::flush(&mut file)?;

    // Index the small input pieces, rather than a city-wide connected polygon.
    // A probe intersects their union exactly when it intersects any one piece.
    let original_footprints = MultiPolygon(
        map.buildings
            .iter()
            .map(|b| polygon(&b.ground_outline))
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
            asphalt,
            obstacles,
        },
    })
}

#[cfg(test)]
mod geometry_tests {
    use super::*;
    use crate::build_surfaces;
    use crate::geometry::{project, BoundingBoxXZ, LatLon};
    use crate::src_files;
    use crate::street_surface;
    use crate::test_support::Temp;
    use crate::world;
    use geo::{Contains, Point};
    fn map(
        nodes: &[XZ],
        roads: &[(usize, usize, f64, usize)],
        buildings: &[Vec<XZ>],
    ) -> SourceLocation<f64> {
        SourceLocation {
            boundary: None,
            origin: Some(LatLon::new(35.162, 33.320)),
            bounding_box: BoundingBoxXZ {
                min: XZ {
                    x: -100.0,
                    z: -100.0,
                },
                max: XZ { x: 100.0, z: 100.0 },
            },
            road_nodes: nodes.iter().map(|p| p.at_height(0.0)).collect(),
            road_lines: roads
                .iter()
                .enumerate()
                .map(|(i, (a, b, width, kind))| SourceRoad {
                    kind: *kind,
                    direction: 0,
                    osm_way_id: Some(i as i64 + 1),
                    lanes: 2,
                    ..SourceRoad::new([*a, *b], *width)
                })
                .collect(),
            buildings: buildings
                .iter()
                .enumerate()
                .map(|(i, p)| world::Building {
                    id: i as u64 + 1,
                    height_meters: 6.0,
                    base_y: 0.0,
                    tagged: false,
                    ground_outline: p.clone(),
                })
                .collect(),
            ..SourceLocation::<f64>::empty()
        }
    }
    fn surfaces(
        map: &SourceLocation<f64>,
    ) -> (
        Temp,
        build_surfaces::Surfaces,
        street_surface::StreetSurface,
    ) {
        let temp = Temp::new();
        src_files::write_city_txt(map, &temp.directory.join("city.txt")).unwrap();
        let s =
            build_surfaces::write_streets_txt(map, &temp.directory.join("streets.txt"), None).unwrap();
        let loaded = street_surface::StreetSurface::load(
            &std::fs::read_to_string(temp.directory.join("streets.txt")).unwrap(),
            map.road_lines.len(),
        )
        .unwrap()
        .0;
        (temp, s.surfaces, loaded)
    }
    fn point(p: XZ) -> Point<f64> {
        Point::new(p.x, p.z)
    }
    #[test]
    fn pavement_follows_left_side_through_a_bend_and_round_end_caps() {
        let mut road = SourceRoad::new([0, 1], 8.0);
        road.sidewalks.right = false;
        let path = [
            XZ { x: -30., z: 0. },
            XZ { x: 0., z: 0. },
            XZ { x: 0., z: -30. },
        ];
        let geometry = pavement_ribbon(&path, &road, &[vec![0], vec![0]], 1.4);
        for p in [
            XZ { x: -15., z: -4.7 },
            XZ { x: -4.7, z: -15. },
            XZ { x: -31., z: -4.7 },
        ] {
            assert!(
                geometry.contains(&point(p)),
                "missing left pavement at {p:?}"
            );
        }
        for p in [
            XZ { x: -15., z: 4.7 },
            XZ { x: 4.7, z: -15. },
            XZ { x: -31., z: 4.7 },
        ] {
            assert!(
                !geometry.contains(&point(p)),
                "unexpected right pavement at {p:?}"
            );
        }
    }

    #[test]
    fn junction_without_sidewalks_has_no_generated_pavement() {
        let mut m = map(
            &[
                XZ { x: -40., z: 0. },
                XZ { x: 0., z: 0. },
                XZ { x: 40., z: 0. },
                XZ { x: 0., z: -40. },
            ],
            &[(0, 1, 8., 0), (1, 2, 8., 0), (1, 3, 6.5, 1)],
            &[],
        );
        for road in &mut m.road_lines {
            road.sidewalks.left = false;
            road.sidewalks.right = false;
        }
        let (_, _, compiled) = surfaces(&m);
        assert!(compiled
            .road_triangles()
            .iter()
            .all(|t| !matches!(t.usage, world::TriangleUsage::Pavement(_))));
    }

    #[test]
    fn junction_fillets_partition_asphalt_and_pavement_without_overlap() {
        let m = map(
            &[
                XZ { x: -40., z: 0. },
                XZ { x: 0., z: 0. },
                XZ { x: 40., z: 0. },
                XZ { x: 0., z: -40. },
                XZ { x: 0., z: 40. },
            ],
            &[(0, 1, 8., 0), (1, 2, 8., 0), (3, 1, 6.5, 1), (1, 4, 5.5, 2)],
            &[],
        );
        let (_temp, s, loaded) = surfaces(&m);
        assert!(s.asphalt.contains(&point(XZ { x: 4.4, z: 3.4 })));
        let mut area = 0.;
        let mut styles = std::collections::BTreeSet::new();
        for t in loaded.road_triangles() {
            let p: Vec<XZ> = t
                .points
                .iter()
                .map(|p| XZ {
                    x: p.x as f64,
                    z: p.z as f64,
                })
                .collect();
            let poly = polygon(&p);
            if t.usage.is_road() {
                area += poly.unsigned_area();
            }
            if matches!(t.usage, world::TriangleUsage::Pavement(_)) {
                assert!(poly.intersection(&s.asphalt).unsigned_area() < 0.0001);
            }
        }
        assert!((area - s.asphalt.unsigned_area()).abs() < 0.01);
        for line in std::fs::read_to_string(_temp.directory.join("streets.txt"))
            .unwrap()
            .lines()
        {
            let mut fields = line.split_whitespace();
            if fields.next() == Some("t") {
                styles.insert(fields.next().unwrap().parse::<usize>().unwrap());
            }
        }
        assert_eq!(styles, (0..8).collect());
    }
    #[test]
    fn t_junction_preserves_far_curb_and_fillets_avoid_buildings() {
        let m = map(
            &[
                XZ { x: -40., z: 0. },
                XZ { x: 0., z: 0. },
                XZ { x: 40., z: 0. },
                XZ { x: 0., z: 40. },
            ],
            &[(0, 1, 8., 0), (1, 2, 8., 0), (1, 3, 10., 1)],
            &[],
        );
        let (_, s, _) = surfaces(&m);
        assert!(!s.asphalt.intersects(&point(XZ { x: 0., z: -4.5 })));
        assert!(s.asphalt.contains(&point(XZ { x: 0., z: -3.9 })));
        let building = vec![
            XZ { x: 4.2, z: 4.2 },
            XZ { x: 7., z: 4.2 },
            XZ { x: 7., z: 7. },
            XZ { x: 4.2, z: 7. },
        ];
        let m = map(
            &[
                XZ { x: -40., z: 0. },
                XZ { x: 0., z: 0. },
                XZ { x: 40., z: 0. },
                XZ { x: 0., z: -40. },
                XZ { x: 0., z: 40. },
            ],
            &[(0, 1, 8., 1), (1, 2, 8., 1), (3, 1, 8., 1), (1, 4, 8., 1)],
            &[building],
        );
        let (_, s, _) = surfaces(&m);
        assert!(!s.asphalt.intersects(&point(XZ { x: 4.5, z: 4.5 })));
    }
    #[test]
    fn parallel_roads_and_enclosed_blocks_keep_their_gaps() {
        let m = map(
            &[
                XZ { x: -30., z: 0. },
                XZ { x: 30., z: 0. },
                XZ { x: -30., z: 10. },
                XZ { x: 30., z: 10. },
            ],
            &[(0, 1, 6., 1), (2, 3, 6., 1)],
            &[],
        );
        let (_, s, _) = surfaces(&m);
        assert_eq!(s.asphalt.0.len(), 2);
        assert!(!s.asphalt.intersects(&point(XZ { x: 0., z: 5. })));
        let m = map(
            &[
                XZ { x: -20., z: -20. },
                XZ { x: 20., z: -20. },
                XZ { x: 20., z: 20. },
                XZ { x: -20., z: 20. },
            ],
            &[(0, 1, 6., 1), (1, 2, 6., 1), (2, 3, 6., 1), (3, 0, 6., 1)],
            &[],
        );
        let (_temp, s, loaded) = surfaces(&m);
        assert!(!s.asphalt.intersects(&point(XZ { x: 0., z: 0. })));
        for t in loaded.road_triangles().iter().filter(|t| t.usage.is_road()) {
            let p: Vec<XZ> = t
                .points
                .iter()
                .map(|p| XZ {
                    x: p.x as f64,
                    z: p.z as f64,
                })
                .collect();
            assert!(!polygon(&p).intersects(&point(XZ { x: 0., z: 0. })));
        }
    }

    #[test]
    fn indexed_obstacle_checks_match_merged_coverage_including_block_holes() {
        let building = vec![
            XZ { x: -3., z: -3. },
            XZ { x: 3., z: -3. },
            XZ { x: 3., z: 3. },
            XZ { x: -3., z: 3. },
        ];
        let m = map(
            &[
                XZ { x: -20., z: -20. },
                XZ { x: 20., z: -20. },
                XZ { x: 20., z: 20. },
                XZ { x: -20., z: 20. },
            ],
            &[(0, 1, 6., 1), (1, 2, 6., 1), (2, 3, 6., 1), (3, 0, 6., 1)],
            std::slice::from_ref(&building),
        );
        let (_, s, _) = surfaces(&m);
        let index = build_surfaces::PolygonIndex::new(&s.obstacles);
        let footprint = polygon(&building);
        for x in (-40..=40).step_by(4) {
            for z in (-40..=40).step_by(4) {
                let probe = point(XZ {
                    x: x as f64,
                    z: z as f64,
                })
                .buffer(0.18);
                assert_eq!(
                    index.intersects(&probe),
                    s.asphalt.intersects(&probe) || footprint.intersects(&probe),
                    "mismatched collision at {x},{z}"
                );
            }
        }
    }

    #[test]
    fn large_polygon_triangulation_preserves_area_and_holes_after_splitting() {
        let ring = |n: usize, radius: f64| {
            line(
                &(0..n)
                    .map(|i| {
                        let t = i as f64 / n as f64 * std::f64::consts::TAU;
                        XZ {
                            x: radius * t.cos(),
                            z: radius * t.sin(),
                        }
                    })
                    .collect::<Vec<_>>(),
            )
        };
        let geometry = MultiPolygon(vec![Polygon::new(ring(5000, 100.), vec![ring(2000, 50.)])]);
        let mut bytes = vec![];
        build_surfaces::triangles(&mut bytes, "t 0", &geometry).unwrap();
        let mut area = 0.;
        for row in std::str::from_utf8(&bytes).unwrap().lines() {
            let values: Vec<f64> = row
                .split_whitespace()
                .skip(2)
                .map(|s| s.parse().unwrap())
                .collect();
            let p = polygon(&[
                XZ {
                    x: values[0],
                    z: values[1],
                },
                XZ {
                    x: values[2],
                    z: values[3],
                },
                XZ {
                    x: values[4],
                    z: values[5],
                },
            ]);
            assert!(!p.contains(&point(XZ { x: 0., z: 0. })));
            area += p.unsigned_area();
        }
        assert!(
            (area - geometry.unsigned_area()).abs() < 0.01,
            "emitted {area}, expected {}",
            geometry.unsigned_area()
        );
    }

    #[test]
    fn disk_precision_does_not_emit_collapsed_triangles() {
        let tiny = polygon(&[
            XZ {
                x: 0.,
                z: 0.00000051,
            },
            XZ {
                x: 100.,
                z: 0.00000251,
            },
            XZ {
                x: 50.,
                z: 0.00000162,
            },
        ]);
        let normal = polygon(&[
            XZ { x: 0., z: 10. },
            XZ { x: 100., z: 10. },
            XZ { x: 50., z: 11. },
        ]);
        let mut bytes = vec![];
        let count = build_surfaces::triangles(&mut bytes, "t 0", &MultiPolygon(vec![tiny, normal]))
            .unwrap();
        assert_eq!(count, 1);
        let text = String::from_utf8(bytes).unwrap();
        let f: Vec<f64> = text
            .split_whitespace()
            .skip(2)
            .map(|s| s.parse().unwrap())
            .collect();
        assert!(
            cross(
                XZ {
                    x: f[2] - f[0],
                    z: f[3] - f[1]
                },
                XZ {
                    x: f[4] - f[0],
                    z: f[5] - f[1]
                }
            ) > 0.00001
        );
    }
    #[test]
    fn bend_smoothing_shares_endpoints_and_stays_within_sixty_centimetres() {
        let m = map(
            &[
                XZ { x: -30., z: 0. },
                XZ { x: 0., z: 0. },
                XZ { x: 20., z: 20. },
            ],
            &[(0, 1, 6.5, 1), (1, 2, 6.5, 1)],
            &[],
        );
        let (_, s, _) = surfaces(&m);
        assert!(dist(*s.paths[0].last().unwrap(), s.paths[1][0]) < 1e-6);
        assert!(dist(s.paths[1][0], m.road_nodes[1].xz()) > 0.01);
        for (r, path) in m.road_lines.iter().zip(&s.paths) {
            for p in path {
                assert!(
                    project(
                        &[m.road_nodes[r.ends[0]].xz(), m.road_nodes[r.ends[1]].xz()],
                        *p
                    )
                    .0 <= 0.601
                );
            }
        }
        let m = map(
            &[
                XZ { x: -30., z: 0. },
                XZ { x: 0., z: 0. },
                XZ { x: 30., z: 0. },
            ],
            &[(0, 1, 6.5, 1), (1, 2, 6.5, 1)],
            &[],
        );
        let (_, s, _) = surfaces(&m);
        assert_eq!(s.paths[0], vec![XZ { x: -30., z: 0. }, XZ { x: 0., z: 0. }]);
    }
}

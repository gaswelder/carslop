use super::*;
use geo::{
    algorithm::{
        buffer::{BufferStyle, LineCap, LineJoin},
        orient::Direction,
        unary_union,
    },
    Area, BooleanOps, BoundingRect, Buffer, Intersects, Orient, Point, TriangulateEarcut,
};
use std::{
    fmt::Write as _,
    io::{BufWriter, Write},
};

pub struct Surfaces {
    pub paths: Vec<Vec<P>>,
    pub adjacency: Vec<Vec<usize>>,
    #[cfg(test)]
    pub asphalt: MultiPolygon<f64>,
    pub obstacles: MultiPolygon<f64>,
    pub text_crc: u32,
}
/// Broad-phase queries avoid walking an entire city's polygons for every post or fillet.
pub struct PolygonIndex<'a> {
    polygons: &'a [Polygon<f64>],
    tree: rstar::RTree<rstar::primitives::GeomWithData<rstar::primitives::Rectangle<P>, usize>>,
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
                            [r.min().x, r.min().y],
                            [r.max().x, r.max().y],
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
        let envelope = rstar::AABB::from_corners([r.min().x, r.min().y], [r.max().x, r.max().y]);
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
pub fn flat_buffer(path: &[P], width: f64) -> MultiPolygon<f64> {
    line(path).buffer_with_style(
        BufferStyle::new(width)
            .line_cap(LineCap::Butt)
            .line_join(LineJoin::Round(std::f64::consts::PI / 16.0)),
    )
}
fn ribbon(path: &[P], road: &Road, adjacency: &[Vec<usize>], extra: f64) -> MultiPolygon<f64> {
    let half = road.width / 2.0 + extra;
    let mut parts = vec![flat_buffer(path, half)];
    for (n, p) in road.ends.iter().zip([path[0], *path.last().unwrap()]) {
        if adjacency[*n].len() < 3 {
            parts.push(Point::new(p[0], p[1]).buffer(half));
        }
    }
    union(&parts)
}
fn paths(map: &Map, adjacency: &[Vec<usize>]) -> Vec<Vec<P>> {
    let end = |node: usize, other: usize| {
        let p = map.nodes[node];
        if adjacency[node].len() != 2 {
            return vec![p];
        }
        let neighbor = adjacency[node]
            .iter()
            .flat_map(|i| map.roads[*i].ends)
            .find(|n| *n != node && *n != other);
        let Some(neighbor) = neighbor else {
            return vec![p];
        };
        let a = unit(sub(map.nodes[other], p));
        let b = unit(sub(map.nodes[neighbor], p));
        let d = dot(a, b);
        if d > 0.5 || d < -0.9995 {
            return vec![p];
        }
        let displacement = length(add(a, b)) * 0.25;
        let trim = 2.0_f64
            .min(dist(p, map.nodes[other]) * 0.2)
            .min(dist(p, map.nodes[neighbor]) * 0.2)
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
    map.roads
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
pub fn triangles(
    writer: &mut impl Write,
    prefix: &str,
    geometry: &MultiPolygon<f64>,
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
                let crop = polygon(&[[b[0], b[1]], [b[2], b[1]], [b[2], b[3]], [b[0], b[3]]]);
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
            let area = cross(sub(p[1], p[0]), sub(p[2], p[0]));
            // Match StreetSurface::load's minimum signed cross product.
            if area.abs() <= 0.00001 {
                continue;
            }
            if area < 0.0 {
                p.reverse();
            }
            writeln!(
                writer,
                "{prefix} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6}",
                p[0][0], p[0][1], p[1][0], p[1][1], p[2][0], p[2][1]
            )?;
            count += 1;
        }
    }
    Ok(count)
}
// Nearest road segments via an R-tree, shared by curbs and roadside controls.
#[derive(Clone)]
pub struct Segment {
    pub a: P,
    pub b: P,
    pub road: usize,
}
impl rstar::RTreeObject for Segment {
    type Envelope = rstar::AABB<P>;
    fn envelope(&self) -> Self::Envelope {
        rstar::AABB::from_corners(
            [self.a[0].min(self.b[0]), self.a[1].min(self.b[1])],
            [self.a[0].max(self.b[0]), self.a[1].max(self.b[1])],
        )
    }
}
impl rstar::PointDistance for Segment {
    fn distance_2(&self, p: &P) -> f64 {
        project(&[self.a, self.b], *p).0.powi(2)
    }
}
pub fn index(paths: &[Vec<P>]) -> rstar::RTree<Segment> {
    rstar::RTree::bulk_load(
        paths
            .iter()
            .enumerate()
            .flat_map(|(road, path)| {
                path.windows(2).map(move |p| Segment {
                    a: p[0],
                    b: p[1],
                    road,
                })
            })
            .collect(),
    )
}
pub fn build(map: &Map, city: &str, streets_path: &Path, report_path: &Path) -> Result<Surfaces> {
    let mut adjacency = vec![vec![]; map.nodes.len()];
    for (i, r) in map.roads.iter().enumerate() {
        for n in r.ends {
            adjacency[n].push(i);
        }
    }
    let paths = paths(map, &adjacency);
    let buildings = union(
        &map.buildings
            .iter()
            .map(|b| polygon(&b.points).buffer(0.1))
            .collect::<Vec<_>>(),
    );
    let building_index = PolygonIndex::new(&buildings);
    let mut patches = Vec::new();
    let mut patch_kinds = Vec::new();
    let mut patch_nodes = Vec::new();
    for (node, arms) in adjacency.iter().enumerate().filter(|(_, a)| a.len() >= 3) {
        let p = map.nodes[node];
        let other = |i: usize| {
            let r = &map.roads[i];
            map.nodes[if r.ends[0] == node {
                r.ends[1]
            } else {
                r.ends[0]
            }]
        };
        let mut arms = arms.clone();
        arms.sort_by(|a, b| {
            let a = sub(other(*a), p);
            let b = sub(other(*b), p);
            a[1].atan2(a[0]).total_cmp(&b[1].atan2(b[0]))
        });
        for j in 0..arms.len() {
            let ia = arms[j];
            let ib = arms[(j + 1) % arms.len()];
            let a = unit(sub(other(ia), p));
            let b = unit(sub(other(ib), p));
            let angle = (b[1].atan2(b[0]) - a[1].atan2(a[0])).rem_euclid(std::f64::consts::TAU);
            if angle <= 40.0_f64.to_radians() || angle >= 165.0_f64.to_radians() {
                continue;
            }
            let qa = add(p, mul([-a[1], a[0]], map.roads[ia].width / 2.0));
            let qb = add(p, mul([b[1], -b[0]], map.roads[ib].width / 2.0));
            let delta = sub(qb, qa);
            let den = cross(a, b);
            let sa = cross(delta, b) / den;
            let sb = cross(delta, a) / den;
            let kind = map.roads[ia].kind.min(map.roads[ib].kind);
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
                patches.push(poly);
                patch_kinds.push(kind);
                patch_nodes.push(node);
            }
        }
    }
    let buffers: Vec<_> = map
        .roads
        .iter()
        .zip(&paths)
        .map(|(r, p)| ribbon(p, r, &adjacency, 0.0))
        .collect();
    let mut cores = vec![];
    let mut core_kinds = vec![];
    for (n, arms) in adjacency.iter().enumerate().filter(|(_, a)| a.len() >= 3) {
        let width = arms
            .iter()
            .map(|i| map.roads[*i].width / 2.0)
            .fold(f64::INFINITY, f64::min);
        let p = map.nodes[n];
        cores.push(Point::new(p[0], p[1]).buffer(width));
        core_kinds.push(arms.iter().map(|i| map.roads[*i].kind).min().unwrap());
    }
    println!("  Unioning carriageways and pavement...");
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
                .zip(&map.roads)
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
            .zip(&map.roads)
            .filter(|(_, r)| r.kind == kind)
            .map(|(p, r)| ribbon(p, r, &adjacency, width))
            .collect();
        pieces.extend(
            patches
                .iter()
                .zip(&patch_kinds)
                .filter(|(_, k)| **k == kind)
                .map(|(g, _)| g.buffer(width)),
        );
        let zone = union(&pieces);
        groups.push((5 + kind, zone.difference(&paved).difference(&buildings)));
        paved = paved.union(&zone);
    }
    println!("  Building continuous road markings...");
    let junctions = union(
        &adjacency
            .iter()
            .enumerate()
            .filter(|(_, a)| a.len() > 2)
            .map(|(n, a)| {
                let p = map.nodes[n];
                Point::new(p[0], p[1]).buffer(
                    a.iter()
                        .map(|i| map.roads[*i].width / 2.0)
                        .fold(0.0, f64::max)
                        + 3.0,
                )
            })
            .collect::<Vec<_>>(),
    );
    let mut phases = vec![0.0; map.roads.len()];
    let mut by_way: BTreeMap<i64, Vec<usize>> = BTreeMap::new();
    for (i, r) in map.roads.iter().enumerate() {
        by_way.entry(r.way).or_default().push(i);
    }
    for ids in by_way.values() {
        let mut pending: std::collections::BTreeSet<_> = ids.iter().copied().collect();
        while !pending.is_empty() {
            let incoming: std::collections::BTreeSet<_> =
                pending.iter().map(|i| map.roads[*i].ends[1]).collect();
            let mut current = *pending
                .iter()
                .find(|i| !incoming.contains(&map.roads[**i].ends[0]))
                .unwrap_or(pending.first().unwrap());
            let mut distance = 0.0;
            while pending.remove(&current) {
                phases[current] = distance;
                distance += path_length(&paths[current]);
                let Some(next) = pending
                    .iter()
                    .find(|i| map.roads[**i].ends[0] == map.roads[current].ends[1])
                    .copied()
                else {
                    break;
                };
                current = next;
            }
        }
    }
    let offset = |path: &[P], amount: f64| -> Vec<P> {
        path.iter()
            .enumerate()
            .map(|(i, p)| {
                let before = path[i.saturating_sub(1)];
                let after = path[(i + 1).min(path.len() - 1)];
                let tangent = unit(sub(after, before));
                add(*p, mul([-tangent[1], tangent[0]], amount))
            })
            .collect()
    };
    let mut marks = vec![];
    for (i, (path, r)) in paths.iter().zip(&map.roads).enumerate() {
        if r.kind == 0 {
            for side in [-1.0, 1.0] {
                marks.push(flat_buffer(
                    &offset(path, side * (r.width / 2.0 - 0.38)),
                    0.045,
                ));
            }
        }
        let mut dashed = vec![];
        if r.direction == 0 && (r.kind == 0 || (r.kind == 1 && r.width >= 7.5)) {
            dashed.push(path.clone());
        } else if r.kind == 0 && r.direction != 0 && r.lanes > 1 {
            for lane in 1..r.lanes {
                dashed.push(offset(
                    path,
                    -(r.width - 0.9) / 2.0 + (r.width - 0.9) * lane as f64 / r.lanes as f64,
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
    groups.push((4, union(&marks).intersection(&inner).difference(&junctions)));
    let path = streets_path;
    let mut file = BufWriter::new(fs::File::create(&path)?);
    writeln!(
        file,
        "STREETS 1 {} {:08x}",
        map.roads.len(),
        crate::street_surface::crc32(city)
    )?;
    if let Some(n) = patch_nodes
        .iter()
        .min_by(|a, b| length(map.nodes[**a]).total_cmp(&length(map.nodes[**b])))
    {
        writeln!(file, "j {n}")?;
    }
    for (i, path) in paths.iter().enumerate() {
        let mut row = format!("p {i} {}", path.len());
        for p in path {
            write!(row, " {:.6} {:.6}", p[0], p[1])?;
        }
        writeln!(file, "{row}")?;
    }
    println!("  Triangulating surfaces...");
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
    let mut curbs = 0;
    for poly in &asphalt.0 {
        for ring in std::iter::once(poly.exterior()).chain(poly.interiors()) {
            for pair in ring.0.windows(2) {
                let a = [pair[0].x, pair[0].y];
                let b = [pair[1].x, pair[1].y];
                if dist(a, b) < 0.001 {
                    continue;
                }
                let tangent = unit(sub(b, a));
                let kind =
                    map.roads[tree.nearest_neighbor(&mul(add(a, b), 0.5)).unwrap().road].kind;
                writeln!(
                    file,
                    "c {kind} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6}",
                    a[0], a[1], b[0], b[1], -tangent[1], tangent[0]
                )?;
                curbs += 1;
            }
        }
    }
    file.flush()?;
    fs::write(
        report_path,
        serde_json::to_string_pretty(
            &serde_json::json!({"compiler":"native-rust-v1","road_area_m2":asphalt.unsigned_area(),"junction_fillets":patches.len(),"curb_segments":curbs,"surface_triangles":counts,"max_centreline_rounding_metres":0.6}),
        )? + "\n",
    )?;
    let text_crc = crc_file(&path)?;
    // Index the small input pieces, rather than a city-wide connected polygon.
    // A probe intersects their union exactly when it intersects any one piece.
    let original_footprints =
        MultiPolygon(map.buildings.iter().map(|b| polygon(&b.points)).collect());
    let obstacles = MultiPolygon(
        buffers
            .into_iter()
            .chain(patches)
            .chain(cores)
            .chain(std::iter::once(original_footprints))
            .flat_map(|g| g.0)
            .collect(),
    );
    Ok(Surfaces {
        paths,
        adjacency,
        #[cfg(test)]
        asphalt,
        obstacles,
        text_crc,
    })
}
pub fn crc_file(path: &Path) -> Result<u32> {
    use std::io::Read;
    let mut file = fs::File::open(path)?;
    let mut crc = !0u32;
    let mut bytes = [0; 65536];
    loop {
        let n = file.read(&mut bytes)?;
        if n == 0 {
            break;
        }
        for byte in &bytes[..n] {
            crc ^= *byte as u32;
            for _ in 0..8 {
                crc = (crc >> 1) ^ if crc & 1 != 0 { 0xedb88320 } else { 0 };
            }
        }
    }
    Ok(!crc)
}

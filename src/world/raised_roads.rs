//! Raised road triangles and their spatial index share one owner.
use super::{HeightField, Location, RoadRise};
use crate::geometry;
use crate::geometry::{XYZ, XZ};
use geo::{BooleanOps, MultiPolygon, TriangulateEarcut};
use rstar::{RTree, RTreeObject, AABB};

#[derive(Debug)]
struct IndexedRise {
    index: usize,
    bounds: AABB<XZ<f64>>,
}
impl RTreeObject for IndexedRise {
    type Envelope = AABB<XZ<f64>>;
    fn envelope(&self) -> Self::Envelope {
        self.bounds
    }
}

#[derive(Debug)]
pub struct RaisedRoads {
    triangles: Vec<RoadRise>,
    index: RTree<IndexedRise>,
}
impl RoadRise {
    /// Linear height offset inside this triangle, with tolerance for shared edges.
    pub fn offset(&self, p: XZ<f32>) -> Option<f32> {
        let [a, b, c] = self.points;
        let cross = |a: XZ<f32>, b: XZ<f32>| a.x as f64 * b.z as f64 - a.z as f64 * b.x as f64;
        let delta = |a: XYZ, b: XYZ| XZ {
            x: a.x - b.x,
            z: a.z - b.z,
        };
        let p = p.at_height(0.0);
        let denominator = cross(delta(b, a), delta(c, a));
        let u = cross(delta(p, a), delta(c, a)) / denominator;
        let v = cross(delta(b, a), delta(p, a)) / denominator;
        if u >= -0.0001 && v >= -0.0001 && u + v <= 1.0001 {
            Some((a.y as f64 + u * (b.y - a.y) as f64 + v * (c.y - a.y) as f64).max(0.0) as f32)
        } else {
            None
        }
    }
}
fn bounds(points: impl Iterator<Item = XZ<f64>>) -> AABB<XZ<f64>> {
    let mut min = XZ {
        x: f64::INFINITY,
        z: f64::INFINITY,
    };
    let mut max = XZ {
        x: f64::NEG_INFINITY,
        z: f64::NEG_INFINITY,
    };
    for point in points {
        min.x = min.x.min(point.x);
        min.z = min.z.min(point.z);
        max.x = max.x.max(point.x);
        max.z = max.z.max(point.z);
    }
    AABB::from_corners(min, max)
}
fn polygon(points: [XZ<f32>; 3]) -> MultiPolygon<f64> {
    MultiPolygon(vec![geometry::polygon(&points.map(|p| p.map(f64::from)))])
}
impl RaisedRoads {
    pub fn new(triangles: Vec<RoadRise>) -> Self {
        let index = RTree::bulk_load(
            triangles
                .iter()
                .enumerate()
                .map(|(index, rise)| IndexedRise {
                    index,
                    bounds: bounds(rise.points.iter().map(|p| p.xz().map(f64::from))),
                })
                .collect(),
        );
        Self { triangles, index }
    }
    fn offset(&self, p: XZ<f32>) -> f32 {
        self.index
            .locate_in_envelope_intersecting(&AABB::from_point(p.map(f64::from)))
            .filter_map(|item| self.triangles[item.index].offset(p))
            .fold(0.0, f32::max)
    }
    fn drape(&self, field: &HeightField, points: [XZ<f32>; 3], mut emit: impl FnMut([XYZ; 3])) {
        let mut candidates = self
            .index
            .locate_in_envelope_intersecting(&bounds(points.into_iter().map(|p| p.map(f64::from))))
            .peekable();
        if candidates.peek().is_none() {
            field.drape(points, emit);
            return;
        }
        let mut remaining = polygon(points);
        for item in candidates {
            let rise = &self.triangles[item.index];
            let footprint = polygon(rise.points.map(|p| p.xz()));
            let raised = remaining.intersection(&footprint);
            for triangle in raised
                .0
                .iter()
                .flat_map(|polygon| polygon.earcut_triangles())
            {
                let points = triangle.to_array().map(|p| XZ {
                    x: p.x as f32,
                    z: p.y as f32,
                });
                field.drape(points, |mut triangle| {
                    for point in &mut triangle {
                        point.y += rise.offset(point.xz()).unwrap_or(0.0);
                    }
                    emit(triangle);
                });
            }
            remaining = remaining.difference(&footprint);
        }
        // Preserve the original fast drape when this triangle has no nearby rises.
        if remaining == polygon(points) {
            field.drape(points, emit);
        } else {
            for triangle in remaining
                .0
                .iter()
                .flat_map(|polygon| polygon.earcut_triangles())
            {
                field.drape(
                    triangle.to_array().map(|p| XZ {
                        x: p.x as f32,
                        z: p.y as f32,
                    }),
                    &mut emit,
                );
            }
        }
    }
    fn segment(&self, field: &HeightField, a: XZ<f32>, b: XZ<f32>) -> Vec<XYZ> {
        let dx = (b.x - a.x) as f64;
        let dz = (b.z - a.z) as f64;
        let length_squared = dx * dx + dz * dz;
        if length_squared < 0.000001 {
            return field.segment(a, b);
        }
        let mut stations: Vec<f64> = field
            .segment(a, b)
            .iter()
            .map(|p| ((p.x - a.x) as f64 * dx + (p.z - a.z) as f64 * dz) / length_squared)
            .collect();
        for item in self
            .index
            .locate_in_envelope_intersecting(&bounds([a, b].into_iter().map(|p| p.map(f64::from))))
        {
            let points = self.triangles[item.index].points;
            for i in 0..3 {
                let c = points[i];
                let d = points[(i + 1) % 3];
                let ex = (d.x - c.x) as f64;
                let ez = (d.z - c.z) as f64;
                let denominator = dx * ez - dz * ex;
                if denominator.abs() < 0.00000001 {
                    continue;
                }
                let cx = (c.x - a.x) as f64;
                let cz = (c.z - a.z) as f64;
                let t = (cx * ez - cz * ex) / denominator;
                let u = (cx * dz - cz * dx) / denominator;
                if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
                    stations.push(t);
                }
            }
        }
        stations.sort_by(f64::total_cmp);
        stations.dedup_by(|a, b| (*a - *b).abs() < 0.000001);
        stations
            .into_iter()
            .map(|t| {
                let p = XZ {
                    x: a.x + dx as f32 * t as f32,
                    z: a.z + dz as f32 * t as f32,
                };
                p.at_height(field.height(p) + self.offset(p))
            })
            .collect()
    }
}
impl Location {
    pub fn road_rise(&self, p: XZ<f32>) -> f32 {
        self.raised_roads.offset(p)
    }
    pub fn drape_road_triangle(&self, points: [XZ<f32>; 3], emit: impl FnMut([XYZ; 3])) {
        let field = self
            .elevation()
            .expect("raised road rendering requires elevation");
        self.raised_roads.drape(field, points, emit);
    }
    pub fn raised_road_segment(&self, a: XZ<f32>, b: XZ<f32>) -> Vec<XYZ> {
        let field = self
            .elevation()
            .expect("raised road rendering requires elevation");
        self.raised_roads.segment(field, a, b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::BoundingBoxXZ;

    #[test]
    fn raised_road_drape_and_contact_share_the_same_piecewise_surface() {
        let mut triangles = Vec::new();
        struct Section {
            station: f32,
            height: f32,
        }
        let sections = [
            Section {
                station: -3.,
                height: 0.,
            },
            Section {
                station: -2.,
                height: 0.1,
            },
            Section {
                station: 2.,
                height: 0.1,
            },
            Section {
                station: 3.,
                height: 0.,
            },
        ];
        for pair in sections.windows(2) {
            let a = XZ {
                x: pair[0].station,
                z: -4.,
            }
            .at_height(pair[0].height);
            let b = XZ {
                x: pair[0].station,
                z: 4.,
            }
            .at_height(pair[0].height);
            let c = XZ {
                x: pair[1].station,
                z: 4.,
            }
            .at_height(pair[1].height);
            let d = XZ {
                x: pair[1].station,
                z: -4.,
            }
            .at_height(pair[1].height);
            triangles.extend([
                RoadRise { points: [a, b, c] },
                RoadRise { points: [a, c, d] },
            ]);
        }
        let rises = RaisedRoads::new(triangles);
        for probe in [
            Section {
                station: -4.,
                height: 0.,
            },
            Section {
                station: -2.5,
                height: 0.05,
            },
            Section {
                station: 0.,
                height: 0.1,
            },
            Section {
                station: 2.5,
                height: 0.05,
            },
            Section {
                station: 4.,
                height: 0.,
            },
        ] {
            assert!(
                (rises.offset(XZ {
                    x: probe.station,
                    z: 0.
                }) - probe.height)
                    .abs()
                    < 0.00001
            );
        }
        let field = HeightField::generate(
            BoundingBoxXZ {
                min: XZ { x: -8., z: -8. },
                max: XZ { x: 8., z: 8. },
            },
            |p| 7. + p.x * 0.1 + p.z * 0.03,
        );
        let points = [
            XZ { x: -8., z: -6. },
            XZ { x: 8., z: -6. },
            XZ { x: 8., z: 6. },
        ];
        let mut area = 0.;
        let mut peak: f32 = 0.;
        rises.drape(&field, points, |triangle| {
            let [a, b, c] = triangle;
            let triangle_area = ((b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x)).abs() / 2.;
            if triangle_area < 0.000001 {
                return;
            }
            area += triangle_area;
            let center = XZ {
                x: (a.x + b.x + c.x) / 3.,
                z: (a.z + b.z + c.z) / 3.,
            };
            let height = (a.y + b.y + c.y) / 3.;
            assert!(
                (height - field.height(center) - rises.offset(center)).abs() < 0.00001,
                "mesh {} contact {} at {:?}: {:?}",
                height,
                field.height(center) + rises.offset(center),
                center,
                triangle
            );
            for point in triangle {
                peak = peak.max(point.y - field.height(point.xz()));
            }
        });
        assert!((area - 96.).abs() < 0.0001);
        assert!((peak - 0.1).abs() < 0.00001);
        let edge = rises.segment(&field, XZ { x: -8., z: 4. }, XZ { x: 8., z: 4. });
        for x in [-3., -2., 2., 3.] {
            assert!(edge.iter().any(|point| (point.x - x).abs() < 0.0001));
        }
        for point in edge {
            assert!(
                (point.y - field.height(point.xz()) - rises.offset(point.xz())).abs() < 0.00001
            );
        }
    }
}

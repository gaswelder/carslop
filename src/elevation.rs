//! A single triangulated height surface for terrain, pavement and ground contact.

use crate::elevation;
use crate::geometry;
use crate::geometry::{BoundingBoxXZ, XYZ, XZ};
pub use crate::world::HeightField;
pub const SEA_LEVEL: f32 = -3.35;
fn clip(polygon: &[XZ<f32>], distance: impl Fn(XZ<f32>) -> f32) -> Vec<XZ<f32>> {
    let mut result = Vec::new();
    let Some(&last) = polygon.last() else {
        return result;
    };
    let mut previous = last;
    let mut before = distance(previous);
    for &current in polygon {
        let after = distance(current);
        if (after >= 0.0) != (before >= 0.0) {
            let t = (before / (before - after)).clamp(0.0, 1.0);
            result.push(geometry::xz_from_fn(|i| {
                geometry::xz_component(previous, i)
                    + (geometry::xz_component(current, i) - geometry::xz_component(previous, i)) * t
            }));
        }
        if after >= 0.0 {
            result.push(current);
        }
        previous = current;
        before = after;
    }
    result
}

pub fn generate(bounds: BoundingBoxXZ<f32>, mut height: impl FnMut(XZ<f32>) -> f32) -> HeightField {
    let span = XZ {
        x: bounds.max.x - bounds.min.x,
        z: bounds.max.z - bounds.min.z,
    };
    let step = 4.0_f32.max(span.x.max(span.z) / 511.0);
    let columns = (span.x / step).ceil() as usize + 1;
    let rows = (span.z / step).ceil() as usize + 1;
    let mut values = Vec::with_capacity(columns * rows);
    for row in 0..rows {
        for column in 0..columns {
            values.push(height(XZ {
                x: bounds.min.x + column as f32 * step,
                z: bounds.min.z + row as f32 * step,
            }));
        }
    }
    HeightField {
        columns,
        rows,
        origin: XZ {
            x: bounds.min.x,
            z: bounds.min.z,
        },
        step,
        values,
    }
}

pub fn point(height_field: &HeightField, column: usize, row: usize) -> XYZ {
    XYZ {
        x: height_field.origin.x + column as f32 * height_field.step,
        y: height_field.values[row * height_field.columns + column],
        z: height_field.origin.z + row as f32 * height_field.step,
    }
}
pub fn height(height_field: &HeightField, p: XZ<f32>) -> f32 {
    let x = ((p.x - height_field.origin.x) / height_field.step)
        .clamp(0.0, (height_field.columns - 1) as f32);
    let z = ((p.z - height_field.origin.z) / height_field.step)
        .clamp(0.0, (height_field.rows - 1) as f32);
    let col = (x.floor() as usize).min(height_field.columns - 2);
    let row = (z.floor() as usize).min(height_field.rows - 2);
    let x = x - col as f32;
    let z = z - row as f32;
    let a = height_field.values[row * height_field.columns + col];
    let b = height_field.values[row * height_field.columns + col + 1];
    let d = height_field.values[(row + 1) * height_field.columns + col];
    let e = height_field.values[(row + 1) * height_field.columns + col + 1];
    if z <= x {
        a + (b - a) * x + (e - b) * z
    } else {
        a + (e - d) * x + (d - a) * z
    }
}
/// Split on grid lines and cell diagonals so a curb/wall edge follows the
/// exact same planar surface as pavement, including where its slope changes.
pub fn segment(height_field: &HeightField, a: XZ<f32>, b: XZ<f32>) -> Vec<XYZ> {
    let delta = XZ {
        x: b.x - a.x,
        z: b.z - a.z,
    };
    let mut ts = vec![0.0, 1.0];
    for axis in 0..2 {
        if geometry::xz_component(delta, axis).abs() < 0.00001 {
            continue;
        }
        let count = if axis == 0 {
            height_field.columns
        } else {
            height_field.rows
        };
        let lo = (((geometry::xz_component(a, axis).min(geometry::xz_component(b, axis))
            - geometry::xz_component(height_field.origin, axis))
            / height_field.step)
            .floor() as i32)
            .max(0);
        let hi = (((geometry::xz_component(a, axis).max(geometry::xz_component(b, axis))
            - geometry::xz_component(height_field.origin, axis))
            / height_field.step)
            .ceil() as i32)
            .min(count as i32 - 1);
        for i in lo..=hi {
            let t = (geometry::xz_component(height_field.origin, axis)
                + i as f32 * height_field.step
                - geometry::xz_component(a, axis))
                / geometry::xz_component(delta, axis);
            if t > 0.000001 && t < 0.999999 {
                ts.push(t);
            }
        }
    }
    ts.sort_by(f32::total_cmp);
    ts.dedup_by(|a, b| (*a - *b).abs() < 0.000001);
    let mut extra = Vec::new();
    if (delta.x - delta.z).abs() > 0.00001 {
        for pair in ts.windows(2) {
            let t = (pair[0] + pair[1]) * 0.5;
            let cell: XZ<f32> = geometry::xz_from_fn(|i| {
                geometry::xz_component(height_field.origin, i)
                    + ((geometry::xz_component(a, i) + geometry::xz_component(delta, i) * t
                        - geometry::xz_component(height_field.origin, i))
                        / height_field.step)
                        .floor()
                        * height_field.step
            });
            let crossing = (cell.x - cell.z - a.x + a.z) / (delta.x - delta.z);
            if crossing > pair[0] + 0.000001 && crossing < pair[1] - 0.000001 {
                extra.push(crossing);
            }
        }
    }
    ts.extend(extra);
    ts.sort_by(f32::total_cmp);
    ts.into_iter()
        .map(|t| {
            let x = a.x + delta.x * t;
            let z = a.z + delta.z * t;
            XYZ {
                x,
                y: height(height_field, XZ { x, z }),
                z,
            }
        })
        .collect()
}
/// Clip flat map triangles onto individual height-grid planes. Merely
/// lifting the original large triangles would bury roads or leave gaps.
pub fn drape(height_field: &HeightField, points: [XZ<f32>; 3], mut emit: impl FnMut([XYZ; 3])) {
    let min: XZ<f32> = geometry::xz_from_fn(|i| {
        points
            .iter()
            .map(|p| geometry::xz_component(*p, i))
            .fold(f32::INFINITY, f32::min)
    });
    let max: XZ<f32> = geometry::xz_from_fn(|i| {
        points
            .iter()
            .map(|p| geometry::xz_component(*p, i))
            .fold(f32::NEG_INFINITY, f32::max)
    });
    let lo: [usize; 2] = std::array::from_fn(|i| {
        (((geometry::xz_component(min, i) - geometry::xz_component(height_field.origin, i))
            / height_field.step)
            .floor()
            .max(0.0) as usize)
            .min(if i == 0 {
                height_field.columns - 2
            } else {
                height_field.rows - 2
            })
    });
    let hi: [usize; 2] = std::array::from_fn(|i| {
        (((geometry::xz_component(max, i) - geometry::xz_component(height_field.origin, i))
            / height_field.step)
            .floor()
            .max(0.0) as usize)
            .min(if i == 0 {
                height_field.columns - 2
            } else {
                height_field.rows - 2
            })
    });
    for row in lo[1]..=hi[1] {
        for column in lo[0]..=hi[0] {
            let x = height_field.origin.x + column as f32 * height_field.step;
            let z = height_field.origin.z + row as f32 * height_field.step;
            let mut polygon = points.to_vec();
            for (axis, value, sign) in [
                (0, x, 1.0),
                (0, x + height_field.step, -1.0),
                (1, z, 1.0),
                (1, z + height_field.step, -1.0),
            ] {
                polygon = elevation::clip(&polygon, |p| {
                    (geometry::xz_component(p, axis) - value) * sign
                });
                if polygon.len() < 3 {
                    break;
                }
            }
            if polygon.len() < 3 {
                continue;
            }
            for side in [-1.0, 1.0] {
                let part = elevation::clip(&polygon, |p| ((p.x - x) - (p.z - z)) * side);
                for i in 1..part.len().saturating_sub(1) {
                    let tri = [part[0], part[i], part[i + 1]];
                    emit(tri.map(|p| geometry::at_height(p, height(height_field, p))));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::elevation;
    use crate::geometry;

    use super::*;
    fn grid() -> HeightField {
        HeightField {
            columns: 3,
            rows: 3,
            origin: XZ { x: 0.0, z: 0.0 },
            step: 10.0,
            values: vec![0.0, 2.0, 4.0, 1.0, 6.0, 5.0, 2.0, 4.0, 8.0],
        }
    }
    #[test]
    fn sampling_and_draping_use_the_same_planes_across_cells() {
        let grid = grid();
        assert_eq!(elevation::height(&(grid), XZ { x: 10.0, z: 10.0 }), 6.0);
        let points = [
            XZ { x: 1.0, z: 1.0 },
            XZ { x: 19.0, z: 3.0 },
            XZ { x: 3.0, z: 19.0 },
        ];
        let mut area = 0.0;
        elevation::drape(&grid, points, |tri| {
            let a = tri[0];
            let b = tri[1];
            let c = tri[2];
            area += ((b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x)).abs() * 0.5;
            let center: XYZ = geometry::xyz_from_fn(|i| {
                (geometry::xyz_component(a, i)
                    + geometry::xyz_component(b, i)
                    + geometry::xyz_component(c, i))
                    / 3.0
            });
            assert!(
                (center.y
                    - elevation::height(
                        &(grid),
                        XZ {
                            x: center.x,
                            z: center.z
                        }
                    ))
                .abs()
                    < 0.0001
            );
        });
        assert!((area - 160.0).abs() < 0.001);
        let points = elevation::segment(&grid, XZ { x: 1.0, z: 2.0 }, XZ { x: 19.0, z: 14.0 });
        assert!(points.len() > 3);
        for pair in points.windows(2) {
            let p: XYZ = geometry::xyz_from_fn(|i| {
                (geometry::xyz_component(pair[0], i) + geometry::xyz_component(pair[1], i)) * 0.5
            });
            assert!((p.y - elevation::height(&(grid), XZ { x: p.x, z: p.z })).abs() < 0.0001);
        }
    }
}

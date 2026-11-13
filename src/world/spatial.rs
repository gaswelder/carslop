//! Shared low-level grid and segment math. Geometry owners keep the grids private.
use crate::geometry::{BoundingBoxXZ, XZ};
use std::collections::HashMap;

pub const CELL: f32 = 32.0;

pub fn cell(p: XZ<f32>) -> (i32, i32) {
    ((p.x / CELL).floor() as i32, (p.z / CELL).floor() as i32)
}
pub fn insert(
    grid: &mut HashMap<(i32, i32), Vec<usize>>,
    bounds: BoundingBoxXZ<f32>,
    id: usize,
) {
    let a = cell(XZ {
        x: bounds.min.x,
        z: bounds.min.z,
    });
    let b = cell(XZ {
        x: bounds.max.x,
        z: bounds.max.z,
    });
    for x in a.0..=b.0 {
        for z in a.1..=b.1 {
            grid.entry((x, z)).or_default().push(id);
        }
    }
}
pub fn bounds(points: impl Iterator<Item = XZ<f32>>, pad: f32) -> BoundingBoxXZ<f32> {
    let mut result = BoundingBoxXZ {
        min: XZ {
            x: f32::INFINITY,
            z: f32::INFINITY,
        },
        max: XZ {
            x: f32::NEG_INFINITY,
            z: f32::NEG_INFINITY,
        },
    };
    for p in points {
        result.min.x = result.min.x.min(p.x - pad);
        result.min.z = result.min.z.min(p.z - pad);
        result.max.x = result.max.x.max(p.x + pad);
        result.max.z = result.max.z.max(p.z + pad);
    }
    result
}
pub fn nearest_edge(p: XZ<f32>, a: XZ<f32>, b: XZ<f32>) -> XZ<f32> {
    let d = XZ {
        x: b.x - a.x,
        z: b.z - a.z,
    };
    let t = (((p.x - a.x) * d.x + (p.z - a.z) * d.z) / (d.x * d.x + d.z * d.z).max(0.0001))
        .clamp(0.0, 1.0);
    XZ {
        x: a.x + d.x * t,
        z: a.z + d.z * t,
    }
}

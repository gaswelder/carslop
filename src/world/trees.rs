//! Static trunk contacts, derived from existing vegetation records.

use crate::model;
use crate::model::map_details::TreeKind;
use crate::model::world::Tree;
use crate::model::world::WorldLocation;
use crate::physics;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;

const MAX_TRUNK_RADIUS: f32 = 0.27;

pub struct Trunk {
    pub radius: f32,
    pub top_offset: f32,
    pub base_offset: f32,
}

pub fn authored_trunk(tree: &Tree) -> Trunk {
    Trunk {
        radius: 0.14,
        top_offset: tree.height_meters * (2.2 / 4.3) - 0.15,
        base_offset: -0.15,
    }
}

pub fn mapped_trunk(world: &WorldLocation, tree: &Tree) -> Trunk {
    // Same stable sample as the city's visual variation(tree.id, 24).
    let mut value = tree
        .id
        .wrapping_add(24_u64.wrapping_mul(0x9e3779b97f4a7c15));
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d049bb133111eb);
    let variation = ((value ^ (value >> 31)) >> 40) as f32 / 16777215.0;
    Trunk {
        radius: (tree.crown_radius_meters * (0.055 + variation * 0.035))
            .clamp(0.08, MAX_TRUNK_RADIUS),
        top_offset: tree.height_meters
            * if model::world::tree_kind(world, tree.id) == Some(TreeKind::Palm) {
                0.88
            } else {
                0.65
            },
        base_offset: -0.05,
    }
}

/// Contacts approximate taper and lean with an upright cylinder. Foliage
/// remains passable. Select the dimensions used by this location's renderer.
fn trunk(world: &WorldLocation, tree: &Tree) -> Trunk {
    if world::street_surface::has_compiled_roads(world) {
        mapped_trunk(world, tree)
    } else {
        authored_trunk(tree)
    }
}

fn overlaps(world: &WorldLocation, tree: &Tree, trunk: &Trunk, position: XYZ, height: f32) -> bool {
    let ground = world::elevation::world_height(world, tree.coords);
    let bottom = position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT;
    bottom <= ground + trunk.top_offset && bottom + height >= ground + trunk.base_offset
}

pub fn clearance(world: &WorldLocation, position: XYZ, height: f32) -> f32 {
    let reach = world::locations::CAR_RADIUS + MAX_TRUNK_RADIUS + 0.15;
    world::landscape::nearby_trees(&world.ground, util::geometry::xz(position), reach)
        .filter_map(|tree| {
            let trunk = trunk(world, tree);
            overlaps(world, tree, &trunk, position, height).then(|| {
                (position.x - tree.coords.x).hypot(position.z - tree.coords.z) - trunk.radius
            })
        })
        .fold(f32::INFINITY, f32::min)
}

pub fn constrain(
    world: &WorldLocation,
    position: &mut XYZ,
    velocity: &mut XZ<f32>,
    height: f32,
) -> f32 {
    let reach = world::locations::CAR_RADIUS + MAX_TRUNK_RADIUS;
    let mut impact = 0.0_f32;
    for tree in world::landscape::nearby_trees(&world.ground, util::geometry::xz(*position), reach)
    {
        let trunk = trunk(world, tree);
        let dx = position.x - tree.coords.x;
        let dz = position.z - tree.coords.z;
        let distance = dx.hypot(dz);
        let radius = world::locations::CAR_RADIUS + trunk.radius;
        if distance >= radius || !overlaps(world, tree, &trunk, *position, height) {
            continue;
        }
        let speed = velocity.x.hypot(velocity.z);
        let normal = if distance > 0.0001 {
            XZ {
                x: dx / distance,
                z: dz / distance,
            }
        } else if speed > 0.0001 {
            XZ {
                x: -velocity.x / speed,
                z: -velocity.z / speed,
            }
        } else {
            XZ { x: 1.0, z: 0.0 }
        };
        position.x = tree.coords.x + normal.x * (radius + 0.01);
        position.z = tree.coords.z + normal.z * (radius + 0.01);
        impact = impact.max(physics::vehicle_location::deflect(
            velocity,
            XZ {
                x: -normal.x,
                z: -normal.z,
            },
        ));
    }
    impact
}

#[cfg(test)]
#[path = "tests/trees.rs"]
mod tests;

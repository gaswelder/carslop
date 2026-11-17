//! Small deterministic tire puffs, advanced only by the fixed simulation clock.

use crate::physics;
use crate::physics::car::Vehicle;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;
use std::time::Duration;

const MAX_PARTICLES: usize = 160;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Smoke,
    Dust,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Puff {
    owner: usize,
    kind: Kind,
    position: XYZ,
    velocity: XYZ,
    age: f32,
    lifetime: f32,
    radius: f32,
    opacity: f32,
}

#[derive(Clone, Copy, Default)]
struct Emitter {
    credit: [f32; 2],
    previous: Option<XYZ>,
}

pub struct TireParticles {
    puffs: Vec<Puff>,
    emitters: [Emitter; 2],
    sequence: u32,
}

impl Default for TireParticles {
    fn default() -> Self {
        Self {
            puffs: Vec::with_capacity(MAX_PARTICLES),
            emitters: [Emitter::default(); 2],
            sequence: 0,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vertex {
    pub position: [f32; 3],
    uv: [f32; 2],
    pub color: [f32; 3],
    alpha: f32,
}

#[derive(Clone, Copy)]
pub struct View {
    pub eye: XYZ,
    pub right: XYZ,
    pub up: XYZ,
    pub forward: XYZ,
}

pub fn advance(tire_particles: &mut TireParticles, dt: Duration) {
    let dt = dt.as_secs_f32();
    for p in &mut tire_particles.puffs {
        p.age += dt;
        for i in 0..3 {
            *util::geometry::xyz_component_mut(&mut p.position, i) +=
                util::geometry::xyz_component(p.velocity, i) * dt;
        }
    }
    tire_particles.puffs.retain(|p| p.age < p.lifetime);
}

pub fn reset_car(tire_particles: &mut TireParticles, owner: usize) {
    tire_particles.emitters[owner] = Emitter::default();
    tire_particles.puffs.retain(|p| p.owner != owner);
}

pub fn record(tire_particles: &mut TireParticles, owner: usize, car: &Vehicle, dt: Duration) {
    let dt = dt.as_secs_f32().min(0.1);
    let motion = car.velocity.x.hypot(car.velocity.z);
    let emitter = &mut tire_particles.emitters[owner];
    let jumped = emitter
        .previous
        .is_some_and(|p| (p.x - car.position.x).hypot(p.z - car.position.z) > 4.0);
    emitter.previous = Some(car.position);
    if jumped || (motion < 3.0 && car.wheel_spin < 0.15) {
        emitter.credit = [0.0; 2];
        return;
    }
    let centers = car.spec.design.mounts;
    let (s, c) = car.orientation.yaw.sin_cos();
    for (wheel, center) in centers[..2].iter().enumerate() {
        let x = car.position.x + center.x * c + center.z * s;
        let z = car.position.z - center.x * s + center.z * c;
        let road = world::roads::project(&car.track, XYZ { x, y: 0.0, z });
        let dusty = if world::street_surface::has_compiled_roads(&car.track) {
            let city = &car.track;
            !world::transport::paved_near(
                city,
                XYZ {
                    x,
                    y: car.position.y,
                    z,
                },
            )
        } else {
            road.offset.abs() > road.width * 0.5 - 0.6
        };
        let effect = if dusty && motion > 5.0 {
            Some((Kind::Dust, (motion / 14.0).clamp(0.0, 1.0) * 0.6))
        } else if car.tire_scrub > 0.25 {
            Some((Kind::Smoke, car.tire_scrub.clamp(0.0, 1.0)))
        } else {
            None
        };
        let Some((kind, strength)) = effect else {
            tire_particles.emitters[owner].credit[wheel] = 0.0;
            continue;
        };
        tire_particles.emitters[owner].credit[wheel] += dt * (10.0 + 10.0 * strength);
        while tire_particles.emitters[owner].credit[wheel] >= 1.0 {
            tire_particles.emitters[owner].credit[wheel] -= 1.0;
            tire_particles.sequence = tire_particles.sequence.wrapping_add(1);
            let jitter = ((tire_particles.sequence.wrapping_mul(2_654_435_761) >> 8) & 65535)
                as f32
                / 65535.0
                - 0.5;
            let smoke = kind == Kind::Smoke;
            if tire_particles.puffs.len() == MAX_PARTICLES {
                tire_particles.puffs.remove(0);
            }
            tire_particles.puffs.push(Puff {
                owner,
                kind,
                position: XYZ {
                    x,
                    y: world::transport::drive_height_near(
                        &car.track,
                        XZ { x, z },
                        car.position.y - physics::vehicle_location::VEHICLE_ORIGIN_HEIGHT,
                    ) + 0.18,
                    z,
                },
                velocity: XYZ {
                    x: car.velocity.x * 0.08 - s * 0.35 + jitter * 0.2,
                    y: if smoke { 0.48 } else { 0.30 },
                    z: car.velocity.z * 0.08 - c * 0.35 - jitter * 0.2,
                },
                age: 0.0,
                lifetime: if smoke { 1.1 } else { 0.75 },
                radius: 0.21 + jitter.abs() * 0.12,
                opacity: if smoke {
                    0.08 + strength * 0.10
                } else {
                    0.07 + strength * 0.08
                },
            });
        }
    }
}

pub fn vertices(tire_particles: &TireParticles, view: View) -> Vec<Vertex> {
    let depth = |p: &Puff| {
        (0..3)
            .map(|i| {
                (util::geometry::xyz_component(p.position, i)
                    - util::geometry::xyz_component(view.eye, i))
                    * util::geometry::xyz_component(view.forward, i)
            })
            .sum::<f32>()
    };
    let mut visible: Vec<_> = tire_particles
        .puffs
        .iter()
        .filter(|p| depth(p) > 0.1)
        .collect();
    visible.sort_by(|a, b| depth(b).total_cmp(&depth(a)));
    let mut vertices = Vec::with_capacity(visible.len() * 6);
    for p in visible {
        let t = (p.age / p.lifetime).clamp(0.0, 1.0);
        let radius = p.radius + p.age * 0.42;
        let alpha = p.opacity * (t / 0.1).clamp(0.0, 1.0) * (1.0 - t).powi(2);
        let color = match p.kind {
            Kind::Smoke => [0.67, 0.70, 0.72],
            Kind::Dust => [0.62, 0.52, 0.36],
        };
        for uv in [
            [-1.0, -1.0],
            [1.0, -1.0],
            [1.0, 1.0],
            [-1.0, -1.0],
            [1.0, 1.0],
            [-1.0, 1.0],
        ] {
            let position = std::array::from_fn(|i| {
                util::geometry::xyz_component(p.position, i)
                    + radius
                        * (util::geometry::xyz_component(view.right, i) * uv[0]
                            + util::geometry::xyz_component(view.up, i) * uv[1])
            });
            vertices.push(Vertex {
                position,
                uv,
                color,
                alpha,
            });
        }
    }
    vertices
}

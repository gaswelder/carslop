//! Small deterministic tire puffs, advanced only by the fixed simulation clock.

#[cfg(test)]
use crate::track;
use crate::vehicle::Vehicle;
use std::time::Duration;

pub const MAX_PARTICLES: usize = 160;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Smoke,
    Dust,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Puff {
    owner: usize,
    kind: Kind,
    position: [f32; 3],
    velocity: [f32; 3],
    age: f32,
    lifetime: f32,
    radius: f32,
    opacity: f32,
}

#[derive(Clone, Copy, Default)]
struct Emitter {
    credit: [f32; 2],
    previous: Option<[f32; 3]>,
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
    pub uv: [f32; 2],
    pub color: [f32; 3],
    pub alpha: f32,
}

#[derive(Clone, Copy)]
pub struct View {
    pub eye: [f32; 3],
    pub right: [f32; 3],
    pub up: [f32; 3],
    pub forward: [f32; 3],
}

impl TireParticles {
    pub fn advance(&mut self, dt: Duration) {
        let dt = dt.as_secs_f32();
        for p in &mut self.puffs {
            p.age += dt;
            for i in 0..3 {
                p.position[i] += p.velocity[i] * dt;
            }
        }
        self.puffs.retain(|p| p.age < p.lifetime);
    }

    pub fn reset_car(&mut self, owner: usize) {
        self.emitters[owner] = Emitter::default();
        self.puffs.retain(|p| p.owner != owner);
    }

    pub fn record(&mut self, owner: usize, car: &Vehicle, dt: Duration) {
        let dt = dt.as_secs_f32().min(0.1);
        let motion = car.velocity[0].hypot(car.velocity[1]);
        let emitter = &mut self.emitters[owner];
        let jumped = emitter
            .previous
            .is_some_and(|p| (p[0] - car.position[0]).hypot(p[2] - car.position[2]) > 4.0);
        emitter.previous = Some(car.position);
        if jumped || (motion < 3.0 && car.wheel_spin < 0.15) {
            emitter.credit = [0.0; 2];
            return;
        }
        let centers = car.model.wheel_centers();
        let (s, c) = car.heading.sin_cos();
        for (wheel, center) in centers[..2].iter().enumerate() {
            let x = car.position[0] + center[0] * c + center[2] * s;
            let z = car.position[2] - center[0] * s + center[2] * c;
            let road = car.track.project([x, 0.0, z]);
            let distance = (x - road.point[0]).hypot(z - road.point[2]);
            let (beyond, dusty) = if let Some(city) = car.track.city() {
                (false, !city.paved([x, z]))
            } else {
                (
                    distance > road.width * 0.5 + 0.7,
                    road.offset.abs() > road.width * 0.5 - 0.6,
                )
            };
            let effect = if beyond {
                None
            } else if dusty && motion > 5.0 {
                Some((Kind::Dust, (motion / 14.0).clamp(0.0, 1.0) * 0.6))
            } else if car.tire_scrub > 0.25 {
                Some((Kind::Smoke, car.tire_scrub.clamp(0.0, 1.0)))
            } else {
                None
            };
            let Some((kind, strength)) = effect else {
                self.emitters[owner].credit[wheel] = 0.0;
                continue;
            };
            self.emitters[owner].credit[wheel] += dt * (10.0 + 10.0 * strength);
            while self.emitters[owner].credit[wheel] >= 1.0 {
                self.emitters[owner].credit[wheel] -= 1.0;
                self.sequence = self.sequence.wrapping_add(1);
                let jitter = ((self.sequence.wrapping_mul(2_654_435_761) >> 8) & 65535) as f32
                    / 65535.0
                    - 0.5;
                let smoke = kind == Kind::Smoke;
                if self.puffs.len() == MAX_PARTICLES {
                    self.puffs.remove(0);
                }
                self.puffs.push(Puff {
                    owner,
                    kind,
                    position: [
                        x,
                        car.track
                            .city()
                            .map_or(road.point[1], |c| c.drive_height([x, z]))
                            + 0.18,
                        z,
                    ],
                    velocity: [
                        car.velocity[0] * 0.08 - s * 0.35 + jitter * 0.2,
                        if smoke { 0.48 } else { 0.30 },
                        car.velocity[1] * 0.08 - c * 0.35 - jitter * 0.2,
                    ],
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

    pub fn vertices(&self, view: View) -> Vec<Vertex> {
        let depth = |p: &Puff| {
            (0..3)
                .map(|i| (p.position[i] - view.eye[i]) * view.forward[i])
                .sum::<f32>()
        };
        let mut visible: Vec<_> = self.puffs.iter().filter(|p| depth(p) > 0.1).collect();
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
                    p.position[i] + radius * (view.right[i] * uv[0] + view.up[i] * uv[1])
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cars::CarModel;

    fn car(offset: f32) -> Vehicle {
        let road = track::TrackId::Compact.track().at_distance(50.0);
        let mut car = Vehicle::new(
            road.lane_position(offset),
            road.heading(),
            CarModel::Roadster,
        );
        car.velocity = road.tangent.map(|v| v * 10.0);
        car.speed = 10.0;
        car.tire_scrub = 0.9;
        car
    }

    fn emit(p: &mut TireParticles, car: &Vehicle, steps: usize) {
        for _ in 0..steps {
            let dt = Duration::from_secs_f64(1.0 / 60.0);
            p.advance(dt);
            p.record(0, car, dt);
        }
    }

    #[test]
    fn scenic_particles_use_scenic_height_and_verge_width() {
        let track = track::TrackId::Scenic.track();
        let road = track.at_distance(50.0);
        let mut car = Vehicle::on_track(
            road.lane_position(2.0),
            road.heading(),
            CarModel::Roadster,
            track.clone(),
        );
        car.velocity = road.tangent.map(|v| v * 10.0);
        car.tire_scrub = 0.0;
        let mut particles = TireParticles::default();
        emit(&mut particles, &car, 30);
        assert!(
            particles.puffs.is_empty(),
            "scenic verge must not use compact width"
        );
        car.position = road.lane_position(2.55);
        emit(&mut particles, &car, 30);
        assert!(!particles.puffs.is_empty());
        assert!(particles.puffs.iter().all(|p| p.kind == Kind::Dust));
        particles.reset_car(0);
        car.position = road.lane_position(0.0);
        car.tire_scrub = 0.9;
        particles.record(0, &car, Duration::from_millis(100));
        assert!(!particles.puffs.is_empty());
        for puff in &particles.puffs {
            assert_eq!(puff.kind, Kind::Smoke);
            assert!(
                (puff.position[1] - track.project(puff.position).point[1] - 0.18).abs() < 0.001
            );
        }
    }

    #[test]
    fn spinning_tires_emit_smoke_before_the_car_has_pulled_away() {
        let mut car = car(0.0);
        car.velocity = [0.0; 2];
        car.speed = 0.0;
        car.wheel_spin = 0.8;
        let mut particles = TireParticles::default();
        emit(&mut particles, &car, 30);
        assert!(!particles.puffs.is_empty());
        assert!(particles.puffs.iter().all(|p| p.kind == Kind::Smoke));
    }

    #[test]
    fn smoke_needs_motion_and_scrub_dust_needs_the_verge() {
        let mut p = TireParticles::default();
        let mut v = car(0.0);
        v.velocity = [0.0; 2];
        emit(&mut p, &v, 60);
        assert!(p.puffs.is_empty());
        v = car(0.0);
        v.tire_scrub = 0.0;
        emit(&mut p, &v, 60);
        assert!(p.puffs.is_empty());
        v.tire_scrub = 0.9;
        emit(&mut p, &v, 30);
        assert!(!p.puffs.is_empty() && p.puffs.iter().all(|p| p.kind == Kind::Smoke));
        // Place the mounted outer tire on the verge, regardless of track width.
        let half_track = v.model.wheel_centers()[0][0].abs();
        let mut edge = car(v.track.width * 0.5 - half_track);
        edge.tire_scrub = 0.0;
        let mut dust = TireParticles::default();
        emit(&mut dust, &edge, 30);
        assert!(!dust.puffs.is_empty() && dust.puffs.iter().all(|p| p.kind == Kind::Dust));
    }

    #[test]
    fn particles_are_bounded_expire_and_recovery_clears_only_its_owner() {
        let v = car(0.0);
        let mut p = TireParticles::default();
        for _ in 0..1000 {
            p.record(0, &v, Duration::from_millis(100));
            p.record(1, &v, Duration::from_millis(100));
        }
        assert_eq!(p.puffs.len(), MAX_PARTICLES);
        p.reset_car(0);
        assert!(!p.puffs.is_empty() && p.puffs.iter().all(|p| p.owner == 1));
        p.advance(Duration::from_secs(2));
        assert!(p.puffs.is_empty());
        assert!(TireParticles::default().puffs.is_empty());
    }

    #[test]
    fn fixed_step_emission_is_repeatable_and_teleports_do_not_burst() {
        let mut a = TireParticles::default();
        let mut b = TireParticles::default();
        let v = car(0.0);
        emit(&mut a, &v, 60);
        emit(&mut b, &v, 60);
        assert_eq!(a.puffs, b.puffs);
        let mut moved = car(0.0);
        moved.position[0] += 20.0;
        let count = a.puffs.len();
        a.record(0, &moved, Duration::from_millis(100));
        assert_eq!(a.puffs.len(), count);
        a.reset_car(0);
        assert!(a.puffs.is_empty());
    }

    #[test]
    fn billboards_are_finite_sorted_and_faded() {
        let mut p = TireParticles::default();
        let v = car(0.0);
        emit(&mut p, &v, 30);
        let view = View {
            eye: [0.0, 10.0, -100.0],
            right: [1.0, 0.0, 0.0],
            up: [0.0, 1.0, 0.0],
            forward: [0.0, 0.0, 1.0],
        };
        let vertices = p.vertices(view);
        assert!(!vertices.is_empty() && vertices.len() <= MAX_PARTICLES * 6);
        assert_eq!(std::mem::size_of::<Vertex>(), 9 * 4);
        let mut last = f32::INFINITY;
        for quad in vertices.as_chunks::<6>().0 {
            let z = quad.iter().map(|v| v.position[2]).sum::<f32>() / 6.0;
            assert!(z <= last + 0.001);
            last = z;
            for v in quad {
                assert!(v.position.iter().all(|x| x.is_finite()));
                assert!((0.0..=0.18).contains(&v.alpha));
            }
        }
        p.advance(Duration::from_secs(2));
        assert!(p.vertices(view).is_empty());
    }
}

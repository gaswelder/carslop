//! Vehicle placement and collision response against location geometry.
use crate::geometry::{XYZ, XZ};
use crate::track;
use crate::world::{Location, RoadSample};

/// Vehicle origin height above the road's reference elevation, in metres.
pub const VEHICLE_ORIGIN_HEIGHT: f32 = 0.48;

impl Location {
    /// Vehicle origin uses the asphalt's 5 cm reference. Sidewalks and other
    /// surfaces retain their rendered heights rather than burying the wheels.
    pub fn drive_height(&self, p: XZ<f32>) -> f32 {
        if !self.has_compiled_roads() {
            let height = self.height(p);
            if self.elevation().is_none() {
                return height;
            }
            let road = self.project(XYZ {
                x: p.x,
                y: 0.0,
                z: p.z,
            });
            return height
                - if road.offset.abs() > road.width * 0.5 {
                    0.07
                } else {
                    0.0
                };
        }
        let lift = self.paving_height(p).unwrap_or_else(|| {
            if self.ground_material(p).is_some() {
                0.005
            } else {
                -0.02
            }
        });
        self.height(p) + lift - 0.05
    }

    fn clear_for_car(&self, p: XYZ) -> bool {
        if self.has_compiled_roads() && self.road_clearance(p) > -track::CAR_RADIUS - 0.15 {
            return false;
        }
        self.building_clearance(XZ { x: p.x, z: p.z }) > track::CAR_RADIUS + 0.15
    }
    pub fn recovery(&self, p: XYZ) -> RoadSample {
        // Service roads may lead into garages. Never recover inside a footprint.
        self.candidates(p)
            .flat_map(|i| {
                let r = &self.road_lines()[i];
                let s = self.projected(i, p).1;
                [-12.0, -6.0, -3.0, 0.0, 3.0, 6.0, 12.0]
                    .into_iter()
                    .map(move |offset| {
                        self.sample(
                            i,
                            ((s.distance - r.distance + offset) / r.length).clamp(0.0, 1.0),
                        )
                    })
            })
            .filter(|s| self.clear_for_car(s.point))
            .min_by(|a, b| {
                let distance = |s: &RoadSample| (s.point.x - p.x).hypot(s.point.z - p.z);
                distance(a).total_cmp(&distance(b))
            })
            .unwrap_or_else(|| self.spawn())
    }

    pub fn spawn(&self) -> RoadSample {
        for (i, r) in self.road_lines().iter().enumerate() {
            for t in [0.5, 0.25, 0.75] {
                let mut s = self.sample(i, t);
                if self.clear_for_car(s.point) {
                    if r.direction == -1 {
                        s.tangent = s.tangent.map(|v| -v);
                    }
                    return s;
                }
            }
        }
        // A narrow authored road may be smaller than the car: its verges are
        // drivable too. Keep the spawn clear of buildings without requiring
        // every wheel to fit inside the asphalt polygon.
        if self.has_compiled_roads() {
            for i in 0..self.road_lines().len() {
                let mut sample = self.sample(i, 0.5);
                if self.building_clearance(XZ {
                    x: sample.point.x,
                    z: sample.point.z,
                }) > track::CAR_RADIUS + 0.15
                {
                    if self.road_lines()[i].direction == -1 {
                        sample.tangent = sample.tangent.map(|v| -v);
                    }
                    return sample;
                }
            }
        }
        panic!("city contains no safe spawn");
    }
    pub fn constrain(&self, position: &mut XYZ, velocity: &mut XZ<f32>) -> f32 {
        let mut impact: f32 = 0.0;
        if !self.has_compiled_roads() {
            let best = self
                .candidates(*position)
                .map(|i| {
                    let (d, s) = self.projected(i, *position);
                    (d - (s.width * 0.5 - track::CAR_RADIUS), d, s)
                })
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .unwrap();

            if best.0 > 0.0 && best.1 > 0.0001 {
                let n = XZ {
                    x: (position.x - best.2.point.x) / best.1,
                    z: (position.z - best.2.point.z) / best.1,
                };
                position.x -= n.x * best.0;
                position.z -= n.z * best.0;
                impact = deflect(velocity, n);
            }
        }
        // Check footprints, not bounding boxes: concave courtyards/streets remain open.
        for b in self.nearby_buildings(XZ {
            x: position.x,
            z: position.z,
        }) {
            let p = XZ {
                x: position.x,
                z: position.z,
            };
            let (q, outward, distance) = b.nearest(p);
            let inside = b.contains(p);
            if inside || distance < track::CAR_RADIUS {
                let n = if inside || distance < 0.0001 {
                    outward
                } else {
                    XZ {
                        x: (p.x - q.x) / distance,
                        z: (p.z - q.z) / distance,
                    }
                };
                position.x = q.x + n.x * (track::CAR_RADIUS + 0.01);
                position.z = q.z + n.z * (track::CAR_RADIUS + 0.01);
                impact = impact.max(deflect(velocity, XZ { x: -n.x, z: -n.z }));
            }
        }
        // The saved extract is a finite driving area. Low curbs are crossable;
        // the crop boundary remains solid so the car cannot leave terrain data.
        for (axis, (world_axis, min, max)) in [
            (0, self.bounding_box().min.x, self.bounding_box().max.x),
            (2, self.bounding_box().min.z, self.bounding_box().max.z),
        ]
        .into_iter()
        .enumerate()
        {
            let value = position.component(world_axis);
            *position.component_mut(world_axis) = value.clamp(
                min + track::CAR_RADIUS,
                max - track::CAR_RADIUS,
            );
            if position.component(world_axis) != value {
                let mut normal = XZ { x: 0.0, z: 0.0 };
                *normal.component_mut(axis) = (value - position.component(world_axis)).signum();
                impact = impact.max(deflect(velocity, normal));
            }
        }
        position.y = self.drive_height(XZ {
            x: position.x,
            z: position.z,
        }) + VEHICLE_ORIGIN_HEIGHT;
        impact
    }

    /// A crossing is an event, not a continuous wall force. Parallel travel or
    /// resting alongside a curb must not repeatedly slow the car.
    pub fn crossed_curb(&self, a: XZ<f32>, b: XZ<f32>) -> Option<(XZ<f32>, f32)> {
        if self.contains_road(a) == self.contains_road(b) {
            return None;
        }
        let mid = XZ {
            x: (a.x + b.x) * 0.5,
            z: (a.z + b.z) * 0.5,
        };
        let (_, normal, distance) = self.nearest_curb(mid);
        if distance > (b.x - a.x).hypot(b.z - a.z) * 0.5 + 0.01 {
            return None;
        }
        // Physical rise is small for service verges, standard for pavements.
        // The rendered support height determines the exact ride height.
        let rise = (self.paving_height(a).unwrap_or(0.005)
            - self.paving_height(b).unwrap_or(0.005))
        .abs()
        .max(0.03);
        // Cap the bump and speed penalty so taller pavements stay climbable.
        Some((normal, rise.min(0.09)))
    }
}

impl RoadSample {
    /// Place the vehicle origin at a lateral offset from the road centreline.
    pub fn vehicle_position(self, offset: f32) -> XYZ {
        let mut point = self.lane_position(offset);
        point.y += VEHICLE_ORIGIN_HEIGHT;
        point
    }
}

pub fn deflect(velocity: &mut XZ<f32>, normal: XZ<f32>) -> f32 {
    let outward = velocity.x * normal.x + velocity.z * normal.z;
    if outward > 0.0 {
        // Friction is limited by the normal impact impulse. A fixed percentage
        // of total speed on every contact makes light, repeated scrapes stop cars.
        let tangent = XZ {
            x: velocity.x - normal.x * outward,
            z: velocity.z - normal.z * outward,
        };
        let speed = tangent.x.hypot(tangent.z);
        let retained = if speed > 0.0 {
            (1.0 - 0.06 * outward / speed).max(0.0)
        } else {
            0.0
        };
        for i in 0..2 {
            *velocity.component_mut(i) =
                tangent.component(i) * retained - normal.component(i) * outward * 0.08;
        }
        outward
    } else {
        0.0
    }
}

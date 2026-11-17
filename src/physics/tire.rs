//! Planar, two-axle tire contact. Forces are solved implicitly in wheel space.
//! Cornering stiffness builds lateral force up to a shared friction circle.
//! Combined lateral and longitudinal slip reduces that circle smoothly from
//! peak grip to sliding grip. Locked wheels use the same sliding grip while
//! stationary contacts retain their full holding capacity. Foot braking
//! approximates a rolling wheel under ideal ABS.

// Regularize slip ratios near rest and preserve static holding grip.
const MIN_SLIP_SPEED: f32 = 0.5;
const SLIP_FALLOFF: f32 = 0.20;
// Extra equivalent lateral slip over which post-peak grip falls to sliding grip.
const SLIP_FALLOFF_RANGE: f32 = 0.35;
// Model tuning: longitudinal post-peak grip loss begins at 10% tread slip.
const LONGITUDINAL_PEAK_SLIP: f32 = 0.10;

// Shared tire model: lateral force per radian per newton of normal load.
// Linear load scaling gives every axle the same response per unit load.
const CORNER_STIFFNESS_PER_LOAD: f32 = 12.0;

/// Axle cornering stiffness (N/rad) from its current normal load (N).
pub fn corner_stiffness(normal_load: f32) -> f32 {
    CORNER_STIFFNESS_PER_LOAD * normal_load
}

fn capacity(contact: &Contact, wheel: &WheelVelocity) -> f32 {
    // An unloaded axle has neither grip nor stiffness; avoid 0 / 0 below.
    if contact.capacity == 0.0 {
        return 0.0;
    }
    let slip_speed = if contact.locked {
        wheel.longitudinal.hypot(wheel.lateral)
    } else {
        contact.longitudinal_slip.hypot(wheel.lateral)
    };
    let moving = ((slip_speed - MIN_SLIP_SPEED) / MIN_SLIP_SPEED).clamp(0.0, 1.0);
    let sliding = if contact.locked {
        moving
    } else {
        let speed = wheel.longitudinal.abs().max(MIN_SLIP_SPEED);
        let peak = contact.capacity / contact.corner_stiffness;
        let lateral = wheel.lateral / speed;
        // Express longitudinal slip in equivalent lateral-slip units so
        // both components contribute to one peak and one falloff curve.
        let longitudinal = contact.longitudinal_slip / speed * peak / LONGITUDINAL_PEAK_SLIP;
        moving * ((lateral.hypot(longitudinal) - peak) / SLIP_FALLOFF_RANGE).clamp(0.0, 1.0)
    };
    contact.capacity * (1.0 - SLIP_FALLOFF * sliding)
}

#[derive(Clone, Copy, Default)]
pub struct Motion {
    pub longitudinal: f32,
    pub lateral: f32,
    pub yaw_rate: f32,
}

#[derive(Clone, Copy, Default)]
pub struct Impulse {
    pub longitudinal: f32,
    pub lateral: f32,
}

pub struct Contact {
    pub offset: f32,
    pub sin: f32,
    pub cos: f32,
    pub capacity: f32,
    pub drive_force: f32,
    /// Wheel tread speed minus road speed along the wheel (m/s).
    pub longitudinal_slip: f32,
    pub brake_force: f32,
    pub corner_stiffness: f32,
    pub locked: bool,
}

struct WheelVelocity {
    longitudinal: f32,
    lateral: f32,
}

fn velocity(contact: &Contact, motion: Motion) -> WheelVelocity {
    let side = motion.lateral + contact.offset * motion.yaw_rate;
    WheelVelocity {
        longitudinal: motion.longitudinal * contact.cos + side * contact.sin,
        lateral: -motion.longitudinal * contact.sin + side * contact.cos,
    }
}

fn apply(contact: &Contact, motion: &mut Motion, impulse: Impulse, mass: f32, inertia: f32) {
    let forward = impulse.longitudinal * contact.cos - impulse.lateral * contact.sin;
    let side = impulse.longitudinal * contact.sin + impulse.lateral * contact.cos;
    motion.longitudinal += forward / mass;
    motion.lateral += side / mass;
    motion.yaw_rate += contact.offset * side / inertia;
}

struct EffectiveMass {
    along: f32,
    cross: f32,
    side: f32,
}

struct Constraint {
    mass: EffectiveMass,
    along_compliance: f32,
    side_compliance: f32,
    drive: f32,
    minimum: f32,
    maximum: f32,
    radius: f32,
}

fn new(contact: &Contact, motion: Motion, mass: f32, inertia: f32, dt: f32) -> Constraint {
    let wheel = velocity(contact, motion);
    let along = wheel.longitudinal.abs();
    let capacity = capacity(contact, &wheel);
    let radius = capacity * dt;
    let drive = contact.drive_force * dt;
    let brake = contact.brake_force * dt;
    // Project the requested driving and cornering forces together. Giving
    // driving force the entire circle would erase cornering under power or
    // engine braking, even when longitudinal tread slip is still small.
    let drive_limit = if !contact.locked && along > MIN_SLIP_SPEED {
        let side_force = contact.corner_stiffness * wheel.lateral / along;
        let demand = contact.drive_force.hypot(side_force);
        if demand > 0.0 {
            radius * contact.drive_force.abs() / demand
        } else {
            radius
        }
    } else {
        radius
    };
    let drive = drive.clamp(-drive_limit, drive_limit);
    Constraint {
        mass: EffectiveMass {
            along: 1.0 / mass + (contact.offset * contact.sin).powi(2) / inertia,
            cross: contact.offset.powi(2) * contact.sin * contact.cos / inertia,
            side: 1.0 / mass + (contact.offset * contact.cos).powi(2) / inertia,
        },
        // At speed this requests up to the pedal's brake force. As wheel
        // speed tends to zero it becomes a bounded no-slip constraint.
        along_compliance: if brake > 0.0 && !contact.locked {
            along / brake
        } else {
            0.0
        },
        side_compliance: if contact.locked {
            0.0
        } else {
            // Use the secant stiffness past saturation so implicit damping
            // follows the declining force curve while a slide unwinds.
            let stiffness = if along > MIN_SLIP_SPEED && wheel.lateral != 0.0 {
                contact
                    .corner_stiffness
                    .min(capacity * along / wheel.lateral.abs())
            } else {
                contact.corner_stiffness
            };
            if stiffness > 0.0 {
                along / (stiffness * dt)
            } else {
                0.0
            }
        },
        drive,
        minimum: (drive - brake).clamp(-radius, radius),
        maximum: (drive + brake).clamp(-radius, radius),
        radius,
    }
}

fn constraint_solve(constraint: &Constraint, velocity: WheelVelocity) -> Impulse {
    if constraint.radius == 0.0 {
        return Impulse::default();
    }
    // Minimize contact kinetic energy plus tire compliance. The brake
    // bound and friction circle are imposed together, so a saturated
    // rolling tire can trade braking force for cornering force.
    let along = velocity.longitudinal - constraint.along_compliance * constraint.drive;
    let at = |lambda: f32| {
        let a = constraint.mass.along + constraint.along_compliance + lambda;
        let b = constraint.mass.cross;
        let c = constraint.mass.side + constraint.side_compliance + lambda;
        let longitudinal = ((b * velocity.lateral - c * along) / (a * c - b * b))
            .clamp(constraint.minimum, constraint.maximum);
        Impulse {
            longitudinal,
            lateral: -(velocity.lateral + b * longitudinal) / c,
        }
    };
    if constraint.minimum >= constraint.radius || constraint.maximum <= -constraint.radius {
        return Impulse {
            longitudinal: constraint
                .minimum
                .clamp(-constraint.radius, constraint.radius),
            lateral: 0.0,
        };
    }
    let impulse = at(0.0);
    if impulse.longitudinal.hypot(impulse.lateral) <= constraint.radius {
        return impulse;
    }
    // The Lagrange multiplier for the circular bound is monotone. Solve
    // it rather than giving longitudinal force priority over lateral.
    let mut low = 0.0;
    let mut high = along.hypot(velocity.lateral) / constraint.radius
        + constraint.mass.along
        + constraint.mass.side
        + constraint.along_compliance
        + constraint.side_compliance;
    for _ in 0..32 {
        let impulse = at(high);
        if impulse.longitudinal.hypot(impulse.lateral) <= constraint.radius {
            break;
        }
        high *= 2.0;
    }
    for _ in 0..24 {
        let middle = (low + high) * 0.5;
        let impulse = at(middle);
        if impulse.longitudinal.hypot(impulse.lateral) > constraint.radius {
            low = middle;
        } else {
            high = middle;
        }
    }
    at(high)
}

pub struct Solution {
    pub motion: Motion,
    pub impulses: [Impulse; 2],
}

pub fn solve(
    initial: Motion,
    free: Motion,
    contacts: [Contact; 2],
    mass: f32,
    inertia: f32,
    dt: f32,
) -> Solution {
    let constraints = contacts
        .each_ref()
        .map(|contact| new(contact, initial, mass, inertia, dt));
    let mut motion = free;
    let mut impulses = [Impulse::default(); 2];
    // Each axle changes the velocity seen by the other through chassis yaw.
    // Accumulate and replace impulses, rather than repeatedly applying forces.
    for _ in 0..24 {
        for axle in 0..2 {
            let contact = &contacts[axle];
            apply(
                contact,
                &mut motion,
                Impulse {
                    longitudinal: -impulses[axle].longitudinal,
                    lateral: -impulses[axle].lateral,
                },
                mass,
                inertia,
            );
            impulses[axle] = constraint_solve(&constraints[axle], velocity(contact, motion));
            apply(contact, &mut motion, impulses[axle], mass, inertia);
        }
    }
    Solution { motion, impulses }
}

#[cfg(test)]
#[path = "tests/tire.rs"]
mod tests;

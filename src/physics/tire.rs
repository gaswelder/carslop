//! Planar, two-axle tire contact. Forces are solved implicitly in wheel space.
//! Rolling tires have lateral damping C_alpha / |v_long|, with a secant
//! stiffness that falls once the tire is past its peak so slides decay. A
//! bounded brake opposes wheel motion, with the same static-friction limit at
//! zero speed. The coupled impulse is constrained by the contact's friction
//! circle. Foot braking approximates a rolling wheel under ideal ABS; a
//! handbraked rear axle instead uses sliding Coulomb friction in both
//! directions.

// Slip speed below which the tire is treated as gripping, matching the
// zero-speed no-slip constraint that holds a parked car.
const MIN_SLIP_SPEED: f32 = 0.5;
// Fraction of lateral force lost once the tire is fully past its peak.
const SLIP_FALLOFF: f32 = 0.20;
// Extra slip angle (radians) over which that loss is reached.
const SLIP_FALLOFF_RANGE: f32 = 0.35;

/// Secant lateral stiffness at a given slip angle. Up to saturation this is the
/// authored cornering stiffness, so the linear region is unchanged. Past the
/// peak the force declines instead of holding a flat plateau, which lets a
/// slide decay when the driver unwinds rather than staying fully developed.
fn lateral_stiffness(along: f32, corner_stiffness: f32, capacity: f32, lateral: f32) -> f32 {
    if along <= MIN_SLIP_SPEED || capacity <= 0.0 {
        return corner_stiffness;
    }
    let alpha = lateral.abs() / along;
    let saturated = capacity / corner_stiffness;
    if alpha <= saturated {
        return corner_stiffness;
    }
    let drop = SLIP_FALLOFF * ((alpha - saturated) / SLIP_FALLOFF_RANGE).clamp(0.0, 1.0);
    (capacity * (1.0 - drop) / alpha).max(corner_stiffness * 0.02)
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
    pub drive_limit: f32,
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
    let radius = contact.capacity * dt;
    let drive = contact.drive_force * dt;
    let brake = contact.brake_force * dt;
    let limit = if brake > 0.0 {
        radius
    } else {
        contact.drive_limit * dt
    };
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
            let stiffness = lateral_stiffness(
                along,
                contact.corner_stiffness,
                contact.capacity,
                wheel.lateral,
            );
            along / (stiffness * dt)
        },
        drive,
        minimum: (drive - brake).clamp(-limit, limit),
        maximum: (drive + brake).clamp(-limit, limit),
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

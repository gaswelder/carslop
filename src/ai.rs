#[cfg(test)]
use crate::track;
use crate::vehicle::{Controls, Vehicle};
use std::f32::consts::{PI, TAU};

// Pure pursuit with speed planning from route curvature. No teleporting,
// hidden speed multiplier or separate AI movement model.
pub fn controls(car: &Vehicle, opponent: Option<&Vehicle>) -> Controls {
    let road = car.track.project(car.position);
    let speed = car.speed.max(0.0);
    let lookahead = 4.0 + speed * 0.35;
    let near = car.track.at_distance(road.distance + lookahead);
    let far = car.track.at_distance(road.distance + lookahead + 7.0);
    let turn = angle(far.heading() - road.heading()).abs();
    let spec = car.model.spec();
    let grip_scale = (spec.lateral_force / spec.mass / 12.0).sqrt();
    let (pace, max_speed) = if car.track.id == crate::track::TrackId::Scenic {
        (13.0, (car.speed_cap() * 0.72).min(29.0))
    } else {
        (10.5, spec.top_speed.min(19.0))
    };
    let desired_speed = (pace * grip_scale / (turn + 0.25).sqrt()).clamp(7.0, max_speed);
    let lane_limit = (car.track.width * 0.5 - crate::track::CAR_RADIUS).max(0.0);
    let lane = opponent
        .filter(|other| std::ptr::eq(car.track, other.track))
        .map(|other| {
            let gap =
                (other.position[0] - car.position[0]).hypot(other.position[2] - car.position[2]);
            if gap < 9.0 {
                if car.track.project(other.position).offset > 0.0 {
                    -1.6
                } else {
                    1.6
                }
            } else {
                0.0
            }
        })
        .unwrap_or(0.0_f32)
        .clamp(-lane_limit, lane_limit);
    let target = near.lane_position(lane);
    let heading = (target[0] - car.position[0]).atan2(target[2] - car.position[2]);
    let error = angle(heading - car.heading);
    Controls {
        steering: (-error * spec.wheelbase).clamp(-1.0, 1.0),
        throttle: ((desired_speed - speed) * 0.7).clamp(0.0, 1.0),
        brake: ((speed - desired_speed - 0.5) * 0.35).clamp(0.0, 1.0),
        handbrake: false,
    }
}

pub fn angle(value: f32) -> f32 {
    (value + PI).rem_euclid(TAU) - PI
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::race::RaceProgress;
    use std::time::Duration;

    #[test]
    fn scenic_pursuit_uses_the_cars_route_and_ignores_other_route_opponents() {
        let route = track::TrackId::Scenic.track();
        let road = route.at_distance(600.0);
        let mut car = Vehicle::on_track(
            road.lane_position(0.0),
            road.heading(),
            crate::cars::CarModel::Roadster,
            route,
        );
        car.speed = 12.0;
        let planned = controls(&car, None);
        let near = route.at_distance(road.distance + 4.0 + car.speed * 0.35);
        let target = near.lane_position(0.0);
        let heading = (target[0] - car.position[0]).atan2(target[2] - car.position[2]);
        let expected =
            (-angle(heading - car.heading) * car.model.spec().wheelbase).clamp(-1.0, 1.0);
        assert!((planned.steering - expected).abs() < 0.001);
        let other_route = Vehicle::new(car.position, car.heading, car.model);
        let isolated = controls(&car, Some(&other_route));
        assert_eq!(planned.steering, isolated.steering);
        assert_eq!(planned.throttle, isolated.throttle);
        assert_eq!(planned.brake, isolated.brake);
    }

    #[test]
    fn speed_planning_keeps_compact_pace_but_uses_highway_calibration() {
        for model in crate::cars::ALL {
            let compact = track::TrackId::Compact.track();
            let compact_road = compact.at_distance(0.0);
            let mut compact_car = Vehicle::on_track(
                compact_road.lane_position(0.0),
                compact_road.heading(),
                model,
                compact,
            );
            compact_car.speed = 20.0;
            let compact_plan = controls(&compact_car, None);
            assert!(
                compact_plan.brake > 0.0,
                "compact route should retain old cautious pace for {model:?}"
            );

            let scenic = track::TrackId::Scenic.track();
            let scenic_road = scenic.at_distance(0.0);
            let mut scenic_car = Vehicle::on_track(
                scenic_road.lane_position(0.0),
                scenic_road.heading(),
                model,
                scenic,
            );
            scenic_car.speed = 20.0;
            let scenic_plan = controls(&scenic_car, None);
            assert_eq!(
                scenic_plan.brake, 0.0,
                "Highway pace should not inherit compact braking for {model:?}"
            );
            assert!(
                scenic_plan.throttle > 0.0,
                "Highway should exercise long-road tuning for {model:?}"
            );
        }
    }

    #[test]
    fn opponent_completes_a_race_using_shared_physics() {
        let start = track::at_distance(1.0);
        let mut car = Vehicle::new(
            start.lane_position(0.0),
            start.heading(),
            crate::cars::CarModel::Roadster,
        );
        let mut race = RaceProgress::new();
        let dt = Duration::from_secs_f64(1.0 / 60.0);
        for _ in 0..60 * 180 {
            let previous = car.position;
            car.update(dt, controls(&car, None));
            race.update(previous, car.position, dt);
            assert!(car.position.iter().all(|v| v.is_finite()));
            if race.finish_time.is_some() {
                break;
            }
        }
        assert!(
            race.finish_time.is_some(),
            "AI stuck: lap={} gate={} position={:?} speed={}",
            race.completed_laps,
            race.next_gate,
            car.position,
            car.speed
        );
    }
}

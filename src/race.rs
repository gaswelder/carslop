use crate::track;
use std::time::Duration;

pub const LAPS: u32 = 3;
pub const GATES: usize = 4;

#[derive(Default)]
pub struct RaceProgress {
    pub completed_laps: u32,
    pub next_gate: usize,
    pub elapsed: f32,
    pub lap_started: f32,
    pub lap_times: Vec<f32>,
    pub finish_time: Option<f32>,
}

impl RaceProgress {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn gate(&self) -> track::RoadSample {
        let track = track::TrackId::Compact.track();
        track.at_distance(track.length() * (self.next_gate + 1) as f32 / GATES as f32)
    }

    pub fn update(&mut self, previous: [f32; 3], current: [f32; 3], dt: Duration) {
        if self.finish_time.is_some() {
            return;
        }
        let track = track::TrackId::Compact.track();
        let gate = self.gate();
        let signed = |p: [f32; 3]| {
            (p[0] - gate.point[0]) * gate.tangent[0] + (p[2] - gate.point[2]) * gate.tangent[1]
        };
        let before = signed(previous);
        let after = signed(current);
        let old_elapsed = self.elapsed;
        self.elapsed += dt.as_secs_f32();
        // Forward crossings only; intersect the movement with the gate plane,
        // then check its width so a distant parallel road cannot count.
        if before <= 0.0 && after > 0.0 {
            let t = (-before / (after - before)).clamp(0.0, 1.0);
            let crossing: [f32; 3] =
                std::array::from_fn(|axis| previous[axis] + (current[axis] - previous[axis]) * t);
            let n = gate.normal();
            let offset =
                (crossing[0] - gate.point[0]) * n[0] + (crossing[2] - gate.point[2]) * n[1];
            if offset.abs() <= track.width * 0.5 {
                self.next_gate += 1;
                if self.next_gate == GATES {
                    let crossing_time = old_elapsed + dt.as_secs_f32() * t;
                    self.lap_times.push(crossing_time - self.lap_started);
                    self.lap_started = crossing_time;
                    self.completed_laps += 1;
                    self.next_gate = 0;
                    if self.completed_laps == LAPS {
                        self.elapsed = crossing_time;
                        self.finish_time = Some(crossing_time);
                    }
                }
            }
        }
    }

    pub fn best_lap(&self) -> Option<f32> {
        self.lap_times.iter().copied().reduce(f32::min)
    }

    pub fn score(&self, position: [f32; 3]) -> f32 {
        let track = track::TrackId::Compact.track();
        if self.finish_time.is_some() {
            return LAPS as f32 * track.length();
        }
        let distance = track.project(position).distance;
        let section = track.length() / GATES as f32;
        // Missing a checkpoint never gives credit for a shortcut or reverse lap.
        self.completed_laps as f32 * track.length()
            + distance.clamp(
                self.next_gate as f32 * section,
                (self.next_gate + 1) as f32 * section,
            )
    }
}

pub fn format_time(seconds: f32) -> String {
    let hundredths = (seconds.max(0.0) * 100.0) as u32;
    format!(
        "{:02}:{:02}.{:02}",
        hundredths / 6000,
        hundredths / 100 % 60,
        hundredths % 100
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scenic_route_initialization_cannot_change_compact_race_gates_or_scores() {
        let race = RaceProgress::new();
        let gate = race.gate();
        let position = track::TrackId::Compact
            .track()
            .at_distance(20.0)
            .lane_position(0.0);
        let score = race.score(position);
        let scenic = track::TrackId::Scenic.track();
        scenic.project(scenic.at_distance(850.0).point);
        assert_eq!(race.gate().point, gate.point);
        assert_eq!(race.score(position), score);
        let compact = track::TrackId::Compact.track();
        assert_eq!(
            gate.point,
            compact.at_distance(compact.length() / GATES as f32).point
        );
        let finished = RaceProgress {
            finish_time: Some(60.0),
            ..RaceProgress::default()
        };
        assert_eq!(finished.score(position), LAPS as f32 * compact.length());
    }

    fn cross(race: &mut RaceProgress, gate_index: usize, reverse: bool) {
        let gate = track::at_distance(track::length() * (gate_index + 1) as f32 / GATES as f32);
        let mut a = gate.point;
        let mut b = gate.point;
        a[0] -= gate.tangent[0];
        a[2] -= gate.tangent[1];
        b[0] += gate.tangent[0];
        b[2] += gate.tangent[1];
        if reverse {
            std::mem::swap(&mut a, &mut b);
        }
        race.update(a, b, Duration::from_secs(1));
    }

    #[test]
    fn rejects_out_of_order_and_reverse_crossings() {
        let mut race = RaceProgress::new();
        cross(&mut race, 3, false);
        cross(&mut race, 0, true);
        assert_eq!(race.next_gate, 0);
        cross(&mut race, 0, false);
        assert_eq!(race.next_gate, 1);
    }

    #[test]
    fn three_ordered_laps_finish_and_freeze_timer() {
        let mut race = RaceProgress::new();
        for _ in 0..LAPS {
            for gate in 0..GATES {
                cross(&mut race, gate, false);
            }
        }
        assert_eq!(race.completed_laps, LAPS);
        assert_eq!(race.lap_times.len(), LAPS as usize);
        let time = race.finish_time.unwrap();
        cross(&mut race, 0, false);
        assert_eq!(race.elapsed, time);
        assert!(race.best_lap().unwrap() > 0.0);
    }

    #[test]
    fn restart_discards_all_progress() {
        let mut race = RaceProgress::new();
        cross(&mut race, 0, false);
        race = RaceProgress::new();
        assert_eq!(race.next_gate, 0);
        assert_eq!(race.elapsed, 0.0);
        assert!(race.lap_times.is_empty());
    }
}

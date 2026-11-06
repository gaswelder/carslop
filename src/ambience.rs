//! Original procedural wind/surf. Sample-clock synthesis, no assets or dependencies.

use crate::track;
use std::f32::consts::TAU;

pub const SEA_LEVEL: f32 = -3.35;
pub const SAMPLE_RATE: f32 = 48_000.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Levels {
    pub wind: f32,
    pub surf: f32,
}

impl Levels {
    #[cfg(test)]
    pub fn at(position: [f32; 3], speed: f32) -> Self {
        Self::on_track(position, speed, &track::TrackId::Compact.track())
    }

    pub fn on_track(position: [f32; 3], speed: f32, track: &track::Track) -> Self {
        let distance_squared = track
            .shoreline()
            .iter()
            .map(|p| (0..3).map(|i| (p[i] - position[i]).powi(2)).sum::<f32>())
            .fold(f32::INFINITY, f32::min);
        let surf = 1.0 / (1.0 + distance_squared / (18.0 * 18.0));
        let exposure = ((position[1] - SEA_LEVEL) / 12.0).clamp(0.0, 1.0);
        let wind = (0.35 + exposure * 0.25 + (speed.max(0.0) / 30.0).clamp(0.0, 1.0) * 0.4)
            .clamp(0.0, 1.0);
        Self { wind, surf }
    }
}

#[cfg(test)]
fn shoreline() -> Vec<[f32; 3]> {
    track::TrackId::Compact.track().shoreline().to_vec()
}

pub struct CoastalAmbience {
    wind_seed: u32,
    surf_seed: u32,
    wind_low: f32,
    wind_mid: f32,
    surf_low: f32,
    surf_mid: f32,
    gust_phase: f32,
    wave_phase: f32,
    wind_gain: f32,
    surf_gain: f32,
}

impl Default for CoastalAmbience {
    fn default() -> Self {
        Self {
            wind_seed: 17,
            surf_seed: 101,
            wind_low: 0.0,
            wind_mid: 0.0,
            surf_low: 0.0,
            surf_mid: 0.0,
            gust_phase: 0.0,
            wave_phase: 0.0,
            wind_gain: 0.0,
            surf_gain: 0.0,
        }
    }
}

impl CoastalAmbience {
    pub fn sample(&mut self, levels: Levels, enabled: bool) -> f32 {
        let gate = if enabled { 1.0 } else { 0.0 };
        self.wind_gain += (levels.wind.clamp(0.0, 1.0) * 0.04 * gate - self.wind_gain) * 0.0005;
        self.surf_gain += (levels.surf.clamp(0.0, 1.0) * 0.05 * gate - self.surf_gain) * 0.0004;
        let wind = noise(&mut self.wind_seed);
        let surf = noise(&mut self.surf_seed);
        self.wind_low += (wind - self.wind_low) * 0.015;
        self.wind_mid += (wind - self.wind_mid) * 0.1;
        self.surf_low += (surf - self.surf_low) * 0.004;
        self.surf_mid += (surf - self.surf_mid) * 0.04;
        self.gust_phase = (self.gust_phase + 0.07 / SAMPLE_RATE).fract();
        self.wave_phase = (self.wave_phase + 0.115 / SAMPLE_RATE).fract();
        let gust = 0.75 + (self.gust_phase * TAU).sin() * 0.25;
        let wash = 0.25 + (0.5 + (self.wave_phase * TAU).sin() * 0.5).powi(2) * 0.75;
        let wind_band = self.wind_low * 1.5 + (self.wind_mid - self.wind_low) * 0.5;
        let surf_band = self.surf_low * 3.0 + self.surf_mid * 0.5;
        (wind_band * self.wind_gain * gust + surf_band * self.surf_gain * wash).clamp(-0.06, 0.06)
    }
}

fn noise(seed: &mut u32) -> f32 {
    *seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
    (*seed >> 8) as f32 / 8_388_607.5 - 1.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shoreline_levels_are_route_local_and_compact_wrapper_is_unchanged() {
        let compact = track::TrackId::Compact.track();
        let scenic = track::TrackId::scenic().track();
        assert_eq!(shoreline(), compact.shoreline());
        let shore = scenic.shoreline()[10];
        let near_scenic = Levels::on_track(shore, 0.0, &scenic);
        let far_compact = Levels::on_track(shore, 0.0, &compact);
        assert_eq!(near_scenic.surf, 1.0);
        assert!(far_compact.surf < 0.02);
        assert_eq!(Levels::at(shore, 0.0), far_compact);
        for i in 0..100 {
            let road = scenic.at_distance(i as f32 * scenic.length() / 100.0);
            let levels = Levels::on_track(road.lane_position(0.0), 30.0, &scenic);
            assert!((0.0..=1.0).contains(&levels.wind));
            assert!((0.0..=1.0).contains(&levels.surf));
        }
        let compact_shore = compact.shoreline()[10];
        assert_eq!(Levels::at(compact_shore, 0.0).surf, 1.0);
    }

    #[test]
    fn exposure_tracks_coast_distance_speed_and_elevation() {
        let shore = shoreline()[10];
        let coast = Levels::at(shore, 0.0);
        assert!((coast.surf - 1.0).abs() < 0.0001);
        let inland = Levels::at([0.0, 1.0, 0.0], 0.0);
        assert!(coast.surf > inland.surf);
        let high = Levels::at([shore[0], 30.0, shore[2]], 0.0);
        assert!(high.surf < coast.surf && high.wind > coast.wind);
        assert!(Levels::at(shore, 25.0).wind > coast.wind);
        for i in 0..300 {
            let road = track::TrackId::Compact
                .track()
                .at_distance(i as f32 * track::TrackId::Compact.track().length() / 300.0);
            let levels = Levels::at(road.lane_position(1.4), 40.0);
            assert!((0.0..=1.0).contains(&levels.wind) && (0.0..=1.0).contains(&levels.surf));
        }
    }

    #[test]
    fn layers_are_independent_subtle_and_repeatable() {
        let mut a = CoastalAmbience::default();
        let mut b = CoastalAmbience::default();
        let mut wind = CoastalAmbience::default();
        let mut surf = CoastalAmbience::default();
        let levels = Levels {
            wind: 1.0,
            surf: 1.0,
        };
        let mut energy = 0.0;
        let mut differences = 0.0;
        for _ in 0..48000 {
            let s = a.sample(levels, true);
            assert_eq!(s, b.sample(levels, true));
            assert!(s.is_finite() && s.abs() <= 0.06);
            energy += s * s;
            differences += (wind.sample(
                Levels {
                    wind: 1.0,
                    surf: 0.0,
                },
                true,
            ) - surf.sample(
                Levels {
                    wind: 0.0,
                    surf: 1.0,
                },
                true,
            ))
            .abs();
        }
        assert!(energy > 0.01 && energy / 48000.0 < 0.001);
        assert!(differences > 1.0);
    }

    #[test]
    fn all_layers_fade_when_disabled_even_with_nonzero_targets() {
        let mut a = CoastalAmbience::default();
        let levels = Levels {
            wind: 1.0,
            surf: 1.0,
        };
        for _ in 0..48000 {
            a.sample(levels, true);
        }
        let mut tail: f32 = 0.0;
        for i in 0..24000 {
            let sample = a.sample(levels, false);
            if i > 20000 {
                tail = tail.max(sample.abs());
            }
        }
        assert!(tail < 0.00001);
    }
}

//! Device-independent engine, road, impact, and tire synthesis.

use crate::model::world::Material;
use crate::sound;
use crate::sound::SAMPLE_RATE;
use std::f32::consts::TAU;
use std::path::Path;

#[derive(Clone, Copy)]
pub struct Voice<'a> {
    pub frequency: f32,
    pub load: f32,
    pub scrub: f32,
    pub surface: Material,
    pub sample_directory: Option<&'a Path>,
    pub enabled: bool,
    pub turbo_target: f32,
    pub road_speed: f32,
}

#[derive(Default)]
pub struct Synth {
    recorded: sound::engine_sample::RecordedEngine,
    turbo_spool: f32,
    turbo_air_low: f32,
    turbo_air_high: f32,
    road_gain: f32,
    road_noise: f32,
    phase: f32,
    frequency: f32,
    load: f32,
    tires: sound::tires::TireSound,
    master: f32,
    noise: u32,
    filtered_noise: f32,
    impact: f32,
    impact_phase: f32,
    impact_cooldown: f32,
}

pub fn hit(synth: &mut Synth, speed: f32) {
    if speed < 1.5 || synth.impact_cooldown > 0.0 {
        return;
    }
    synth.impact = ((speed - 1.5) / 9.0).clamp(0.0, 1.0);
    synth.impact_phase = 0.0;
    synth.impact_cooldown = 0.09;
}

pub fn suppress_impacts(synth: &mut Synth) {
    synth.impact = 0.0;
    synth.impact_cooldown = 0.0;
}

pub fn sample(synth: &mut Synth, voice: Voice<'_>) -> f32 {
    synth.frequency += (voice.frequency - synth.frequency) * 0.002;
    let load = if voice.enabled {
        voice.load.clamp(0.0, 1.0)
    } else {
        0.0
    };
    let tire = if voice.enabled {
        voice.scrub.clamp(0.0, 1.0)
    } else {
        0.0
    };
    synth.load += (load - synth.load) * 0.001;
    synth.master += (f32::from(u8::from(voice.enabled)) - synth.master) * 0.002;
    synth.phase = (synth.phase + synth.frequency / SAMPLE_RATE).rem_euclid(2.0);
    let p = synth.phase * TAU;
    // Shared placeholder timbre when no recorded engine samples are available.
    let harmonics = p.sin() * 0.6 + (p * 2.0).sin() * 0.25 + (p * 3.0).sin() * 0.15;
    // Deterministic filtered noise: independent tire scrub, intake hiss and impact grit.
    synth.noise = synth
        .noise
        .wrapping_mul(1_664_525)
        .wrapping_add(1_013_904_223);
    let noise = (synth.noise >> 8) as f32 / 8_388_607.5 - 1.0;
    synth.filtered_noise += (noise - synth.filtered_noise) * 0.08;
    let procedural = (p.sin() * (1.0 - synth.load) + harmonics * synth.load)
        * (0.065 + synth.load * 0.065)
        + synth.filtered_noise * synth.load * 0.025;
    sound::engine_sample::select(&mut synth.recorded, voice.sample_directory);
    let engine = if sound::engine_sample::available(&synth.recorded) {
        sound::engine_sample::sample(&mut synth.recorded, synth.frequency * 30.0, synth.load)
    } else {
        procedural
    };
    let turbo_target = if voice.enabled {
        voice.turbo_target
    } else {
        0.0
    };
    synth.turbo_spool += (turbo_target - synth.turbo_spool) * 0.0007;
    // Two low-pass filters keep compressor airflow in a higher, airy band.
    // It brightens with turbine speed but stays behind the engine in the mix.
    let low_cutoff = 900.0 + 1500.0 * synth.turbo_spool;
    let high_cutoff = 3500.0 + 3500.0 * synth.turbo_spool;
    synth.turbo_air_low +=
        (noise - synth.turbo_air_low) * (1.0 - (-TAU * low_cutoff / SAMPLE_RATE).exp());
    synth.turbo_air_high +=
        (noise - synth.turbo_air_high) * (1.0 - (-TAU * high_cutoff / SAMPLE_RATE).exp());
    let whoosh = (synth.turbo_air_high - synth.turbo_air_low) * 0.10 * synth.turbo_spool.powf(1.4);
    let road_target = (voice.road_speed.abs() / 45.0).clamp(0.0, 1.0).powf(1.3);
    synth.road_gain += (road_target - synth.road_gain) * 0.0005;
    synth.road_noise += (noise - synth.road_noise) * 0.025;
    let rolling = (synth.road_noise * 0.18 + synth.filtered_noise * 0.035) * synth.road_gain;
    let tires = sound::tires::sample(&mut synth.tires, tire, voice.road_speed, voice.surface);
    synth.impact_phase = (synth.impact_phase + (48.0 + synth.impact * 42.0) / SAMPLE_RATE).fract();
    let impact = ((synth.impact_phase * TAU).sin() * 0.7 + noise * 0.3) * synth.impact * 0.22;
    synth.impact *= 0.99981;
    synth.impact_cooldown = (synth.impact_cooldown - 1.0 / SAMPLE_RATE).max(0.0);
    ((engine + whoosh + tires + rolling + impact) * synth.master).clamp(-0.3, 0.3)
}

//! Surface-dependent tire scrub, rustle and squeal generation.

use crate::model::world::Material;
use crate::sound::SAMPLE_RATE;
use std::f32::consts::TAU;

/// Rubber scrub grows into a wavering squeal as slip builds. Independent noise,
/// two resonances and slightly detuned tones avoid a fixed whistle or white hiss.
#[derive(Default)]
pub struct TireSound {
    gain: f32,
    slip: f32,
    speed: f32,
    noise: u32,
    wander: f32,
    flutter: f32,
    phase: [f32; 2],
    low: [f32; 2],
    band: [f32; 2],
    scrub_low: f32,
    scrub_high: f32,
    rubber_mix: f32,
    texture_gain: f32,
    texture_cutoff: f32,
    texture_low: f32,
}

struct Texture {
    rubber: f32,
    gain: f32,
    cutoff: f32,
}

fn texture(material: Material) -> Texture {
    match material {
        Material::Grass | Material::Park | Material::Scrub | Material::Sports => Texture {
            rubber: 0.0,
            gain: 0.15,
            cutoff: 0.055,
        },
        Material::BareGround => Texture {
            rubber: 0.0,
            gain: 0.18,
            cutoff: 0.32,
        },
        Material::Paving => Texture {
            rubber: 0.45,
            gain: 0.10,
            cutoff: 0.22,
        },
        Material::Concrete => Texture {
            rubber: 0.85,
            gain: 0.045,
            cutoff: 0.24,
        },
        Material::Water => Texture {
            rubber: 0.0,
            gain: 0.12,
            cutoff: 0.12,
        },
        Material::Asphalt | Material::Paint => Texture {
            rubber: 1.0,
            gain: 0.0,
            cutoff: 0.18,
        },
    }
}

pub fn sample(tire_sound: &mut TireSound, slip: f32, speed: f32, material: Material) -> f32 {
    let texture = texture(material);
    // Crossfade in about 30 ms so surface boundaries never switch abruptly.
    tire_sound.rubber_mix += (texture.rubber - tire_sound.rubber_mix) * 0.0007;
    tire_sound.texture_gain += (texture.gain - tire_sound.texture_gain) * 0.0007;
    tire_sound.texture_cutoff += (texture.cutoff - tire_sound.texture_cutoff) * 0.0007;
    // Quiet grip corrections should not sound like a sustained slide.
    let slip = ((slip - 0.10) / 0.90).clamp(0.0, 1.0);
    let speed_gain = ((speed.abs() - 1.5) / 7.0).clamp(0.0, 1.0);
    let target = slip * speed_gain;
    // About 20 ms attack / 65 ms release; no clicks at physics updates.
    let envelope = if target > tire_sound.gain {
        0.001
    } else {
        0.00032
    };
    tire_sound.gain += (target - tire_sound.gain) * envelope;
    tire_sound.slip += (slip - tire_sound.slip) * 0.0005;
    tire_sound.speed += (speed.abs().min(60.0) - tire_sound.speed) * 0.0002;
    tire_sound.noise = tire_sound
        .noise
        .wrapping_mul(1_664_525)
        .wrapping_add(1_013_904_223);
    let noise = (tire_sound.noise >> 8) as f32 / 8_388_607.5 - 1.0;
    tire_sound.wander += (noise - tire_sound.wander) * 0.0008;
    tire_sound.flutter =
        (tire_sound.flutter + (11.0 + tire_sound.speed * 0.18) / SAMPLE_RATE).fract();
    let flutter = (tire_sound.flutter * TAU).sin();
    let pitch = 850.0 + tire_sound.slip * 460.0 + tire_sound.speed * 3.0;
    let pitch = pitch * (1.0 + tire_sound.wander * 1.8 + flutter * 0.012);
    let mut resonance = 0.0;
    let mut tone = 0.0;
    for i in 0..2 {
        let frequency = pitch * if i == 0 { 1.0 } else { 1.47 };
        // Damped state-variable bandpass: energy stays around rubber's
        // resonances rather than spraying high-frequency noise into the mix.
        let coefficient = 2.0 * (std::f32::consts::PI * frequency / SAMPLE_RATE).sin();
        tire_sound.low[i] += coefficient * tire_sound.band[i];
        let high = noise - tire_sound.low[i] - 0.24 * tire_sound.band[i];
        tire_sound.band[i] += coefficient * high;
        resonance += tire_sound.band[i] * if i == 0 { 0.032 } else { 0.018 };
        tire_sound.phase[i] = (tire_sound.phase[i] + frequency / SAMPLE_RATE).fract();
        tone += (tire_sound.phase[i] * TAU).sin() * if i == 0 { 0.024 } else { 0.010 };
    }
    tire_sound.scrub_low += (noise - tire_sound.scrub_low) * 0.025;
    tire_sound.scrub_high += (noise - tire_sound.scrub_high) * 0.18;
    let scrub = (tire_sound.scrub_high - tire_sound.scrub_low) * 0.055;
    let squeal = tire_sound.slip * tire_sound.slip;
    let rubber = scrub + (resonance + tone) * squeal;
    tire_sound.texture_low += (noise - tire_sound.texture_low) * tire_sound.texture_cutoff;
    // Loose surfaces produce irregular friction noise without rubber resonances.
    let texture = (tire_sound.texture_low - tire_sound.scrub_low * 0.5)
        * tire_sound.texture_gain
        * (0.8 + flutter * 0.2);
    (rubber * tire_sound.rubber_mix + texture) * tire_sound.gain * (0.92 + flutter * 0.08)
}

#[cfg(test)]
#[path = "tests/tires.rs"]
mod tests;

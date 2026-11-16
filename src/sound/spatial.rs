//! Sample-rate spatialization independent of the playback device.

use crate::sound::SAMPLE_RATE;
use crate::util::geometry::{XYZ, XZ};
use crate::{render, util};
use std::f32::consts::TAU;

#[derive(Clone, Copy)]
pub struct Placement {
    gain: f32,
    pan: f32,
    filter: f32,
}

// The camera is the listener. Smooth at sample rate so camera switches and
// recovery do not create volume or stereo discontinuities.
pub struct SpatialAudio {
    gain: f32,
    pan: f32,
    filtered: f32,
    reflections: Vec<f32>,
    cursor: usize,
    reflection_low: f32,
}

impl Default for SpatialAudio {
    fn default() -> Self {
        Self {
            gain: 0.0,
            pan: 0.0,
            filtered: 0.0,
            reflections: vec![0.0; (SAMPLE_RATE * 0.15) as usize],
            cursor: 0,
            reflection_low: 0.0,
        }
    }
}

pub fn placement(source: XYZ, listener: render::camera::Pose) -> Placement {
    let offset: XYZ = util::geometry::xyz_from_fn(|i| {
        util::geometry::xyz_component(source, i) - util::geometry::xyz_component(listener.eye, i)
    });
    let distance = util::geometry::iter(&offset)
        .map(|v| v * v)
        .sum::<f32>()
        .sqrt();
    // Gentle falloff keeps driving cues present; distance also changes
    // the direct/reflected balance below, rather than just turning down.
    let gain = (6.0 / (distance + 3.0)).min(1.0);
    let forward = XZ {
        x: listener.target.x - listener.eye.x,
        z: listener.target.z - listener.eye.z,
    };
    let length = forward.x.hypot(forward.z);
    let pan = if length > 0.001 && distance > 0.001 {
        ((offset.x * forward.z - offset.z * forward.x) / (length * distance)).clamp(-1.0, 1.0)
    } else {
        0.0
    };
    let cutoff = 6500.0 / (1.0 + distance / 8.0);
    let filter = 1.0 - (-TAU * cutoff.max(350.0) / SAMPLE_RATE).exp();
    Placement { gain, pan, filter }
}

pub fn sample(spatial_audio: &mut SpatialAudio, input: f32, placement: Placement) -> [f32; 2] {
    let Placement { gain, pan, filter } = placement;
    let blend = 1.0 - (-1.0 / (SAMPLE_RATE * 0.03)).exp();
    spatial_audio.gain += (gain - spatial_audio.gain) * blend;
    spatial_audio.pan += (pan - spatial_audio.pan) * blend;
    spatial_audio.filtered += (input - spatial_audio.filtered) * filter;
    let sample = spatial_audio.filtered * spatial_audio.gain;
    // A ground bounce arrives first, followed by sparse, asymmetric outdoor
    // reflections. Different arrival times in each ear externalize even a
    // centred source. A damped feedback path adds a short diffuse decay,
    // rather than a long indoor/hall reverb.
    let tap = |ms: f32| {
        let delay = (SAMPLE_RATE * ms / 1000.0) as usize;
        spatial_audio.reflections[(spatial_audio.cursor + spatial_audio.reflections.len() - delay)
            % spatial_audio.reflections.len()]
    };
    let ground = tap(4.3) * 0.28;
    let left = ground + tap(19.7) * 0.26 + tap(43.1) * 0.17 + tap(79.3) * 0.10;
    let right = ground + tap(25.3) * 0.26 + tap(53.7) * 0.17 + tap(89.9) * 0.10;
    let feedback = tap(113.0);
    spatial_audio.reflection_low += (feedback - spatial_audio.reflection_low) * 0.12;
    spatial_audio.reflections[spatial_audio.cursor] = sample + spatial_audio.reflection_low * 0.28;
    spatial_audio.cursor = (spatial_audio.cursor + 1) % spatial_audio.reflections.len();
    let wet = 0.35 + (1.0 - spatial_audio.gain) * 0.65;
    let direct = 0.85 - (1.0 - spatial_audio.gain) * 0.20;
    [
        sample * direct * (1.0 - spatial_audio.pan).sqrt() + left * wet,
        sample * direct * (1.0 + spatial_audio.pan).sqrt() + right * wet,
    ]
}

use crate::model_car;
use crate::physics_car;
use crate::render_camera;
use crate::sound_engine_sample;
use crate::util_geometry;
use crate::util_geometry::{XYZ, XZ};
use sdl3::audio::{AudioFormat, AudioSpec, AudioStreamOwner};
use std::f32::consts::TAU;

pub const SAMPLE_RATE: f32 = 48_000.0;
const QUEUE_SAMPLES: usize = 2400;

pub struct EngineAudio {
    stream: AudioStreamOwner,
    _subsystem: sdl3::AudioSubsystem,
    synth: Synth,
    spatial: SpatialAudio,
    pub muted: bool,
}

#[derive(Clone, Copy)]
struct Voice<'a> {
    frequency: f32,
    load: f32,
    scrub: f32,
    tuning: &'a model_car::EngineVoice,
    enabled: bool,
}

#[derive(Default)]
struct Synth {
    recorded: sound_engine_sample::RecordedEngine,
    turbo_target: f32,
    turbo_spool: f32,
    turbo_air_low: f32,
    turbo_air_high: f32,
    road_speed: f32,
    road_gain: f32,
    road_noise: f32,
    phase: f32,
    frequency: f32,
    load: f32,
    tires: TireSound,
    master: f32,
    noise: u32,
    filtered_noise: f32,
    impact: f32,
    impact_phase: f32,
    impact_cooldown: f32,
}

fn hit(synth: &mut Synth, speed: f32) {
    if speed < 1.5 || synth.impact_cooldown > 0.0 {
        return;
    }
    synth.impact = ((speed - 1.5) / 9.0).clamp(0.0, 1.0);
    synth.impact_phase = 0.0;
    synth.impact_cooldown = 0.09;
}

fn suppress_impacts(synth: &mut Synth) {
    synth.impact = 0.0;
    synth.impact_cooldown = 0.0;
}

fn synth_sample(synth: &mut Synth, voice: Voice<'_>) -> f32 {
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
    let tuning = voice.tuning;
    let harmonics: f32 = tuning
        .harmonics
        .iter()
        .map(|[multiple, amplitude]| (p * multiple).sin() * amplitude)
        .sum();
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
    sound_engine_sample::select(&mut synth.recorded, tuning.sample_directory.as_deref());
    let engine = if sound_engine_sample::available(&synth.recorded) {
        sound_engine_sample::sample(
            &mut synth.recorded,
            synth.frequency * 30.0,
            tuning.pulses_per_revolution,
            synth.load,
        )
    } else {
        procedural
    };
    let engine =
        engine * tuning.gain * (tuning.coast_gain + synth.load * (1.0 - tuning.coast_gain));
    let engine = if let Some([knee, headroom]) = tuning.soft_limit {
        let magnitude = engine.abs();
        engine.signum()
            * (magnitude.min(knee) + headroom * ((magnitude - knee).max(0.0) / headroom).tanh())
    } else {
        engine
    };
    let turbo_target = if voice.enabled {
        synth.turbo_target
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
    let road_target = (synth.road_speed.abs() / 45.0).clamp(0.0, 1.0).powf(1.3);
    synth.road_gain += (road_target - synth.road_gain) * 0.0005;
    synth.road_noise += (noise - synth.road_noise) * 0.025;
    let rolling = (synth.road_noise * 0.18 + synth.filtered_noise * 0.035) * synth.road_gain;
    let tires = tire_sound_sample(&mut synth.tires, tire, synth.road_speed);
    synth.impact_phase = (synth.impact_phase + (48.0 + synth.impact * 42.0) / SAMPLE_RATE).fract();
    let impact = ((synth.impact_phase * TAU).sin() * 0.7 + noise * 0.3) * synth.impact * 0.22;
    synth.impact *= 0.99981;
    synth.impact_cooldown = (synth.impact_cooldown - 1.0 / SAMPLE_RATE).max(0.0);
    ((engine + whoosh + tires + rolling + impact) * synth.master).clamp(-0.3, 0.3)
}

/// Rubber scrub grows into a wavering squeal as slip builds. Independent noise,
/// two resonances and slightly detuned tones avoid a fixed whistle or white hiss.
#[derive(Default)]
struct TireSound {
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
}

fn tire_sound_sample(tire_sound: &mut TireSound, slip: f32, speed: f32) -> f32 {
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
    (scrub + (resonance + tone) * squeal) * tire_sound.gain * (0.92 + flutter * 0.08)
}

// The camera is the listener. Smooth at sample rate so camera switches and
// recovery do not create volume or stereo discontinuities.
struct SpatialAudio {
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

fn placement(source: XYZ, listener: render_camera::Pose) -> (f32, f32, f32) {
    let offset: XYZ = util_geometry::xyz_from_fn(|i| {
        util_geometry::xyz_component(source, i) - util_geometry::xyz_component(listener.eye, i)
    });
    let distance = util_geometry::iter(&offset)
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
    (gain, pan, filter)
}

fn spatial_audio_sample(
    spatial_audio: &mut SpatialAudio,
    input: f32,
    placement: (f32, f32, f32),
) -> [f32; 2] {
    let (gain, pan, filter) = placement;
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

// Preserve surf/gust phase across recovery/restart; only clear vehicle transients.
pub fn reset_feedback(engine_audio: &mut EngineAudio) {
    engine_audio.synth = Synth::default();
    engine_audio.spatial = SpatialAudio::default();
}

pub fn new(sdl: &sdl3::Sdl) -> Result<EngineAudio, String> {
    let audio = sdl.audio().map_err(|e| e.to_string())?;
    let spec = AudioSpec {
        freq: Some(SAMPLE_RATE as i32),
        channels: Some(2),
        format: Some(AudioFormat::f32_sys()),
    };
    let device = audio
        .open_playback_device(&spec)
        .map_err(|e| e.to_string())?;
    let stream = device
        .open_device_stream(Some(&spec))
        .map_err(|e| e.to_string())?;
    stream.resume().map_err(|e| e.to_string())?;
    Ok(EngineAudio {
        stream,
        _subsystem: audio,
        synth: Synth::default(),
        spatial: SpatialAudio::default(),
        muted: false,
    })
}

pub fn update(
    engine_audio: &mut EngineAudio,
    car: &physics_car::Vehicle,
    listener: render_camera::Pose,
    throttle: bool,
    active: bool,
    impact: f32,
) -> Result<(), String> {
    let queued = engine_audio
        .stream
        .queued_bytes()
        .map_err(|e| e.to_string())?
        .max(0) as usize
        / (2 * std::mem::size_of::<f32>());
    let count = QUEUE_SAMPLES.saturating_sub(queued);
    let enabled = active && !engine_audio.muted;
    engine_audio.synth.turbo_target = car.turbo_spool;
    engine_audio.synth.road_speed = car.velocity.x.hypot(car.velocity.z);
    if enabled {
        hit(&mut engine_audio.synth, impact);
    } else {
        suppress_impacts(&mut engine_audio.synth);
    }
    let voice = Voice {
        frequency: engine_frequency(car.engine_rpm),
        load: if throttle {
            car.engine_load.max(0.15)
        } else {
            car.engine_load
        },
        scrub: car.tire_scrub,
        tuning: &car.spec.audio,
        enabled,
    };
    let placement = placement(car.position, listener);
    let samples: Vec<f32> = (0..count)
        .flat_map(|_| {
            let driving = spatial_audio_sample(
                &mut engine_audio.spatial,
                synth_sample(&mut engine_audio.synth, voice),
                placement,
            );
            driving.map(output_sample)
        })
        .collect();
    if !samples.is_empty() {
        engine_audio
            .stream
            .put_data_f32(&samples)
            .map_err(|e| e.to_string())?;
    }
    Ok(())
}

fn output_sample(driving: f32) -> f32 {
    // Preserve driving gain and output headroom.
    (driving * 0.92).clamp(-0.3, 0.3)
}

fn engine_frequency(rpm: f32) -> f32 {
    rpm.max(physics_car::IDLE_RPM) / 30.0
}

#[cfg(test)]
mod tests {
    use crate::sound_car;

    use super::*;
    use crate::render_camera;
    use crate::test_support;

    #[test]
    fn engine_voice_stays_present_with_headroom_for_the_driving_mix() {
        for load in [0.12, 1.0] {
            let mut synth = Synth::default();
            let mut energy = 0.0;
            let mut clipped = 0;
            for i in 0..48000 {
                let sample = sound_car::synth_sample(
                    &mut synth,
                    Voice {
                        frequency: 150.0,
                        load,
                        scrub: 0.0,
                        tuning: test_support::audio_voice(),
                        enabled: true,
                    },
                );
                if i > 12000 {
                    energy += sample * sample;
                    if sample.abs() >= 0.299 {
                        clipped += 1;
                    }
                }
            }
            let rms = (energy / 35999.0_f32).sqrt();
            assert!(
                rms > if load < 0.5 { 0.012 } else { 0.035 },
                "fixture voice too quiet at load {load}: {rms}"
            );
            assert_eq!(clipped, 0, "engine alone must leave headroom");
        }
    }

    #[test]
    fn outdoor_reflections_spread_a_centred_source_and_decay() {
        let mut spatial = SpatialAudio {
            gain: 0.7,
            ..SpatialAudio::default()
        };
        let placement = (0.7, 0.0, 0.3);
        let mut early = 0.0;
        let mut reflected = 0.0;
        let mut width = 0.0;
        let mut tail = 0.0_f32;
        for i in 0..48000 {
            let output = sound_car::spatial_audio_sample(
                &mut spatial,
                if i == 0 { 0.2 } else { 0.0 },
                placement,
            );
            assert!(output.iter().all(|v| v.is_finite() && v.abs() < 0.3));
            let energy = output[0] * output[0] + output[1] * output[1];
            if i < 100 {
                early += energy;
            }
            if i > 150 {
                reflected += energy;
            }
            width += (output[0] - output[1]).abs();
            if i > 40000 {
                tail = tail.max(output[0].abs()).max(output[1].abs());
            }
        }
        assert!(early > 0.0 && reflected > early * 0.01);
        assert!(
            width > 0.01,
            "reflections must have different ear arrival times"
        );
        assert!(tail < 0.00001, "outdoor decay must stay short");
    }

    #[test]
    fn stereo_follows_listener_orientation_and_changes_smoothly() {
        let listener = render_camera::Pose {
            eye: XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            target: XYZ {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        };
        let right = sound_car::placement(
            XYZ {
                x: 5.0,
                y: 0.0,
                z: 5.0,
            },
            listener,
        );
        let left = sound_car::placement(
            XYZ {
                x: -5.0,
                y: 0.0,
                z: 5.0,
            },
            listener,
        );
        assert!(right.1 > 0.0 && left.1 < 0.0);
        let reversed = sound_car::placement(
            XYZ {
                x: 5.0,
                y: 0.0,
                z: 5.0,
            },
            render_camera::Pose {
                target: XYZ {
                    x: 0.0,
                    y: 0.0,
                    z: -1.0,
                },
                ..listener
            },
        );
        assert!(reversed.1 < 0.0);
        let mut spatial = SpatialAudio::default();
        for _ in 0..24000 {
            let output = sound_car::spatial_audio_sample(&mut spatial, 0.1, right);
            assert!(output.iter().all(|v| v.is_finite() && v.abs() < 0.3));
        }
        let before = sound_car::spatial_audio_sample(&mut spatial, 0.1, right);
        assert!(before[1] > before[0]);
        let after = sound_car::spatial_audio_sample(&mut spatial, 0.1, left);
        assert!((after[0] - before[0]).abs() < 0.001);
        assert!((after[1] - before[1]).abs() < 0.001);
    }

    #[test]
    fn impacts_scale_decay_and_do_not_retrigger_on_tiny_contacts() {
        let mut synth = Synth::default();
        sound_car::hit(&mut synth, 1.0);
        assert_eq!(synth.impact, 0.0);
        sound_car::hit(&mut synth, 4.0);
        let initial = synth.impact;
        sound_car::hit(&mut synth, 12.0);
        assert_eq!(synth.impact, initial); // cooldown suppresses contact chatter
        let voice = Voice {
            frequency: 30.0,
            load: 0.0,
            scrub: 0.0,
            tuning: test_support::audio_voice(),
            enabled: true,
        };
        for _ in 0..48000 {
            sound_car::synth_sample(&mut synth, voice);
        }
        assert!(synth.impact < 0.0001);
        sound_car::hit(&mut synth, 12.0);
        assert!(synth.impact > initial);
        sound_car::suppress_impacts(&mut synth);
        assert_eq!(synth.impact, 0.0);
    }

    #[test]
    fn driving_output_is_bounded_and_fades_on_mute_or_pause() {
        let mut engine = Synth::default();
        let mut voice = Voice {
            frequency: 180.0,
            load: 1.0,
            scrub: 1.0,
            tuning: test_support::audio_voice(),
            enabled: true,
        };
        let mut driving_energy = 0.0;
        for i in 0..48000 {
            if i % 6000 == 0 {
                sound_car::hit(&mut engine, 20.0);
            }
            let driving = sound_car::synth_sample(&mut engine, voice);
            let sample = output_sample(driving);
            assert!(sample.is_finite() && sample.abs() <= 0.3);
            driving_energy += driving * driving;
        }
        assert!(driving_energy > 0.01);
        voice.enabled = false;
        sound_car::suppress_impacts(&mut engine);
        let mut tail: f32 = 0.0;
        for i in 0..24000 {
            let sample = output_sample(sound_car::synth_sample(&mut engine, voice));
            if i > 20000 {
                tail = tail.max(sample.abs());
            }
        }
        assert!(tail < 0.00001);
    }

    #[test]
    fn tire_squeal_is_band_limited_and_releases_smoothly() {
        let mut tires = TireSound::default();
        let mut previous = 0.0;
        let mut energy = 0.0;
        let mut difference_energy = 0.0;
        for _ in 0..48000 {
            let sample = sound_car::tire_sound_sample(&mut tires, 1.0, 30.0);
            assert!(sample.is_finite() && sample.abs() < 0.2);
            energy += sample * sample;
            difference_energy += (sample - previous).powi(2);
            previous = sample;
        }
        // Broadband hiss has roughly twice as much first-difference energy.
        assert!(energy > 1.0 && difference_energy < energy * 0.4);
        let before = tires.gain;
        sound_car::tire_sound_sample(&mut tires, 0.0, 30.0);
        assert!(tires.gain < before && tires.gain > before * 0.99);
        let mut tail = 0.0_f32;
        for i in 0..48000 {
            let sample = sound_car::tire_sound_sample(&mut tires, 0.0, 30.0);
            if i > 40000 {
                tail = tail.max(sample.abs());
            }
        }
        assert!(tail < 0.00001);
    }
}

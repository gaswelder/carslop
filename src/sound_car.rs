use crate::asset_car;
use crate::geometry::{XYZ, XZ};
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
    tuning: &'a asset_car::EngineVoice,
    enabled: bool,
}

#[derive(Default)]
struct Synth {
    recorded: crate::engine_sample::RecordedEngine,
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

impl Synth {
    fn hit(&mut self, speed: f32) {
        if speed < 1.5 || self.impact_cooldown > 0.0 {
            return;
        }
        self.impact = ((speed - 1.5) / 9.0).clamp(0.0, 1.0);
        self.impact_phase = 0.0;
        self.impact_cooldown = 0.09;
    }

    fn suppress_impacts(&mut self) {
        self.impact = 0.0;
        self.impact_cooldown = 0.0;
    }

    fn sample(&mut self, voice: Voice<'_>) -> f32 {
        self.frequency += (voice.frequency - self.frequency) * 0.002;
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
        self.load += (load - self.load) * 0.001;
        self.master += (f32::from(u8::from(voice.enabled)) - self.master) * 0.002;
        self.phase = (self.phase + self.frequency / SAMPLE_RATE).rem_euclid(2.0);
        let p = self.phase * TAU;
        let tuning = voice.tuning;
        let harmonics: f32 = tuning
            .harmonics
            .iter()
            .map(|[multiple, amplitude]| (p * multiple).sin() * amplitude)
            .sum();
        // Deterministic filtered noise: independent tire scrub, intake hiss and impact grit.
        self.noise = self
            .noise
            .wrapping_mul(1_664_525)
            .wrapping_add(1_013_904_223);
        let noise = (self.noise >> 8) as f32 / 8_388_607.5 - 1.0;
        self.filtered_noise += (noise - self.filtered_noise) * 0.08;
        let procedural = (p.sin() * (1.0 - self.load) + harmonics * self.load)
            * (0.065 + self.load * 0.065)
            + self.filtered_noise * self.load * 0.025;
        self.recorded.select(tuning.samples.as_deref());
        let engine = if self.recorded.available() {
            self.recorded.sample(
                self.frequency * 30.0,
                tuning.pulses_per_revolution,
                self.load,
            )
        } else {
            procedural
        };
        let engine =
            engine * tuning.gain * (tuning.coast_gain + self.load * (1.0 - tuning.coast_gain));
        let engine = if let Some([knee, headroom]) = tuning.soft_limit {
            let magnitude = engine.abs();
            engine.signum()
                * (magnitude.min(knee) + headroom * ((magnitude - knee).max(0.0) / headroom).tanh())
        } else {
            engine
        };
        let turbo_target = if voice.enabled {
            self.turbo_target
        } else {
            0.0
        };
        self.turbo_spool += (turbo_target - self.turbo_spool) * 0.0007;
        // Two low-pass filters keep compressor airflow in a higher, airy band.
        // It brightens with turbine speed but stays behind the engine in the mix.
        let low_cutoff = 900.0 + 1500.0 * self.turbo_spool;
        let high_cutoff = 3500.0 + 3500.0 * self.turbo_spool;
        self.turbo_air_low +=
            (noise - self.turbo_air_low) * (1.0 - (-TAU * low_cutoff / SAMPLE_RATE).exp());
        self.turbo_air_high +=
            (noise - self.turbo_air_high) * (1.0 - (-TAU * high_cutoff / SAMPLE_RATE).exp());
        let whoosh = (self.turbo_air_high - self.turbo_air_low) * 0.10 * self.turbo_spool.powf(1.4);
        let road_target = (self.road_speed.abs() / 45.0).clamp(0.0, 1.0).powf(1.3);
        self.road_gain += (road_target - self.road_gain) * 0.0005;
        self.road_noise += (noise - self.road_noise) * 0.025;
        let rolling = (self.road_noise * 0.18 + self.filtered_noise * 0.035) * self.road_gain;
        let tires = self.tires.sample(tire, self.road_speed);
        self.impact_phase = (self.impact_phase + (48.0 + self.impact * 42.0) / SAMPLE_RATE).fract();
        let impact = ((self.impact_phase * TAU).sin() * 0.7 + noise * 0.3) * self.impact * 0.22;
        self.impact *= 0.99981;
        self.impact_cooldown = (self.impact_cooldown - 1.0 / SAMPLE_RATE).max(0.0);
        ((engine + whoosh + tires + rolling + impact) * self.master).clamp(-0.3, 0.3)
    }
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

impl TireSound {
    fn sample(&mut self, slip: f32, speed: f32) -> f32 {
        // Quiet grip corrections should not sound like a sustained slide.
        let slip = ((slip - 0.10) / 0.90).clamp(0.0, 1.0);
        let speed_gain = ((speed.abs() - 1.5) / 7.0).clamp(0.0, 1.0);
        let target = slip * speed_gain;
        // About 20 ms attack / 65 ms release; no clicks at physics updates.
        let envelope = if target > self.gain { 0.001 } else { 0.00032 };
        self.gain += (target - self.gain) * envelope;
        self.slip += (slip - self.slip) * 0.0005;
        self.speed += (speed.abs().min(60.0) - self.speed) * 0.0002;
        self.noise = self
            .noise
            .wrapping_mul(1_664_525)
            .wrapping_add(1_013_904_223);
        let noise = (self.noise >> 8) as f32 / 8_388_607.5 - 1.0;
        self.wander += (noise - self.wander) * 0.0008;
        self.flutter = (self.flutter + (11.0 + self.speed * 0.18) / SAMPLE_RATE).fract();
        let flutter = (self.flutter * TAU).sin();
        let pitch = 850.0 + self.slip * 460.0 + self.speed * 3.0;
        let pitch = pitch * (1.0 + self.wander * 1.8 + flutter * 0.012);
        let mut resonance = 0.0;
        let mut tone = 0.0;
        for i in 0..2 {
            let frequency = pitch * if i == 0 { 1.0 } else { 1.47 };
            // Damped state-variable bandpass: energy stays around rubber's
            // resonances rather than spraying high-frequency noise into the mix.
            let coefficient = 2.0 * (std::f32::consts::PI * frequency / SAMPLE_RATE).sin();
            self.low[i] += coefficient * self.band[i];
            let high = noise - self.low[i] - 0.24 * self.band[i];
            self.band[i] += coefficient * high;
            resonance += self.band[i] * if i == 0 { 0.032 } else { 0.018 };
            self.phase[i] = (self.phase[i] + frequency / SAMPLE_RATE).fract();
            tone += (self.phase[i] * TAU).sin() * if i == 0 { 0.024 } else { 0.010 };
        }
        self.scrub_low += (noise - self.scrub_low) * 0.025;
        self.scrub_high += (noise - self.scrub_high) * 0.18;
        let scrub = (self.scrub_high - self.scrub_low) * 0.055;
        let squeal = self.slip * self.slip;
        (scrub + (resonance + tone) * squeal) * self.gain * (0.92 + flutter * 0.08)
    }
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

impl SpatialAudio {
    fn placement(source: XYZ, listener: crate::camera::Pose) -> (f32, f32, f32) {
        let offset: XYZ = XYZ::from_fn(|i| source.component(i) - listener.eye.component(i));
        let distance = offset.iter().map(|v| v * v).sum::<f32>().sqrt();
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

    fn sample(&mut self, input: f32, placement: (f32, f32, f32)) -> [f32; 2] {
        let (gain, pan, filter) = placement;
        let blend = 1.0 - (-1.0 / (SAMPLE_RATE * 0.03)).exp();
        self.gain += (gain - self.gain) * blend;
        self.pan += (pan - self.pan) * blend;
        self.filtered += (input - self.filtered) * filter;
        let sample = self.filtered * self.gain;
        // A ground bounce arrives first, followed by sparse, asymmetric outdoor
        // reflections. Different arrival times in each ear externalize even a
        // centred source. A damped feedback path adds a short diffuse decay,
        // rather than a long indoor/hall reverb.
        let tap = |ms: f32| {
            let delay = (SAMPLE_RATE * ms / 1000.0) as usize;
            self.reflections
                [(self.cursor + self.reflections.len() - delay) % self.reflections.len()]
        };
        let ground = tap(4.3) * 0.28;
        let left = ground + tap(19.7) * 0.26 + tap(43.1) * 0.17 + tap(79.3) * 0.10;
        let right = ground + tap(25.3) * 0.26 + tap(53.7) * 0.17 + tap(89.9) * 0.10;
        let feedback = tap(113.0);
        self.reflection_low += (feedback - self.reflection_low) * 0.12;
        self.reflections[self.cursor] = sample + self.reflection_low * 0.28;
        self.cursor = (self.cursor + 1) % self.reflections.len();
        let wet = 0.35 + (1.0 - self.gain) * 0.65;
        let direct = 0.85 - (1.0 - self.gain) * 0.20;
        [
            sample * direct * (1.0 - self.pan).sqrt() + left * wet,
            sample * direct * (1.0 + self.pan).sqrt() + right * wet,
        ]
    }
}

impl EngineAudio {
    // Preserve surf/gust phase across recovery/restart; only clear vehicle transients.
    pub fn reset_feedback(&mut self) {
        self.synth = Synth::default();
        self.spatial = SpatialAudio::default();
    }

    pub fn new(sdl: &sdl3::Sdl) -> Result<Self, String> {
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
        Ok(Self {
            stream,
            _subsystem: audio,
            synth: Synth::default(),
            spatial: SpatialAudio::default(),
            muted: false,
        })
    }

    pub fn update(
        &mut self,
        car: &crate::phys_car::Vehicle,
        listener: crate::camera::Pose,
        throttle: bool,
        active: bool,
        impact: f32,
    ) -> Result<(), String> {
        let queued = self
            .stream
            .queued_bytes()
            .map_err(|e| e.to_string())?
            .max(0) as usize
            / (2 * std::mem::size_of::<f32>());
        let count = QUEUE_SAMPLES.saturating_sub(queued);
        let enabled = active && !self.muted;
        self.synth.turbo_target = car.turbo_spool;
        self.synth.road_speed = car.velocity.x.hypot(car.velocity.z);
        if enabled {
            self.synth.hit(impact);
        } else {
            self.synth.suppress_impacts();
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
        let placement = SpatialAudio::placement(car.position, listener);
        let samples: Vec<f32> = (0..count)
            .flat_map(|_| {
                let driving = self.spatial.sample(self.synth.sample(voice), placement);
                driving.map(output_sample)
            })
            .collect();
        if !samples.is_empty() {
            self.stream
                .put_data_f32(&samples)
                .map_err(|e| e.to_string())?;
        }
        Ok(())
    }
}

fn output_sample(driving: f32) -> f32 {
    // Preserve driving gain and output headroom.
    (driving * 0.92).clamp(-0.3, 0.3)
}

fn engine_frequency(rpm: f32) -> f32 {
    rpm.max(crate::phys_car::IDLE_RPM) / 30.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn engine_voice_stays_present_with_headroom_for_the_driving_mix() {
        for load in [0.12, 1.0] {
            let mut synth = Synth::default();
            let mut energy = 0.0;
            let mut clipped = 0;
            for i in 0..48000 {
                let sample = synth.sample(Voice {
                    frequency: 150.0,
                    load,
                    scrub: 0.0,
                    tuning: crate::test_support::audio_voice(),
                    enabled: true,
                });
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
            let output = spatial.sample(if i == 0 { 0.2 } else { 0.0 }, placement);
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
        let listener = crate::camera::Pose {
            eye: crate::geometry::XYZ {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            target: crate::geometry::XYZ {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        };
        let right = SpatialAudio::placement(
            crate::geometry::XYZ {
                x: 5.0,
                y: 0.0,
                z: 5.0,
            },
            listener,
        );
        let left = SpatialAudio::placement(
            crate::geometry::XYZ {
                x: -5.0,
                y: 0.0,
                z: 5.0,
            },
            listener,
        );
        assert!(right.1 > 0.0 && left.1 < 0.0);
        let reversed = SpatialAudio::placement(
            crate::geometry::XYZ {
                x: 5.0,
                y: 0.0,
                z: 5.0,
            },
            crate::camera::Pose {
                target: crate::geometry::XYZ {
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
            let output = spatial.sample(0.1, right);
            assert!(output.iter().all(|v| v.is_finite() && v.abs() < 0.3));
        }
        let before = spatial.sample(0.1, right);
        assert!(before[1] > before[0]);
        let after = spatial.sample(0.1, left);
        assert!((after[0] - before[0]).abs() < 0.001);
        assert!((after[1] - before[1]).abs() < 0.001);
    }

    #[test]
    fn impacts_scale_decay_and_do_not_retrigger_on_tiny_contacts() {
        let mut synth = Synth::default();
        synth.hit(1.0);
        assert_eq!(synth.impact, 0.0);
        synth.hit(4.0);
        let initial = synth.impact;
        synth.hit(12.0);
        assert_eq!(synth.impact, initial); // cooldown suppresses contact chatter
        let voice = Voice {
            frequency: 30.0,
            load: 0.0,
            scrub: 0.0,
            tuning: crate::test_support::audio_voice(),
            enabled: true,
        };
        for _ in 0..48000 {
            synth.sample(voice);
        }
        assert!(synth.impact < 0.0001);
        synth.hit(12.0);
        assert!(synth.impact > initial);
        synth.suppress_impacts();
        assert_eq!(synth.impact, 0.0);
    }

    #[test]
    fn driving_output_is_bounded_and_fades_on_mute_or_pause() {
        let mut engine = Synth::default();
        let mut voice = Voice {
            frequency: 180.0,
            load: 1.0,
            scrub: 1.0,
            tuning: crate::test_support::audio_voice(),
            enabled: true,
        };
        let mut driving_energy = 0.0;
        for i in 0..48000 {
            if i % 6000 == 0 {
                engine.hit(20.0);
            }
            let driving = engine.sample(voice);
            let sample = output_sample(driving);
            assert!(sample.is_finite() && sample.abs() <= 0.3);
            driving_energy += driving * driving;
        }
        assert!(driving_energy > 0.01);
        voice.enabled = false;
        engine.suppress_impacts();
        let mut tail: f32 = 0.0;
        for i in 0..24000 {
            let sample = output_sample(engine.sample(voice));
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
            let sample = tires.sample(1.0, 30.0);
            assert!(sample.is_finite() && sample.abs() < 0.2);
            energy += sample * sample;
            difference_energy += (sample - previous).powi(2);
            previous = sample;
        }
        // Broadband hiss has roughly twice as much first-difference energy.
        assert!(energy > 1.0 && difference_energy < energy * 0.4);
        let before = tires.gain;
        tires.sample(0.0, 30.0);
        assert!(tires.gain < before && tires.gain > before * 0.99);
        let mut tail = 0.0_f32;
        for i in 0..48000 {
            let sample = tires.sample(0.0, 30.0);
            if i > 40000 {
                tail = tail.max(sample.abs());
            }
        }
        assert!(tail < 0.00001);
    }
}

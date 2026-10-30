use crate::ambience::{CoastalAmbience, Levels};
use crate::cars::CarModel;
use sdl3::audio::{AudioFormat, AudioSpec, AudioStreamOwner};
use std::f32::consts::TAU;

const RATE: f32 = crate::ambience::SAMPLE_RATE;
const QUEUE_SAMPLES: usize = 2400;

// Recorded Roadster engine with procedural fallback voices, tires and environment.
pub struct EngineAudio {
    stream: AudioStreamOwner,
    _subsystem: sdl3::AudioSubsystem,
    synth: Synth,
    ambience: CoastalAmbience,
    spatial: SpatialAudio,
    pub muted: bool,
}

#[derive(Clone, Copy)]
struct Voice {
    frequency: f32,
    load: f32,
    scrub: f32,
    model: CarModel,
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
    tire_gain: f32,
    tire_phase: f32,
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

    fn sample(&mut self, voice: Voice) -> f32 {
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
        self.tire_gain += (tire - self.tire_gain) * 0.001;
        self.master += (f32::from(u8::from(voice.enabled)) - self.master) * 0.002;
        self.phase = (self.phase + self.frequency / RATE).rem_euclid(2.0);
        // Rubber resonance follows slip, independently of engine speed or gear.
        self.tire_phase = (self.tire_phase + (1050.0 + self.tire_gain * 420.0) / RATE).fract();
        let p = self.phase * TAU;
        let harmonics = match voice.model.appearance() {
            CarModel::Roadster => p.sin() * 0.65 + (p * 2.0).sin() * 0.23 + (p * 4.0).sin() * 0.12,
            CarModel::Muscle => p.sin() * 0.55 + (p * 0.5).sin() * 0.30 + (p * 3.0).sin() * 0.15,
            CarModel::Club | CarModel::Custom(_) => {
                p.sin() * 0.52 + (p * 3.0).sin() * 0.30 + (p * 6.0).sin() * 0.18
            }
        };
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
        let engine = if voice.model.appearance() == CarModel::Roadster && self.recorded.available()
        {
            // Kestrel is a four-stroke six-cylinder: three firing pulses per revolution.
            self.recorded.sample(self.frequency * 1.5, self.load)
        } else {
            procedural
        };
        // Lift-off keeps the in-gear pitch but softens the engine independently
        // of rolling noise, tire scrub and impacts. Load is already smoothed.
        let engine = engine * (0.35 + self.load * 0.65);
        let turbo_target = if voice.enabled {
            self.turbo_target
        } else {
            0.0
        };
        self.turbo_spool += (turbo_target - self.turbo_spool) * 0.0007;
        // Broad compressor airflow rather than a pitched oscillator. Two low-pass
        // filters form a soft band: remove rumble and the brittle top-end hiss.
        // The air gets brighter and stronger as turbine speed builds.
        let low_cutoff = 180.0 + 220.0 * self.turbo_spool;
        let high_cutoff = 1500.0 + 2800.0 * self.turbo_spool;
        self.turbo_air_low +=
            (noise - self.turbo_air_low) * (1.0 - (-TAU * low_cutoff / RATE).exp());
        self.turbo_air_high +=
            (noise - self.turbo_air_high) * (1.0 - (-TAU * high_cutoff / RATE).exp());
        let whoosh = (self.turbo_air_high - self.turbo_air_low) * 0.22 * self.turbo_spool.powf(1.4);
        let road_target = (self.road_speed.abs() / 45.0).clamp(0.0, 1.0).powf(1.3);
        self.road_gain += (road_target - self.road_gain) * 0.0005;
        self.road_noise += (noise - self.road_noise) * 0.025;
        let rolling = (self.road_noise * 0.18 + self.filtered_noise * 0.035) * self.road_gain;
        let tires = ((noise - self.filtered_noise) * 0.065 + (self.tire_phase * TAU).sin() * 0.012)
            * self.tire_gain;
        self.impact_phase = (self.impact_phase + (48.0 + self.impact * 42.0) / RATE).fract();
        let impact = ((self.impact_phase * TAU).sin() * 0.7 + noise * 0.3) * self.impact * 0.22;
        self.impact *= 0.99981;
        self.impact_cooldown = (self.impact_cooldown - 1.0 / RATE).max(0.0);
        ((engine + whoosh + tires + rolling + impact) * self.master).clamp(-0.3, 0.3)
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
            reflections: vec![0.0; (RATE * 0.15) as usize],
            cursor: 0,
            reflection_low: 0.0,
        }
    }
}

impl SpatialAudio {
    fn placement(source: [f32; 3], listener: crate::camera::Pose) -> (f32, f32, f32) {
        let offset: [f32; 3] = std::array::from_fn(|i| source[i] - listener.eye[i]);
        let distance = offset.iter().map(|v| v * v).sum::<f32>().sqrt();
        // Gentle falloff keeps driving cues present; distance also changes
        // the direct/reflected balance below, rather than just turning down.
        let gain = (6.0 / (distance + 3.0)).min(1.0);
        let forward = [
            listener.target[0] - listener.eye[0],
            listener.target[2] - listener.eye[2],
        ];
        let length = forward[0].hypot(forward[1]);
        let pan = if length > 0.001 && distance > 0.001 {
            ((offset[0] * forward[1] - offset[2] * forward[0]) / (length * distance))
                .clamp(-1.0, 1.0)
        } else {
            0.0
        };
        let cutoff = 6500.0 / (1.0 + distance / 8.0);
        let filter = 1.0 - (-TAU * cutoff.max(350.0) / RATE).exp();
        (gain, pan, filter)
    }

    fn sample(&mut self, input: f32, placement: (f32, f32, f32)) -> [f32; 2] {
        let (gain, pan, filter) = placement;
        let blend = 1.0 - (-1.0 / (RATE * 0.03)).exp();
        self.gain += (gain - self.gain) * blend;
        self.pan += (pan - self.pan) * blend;
        self.filtered += (input - self.filtered) * filter;
        let sample = self.filtered * self.gain;
        // A ground bounce arrives first, followed by sparse, asymmetric outdoor
        // reflections. Different arrival times in each ear externalize even a
        // centred source. A damped feedback path adds a short diffuse decay,
        // rather than a long indoor/hall reverb.
        let tap = |ms: f32| {
            let delay = (RATE * ms / 1000.0) as usize;
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
            freq: Some(RATE as i32),
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
            ambience: CoastalAmbience::default(),
            spatial: SpatialAudio::default(),
            muted: false,
        })
    }

    pub fn update(
        &mut self,
        car: &crate::vehicle::Vehicle,
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
        self.synth.road_speed = car.velocity[0].hypot(car.velocity[1]);
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
            model: car.model,
            enabled,
        };
        let levels = Levels::on_track(
            car.position,
            car.velocity[0].hypot(car.velocity[1]),
            &car.track,
        );
        let placement = SpatialAudio::placement(car.position, listener);
        let samples: Vec<f32> = (0..count)
            .flat_map(|_| {
                let driving = self.spatial.sample(self.synth.sample(voice), placement);
                let ambience = self.ambience.sample(levels, enabled);
                driving.map(|channel| mix(channel, ambience))
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

fn mix(driving: f32, ambience: f32) -> f32 {
    // Keep the environment behind driving cues with headroom and one final safety limit.
    (driving * 0.92 + ambience).clamp(-0.3, 0.3)
}

fn engine_frequency(rpm: f32) -> f32 {
    rpm.clamp(crate::vehicle::IDLE_RPM, crate::vehicle::REDLINE_RPM) / 30.0
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn distance_reduces_level_and_high_frequencies() {
        let listener = crate::camera::Pose {
            eye: [0.0; 3],
            target: [0.0, 0.0, 1.0],
        };
        let near = SpatialAudio::placement([0.0, 0.0, 5.5], listener);
        let far = SpatialAudio::placement([0.0, 0.0, 10.0], listener);
        assert!(near.0 > far.0 && near.2 > far.2);
        assert_eq!(near.1, 0.0);
        let origin = SpatialAudio::placement(listener.eye, listener);
        assert_eq!(origin.0, 1.0);
        assert!(origin.2.is_finite());
    }

    #[test]
    fn stereo_follows_listener_orientation_and_changes_smoothly() {
        let listener = crate::camera::Pose {
            eye: [0.0; 3],
            target: [0.0, 0.0, 1.0],
        };
        let right = SpatialAudio::placement([5.0, 0.0, 5.0], listener);
        let left = SpatialAudio::placement([-5.0, 0.0, 5.0], listener);
        assert!(right.1 > 0.0 && left.1 < 0.0);
        let reversed = SpatialAudio::placement(
            [5.0, 0.0, 5.0],
            crate::camera::Pose {
                target: [0.0, 0.0, -1.0],
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
    fn coasting_quiets_every_engine_without_changing_pitch() {
        for model in crate::cars::ALL {
            for rpm in [2500.0, 5500.0] {
                let render = |load| {
                    let mut synth = Synth::default();
                    let voice = Voice {
                        frequency: engine_frequency(rpm),
                        load,
                        scrub: 0.0,
                        model,
                        enabled: true,
                    };
                    let mut energy = 0.0;
                    for i in 0..24000 {
                        let sample = synth.sample(voice);
                        if i >= 12000 {
                            energy += sample * sample;
                        }
                    }
                    (energy, synth.frequency)
                };
                let (coast, coast_pitch) = render(0.12);
                let (powered, powered_pitch) = render(1.0);
                assert!(
                    coast > 0.0 && coast < powered * 0.4,
                    "{model:?} at {rpm} RPM: coast {coast}, powered {powered}"
                );
                assert_eq!(coast_pitch, powered_pitch);
            }
        }
    }

    #[test]
    fn rolling_noise_follows_speed_independently_of_rpm() {
        let mut stopped = Synth::default();
        let mut moving = Synth {
            road_speed: 30.0,
            ..Synth::default()
        };
        let voice = Voice {
            frequency: 60.0,
            load: 0.0,
            scrub: 0.0,
            model: CarModel::Roadster,
            enabled: true,
        };
        let difference: f32 = (0..24000)
            .map(|_| {
                let a = stopped.sample(voice);
                let b = moving.sample(voice);
                (a - b).abs()
            })
            .sum();
        assert!(difference > 1.0);
        assert_eq!(stopped.road_gain, 0.0);
        assert!(moving.road_gain > 0.5);
    }

    #[test]
    #[ignore = "exports a WAV preview; set NFS_AUDIO_PREVIEW to the output path"]
    fn recorded_driving_preview() {
        let path = std::env::var_os("NFS_AUDIO_PREVIEW").expect("set NFS_AUDIO_PREVIEW");
        let mut synth = Synth::default();
        let mut pcm = Vec::new();
        let mut peak = 0.0_f32;
        for i in 0..(RATE as usize * 18) {
            let t = i as f32 / RATE;
            // Idle, accelerate through three shifts, coast, skid, then mute.
            let rpm = if t < 2.0 {
                900.0
            } else if t < 12.0 {
                2200.0 + ((t - 2.0) % 3.0) / 3.0 * 4000.0
            } else {
                (5800.0 - (t - 12.0) * 900.0).max(900.0)
            };
            synth.road_speed = ((t - 2.0) * 3.5).clamp(0.0, 35.0);
            let sample = synth.sample(Voice {
                frequency: engine_frequency(rpm),
                load: if t < 12.0 { 0.85 } else { 0.0 },
                scrub: if (14.0..16.0).contains(&t) { 0.8 } else { 0.0 },
                model: CarModel::Roadster,
                enabled: t < 16.0,
            });
            assert!(sample.is_finite() && sample.abs() <= 0.3);
            if t > 17.5 {
                assert!(sample.abs() < 0.00001);
            }
            peak = peak.max(sample.abs());
            pcm.extend_from_slice(&((sample * 32767.0) as i16).to_le_bytes());
        }
        assert!(peak > 0.03);
        let mut wav = Vec::new();
        wav.extend_from_slice(b"RIFF");
        wav.extend_from_slice(&(36 + pcm.len() as u32).to_le_bytes());
        wav.extend_from_slice(b"WAVEfmt \x10\0\0\0\x01\0\x01\0");
        wav.extend_from_slice(&(RATE as u32).to_le_bytes());
        wav.extend_from_slice(&(RATE as u32 * 2).to_le_bytes());
        wav.extend_from_slice(b"\x02\0\x10\0data");
        wav.extend_from_slice(&(pcm.len() as u32).to_le_bytes());
        wav.extend_from_slice(&pcm);
        std::fs::write(path, wav).unwrap();
    }

    #[test]
    fn engine_pitch_follows_ratio_based_shifts_on_every_route() {
        for track in crate::track::ALL.map(|id| id.track()) {
            let road = track.at_distance(50.0);
            for model in crate::cars::ALL {
                let mut car = crate::vehicle::Vehicle::on_track(
                    road.lane_position(0.0),
                    road.heading(),
                    model,
                    track.clone(),
                );
                let step = car.upshift_speed(1);
                car.velocity = road.tangent.map(|v| v * (step - 0.01));
                car.refresh_speed();
                let before = engine_frequency(car.target_engine_rpm());
                car.velocity = road.tangent.map(|v| v * (step + 0.01));
                car.refresh_speed();
                let after = engine_frequency(car.target_engine_rpm());
                assert_eq!(car.gear, 2);
                assert!(before > after + 30.0);
            }
        }
    }

    #[test]
    fn engine_pitch_matches_every_gears_ratio_drop() {
        for model in crate::cars::ALL {
            let g = &model.spec().gearbox;
            for gear in 1..g.gears() {
                let speed = g.speed_at_rpm(g.upshift_rpm, gear);
                let before = engine_frequency(g.rpm(speed, gear));
                let after = engine_frequency(g.rpm(speed, gear + 1));
                let expected = g.ratios[gear as usize] / g.ratios[gear as usize - 1];
                assert!((after / before - expected).abs() < 0.001);
                assert!(after < before);
            }
        }
    }

    #[test]
    fn procedural_audio_is_bounded_and_fades_to_silence() {
        for model in crate::cars::ALL {
            let mut synth = Synth::default();
            let mut voice = Voice {
                frequency: 180.0,
                load: 1.0,
                scrub: 1.0,
                model,
                enabled: true,
            };
            for i in 0..48000 {
                if i % 6000 == 0 {
                    synth.hit(20.0);
                }
                let s = synth.sample(voice);
                assert!(s.is_finite() && s.abs() <= 0.3);
            }
            synth.suppress_impacts();
            voice.enabled = false;
            let mut last = 1.0;
            for _ in 0..12000 {
                last = synth.sample(voice);
            }
            assert!(last.abs() < 0.0001);
        }
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
            model: CarModel::Roadster,
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
    fn models_have_distinct_timbres_and_scrub_adds_an_independent_layer() {
        let voices = crate::cars::ALL.map(|model| {
            let mut synth = Synth::default();
            let voice = Voice {
                frequency: 120.0,
                load: 1.0,
                scrub: 0.0,
                model,
                enabled: true,
            };
            (0..4000).map(|_| synth.sample(voice)).collect::<Vec<_>>()
        });
        assert_ne!(voices[0], voices[1]);
        assert_ne!(voices[0], voices[2]);
        let mut quiet = Synth::default();
        let mut sliding = Synth::default();
        let voice = Voice {
            frequency: 80.0,
            load: 0.0,
            scrub: 0.0,
            model: CarModel::Club,
            enabled: true,
        };
        let difference: f32 = (0..4000)
            .map(|_| {
                (quiet.sample(voice)
                    - sliding.sample(Voice {
                        scrub: 1.0,
                        ..voice
                    }))
                .abs()
            })
            .sum();
        assert!(difference > 1.0);
    }

    #[test]
    fn full_coastal_mix_is_bounded_subordinate_and_fades_on_mute_or_pause() {
        for model in crate::cars::ALL {
            let mut engine = Synth::default();
            let mut coast = CoastalAmbience::default();
            let mut voice = Voice {
                frequency: 180.0,
                load: 1.0,
                scrub: 1.0,
                model,
                enabled: true,
            };
            let levels = Levels {
                wind: 1.0,
                surf: 1.0,
            };
            let (mut driving_energy, mut coast_energy) = (0.0, 0.0);
            for i in 0..48000 {
                if i % 6000 == 0 {
                    engine.hit(20.0);
                }
                let driving = engine.sample(voice);
                let ambience = coast.sample(levels, true);
                let sample = mix(driving, ambience);
                assert!(sample.is_finite() && sample.abs() <= 0.3);
                driving_energy += driving * driving;
                coast_energy += ambience * ambience;
            }
            assert!(coast_energy > 0.01 && coast_energy < driving_energy * 0.05);
            voice.enabled = false;
            engine.suppress_impacts();
            let mut tail: f32 = 0.0;
            for i in 0..24000 {
                let sample = mix(engine.sample(voice), coast.sample(levels, false));
                if i > 20000 {
                    tail = tail.max(sample.abs());
                }
            }
            assert!(tail < 0.00001);
        }
    }

    #[test]
    fn tire_tone_phase_does_not_reset_at_engine_cycle_boundaries() {
        let mut synth = Synth {
            phase: 1.999,
            frequency: 120.0,
            tire_phase: 0.22,
            ..Synth::default()
        };
        let voice = Voice {
            frequency: 120.0,
            load: 0.0,
            scrub: 1.0,
            model: CarModel::Roadster,
            enabled: true,
        };
        synth.sample(voice);
        assert!(synth.phase < 0.01);
        assert!((synth.tire_phase - (0.22 + 1050.42 / RATE)).abs() < 0.00001);
    }

    #[test]
    fn reverse_pitch_increases_with_backwards_speed() {
        for model in crate::cars::ALL {
            let road = crate::track::at_distance(50.0);
            let mut car =
                crate::vehicle::Vehicle::new(road.lane_position(0.0), road.heading(), model);
            car.toggle_direction();
            let idle = engine_frequency(car.target_engine_rpm());
            let gearbox = &model.spec().gearbox;
            let speed = 2000.0 * std::f32::consts::TAU * gearbox.rolling_radius
                / (60.0 * gearbox.reverse_ratio * gearbox.final_drive);
            car.speed = -speed;
            let slow = engine_frequency(car.target_engine_rpm());
            car.speed = -speed * 2.0;
            let fast = engine_frequency(car.target_engine_rpm());
            assert!(idle < slow && slow < fast && fast <= 6500.0 / 30.0);
        }
    }
}

#[cfg(test)]
mod turbo_tests {
    use super::*;

    #[test]
    fn whoosh_follows_spool_and_fades_when_disabled() {
        let voice = Voice {
            frequency: 120.0,
            load: 0.7,
            scrub: 0.0,
            model: CarModel::Roadster,
            enabled: true,
        };
        let mut quiet = Synth::default();
        let mut turbo = Synth {
            turbo_target: 1.0,
            ..Synth::default()
        };
        let mut energy = 0.0;
        for _ in 0..24000 {
            let difference = turbo.sample(voice) - quiet.sample(voice);
            energy += difference * difference;
        }
        assert!(energy > 1.0);
        assert!(turbo.turbo_spool > 0.99);
        for _ in 0..24000 {
            turbo.sample(Voice {
                enabled: false,
                ..voice
            });
        }
        assert!(turbo.turbo_spool < 0.001);
    }
}

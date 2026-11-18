//! Tests through the group's production interfaces.

mod synthesis {
    use crate::model::world::Material;
    use crate::render;
    use crate::sound;
    use crate::sound::spatial::SpatialAudio;
    use crate::sound::synth::{Synth, Voice};
    use crate::sound::tires::TireSound;
    use crate::util::geometry::XYZ;

    #[test]
    fn engine_voice_stays_present_with_headroom_for_the_driving_mix() {
        for load in [0.12, 1.0] {
            let mut synth = Synth::default();
            let mut energy = 0.0;
            let mut clipped = 0;
            for i in 0..48000 {
                let sample = sound::synth::sample(
                    &mut synth,
                    Voice {
                        turbo_target: 0.0,
                        road_speed: 0.0,
                        frequency: 150.0,
                        load,
                        scrub: 0.0,
                        surface: Material::Asphalt,
                        sample_directory: None,
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
        let mut spatial = SpatialAudio::default();
        let placement = sound::spatial::placement(
            XYZ {
                x: 0.0,
                y: 0.0,
                z: 6.0,
            },
            render::camera::Pose {
                eye: XYZ::default(),
                target: XYZ {
                    x: 0.0,
                    y: 0.0,
                    z: 1.0,
                },
            },
        );
        for _ in 0..24000 {
            sound::spatial::sample(&mut spatial, 0.0, placement);
        }
        let mut early = 0.0;
        let mut reflected = 0.0;
        let mut width = 0.0;
        let mut tail = 0.0_f32;
        for i in 0..48000 {
            let output =
                sound::spatial::sample(&mut spatial, if i == 0 { 0.2 } else { 0.0 }, placement);
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
        let listener = render::camera::Pose {
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
        let right = sound::spatial::placement(
            XYZ {
                x: 5.0,
                y: 0.0,
                z: 5.0,
            },
            listener,
        );
        let left = sound::spatial::placement(
            XYZ {
                x: -5.0,
                y: 0.0,
                z: 5.0,
            },
            listener,
        );
        let reversed = sound::spatial::placement(
            XYZ {
                x: 5.0,
                y: 0.0,
                z: 5.0,
            },
            render::camera::Pose {
                target: XYZ {
                    x: 0.0,
                    y: 0.0,
                    z: -1.0,
                },
                ..listener
            },
        );
        let mut reversed_audio = SpatialAudio::default();
        let mut output = [0.0; 2];
        for _ in 0..24000 {
            output = sound::spatial::sample(&mut reversed_audio, 0.1, reversed);
        }
        assert!(output[0] > output[1]);
        let mut spatial = SpatialAudio::default();
        for _ in 0..24000 {
            let output = sound::spatial::sample(&mut spatial, 0.1, right);
            assert!(output.iter().all(|v| v.is_finite() && v.abs() < 0.3));
        }
        let before = sound::spatial::sample(&mut spatial, 0.1, right);
        assert!(before[1] > before[0]);
        let after = sound::spatial::sample(&mut spatial, 0.1, left);
        assert!((after[0] - before[0]).abs() < 0.001);
        assert!((after[1] - before[1]).abs() < 0.001);
    }

    #[test]
    fn driving_output_is_bounded_and_fades_on_mute_or_pause() {
        let mut engine = Synth::default();
        let mut voice = Voice {
            turbo_target: 0.0,
            road_speed: 0.0,
            frequency: 180.0,
            load: 1.0,
            scrub: 1.0,
            surface: Material::Asphalt,
            sample_directory: None,
            enabled: true,
        };
        let mut driving_energy = 0.0;
        for i in 0..48000 {
            if i % 6000 == 0 {
                sound::synth::hit(&mut engine, 20.0);
            }
            let sample = sound::synth::sample(&mut engine, voice);
            assert!(sample.is_finite() && sample.abs() <= 0.3);
            driving_energy += sample * sample;
        }
        assert!(driving_energy > 0.01);
        voice.enabled = false;
        sound::synth::suppress_impacts(&mut engine);
        let mut tail: f32 = 0.0;
        for i in 0..24000 {
            let sample = sound::synth::sample(&mut engine, voice);
            if i > 20000 {
                tail = tail.max(sample.abs());
            }
        }
        assert!(tail < 0.00001);
    }

    #[test]
    fn tire_squeal_limits_rapid_sample_changes_and_fades_on_release() {
        let mut tires = TireSound::default();
        let mut previous = 0.0;
        let mut energy = 0.0;
        let mut difference_energy = 0.0;
        for _ in 0..48000 {
            let sample = sound::tires::sample(&mut tires, 1.0, 30.0, Material::Asphalt);
            assert!(sample.is_finite() && sample.abs() < 0.2);
            energy += sample * sample;
            difference_energy += (sample - previous).powi(2);
            previous = sample;
        }
        // First differences measure rapid sample changes; this is a roughness
        // check, not a spectral cutoff measurement. White noise has a ratio near 2.
        assert!(energy > 1.0 && difference_energy < energy * 0.4);
        let released = sound::tires::sample(&mut tires, 0.0, 30.0, Material::Asphalt);
        assert!((released - previous).abs() < 0.1, "release must not click");
        let mut tail = 0.0_f32;
        for i in 0..48000 {
            let sample = sound::tires::sample(&mut tires, 0.0, 30.0, Material::Asphalt);
            if i > 40000 {
                tail = tail.max(sample.abs());
            }
        }
        assert!(tail < 0.00001);
    }

    #[test]
    fn impacts_scale_with_speed_decay_and_ignore_contacts_during_cooldown() {
        let voice = Voice {
            frequency: 30.0,
            load: 0.0,
            scrub: 0.0,
            surface: Material::Asphalt,
            sample_directory: None,
            enabled: true,
            turbo_target: 0.0,
            road_speed: 0.0,
        };
        let mut quiet = Synth::default();
        let mut tiny = Synth::default();
        sound::synth::hit(&mut tiny, 1.0);
        for _ in 0..1000 {
            assert_eq!(
                sound::synth::sample(&mut tiny, voice),
                sound::synth::sample(&mut quiet, voice)
            );
        }
        let mut baseline = Synth::default();
        let mut impact = Synth::default();
        let mut chatter = Synth::default();
        let mut stronger = Synth::default();
        sound::synth::hit(&mut stronger, 12.0);
        sound::synth::hit(&mut impact, 4.0);
        sound::synth::hit(&mut chatter, 4.0);
        sound::synth::hit(&mut chatter, 12.0);
        let mut energy = 0.0;
        let mut stronger_energy = 0.0;
        let mut tail = 0.0;
        for i in 0..192000 {
            let sample = sound::synth::sample(&mut impact, voice);
            assert_eq!(sample, sound::synth::sample(&mut chatter, voice));
            let reference = sound::synth::sample(&mut baseline, voice);
            let transient = sample - reference;
            let stronger_transient = sound::synth::sample(&mut stronger, voice) - reference;
            if i < 12000 {
                energy += transient * transient;
                stronger_energy += stronger_transient * stronger_transient;
            }
            if i > 180000 {
                tail += transient * transient;
            }
        }
        assert!(energy > 0.0 && tail < energy * 0.01, "impact did not decay");
        assert!(
            stronger_energy > energy * 2.0,
            "stronger impact must carry more energy"
        );
        sound::synth::suppress_impacts(&mut impact);
        sound::synth::suppress_impacts(&mut chatter);
        sound::synth::hit(&mut impact, 12.0);
        let mut difference = 0.0;
        for _ in 0..4800 {
            difference += (sound::synth::sample(&mut impact, voice)
                - sound::synth::sample(&mut chatter, voice))
            .abs();
        }
        assert!(difference > 1.0);
    }
}

mod recorded_engine {
    use crate::sound;
    use crate::sound::engine_sample::RecordedEngine;
    use crate::sound::test_support::audio_dir;
    use std::path::Path;
    #[test]
    fn missing_or_malformed_car_audio_is_rejected() {
        let missing = tempfile::tempdir().unwrap();
        assert!(!bank_available(missing.path()));
        let valid = tempfile::tempdir().unwrap();
        sound::test_support::copy_bank(valid.path());
        assert!(bank_available(valid.path()));
        // Detailed malformed-WAV cases belong to the decoder tests. Here we
        // only check that a broken recording makes the whole bank unavailable.
        let broken = tempfile::tempdir().unwrap();
        sound::test_support::copy_bank(broken.path());
        std::fs::write(broken.path().join("1200-coast.wav"), b"not a WAV").unwrap();
        assert!(!bank_available(broken.path()));
    }

    #[test]
    fn invalid_recording_names_are_rejected_before_playback() {
        for name in [
            "0-load.wav",
            "299-load.wav",
            "20001-load.wav",
            "1200.5-load.wav",
            "idle-load.wav",
            "1200-both.wav",
            "1200.wav",
        ] {
            let directory = tempfile::tempdir().unwrap();
            sound::test_support::copy_bank(directory.path());
            std::fs::copy(
                audio_dir().join("1200-load.wav"),
                directory.path().join(name),
            )
            .unwrap();
            assert!(!bank_available(directory.path()), "accepted {name}");
        }
    }

    #[test]
    fn duplicate_rpm_is_rejected_even_with_different_filename_spelling() {
        let directory = tempfile::tempdir().unwrap();
        sound::test_support::copy_bank(directory.path());
        std::fs::rename(
            directory.path().join("1200-load.wav"),
            directory.path().join("01200-load.wav"),
        )
        .unwrap();
        assert!(
            bank_available(directory.path()),
            "a leading zero alone is valid"
        );
        let duplicate = tempfile::tempdir().unwrap();
        sound::test_support::copy_bank(duplicate.path());
        std::fs::copy(
            audio_dir().join("1200-load.wav"),
            duplicate.path().join("01200-load.wav"),
        )
        .unwrap();
        assert!(
            !bank_available(duplicate.path()),
            "duplicate numeric RPM must fail"
        );
    }

    #[test]
    fn missing_axis_and_excess_recordings_are_rejected() {
        for state in ["load", "coast"] {
            let directory = tempfile::tempdir().unwrap();
            sound::test_support::copy_bank(directory.path());
            for rpm in [1200, 3000, 6000] {
                std::fs::remove_file(directory.path().join(format!("{rpm}-{state}.wav"))).unwrap();
            }
            assert!(!bank_available(directory.path()));
        }
        let directory = tempfile::tempdir().unwrap();
        sound::test_support::copy_bank(directory.path());
        for rpm in 7000..7014 {
            std::fs::copy(
                audio_dir().join("1200-load.wav"),
                directory.path().join(format!("{rpm}-load.wav")),
            )
            .unwrap();
        }
        assert!(!bank_available(directory.path()));
    }

    fn bank_available(path: &Path) -> bool {
        let mut engine = RecordedEngine::default();
        sound::engine_sample::select(&mut engine, Some(path));
        sound::engine_sample::available(&engine)
    }
    fn recorded() -> RecordedEngine {
        let mut engine = RecordedEngine::default();
        sound::engine_sample::select(&mut engine, Some(audio_dir()));
        engine
    }
    #[test]
    fn switching_recording_banks_changes_the_output_and_resets_playback() {
        let quieter = tempfile::tempdir().unwrap();
        sound::test_support::write_bank(quieter.path(), 0.15);
        let mut engine = recorded();
        let original: Vec<_> = (0..1024)
            .map(|_| sound::engine_sample::sample(&mut engine, 3000.0, 1.0))
            .collect();
        sound::engine_sample::select(&mut engine, Some(quieter.path()));
        let changed: Vec<_> = (0..1024)
            .map(|_| sound::engine_sample::sample(&mut engine, 3000.0, 1.0))
            .collect();
        let original_energy: f32 = original.iter().map(|v| v * v).sum();
        let changed_energy: f32 = changed.iter().map(|v| v * v).sum();
        assert!(original_energy > 0.0);
        let ratio = changed_energy / original_energy;
        assert!(
            (0.24..0.26).contains(&ratio),
            "half amplitude must give quarter energy: {ratio}"
        );
        sound::engine_sample::select(&mut engine, Some(audio_dir()));
        let restarted: Vec<_> = (0..1024)
            .map(|_| sound::engine_sample::sample(&mut engine, 3000.0, 1.0))
            .collect();
        assert_eq!(
            restarted, original,
            "switching back must reset playback and filtering"
        );
        sound::engine_sample::select(&mut engine, None);
        assert!(!sound::engine_sample::available(&engine));
        assert_eq!(sound::engine_sample::sample(&mut engine, 3000.0, 1.0), 0.0);
        sound::engine_sample::select(&mut engine, Some(&quieter.path().join("missing")));
        assert!(!sound::engine_sample::available(&engine));
    }

    #[test]
    fn shipped_wav_banks_load() {
        for name in [
            "1964-kestrel",
            "1973-kestrel-turbo",
            "1969-mosquito-ss",
            "2016-pitsunda-2",
        ] {
            let path = Path::new("data/cars").join(name).join("audio");
            let mut engine = RecordedEngine::default();
            sound::engine_sample::select(&mut engine, Some(&path));
            assert!(sound::engine_sample::available(&engine), "{name}");
            let energy: f32 = (0..24000)
                .map(|_| sound::engine_sample::sample(&mut engine, 3000.0, 1.0).powi(2))
                .sum();
            assert!(
                energy.is_finite() && energy > 0.0,
                "{name} must produce audio"
            );
        }
    }
    #[test]
    fn crossing_rpm_boundaries_preserves_recording_phase() {
        for boundary in [1200.0, 3000.0, 6000.0] {
            for direction in [-1.0, 1.0] {
                for load in [0.0, 0.5, 1.0] {
                    let mut reference = recorded();
                    let mut crossing = recorded();
                    let before = boundary - direction * 0.01;
                    let after = boundary + direction * 0.01;
                    // Start at a nonzero waveform phase, away from a loop seam.
                    for _ in 0..13001 {
                        sound::engine_sample::sample(&mut reference, before, load);
                        sound::engine_sample::sample(&mut crossing, before, load);
                    }
                    let mut reference_energy = 0.0;
                    let mut difference_energy = 0.0;
                    for _ in 0..256 {
                        let unchanged = sound::engine_sample::sample(&mut reference, before, load);
                        let crossed = sound::engine_sample::sample(&mut crossing, after, load);
                        reference_energy += unchanged * unchanged;
                        difference_energy += (crossed - unchanged).powi(2);
                    }
                    assert!(reference_energy > 0.0);
                    // A tiny RPM change should differ by under 1% RMS, rather
                    // than restarting a recording at the newly selected band.
                    assert!(
                        difference_energy < reference_energy * 0.0001,
                        "phase discontinuity at {boundary}, direction {direction}, load {load}"
                    );
                }
            }
        }
    }
    #[test]
    fn rpm_sweep_keeps_level_across_recording_boundaries() {
        let points = [900.0, 1200.0, 2100.0, 3000.0, 4500.0, 6000.0, 6500.0];
        for load in [0.0, 1.0] {
            let mut levels = Vec::new();
            for rpm in &points {
                let mut engine = recorded();
                let mut energy = 0.0;
                for i in 0..24000 {
                    let sample = sound::engine_sample::sample(&mut engine, *rpm, load);
                    if i >= 4000 {
                        energy += sample * sample;
                    }
                }
                levels.push((energy / 20000.0).sqrt());
            }
            let quietest = levels.iter().copied().fold(f32::INFINITY, f32::min);
            let loudest = levels.iter().copied().fold(0.0, f32::max);
            assert!(
                levels.iter().all(|level| level.is_finite() && *level > 0.0),
                "load {load}: {levels:?}"
            );
            // Equal-power mixing of these in-phase fixture tones can raise
            // amplitude by sqrt(2). Allow a margin for filtering and resampling.
            assert!(
                loudest / quietest < 1.6,
                "RPM blend changes level too far: {levels:?}"
            );
        }
    }
}

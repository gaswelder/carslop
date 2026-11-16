//! Tests through the group's production interfaces.

mod synthesis {
    use crate::model::world::Material;
    use crate::render;
    use crate::sound;
    use crate::sound::spatial::SpatialAudio;
    use crate::sound::synth::{Synth, Voice};
    use crate::sound::tires::TireSound;
    use crate::test;
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
                        tuning: test::support::audio_voice(),
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
            tuning: test::support::audio_voice(),
            enabled: true,
        };
        let mut driving_energy = 0.0;
        for i in 0..48000 {
            if i % 6000 == 0 {
                sound::synth::hit(&mut engine, 20.0);
            }
            let driving = sound::synth::sample(&mut engine, voice);
            let sample = driving;
            assert!(sample.is_finite() && sample.abs() <= 0.3);
            driving_energy += driving * driving;
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
    fn tire_squeal_is_band_limited_and_releases_smoothly() {
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
        // Broadband hiss has roughly twice as much first-difference energy.
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
    fn impacts_scale_decay_and_do_not_retrigger_on_tiny_contacts() {
        let voice = Voice {
            frequency: 30.0,
            load: 0.0,
            scrub: 0.0,
            surface: Material::Asphalt,
            tuning: test::support::audio_voice(),
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
        sound::synth::hit(&mut impact, 4.0);
        sound::synth::hit(&mut chatter, 4.0);
        sound::synth::hit(&mut chatter, 12.0);
        let mut energy = 0.0;
        let mut tail = 0.0;
        for i in 0..48000 {
            let sample = sound::synth::sample(&mut impact, voice);
            assert_eq!(sample, sound::synth::sample(&mut chatter, voice));
            let transient = sample - sound::synth::sample(&mut baseline, voice);
            if i < 12000 {
                energy += transient * transient;
            }
            if i > 36000 {
                tail += transient * transient;
            }
        }
        assert!(energy > tail * 2.0);
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
    use crate::sound::SAMPLE_RATE;
    use std::path::Path;
    #[test]
    fn missing_or_malformed_car_audio_is_rejected() {
        let missing = tempfile::tempdir().unwrap();
        assert!(!bank_available(missing.path()));
        let valid = tempfile::tempdir().unwrap();
        sound::test_support::copy_bank(valid.path());
        assert!(bank_available(valid.path()));
        let manifest: serde_json::Value =
            serde_json::from_slice(&std::fs::read(audio_dir().join("engine.json")).unwrap())
                .unwrap();
        let names: std::collections::BTreeSet<_> = manifest["bands"]
            .as_array()
            .unwrap()
            .iter()
            .flat_map(|band| {
                [
                    band["load"]["file"].as_str().unwrap(),
                    band["coast"]["file"].as_str().unwrap(),
                ]
            })
            .collect();
        for name in names {
            for bytes in [vec![], vec![0; 100], vec![0; 10_001]] {
                let directory = tempfile::tempdir().unwrap();
                sound::test_support::copy_bank(directory.path());
                std::fs::write(directory.path().join(name), bytes).unwrap();
                assert!(!bank_available(directory.path()), "accepted {name}");
            }
        }
    }

    #[test]
    fn invalid_manifest_is_rejected_before_playback() {
        let original: serde_json::Value =
            serde_json::from_slice(&std::fs::read(audio_dir().join("engine.json")).unwrap())
                .unwrap();
        for (pointer, value) in [
            ("/version", serde_json::json!(2)),
            ("/sample_rate", serde_json::json!(44100)),
            ("/bands/1/rpm", serde_json::json!(500)),
            ("/bands/1/rpm", original["bands"][0]["rpm"].clone()),
            ("/bands/0/load/pulse_hz", serde_json::json!(0)),
            ("/bands/0/coast/overlap", serde_json::json!(0)),
            ("/bands/0/load/file", serde_json::json!("../outside.wav")),
            ("/bands/0/load/file", serde_json::json!("/outside.wav")),
            ("/bands/0/load/file", serde_json::json!("..\\outside.wav")),
            ("/bands/0/load/file", serde_json::json!("outside.pcm")),
            ("/bands", serde_json::json!([])),
        ] {
            let directory = tempfile::tempdir().unwrap();
            sound::test_support::copy_bank(directory.path());
            let mut manifest = original.clone();
            *manifest.pointer_mut(pointer).unwrap() = value;
            std::fs::write(directory.path().join("engine.json"), manifest.to_string()).unwrap();
            assert!(!bank_available(directory.path()), "accepted {pointer}");
        }
        for bytes in [vec![b' '; 65_537], b"{invalid".to_vec()] {
            let directory = tempfile::tempdir().unwrap();
            sound::test_support::copy_bank(directory.path());
            std::fs::write(directory.path().join("engine.json"), bytes).unwrap();
            assert!(!bank_available(directory.path()));
        }
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
    fn recording_banks_follow_the_selected_asset_directory() {
        let directory = tempfile::tempdir().unwrap();
        for entry in std::fs::read_dir(audio_dir()).unwrap() {
            let entry = entry.unwrap();
            if entry.path().is_file() {
                std::fs::copy(entry.path(), directory.path().join(entry.file_name())).unwrap();
            }
        }
        let mut engine = RecordedEngine::default();
        assert!(!sound::engine_sample::available(&(engine)));
        sound::engine_sample::select(&mut engine, Some(directory.path()));
        assert!(sound::engine_sample::available(&(engine)));
        let first = sound::engine_sample::sample(&mut engine, 3000.0, 3.0, 1.0);
        for _ in 0..100 {
            sound::engine_sample::sample(&mut engine, 3000.0, 3.0, 1.0);
        }
        sound::engine_sample::select(&mut engine, None);
        assert!(!sound::engine_sample::available(&(engine)));
        assert_eq!(
            sound::engine_sample::sample(&mut (engine), 3000.0, 3.0, 1.0),
            0.0
        );
        sound::engine_sample::select(&mut engine, Some(directory.path()));
        assert_eq!(
            sound::engine_sample::sample(&mut (engine), 3000.0, 3.0, 1.0),
            first
        );
        sound::engine_sample::select(&mut engine, Some(&directory.path().join("missing")));
        assert!(!sound::engine_sample::available(&(engine)));
    }

    #[test]
    fn shipped_wav_banks_load() {
        for name in ["1964-kestrel", "1973-kestrel-turbo", "2016-pitsunda-2"] {
            let path = Path::new("data/cars").join(name).join("audio");
            let mut engine = RecordedEngine::default();
            sound::engine_sample::select(&mut engine, Some(&path));
            assert!(sound::engine_sample::available(&engine), "{name}");
            let energy: f32 = (0..24000)
                .map(|_| sound::engine_sample::sample(&mut engine, 3000.0, 3.0, 1.0).powi(2))
                .sum();
            assert!(
                energy.is_finite() && energy > 0.0,
                "{name} must produce audio"
            );
        }
    }
    #[test]
    fn recorded_layers_survive_seams_shifts_and_load_changes() {
        let mut engine = recorded();
        assert!(sound::engine_sample::available(&(engine)));
        let mut previous = 0.0_f32;
        let mut largest_step = 0.0_f32;
        for i in 0..480_000 {
            let seconds = i as f32 / SAMPLE_RATE;
            let rpm = 900.0 + (seconds % 3.0) / 3.0 * 5600.0;
            let load = if seconds % 4.0 < 2.0 { 1.0 } else { 0.0 };
            let sample = sound::engine_sample::sample(&mut engine, rpm, 3.0, load);
            assert!(sample.is_finite() && sample.abs() < 0.5);
            largest_step = largest_step.max((sample - previous).abs());
            previous = sample;
        }
        assert!(largest_step < 0.2, "discontinuous voice: {largest_step}");
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
                    let sample = sound::engine_sample::sample(&mut engine, *rpm, 3.0, load);
                    if i >= 4000 {
                        energy += sample * sample;
                    }
                }
                levels.push((energy / 20000.0).sqrt());
            }
            let quietest = levels.iter().copied().fold(f32::INFINITY, f32::min);
            let loudest = levels.iter().copied().fold(0.0, f32::max);
            assert!(quietest > 0.01 && loudest < 0.08, "load {load}: {levels:?}");
            assert!(
                loudest / quietest < 2.5,
                "RPM blend changes level too far: {levels:?}"
            );
        }
    }
}

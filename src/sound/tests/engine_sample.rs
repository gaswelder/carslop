use super::*;
use crate::sound;
use crate::sound::test_support::audio_dir;
use crate::test;

#[test]
fn wav_metadata_chunks_do_not_change_decoded_samples() {
    let directory = tempfile::tempdir().unwrap();
    for entry in std::fs::read_dir(audio_dir()).unwrap() {
        let entry = entry.unwrap();
        std::fs::copy(entry.path(), directory.path().join(entry.file_name())).unwrap();
    }
    let path = directory.path().join("1200-load.wav");
    let mut wav = std::fs::read(&path).unwrap();
    let expected_pcm = wav[44..].to_vec();
    // Editors may insert ancillary chunks, including odd-length payloads
    // followed by a padding byte. Audio does not always start at byte 44.
    wav.splice(12..12, b"JUNK\x03\0\0\0abc\0".iter().copied());
    let size = (wav.len() as u32 - 8).to_le_bytes();
    wav[4..8].copy_from_slice(&size);
    std::fs::write(path, wav).unwrap();
    let samples = sound::engine_sample::load(directory.path()).unwrap();
    assert_eq!(samples.load[0].pcm, expected_pcm);
}

#[test]
fn extensible_pcm_wav_preserves_samples() {
    let pcm = [0x12, 0x34, 0xab, 0x7f];
    let mut wav = test::support::engine_wav(&pcm);
    let mut extension = Vec::new();
    extension.extend_from_slice(&22_u16.to_le_bytes());
    extension.extend_from_slice(&16_u16.to_le_bytes());
    extension.extend_from_slice(&4_u32.to_le_bytes()); // Front-center mono
    extension.extend_from_slice(&[1, 0, 0, 0, 0, 0, 16, 0, 128, 0, 0, 170, 0, 56, 155, 113]);
    wav.splice(36..36, extension);
    let size = (wav.len() as u32 - 8).to_le_bytes();
    wav[4..8].copy_from_slice(&size);
    wav[16..20].copy_from_slice(&40_u32.to_le_bytes());
    wav[20..22].copy_from_slice(&0xfffe_u16.to_le_bytes());
    assert_eq!(wav_pcm(&wav).unwrap(), pcm);
    wav[44] = 3; // IEEE float subformat is not PCM.
    assert!(wav_pcm(&wav).is_err());
}

#[test]
fn truncated_and_misreported_wav_chunks_are_rejected() {
    let wav = test::support::engine_wav(&[0x12, 0x34, 0xab, 0x7f]);
    for length in 0..wav.len() {
        assert!(wav_pcm(&wav[..length]).is_err(), "accepted {length} bytes");
    }
    for offset in [4, 16, 40] {
        let mut invalid = wav.clone();
        invalid[offset..offset + 4].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(wav_pcm(&invalid).is_err());
    }
    let mut invalid = wav.clone();
    invalid[4..8].copy_from_slice(&3_u32.to_le_bytes());
    assert!(wav_pcm(&invalid).is_err());
    for offset in [12, 36] {
        let mut invalid = wav.clone();
        invalid[offset..offset + 4].copy_from_slice(b"JUNK");
        assert!(wav_pcm(&invalid).is_err());
    }
}

#[test]
fn unsupported_wav_formats_and_invalid_lengths_are_rejected() {
    struct Format {
        channels: u16,
        sample_rate: u32,
        bits_per_sample: u16,
    }
    let directory = tempfile::tempdir().unwrap();
    for entry in std::fs::read_dir(audio_dir()).unwrap() {
        let entry = entry.unwrap();
        std::fs::copy(entry.path(), directory.path().join(entry.file_name())).unwrap();
    }
    let path = directory.path().join("1200-load.wav");
    for format in [
        Format {
            channels: 2,
            sample_rate: 48_000,
            bits_per_sample: 16,
        },
        Format {
            channels: 1,
            sample_rate: 44_100,
            bits_per_sample: 16,
        },
        Format {
            channels: 1,
            sample_rate: 48_000,
            bits_per_sample: 8,
        },
    ] {
        let mut wav = test::support::engine_wav(&vec![0; 9600]);
        let frame_bytes = format.channels * format.bits_per_sample / 8;
        wav[22..24].copy_from_slice(&format.channels.to_le_bytes());
        wav[24..28].copy_from_slice(&format.sample_rate.to_le_bytes());
        wav[28..32].copy_from_slice(&(format.sample_rate * u32::from(frame_bytes)).to_le_bytes());
        wav[32..34].copy_from_slice(&frame_bytes.to_le_bytes());
        wav[34..36].copy_from_slice(&format.bits_per_sample.to_le_bytes());
        std::fs::write(&path, wav).unwrap();
        assert!(sound::engine_sample::load(directory.path()).is_err());
    }
    for length in [0, 1, 2, 3, 960_002, 1_025_538] {
        std::fs::write(&path, test::support::engine_wav(&vec![0; length])).unwrap();
        assert!(sound::engine_sample::load(directory.path()).is_err());
    }
}

#[test]
fn interpolation_wraps_from_the_last_sample_to_the_first() {
    let samples = sound::engine_sample::load(audio_dir()).unwrap();
    for clip in samples.load.iter().chain(&samples.coast) {
        let pcm = &clip.pcm;
        assert_eq!(interpolated(pcm, (pcm.len() / 2) as f32), point(pcm, 0));
        let last = pcm.len() / 2 - 1;
        assert!(
            (interpolated(pcm, last as f32 + 0.5) - (point(pcm, last) + point(pcm, 0)) * 0.5).abs()
                < 1e-6
        );
        let mut position = last as f32;
        assert_eq!(loop_sample(pcm, &mut position, 1.0), point(pcm, last));
        assert_eq!(position, 0.0);
        assert_eq!(loop_sample(pcm, &mut position, 1.0), point(pcm, 0));
    }
}

#[test]
fn load_and_coast_blend_independent_recording_lists() {
    let directory = tempfile::tempdir().unwrap();
    sound::test_support::copy_bank(directory.path());
    // Load spans 1200–6000 RPM, while coast has just one 3000 RPM recording.
    for name in ["3000-load.wav", "1200-coast.wav", "6000-coast.wav"] {
        std::fs::remove_file(directory.path().join(name)).unwrap();
    }
    let mut samples = load(directory.path()).unwrap();
    assert_eq!(samples.load.len(), 2);
    assert_eq!(samples.coast.len(), 1);
    samples.load[0].pcm = 8192_i16.to_le_bytes().repeat(4800);
    samples.load[1].pcm = 16384_i16.to_le_bytes().repeat(4800);
    samples.coast[0].pcm = 4096_i16.to_le_bytes().repeat(4800);

    // At 3600 RPM, only load crossfades. Coast uses its single recording.
    let loaded = axis_sample(&samples.load, &mut [0.0; 2], 3600.0, 1.0);
    let coasting = axis_sample(&samples.coast, &mut [0.0], 3600.0, 1.0);
    assert!((loaded - 0.75 / 2.0_f32.sqrt()).abs() < 1e-6);
    assert!((coasting - 0.125).abs() < 1e-6);
    assert_eq!(axis_sample(&samples.load, &mut [0.0; 2], 600.0, 1.0), 0.25);
    assert_eq!(axis_sample(&samples.load, &mut [0.0; 2], 7000.0, 1.0), 0.5);
}

#[test]
fn recording_speed_follows_rpm_even_while_inaudible() {
    let samples = load(audio_dir()).unwrap();
    let clips = &samples.load[..1];
    let mut positions = [1000.0];
    // The 1200 RPM recording plays at twice its original speed at 2400 RPM.
    let first = axis_sample(clips, &mut positions, 2400.0, 1.0);
    let second = axis_sample(clips, &mut positions, 2400.0, 1.0);
    assert_eq!(first, point(&clips[0].pcm, 1000));
    assert_eq!(second, point(&clips[0].pcm, 1002));
    assert_eq!(positions[0], 1004.0);
    assert_eq!(axis_sample(clips, &mut positions, 2400.0, 0.0), 0.0);
    assert_eq!(positions[0], 1006.0);
}

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
    let path = directory.path().join("band-0.wav");
    let mut wav = std::fs::read(&path).unwrap();
    let expected_pcm = wav[44..].to_vec();
    // Editors may insert ancillary chunks, including odd-length payloads
    // followed by a padding byte. Audio does not always start at byte 44.
    wav.splice(12..12, b"JUNK\x03\0\0\0abc\0".iter().copied());
    let size = (wav.len() as u32 - 8).to_le_bytes();
    wav[4..8].copy_from_slice(&size);
    std::fs::write(path, wav).unwrap();
    let samples = sound::engine_sample::load(directory.path()).unwrap();
    assert_eq!(samples.bands[0].load.pcm, expected_pcm);
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
    let path = directory.path().join("band-0.wav");
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
    for length in [0, 100, 1920, 960_002, 1_025_538] {
        std::fs::write(&path, test::support::engine_wav(&vec![0; length])).unwrap();
        assert!(sound::engine_sample::load(directory.path()).is_err());
    }
}

#[test]
fn interpolation_handles_tail_offset_rounding_to_length() {
    let samples = sound::engine_sample::load(audio_dir()).unwrap();
    for clip in samples
        .bands
        .iter()
        .flat_map(|band| [&band.load, &band.coast])
    {
        let pcm = &clip.pcm;
        assert_eq!(
            interpolated(pcm, (pcm.len() / 2) as f32),
            point(pcm, pcm.len() / 2 - 1)
        );
    }
}

//! Controlled recorded-engine fixtures.

use crate::sound::SAMPLE_RATE;
use crate::test;
use std::{path::Path, sync::OnceLock};

pub fn audio_dir() -> &'static Path {
    static BANK: OnceLock<tempfile::TempDir> = OnceLock::new();
    BANK.get_or_init(|| {
        let directory = tempfile::tempdir().unwrap();
        write_bank(directory.path(), 0.30);
        directory
    })
    .path()
}

pub fn write_bank(directory: &Path, amplitude: f32) {
    for rpm in [1200, 3000, 6000] {
        let pulse_hz = rpm as f32 / 20.0;
        let pcm: Vec<u8> = (0..4800)
            .flat_map(|frame| {
                let phase = std::f32::consts::TAU * pulse_hz * frame as f32 / SAMPLE_RATE;
                ((phase.sin() * amplitude * i16::MAX as f32) as i16).to_le_bytes()
            })
            .collect();
        for state in ["load", "coast"] {
            std::fs::write(
                directory.join(format!("{rpm}-{state}.wav")),
                test::support::engine_wav(&pcm),
            )
            .unwrap();
        }
    }
}

pub fn copy_bank(directory: &Path) {
    for entry in std::fs::read_dir(audio_dir()).unwrap() {
        let entry = entry.unwrap();
        if entry.path().is_file() {
            std::fs::copy(entry.path(), directory.join(entry.file_name())).unwrap();
        }
    }
}

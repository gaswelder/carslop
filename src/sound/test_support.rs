//! Controlled recorded-engine fixtures.

use crate::sound::SAMPLE_RATE;
use crate::test;
use std::{path::Path, sync::OnceLock};

pub fn audio_dir() -> &'static Path {
    static BANK: OnceLock<tempfile::TempDir> = OnceLock::new();
    BANK.get_or_init(|| {
        let directory = tempfile::tempdir().unwrap();
        let bands: Vec<_> = [1200.0, 3000.0, 6000.0]
            .into_iter()
            .enumerate()
            .map(|(i, rpm)| {
                let pulse_hz = rpm / 20.0;
                let filename = format!("band-{i}.wav");
                let pcm: Vec<u8> = (0..4800)
                    .flat_map(|frame| {
                        let phase = std::f32::consts::TAU * pulse_hz * frame as f32 / SAMPLE_RATE;
                        ((phase.sin() * 0.30 * i16::MAX as f32) as i16).to_le_bytes()
                    })
                    .collect();
                std::fs::write(
                    directory.path().join(&filename),
                    test::support::engine_wav(&pcm),
                )
                .unwrap();
                let clip = serde_json::json!({"file":filename, "pulse_hz":pulse_hz, "overlap":480});
                serde_json::json!({"rpm":rpm, "load":clip, "coast":clip})
            })
            .collect();
        let manifest =
            serde_json::json!({"version":1, "sample_rate":SAMPLE_RATE as u32, "bands":bands});
        std::fs::write(directory.path().join("engine.json"), manifest.to_string()).unwrap();
        directory
    })
    .path()
}

pub fn copy_bank(directory: &Path) {
    for entry in std::fs::read_dir(audio_dir()).unwrap() {
        let entry = entry.unwrap();
        if entry.path().is_file() {
            std::fs::copy(entry.path(), directory.join(entry.file_name())).unwrap();
        }
    }
}

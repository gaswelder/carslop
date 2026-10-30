//! File-backed Kestrel engine voice, cached once alongside the car artwork.
use crate::ambience::SAMPLE_RATE;
use std::{io, io::Read, path::Path, sync::OnceLock};

const AUDIO_DIR: &str = "data/cars/kestrel/audio";
static SAMPLES: OnceLock<Option<EngineSamples>> = OnceLock::new();

struct EngineSamples {
    idle: Vec<u8>,
    load: Vec<u8>,
    coast: Vec<u8>,
}

impl EngineSamples {
    fn load(directory: &Path) -> io::Result<Self> {
        fn read(directory: &Path, name: &str, overlap: usize) -> io::Result<Vec<u8>> {
            let path = directory.join(name);
            // Raw mono 48 kHz s16le. Bound allocations and reject truncated/short loops.
            let mut bytes = Vec::new();
            std::fs::File::open(&path)?
                .take(9_600_001)
                .read_to_end(&mut bytes)?;
            if bytes.len() % 2 != 0 || bytes.len() / 2 <= overlap * 2 || bytes.len() > 9_600_000 {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("invalid PCM loop: {}", path.display()),
                ));
            }
            Ok(bytes)
        }
        Ok(Self {
            idle: read(directory, "roadster-engine.pcm", OVERLAP)?,
            load: read(directory, "engine-load.pcm", 2068)?,
            coast: read(directory, "engine-coast.pcm", 2064)?,
        })
    }
}
// Measured dominant pulse of this recording, rather than an asserted source RPM.
const SOURCE_HZ: f32 = 42.82;
// Two pulse periods hide the source seam with an overlap, without a silent gap.
const OVERLAP: usize = 2242;

pub struct RecordedEngine {
    samples: Option<&'static EngineSamples>,
    position: f32,
    load_position: f32,
    coast_position: f32,
    low: f32,
}

impl Default for RecordedEngine {
    fn default() -> Self {
        let samples = SAMPLES
            .get_or_init(|| match EngineSamples::load(Path::new(AUDIO_DIR)) {
                Ok(samples) => Some(samples),
                Err(error) => {
                    log::warn!("Cannot load {AUDIO_DIR}: {error}; using procedural Kestrel engine");
                    None
                }
            })
            .as_ref();
        Self {
            samples,
            position: 0.0,
            load_position: 0.0,
            coast_position: 0.0,
            low: 0.0,
        }
    }
}

fn point(pcm: &[u8], index: usize) -> f32 {
    i16::from_le_bytes([pcm[index * 2], pcm[index * 2 + 1]]) as f32 / 32768.0
}

fn interpolated(pcm: &[u8], position: f32) -> f32 {
    // Adding the tail offset can round the last fractional position to len.
    let position = position.clamp(0.0, (pcm.len() / 2 - 1) as f32);
    let i = position as usize;
    let fraction = position.fract();
    let next = (i + 1).min(pcm.len() / 2 - 1);
    point(pcm, i) + (point(pcm, next) - point(pcm, i)) * fraction
}

fn loop_sample(pcm: &[u8], position: &mut f32, ratio: f32, overlap: usize) -> f32 {
    let end = (pcm.len() / 2 - overlap) as f32;
    let mut value = interpolated(pcm, *position);
    if *position < overlap as f32 {
        let t = *position / overlap as f32;
        let blend = t * t * (3.0 - 2.0 * t);
        value = interpolated(pcm, end + *position) * (1.0 - blend) + value * blend;
    }
    *position = (*position + ratio).rem_euclid(end);
    value
}

impl RecordedEngine {
    pub fn available(&self) -> bool {
        self.samples.is_some()
    }

    pub fn sample(&mut self, pulse_hz: f32, load: f32) -> f32 {
        let Some(samples) = self.samples else {
            return 0.0;
        };
        let load = load.clamp(0.0, 1.0);
        let pulse_hz = pulse_hz.max(1.0);
        let idle = loop_sample(
            &samples.idle,
            &mut self.position,
            pulse_hz / SOURCE_HZ,
            OVERLAP,
        );
        // Source filenames specify ~5570 RPM; measured V8 firing pulse is 371 Hz.
        // All voices run continuously so changing load never restarts a loop.
        let on = loop_sample(
            &samples.load,
            &mut self.load_position,
            pulse_hz / 371.33,
            2068,
        );
        let off = loop_sample(
            &samples.coast,
            &mut self.coast_position,
            pulse_hz / 372.07,
            2064,
        );
        let rpm = pulse_hz * 20.0; // target six-cylinder firing rate -> RPM
        let blend = ((rpm - 1150.0) / 1650.0).clamp(0.0, 1.0);
        let blend = blend * blend * (3.0 - 2.0 * blend);
        let running = off * (1.0 - load) + on * load;
        let value = idle * (1.0 - blend) + running * blend * 1.8;
        // Coasting emphasizes the body; throttle opens the upper engine texture.
        let cutoff = 320.0 + load * 2200.0 + blend * 2400.0;
        let alpha = 1.0 - (-std::f32::consts::TAU * cutoff / SAMPLE_RATE).exp();
        self.low += (value - self.low) * alpha;
        (self.low + (value - self.low) * (0.12 + load * 0.55)) * (0.13 + load * 0.10)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_or_malformed_car_audio_is_rejected() {
        let directory = tempfile::tempdir().unwrap();
        assert!(EngineSamples::load(directory.path()).is_err());
        let names = ["roadster-engine.pcm", "engine-load.pcm", "engine-coast.pcm"];
        for name in names {
            std::fs::copy(Path::new(AUDIO_DIR).join(name), directory.path().join(name)).unwrap();
        }
        assert!(EngineSamples::load(directory.path()).is_ok());
        for name in names {
            let path = directory.path().join(name);
            for bytes in [vec![], vec![0; 100], vec![0; 10_001]] {
                std::fs::write(&path, bytes).unwrap();
                assert!(
                    EngineSamples::load(directory.path()).is_err(),
                    "accepted {name}"
                );
            }
            std::fs::copy(Path::new(AUDIO_DIR).join(name), path).unwrap();
        }
    }

    #[test]
    fn unavailable_recording_is_reported_without_panicking() {
        let mut engine = RecordedEngine {
            samples: None,
            ..RecordedEngine::default()
        };
        assert!(!engine.available());
        assert_eq!(engine.sample(300.0, 1.0), 0.0);
    }

    #[test]
    fn interpolation_handles_tail_offset_rounding_to_length() {
        let samples = EngineSamples::load(Path::new(AUDIO_DIR)).unwrap();
        for pcm in [&samples.idle, &samples.load, &samples.coast] {
            assert_eq!(
                interpolated(pcm, (pcm.len() / 2) as f32),
                point(pcm, pcm.len() / 2 - 1)
            );
        }
    }

    #[test]
    fn recorded_layers_survive_seams_shifts_and_load_changes() {
        let mut engine = RecordedEngine::default();
        assert!(engine.available());
        let mut previous = 0.0_f32;
        let mut largest_step = 0.0_f32;
        for i in 0..480_000 {
            let seconds = i as f32 / SAMPLE_RATE;
            let rpm = 900.0 + (seconds % 3.0) / 3.0 * 5600.0;
            let load = if seconds % 4.0 < 2.0 { 1.0 } else { 0.0 };
            let sample = engine.sample(rpm / 20.0, load);
            assert!(sample.is_finite() && sample.abs() < 0.5);
            largest_step = largest_step.max((sample - previous).abs());
            previous = sample;
        }
        assert!(largest_step < 0.2, "discontinuous voice: {largest_step}");
    }
}

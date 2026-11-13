use crate::audio::SAMPLE_RATE;
use std::{
    io,
    io::Read,
    path::{Path, PathBuf},
    sync::{Arc, Mutex, OnceLock},
};

static SAMPLES: OnceLock<Mutex<std::collections::HashMap<PathBuf, Option<Arc<EngineSamples>>>>> =
    OnceLock::new();

struct EngineSamples {
    bands: Vec<RpmBand>,
}

struct RpmBand {
    rpm: f32,
    load: EngineLoop,
    coast: EngineLoop,
}

struct EngineLoop {
    pcm: Vec<u8>,
    pulse_hz: f32,
    overlap: usize,
}

fn invalid(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.into())
}

impl EngineSamples {
    fn load(directory: &Path) -> io::Result<Self> {
        fn read(directory: &Path, value: &serde_json::Value) -> io::Result<EngineLoop> {
            let name = value["file"]
                .as_str()
                .filter(|name| {
                    !name.contains(['/', '\\'])
                        && Path::new(name).components().count() == 1
                        && Path::new(name).extension().is_some_and(|e| e == "pcm")
                })
                .ok_or_else(|| invalid("engine loop must name a local .pcm file"))?;
            let pulse_hz = value["pulse_hz"]
                .as_f64()
                .filter(|v| v.is_finite() && (10.0..=1500.0).contains(v))
                .ok_or_else(|| invalid("invalid engine loop firing frequency"))?
                as f32;
            let overlap = value["overlap"]
                .as_u64()
                .filter(|v| (16..=24000).contains(v))
                .ok_or_else(|| invalid("invalid engine loop overlap"))?
                as usize;
            let path = directory.join(name);
            // Raw mono 48 kHz s16le, at most ten seconds per loop.
            let mut bytes = Vec::new();
            std::fs::File::open(&path)?
                .take(960_001)
                .read_to_end(&mut bytes)?;
            if bytes.len() % 2 != 0 || bytes.len() / 2 <= overlap * 2 || bytes.len() > 960_000 {
                return Err(invalid(format!("invalid PCM loop: {}", path.display())));
            }
            Ok(EngineLoop {
                pcm: bytes,
                pulse_hz,
                overlap,
            })
        }
        let mut manifest = Vec::new();
        std::fs::File::open(directory.join("engine.json"))?
            .take(65_537)
            .read_to_end(&mut manifest)?;
        if manifest.len() > 65_536 {
            return Err(invalid("engine manifest exceeds 64 KiB"));
        }
        let manifest: serde_json::Value =
            serde_json::from_slice(&manifest).map_err(|e| invalid(e.to_string()))?;
        if manifest["version"].as_u64() != Some(1)
            || manifest["sample_rate"].as_u64() != Some(SAMPLE_RATE as u64)
        {
            return Err(invalid("unsupported engine bank version or sample rate"));
        }
        let values = manifest["bands"]
            .as_array()
            .filter(|v| (2..=16).contains(&v.len()))
            .ok_or_else(|| invalid("engine bank requires 2 to 16 RPM bands"))?;
        let mut bands = Vec::with_capacity(values.len());
        let mut previous = 0.0;
        for value in values {
            let rpm = value["rpm"]
                .as_f64()
                .filter(|v| v.is_finite() && (300.0..=20000.0).contains(v))
                .ok_or_else(|| invalid("invalid engine band RPM"))? as f32;
            if rpm <= previous {
                return Err(invalid("engine bands must have strictly increasing RPM"));
            }
            bands.push(RpmBand {
                rpm,
                load: read(directory, &value["load"])?,
                coast: read(directory, &value["coast"])?,
            });
            previous = rpm;
        }
        Ok(Self { bands })
    }

    fn blend(&self, rpm: f32) -> (usize, usize, f32) {
        let upper = self.bands.partition_point(|band| band.rpm < rpm);
        if upper == 0 {
            return (0, 0, 0.0);
        }
        if upper == self.bands.len() {
            return (upper - 1, upper - 1, 0.0);
        }
        let lower = upper - 1;
        let t = (rpm - self.bands[lower].rpm) / (self.bands[upper].rpm - self.bands[lower].rpm);
        (lower, upper, t * t * (3.0 - 2.0 * t))
    }
}

#[derive(Default)]
pub struct RecordedEngine {
    directory: Option<PathBuf>,
    samples: Option<Arc<EngineSamples>>,
    positions: Vec<[f32; 2]>,
    low: f32,
}

impl RecordedEngine {
    pub fn select(&mut self, directory: Option<&Path>) {
        if self.directory.as_deref() == directory {
            return;
        }
        self.directory = directory.map(Path::to_owned);
        self.samples = directory.and_then(|directory| {
            SAMPLES
                .get_or_init(Default::default)
                .lock()
                .unwrap()
                .entry(directory.to_owned())
                .or_insert_with(|| match EngineSamples::load(directory) {
                    Ok(samples) => Some(Arc::new(samples)),
                    Err(error) => {
                        log::warn!(
                            "Cannot load {}: {error}; using procedural engine",
                            directory.display()
                        );
                        None
                    }
                })
                .clone()
        });
        self.positions = vec![[0.0; 2]; self.samples.as_ref().map_or(0, |s| s.bands.len())];
        self.low = 0.0;
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

    pub fn sample(&mut self, rpm: f32, pulses_per_revolution: f32, load: f32) -> f32 {
        let Some(samples) = &self.samples else {
            return 0.0;
        };
        let load = load.clamp(0.0, 1.0);
        let pulse_hz = (rpm * pulses_per_revolution / 60.0).max(1.0);
        let (lower, upper, blend) = samples.blend(rpm);
        let mut value = 0.0;
        for (i, (band, positions)) in samples.bands.iter().zip(&mut self.positions).enumerate() {
            let weight = if lower == upper && i == lower {
                1.0
            } else if i == lower {
                (1.0 - blend).sqrt()
            } else if i == upper {
                blend.sqrt()
            } else {
                0.0
            };
            // Every loop advances even while inaudible. Crossing an RPM boundary
            // or lifting the throttle never restarts its source recording.
            for (state, clip) in [&band.load, &band.coast].into_iter().enumerate() {
                let ratio = pulse_hz / clip.pulse_hz;
                let position = &mut positions[state];
                if weight > 0.0 {
                    let sample = loop_sample(&clip.pcm, position, ratio, clip.overlap);
                    let load_weight = if state == 0 { load } else { 1.0 - load };
                    value += sample * load_weight * weight;
                } else {
                    let end = (clip.pcm.len() / 2 - clip.overlap) as f32;
                    *position = (*position + ratio).rem_euclid(end);
                }
            }
        }
        // Coasting emphasizes the body; throttle opens the upper engine texture.
        let cutoff = 1800.0 + load * 4200.0 + (rpm / 6500.0).clamp(0.0, 1.0) * 2000.0;
        let alpha = 1.0 - (-std::f32::consts::TAU * cutoff / SAMPLE_RATE).exp();
        self.low += (value - self.low) * alpha;
        (self.low + (value - self.low) * (0.22 + load * 0.50)) * (0.13 + load * 0.10)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn audio_dir() -> &'static Path {
        static BANK: OnceLock<tempfile::TempDir> = OnceLock::new();
        BANK.get_or_init(|| {
            let directory = tempfile::tempdir().unwrap();
            let bands: Vec<_> = [1200.0, 3000.0, 6000.0]
                .into_iter()
                .enumerate()
                .map(|(i, rpm)| {
                    let pulse_hz = rpm / 20.0;
                    let filename = format!("band-{i}.pcm");
                    let pcm: Vec<u8> = (0..4800)
                        .flat_map(|frame| {
                            let phase =
                                std::f32::consts::TAU * pulse_hz * frame as f32 / SAMPLE_RATE;
                            ((phase.sin() * 0.30 * i16::MAX as f32) as i16).to_le_bytes()
                        })
                        .collect();
                    std::fs::write(directory.path().join(&filename), pcm).unwrap();
                    let clip =
                        serde_json::json!({"file":filename, "pulse_hz":pulse_hz, "overlap":480});
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
    fn recorded() -> RecordedEngine {
        let mut engine = RecordedEngine::default();
        engine.select(Some(audio_dir()));
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
        assert!(!engine.available());
        engine.select(Some(directory.path()));
        assert!(engine.available());
        let first = engine.sample(3000.0, 3.0, 1.0);
        for _ in 0..100 {
            engine.sample(3000.0, 3.0, 1.0);
        }
        engine.select(None);
        assert!(!engine.available());
        assert_eq!(engine.sample(3000.0, 3.0, 1.0), 0.0);
        engine.select(Some(directory.path()));
        assert_eq!(engine.sample(3000.0, 3.0, 1.0), first);
        engine.select(Some(&directory.path().join("missing")));
        assert!(!engine.available());
    }

    #[test]
    fn missing_or_malformed_car_audio_is_rejected() {
        let directory = tempfile::tempdir().unwrap();
        assert!(EngineSamples::load(directory.path()).is_err());
        let manifest = std::fs::read(audio_dir().join("engine.json")).unwrap();
        std::fs::write(directory.path().join("engine.json"), &manifest).unwrap();
        let manifest: serde_json::Value = serde_json::from_slice(&manifest).unwrap();
        let names: Vec<_> = manifest["bands"]
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
        for name in &names {
            std::fs::copy(audio_dir().join(name), directory.path().join(name)).unwrap();
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
            std::fs::copy(audio_dir().join(name), path).unwrap();
        }
    }

    #[test]
    fn invalid_manifest_is_rejected_before_playback() {
        let directory = tempfile::tempdir().unwrap();
        let original: serde_json::Value =
            serde_json::from_slice(&std::fs::read(audio_dir().join("engine.json")).unwrap())
                .unwrap();
        // Manifest validation cannot depend on the presence of loop assets.
        for entry in std::fs::read_dir(audio_dir()).unwrap() {
            let entry = entry.unwrap();
            if entry.path().extension().is_some_and(|e| e == "pcm") {
                std::fs::copy(entry.path(), directory.path().join(entry.file_name())).unwrap();
            }
        }
        for (pointer, value) in [
            ("/version", serde_json::json!(2)),
            ("/sample_rate", serde_json::json!(44100)),
            ("/bands/1/rpm", serde_json::json!(500)),
            ("/bands/1/rpm", original["bands"][0]["rpm"].clone()),
            ("/bands/0/load/pulse_hz", serde_json::json!(0)),
            ("/bands/0/coast/overlap", serde_json::json!(0)),
            ("/bands/0/load/file", serde_json::json!("../outside.pcm")),
            ("/bands/0/load/file", serde_json::json!("/outside.pcm")),
            ("/bands/0/load/file", serde_json::json!("..\\outside.pcm")),
            ("/bands", serde_json::json!([])),
        ] {
            let mut manifest = original.clone();
            *manifest.pointer_mut(pointer).unwrap() = value;
            std::fs::write(directory.path().join("engine.json"), manifest.to_string()).unwrap();
            assert!(
                EngineSamples::load(directory.path()).is_err(),
                "accepted {pointer}"
            );
        }
        for bytes in [vec![b' '; 65_537], b"{invalid".to_vec()] {
            std::fs::write(directory.path().join("engine.json"), bytes).unwrap();
            assert!(EngineSamples::load(directory.path()).is_err());
        }
    }

    #[test]
    fn interpolation_handles_tail_offset_rounding_to_length() {
        let samples = EngineSamples::load(audio_dir()).unwrap();
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

    #[test]
    fn recorded_layers_survive_seams_shifts_and_load_changes() {
        let mut engine = recorded();
        assert!(engine.available());
        let mut previous = 0.0_f32;
        let mut largest_step = 0.0_f32;
        for i in 0..480_000 {
            let seconds = i as f32 / SAMPLE_RATE;
            let rpm = 900.0 + (seconds % 3.0) / 3.0 * 5600.0;
            let load = if seconds % 4.0 < 2.0 { 1.0 } else { 0.0 };
            let sample = engine.sample(rpm, 3.0, load);
            assert!(sample.is_finite() && sample.abs() < 0.5);
            largest_step = largest_step.max((sample - previous).abs());
            previous = sample;
        }
        assert!(largest_step < 0.2, "discontinuous voice: {largest_step}");
    }

    #[test]
    fn rpm_sweep_keeps_level_across_recording_boundaries() {
        let samples = EngineSamples::load(audio_dir()).unwrap();
        let mut points = vec![900.0, 6500.0];
        for bands in samples.bands.windows(2) {
            points.extend([bands[0].rpm, (bands[0].rpm + bands[1].rpm) * 0.5]);
        }
        points.push(samples.bands.last().unwrap().rpm);
        for load in [0.0, 1.0] {
            let mut levels = Vec::new();
            for rpm in &points {
                let mut engine = recorded();
                let mut energy = 0.0;
                for i in 0..24000 {
                    let sample = engine.sample(*rpm, 3.0, load);
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

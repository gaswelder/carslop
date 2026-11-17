use crate::sound::SAMPLE_RATE;
use std::{
    io,
    io::Read,
    path::{Path, PathBuf},
    sync::{Arc, Mutex, OnceLock},
};

static SAMPLES: OnceLock<Mutex<std::collections::HashMap<PathBuf, Option<Arc<EngineSamples>>>>> =
    OnceLock::new();

struct EngineSamples {
    load: Vec<EngineLoop>,
    coast: Vec<EngineLoop>,
}

struct EngineLoop {
    pcm: Vec<u8>,
    rpm: f32,
}

fn invalid(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.into())
}

/// Extract mono 48 kHz signed 16-bit PCM, skipping padded RIFF metadata chunks.
fn wav_pcm(bytes: &[u8]) -> io::Result<&[u8]> {
    if bytes.len() < 12 || &bytes[..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return Err(invalid("expected RIFF/WAVE audio"));
    }
    let riff_size = u32::from_le_bytes(bytes[4..8].try_into().unwrap()) as usize;
    let end = riff_size
        .checked_add(8)
        .ok_or_else(|| invalid("invalid RIFF length"))?;
    let mut chunks = bytes
        .get(12..end)
        .ok_or_else(|| invalid("truncated RIFF/WAVE"))?;
    let mut format_found = false;
    let mut pcm = None;
    while !chunks.is_empty() {
        let header = chunks
            .get(..8)
            .ok_or_else(|| invalid("truncated WAV chunk header"))?;
        let size = u32::from_le_bytes(header[4..8].try_into().unwrap()) as usize;
        chunks = &chunks[8..];
        let payload = chunks
            .get(..size)
            .ok_or_else(|| invalid("truncated WAV chunk"))?;
        match &header[..4] {
            b"fmt " => {
                if format_found || payload.len() < 16 {
                    return Err(invalid("invalid WAV format chunk"));
                }
                let encoding = u16::from_le_bytes(payload[..2].try_into().unwrap());
                let channels = u16::from_le_bytes(payload[2..4].try_into().unwrap());
                let rate = u32::from_le_bytes(payload[4..8].try_into().unwrap());
                let byte_rate = u32::from_le_bytes(payload[8..12].try_into().unwrap());
                let frame_bytes = u16::from_le_bytes(payload[12..14].try_into().unwrap());
                let bits = u16::from_le_bytes(payload[14..16].try_into().unwrap());
                // WAVE_FORMAT_EXTENSIBLE can also describe ordinary PCM.
                let extensible_pcm = encoding == 0xfffe
                    && payload.len() >= 40
                    && u16::from_le_bytes(payload[16..18].try_into().unwrap()) >= 22
                    && u16::from_le_bytes(payload[18..20].try_into().unwrap()) == 16
                    && payload[24..40]
                        == [1, 0, 0, 0, 0, 0, 16, 0, 128, 0, 0, 170, 0, 56, 155, 113];
                if !(encoding == 1 || extensible_pcm)
                    || channels != 1
                    || rate != SAMPLE_RATE as u32
                    || bits != 16
                    || frame_bytes != 2
                    || byte_rate != rate * 2
                {
                    return Err(invalid("engine WAV must be mono 48 kHz signed 16-bit PCM"));
                }
                format_found = true;
            }
            b"data" => {
                if pcm.replace(payload).is_some() {
                    return Err(invalid("multiple WAV data chunks are unsupported"));
                }
            }
            _ => {}
        }
        let padded_size = size
            .checked_add(size % 2)
            .ok_or_else(|| invalid("invalid WAV chunk length"))?;
        chunks = chunks
            .get(padded_size..)
            .ok_or_else(|| invalid("missing WAV chunk padding"))?;
    }
    if !format_found {
        return Err(invalid("missing WAV format chunk"));
    }
    pcm.ok_or_else(|| invalid("missing WAV data chunk"))
}

fn load(directory: &Path) -> io::Result<EngineSamples> {
    fn read(path: &Path, rpm: f32) -> io::Result<EngineLoop> {
        // At most ten seconds of mono 48 kHz s16le plus 64 KiB of WAV
        // headers/metadata. The WAV reader handles editor RIFF chunks.
        let mut bytes = Vec::new();
        std::fs::File::open(path)?
            .take(1_025_537)
            .read_to_end(&mut bytes)?;
        if bytes.len() > 1_025_536 {
            return Err(invalid("engine WAV exceeds file size limit"));
        }
        let pcm =
            wav_pcm(&bytes).map_err(|error| invalid(format!("{}: {error}", path.display())))?;
        if pcm.len() % 2 != 0 || pcm.len() < 4 || pcm.len() > 960_000 {
            return Err(invalid(format!(
                "invalid WAV loop length: {}",
                path.display()
            )));
        }
        Ok(EngineLoop {
            pcm: pcm.to_vec(),
            rpm,
        })
    }
    let mut samples = EngineSamples {
        load: Vec::new(),
        coast: Vec::new(),
    };
    for entry in std::fs::read_dir(directory)? {
        let path = entry?.path();
        if path.extension().is_none_or(|extension| extension != "wav") {
            continue;
        }
        let stem = path
            .file_stem()
            .and_then(|stem| stem.to_str())
            .ok_or_else(|| invalid("engine WAV filename must be valid UTF-8"))?;
        let (rpm, state) = stem
            .split_once('-')
            .ok_or_else(|| invalid("engine WAV must be named <rpm>-load.wav or <rpm>-coast.wav"))?;
        let rpm = rpm
            .parse::<u32>()
            .ok()
            .filter(|rpm| (300..=20000).contains(rpm))
            .ok_or_else(|| invalid("invalid engine recording RPM in filename"))?
            as f32;
        let clips = match state {
            "load" => &mut samples.load,
            "coast" => &mut samples.coast,
            _ => return Err(invalid("engine WAV must end in -load.wav or -coast.wav")),
        };
        if clips.len() == 16 {
            return Err(invalid("engine axis supports at most 16 recordings"));
        }
        clips.push(read(&path, rpm)?);
    }
    for clips in [&mut samples.load, &mut samples.coast] {
        if clips.is_empty() {
            return Err(invalid("engine bank requires load and coast recordings"));
        }
        clips.sort_by(|a, b| a.rpm.total_cmp(&b.rpm));
        if clips.windows(2).any(|pair| pair[0].rpm == pair[1].rpm) {
            return Err(invalid("duplicate engine recording RPM"));
        }
    }
    Ok(samples)
}

struct Blend {
    lower: usize,
    upper: usize,
    amount: f32,
}

fn blend(clips: &[EngineLoop], rpm: f32) -> Blend {
    let upper = clips.partition_point(|clip| clip.rpm < rpm);
    if upper == 0 {
        return Blend {
            lower: 0,
            upper: 0,
            amount: 0.0,
        };
    }
    if upper == clips.len() {
        return Blend {
            lower: upper - 1,
            upper: upper - 1,
            amount: 0.0,
        };
    }
    let lower = upper - 1;
    let t = (rpm - clips[lower].rpm) / (clips[upper].rpm - clips[lower].rpm);
    Blend {
        lower,
        upper,
        amount: t * t * (3.0 - 2.0 * t),
    }
}

#[derive(Default)]
pub struct RecordedEngine {
    directory: Option<PathBuf>,
    samples: Option<Arc<EngineSamples>>,
    load_positions: Vec<f32>,
    coast_positions: Vec<f32>,
    low: f32,
}

pub fn select(recorded_engine: &mut RecordedEngine, directory: Option<&Path>) {
    if recorded_engine.directory.as_deref() == directory {
        return;
    }
    recorded_engine.directory = directory.map(Path::to_owned);
    recorded_engine.samples = directory.and_then(|directory| {
        SAMPLES
            .get_or_init(Default::default)
            .lock()
            .unwrap()
            .entry(directory.to_owned())
            .or_insert_with(|| match load(directory) {
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
    recorded_engine.load_positions =
        vec![0.0; recorded_engine.samples.as_ref().map_or(0, |s| s.load.len())];
    recorded_engine.coast_positions = vec![
        0.0;
        recorded_engine
            .samples
            .as_ref()
            .map_or(0, |s| s.coast.len())
    ];
    recorded_engine.low = 0.0;
}

fn point(pcm: &[u8], index: usize) -> f32 {
    i16::from_le_bytes([pcm[index * 2], pcm[index * 2 + 1]]) as f32 / 32768.0
}

fn interpolated(pcm: &[u8], position: f32) -> f32 {
    let position = position.rem_euclid((pcm.len() / 2) as f32);
    let i = position as usize;
    let fraction = position.fract();
    let next = (i + 1) % (pcm.len() / 2);
    point(pcm, i) + (point(pcm, next) - point(pcm, i)) * fraction
}

fn loop_sample(pcm: &[u8], position: &mut f32, ratio: f32) -> f32 {
    let value = interpolated(pcm, *position);
    *position = (*position + ratio).rem_euclid((pcm.len() / 2) as f32);
    value
}

pub fn available(recorded_engine: &RecordedEngine) -> bool {
    recorded_engine.samples.is_some()
}

fn axis_sample(clips: &[EngineLoop], positions: &mut [f32], rpm: f32, gain: f32) -> f32 {
    let blend = blend(clips, rpm);
    let mut value = 0.0;
    for (i, (clip, position)) in clips.iter().zip(positions).enumerate() {
        let weight = if blend.lower == blend.upper && i == blend.lower {
            1.0
        } else if i == blend.lower {
            (1.0 - blend.amount).sqrt()
        } else if i == blend.upper {
            blend.amount.sqrt()
        } else {
            0.0
        } * gain;
        let ratio = rpm / clip.rpm;
        // Every recording advances while inaudible, so RPM and load changes
        // never restart a loop.
        if weight > 0.0 {
            value += loop_sample(&clip.pcm, position, ratio) * weight;
        } else {
            let end = (clip.pcm.len() / 2) as f32;
            *position = (*position + ratio).rem_euclid(end);
        }
    }
    value
}

pub fn sample(recorded_engine: &mut RecordedEngine, rpm: f32, load: f32) -> f32 {
    let Some(samples) = &recorded_engine.samples else {
        return 0.0;
    };
    let load = load.clamp(0.0, 1.0);
    let rpm = rpm.max(1.0);
    let value = axis_sample(
        &samples.load,
        &mut recorded_engine.load_positions,
        rpm,
        load,
    ) + axis_sample(
        &samples.coast,
        &mut recorded_engine.coast_positions,
        rpm,
        1.0 - load,
    );
    // Coasting emphasizes the body; throttle opens the upper engine texture.
    let cutoff = 1800.0 + load * 4200.0 + (rpm / 6500.0).clamp(0.0, 1.0) * 2000.0;
    let alpha = 1.0 - (-std::f32::consts::TAU * cutoff / SAMPLE_RATE).exp();
    recorded_engine.low += (value - recorded_engine.low) * alpha;
    (recorded_engine.low + (value - recorded_engine.low) * (0.22 + load * 0.50))
        * (0.13 + load * 0.10)
}

#[cfg(test)]
#[path = "tests/engine_sample.rs"]
mod tests;

use crate::physics;
use crate::render;
use crate::sound;
use crate::sound::SAMPLE_RATE;
use sdl3::audio::{AudioFormat, AudioSpec, AudioStreamOwner};
const QUEUE_SAMPLES: usize = 2400;

pub struct EngineAudio {
    stream: AudioStreamOwner,
    _subsystem: sdl3::AudioSubsystem,
    synth: sound::synth::Synth,
    spatial: sound::spatial::SpatialAudio,
    pub muted: bool,
}

// Preserve surf/gust phase across recovery/restart; only clear vehicle transients.
pub fn reset_feedback(engine_audio: &mut EngineAudio) {
    engine_audio.synth = sound::synth::Synth::default();
    engine_audio.spatial = sound::spatial::SpatialAudio::default();
}

pub fn new(sdl: &sdl3::Sdl) -> Result<EngineAudio, String> {
    let audio = sdl.audio().map_err(|e| e.to_string())?;
    let spec = AudioSpec {
        freq: Some(SAMPLE_RATE as i32),
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
    Ok(EngineAudio {
        stream,
        _subsystem: audio,
        synth: sound::synth::Synth::default(),
        spatial: sound::spatial::SpatialAudio::default(),
        muted: false,
    })
}

pub fn update(
    engine_audio: &mut EngineAudio,
    car: &physics::car::Vehicle,
    listener: render::camera::Pose,
    throttle: bool,
    active: bool,
    impact: f32,
) -> Result<(), String> {
    let queued = engine_audio
        .stream
        .queued_bytes()
        .map_err(|e| e.to_string())?
        .max(0) as usize
        / (2 * std::mem::size_of::<f32>());
    let count = QUEUE_SAMPLES.saturating_sub(queued);
    let enabled = active && !engine_audio.muted;
    if enabled {
        sound::synth::hit(&mut engine_audio.synth, impact);
    } else {
        sound::synth::suppress_impacts(&mut engine_audio.synth);
    }
    let voice = sound::synth::Voice {
        turbo_target: car.turbo_spool,
        road_speed: car.velocity.x.hypot(car.velocity.z),
        frequency: engine_frequency(car.engine_rpm),
        load: if throttle {
            car.engine_load.max(0.15)
        } else {
            car.engine_load
        },
        scrub: car.tire_scrub,
        tuning: &car.spec.audio,
        enabled,
    };
    let placement = sound::spatial::placement(car.position, listener);
    let samples: Vec<f32> = (0..count)
        .flat_map(|_| {
            let driving = sound::spatial::sample(
                &mut engine_audio.spatial,
                sound::synth::sample(&mut engine_audio.synth, voice),
                placement,
            );
            driving.map(output_sample)
        })
        .collect();
    if !samples.is_empty() {
        engine_audio
            .stream
            .put_data_f32(&samples)
            .map_err(|e| e.to_string())?;
    }
    Ok(())
}

fn output_sample(driving: f32) -> f32 {
    // Preserve driving gain and output headroom.
    (driving * 0.92).clamp(-0.3, 0.3)
}

fn engine_frequency(rpm: f32) -> f32 {
    rpm.max(physics::car::IDLE_RPM) / 30.0
}

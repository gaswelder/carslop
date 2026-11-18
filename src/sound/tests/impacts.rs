use super::*;

#[test]
fn assets_match_the_embedded_pcm_format() {
    for clip in [METAL, CRASH] {
        assert_eq!(&clip[..4], b"RIFF");
        assert_eq!(&clip[8..16], b"WAVEfmt ");
        assert_eq!(u32::from_le_bytes(clip[16..20].try_into().unwrap()), 16);
        assert_eq!(&clip[20..24], &[1, 0, 1, 0]);
        assert_eq!(u32::from_le_bytes(clip[24..28].try_into().unwrap()), 48000);
        assert_eq!(&clip[32..40], &[2, 0, 16, 0, b'd', b'a', b't', b'a']);
        assert_eq!(
            u32::from_le_bytes(clip[40..44].try_into().unwrap()) as usize,
            clip.len() - WAV_HEADER
        );
        assert_eq!(point(clip, 0), 0.0);
        assert_eq!(point(clip, (clip.len() - WAV_HEADER) / 2 - 1), 0.0);
    }
}

#[test]
fn later_collision_preserves_the_first_tail_and_both_finish_without_looping() {
    let mut first = Impacts::default();
    let mut second = Impacts::default();
    let mut combined = Impacts::default();
    hit(&mut first, 12.0);
    hit(&mut combined, 12.0);
    for _ in 0..4800 {
        assert_eq!(sample(&mut first), sample(&mut combined));
    }
    hit(&mut second, 4.0);
    hit(&mut combined, 4.0);
    let mut overlap_energy = 0.0;
    for _ in 0..192000 {
        let original = sample(&mut first);
        let added = sample(&mut second);
        let output = sample(&mut combined);
        assert!((output - original - added).abs() < 0.000001);
        overlap_energy += original.abs() * added.abs();
    }
    assert!(overlap_energy > 0.1);
    assert_eq!(sample(&mut combined), 0.0);
    hit(&mut combined, 12.0);
    let mut restart_energy = 0.0;
    for _ in 0..4800 {
        restart_energy += sample(&mut combined).powi(2);
    }
    assert!(restart_energy > 1.0);
    suppress(&mut combined);
    assert_eq!(sample(&mut combined), 0.0);
}

struct Measurement {
    peak: f32,
    energy_duration: usize,
}

fn measure(speed: f32) -> Measurement {
    let mut impacts = Impacts::default();
    hit(&mut impacts, speed);
    let samples: Vec<_> = (0..192000).map(|_| sample(&mut impacts)).collect();
    let energy: f32 = samples.iter().map(|value| value * value).sum();
    assert!(energy > 0.0);
    let mut accumulated = 0.0;
    let energy_duration = samples
        .iter()
        .position(|value| {
            accumulated += value * value;
            accumulated >= energy * 0.9
        })
        .unwrap();
    Measurement {
        peak: samples.iter().map(|value| value.abs()).fold(0.0, f32::max),
        energy_duration,
    }
}

#[test]
fn fast_crashes_get_louder_and_shorter_beyond_the_old_speed_limit() {
    let moderate = measure(10.5);
    let fast = measure(20.0);
    let severe = measure(35.5);
    assert!(fast.peak > moderate.peak * 1.1);
    assert!(severe.peak > fast.peak * 1.1);
    assert!(fast.energy_duration < moderate.energy_duration / 2);
    assert!(severe.energy_duration < fast.energy_duration);
    assert!(severe.energy_duration < SAMPLE_RATE as usize / 10);
}

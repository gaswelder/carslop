use super::*;

struct Measurement {
    energy: f32,
    difference_energy: f32,
}

fn measure(material: Material) -> Measurement {
    let mut tires = TireSound::default();
    let mut result = Measurement {
        energy: 0.,
        difference_energy: 0.,
    };
    let mut previous = 0.;
    for i in 0..48_000 {
        let output = sample(&mut tires, 1., 30., material);
        assert!(output.is_finite() && output.abs() < 0.2);
        if i > 12_000 {
            result.energy += output * output;
            result.difference_energy += (output - previous).powi(2);
        }
        previous = output;
    }
    result
}

#[test]
fn grass_is_softer_and_dirt_is_grittier_than_rubber_squeal() {
    let asphalt = measure(Material::Asphalt);
    let grass = measure(Material::Grass);
    let dirt = measure(Material::BareGround);
    assert!(grass.energy > 0.1 && grass.energy < asphalt.energy * 0.6);
    assert!(dirt.energy > grass.energy * 2.);
    assert!(dirt.difference_energy / dirt.energy > grass.difference_energy / grass.energy * 2.);
}

#[test]
fn every_surface_fades_and_transitions_without_clicks() {
    let mut tires = TireSound::default();
    let mut previous = 0.;
    for material in [
        Material::Asphalt,
        Material::Grass,
        Material::BareGround,
        Material::Paving,
        Material::Concrete,
        Material::Water,
    ] {
        for i in 0..24_000 {
            let output = sample(&mut tires, 1., 30., material);
            assert!(output.is_finite() && output.abs() < 0.2);
            if i == 0 {
                assert!((output - previous).abs() < 0.05);
            }
            previous = output;
        }
    }
    for _ in 0..48_000 {
        previous = sample(&mut tires, 0., 30., Material::Grass);
    }
    assert!(previous.abs() < 0.00001);
    let mut stopped = TireSound::default();
    for _ in 0..1000 {
        assert_eq!(sample(&mut stopped, 1., 0., Material::BareGround), 0.);
    }
}

#[test]
fn embedded_recording_has_expected_format_and_sustains_across_loop_boundaries() {
    // Playback reads a canonical PCM WAV directly; catch accidental replacement
    // with an editor export containing extra metadata or a different encoding.
    assert_eq!(&SQUEAL[..4], b"RIFF");
    assert_eq!(&SQUEAL[8..16], b"WAVEfmt ");
    assert_eq!(u32::from_le_bytes(SQUEAL[16..20].try_into().unwrap()), 16);
    assert_eq!(&SQUEAL[20..24], &[1, 0, 1, 0]); // PCM, mono
    assert_eq!(
        u32::from_le_bytes(SQUEAL[24..28].try_into().unwrap()),
        48_000
    );
    assert_eq!(&SQUEAL[32..36], &[2, 0, 16, 0]); // block size, bit depth
    assert_eq!(&SQUEAL[36..40], b"data");
    assert_eq!(
        u32::from_le_bytes(SQUEAL[40..44].try_into().unwrap()) as usize,
        SQUEAL.len() - WAV_HEADER
    );
    for speed in [10., 30., 60.] {
        let mut tires = TireSound::default();
        let mut previous = 0.;
        // Several complete wraps, including interpolation across the seam.
        for _ in 0..SAMPLE_RATE as usize * 5 {
            let before = tires.position;
            let output = sample(&mut tires, 1., speed, Material::Asphalt);
            assert!(output.is_finite() && output.abs() < 0.2);
            if tires.position < before {
                assert!((output - previous).abs() < 0.05, "loop seam clicks");
            }
            previous = output;
        }
        assert!(tires.gain > 0.99);
    }
}

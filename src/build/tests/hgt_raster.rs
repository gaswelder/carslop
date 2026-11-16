use super::*;
use crate::build;
use crate::test;
use crate::util;
#[test]
fn hgt_endian_orientation_interpolation_and_voids() {
    let temp = test::support::new();
    let path = temp.directory.join("N35E033.hgt");
    let mut samples: Vec<i16> = (0..9)
        .flat_map(|r| (0..9).map(move |c| 100 + r * 2 + c))
        .collect();
    let save = |values: &[i16]| {
        fs::write(
            &path,
            values
                .iter()
                .flat_map(|n| n.to_be_bytes())
                .collect::<Vec<_>>(),
        )
        .unwrap()
    };
    save(&samples);
    let dem = build::hgt::read(&path).unwrap();
    for (p, expected) in [
        ([35.5, 33.5], 112.0),
        ([35.625, 33.5], 110.0),
        ([35.5, 33.625], 113.0),
        ([35.5625, 33.5625], 111.5),
    ] {
        assert!(
            (build::hgt::sample(&(dem), util::geometry::new(p[0], p[1])).unwrap() - expected).abs()
                < 1e-10
        );
    }
    assert!(build::hgt::sample(&(dem), util::geometry::new(36.0, 33.0)).is_err());
    samples[40] = -32768;
    save(&samples);
    assert!(build::hgt::sample(
        &(build::hgt::read(&path).unwrap()),
        util::geometry::new(35.5, 33.5)
    )
    .is_err());
}

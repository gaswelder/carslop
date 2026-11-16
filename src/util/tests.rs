//! Tests through the group's production interfaces.

mod geometry {
    use crate::model;
    use crate::test;
    use crate::util;
    use crate::util::geometry::*;

    #[test]
    fn geographic_positions_round_trip_through_local_metres() {
        let origin = util::geometry::new(35.1654321, 33.3256789);
        let mut compiled = model::source_location::empty::<f64>();
        compiled.origin = Some(origin);
        let mut runtime = test::support::empty_world();
        runtime.origin = Some(origin);
        for target in [
            origin,
            util::geometry::new(35.17, 33.34),
            util::geometry::new(35.15, 33.31),
        ] {
            let local = model::source_location::metres(&compiled, target);
            let result = util::geometry::geographic_position(
                &runtime,
                XYZ {
                    x: local.x as f32,
                    y: 123.0,
                    z: local.z as f32,
                },
            )
            .unwrap();
            assert!((result.lat - target.lat).abs() < 1e-8);
            assert!((result.lon - target.lon).abs() < 1e-8);
        }
    }
}

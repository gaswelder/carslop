use crate::model::world::HeightField;
use crate::storage::elevation;
use crate::util::geometry::XZ;

#[test]
fn sampled_grid_preserves_datum_origin_and_row_order() {
    let directory = tempfile::tempdir().unwrap();
    elevation::write_sampled(
        directory.path(),
        elevation::Grid {
            columns: 2,
            rows: 2,
            origin: XZ { x: -40., z: 20. },
            step: 20.,
            datum: 123.4567,
        },
        |column, row| Ok(row as f64 * 10. + column as f64 + 0.12345),
    )
    .unwrap();
    let elevation = elevation::read(directory.path()).unwrap();
    assert_eq!(elevation.datum, 123.457);
    assert_eq!(elevation.field.origin, XZ { x: -40., z: 20. });
    assert_eq!(elevation.field.step, 20.);
    assert_eq!(elevation.field.values, [0.123, 1.123, 10.123, 11.123]);
}

#[test]
fn authored_grid_round_trip_preserves_float_precision() {
    let directory = tempfile::tempdir().unwrap();
    let field = HeightField {
        columns: 2,
        rows: 2,
        origin: XZ {
            x: -2.1234567,
            z: 3.1234567,
        },
        step: 1.1234567,
        values: vec![0.12345679, -0.9876543, 5.4321985, -6.7891235],
    };
    elevation::write(directory.path(), &field).unwrap();
    let loaded = elevation::read(directory.path()).unwrap();
    assert_eq!(loaded.datum, 0.);
    assert_eq!(loaded.field.origin, field.origin);
    assert_eq!(loaded.field.step, field.step);
    assert_eq!(loaded.field.values, field.values);
}

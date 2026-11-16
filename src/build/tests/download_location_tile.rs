use super::*;
#[test]
fn elevation_downloads_cover_both_sides_of_degree_edges() {
    assert_eq!(
        elevation_tiles(BoundingBox {
            min_lat: 34.7,
            min_lon: 32.99,
            max_lat: 34.8,
            max_lon: 33.01
        }),
        vec!["N34E032", "N34E033"]
    );
    assert_eq!(
        elevation_tiles(BoundingBox {
            min_lat: 34.998,
            min_lon: 33.2,
            max_lat: 35.001,
            max_lon: 33.3
        }),
        vec!["N34E033", "N35E033"]
    );
}

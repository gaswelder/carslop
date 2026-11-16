use super::*;
use crate::build;

#[test]
fn extract_selection_checks_full_geometry_and_chooses_smallest_cover() {
    let index: RegionalIndex = serde_json::from_value(serde_json::json!({
        "features": [
            {"properties": {"urls": {"pbf": "large"}}, "geometry": {
                "type": "MultiPolygon", "coordinates": [[[[0,0],[10,0],[10,10],[0,10],[0,0]]]]}},
            {"properties": {"urls": {"pbf": "partial"}}, "geometry": {
                "type": "Polygon", "coordinates": [[[1,1],[3,1],[3,3],[1,3],[1,1]]]}},
            {"properties": {"urls": {"pbf": "with-hole"}}, "geometry": {
                "type": "Polygon", "coordinates": [
                    [[1,1],[5,1],[5,5],[1,5],[1,1]],
                    [[2.5,2.5],[2.5,3.5],[3.5,3.5],[3.5,2.5],[2.5,2.5]]]}},
            {"properties": {"urls": {"pbf": "smallest-cover"}}, "geometry": {
                "type": "Polygon", "coordinates": [[[1,1],[6,1],[6,6],[1,6],[1,1]]]}}
        ]
    }))
    .unwrap();
    let boundary = build::osm::OSMArea {
        geometry: geo::MultiPolygon(vec![geo::Rect::new(
            geo::Coord { x: 2., y: 2. },
            geo::Coord { x: 4., y: 4. },
        )
        .to_polygon()]),
    };
    assert_eq!(
        build::download_location::covering_extract(&(index), &boundary).unwrap(),
        "smallest-cover"
    );
    let boundary = build::osm::OSMArea {
        geometry: geo::MultiPolygon(vec![geo::Rect::new(
            geo::Coord { x: 9., y: 9. },
            geo::Coord { x: 11., y: 11. },
        )
        .to_polygon()]),
    };
    assert!(build::download_location::covering_extract(&(index), &boundary).is_err());
}

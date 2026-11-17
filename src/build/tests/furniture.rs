use super::*;
use crate::build;
use crate::model;
use crate::model::source_location::SourceRoad;
use crate::model::world::FurnitureKind;
use crate::test;
use crate::util;
use crate::util::geometry::BoundingBoxXZ;

fn element(id: i64, p: XZ, tags: &[(&str, &str)]) -> OSMElement {
    let mut element = test::support::osm_element(id, tags);
    element.coord = Some(util::geometry::new(
        -p.z.to_degrees() / 6371000.,
        p.x.to_degrees() / 6371000.,
    ));
    element
}

fn junction() -> (OSMMap, SourceLocation<f64>, Surfaces) {
    let points = [
        XZ { x: -40., z: 0. },
        XZ { x: 0., z: 0. },
        XZ { x: 0., z: -40. },
    ];
    let mut source = OSMMap {
        nodes: BTreeMap::new(),
        ways: BTreeMap::new(),
        relations: vec![],
        bounds: None,
    };
    for (i, p) in points.into_iter().enumerate() {
        source
            .nodes
            .insert(i as i64 + 1, element(i as i64 + 1, p, &[]));
    }
    test::support::set_osm_tags(
        source.nodes.get_mut(&2).unwrap(),
        &[
            ("highway", "traffic_signals"),
            ("traffic_signals:direction", "both"),
        ],
    );
    for (id, nodes) in [(101, vec![1, 2]), (102, vec![3, 2])] {
        source.ways.insert(id, {
            let mut way = OSMElement::default();
            way.id = id;
            way.nodes = nodes;
            way
        });
    }
    let mut map = model::source_location::empty();
    map.origin = Some(util::geometry::new(0., 0.));
    map.bounding_box = BoundingBoxXZ {
        min: XZ { x: -50., z: -50. },
        max: XZ { x: 50., z: 50. },
    };
    map.road_nodes = points
        .into_iter()
        .map(|p| util::geometry::at_height(p, 0.))
        .collect();
    map.road_lines = vec![
        SourceRoad {
            osm_way_id: Some(101),
            ..model::source_location::new([0, 1], 8.)
        },
        SourceRoad {
            osm_way_id: Some(102),
            ..model::source_location::new([2, 1], 6.5)
        },
    ];
    let output = tempfile::tempdir().unwrap();
    let mut surfaces = build::surfaces::build_streets(&map, &output.path(), None)
        .unwrap()
        .surfaces;
    surfaces.paths = vec![vec![points[0], points[1]], vec![points[2], points[1]]];
    surfaces.adjacency = vec![vec![0], vec![0, 1], vec![1]];
    surfaces.obstacles = MultiPolygon(vec![]);
    (source, map, surfaces)
}

#[test]
fn crossing_signal_tags_override_the_legacy_crossing_type() {
    let (mut source, map, surfaces) = junction();
    test::support::set_osm_tags(
        source.nodes.get_mut(&2).unwrap(),
        &[("highway", "crossing"), ("crossing:signals", "yes")],
    );
    let placed = place_furniture(&source, &map, &surfaces);
    assert_eq!(placed.len(), 2);
    assert!(placed
        .iter()
        .all(|feature| feature.kind == FurnitureKind::Signal));
    test::support::set_osm_tags(
        source.nodes.get_mut(&2).unwrap(),
        &[
            ("highway", "crossing"),
            ("crossing", "traffic_signals"),
            ("crossing:signals", "no"),
        ],
    );
    assert!(place_furniture(&source, &map, &surfaces).is_empty());
}

#[test]
fn placement_without_files_preserves_left_curbs_and_signal_phase_groups() {
    let (source, map, surfaces) = junction();
    let placed = place_furniture(&source, &map, &surfaces);
    assert_eq!(placed.len(), 2);
    assert!(placed
        .iter()
        .all(|p| p.kind == FurnitureKind::Signal && p.source_id == 2));
    let horizontal = placed.iter().find(|p| p.travel.x > 0.9).unwrap();
    let vertical = placed.iter().find(|p| p.travel.z > 0.9).unwrap();
    assert!(horizontal.position.z < -4.);
    assert!(vertical.position.x > 3.25);
    assert_eq!(horizontal.phase, 2);
    assert_eq!(vertical.phase, 27);
    // A wrong-way control must not be placed on either incoming one-way road.
    let mut wrong_way = map;
    for road in &mut wrong_way.road_lines {
        road.direction = -1;
    }
    assert!(place_furniture(&source, &wrong_way, &surfaces).is_empty());
}

#[test]
fn placement_filters_ambiguous_stops_invalid_speeds_and_blocked_poles() {
    let (mut source, map, mut surfaces) = junction();
    let node = source.nodes.get_mut(&2).unwrap();
    test::support::set_osm_tags(node, &[("highway", "stop")]);
    assert!(place_furniture(&source, &map, &surfaces).is_empty());
    test::support::set_osm_tags(
        source.nodes.get_mut(&2).unwrap(),
        &[("highway", "stop"), ("direction", "forward")],
    );
    let stops = place_furniture(&source, &map, &surfaces);
    assert_eq!(stops.len(), 2);
    assert!(stops.iter().all(|p| p.kind == FurnitureKind::Stop));
    test::support::set_osm_tags(
        source.nodes.get_mut(&2).unwrap(),
        &[("traffic_sign", "maxspeed"), ("maxspeed", "200")],
    );
    assert!(place_furniture(&source, &map, &surfaces).is_empty());
    test::support::set_osm_tags(
        source.nodes.get_mut(&2).unwrap(),
        &[("traffic_sign", "maxspeed"), ("maxspeed", "30")],
    );
    let speeds = place_furniture(&source, &map, &surfaces);
    assert_eq!(speeds.len(), 2);
    assert!(speeds
        .iter()
        .all(|p| p.kind == FurnitureKind::Speed && p.value == 30));
    surfaces.obstacles = MultiPolygon(vec![util::geometry::polygon(&[
        XZ { x: -50., z: -50. },
        XZ { x: 50., z: -50. },
        XZ { x: 50., z: 50. },
        XZ { x: -50., z: 50. },
    ])]);
    assert!(place_furniture(&source, &map, &surfaces).is_empty());
}

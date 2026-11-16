use super::*;
use crate::model;
use crate::storage;
use crate::test;
use crate::util;
use crate::util::geometry::BoundingBoxXZ;
use std::collections::BTreeMap;

#[test]
fn mapped_barriers_clip_to_crop_avoid_roads_and_leave_gate_openings() {
    let mut map = model::source_location::empty();
    let origin = util::geometry::new(35.162, 33.320);
    map.origin = Some(origin);
    map.bounding_box = BoundingBoxXZ {
        min: XZ { x: -50., z: -50. },
        max: XZ { x: 50., z: 50. },
    };
    let mut source = build::osm::OSMMap {
        nodes: BTreeMap::new(),
        ways: BTreeMap::new(),
        relations: Vec::new(),
        bounds: None,
    };
    let points: [XZ<f64>; 5] = [
        XZ { x: -60., z: 20. },
        XZ { x: 0., z: 20. },
        XZ { x: 60., z: 20. },
        XZ { x: 20., z: -30. },
        XZ { x: 20., z: 30. },
    ];
    for (i, p) in points.iter().enumerate() {
        let tags = if i == 1 {
            vec![("barrier", "gate"), ("width", "4")]
        } else {
            vec![]
        };
        let mut node = test::support::osm_element(i as i64 + 1, &tags);
        node.coord = Some(util::geometry::new(
            origin.lat - p.z.to_degrees() / 6371000.,
            origin.lon + p.x.to_degrees() / 6371000. / origin.lat.to_radians().cos(),
        ));
        source.nodes.insert(node.id, node);
    }
    let mut wall = test::support::osm_element(
        10,
        &[("barrier", "wall"), ("height", "2.3"), ("width", "0.4")],
    );
    wall.nodes = vec![1, 2, 3];
    source.ways.insert(wall.id, wall);
    let mut fence =
        test::support::osm_element(11, &[("barrier", "fence"), ("fence_type", "chain_link")]);
    fence.nodes = vec![4, 5];
    source.ways.insert(fence.id, fence);
    map.road_nodes = vec![
        util::geometry::at_height(XZ { x: -40., z: 0. }, 0.),
        util::geometry::at_height(XZ { x: 40., z: 0. }, 0.),
    ];
    let mut road = model::source_location::new([0, 1], 8.0);
    road.osm_way_id = Some(20);
    map.road_lines.push(road);
    map.buildings.push(model::world::Building {
        id: 40,
        height_meters: 3.0,
        base_y: 0.0,
        tagged: true,
        holes: Vec::new(),
        min_height_meters: 0.0,
        ground_outline: vec![
            XZ { x: 30., z: 15. },
            XZ { x: 35., z: 15. },
            XZ { x: 35., z: 25. },
            XZ { x: 30., z: 25. },
        ],
    });
    let folder = tempfile::tempdir().unwrap();
    let surfaces =
        build::surfaces::write_streets_txt(&map, &folder.path().join("streets.txt"), None)
            .unwrap()
            .surfaces;
    let barriers = extract(&source, &map, &surfaces);
    assert_eq!(barriers.len(), 5);
    for barrier in &barriers {
        assert!(barrier
            .ends
            .iter()
            .all(|p| p.x >= -50. && p.x <= 50. && p.z >= -50. && p.z <= 50.));
        if barrier.kind == BarrierKind::Wall {
            assert_eq!(barrier.height_meters, 2.3);
            assert_eq!(barrier.width_meters, 0.4);
            assert!(barrier.ends.iter().all(|p| p.x.abs() >= 2.));
            assert!(barrier.ends[0].x * barrier.ends[1].x > 0.);
            assert!(
                barrier.ends.iter().all(|p| p.x < 30.) || barrier.ends.iter().all(|p| p.x > 35.)
            );
        } else {
            assert_eq!(barrier.kind, BarrierKind::ChainLink);
            assert!(barrier.ends.iter().all(|p| p.z.abs() >= 4.08));
            assert!(barrier.ends[0].z * barrier.ends[1].z > 0.);
        }
    }
    source.ways.insert(
        20,
        test::support::osm_element(20, &[("highway", "residential")]),
    );
    let furniture_path = folder.path().join("furniture.txt");
    build::furniture::build_furniture(&source, &map, &surfaces, &furniture_path).unwrap();
    let text = std::fs::read_to_string(furniture_path).unwrap();
    let stored = storage::codec_furniture::parse(&text).unwrap();
    assert_eq!(stored.barriers, barriers);
    assert!(!folder.path().join("barriers.txt").exists());
    source.ways.get_mut(&11).unwrap().nodes = vec![4, 999, 5];
    assert!(extract(&source, &map, &surfaces)
        .iter()
        .all(|b| b.kind == BarrierKind::Wall));
}

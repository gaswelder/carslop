use super::*;
use crate::test;

#[test]
fn imports_skip_every_gate_type_but_preserve_other_objects() {
    let mut source = build::osm::OSMMap {
        nodes: BTreeMap::new(),
        ways: BTreeMap::new(),
        relations: vec![],
        bounds: None,
    };
    let mut map = model::source_location::empty();
    map.origin = Some(util::geometry::new(35., 33.));
    map.bounding_box.min = XZ { x: -100., z: -100. };
    map.bounding_box.max = XZ { x: 100., z: 100. };
    for (index, barrier) in ["gate", "lift_gate", "swing_gate", "bollard"]
        .iter()
        .enumerate()
    {
        for locked in ["yes", "no"] {
            let id = (source.nodes.len() + 1) as i64;
            let mut node =
                test::support::osm_element(id, &[("barrier", barrier), ("locked", locked)]);
            node.coord = Some(util::geometry::new(35., 33. + index as f64 * 0.0001));
            source.nodes.insert(id, node);
        }
    }
    let mut bench = test::support::osm_element(20, &[("amenity", "bench")]);
    bench.coord = Some(util::geometry::new(35., 33.));
    source.nodes.insert(20, bench);
    let surfaces = build::surfaces::Surfaces {
        paths: vec![],
        adjacency: vec![],
        obstacles: util::geometry::empty(),
    };
    let details = build(&source, &map, &surfaces);
    assert_eq!(details.objects.len(), 3);
    assert_eq!(
        details
            .objects
            .iter()
            .filter(|object| object.kind == ObjectKind::Bollard)
            .count(),
        2
    );
    assert_eq!(
        details
            .objects
            .iter()
            .filter(|object| object.kind == ObjectKind::Bench)
            .count(),
        1
    );
}

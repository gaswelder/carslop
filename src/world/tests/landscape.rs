use super::*;
use crate::model;
use crate::model::map_details::Deck;
use crate::model::map_details::RoadSurface;
use crate::storage;
use crate::test;
use crate::util::geometry::BoundingBoxXZ;
use crate::util::geometry::XYZ;
use std::fs;
use std::hint::black_box;
use std::io::BufReader;
use std::path::Path;
use std::time::Instant;

#[test]
fn tree_index_matches_brute_force_across_cell_boundaries() {
    let trees = (-100..100)
        .map(|i| Tree {
            id: (i + 100) as u64,
            coords: XZ {
                x: i as f32 * 0.4,
                z: i as f32 * 0.3,
            },
            height_meters: 8.,
            crown_radius_meters: 3.,
        })
        .collect();
    let cover = new(vec![], trees);
    for p in [
        XZ { x: -32., z: -24. },
        XZ { x: 0., z: 0. },
        XZ { x: 32., z: 24. },
    ] {
        let mut actual: Vec<_> = nearby_trees(&cover, p, 1.82).map(|t| t.id).collect();
        actual.sort_unstable();
        let expected: Vec<_> = cover
            .trees
            .iter()
            .filter(|t| (t.coords.x - p.x).abs() <= 1.82 && (t.coords.z - p.z).abs() <= 1.82)
            .map(|t| t.id)
            .collect();
        assert_eq!(actual, expected);
    }
}

fn occupied_node_bytes(
    parent: &rstar::ParentNode<rstar::primitives::GeomWithData<[f32; 2], usize>>,
) -> usize {
    std::mem::size_of_val(parent.children())
        + parent
            .children()
            .iter()
            .map(|node| match node {
                rstar::RTreeNode::Leaf(_) => 0,
                rstar::RTreeNode::Parent(parent) => occupied_node_bytes(parent),
            })
            .sum::<usize>()
}

fn resident_bytes() -> usize {
    let status = fs::read_to_string("/proc/self/status").unwrap();
    status
        .lines()
        .find_map(|line| line.strip_prefix("VmRSS:"))
        .unwrap()
        .split_whitespace()
        .next()
        .unwrap()
        .parse::<usize>()
        .unwrap()
        * 1024
}

/// Opt-in measurements against installed vegetation, excluding file parsing.
#[test]
#[ignore]
fn tree_collision_cost() {
    for name in ["Nicosia", "location-20261118-213838"] {
        let path = Path::new("data/locations").join(name).join("landscape.txt");
        if !path.exists() {
            continue;
        }
        let mut trees = Vec::new();
        storage::landscape::read(BufReader::new(fs::File::open(path).unwrap()), |record| {
            if let storage::landscape::Record::Tree(tree) = record {
                trees.push(tree);
            }
            Ok(())
        })
        .unwrap();
        let count = trees.len();
        let before = resident_bytes();
        let start = Instant::now();
        let cover = new(vec![], trees);
        let build = start.elapsed();
        let rss = resident_bytes().saturating_sub(before);
        let occupied = occupied_node_bytes(cover.tree_index.root());
        let mut location = model::world::empty();
        let (surfaces, _) = world::street_surface::from_geometry(
            model::location::Streets {
                paths: vec![],
                triangles: vec![],
                curbs: vec![],
            },
            world::street_surface::DEFAULT_PAVEMENT_HEIGHT,
        )
        .unwrap();
        test::support::install_surfaces(&mut location, surfaces);
        location.ground = cover;
        location.elevation = Some(world::elevation::generate(
            BoundingBoxXZ {
                min: XZ {
                    x: -100_000.,
                    z: -100_000.,
                },
                max: XZ {
                    x: 100_000.,
                    z: 100_000.,
                },
            },
            |_| 0.,
        ));
        let samples = 100_000;
        let start = Instant::now();
        for i in 0..samples {
            let tree = &location.ground.trees[i * 7919 % count];
            let mut position = XYZ {
                x: tree.coords.x - 1.,
                y: 0.48,
                z: tree.coords.z,
            };
            let mut velocity = XZ { x: 20., z: 0. };
            black_box(world::trees::constrain(
                black_box(&location),
                &mut position,
                &mut velocity,
                1.7,
            ));
            black_box(position);
            black_box(velocity);
        }
        let contact_ns = start.elapsed().as_nanos() / samples as u128;
        eprintln!("{name}: trees={count}, index_build={build:?}, RSS_delta={rss} bytes, occupied_node_storage={occupied} bytes (excludes spare capacity/allocator overhead), contact={contact_ns} ns/query ({:.3} ms/frame at 60 FPS / 240 Hz)", contact_ns as f64 * 4. / 1e6);
    }
}

#[test]
fn bridge_clearance_checks_crowns_heights_and_all_tree_sources() {
    let field = world::elevation::generate(
        BoundingBoxXZ {
            min: XZ { x: -50., z: -50. },
            max: XZ { x: 50., z: 50. },
        },
        |_| 0.,
    );
    let mut details = Data::default();
    details.decks.push(Deck {
        road: 0,
        grade: 1,
        width: 8.,
        surface: RoadSurface::Asphalt,
        path: vec![[-20., 5., 0.], [20., 5., 0.]],
    });
    details.decks.push(Deck {
        road: 1,
        grade: -1,
        width: 8.,
        surface: RoadSurface::Asphalt,
        path: vec![[-20., -5., 30.], [20., -5., 30.]],
    });
    let mut trees: Vec<_> = [
        Tree {
            id: 1,
            coords: XZ { x: 0., z: 0. },
            height_meters: 8.,
            crown_radius_meters: 2.,
        },
        Tree {
            id: 1 << 62,
            coords: XZ { x: 0., z: 6. },
            height_meters: 8.,
            crown_radius_meters: 2.5,
        },
        Tree {
            id: 1 << 63,
            coords: XZ { x: 21., z: 0. },
            height_meters: 8.,
            crown_radius_meters: 2.,
        },
        Tree {
            id: 2,
            coords: XZ { x: 0., z: 0. },
            height_meters: 3.,
            crown_radius_meters: 1.,
        },
        Tree {
            id: 3,
            coords: XZ { x: 0., z: 8. },
            height_meters: 8.,
            crown_radius_meters: 2.,
        },
        Tree {
            id: 4,
            coords: XZ { x: 0., z: 30. },
            height_meters: 8.,
            crown_radius_meters: 2.,
        },
    ]
    .into();
    clear_transport_trees(&mut trees, &field, &details, &[]);
    assert_eq!(
        trees.iter().map(|tree| tree.id).collect::<Vec<_>>(),
        vec![2, 3, 4]
    );
}

#[test]
fn approach_clearance_uses_offsets_above_local_terrain() {
    let field = world::elevation::generate(
        BoundingBoxXZ {
            min: XZ { x: -20., z: -20. },
            max: XZ { x: 20., z: 20. },
        },
        |_| 12.,
    );
    let mut trees = vec![
        Tree {
            id: 1,
            coords: XZ { x: 0., z: 0. },
            height_meters: 8.,
            crown_radius_meters: 2.,
        },
        Tree {
            id: 2,
            coords: XZ { x: 6., z: 0. },
            height_meters: 8.,
            crown_radius_meters: 2.,
        },
        Tree {
            id: 3,
            coords: XZ { x: 10., z: 0. },
            height_meters: 8.,
            crown_radius_meters: 2.,
        },
    ];
    let rises = [RoadRise {
        points: [
            util::geometry::at_height(XZ { x: -5., z: -5. }, 2.),
            util::geometry::at_height(XZ { x: 5., z: -5. }, 2.),
            util::geometry::at_height(XZ { x: 5., z: 5. }, 2.),
        ],
    }];
    clear_transport_trees(&mut trees, &field, &Data::default(), &rises);
    assert_eq!(
        trees.iter().map(|tree| tree.id).collect::<Vec<_>>(),
        vec![3]
    );
}

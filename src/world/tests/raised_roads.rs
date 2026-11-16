use super::*;
use crate::util;
use crate::util::geometry::BoundingBoxXZ;
use crate::world;

#[test]
fn clipped_ramp_fragments_match_contact_even_when_thin() {
    let field = world::elevation::generate(
        BoundingBoxXZ {
            min: XZ { x: -180., z: 190. },
            max: XZ { x: -130., z: 240. },
        },
        |_| -6.,
    );
    let rise = RoadRise {
        points: [
            util::geometry::at_height(
                XZ {
                    x: -155.231,
                    z: 217.486,
                },
                4.6416945,
            ),
            util::geometry::at_height(
                XZ {
                    x: -147.85423,
                    z: 218.83997,
                },
                4.6416807,
            ),
            util::geometry::at_height(
                XZ {
                    x: -148.4366,
                    z: 218.83043,
                },
                4.4638968,
            ),
        ],
    };
    let rises = new(vec![rise]);
    let mut count = 0;
    for step in 0..40 {
        let shift = step as f32 * 0.017;
        let points = [
            XZ {
                x: -165. + shift,
                z: 215. + shift,
            },
            XZ {
                x: -140.,
                z: 220. - shift,
            },
            XZ {
                x: -150. + shift,
                z: 230.,
            },
        ];
        drape(&rises, &field, points, |triangle| {
            count += 1;
            let center = util::geometry::xz_from_fn(|axis| {
                triangle
                    .iter()
                    .map(|p| util::geometry::xz_component(util::geometry::xz(*p), axis))
                    .sum::<f32>()
                    / 3.
            });
            let expected =
                world::elevation::height(&field, center) + raised_roads_offset(&rises, center);
            let height = triangle.iter().map(|p| p.y).sum::<f32>() / 3.;
            assert!(
                (height - expected).abs() < 0.001,
                "mesh {triangle:?} differs from contact {expected} at {center:?}"
            );
        });
    }
    assert!(count > 0);
}

#[test]
fn raised_road_drape_and_contact_share_the_same_piecewise_surface() {
    let mut triangles = Vec::new();
    struct Section {
        station: f32,
        height: f32,
    }
    let sections = [
        Section {
            station: -3.,
            height: 0.,
        },
        Section {
            station: -2.,
            height: 0.1,
        },
        Section {
            station: 2.,
            height: 0.1,
        },
        Section {
            station: 3.,
            height: 0.,
        },
    ];
    for pair in sections.windows(2) {
        let a = util::geometry::at_height(
            XZ {
                x: pair[0].station,
                z: -4.,
            },
            pair[0].height,
        );
        let b = util::geometry::at_height(
            XZ {
                x: pair[0].station,
                z: 4.,
            },
            pair[0].height,
        );
        let c = util::geometry::at_height(
            XZ {
                x: pair[1].station,
                z: 4.,
            },
            pair[1].height,
        );
        let d = util::geometry::at_height(
            XZ {
                x: pair[1].station,
                z: -4.,
            },
            pair[1].height,
        );
        triangles.extend([
            RoadRise { points: [a, b, c] },
            RoadRise { points: [a, c, d] },
        ]);
    }
    let rises = world::raised_roads::new(triangles);
    for probe in [
        Section {
            station: -4.,
            height: 0.,
        },
        Section {
            station: -2.5,
            height: 0.05,
        },
        Section {
            station: 0.,
            height: 0.1,
        },
        Section {
            station: 2.5,
            height: 0.05,
        },
        Section {
            station: 4.,
            height: 0.,
        },
    ] {
        assert!(
            (world::raised_roads::raised_roads_offset(
                &(rises),
                XZ {
                    x: probe.station,
                    z: 0.
                }
            ) - probe.height)
                .abs()
                < 0.00001
        );
    }
    let field = world::elevation::generate(
        BoundingBoxXZ {
            min: XZ { x: -8., z: -8. },
            max: XZ { x: 8., z: 8. },
        },
        |p| 7. + p.x * 0.1 + p.z * 0.03,
    );
    let points = [
        XZ { x: -8., z: -6. },
        XZ { x: 8., z: -6. },
        XZ { x: 8., z: 6. },
    ];
    let mut area = 0.;
    let mut peak: f32 = 0.;
    world::raised_roads::drape(&rises, &field, points, |triangle| {
        let [a, b, c] = triangle;
        let triangle_area = ((b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x)).abs() / 2.;
        if triangle_area < 0.000001 {
            return;
        }
        area += triangle_area;
        let center = XZ {
            x: (a.x + b.x + c.x) / 3.,
            z: (a.z + b.z + c.z) / 3.,
        };
        let height = (a.y + b.y + c.y) / 3.;
        assert!(
            (height
                - world::elevation::height(&(field), center)
                - world::raised_roads::raised_roads_offset(&(rises), center))
            .abs()
                < 0.00001,
            "mesh {} contact {} at {:?}: {:?}",
            height,
            world::elevation::height(&(field), center)
                + world::raised_roads::raised_roads_offset(&(rises), center),
            center,
            triangle
        );
        for point in triangle {
            peak = peak.max(point.y - world::elevation::height(&field, util::geometry::xz(point)));
        }
    });
    assert!((area - 96.).abs() < 0.0001);
    assert!((peak - 0.1).abs() < 0.00001);
    let edge =
        world::raised_roads::segment(&rises, &field, XZ { x: -8., z: 4. }, XZ { x: 8., z: 4. });
    for x in [-3., -2., 2., 3.] {
        assert!(edge.iter().any(|point| (point.x - x).abs() < 0.0001));
    }
    for point in edge {
        assert!(
            (point.y
                - world::elevation::height(&(field), util::geometry::xz(point))
                - world::raised_roads::raised_roads_offset(&(rises), util::geometry::xz(point)))
            .abs()
                < 0.00001
        );
    }
}

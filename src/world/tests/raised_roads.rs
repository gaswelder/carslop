use super::*;
use crate::util;
use crate::util::geometry::BoundingBoxXZ;
use crate::world;
use geo::Area;

#[test]
fn warsaw_tunnel_ramp_slivers_preserve_road_coverage() {
    use crate::storage;
    // Reduced from the 4 km Warsaw import. Repeated overlay differences left
    // near-collinear slivers and panicked during OGC hole extraction here.
    let mut triangles = storage::road_rises::parse(
        r#"ROAD_RISES 2
t 1977.0885 -3.2000103 -1768.2445 1976.6497 -3.398543 -1768.881 1976.996 -2.8844166 -1769.033
t 1973.8123 -3.150655 -1767.6356 1976.6497 -3.398543 -1768.881 1979.0363 -3.3985376 -1763.4432
t 1977.0885 -3.2001202 -1768.2445 1977.1969 -2.9301171 -1768.4915 1978.8369 -2.9572952 -1769.0352
t 1977.0531 -3.3366477 -1768.0751 1979.0388 -3.392375 -1763.4489 1979.0363 -3.398634 -1763.4432
t 1979.0363 -3.3985376 -1763.4432 1978.8252 -3.380404 -1763.3601 1977.5726 -3.2927752 -1763.4884"#,
    )
    .unwrap();
    let field = world::elevation::generate(
        BoundingBoxXZ {
            min: XZ {
                x: 1950.,
                z: -1810.,
            },
            max: XZ {
                x: 1990.,
                z: -1750.,
            },
        },
        |_| 0.,
    );
    let points = [
        XZ {
            x: 1960.7941,
            z: -1793.5007,
        },
        XZ {
            x: 1974.5582,
            z: -1778.7391,
        },
        XZ {
            x: 1981.1296,
            z: -1759.2704,
        },
    ];
    let flat = points.map(|p| util::geometry::xz_map(p, f64::from));
    let expected = util::geometry::polygon(&flat).unsigned_area();
    for reverse in [false, true] {
        if reverse {
            for rise in &mut triangles {
                rise.points.reverse();
            }
        }
        for _ in 0..triangles.len() {
            triangles.rotate_left(1);
            let rises = new(triangles.clone());
            let mut area = 0.0;
            drape(&rises, &field, points, |face| {
                assert!(face.iter().all(|p| p.y.is_finite()));
                let flat = face.map(|p| util::geometry::xz_map(util::geometry::xz(p), f64::from));
                area += util::geometry::polygon(&flat).unsigned_area();
            });
            assert!(
                (area - expected).abs() < 0.002,
                "area {area} differs from {expected}"
            );
        }
    }
}

#[test]
#[ignore = "requires local compiled city assets"]
fn compiled_city_road_drape() {
    use crate::storage;
    let path = std::env::var_os("NFS_ROAD_DRAPE_CITY").expect("set NFS_ROAD_DRAPE_CITY");
    let city =
        world::preparation::world_location_load(&storage::city_assets::new(path.into())).unwrap();
    let mut count = 0;
    let mut total_area = 0.0;
    let mut total_expected = 0.0;
    for triangle in world::roads::road_triangles(&city) {
        let points = triangle.points.map(Into::into);
        let mut area = 0.0;
        drape_road_triangle(&city, points, |face| {
            assert!(face
                .iter()
                .all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite()));
            let flat = face.map(|p| util::geometry::xz_map(util::geometry::xz(p), f64::from));
            area += util::geometry::polygon(&flat).unsigned_area();
            count += 1;
        });
        let flat = points.map(|p| util::geometry::xz_map(p, f64::from));
        let expected = util::geometry::polygon(&flat).unsigned_area();
        // Clipped vertices return to f32 for rendering. Bound the resulting area
        // error by the perimeter and coordinate precision, including long slivers.
        let coordinate_scale = flat
            .iter()
            .map(|p| p.x.abs().max(p.z.abs()))
            .fold(1.0, f64::max);
        let perimeter: f64 = (0..3)
            .map(|i| {
                let a = flat[i];
                let b = flat[(i + 1) % 3];
                (b.x - a.x).hypot(b.z - a.z)
            })
            .sum();
        let rounding_allowance = perimeter * coordinate_scale * f32::EPSILON as f64;
        total_area += area;
        total_expected += expected;
        assert!(
            (area - expected).abs() < rounding_allowance + expected * 0.001,
            "area {area} differs from {expected} for {points:?}"
        );
    }
    assert!(count > 0);
    assert!((total_area - total_expected).abs() < total_expected * 0.0001);
    eprintln!(
        "Draped {} road triangles into {count} terrain faces",
        world::roads::road_triangles(&city).len()
    );
}

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

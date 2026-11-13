//! Offline OSM street network. Geometry, collision, recovery and minimap share it.
#[cfg(test)]
use crate::geometry::{BoundingBoxXZ, XZ};
#[cfg(test)]
use crate::track::CAR_RADIUS;
#[cfg(test)]
use crate::vehicle_location::deflect;
pub use crate::world::Building;
use std::collections::HashMap;

#[cfg(test)]
use crate::world::spatial::*;

/// Read just way labels, without building the full OSM geometry again.
pub struct OsmLabels {
    pub streets: HashMap<i64, String>,
    pub buildings: HashMap<u64, String>,
}

pub fn read_osm_labels(source: impl std::io::BufRead) -> Result<OsmLabels, String> {
    use quick_xml::{events::Event, Reader};
    let mut reader = Reader::from_reader(source);
    let mut buffer = Vec::new();
    let mut names = HashMap::new();
    let mut buildings = HashMap::new();
    let mut building = false;
    let mut way = None;
    let mut labels = [String::new(), String::new(), String::new()];
    loop {
        match reader
            .read_event_into(&mut buffer)
            .map_err(|e| e.to_string())?
        {
            Event::Start(e) if e.name().as_ref() == "way" => {
                way = None;
                building = false;
                labels.iter_mut().for_each(String::clear);
                for attr in e.attributes() {
                    let attr = attr.map_err(|e| e.to_string())?;
                    if attr.key.as_ref() == "id" {
                        way = attr.value.parse().ok();
                    }
                }
            }
            Event::Start(e) | Event::Empty(e) if way.is_some() && e.name().as_ref() == "tag" => {
                let mut key = String::new();
                let mut value = String::new();
                for attr in e.attributes() {
                    let attr = attr.map_err(|e| e.to_string())?;
                    let decoded = attr
                        .normalized_value(quick_xml::XmlVersion::Implicit1_0)
                        .map_err(|e| e.to_string())?;
                    match attr.key.as_ref() {
                        "k" => key = decoded.into_owned(),
                        "v" => value = decoded.into_owned(),
                        _ => {}
                    }
                }
                if key == "building" {
                    building = !matches!(value.as_str(), "" | "no");
                }
                let index = match key.as_str() {
                    "name:en" => Some(0),
                    "name" => Some(1),
                    "ref" => Some(2),
                    _ => None,
                };
                if let Some(index) = index {
                    labels[index] = value.trim().to_owned();
                }
            }
            Event::End(e) if e.name().as_ref() == "way" => {
                if let Some(id) = way.take() {
                    if building {
                        if let Some(label) = labels[..2].iter().find(|s| !s.is_empty()) {
                            buildings.insert(id as u64, label.clone());
                        }
                    }
                    if let Some(label) = labels.iter().find(|label| !label.is_empty()) {
                        names.insert(id, label.clone());
                    }
                }
            }
            Event::Eof => break,
            _ => {}
        }
        buffer.clear();
    }
    Ok(OsmLabels {
        streets: names,
        buildings,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn repeated_light_barrier_contacts_preserve_sliding_speed() {
        // Exercise a rotated wall so the response cannot depend on world axes.
        let normal = crate::geometry::XZ { x: 0.6, z: 0.8 };
        let tangent = crate::geometry::XZ { x: -0.8, z: 0.6 };
        let mut speed = 20.0;
        for _ in 0..120 {
            let mut velocity = crate::geometry::XZ::from_fn(|i| {
                tangent.component(i) * speed + normal.component(i) * 0.01
            });
            let impact = deflect(&mut velocity, normal);
            assert!((impact - 0.01).abs() < 0.00001);
            assert!(velocity.x * normal.x + velocity.z * normal.z <= 0.0);
            speed = velocity.x * tangent.x + velocity.z * tangent.z;
        }
        assert!(speed > 19.9, "light scrapes drained sliding speed: {speed}");
    }

    #[test]
    fn barrier_response_rebounds_head_on_and_leaves_departing_cars_alone() {
        let mut velocity = crate::geometry::XZ { x: 10.0, z: 0.0 };
        assert_eq!(
            deflect(&mut velocity, crate::geometry::XZ { x: 1.0, z: 0.0 }),
            10.0
        );
        assert!((velocity.x + 0.8).abs() < 0.00001);
        assert_eq!(velocity.z, 0.0);
        let departing = velocity;
        assert_eq!(
            deflect(&mut velocity, crate::geometry::XZ { x: 1.0, z: 0.0 }),
            0.0
        );
        assert_eq!(velocity, departing);
    }

    const CROSS:&str="CITY 2\norigin 0 0\nbounds -50 -50 50 50\nn -40 0\nn 0 0\nn 40 0\nn 0 -40\nn 0 40\nr 0 1 8 0 1\nr 1 2 8 0 1\nr 3 1 8 0 2\nr 1 4 8 0 2\nb 1 10 1 4 8 8 18 8 18 18 8 18\n";

    #[test]
    fn branches_are_open_and_dead_ends_do_not_join_other_streets() {
        let city = crate::test_support::parse_world(CROSS).unwrap();
        for i in 0..4 {
            for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
                let s = city.sample(i, t);
                let mut p = s.vehicle_position(0.0);
                let before = p;
                city.constrain(&mut p, &mut crate::geometry::XZ::default());
                assert_eq!(p, before);
            }
        }
        assert!(
            city.road_clearance(crate::geometry::XYZ {
                x: 25.0,
                y: 0.0,
                z: 25.0
            }) > 10.0
        );
        let mut p = crate::geometry::XYZ {
            x: 0.0,
            y: 0.48,
            z: 45.0,
        };
        city.constrain(&mut p, &mut crate::geometry::XZ { x: 0.0, z: 10.0 });
        assert!(p.z <= 42.61);
    }
    #[test]
    fn building_contacts_and_camera_respect_actual_footprints() {
        let collision = crate::test_support::parse_world(
            &CROSS.replace("8 8 18 8 18 18 8 18", "8 -4 18 -4 18 4 8 4"),
        )
        .unwrap();
        let mut position = crate::geometry::XYZ {
            x: 10.0,
            y: 0.48,
            z: 0.0,
        };
        let mut velocity = crate::geometry::XZ { x: 10.0, z: 0.0 };
        assert!(collision.constrain(&mut position, &mut velocity) > 0.0);
        assert!(!collision.buildings()[0].contains(crate::geometry::XZ {
            x: position.x,
            z: position.z
        }));
        let city = crate::test_support::parse_world(CROSS).unwrap();
        let b = &city.buildings()[0];
        assert!(b.contains(crate::geometry::XZ { x: 10.0, z: 10.0 }));
        assert!(!b.contains(crate::geometry::XZ { x: 7.0, z: 10.0 }));
        let eye = city.camera_eye(
            crate::geometry::XYZ {
                x: 0.0,
                y: 1.0,
                z: 10.0,
            },
            crate::geometry::XYZ {
                x: 25.0,
                y: 4.0,
                z: 10.0,
            },
        );
        assert!(eye.x < 7.6);
        assert_eq!(
            city.camera_eye(
                crate::geometry::XYZ {
                    x: 0.0,
                    y: 12.0,
                    z: 10.0
                },
                crate::geometry::XYZ {
                    x: 25.0,
                    y: 14.0,
                    z: 10.0
                }
            ),
            crate::geometry::XYZ {
                x: 25.0,
                y: 14.0,
                z: 10.0
            }
        );
    }
    #[test]
    fn engomi_ground_contact_props_and_foundations_share_elevation() {
        let city_track = crate::test_support::nicosia_world();
        let city = &city_track;
        let mut min = f32::INFINITY;
        let mut max = f32::NEG_INFINITY;
        for i in 0..city.road_lines().len() {
            for t in [0.0, 0.37, 1.0] {
                let sample = city.sample(i, t);
                min = min.min(sample.point.y);
                max = max.max(sample.point.y);
                assert!(
                    (sample.point.y
                        - city.height(crate::geometry::XZ {
                            x: sample.point.x,
                            z: sample.point.z
                        }))
                    .abs()
                        < 0.001
                );
                let mut p = sample.vehicle_position(0.7);
                city.constrain(&mut p, &mut crate::geometry::XZ::default());
                assert!(
                    (p.y - city.drive_height(crate::geometry::XZ { x: p.x, z: p.z }) - 0.48).abs()
                        < 0.001
                );
            }
        }
        assert!(max - min > 20.0 && max - min < 1000.0);
        for prop in &city.furniture {
            assert!(
                (prop.coords.y
                    - city.height(crate::geometry::XZ {
                        x: prop.coords.x,
                        z: prop.coords.z
                    }))
                .abs()
                    < 0.001
            );
        }
        for building in city.buildings() {
            for i in 0..building.ground_outline.len() {
                for p in city.ground_segment(
                    building.ground_outline[i],
                    building.ground_outline[(i + 1) % building.ground_outline.len()],
                ) {
                    assert!(building.base_y >= p.y);
                }
            }
        }
    }
    #[test]
    fn sidewalks_and_parking_are_drivable_at_rendered_height_and_crop_stays_solid() {
        let city_track = crate::test_support::nicosia_world();
        let city = &city_track;
        let mut checked = 0;
        for triangle in city.road_triangles() {
            if !matches!(
                triangle.usage,
                crate::world::TriangleUsage::Pavement(crate::world::CurbStyle::MajorRoad)
                    | crate::world::TriangleUsage::Pavement(crate::world::CurbStyle::Road)
            ) {
                continue;
            }
            let q = crate::geometry::XZ {
                x: triangle.points.iter().map(|p| p.x).sum::<f32>() / 3.0,
                z: triangle.points.iter().map(|p| p.z).sum::<f32>() / 3.0,
            };
            // Sliver centroids can round onto shared edges at city coordinates.
            // Sample pavement interiors, outside the 2 mm edge tolerance.
            if (0..3).any(|i| {
                let a = triangle.points[i];
                let b = triangle.points[(i + 1) % 3];
                let edge = nearest_edge(q, a.into(), b.into());
                (q.x - edge.x).hypot(q.z - edge.z) < 0.02
            }) || city.building_clearance(q) < CAR_RADIUS + 0.1
            {
                continue;
            }
            let mut p = crate::geometry::XYZ {
                x: q.x,
                y: 0.0,
                z: q.z,
            };
            let mut velocity = crate::geometry::XZ { x: 0.0, z: 0.0 };
            assert_eq!(city.constrain(&mut p, &mut velocity), 0.0);
            assert!((p.x - q.x).hypot(p.z - q.z) < 0.001);
            assert!(
                (p.y - city.height(q) - (0.48 + city.pavement_height_meters() - 0.05)).abs()
                    < 0.001,
                "q={q:?} offset={} style={:?}",
                p.y - city.height(q),
                triangle.usage
            );
            assert!(city.paved(q));
            checked += 1;
            if checked == 20 {
                break;
            }
        }
        assert_eq!(checked, 20);
        let mut parking = 0;
        for triangle in city.ground_triangles() {
            if triangle.material != crate::world::Material::Asphalt {
                continue;
            }
            let q = crate::geometry::XZ {
                x: triangle.points.iter().map(|p| p.x).sum::<f32>() / 3.0,
                z: triangle.points.iter().map(|p| p.z).sum::<f32>() / 3.0,
            };
            // Land cover is layered underneath streets; test exposed parking.
            if city.paving_height(q).is_some() || city.building_clearance(q) < CAR_RADIUS + 0.1 {
                continue;
            }
            let mut p = crate::geometry::XYZ {
                x: q.x,
                y: 0.0,
                z: q.z,
            };
            city.constrain(&mut p, &mut crate::geometry::XZ::default());
            assert!((p.x - q.x).hypot(p.z - q.z) < 0.001);
            assert!(city.paved(q));
            parking += 1;
            if parking == 20 {
                break;
            }
        }
        assert_eq!(parking, 20);
        let mut p = crate::geometry::XYZ {
            x: city.bounding_box().max.x + 10.0,
            y: 0.0,
            z: 0.0,
        };
        let mut velocity = crate::geometry::XZ { x: 10.0, z: 0.0 };
        assert!(city.constrain(&mut p, &mut velocity) > 0.0);
        assert!(p.x <= city.bounding_box().max.x - CAR_RADIUS && velocity.x <= 0.0);
    }

    #[test]
    fn urban_map_covers_centres_on_both_sides_and_queries_only_nearby_streets() {
        let city_track = crate::test_support::nicosia_world();
        let city = &city_track;
        assert!(city.road_lines().len() > 100_000 && city.buildings().len() > 70_000);
        for (name, lat, lon) in [
            ("Nicosia", 35.1746503, 33.3638783),
            ("Strovolos", 35.1328993, 33.3454972),
            ("Lakatamia", 35.1200946, 33.3078496),
            ("Aglandjia", 35.1474032, 33.4002217),
            ("Gonyeli", 35.2164170, 33.3058171),
            ("Taskinkoy", 35.2070072, 33.3457502),
        ] {
            let p = crate::geometry::XYZ {
                x: ((lon - 33.32_f64).to_radians() * 6_371_000.0 * 35.162_f64.to_radians().cos())
                    as f32,
                y: 0.0,
                z: (-(lat - 35.162_f64).to_radians() * 6_371_000.0) as f32,
            };
            let road = city.project(p);
            assert!(
                (road.point.x - p.x).hypot(road.point.z - p.z) < 250.0,
                "missing public surface roads near {name}"
            );
        }
        let bounds = BoundingBoxXZ {
            min: XZ {
                x: -200.0,
                z: -130.0,
            },
            max: XZ { x: 200.0, z: 130.0 },
        };
        let intersects = |a: crate::geometry::XYZ, b: crate::geometry::XYZ| {
            a.x.min(b.x) <= bounds.max.x
                && a.x.max(b.x) >= bounds.min.x
                && a.z.min(b.z) <= bounds.max.z
                && a.z.max(b.z) >= bounds.min.z
        };
        let nearby = city.road_segments_in(bounds);
        assert!(nearby.len() < city.road_lines().len() / 10);
        let actual = nearby.iter().filter(|(a, b)| intersects(*a, *b)).count();
        let all = city
            .road_lines()
            .iter()
            .flat_map(|r| r.path.windows(2))
            .filter(|p| intersects(p[0], p[1]))
            .count();
        assert_eq!(
            actual, all,
            "spatial minimap query must not omit crossing roads"
        );
    }

    #[test]
    fn expanding_grid_projection_matches_brute_force_outside_road_cells() {
        let city = crate::test_support::parse_world(CROSS).unwrap();
        for p in [
            [125.0, 0.0, 175.0],
            [-200.0, 0.0, -300.0],
            [10.0, 0.0, 1000.0],
        ] {
            let indexed = city.project((p).into());
            let brute = (0..city.road_lines().len())
                .map(|i| city.projected(i, (p).into()))
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .unwrap()
                .1;
            assert!(
                (indexed.point.x - brute.point.x).hypot(indexed.point.z - brute.point.z) < 0.001
            );
        }
    }
}

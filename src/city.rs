//! Offline OSM street network. Geometry, collision, recovery and minimap share it.
use crate::geometry;
#[cfg(test)]
use crate::geometry::BoundingBoxXZ;
use crate::geometry::XZ;
use crate::osm;
#[cfg(test)]
use crate::track::CAR_RADIUS;
#[cfg(test)]
use crate::vehicle_location::deflect;
#[cfg(test)]
use crate::world::spatial::*;
pub use crate::world::Building;
use std::collections::HashMap;

/// Source labels and businesses associated with their containing building.
pub struct OsmLabels {
    pub streets: HashMap<i64, String>,
    pub buildings: HashMap<u64, String>,
    pub storefronts: Vec<(u64, String)>,
}

pub fn read_osm_labels(source: impl std::io::BufRead) -> Result<OsmLabels, String> {
    let map = osm::read_xml(source).map_err(|e| e.to_string())?;
    Ok(osm_labels(&map))
}

fn osm_labels(map: &osm::OSMMap) -> OsmLabels {
    use geo::{Area, BoundingRect, Intersects};
    let name = |e: &osm::OSMElement| {
        [e.tag("name:en"), e.tag("name")]
            .into_iter()
            .map(str::trim)
            .find(|s| !s.is_empty())
            .unwrap_or("")
            .to_owned()
    };
    let mut labels = OsmLabels {
        streets: HashMap::new(),
        buildings: HashMap::new(),
        storefronts: Vec::new(),
    };
    let mut footprints = Vec::new();
    for way in map.ways.values() {
        let label = name(way);
        let street = if label.is_empty() {
            way.tag("ref").trim()
        } else {
            &label
        };
        if !street.is_empty() {
            labels.streets.insert(way.id, street.to_owned());
        }
        if matches!(way.tag("building"), "" | "no") {
            continue;
        }
        if !label.is_empty() {
            labels.buildings.insert(way.id as u64, label);
        }
        if way.nodes.len() < 4 || way.nodes.first() != way.nodes.last() {
            continue;
        }
        let points: Option<Vec<_>> = way
            .nodes
            .iter()
            .map(|id| {
                map.nodes
                    .get(id)?
                    .coord
                    .map(|p| XZ { x: p.lon, z: p.lat })
            })
            .collect();
        if let Some(points) = points {
            let polygon = geometry::polygon(&points);
            if let Some(bounds) = polygon.bounding_rect() {
                footprints.push((way.id as u64, polygon, bounds));
            }
        }
    }
    let index = rstar::RTree::bulk_load(
        footprints
            .iter()
            .enumerate()
            .map(|(i, (_, _, bounds))| {
                rstar::primitives::GeomWithData::new(
                    rstar::primitives::Rectangle::from_corners(
                        [bounds.min().x, bounds.min().y],
                        [bounds.max().x, bounds.max().y],
                    ),
                    i,
                )
            })
            .collect(),
    );
    for node in map.nodes.values() {
        // Named outdoor features and transport stops are not building tenants.
        let business = ["shop", "office", "craft"]
            .iter()
            .any(|key| !matches!(node.tag(key), "" | "no" | "vacant"))
            || matches!(
                node.tag("amenity"),
                "bank"
                    | "bar"
                    | "cafe"
                    | "restaurant"
                    | "fast_food"
                    | "ice_cream"
                    | "pub"
                    | "pharmacy"
                    | "clinic"
                    | "dentist"
                    | "doctors"
                    | "veterinary"
            )
            || node.tag("tourism") == "gallery"
            || !matches!(node.tag("healthcare"), "" | "no");
        let label = name(node);
        if !business || label.is_empty() {
            continue;
        }
        let Some(p) = node.coord else {
            continue;
        };
        let point = geo::Point::new(p.lon, p.lat);
        let envelope = rstar::AABB::from_point([p.lon, p.lat]);
        let owner = index
            .locate_in_envelope_intersecting(&envelope)
            .map(|entry| &footprints[entry.data])
            .filter(|(_, polygon, _)| polygon.intersects(&point))
            .min_by(|a, b| {
                a.1.unsigned_area()
                    .total_cmp(&b.1.unsigned_area())
                    .then(a.0.cmp(&b.0))
            });
        if let Some((id, _, _)) = owner {
            labels.storefronts.push((*id, label));
        }
    }
    labels.storefronts.sort();
    labels.storefronts.dedup();
    labels
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry;
    use crate::location;
    use crate::source_location;
    use crate::test_support;
    use crate::world;

    #[test]
    fn business_nodes_share_buildings_without_losing_names_or_label_precedence() {
        let source = r#"<osm>
<node id="1" lat="0" lon="0"/><node id="2" lat="0" lon="1"/>
<node id="3" lat="1" lon="1"/><node id="4" lat="1" lon="0"/>
<node id="5" lat="0.2" lon="0.2"><tag k="shop" v="clothes"/><tag k="name" v="Orchestra"/></node>
<node id="6" lat="0.8" lon="0.2"><tag k="amenity" v="ice_cream"/><tag k="name" v="Local"/><tag k="name:en" v="Pahit &amp; Ice"/></node>
<node id="7" lat="0.8" lon="0.8"><tag k="highway" v="bus_stop"/><tag k="name" v="Bus stop"/></node>
<node id="8" lat="2" lon="2"><tag k="shop" v="electronics"/><tag k="name" v="Outside"/></node>
<node id="9" lat="0.4" lon="0.2"><tag k="shop" v="clothes"/><tag k="name" v="Orchestra"/></node>
<way id="10"><nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/><nd ref="1"/>
<tag k="building" v="yes"/><tag k="name" v="Shopping centre"/></way>
<way id="11"><nd ref="1"/><nd ref="2"/><tag k="highway" v="primary"/><tag k="name" v="Local road"/><tag k="name:en" v="28th October"/></way>
</osm>"#;
        let labels = read_osm_labels(source.as_bytes()).unwrap();
        assert_eq!(labels.buildings[&10], "Shopping centre");
        assert_eq!(labels.streets[&11], "28th October");
        assert_eq!(
            labels.storefronts,
            vec![(10, "Orchestra".into()), (10, "Pahit & Ice".into())]
        );
        let stored = location::Labels {
            streets: labels.streets.into_iter().collect(),
            buildings: labels.buildings.into_iter().collect(),
            storefronts: labels.storefronts,
        };
        let bytes = serde_json::to_vec(&stored).unwrap();
        assert_eq!(
            serde_json::from_slice::<location::Labels>(&bytes).unwrap(),
            stored
        );
        let legacy: location::Labels =
            serde_json::from_str(r#"{"streets":[],"buildings":[]}"#).unwrap();
        assert!(legacy.storefronts.is_empty());
    }

    #[test]
    #[ignore = "requires the downloaded Engomi OSM extract; exports labels to /tmp/carslop-engomi-labels.json"]
    fn downloaded_engomi_businesses_have_storefronts() {
        let labels = source_location::read_legacy_labels(std::path::Path::new(
            "src-locations/relation-17716182",
        ))
        .unwrap();
        for (id, expected) in [
            (1019204039, "Stephanis Electronics"),
            (1019204039, "EuroBank"),
            (1019204039, "Cytashop"),
            (1019204034, "Orchestra"),
            (1019204038, "Pahit Ice"),
            (1019204038, "Εν ελλαδι"),
        ] {
            assert!(
                labels.storefronts.contains(&(id, expected.into())),
                "missing {expected} on {id}"
            );
        }
        for expected in ["Art Works", "Dr Xristakis Xristodoulou"] {
            assert!(
                labels.storefronts.iter().any(|(_, name)| name == expected),
                "missing {expected}"
            );
        }
        eprintln!(
            "{} named buildings; {} business storefronts",
            labels.buildings.len(),
            labels.storefronts.len()
        );
        std::fs::write(
            std::env::temp_dir().join("carslop-engomi-labels.json"),
            serde_json::to_vec(&labels).unwrap(),
        )
        .unwrap();
    }

    #[test]
    fn repeated_light_barrier_contacts_preserve_sliding_speed() {
        // Exercise a rotated wall so the response cannot depend on world axes.
        let normal = XZ { x: 0.6, z: 0.8 };
        let tangent = XZ { x: -0.8, z: 0.6 };
        let mut speed = 20.0;
        for _ in 0..120 {
            let mut velocity = XZ::from_fn(|i| {
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
        let mut velocity = XZ { x: 10.0, z: 0.0 };
        assert_eq!(
            deflect(&mut velocity, XZ { x: 1.0, z: 0.0 }),
            10.0
        );
        assert!((velocity.x + 0.8).abs() < 0.00001);
        assert_eq!(velocity.z, 0.0);
        let departing = velocity;
        assert_eq!(
            deflect(&mut velocity, XZ { x: 1.0, z: 0.0 }),
            0.0
        );
        assert_eq!(velocity, departing);
    }

    const CROSS:&str="CITY 2\norigin 0 0\nbounds -50 -50 50 50\nn -40 0\nn 0 0\nn 40 0\nn 0 -40\nn 0 40\nr 0 1 8 0 1\nr 1 2 8 0 1\nr 3 1 8 0 2\nr 1 4 8 0 2\nb 1 10 1 4 8 8 18 8 18 18 8 18\n";

    #[test]
    fn branches_are_open_and_dead_ends_do_not_join_other_streets() {
        let city = test_support::parse_world(CROSS).unwrap();
        for i in 0..4 {
            for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
                let s = city.sample(i, t);
                let mut p = s.vehicle_position(0.0);
                let before = p;
                city.constrain(&mut p, &mut XZ::default());
                assert_eq!(p, before);
            }
        }
        assert!(
            city.road_clearance(geometry::XYZ {
                x: 25.0,
                y: 0.0,
                z: 25.0
            }) > 10.0
        );
        let mut p = geometry::XYZ {
            x: 0.0,
            y: 0.48,
            z: 45.0,
        };
        city.constrain(&mut p, &mut XZ { x: 0.0, z: 10.0 });
        assert!(p.z <= 42.61);
    }
    #[test]
    fn building_contacts_and_camera_respect_actual_footprints() {
        let collision = test_support::parse_world(
            &CROSS.replace("8 8 18 8 18 18 8 18", "8 -4 18 -4 18 4 8 4"),
        )
        .unwrap();
        let mut position = geometry::XYZ {
            x: 10.0,
            y: 0.48,
            z: 0.0,
        };
        let mut velocity = XZ { x: 10.0, z: 0.0 };
        assert!(collision.constrain(&mut position, &mut velocity) > 0.0);
        assert!(!collision.buildings()[0].contains(XZ {
            x: position.x,
            z: position.z
        }));
        let city = test_support::parse_world(CROSS).unwrap();
        let b = &city.buildings()[0];
        assert!(b.contains(XZ { x: 10.0, z: 10.0 }));
        assert!(!b.contains(XZ { x: 7.0, z: 10.0 }));
        let eye = city.camera_eye(
            geometry::XYZ {
                x: 0.0,
                y: 1.0,
                z: 10.0,
            },
            geometry::XYZ {
                x: 25.0,
                y: 4.0,
                z: 10.0,
            },
        );
        assert!(eye.x < 7.6);
        assert_eq!(
            city.camera_eye(
                geometry::XYZ {
                    x: 0.0,
                    y: 12.0,
                    z: 10.0
                },
                geometry::XYZ {
                    x: 25.0,
                    y: 14.0,
                    z: 10.0
                }
            ),
            geometry::XYZ {
                x: 25.0,
                y: 14.0,
                z: 10.0
            }
        );
    }
    #[test]
    fn engomi_ground_contact_props_and_foundations_share_elevation() {
        let city_track = test_support::nicosia_world();
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
                        - city.height(XZ {
                            x: sample.point.x,
                            z: sample.point.z
                        }))
                    .abs()
                        < 0.001
                );
                let mut p = sample.vehicle_position(0.7);
                city.constrain(&mut p, &mut XZ::default());
                assert!(
                    (p.y - city.drive_height(XZ { x: p.x, z: p.z }) - 0.48).abs()
                        < 0.001
                );
            }
        }
        assert!(max - min > 20.0 && max - min < 1000.0);
        for prop in &city.furniture {
            assert!(
                (prop.coords.y
                    - city.height(XZ {
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
        let city_track = test_support::nicosia_world();
        let city = &city_track;
        let mut checked = 0;
        for triangle in city.road_triangles() {
            if !matches!(
                triangle.usage,
                world::TriangleUsage::Pavement(world::CurbStyle::MajorRoad)
                    | world::TriangleUsage::Pavement(world::CurbStyle::Road)
            ) {
                continue;
            }
            let q = XZ {
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
            let mut p = geometry::XYZ {
                x: q.x,
                y: 0.0,
                z: q.z,
            };
            let mut velocity = XZ { x: 0.0, z: 0.0 };
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
            if triangle.material != world::Material::Asphalt {
                continue;
            }
            let q = XZ {
                x: triangle.points.iter().map(|p| p.x).sum::<f32>() / 3.0,
                z: triangle.points.iter().map(|p| p.z).sum::<f32>() / 3.0,
            };
            // Land cover is layered underneath streets; test exposed parking.
            if city.paving_height(q).is_some() || city.building_clearance(q) < CAR_RADIUS + 0.1 {
                continue;
            }
            let mut p = geometry::XYZ {
                x: q.x,
                y: 0.0,
                z: q.z,
            };
            city.constrain(&mut p, &mut XZ::default());
            assert!((p.x - q.x).hypot(p.z - q.z) < 0.001);
            assert!(city.paved(q));
            parking += 1;
            if parking == 20 {
                break;
            }
        }
        assert_eq!(parking, 20);
        let mut p = geometry::XYZ {
            x: city.bounding_box().max.x + 10.0,
            y: 0.0,
            z: 0.0,
        };
        let mut velocity = XZ { x: 10.0, z: 0.0 };
        assert!(city.constrain(&mut p, &mut velocity) > 0.0);
        assert!(p.x <= city.bounding_box().max.x - CAR_RADIUS && velocity.x <= 0.0);
    }

    #[test]
    fn urban_map_covers_centres_on_both_sides_and_queries_only_nearby_streets() {
        let city_track = test_support::nicosia_world();
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
            let p = geometry::XYZ {
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
        let intersects = |a: geometry::XYZ, b: geometry::XYZ| {
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
        let city = test_support::parse_world(CROSS).unwrap();
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

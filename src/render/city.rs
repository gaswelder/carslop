use crate::build;
use crate::build::osm_labels::Building;
use crate::model;
use crate::model::map_details::RoofShape;
use crate::model::map_details::TreeKind;
use crate::model::world::WorldLocation;
use crate::render;
use crate::render::building_parts::Catalog;
use crate::render::building_parts::Tints;
use crate::render::scene::Builder;
use crate::render::scene::Surface;
use crate::ui;
use crate::util;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;
use geo::TriangulateEarcut;
use std::ptr;

fn cross(a: XZ<f32>, b: XZ<f32>, c: XZ<f32>) -> f64 {
    (b.x as f64 - a.x as f64) * (c.z as f64 - a.z as f64)
        - (b.z as f64 - a.z as f64) * (c.x as f64 - a.x as f64)
}
/// Ear clipping preserves concave footprints rather than filling their courtyards.
fn roof_triangles(polygon: &[XZ<f32>]) -> Vec<[usize; 3]> {
    let mut indices: Vec<_> = (0..polygon.len()).collect();
    let mut triangles = Vec::new();
    while indices.len() > 3 {
        let ear = (0..indices.len()).find(|&i| {
            let a = indices[(i + indices.len() - 1) % indices.len()];
            let b = indices[i];
            let c = indices[(i + 1) % indices.len()];
            cross(polygon[a], polygon[b], polygon[c]) > 0.0001
                && !indices.iter().copied().any(|n| {
                    n != a
                        && n != b
                        && n != c
                        && cross(polygon[a], polygon[b], polygon[n]) >= -0.0001
                        && cross(polygon[b], polygon[c], polygon[n]) >= -0.0001
                        && cross(polygon[c], polygon[a], polygon[n]) >= -0.0001
                })
        });
        if let Some(i) = ear {
            triangles.push([
                indices[(i + indices.len() - 1) % indices.len()],
                indices[i],
                indices[(i + 1) % indices.len()],
            ]);
            indices.remove(i);
        } else {
            // Remove a redundant collinear vertex before considering this invalid.
            if let Some(i) = (0..indices.len()).find(|&i| {
                cross(
                    polygon[indices[(i + indices.len() - 1) % indices.len()]],
                    polygon[indices[i]],
                    polygon[indices[(i + 1) % indices.len()]],
                )
                .abs()
                    < 0.0001
            }) {
                indices.remove(i);
            } else {
                break;
            }
        }
    }
    if indices.len() == 3 {
        triangles.push([indices[0], indices[1], indices[2]]);
    }
    triangles
}

pub fn buildings(b: &mut Builder, city: &WorldLocation, detailed: bool) -> std::io::Result<()> {
    let models = render::building_parts::load_default()?;
    for building in world::buildings::buildings(city) {
        building_mesh(b, building, city, detailed, &models);
    }
    Ok(())
}
fn terrain_mesh(
    b: &mut Builder,
    field: &world::elevation::HeightField,
    boundary: Option<&geo::MultiPolygon<f64>>,
    openings: Option<&build::surfaces::PolygonIndex<'_>>,
) {
    use geo::{BooleanOps, Contains, Intersects, TriangulateEarcut};
    for row in 0..field.rows - 1 {
        for col in 0..field.columns - 1 {
            let mut p = [
                world::elevation::point(field, col, row),
                world::elevation::point(field, col + 1, row),
                world::elevation::point(field, col + 1, row + 1),
                world::elevation::point(field, col, row + 1),
            ];
            for v in &mut p {
                v.y -= 0.02;
            }
            if boundary.is_some() || openings.is_some() {
                let cell = geo::MultiPolygon(vec![util::geometry::polygon(&p.map(|v| XZ {
                    x: v.x as f64,
                    z: v.z as f64,
                }))]);
                if boundary.is_some_and(|boundary| !boundary.intersects(&cell)) {
                    continue;
                }
                let contained = boundary.is_none_or(|boundary| boundary.contains(&cell));
                let nearby = openings.map(|openings| build::surfaces::nearby(openings, &cell));
                let nearby = nearby.as_ref().filter(|nearby| !nearby.0.is_empty());
                if !contained || nearby.is_some() {
                    // Preserve the height field's diagonal and its two planar faces.
                    for indices in [[0, 1, 2], [0, 2, 3]] {
                        let face = util::geometry::polygon(&indices.map(|i| XZ {
                            x: p[i].x as f64,
                            z: p[i].z as f64,
                        }));
                        let mut clipped = if contained {
                            geo::MultiPolygon(vec![face])
                        } else {
                            boundary.unwrap().intersection(&face)
                        };
                        if let Some(nearby) = nearby {
                            clipped = clipped.difference(nearby);
                        }
                        for polygon in clipped {
                            for triangle in polygon.earcut_triangles() {
                                let points =
                                    [triangle.v1(), triangle.v2(), triangle.v3()].map(|c| {
                                        let x = c.x as f32;
                                        let z = c.y as f32;
                                        XYZ {
                                            x,
                                            y: world::elevation::height(field, XZ { x, z }) - 0.02,
                                            z,
                                        }
                                    });
                                render::scene::upward_quad(
                                    b,
                                    [(points[0]), (points[1]), (points[2]), (points[0])],
                                    render::scene::surface_road([0.64, 0.63, 0.54]),
                                );
                            }
                        }
                    }
                    continue;
                }
            }
            for t in [[0, 1, 2], [0, 2, 3]] {
                render::scene::upward_quad(
                    b,
                    [p[t[0]], p[t[1]], p[t[2]], p[t[0]]],
                    render::scene::surface_road([0.64, 0.63, 0.54]),
                );
            }
        }
    }
}
// Distant tunnel openings must not trigger boolean clipping or retriangulation.
fn drape_ground_cover(
    field: &world::elevation::HeightField,
    points: [XZ<f32>; 3],
    openings: Option<&build::surfaces::PolygonIndex<'_>>,
    mut emit: impl FnMut([XYZ; 3]),
) {
    let Some(openings) = openings else {
        world::elevation::drape(field, points, emit);
        return;
    };
    let footprint = geo::MultiPolygon(vec![util::geometry::polygon(
        &points.map(|p| util::geometry::xz_map(p, f64::from)),
    )]);
    let nearby = build::surfaces::nearby(openings, &footprint);
    if nearby.0.is_empty() {
        world::elevation::drape(field, points, emit);
        return;
    }
    use geo::BooleanOps;
    for face in footprint
        .difference(&nearby)
        .0
        .iter()
        .flat_map(|p| p.earcut_triangles())
    {
        world::elevation::drape(
            field,
            face.to_array().map(|p| XZ {
                x: p.x as f32,
                z: p.y as f32,
            }),
            &mut emit,
        );
    }
}

pub fn ground_mesh(
    b: &mut Builder,
    city: &WorldLocation,
    boundary: Option<&geo::MultiPolygon<f64>>,
) {
    let field = model::world::elevation(city).expect("city rendering requires elevation");
    let openings = render::scene::generation_stage("tunnel openings", || {
        world::transport::tunnel_openings(city)
    });
    let opening_index = build::surfaces::new(&openings);
    let opening_index = (!openings.0.is_empty()).then_some(&opening_index);
    // Retain the terrain extent used for imports without an explicit area boundary.
    let terrain_boundary = (boundary.is_none() && opening_index.is_some()).then(|| {
        let bounds = world::elevation::terrain_bounds(city);
        geo::MultiPolygon(vec![util::geometry::polygon(&[
            util::geometry::xz_map(bounds.min, f64::from),
            XZ {
                x: bounds.max.x as f64,
                z: bounds.min.z as f64,
            },
            util::geometry::xz_map(bounds.max, f64::from),
            XZ {
                x: bounds.min.x as f64,
                z: bounds.max.z as f64,
            },
        ])])
    });
    render::scene::generation_stage("terrain", || {
        terrain_mesh(
            b,
            field,
            boundary.or(terrain_boundary.as_ref()),
            opening_index,
        );
    });
    render::scene::generation_stage("ground cover", || {
        for triangle in model::world::ground_triangles(city) {
            let mut emit = |mut p: [XYZ; 3]| {
                for v in &mut p {
                    v.y += 0.005;
                }
                render::scene::upward_quad(
                    b,
                    [p[0], p[1], p[2], p[0]],
                    render::scene::surface_road(render::scene::color(triangle.material)),
                );
            };
            drape_ground_cover(field, triangle.points, opening_index, &mut emit);
        }
    });
    render::scene::generation_stage("trees", || {
        for tree in model::world::trees(city) {
            mapped_tree(b, city, tree);
        }
    });
    let pavement = world::roads::pavement_height_meters(city);
    render::scene::generation_stage("roads and pavement", || {
        assert!(
            world::street_surface::has_compiled_roads(city),
            "city rendering requires compiled street surfaces"
        );
        // The compiler partitions these surfaces, so gutters and street classes
        // meet along shared edges rather than overlapping coplanar strips.
        for triangle in world::roads::road_triangles(city) {
            let height = render_height(triangle.usage, pavement);
            let mut emit = |mut p: [XYZ; 3]| {
                for v in &mut p {
                    v.y += height;
                }
                render::scene::upward_quad(
                    b,
                    [p[0], p[1], p[2], p[0]],
                    render::scene::surface_road(render::scene::color(triangle.material)),
                );
            };
            if matches!(triangle.usage, model::world::TriangleUsage::Pavement(_)) {
                world::raised_roads::drape_road_triangle(
                    city,
                    triangle.points.map(Into::into),
                    &mut emit,
                );
            } else {
                world::raised_roads::drape_road_triangle(
                    city,
                    triangle.points.map(Into::into),
                    &mut emit,
                );
            }
        }
    });
    render::scene::generation_stage("curbs", || {
        for curb in world::roads::curbs(city) {
            let [a, c] = curb.ends;
            let top = if curb.style == model::world::CurbStyle::ServiceRoad {
                0.08
            } else {
                pavement
            };
            for pair in world::raised_roads::raised_road_segment(city, a, c).windows(2) {
                let a = pair[0];
                let c = pair[1];
                render::scene::quad(
                    b,
                    [
                        XYZ {
                            x: a.x,
                            y: a.y + 0.05,
                            z: a.z,
                        },
                        XYZ {
                            x: c.x,
                            y: c.y + 0.05,
                            z: c.z,
                        },
                        XYZ {
                            x: c.x,
                            y: (world::elevation::world_height(city, util::geometry::xz(c)) + top)
                                .max(c.y + 0.05),
                            z: c.z,
                        },
                        XYZ {
                            x: a.x,
                            y: (world::elevation::world_height(city, util::geometry::xz(a)) + top)
                                .max(a.y + 0.05),
                            z: a.z,
                        },
                    ],
                    render::scene::matte([0.77, 0.75, 0.66]),
                );
            }
        }
    });
    render::scene::generation_stage("street furniture", || {
        for feature in &city.furniture {
            render::road_furniture::build(b, feature);
        }
    });
}

// Stable independent samples keep neighbouring OSM IDs from repeating a pattern.
fn visual_seed(id: u64, salt: u64) -> u64 {
    let mut value = id.wrapping_add(salt.wrapping_mul(0x9e3779b97f4a7c15));
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d049bb133111eb);
    value ^ (value >> 31)
}
fn variation(id: u64, salt: u64) -> f32 {
    (visual_seed(id, salt) >> 40) as f32 / 16777215.0
}
fn tinted(color: [f32; 3], gain: f32) -> [f32; 3] {
    color.map(|c| (c * gain).clamp(0.0, 1.0))
}

// Entrance nodes often split a straight wall. Merge those segments for facade
// details while keeping the surveyed footprint for the shell and collision.
fn facade_edges(outline: &[XZ<f32>]) -> Vec<[XZ<f32>; 2]> {
    let mut corners = Vec::new();
    for (i, point) in outline.iter().enumerate() {
        let before = outline[(i + outline.len() - 1) % outline.len()];
        let after = outline[(i + 1) % outline.len()];
        let length = (after.x - before.x).hypot(after.z - before.z);
        if length < 0.0001 || cross(before, *point, after).abs() / length as f64 > 0.02 {
            corners.push(*point);
        }
    }
    let points = if corners.len() >= 3 {
        &corners
    } else {
        outline
    };
    (0..points.len())
        .map(|i| [points[i], points[(i + 1) % points.len()]])
        .collect()
}

struct Door {
    wall: usize,
    center: f32,
    bottom: f32,
}
struct DoorCandidate {
    placement: Door,
    distance: f32,
}
fn door_placements(
    building: &Building,
    city: &WorldLocation,
    edges: &[[XZ<f32>; 2]],
    models: &Catalog,
) -> Vec<Door> {
    if building.min_height_meters > 0.0 {
        return Vec::new();
    }
    let entrances = world::buildings::building_entrances(city, building.id);
    if entrances.is_empty() {
        if !world::buildings::primary_building_volume(city, building.id)
            .is_some_and(|primary| ptr::eq(primary, building))
        {
            return Vec::new();
        }
        if building.height_meters <= models.placement.entrance_min_height {
            return Vec::new();
        }
        return edges
            .iter()
            .enumerate()
            .find_map(|(wall, [a, c])| {
                let length = (c.x - a.x).hypot(c.z - a.z);
                (length > models.placement.entrance_min_wall).then_some(Door {
                    wall,
                    center: length * 0.5,
                    bottom: building.base_y + 0.02,
                })
            })
            .into_iter()
            .collect();
    }
    if building.height_meters < models.entrance.size.y + 0.04 {
        return Vec::new();
    }
    let margin = models.entrance.size.x / 2.0 + 0.02;
    let mut doors: Vec<Door> = Vec::new();
    for position in entrances {
        if !world::buildings::entrance_building_volume(city, building.id, *position)
            .is_some_and(|owner| ptr::eq(owner, building))
        {
            continue;
        }
        let candidate = edges
            .iter()
            .enumerate()
            .filter_map(|(wall, [a, c])| {
                let length = (c.x - a.x).hypot(c.z - a.z);
                if length < margin * 2.0 {
                    return None;
                }
                let center = (((position.x - a.x) * (c.x - a.x)
                    + (position.z - a.z) * (c.z - a.z))
                    / length)
                    .clamp(margin, length - margin);
                let point = XZ {
                    x: a.x + (c.x - a.x) * center / length,
                    z: a.z + (c.z - a.z) * center / length,
                };
                let distance = (position.x - point.x).hypot(position.z - point.z);
                (distance <= 0.75).then_some(DoorCandidate {
                    placement: Door {
                        wall,
                        center,
                        bottom: world::elevation::world_height(city, point)
                            + world::street_surface::paving_height(city, point).unwrap_or(0.02)
                            + 0.01,
                    },
                    distance,
                })
            })
            .min_by(|a, b| a.distance.total_cmp(&b.distance));
        if let Some(candidate) = candidate {
            let door = candidate.placement;
            if !doors.iter().any(|other| {
                other.wall == door.wall
                    && (other.center - door.center).abs() < models.entrance.size.x * 0.9
            }) {
                doors.push(door);
            }
        }
    }
    doors
}

fn building_mesh(
    b: &mut Builder,
    building: &Building,
    city: &WorldLocation,
    detailed: bool,
    models: &Catalog,
) {
    let mapped = world::buildings::building_style(city, building);
    let roof_rise = mapped.map_or(0.0, |style| style.roof_height);
    let wall_height = building.height_meters - roof_rise;
    // These are regional art variations, rather than surveyed OSM materials.
    let nicosia = city.origin.is_some_and(|origin| {
        (35.10..35.24).contains(&origin.lat) && (33.24..33.48).contains(&origin.lon)
    });
    let palette = if nicosia {
        [
            [0.97, 0.96, 0.94], // white plaster
            [0.95, 0.94, 0.91], // warm white concrete
            [0.91, 0.88, 0.81], // pale limestone
            [0.96, 0.93, 0.86], // ivory
        ]
    } else {
        [
            [0.94, 0.93, 0.90], // white concrete
            [0.94, 0.90, 0.80], // cream
            [0.83, 0.77, 0.65], // beige
            [0.95, 0.90, 0.68], // pale yellow
        ]
    };
    let id = building.id;
    let seed = visual_seed(id, 1);
    let wall_color = mapped
        .and_then(|style| style.wall_color)
        .unwrap_or_else(|| {
            tinted(
                palette[seed as usize % palette.len()],
                0.95 + variation(id, 2) * 0.08,
            )
        });
    let wall = render::scene::matte(wall_color);
    let roof = render::scene::matte(mapped.and_then(|style| style.roof_color).unwrap_or_else(
        || {
            if nicosia {
                tinted([0.86, 0.85, 0.81], 0.94 + variation(id, 3) * 0.08)
            } else {
                [
                    [0.64, 0.65, 0.62],
                    [0.77, 0.75, 0.68],
                    [0.58, 0.62, 0.63],
                    [0.71, 0.64, 0.56],
                    [0.69, 0.57, 0.48],
                ][visual_seed(id, 3) as usize % 5]
            }
        },
    ));
    let height = building.base_y + wall_height;
    // Generate the complete shared shell first. Near detail never changes the
    // mapped footprint and roof outline. Balcony furniture is near detail.
    let bottom = building.base_y + building.min_height_meters;
    for ring in model::world::rings(building) {
        for i in 0..ring.len() {
            let a = ring[i];
            let c = ring[(i + 1) % ring.len()];
            for pair in world::elevation::ground_segment(city, a, c).windows(2) {
                let a = pair[0];
                let c = pair[1];
                let a_bottom = if building.min_height_meters > 0.0 {
                    bottom
                } else {
                    a.y - 0.03
                };
                let c_bottom = if building.min_height_meters > 0.0 {
                    bottom
                } else {
                    c.y - 0.03
                };
                render::scene::quad(
                    b,
                    [
                        XYZ { y: c_bottom, ..c },
                        XYZ { y: a_bottom, ..a },
                        XYZ { y: height, ..a },
                        XYZ { y: height, ..c },
                    ],
                    wall,
                );
            }
        }
    }
    if let Some(style) = mapped.filter(|style| style.shape != RoofShape::Flat) {
        render::roofs::build(b, building, style, roof, wall);
    } else if building.holes.is_empty() {
        for triangle in roof_triangles(&building.ground_outline) {
            let p = triangle.map(|i| util::geometry::at_height(building.ground_outline[i], height));
            render::scene::upward_quad(b, [p[0], p[1], p[2], p[0]], roof);
        }
    } else {
        for triangle in model::world::footprint(building).earcut_triangles() {
            let p = triangle.to_array().map(|p| XYZ {
                x: p.x as f32,
                y: height,
                z: p.y as f32,
            });
            render::scene::upward_quad(b, [p[0], p[1], p[2], p[0]], roof);
        }
    }
    if building.min_height_meters > 0.0 {
        for triangle in model::world::footprint(building).earcut_triangles() {
            let p = triangle.to_array().map(|p| XYZ {
                x: p.x as f32,
                y: bottom,
                z: p.y as f32,
            });
            render::scene::quad(b, [p[2], p[1], p[0], p[0]], wall);
        }
    }
    if !detailed {
        return;
    }

    // Four coherent facade families: shuttered plaster, banded apartments,
    // wider modern glazing, and modest punched windows.
    let style = visual_seed(id, 4) % 4;
    let trim = render::scene::matte(tinted(
        wall_color,
        if nicosia {
            0.97
        } else if style == 2 {
            0.69
        } else {
            0.85
        },
    ));
    let frame = render::scene::matte(
        [
            [0.73, 0.71, 0.63],
            [0.33, 0.37, 0.37],
            [0.49, 0.42, 0.33],
            [0.83, 0.81, 0.73],
        ][visual_seed(id, 5) as usize % 4],
    );
    let shutter = render::scene::matte(
        [
            [0.27, 0.35, 0.29],
            [0.33, 0.39, 0.43],
            [0.45, 0.31, 0.23],
            [0.54, 0.52, 0.40],
        ][visual_seed(id, 6) as usize % 4],
    );
    let spacing = if style == 2 {
        models.placement.modern_window_spacing
    } else {
        let [min, max] = models.placement.window_spacing;
        min + variation(id, 7) * (max - min)
    };
    let storey = if nicosia {
        // OSM heights commonly use 3.1 m per level. Fit complete storeys to the
        // measured height so rounding cannot leave a blank top floor.
        (wall_height - building.min_height_meters)
            / ((wall_height - building.min_height_meters) / 3.1)
                .round()
                .max(1.0)
    } else {
        2.85 + variation(id, 8) * 0.55
    };
    let window_width = if style == 2 {
        1.7
    } else {
        0.95 + variation(id, 9) * 0.65
    };
    let window_height = if nicosia && (style == 1 || style == 2) {
        1.95
    } else {
        1.15 + variation(id, 10) * 0.6
    };
    let mut tints = Tints {
        wall: wall_color,
        frame: frame.color,
        trim: trim.color,
        shutter: shutter.color,
        glass: [1.0; 3],
    };
    let edges: Vec<_> = model::world::rings(building)
        .flat_map(facade_edges)
        .collect();
    let doors: Vec<_> = door_placements(building, city, &edges, models)
        .into_iter()
        .filter(|door| door.bottom + models.entrance.size.y < height - 0.02)
        .collect();
    for (i, [a, c]) in edges.iter().copied().enumerate() {
        let length = (c.x - a.x).hypot(c.z - a.z);
        if length < 3.0 {
            continue;
        }
        let normal = XZ {
            x: (c.z - a.z) / length,
            z: -(c.x - a.x) / length,
        };
        // Coordinates are metres along the wall and absolute elevation.
        let panel = |b: &mut Builder,
                     left: f32,
                     right: f32,
                     bottom: f32,
                     top: f32,
                     depth: f32,
                     surface: Surface| {
            let p = |along: f32, y: f32| XYZ {
                x: a.x + (c.x - a.x) * along / length + normal.x * depth,
                y: y,
                z: a.z + (c.z - a.z) * along / length + normal.z * depth,
            };
            render::scene::quad(
                b,
                [
                    (p(right, bottom)),
                    (p(left, bottom)),
                    (p(left, top)),
                    (p(right, top)),
                ],
                surface,
            );
        };
        panel(b, 0.0, length, height - 0.24, height - 0.04, 0.018, trim);
        panel(b, 0.0, length, bottom + 0.06, bottom + 0.40, 0.018, trim);
        let columns = ((length - 1.0) / spacing).floor().clamp(1.0, 20.0) as usize;
        let half = (window_width * 0.5).min(length / columns as f32 * 0.29);
        for floor in 0..(((wall_height - building.min_height_meters) / storey + 0.001).floor()
            as usize)
            .min(16)
        {
            let bottom =
                building.base_y + building.min_height_meters + 0.85 + floor as f32 * storey;
            let top = (bottom + window_height).min(height - 0.45);
            if top <= bottom + 0.3 {
                continue;
            }
            if style == 1 && floor > 0 {
                panel(b, 0.0, length, bottom - 0.42, bottom - 0.29, 0.02, trim);
            }
            for column in 0..columns {
                let center = (column as f32 + 0.5) * length / columns as f32;
                if floor == 0
                    && doors.iter().any(|door| {
                        door.wall == i
                            && (center - door.center).abs()
                                < half + models.entrance.size.x * 0.5 + 0.45
                            && bottom < door.bottom + models.entrance.size.y + 0.1
                    })
                {
                    continue;
                }
                let key = id ^ ((i as u64) << 32) ^ ((floor as u64) << 16) ^ column as u64;
                tints.glass = tinted([0.16, 0.24, 0.27], 0.75 + variation(key, 11) * 0.65);
                render::building_parts::place(
                    &models.window,
                    b,
                    [a, c],
                    center,
                    bottom - 0.07,
                    XYZ {
                        x: half * 2.0 + 0.14,
                        y: top - bottom + 0.14,
                        z: models.window.size.z,
                    },
                    &tints,
                );
                if style == 0 && length / columns as f32 > window_width + 0.8 {
                    for side in [-1.0, 1.0] {
                        let edge = center + side * (half + 0.24);
                        panel(b, edge - 0.15, edge + 0.15, bottom, top, 0.045, shutter);
                    }
                } else if variation(key, 12) > 0.70 {
                    let blind = render::scene::matte(tinted(wall_color, 0.72));
                    panel(
                        b,
                        center - half,
                        center + half,
                        top - (top - bottom) * (0.20 + variation(key, 13) * 0.45),
                        top,
                        0.045,
                        blind,
                    );
                }
            }
        }
        // Broad stacked terraces on two substantial exterior faces. Keep them
        // off concave footprints, where an outward projection could fill a court.
        let convex = (0..building.ground_outline.len()).all(|j| {
            cross(
                building.ground_outline[j],
                building.ground_outline[(j + 1) % building.ground_outline.len()],
                building.ground_outline[(j + 2) % building.ground_outline.len()],
            ) >= -0.0001
        });
        if nicosia
            && convex
            && (style == 1 || style == 2)
            && wall_height >= models.placement.balcony_min_height
            && length >= models.placement.balcony_min_wall
            && i < models.placement.balcony_max_walls
        {
            let mut facade = building.clone();
            facade.height_meters = wall_height;
            apartment_balconies(b, a, c, &facade, storey, style, models, &tints);
        }
    }
    for door in doors {
        render::building_parts::place(
            &models.entrance,
            b,
            edges[door.wall],
            door.center,
            door.bottom,
            models.entrance.size,
            &tints,
        );
    }
}

fn apartment_balconies(
    b: &mut Builder,
    a: XZ<f32>,
    c: XZ<f32>,
    building: &Building,
    storey: f32,
    style: u64,
    models: &Catalog,
    tints: &Tints,
) {
    let length = (c.x - a.x).hypot(c.z - a.z);
    let rules = &models.placement;
    let width = (length * rules.balcony_width_ratio).min(rules.balcony_max_width);
    let depth = rules.balcony_depth[0]
        + variation(building.id, 21) * (rules.balcony_depth[1] - rules.balcony_depth[0]);
    let model = &models.balconies[usize::from(style == 2)];
    for floor in 1..(((building.height_meters - building.min_height_meters) / storey + 0.001)
        .floor() as usize)
        .min(16)
    {
        let y = building.base_y
            + building.min_height_meters
            + floor as f32 * storey
            + rules.balcony_floor_offset;
        if y + rules.balcony_headroom >= building.base_y + building.height_meters {
            continue;
        }
        render::building_parts::place(
            model,
            b,
            [a, c],
            length * 0.5,
            y,
            XYZ {
                x: width,
                y: model.size.y,
                z: depth,
            },
            tints,
        );
    }
}

// Smooth normals and baked foliage shading give the crown volume without an
// extra texture, draw call or runtime shader work. Shared positions use identical
// colors, including the seam between the first and last sector.
fn canopy_quad(
    b: &mut Builder,
    points: [XYZ; 4],
    center: XYZ,
    vertical_scale: f32,
    color: [f32; 3],
) {
    for indices in [[0, 1, 2], [0, 2, 3]] {
        let p = indices.map(|i| points[i]);
        if p[0] == p[1] || p[1] == p[2] || p[2] == p[0] {
            continue;
        }
        let vertices = p.map(|position| {
            let delta = XYZ {
                x: position.x - center.x,
                y: (position.y - center.y) * vertical_scale,
                z: position.z - center.z,
            };
            let length = util::geometry::iter(&delta)
                .map(|n| n * n)
                .sum::<f32>()
                .sqrt()
                .max(0.00001);
            let normal = util::geometry::xyz_map(delta, |n| n / length);
            let mottling = (position.x * 2.3 + position.z * 1.7 + position.y * 3.1).sin();
            let shade = 0.90 + 0.14 * normal.y + 0.055 * mottling;
            render::scene::Vertex {
                position: position.into(),
                normal: normal.into(),
                color: tinted(color, shade),
                material: [0.0, 0.0],
            }
        });
        if let Some(chunks) = &mut b.chunks {
            render::chunks::push(chunks, vertices);
        } else {
            b.vertices.extend(vertices);
        }
    }
}

#[derive(Clone, Copy)]
struct CrownRing {
    height: f32,
    radius: f32,
}

fn mapped_tree(b: &mut Builder, city: &WorldLocation, tree: &world::landscape::Tree) {
    let XZ { x, z } = tree.coords;
    let y = world::elevation::world_height(city, XZ { x, z });
    let h = tree.height_meters;
    let r = tree.crown_radius_meters;
    let id = tree.id;
    let inferred = id >= (1_u64 << 63);
    let kind = model::world::tree_kind(city, id);
    let family = match kind {
        Some(TreeKind::Conifer | TreeKind::Cypress) => 2,
        Some(TreeKind::Olive) => 3,
        Some(TreeKind::Broadleaf) => 0,
        _ => visual_seed(id, 20) % 4,
    };
    let rotation = variation(id, 21) * std::f32::consts::TAU;
    let lean = XZ {
        x: (variation(id, 22) - 0.5) * r * 0.16,
        z: (variation(id, 23) - 0.5) * r * 0.16,
    };
    let trunk = world::trees::mapped_trunk(city, tree);
    let trunk_radius = trunk.radius;
    let bark = render::scene::matte(tinted([0.38, 0.31, 0.23], 0.78 + variation(id, 25) * 0.35));
    // A tapered six-sided trunk has the same triangle budget as the old cuboid.
    for i in 0..6 {
        let a = rotation + i as f32 * std::f32::consts::TAU / 6.0;
        let c = rotation + (i + 1) as f32 * std::f32::consts::TAU / 6.0;
        let p = |angle: f32, top: bool| {
            let scale = if top { 0.55 } else { 1.0 };
            XYZ {
                x: x + angle.cos() * trunk_radius * scale + if top { lean.x } else { 0.0 },
                y: y + if top {
                    trunk.top_offset
                } else {
                    trunk.base_offset
                },
                z: z + angle.sin() * trunk_radius * scale + if top { lean.z } else { 0.0 },
            }
        };
        render::scene::quad(
            b,
            [(p(a, false)), (p(a, true)), (p(c, true)), (p(c, false))],
            bark,
        );
    }
    if kind == Some(TreeKind::Palm) {
        let green = render::scene::matte([0.22, 0.38, 0.18]);
        for frond in 0..10 {
            let angle = rotation + frond as f32 * std::f32::consts::TAU / 10.0;
            let center = XYZ {
                x: x + lean.x,
                y: y + h * 0.88,
                z: z + lean.z,
            };
            let middle = XYZ {
                x: x + lean.x + angle.cos() * r * 0.55,
                y: y + h,
                z: z + lean.z + angle.sin() * r * 0.55,
            };
            let tip = XYZ {
                x: x + lean.x + angle.cos() * r * 0.90,
                y: y + h * (0.74 + variation(id, frond + 40) * 0.08),
                z: z + lean.z + angle.sin() * r * 0.90,
            };
            let side = XZ {
                x: -angle.sin() * r * 0.14,
                z: angle.cos() * r * 0.14,
            };
            for segment in 0..2 {
                let pair = [[center, middle], [middle, tip]][segment];
                let widths = [[0.12, 1.0], [1.0, 0.02]][segment];
                render::scene::upward_quad(
                    b,
                    [
                        XYZ {
                            x: pair[0].x - side.x * widths[0],
                            z: pair[0].z - side.z * widths[0],
                            ..pair[0]
                        },
                        XYZ {
                            x: pair[1].x - side.x * widths[1],
                            z: pair[1].z - side.z * widths[1],
                            ..pair[1]
                        },
                        XYZ {
                            x: pair[1].x + side.x * widths[1],
                            z: pair[1].z + side.z * widths[1],
                            ..pair[1]
                        },
                        XYZ {
                            x: pair[0].x + side.x * widths[0],
                            z: pair[0].z + side.z * widths[0],
                            ..pair[0]
                        },
                    ],
                    render::scene::matte(tinted(
                        green.color,
                        if segment == 0 { 1.08 } else { 0.83 },
                    )),
                );
            }
        }
        return;
    }
    let rings: &[CrownRing] = match kind {
        Some(TreeKind::Conifer) => &[
            CrownRing {
                height: 0.24,
                radius: 0.0,
            },
            CrownRing {
                height: 0.32,
                radius: 0.94,
            },
            CrownRing {
                height: 0.57,
                radius: 0.63,
            },
            CrownRing {
                height: 0.78,
                radius: 0.37,
            },
            CrownRing {
                height: 1.0,
                radius: 0.0,
            },
        ],
        Some(TreeKind::Cypress) => &[
            CrownRing {
                height: 0.16,
                radius: 0.0,
            },
            CrownRing {
                height: 0.30,
                radius: 0.48,
            },
            CrownRing {
                height: 0.57,
                radius: 0.55,
            },
            CrownRing {
                height: 0.82,
                radius: 0.32,
            },
            CrownRing {
                height: 1.0,
                radius: 0.0,
            },
        ],
        _ => &[
            CrownRing {
                height: 0.26,
                radius: 0.0,
            },
            CrownRing {
                height: 0.39,
                radius: 0.43,
            },
            CrownRing {
                height: 0.61,
                radius: 0.55,
            },
            CrownRing {
                height: 0.83,
                radius: 0.35,
            },
            CrownRing {
                height: 1.0,
                radius: 0.0,
            },
        ],
    };
    // Imagery trees keep their four-ring budget even with a mapped species.
    let sparse_rings = [
        rings[0],
        rings[1],
        rings[rings.len() - 2],
        rings[rings.len() - 1],
    ];
    let rings = if inferred { &sparse_rings[..] } else { rings };
    let palette = [
        [0.22, 0.35, 0.17],
        [0.28, 0.38, 0.22],
        [0.17, 0.29, 0.16],
        [0.31, 0.40, 0.24],
        [0.24, 0.34, 0.20],
        [0.19, 0.33, 0.20],
    ];
    let base = tinted(
        if kind == Some(TreeKind::Olive) {
            [0.36, 0.44, 0.30]
        } else {
            palette[visual_seed(id, 26) as usize % palette.len()]
        },
        (0.89 + variation(id, 27) * 0.19)
            * if model::world::tree_evergreen(city, id) {
                0.91
            } else {
                1.0
            },
    );
    if family != 2 {
        // Three overlapping foliage masses replace the single balloon-shaped
        // crown. Each mass costs 20 triangles (8 for imagery trees), preserving
        // the old 60/24-triangle canopy budget and mapped clearance envelope.
        let segments = if inferred { 4 } else { 5 };
        let equators = [
            CrownRing {
                height: -1.0,
                radius: 0.0,
            },
            CrownRing {
                height: -0.38,
                radius: 0.92,
            },
            CrownRing {
                height: 0.38,
                radius: 0.92,
            },
            CrownRing {
                height: 1.0,
                radius: 0.0,
            },
        ];
        let sparse = [
            equators[0],
            CrownRing {
                height: 0.0,
                radius: 1.0,
            },
            equators[3],
        ];
        let profile = if inferred { &sparse[..] } else { &equators[..] };
        for lobe in 0..3 {
            let angle = rotation + lobe as f32 * std::f32::consts::TAU / 3.0;
            let top = lobe == 2;
            let half_height = h * if top { 0.28 } else { 0.25 };
            let radius = r * (0.61 + variation(id, 60 + lobe) * 0.04);
            let center = XYZ {
                x: x + angle.cos() * r * 0.28 + lean.x * 0.4,
                y: y + h * if top { 0.72 } else { 0.55 + lobe as f32 * 0.05 },
                z: z + angle.sin() * r * 0.28 + lean.z * 0.4,
            };
            let color = tinted(base, 0.95 + lobe as f32 * 0.055);
            for pair in profile.windows(2) {
                for sector in 0..segments {
                    let point = |sector: usize, ring: CrownRing| {
                        let angle = rotation
                            + (sector % segments) as f32 * std::f32::consts::TAU / segments as f32;
                        XYZ {
                            x: center.x + angle.cos() * radius * ring.radius,
                            y: center.y + half_height * ring.height,
                            z: center.z + angle.sin() * radius * ring.radius,
                        }
                    };
                    canopy_quad(
                        b,
                        [
                            point(sector, pair[0]),
                            point(sector, pair[1]),
                            point(sector + 1, pair[1]),
                            point(sector + 1, pair[0]),
                        ],
                        center,
                        (radius / half_height).powi(2),
                        color,
                    );
                }
            }
        }
        return;
    }
    let segments = if inferred { 6 } else { 10 };
    let oval = 0.72 + variation(id, 28) * 0.24;
    let (rotation_sin, rotation_cos) = rotation.sin_cos();
    let crown_bottom = rings.first().unwrap().height;
    let crown_top = rings.last().unwrap().height;
    let crown_center = XYZ {
        x: x + lean.x * 0.2,
        y: y + h * (crown_bottom + crown_top) * 0.5,
        z: z + lean.z * 0.2,
    };
    let vertical_scale = (r / (h * (crown_top - crown_bottom) * 0.5)).powi(2);
    for pair in rings.windows(2) {
        for i in 0..segments {
            let p = |index: usize, ring: CrownRing| {
                let sector = (index % segments) as u64;
                let noise = variation(id ^ sector.wrapping_mul(0x9e3779b97f4a7c15), 29);
                // Offset the rings and scallop their edges to break the lathed
                // silhouette. Tips stay shared and the mapped envelope is kept.
                let angle = sector as f32 * std::f32::consts::TAU / segments as f32
                    + (ring.height * 9.0).sin() * 0.16;
                let ripple = 0.80 + noise * 0.18;
                let radius = r * ring.radius * ripple;
                let drift = (ring.height - 0.3).max(0.0);
                let height = ring.height
                    + if ring.radius > 0.0 {
                        (noise - 0.5) * 0.075
                    } else {
                        0.0
                    };
                let local = XZ {
                    x: angle.cos() * radius,
                    z: angle.sin() * radius * oval,
                };
                XYZ {
                    x: x + local.x * rotation_cos - local.z * rotation_sin + lean.x * drift,
                    y: y + h * height,
                    z: z + local.x * rotation_sin + local.z * rotation_cos + lean.z * drift,
                }
            };
            let a = p(i, pair[0]);
            let c = p(i, pair[1]);
            let d = p(i + 1, pair[1]);
            let e = p(i + 1, pair[0]);
            canopy_quad(b, [(a), (c), (d), (e)], crown_center, vertical_scale, base);
        }
    }
}

fn render_height(triangle_usage: model::world::TriangleUsage, pavement_height: f32) -> f32 {
    match triangle_usage {
        model::world::TriangleUsage::Marking => 0.07,
        _ => model::world::support_height(triangle_usage, pavement_height),
    }
}

#[cfg(test)]
#[path = "tests/city.rs"]
mod tests;

/// Runtime signs also work with existing baked city archives.
pub fn storefront_signs(b: &mut Builder, city: &WorldLocation) {
    for building in world::buildings::buildings(city) {
        if !world::buildings::primary_building_volume(city, building.id)
            .is_some_and(|primary| ptr::eq(primary, building))
        {
            continue;
        }
        let names = model::world::storefront_names(city, building.id);
        if names.is_empty() {
            continue;
        }
        // Put the shopfront on the wall closest to the street network.
        let edge = (0..building.ground_outline.len())
            .filter_map(|i| {
                let a = building.ground_outline[i];
                let c = building.ground_outline[(i + 1) % building.ground_outline.len()];
                let length = (c.x - a.x).hypot(c.z - a.z);
                if length < 2.0 {
                    return None;
                }
                let midpoint = XYZ {
                    x: (a.x + c.x) * 0.5,
                    y: building.base_y,
                    z: (a.z + c.z) * 0.5,
                };
                let road = world::roads::project(city, midpoint);
                let distance = (midpoint.x - road.point.x).hypot(midpoint.z - road.point.z);
                Some((distance, a, c))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0));
        let Some((_, a, c)) = edge else {
            continue;
        };
        // Keep narrow frontages readable by stacking tenants in additional rows.
        let length = (c.x - a.x).hypot(c.z - a.z);
        let columns = names.len().min((length / 2.0).floor().max(1.0) as usize);
        let rows = names.len().div_ceil(columns);
        let height = building.height_meters / rows as f32;
        for (i, name) in names.iter().enumerate() {
            let label: Vec<_> = name.to_uppercase().chars().take(80).collect();
            let point = |t: f32| XZ {
                x: a.x + (c.x - a.x) * t,
                z: a.z + (c.z - a.z) * t,
            };
            let column = i % columns;
            let start = point(column as f32 / columns as f32);
            let end = point((column + 1) as f32 / columns as f32);
            storefront_sign(
                b,
                &label,
                [start, end],
                building.base_y + (i / columns) as f32 * height,
                height,
                storefront_seed(building.id, name),
            );
        }
    }
}

// Hash the tenant name as well as the building ID so neighbours sharing a wall
// differ, and adding another tenant does not change the existing tenants' styles.
fn storefront_seed(building: u64, name: &str) -> u64 {
    name.bytes().fold(visual_seed(building, 40), |seed, byte| {
        (seed ^ u64::from(byte)).wrapping_mul(0x100000001b3)
    })
}

// Paired colors preserve contrast for both dark and light signboards.
const STOREFRONT_PALETTES: [([f32; 3], [f32; 3]); 8] = [
    ([0.035, 0.09, 0.10], [0.96, 0.93, 0.80]),
    ([0.32, 0.035, 0.025], [1.0, 0.94, 0.84]),
    ([0.025, 0.07, 0.24], [0.92, 0.96, 1.0]),
    ([0.035, 0.19, 0.085], [0.97, 0.94, 0.79]),
    ([0.075, 0.065, 0.055], [0.93, 0.66, 0.25]),
    ([0.88, 0.82, 0.66], [0.12, 0.07, 0.045]),
    ([0.90, 0.91, 0.88], [0.055, 0.12, 0.19]),
    ([0.70, 0.37, 0.055], [0.065, 0.045, 0.025]),
];

// X runs toward the viewer's right on the outward face of the wall.
fn storefront_sign(
    b: &mut Builder,
    label: &[char],
    [a, c]: [XZ<f32>; 2],
    base_y: f32,
    height: f32,
    seed: u64,
) {
    let (background, lettering) =
        STOREFRONT_PALETTES[visual_seed(seed, 41) as usize % STOREFRONT_PALETTES.len()];
    let length = (c.x - a.x).hypot(c.z - a.z);
    let normal = XZ {
        x: (c.z - a.z) / length,
        z: -(c.x - a.x) / length,
    };
    let (lines, pixel) = storefront_layout(label, length, height);
    let pixel = pixel * (0.78 + variation(seed, 42) * 0.22);
    let width = lines
        .iter()
        .map(|line| ui::font::width(ui::font::labels(), &line.iter().collect::<String>()))
        .fold(0.0_f32, f32::max)
        * pixel;
    let alignment = [0.0, 0.5, 1.0][visual_seed(seed, 43) as usize % 3];
    let panel_width = if visual_seed(seed, 44) % 2 == 0 {
        width + 2.0 * pixel
    } else {
        length - 0.3
    };
    let panel_left = (length - panel_width) * 0.5;
    let panel_height = lines.len() as f32 * 9.0 * pixel;
    let top = base_y
        + (panel_height + 0.8 + variation(seed, 45) * 0.9).min(height - (height * 0.1).min(0.3));
    let panel_point = |x: f32, y: f32, depth: f32| XYZ {
        x: c.x + (a.x - c.x) * x / length + normal.x * depth,
        y,
        z: c.z + (a.z - c.z) * x / length + normal.z * depth,
    };
    let mut panel = |x0: f32, x1: f32, y0: f32, y1: f32, depth: f32, color| {
        let point = |x, y| panel_point(x, y, depth);
        render::scene::quad(
            b,
            [
                (point(x0, y0)),
                (point(x1, y0)),
                (point(x1, y1)),
                (point(x0, y1)),
            ],
            Surface {
                color,
                material: [-3.0, 0.0],
            },
        );
    };
    panel(
        panel_left,
        panel_left + panel_width,
        top - panel_height,
        top,
        0.14,
        background,
    );
    for (line, chars) in lines.iter().enumerate() {
        let label: String = chars.iter().collect();
        let line_width = ui::font::width(ui::font::labels(), &label) * pixel;
        let line_left = panel_left + pixel + (panel_width - 2.0 * pixel - line_width) * alignment;
        ui::font::draw(ui::font::labels(), &label, |triangle| {
            render::scene::emit_triangle(
                b,
                triangle.map(|p| {
                    panel_point(
                        line_left + p[0] * pixel,
                        top - (1.0 + line as f32 * 9.0 + p[1]) * pixel,
                        0.15,
                    )
                }),
                Surface {
                    color: lettering,
                    material: [-3.0, 0.0],
                },
            );
        });
    }
}

fn storefront_layout(label: &[char], length: f32, height: f32) -> (Vec<Vec<char>>, f32) {
    let wrap = |columns: usize| {
        let mut lines = Vec::new();
        let mut start = 0;
        while start < label.len() {
            while start < label.len() && label[start].is_whitespace() {
                start += 1;
            }
            if start == label.len() {
                break;
            }
            let mut end = (start + columns).min(label.len());
            if end < label.len() {
                if let Some(space) = label[start..end].iter().rposition(|c| c.is_whitespace()) {
                    if space > 0 {
                        end = start + space;
                    }
                }
            }
            lines.push(label[start..end].to_vec());
            start = end;
        }
        lines
    };
    let mut columns = ((length - 0.6) / (6.0 * 0.25)).floor().max(6.0) as usize;
    let mut lines = wrap(columns);
    while lines.len() > 3 {
        columns += 1;
        lines = wrap(columns);
    }
    let widest = lines
        .iter()
        .map(|line| ui::font::width(ui::font::labels(), &line.iter().collect::<String>()))
        .fold(1.0_f32, f32::max);
    let pixel = 0.25_f32
        .min((length - 0.6) / (widest + 2.0))
        .min((height - 0.6).max(height * 0.5).min(3.8) / (lines.len().max(1) as f32 * 9.0));
    (lines, pixel)
}

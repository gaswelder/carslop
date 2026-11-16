//! Tests through the group's production interfaces.

mod map_and_selection {
    use crate::ui::hud::MenuAction;
    use crate::ui::map::{FULL_MAP_CONTENT, MAP_CONTENT};
    use crate::util::geometry::XYZ;
    use crate::world::locations::LocationId;
    use crate::{model, physics, test, ui, world};
    fn location_hit(x: f32, y: f32, selected: usize, count: usize) -> Option<MenuAction> {
        ui::hud::selection_menu_hit_test(x, y, selected, count, 0, 0)
    }
    #[test]
    fn map_zoom_factor_applies_to_every_location() {
        let track = world::locations::try_track(&LocationId::Compact).unwrap();
        let other = world::locations::try_track(&test::support::authored_id()).unwrap();
        let viewport = [1280.0, 720.0];

        let mut view = ui::map::full_map_view_new(
            &track,
            physics::vehicle_location::spawn(&track).point,
            viewport,
        );
        assert!(ui::map::zoom_full_map(&mut (view), 3.0));
        let factor = ui::map::zoom_factor(&view);
        assert!(factor > 1.0);

        for track in [&track, &other] {
            let mut view = ui::map::full_map_view_new(
                track,
                physics::vehicle_location::spawn(track).point,
                viewport,
            );
            assert!((ui::map::zoom_factor(&(view)) - 1.0).abs() < 0.000001);
            ui::map::set_zoom_factor(&mut view, factor);
            assert!((ui::map::zoom_factor(&(view)) - factor).abs() < 0.00001);
        }
    }
    #[test]
    fn minimap_heading_keeps_forward_up_and_queries_rotated_corners() {
        let track = world::locations::try_track(&LocationId::Compact).unwrap();
        for heading in [0.0, 0.7, 1.57, 3.14, -2.0] {
            let mut fit = ui::map::map_fit_new(
                &track,
                XYZ {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                [1280.0, 720.0],
            );
            fit.heading = Some(heading);
            let center = ui::map::map_fit_point(
                &fit,
                XYZ {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            );
            let forward = ui::map::map_fit_point(
                &fit,
                XYZ {
                    x: heading.sin() * 30.0,
                    y: 0.0,
                    z: heading.cos() * 30.0,
                },
            );
            assert!((forward[0] - center[0]).abs() < 0.00001);
            assert!(forward[1] > center[1]);
            let camera_right = ui::map::map_fit_point(
                &fit,
                XYZ {
                    x: -heading.cos() * 30.0,
                    y: 0.0,
                    z: heading.sin() * 30.0,
                },
            );
            assert!(camera_right[0] > center[0]);
            assert!((camera_right[1] - center[1]).abs() < 0.00001);
            let bounds = ui::map::world_bounds(&fit);
            for x in [MAP_CONTENT[0][0], MAP_CONTENT[1][0]] {
                for y in [MAP_CONTENT[0][1], MAP_CONTENT[1][1]] {
                    let right = (x - center[0]) / fit.scale[0];
                    let ahead = (y - center[1]) / fit.scale[1];
                    let world = [
                        -right * heading.cos() + ahead * heading.sin(),
                        right * heading.sin() + ahead * heading.cos(),
                    ];
                    for i in 0..2 {
                        assert!(
                            world[i] >= [bounds.min.x, bounds.min.z][i] - 0.001
                                && world[i] <= [bounds.max.x, bounds.max.z][i] + 0.001
                        );
                    }
                }
            }
        }
    }
    #[test]
    fn full_map_zoom_matches_minimap_and_can_fit_every_road() {
        for id in test::support::location_ids() {
            let track = world::locations::try_track(&id).unwrap();
            for player in [
                physics::vehicle_location::spawn(&track).point,
                XYZ {
                    x: model::world::bounding_box(&track).max.x + 100.0,
                    y: 0.0,
                    z: model::world::bounding_box(&track).max.z + 100.0,
                },
            ] {
                for viewport in [[1280.0, 720.0], [1024.0, 768.0], [2560.0, 1080.0]] {
                    let mini = ui::map::map_fit_new(&track, player, viewport);
                    let mut view = ui::map::full_map_view_new(&track, player, viewport);
                    assert_eq!(view.fit.scale, mini.scale);

                    let center = ui::map::map_fit_point(&view.fit, player);
                    assert!(center[0].abs() < 0.00001);
                    assert!((center[1] - 0.015).abs() < 0.00001);
                    assert!(ui::map::zoom_full_map(&mut (view), 1.0));
                    assert!((view.fit.scale[0] / mini.scale[0] - 1.25).abs() < 0.00001);
                    assert_eq!(ui::map::map_fit_point(&(view.fit), player), center);
                    assert!(ui::map::zoom_full_map(&mut (view), -1.0));
                    assert!((view.fit.scale[0] - mini.scale[0]).abs() < 0.00001);
                    assert!(ui::map::zoom_full_map(&mut (view), 100.0));
                    assert!(!ui::map::zoom_full_map(&mut (view), 1.0));

                    assert!(ui::map::zoom_full_map(&mut (view), -100.0));
                    assert!(!ui::map::zoom_full_map(&mut (view), -1.0));

                    for invalid in [0.0, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
                        assert!(!ui::map::zoom_full_map(&mut (view), invalid));
                    }
                    assert_eq!(ui::map::map_fit_point(&(view.fit), player), center);
                    let fit = &view.fit;
                    for p in
                        world::roads::road_segments_in(&track, model::world::bounding_box(&track))
                            .into_iter()
                            .flat_map(|(a, b)| [a, b])
                            .chain([player])
                    {
                        let point = ui::map::map_fit_point(fit, p);
                        for axis in 0..2 {
                            assert!(point[axis] > FULL_MAP_CONTENT[0][axis]);
                            assert!(point[axis] < FULL_MAP_CONTENT[1][axis]);
                        }
                    }
                    let center = ui::map::map_fit_point(fit, player);
                    let east = ui::map::map_fit_point(
                        fit,
                        XYZ {
                            x: player.x + 100.0,
                            y: 0.0,
                            z: player.z,
                        },
                    );
                    let south = ui::map::map_fit_point(
                        fit,
                        XYZ {
                            x: player.x,
                            y: 0.0,
                            z: player.z + 100.0,
                        },
                    );
                    let width = (east[0] - center[0]) * viewport[0];
                    let height = (south[1] - center[1]).abs() * viewport[1];
                    assert!((width - height).abs() < 0.001);
                    assert_eq!(south[1] < center[1], id == test::support::city_id());
                }
            }
        }
    }
    #[test]
    fn location_pages_hit_only_visible_entries() {
        assert_eq!(
            location_hit(-0.5, 0.76, 6, 6 + 1),
            Some(MenuAction::Choice(6))
        );
        assert_eq!(location_hit(-0.5, 0.66, 6, 6 + 1), None);
        assert_eq!(location_hit(-0.5, 0.71, 0, 4), None);
        assert_eq!(location_hit(-0.5, 0.91, 0, 4), None);
        assert_eq!(location_hit(-0.5, 0.56, 0, 4), Some(MenuAction::Choice(2)));
        assert_eq!(location_hit(-0.5, -0.07, 0, 4), None);
    }
    #[test]
    fn car_pages_select_visible_rows_and_ignore_empty_rows() {
        let car_count = 6 + 2;
        for selected in 0..car_count {
            for slot in 0..6 {
                let top = 0.80 - slot as f32 * 0.10;
                let r = [-0.16, top - 0.08, 0.56, top];
                let index = selected / 6 * 6 + slot;
                let expected = (index < car_count).then_some(MenuAction::Car(index));
                assert_eq!(
                    ui::hud::selection_menu_hit_test(
                        0.5,
                        (r[1] + r[3]) * 0.5,
                        0,
                        4,
                        selected,
                        car_count
                    ),
                    expected
                );
            }
            assert_eq!(
                ui::hud::selection_menu_hit_test(0.5, 0.91, 0, 4, selected, car_count),
                None
            );
        }
    }
}

mod font {
    use crate::ui;
    use crate::ui::font::*;

    #[test]
    fn bundled_weights_keep_counters_open_and_measure_proportional_text() {
        for font in [labels(), digits()] {
            for ch in "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789éö".chars() {
                let mut triangles = Vec::new();
                ui::font::draw(font, &ch.to_string(), |triangle| triangles.push(triangle));
                let advance = ui::font::width(font, &ch.to_string());
                assert!(advance > 0.0);
                assert!(!triangles.is_empty(), "missing outline for {ch}");
                assert!(triangles.iter().flatten().flatten().all(|n| n.is_finite()));
            }
            let advance = ui::font::width(font, "O");
            let mut triangles = Vec::new();
            ui::font::draw(font, "O", |triangle| triangles.push(triangle));
            let center = [advance * 0.5, 3.5];
            for triangle in &triangles {
                let cross = |a: [f32; 2], b: [f32; 2]| {
                    (b[0] - a[0]) * (center[1] - a[1]) - (b[1] - a[1]) * (center[0] - a[0])
                };
                let edges = [
                    cross(triangle[0], triangle[1]),
                    cross(triangle[1], triangle[2]),
                    cross(triangle[2], triangle[0]),
                ];
                assert!(
                    edges.iter().any(|n| *n > 0.0) && edges.iter().any(|n| *n < 0.0),
                    "O's counter must stay unfilled"
                );
            }
            assert!(ui::font::width(font, "WWW") > ui::font::width(font, "III"));
            assert!(ui::font::width(font, " ") > 0.0);
            ui::font::draw(font, " ", |_| panic!("spaces must not draw triangles"));
        }
    }

    #[test]
    fn greek_names_are_supported_by_both_weights() {
        for font in [labels(), digits()] {
            for ch in "ΟΔΟΣ ΛΗΔΡΑΣ Λευκωσία ΆΈΉΊΌΎΏάέήίόύώϊϋΐΰ".chars()
            {
                assert!(
                    ui::font::glyph_index(font, ch) != 0,
                    "missing Greek character {ch}"
                );
                if !ch.is_whitespace() {
                    let mut count = 0;
                    ui::font::draw(font, &ch.to_string(), |_| count += 1);
                    assert!(count > 0);
                }
            }
        }
    }
}

mod hud_atlas {
    use crate::ui;
    use crate::ui::hud_atlas::*;

    #[test]
    fn both_weights_pack_greek_latin_and_spaces_without_overlap() {
        let atlas = get();
        assert_eq!(atlas.pixels.len(), atlas.width * atlas.height);
        for font in [&atlas.labels, &atlas.digits] {
            let mut count = 0;
            ui::hud_atlas::draw(font, "A Ωά 09", |sprite| {
                count += 1;
                for p in sprite.uv {
                    assert!(p.iter().all(|n| (0.0..=1.0).contains(n)));
                }
                assert!(sprite.bounds[1][0] > sprite.bounds[0][0]);
                assert!(sprite.bounds[1][1] > sprite.bounds[0][1]);
            });
            assert_eq!(count, 5);
            let characters: String = (32..=255)
                .chain(0x370..=0x3ff)
                .filter_map(char::from_u32)
                .collect();
            ui::hud_atlas::draw(font, &characters, |sprite| {
                let [left, top] = sprite.uv[0];
                let left = (left * atlas.width as f32).round() as usize;
                let top = (top * atlas.height as f32).round() as usize;
                let right = (sprite.uv[1][0] * atlas.width as f32).round() as usize;
                let bottom = (sprite.uv[1][1] * atlas.height as f32).round() as usize;
                assert!(right < atlas.width && bottom < atlas.height);
                assert_eq!(atlas.pixels[top * atlas.width + left - 1], 0);
                assert_eq!(atlas.pixels[top * atlas.width + right], 0);
                assert!(atlas.pixels
                    [(top - 1) * atlas.width + left..(top - 1) * atlas.width + right]
                    .iter()
                    .all(|n| *n == 0));
            });
        }
    }
}

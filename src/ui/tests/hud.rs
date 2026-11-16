use super::*;
use crate::physics;
use crate::test;
use crate::ui;
use crate::ui::map::FULL_MAP_CONTENT;
use crate::world;

#[test]
#[ignore = "reports HUD generation time and geometry size"]
fn hud_geometry_profile() {
    let mut vertices = Vec::new();
    let mut triangles = 0;
    let cold = std::time::Instant::now();
    let atlas = ui::hud_atlas::get();
    eprintln!(
        "Atlas: {}x{}, {:.3} s to initialize",
        atlas.width,
        atlas.height,
        cold.elapsed().as_secs_f64()
    );
    let started = std::time::Instant::now();
    for frame in 0..1000 {
        vertices.clear();
        append_instruments(
            &mut vertices,
            Gauges {
                rpm: 3000.0 + frame as f32,
                redline_rpm: 6500.0,
                speed_kmh: 90.0,
                gear: 3,
            },
            [1280.0, 720.0],
        );
        append_street_name(&mut vertices, "ΛΕΩΦΟΡΟΣ ΑΡΧΙΕΠΙΣΚΟΠΟΥ ΜΑΚΑΡΙΟΥ III");
        std::hint::black_box(&vertices);
        triangles = vertices.len() / 3;
    }
    eprintln!(
        "HUD: {triangles} triangles, {} bytes, {:.3} ms/frame",
        std::mem::size_of_val(vertices.as_slice()),
        started.elapsed().as_secs_f64()
    );
}

#[test]
fn full_map_zoom_clips_road_strokes_inside_the_panel() {
    let track = world::locations::try_track(&test::support::authored_id()).unwrap();
    let player = physics::vehicle_location::spawn(&track).point;
    for viewport in [[1280.0, 720.0], [1024.0, 768.0], [2560.0, 1080.0]] {
        for steps in [-100.0, 0.0, 100.0] {
            let mut view = ui::map::full_map_view_new(&track, player, viewport);
            ui::map::zoom_full_map(&mut view, steps);
            let mut vertices = Vec::new();
            append_full_map(
                &mut vertices,
                Map {
                    track: &track,
                    player,
                },
                0.0,
                viewport,
                &view.fit,
            );
            let roads: Vec<_> = vertices
                .iter()
                .filter(|p| p.color == [0.40, 0.55, 0.60])
                .map(|p| p.position)
                .collect();
            assert!(!roads.is_empty());
            for p in roads {
                for axis in 0..2 {
                    assert!(p[axis] >= FULL_MAP_CONTENT[0][axis] - 0.00001);
                    assert!(p[axis] <= FULL_MAP_CONTENT[1][axis] + 0.00001);
                }
            }
        }
    }
}

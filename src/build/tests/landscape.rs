use super::*;
use crate::build;
use geo::{Area, Coord, Rect};

#[test]
fn overlapping_exclusions_do_not_leave_path_patches_inside_obstacles() {
    let path = MultiPolygon(vec![Rect::new(
        Coord { x: 0.0, y: 0.0 },
        Coord { x: 10.0, y: 2.0 },
    )
    .to_polygon()]);
    let obstacles = MultiPolygon(vec![
        Rect::new(Coord { x: 2.0, y: -1.0 }, Coord { x: 6.0, y: 3.0 })
            .to_polygon()
            .orient(Direction::Reversed),
        Rect::new(Coord { x: 4.0, y: -1.0 }, Coord { x: 8.0, y: 3.0 }).to_polygon(),
    ]);
    let result = exclude_obstacles(&path, &build::surfaces::new(&obstacles));
    assert!((result.unsigned_area() - 8.0).abs() < 1e-8);
    for obstacle in &obstacles.0 {
        assert!(result.intersection(obstacle).unsigned_area() < 1e-8);
    }
}

#[test]
fn exclusions_preserve_courtyards_except_where_another_obstacle_covers_them() {
    let path = MultiPolygon(vec![Rect::new(
        Coord { x: 0.0, y: 0.0 },
        Coord { x: 10.0, y: 2.0 },
    )
    .to_polygon()]);
    let building = geo::Polygon::new(
        Rect::new(Coord { x: 2.0, y: -1.0 }, Coord { x: 8.0, y: 3.0 })
            .to_polygon()
            .exterior()
            .clone(),
        vec![
            Rect::new(Coord { x: 3.0, y: 0.5 }, Coord { x: 7.0, y: 1.5 })
                .to_polygon()
                .exterior()
                .clone(),
        ],
    );
    let road = Rect::new(Coord { x: 5.0, y: -1.0 }, Coord { x: 6.0, y: 3.0 }).to_polygon();
    let obstacles = MultiPolygon(vec![building, road]);
    let result = exclude_obstacles(&path, &build::surfaces::new(&obstacles));
    assert!((result.unsigned_area() - 11.0).abs() < 1e-8);
    assert!(result.contains(&Point::new(4.0, 1.0)));
    assert!(!result.contains(&Point::new(5.5, 1.0)));
    assert!(!result.contains(&Point::new(2.5, 1.0)));
}

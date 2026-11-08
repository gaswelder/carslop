//! Map-space exclusion of proposed tree centres, using the actual road mesh.
use crate::location::{invalid, Result};
use geo::{Coord, Intersects, LineString, Point, Polygon};
use rstar::{RTree, RTreeObject, AABB};
use std::{
    io::{BufRead, BufReader},
    path::Path,
};
struct Obstacle {
    polygon: Polygon<f64>,
    bounds: AABB<[f64; 2]>,
    reason: &'static str,
}
impl RTreeObject for Obstacle {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        self.bounds
    }
}
pub struct Obstacles {
    index: RTree<Obstacle>,
}
fn local(origin: [f64; 2], lon: f64, lat: f64) -> [f64; 2] {
    [
        (lon - origin[1]).to_radians() * 6371000. * origin[0].to_radians().cos(),
        -(lat - origin[0]).to_radians() * 6371000.,
    ]
}
fn obstacle(
    points: Vec<[f64; 2]>,
    reason: &'static str,
    region: &AABB<[f64; 2]>,
) -> Option<Obstacle> {
    let lo = std::array::from_fn(|i| points.iter().map(|p| p[i]).fold(f64::INFINITY, f64::min));
    let hi = std::array::from_fn(|i| {
        points
            .iter()
            .map(|p| p[i])
            .fold(f64::NEG_INFINITY, f64::max)
    });
    let bounds = AABB::from_corners(lo, hi);
    use rstar::Envelope;
    if !bounds.intersects(region) {
        return None;
    }
    let ring = LineString::new(
        points
            .into_iter()
            .map(|p| Coord { x: p[0], y: p[1] })
            .collect(),
    );
    Some(Obstacle {
        polygon: Polygon::new(ring, vec![]),
        bounds,
        reason,
    })
}
impl Obstacles {
    pub(crate) fn read_paths(
        city_path: &Path,
        streets_path: &Path,
        lonlat_bounds: [f64; 4],
    ) -> Result<Self> {
        let text = std::fs::read_to_string(city_path)?;
        let city = crate::city::City::parse(&text).map_err(invalid)?;
        let origin_line = text
            .lines()
            .find(|l| l.starts_with("origin "))
            .ok_or_else(|| invalid("city missing origin"))?;
        let mut parts = origin_line.split_whitespace().skip(1);
        let origin = [
            parts
                .next()
                .ok_or_else(|| invalid("missing origin latitude"))?
                .parse::<f64>()?,
            parts
                .next()
                .ok_or_else(|| invalid("missing origin longitude"))?
                .parse::<f64>()?,
        ];
        let nw = local(origin, lonlat_bounds[0], lonlat_bounds[3]);
        let se = local(origin, lonlat_bounds[2], lonlat_bounds[1]);
        let region = AABB::from_corners(nw, se);
        let mut objects = vec![];
        for building in &city.buildings {
            if let Some(o) = obstacle(
                building
                    .polygon
                    .iter()
                    .map(|p| [p[0] as f64, p[1] as f64])
                    .collect(),
                "building",
                &region,
            ) {
                objects.push(o);
            }
        }
        println!("Checking road surfaces for tree exclusions...");
        let mut input = BufReader::new(std::fs::File::open(streets_path)?);
        let mut line = String::new();
        input.read_line(&mut line)?;
        let expected = format!(
            "STREETS 1 {} {:08x}",
            city.streets.len(),
            crate::street_surface::crc32(&text)
        );
        if line.trim() != expected {
            return Err(invalid(
                "street surfaces do not match city.txt; reimport the map",
            ));
        }
        loop {
            line.clear();
            if input.read_line(&mut line)? == 0 {
                break;
            }
            let mut f = line.split_whitespace();
            if f.next() != Some("t") {
                continue;
            }
            let style = f
                .next()
                .ok_or_else(|| invalid("missing surface style"))?
                .parse::<usize>()?;
            // 0..=3 are asphalt; 4..=7 are pavement/other pedestrian surfaces.
            if style > 3 {
                continue;
            }
            let mut points = vec![];
            for _ in 0..3 {
                let mut p = [0.; 2];
                for v in &mut p {
                    *v = f
                        .next()
                        .ok_or_else(|| invalid("missing surface coordinate"))?
                        .parse::<f64>()?;
                    if !v.is_finite() {
                        return Err(invalid("invalid road coordinate"));
                    }
                }
                points.push(p);
            }
            if f.next().is_some() {
                return Err(invalid("invalid road triangle"));
            }
            let [a, b, c] = [points[0], points[1], points[2]];
            if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) <= 0.00001 {
                continue;
            }
            if let Some(o) = obstacle(points, "road", &region) {
                objects.push(o);
            }
        }
        Ok(Self {
            index: RTree::bulk_load(objects),
        })
    }
    pub fn rejection_at(&self, p: [f64; 2]) -> Option<&'static str> {
        let point = Point::new(p[0], p[1]);
        let mut road = false;
        for o in self
            .index
            .locate_in_envelope_intersecting(&AABB::from_point(p))
        {
            if o.polygon.intersects(&point) {
                if o.reason == "building" {
                    return Some("building");
                }
                road = true;
            }
        }
        if road {
            Some("road")
        } else {
            None
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    #[test]
    fn reads_matching_map_surfaces_and_preserves_pavement() {
        let temp = tempfile::tempdir().unwrap();
        let city="CITY 2\norigin 35.162 33.32\nbounds -50 -50 50 50\nn -20 0\nn 20 0\nr 0 1 8 0 1\nb 1 10 1 4 10 10 20 10 20 20 10 20\n";
        fs::write(temp.path().join("city.txt"), city).unwrap();
        let header = format!("STREETS 1 1 {:08x}\n", crate::street_surface::crc32(city));
        let mesh = "t 0 -20 -2 20 -2 20 2\nt 0 -20 -2 20 2 -20 2\nt 4 -20 3 20 3 20 6\n";
        fs::write(temp.path().join("streets.txt"), format!("{header}{mesh}")).unwrap();
        let o = Obstacles::read_paths(
            &temp.path().join("city.txt"),
            &temp.path().join("streets.txt"),
            [33.319, 35.161, 33.321, 35.163],
        )
        .unwrap();
        let check = |x: f64, z: f64| o.rejection_at([x, z]);
        assert_eq!(check(0., 0.), Some("road"));
        assert_eq!(check(15., 15.), Some("building"));
        assert_eq!(
            check(10., 4.),
            None,
            "pavement must remain available for trees"
        );
        fs::write(temp.path().join("streets.txt"), "STREETS 1 1 00000000\n").unwrap();
        assert!(Obstacles::read_paths(
            &temp.path().join("city.txt"),
            &temp.path().join("streets.txt"),
            [33.319, 35.161, 33.321, 35.163]
        )
        .is_err());
    }
    #[test]
    fn rejects_centres_on_buildings_and_road_junctions_not_overhanging_crowns() {
        let region = AABB::from_corners([-50., -50.], [50., 50.]);
        let objects = [
            obstacle(
                vec![[10., 10.], [20., 10.], [20., 20.], [10., 20.]],
                "building",
                &region,
            )
            .unwrap(),
            obstacle(vec![[-20., -2.], [20., -2.], [20., 2.]], "road", &region).unwrap(),
            obstacle(vec![[-20., -2.], [20., 2.], [-20., 2.]], "road", &region).unwrap(),
        ];
        let o = Obstacles {
            index: RTree::bulk_load(objects.into()),
        };
        let check = |x: f64, z: f64| o.rejection_at([x, z]);
        assert_eq!(check(15., 15.), Some("building"));
        assert_eq!(check(0., 0.), Some("road"));
        assert_eq!(check(10., 10.), Some("building"));
        assert_eq!(check(0., 3.), None);
        assert_eq!(check(9., 15.), None);
    }
}

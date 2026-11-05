//! Immutable authored routes. No global mutable "active track".

use std::sync::OnceLock;

pub const ROAD_WIDTH: f32 = 7.2;
pub const CAR_RADIUS: f32 = 1.4;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrackId {
    Compact,
    Scenic,
    Engomi,
}

pub const ALL: [TrackId; 3] = [TrackId::Compact, TrackId::Scenic, TrackId::Engomi];

// Preserve the original compact route and its sampling exactly.
pub const ROAD_POINTS: [[f32; 3]; 12] = [
    [0.0, 0.0, -35.0],
    [14.0, 1.0, -30.0],
    [25.0, 2.0, -18.0],
    [30.0, 3.0, -2.0],
    [24.0, 2.0, 14.0],
    [10.0, 1.0, 27.0],
    [-8.0, 0.0, 34.0],
    [-24.0, 2.0, 25.0],
    [-31.0, 1.0, 10.0],
    [-27.0, 0.0, -8.0],
    [-16.0, -1.0, -24.0],
    [-4.0, 0.0, -34.0],
];

// Separately authored road: coast straight, climbing bends, ridge, western
// sweep, and a tighter descending return. World units are metres.
const SCENIC_POINTS: &[[f32; 3]] = &[
    [0.0, 3.0, -300.0],
    [160.0, 3.0, -300.0],
    [300.0, 5.0, -250.0],
    [365.0, 10.0, -150.0],
    [350.0, 18.0, -30.0],
    [290.0, 24.0, 60.0],
    [235.0, 28.0, 140.0],
    // Mountain hairpin: a clockwise reversal with a climbing entry.
    [235.0, 28.8, 170.0],
    [238.75, 29.1, 184.0],
    [249.0, 29.4, 194.25],
    [263.0, 29.7, 198.0],
    [277.0, 30.0, 194.25],
    [287.25, 30.3, 184.0],
    [291.0, 30.6, 170.0],
    [291.0, 31.0, 140.0],
    // Opposing tight bend returns to a fast, open ridge sweep.
    [298.0, 31.6, 122.0],
    [316.0, 32.0, 115.0],
    [334.0, 32.4, 122.0],
    [341.0, 32.8, 140.0],
    [341.0, 32.0, 195.0],
    [300.0, 30.5, 270.0],
    [200.0, 29.0, 310.0],
    [80.0, 22.0, 345.0],
    [-50.0, 14.0, 330.0],
    [-140.0, 9.0, 265.0],
    [-210.0, 7.0, 200.0],
    [-300.0, 5.0, 190.0],
    [-370.0, 4.0, 110.0],
    [-390.0, 4.0, -20.0],
    [-345.0, 7.0, -120.0],
    [-270.0, 12.0, -160.0],
    [-210.0, 14.0, -230.0],
    [-160.0, 10.0, -295.0],
    [-70.0, 5.0, -320.0],
];

pub struct Track {
    pub id: TrackId,
    pub name: &'static str,
    pub width: f32,
    pub bounds: [[f32; 2]; 2], // min/max XZ, from actual spline samples
    #[cfg(test)]
    pub speed_scale: f32,
    pub brake_scale: f32,
    pub drag: f32,
    points: Vec<[f32; 3]>,
    distances: Vec<f32>,
    lengths: Vec<f32>,
    tangents: Vec<[f32; 2]>,
    total: f32,
    shoreline: Vec<[f32; 3]>,
    // Conservative broad-phase cells make long-route projection inexpensive.
    buckets: std::collections::HashMap<(i32, i32), Vec<usize>>,
    city: Option<crate::city::City>,
}

impl TrackId {
    pub fn index(self) -> usize {
        match self {
            Self::Compact => 0,
            Self::Scenic => 1,
            Self::Engomi => 2,
        }
    }

    pub fn track(self) -> std::sync::Arc<Track> {
        self.try_track().expect("cannot load required track")
    }

    pub fn try_track(self) -> Result<std::sync::Arc<Track>, String> {
        use std::sync::{Arc, Mutex};
        static COMPACT: OnceLock<Arc<Track>> = OnceLock::new();
        static SCENIC: OnceLock<Arc<Track>> = OnceLock::new();
        // Retain only the most recent city; sessions own their own references.
        type CityCache = Mutex<Option<(std::path::PathBuf, Arc<Track>)>>;
        static CITY: OnceLock<CityCache> = OnceLock::new();
        // Serialize construction separately: a failed builder cannot poison the cache,
        // and concurrent requests cannot allocate multiple copies of a large city.
        static BUILD: Mutex<()> = Mutex::new(());
        Ok(match self {
            Self::Compact => COMPACT
                .get_or_init(|| Arc::new(Track::build(self, &ROAD_POINTS)))
                .clone(),
            Self::Scenic => SCENIC
                .get_or_init(|| Arc::new(Track::build(self, SCENIC_POINTS)))
                .clone(),
            Self::Engomi => {
                let path = crate::city_assets::directory().to_path_buf();
                let cache = CITY.get_or_init(Default::default);
                if let Some((key, track)) = cache.lock().unwrap().as_ref() {
                    if *key == path {
                        return Ok(track.clone());
                    }
                }
                let _build = BUILD.lock().unwrap_or_else(|error| error.into_inner());
                if let Some((key, track)) = cache.lock().unwrap().as_ref() {
                    if *key == path {
                        return Ok(track.clone());
                    }
                }
                let track = Arc::new(Track::engomi()?);
                *cache.lock().unwrap() = Some((path, track.clone()));
                track
            }
        })
    }
}

#[derive(Debug, Clone, Copy)]
pub struct RoadSample {
    pub point: [f32; 3],
    pub tangent: [f32; 2],
    pub distance: f32,
    pub offset: f32,
    pub width: f32,
}

impl RoadSample {
    pub fn normal(self) -> [f32; 2] {
        [-self.tangent[1], self.tangent[0]]
    }
    pub fn lane_position(self, offset: f32) -> [f32; 3] {
        let normal = self.normal();
        [
            self.point[0] + normal[0] * offset,
            self.point[1] + 0.48,
            self.point[2] + normal[1] * offset,
        ]
    }
    pub fn heading(self) -> f32 {
        self.tangent[0].atan2(self.tangent[1])
    }
}

impl Track {
    #[cfg(test)]
    pub fn flat_test_track(&self) -> Self {
        let mut flat = Self::build(self.id, &ROAD_POINTS.map(|[x, _, z]| [x, 0.0, z]));
        flat.speed_scale = self.speed_scale;
        flat.brake_scale = self.brake_scale;
        flat.drag = self.drag;
        flat
    }

    #[cfg(test)]
    pub fn bump_test_track(&self) -> Self {
        let mut road = self.flat_test_track();
        for (point, distance) in road.points.iter_mut().zip(&road.distances) {
            point[1] = 0.14 * (-((distance - 55.0) / 1.0).powi(2)).exp();
        }
        road
    }

    fn engomi() -> Result<Self, String> {
        let city = crate::city::City::engomi()?;
        Ok(Self {
            id: TrackId::Engomi,
            name: crate::city_assets::name(),
            width: 8.0,
            bounds: city.bounds,
            #[cfg(test)]
            speed_scale: 1.6,
            brake_scale: 0.65,
            drag: 0.004,
            points: city.nodes.clone(),
            distances: Vec::new(),
            lengths: Vec::new(),
            tangents: Vec::new(),
            total: city.length,
            shoreline: Vec::new(),
            buckets: Default::default(),
            city: Some(city),
        })
    }
    pub fn city(&self) -> Option<&crate::city::City> {
        self.city.as_ref()
    }

    /// Ground rise per horizontal metre (XZ). Ignore discrete curb heights:
    /// those already have their own impact response.
    pub fn ground_gradient(&self, position: [f32; 3]) -> [f32; 2] {
        let height = |x, z| {
            self.city.as_ref().map_or_else(
                || self.project([x, position[1], z]).point[1],
                |city| city.height([x, z]),
            )
        };
        let [x, _, z] = position;
        [
            height(x + 0.5, z) - height(x - 0.5, z),
            height(x, z + 0.5) - height(x, z - 0.5),
        ]
    }
    pub fn spawn(&self) -> RoadSample {
        self.city
            .as_ref()
            .map_or_else(|| self.at_distance(1.0), |city| city.spawn())
    }
    pub fn recovery(&self, position: [f32; 3]) -> RoadSample {
        self.city
            .as_ref()
            .map_or_else(|| self.project(position), |city| city.recovery(position))
    }
    pub fn map_segments(&self) -> Box<dyn Iterator<Item = ([f32; 3], [f32; 3])> + '_> {
        if let Some(city) = &self.city {
            Box::new(
                city.streets
                    .iter()
                    .flat_map(|r| r.path.windows(2).map(|p| (p[0], p[1]))),
            )
        } else {
            Box::new(
                (0..self.points.len())
                    .map(|i| (self.points[i], self.points[(i + 1) % self.points.len()])),
            )
        }
    }
    pub fn map_segments_near(
        &self,
        bounds: [[f32; 2]; 2],
    ) -> Box<dyn Iterator<Item = ([f32; 3], [f32; 3])> + '_> {
        if let Some(city) = &self.city {
            Box::new(city.map_segments_in(bounds).into_iter())
        } else {
            self.map_segments()
        }
    }
    fn build(id: TrackId, controls: &[[f32; 3]]) -> Self {
        let mut points = Vec::new();
        let n = controls.len();
        for segment in 0..n {
            let a = controls[(segment + n - 1) % n];
            let b = controls[segment];
            let c = controls[(segment + 1) % n];
            let d = controls[(segment + 2) % n];
            let steps = if id == TrackId::Compact {
                16
            } else {
                (((c[0] - b[0]).hypot(c[2] - b[2])).ceil() as usize).max(16)
            };
            for i in 0..steps {
                let t = i as f32 / steps as f32;
                points.push(std::array::from_fn(|axis| {
                    0.5 * (2.0 * b[axis]
                        + (-a[axis] + c[axis]) * t
                        + (2.0 * a[axis] - 5.0 * b[axis] + 4.0 * c[axis] - d[axis]) * t * t
                        + (-a[axis] + 3.0 * b[axis] - 3.0 * c[axis] + d[axis]) * t * t * t)
                }));
            }
        }
        let mut distances = Vec::with_capacity(points.len());
        let mut lengths = Vec::with_capacity(points.len());
        let mut tangents = Vec::with_capacity(points.len());
        let mut total = 0.0;
        let mut bounds = [[f32::INFINITY; 2], [f32::NEG_INFINITY; 2]];
        let mut buckets = std::collections::HashMap::<_, Vec<usize>>::new();
        for i in 0..points.len() {
            let a = points[i];
            let b = points[(i + 1) % points.len()];
            let length = (b[0] - a[0]).hypot(b[2] - a[2]);
            assert!(
                length.is_finite() && length > 0.0001,
                "degenerate route segment"
            );
            distances.push(total);
            lengths.push(length);
            tangents.push([(b[0] - a[0]) / length, (b[2] - a[2]) / length]);
            total += length;
            for (axis, coordinate) in [a[0], a[2]].into_iter().enumerate() {
                bounds[0][axis] = bounds[0][axis].min(coordinate);
                bounds[1][axis] = bounds[1][axis].max(coordinate);
            }
            // Expand by 32 m; only use this local set when a result is <=32 m.
            for x in ((a[0].min(b[0]) - 32.0) / 32.0).floor() as i32
                ..=((a[0].max(b[0]) + 32.0) / 32.0).floor() as i32
            {
                for z in ((a[2].min(b[2]) - 32.0) / 32.0).floor() as i32
                    ..=((a[2].max(b[2]) + 32.0) / 32.0).floor() as i32
                {
                    buckets.entry((x, z)).or_default().push(i);
                }
            }
        }
        let scenic = id == TrackId::Scenic;
        let mut track = Self {
            id,
            name: if scenic {
                "COASTAL HIGHWAY"
            } else {
                "COASTAL LOOP"
            },
            width: if scenic { 8.0 } else { ROAD_WIDTH },
            bounds,
            #[cfg(test)]
            speed_scale: if scenic { 1.6 } else { 1.0 },
            brake_scale: if scenic { 0.65 } else { 1.0 },
            drag: if scenic { 0.004 } else { 0.009 },
            points,
            distances,
            lengths,
            tangents,
            total,
            shoreline: Vec::new(),
            buckets,
            city: None,
        };
        let count = if scenic {
            (total / 8.0).ceil() as usize
        } else {
            64
        };
        track.shoreline = (0..count)
            .map(|i| {
                let sample = track.at_distance(total * i as f32 / count as f32);
                let n = sample.normal();
                let offset = track.width * 0.5 + if scenic { 22.0 } else { 10.0 };
                [
                    sample.point[0] - n[0] * offset,
                    crate::ambience::SEA_LEVEL,
                    sample.point[2] - n[1] * offset,
                ]
            })
            .collect();
        track
    }

    #[cfg(test)]
    pub fn centerline(&self) -> &[[f32; 3]] {
        &self.points
    }
    pub fn length(&self) -> f32 {
        self.total
    }
    pub fn shoreline(&self) -> &[[f32; 3]] {
        &self.shoreline
    }

    /// Circuit arc length, or enumeration of city street segments. City samples
    /// do not describe one continuous route; navigation must use the network.
    pub fn at_distance(&self, distance: f32) -> RoadSample {
        if let Some(city) = &self.city {
            return city.at_distance(distance);
        }
        let distance = distance.rem_euclid(self.total);
        // The first segment whose end reaches the requested horizontal distance.
        let index = self
            .distances
            .partition_point(|start| *start < distance)
            .saturating_sub(1);
        let t = ((distance - self.distances[index]) / self.lengths[index]).clamp(0.0, 1.0);
        let a = self.points[index];
        let b = self.points[(index + 1) % self.points.len()];
        RoadSample {
            point: std::array::from_fn(|axis| a[axis] + (b[axis] - a[axis]) * t),
            tangent: self.tangents[index],
            distance,
            offset: 0.0,
            width: self.width,
        }
    }

    fn nearest(
        &self,
        position: [f32; 3],
        indices: impl Iterator<Item = usize>,
    ) -> (f32, RoadSample) {
        let mut squared = f32::INFINITY;
        let mut result = self.at_distance(0.0);
        for i in indices {
            let a = self.points[i];
            let b = self.points[(i + 1) % self.points.len()];
            let tangent = self.tangents[i];
            let len = self.lengths[i];
            let t = (((position[0] - a[0]) * tangent[0] + (position[2] - a[2]) * tangent[1]) / len)
                .clamp(0.0, 1.0);
            let point = std::array::from_fn(|axis| a[axis] + (b[axis] - a[axis]) * t);
            let dx = position[0] - point[0];
            let dz = position[2] - point[2];
            let candidate = dx * dx + dz * dz;
            if candidate < squared {
                squared = candidate;
                result = RoadSample {
                    point,
                    tangent,
                    distance: self.distances[i] + t * len,
                    offset: -dx * tangent[1] + dz * tangent[0],
                    width: self.width,
                };
            }
        }
        (squared, result)
    }

    pub fn project(&self, position: [f32; 3]) -> RoadSample {
        if let Some(city) = &self.city {
            return city.project(position);
        }
        if self.id == TrackId::Scenic {
            let cell = (
                (position[0] / 32.0).floor() as i32,
                (position[2] / 32.0).floor() as i32,
            );
            if let Some(indices) = self.buckets.get(&cell) {
                let (distance, sample) = self.nearest(position, indices.iter().copied());
                if distance <= 32.0 * 32.0 {
                    return sample;
                }
            }
        }
        self.nearest(position, 0..self.points.len()).1
    }
}

// Compact-route fixtures retained for existing regression tests, never selected globally.
#[cfg(test)]
pub fn length() -> f32 {
    TrackId::Compact.track().length()
}
#[cfg(test)]
pub fn at_distance(distance: f32) -> RoadSample {
    TrackId::Compact.track().at_distance(distance)
}
#[cfg(test)]
pub fn project(position: [f32; 3]) -> RoadSample {
    TrackId::Compact.track().project(position)
}

/// Normalize a heading difference to [-PI, PI).
#[cfg(test)]
pub fn angle(value: f32) -> f32 {
    (value + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn route_is_closed_and_projection_follows_elevation() {
        assert!(length() > 180.0);
        for distance in [0.0, 20.0, 50.0, 100.0, 160.0] {
            let sample = at_distance(distance);
            let projected = project(sample.lane_position(1.0));
            assert!((projected.point[1] - sample.point[1]).abs() < 0.15);
            assert!((projected.offset - 1.0).abs() < 0.15);
        }
        assert_eq!(at_distance(0.0).point, at_distance(length()).point);
    }

    #[test]
    fn scenic_route_has_real_scale_moderate_grades_and_smooth_hairpins() {
        let scenic = TrackId::Scenic.track();
        assert!(
            (1500.0..3000.0).contains(&scenic.length()),
            "length {}",
            scenic.length()
        );
        assert_eq!(
            scenic.at_distance(0.0).point,
            scenic.at_distance(scenic.length()).point
        );
        assert!(scenic.centerline().len() > 800);
        assert!(!std::sync::Arc::ptr_eq(&scenic, &TrackId::Compact.track()));
        for i in 0..scenic.points.len() {
            let a = scenic.points[i];
            let b = scenic.points[(i + 1) % scenic.points.len()];
            assert!(a.iter().all(|p| p.is_finite()));
            assert!((b[1] - a[1]).abs() / scenic.lengths[i] < 0.13);
            assert!(scenic.lengths[i] < 3.0);
            let heading = scenic.tangents[i][0].atan2(scenic.tangents[i][1]);
            let j = (i + 1) % scenic.points.len();
            let next = scenic.tangents[j][0].atan2(scenic.tangents[j][1]);
            assert!(crate::track::angle(next - heading).abs() / scenic.lengths[i] < 0.11);
        }
    }

    #[test]
    fn highway_includes_straights_sweeps_and_opposing_hairpins() {
        let road = TrackId::Scenic.track();
        let mut left: f32 = 0.0;
        let mut right: f32 = 0.0;
        let mut straight = 0;
        let mut sweeping = 0;
        for i in 0..road.points.len() {
            let j = (i + 1) % road.points.len();
            let a = road.tangents[i][0].atan2(road.tangents[i][1]);
            let b = road.tangents[j][0].atan2(road.tangents[j][1]);
            let curvature = angle(b - a) / road.lengths[i];
            left = left.max(curvature);
            right = right.min(curvature);
            straight += usize::from(curvature.abs() < 0.002);
            sweeping += usize::from((0.008..0.025).contains(&curvature.abs()));
        }
        println!(
            "highway: length {}, curvature {right}..{left}",
            road.length()
        );
        assert!(
            left > 0.035 && right < -0.035,
            "include hairpins in both directions"
        );
        assert!(
            straight > 100 && sweeping > 100,
            "retain useful straights and medium turns"
        );
    }

    #[test]
    fn spatial_projection_matches_full_search_and_route_caches_are_isolated() {
        let scenic = TrackId::Scenic.track();
        for i in 0..400 {
            let s = scenic.at_distance(scenic.length() * i as f32 / 400.0);
            for offset in [-45.0, -3.0, 0.0, 3.0, 45.0] {
                let p = s.lane_position(offset);
                let indexed = scenic.project(p);
                let exact = scenic.nearest(p, 0..scenic.points.len()).1;
                assert!((indexed.distance - exact.distance).abs() < 0.001);
                assert!((indexed.point[1] - exact.point[1]).abs() < 0.001);
            }
        }
        let compact_length = length();
        assert!(scenic.length() > compact_length * 7.0);
        assert_eq!(length(), compact_length);
    }

    #[test]
    fn scenic_route_has_no_crossings_or_ambiguous_neighbouring_roads() {
        let route = TrackId::Scenic.track();
        let count = (route.length() / 10.0).ceil() as usize;
        for i in 0..count {
            let a = route.at_distance(route.length() * i as f32 / count as f32);
            for j in i + 1..count {
                let b = route.at_distance(route.length() * j as f32 / count as f32);
                let along = (a.distance - b.distance).abs();
                if along.min(route.length() - along) < 35.0 {
                    continue;
                }
                let gap = (a.point[0] - b.point[0]).hypot(a.point[2] - b.point[2]);
                assert!(
                    gap > route.width * 2.0,
                    "ambiguous road near {} / {} m",
                    a.distance,
                    b.distance
                );
            }
        }
    }
}

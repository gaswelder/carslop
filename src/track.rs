//! Immutable authored routes. No global mutable "active track".

use std::sync::OnceLock;

pub const ROAD_WIDTH: f32 = 7.2;
pub const CAR_RADIUS: f32 = 1.4;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrackId {
    Compact,
    Engomi,
    File(usize),
}

#[cfg(test)]
pub fn all() -> [TrackId; 3] {
    [TrackId::Compact, TrackId::scenic(), TrackId::Engomi]
}

struct RuntimeTrack {
    path: std::path::PathBuf,
    name: &'static str,
    width: f32,
    coastal: bool,
    points: Vec<[f32; 3]>,
    track: OnceLock<Result<std::sync::Arc<Track>, String>>,
}
fn runtime_tracks() -> &'static std::sync::RwLock<Vec<std::sync::Arc<RuntimeTrack>>> {
    static TRACKS: OnceLock<std::sync::RwLock<Vec<std::sync::Arc<RuntimeTrack>>>> = OnceLock::new();
    TRACKS.get_or_init(Default::default)
}
/// Register a data-only closed circuit. Spline and render geometry load on selection.
/// Optional `scenery` selects `standard` (default) or `coastal` terrain and roadside scenery.
pub fn register_file(path: &std::path::Path) -> Result<(TrackId, &'static str), String> {
    let path = path.canonicalize().map_err(|e| e.to_string())?;
    let mut tracks = runtime_tracks().write().unwrap();
    if let Some(index) = tracks.iter().position(|t| t.path == path) {
        return Ok((TrackId::File(index), tracks[index].name));
    }
    let data = std::fs::read_to_string(&path).map_err(|e| e.to_string())?;
    let v: serde_json::Value = serde_json::from_str(&data).map_err(|e| e.to_string())?;
    if v["version"].as_u64() != Some(1) {
        return Err("track version must be 1".into());
    }
    let name = v["name"]
        .as_str()
        .filter(|s| !s.trim().is_empty() && s.chars().count() <= 80)
        .ok_or("track needs a name of 1..80 characters")?;
    let width = match v.get("width") {
        None => ROAD_WIDTH,
        Some(value) => value.as_f64().ok_or("track width must be a number")? as f32,
    };
    if !width.is_finite() || !(2.0..=30.0).contains(&width) {
        return Err("track width must be 2..30 metres".into());
    }
    let coastal = match v.get("scenery").and_then(serde_json::Value::as_str) {
        None if v.get("scenery").is_none() => false,
        Some("standard") => false,
        Some("coastal") => true,
        _ => return Err("track scenery must be standard or coastal".into()),
    };
    let points: Vec<[f32; 3]> =
        serde_json::from_value(v["points"].clone()).map_err(|e| e.to_string())?;
    if !(4..=4096).contains(&points.len())
        || points
            .iter()
            .flatten()
            .any(|p| !p.is_finite() || p.abs() > 1_000_000.0)
    {
        return Err("track needs 4..4096 finite XYZ points within one million metres".into());
    }
    let mut perimeter = 0.0;
    for i in 0..points.len() {
        let a = points[i];
        let b = points[(i + 1) % points.len()];
        let distance = (a[0] - b[0]).hypot(a[2] - b[2]);
        if distance < 0.5 {
            return Err("adjacent track points must be at least 0.5 metres apart in XZ".into());
        }
        perimeter += distance;
    }
    if perimeter > 50_000.0 {
        return Err("track control perimeter exceeds 50 km".into());
    }
    let name = Box::leak(name.trim().to_uppercase().into_boxed_str());
    let index = tracks.len();
    tracks.push(std::sync::Arc::new(RuntimeTrack {
        path,
        name,
        width,
        coastal,
        points,
        track: OnceLock::new(),
    }));
    Ok((TrackId::File(index), name))
}
static SELECTED_FILE: OnceLock<TrackId> = OnceLock::new();
pub fn select_file(path: &std::path::Path) -> Result<(), String> {
    let (id, _) = register_file(path)?;
    SELECTED_FILE
        .set(id)
        .map_err(|_| "track already selected".into())
}
pub fn selected_file() -> Option<TrackId> {
    SELECTED_FILE.get().copied()
}

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

pub struct Track {
    pub id: TrackId,
    pub name: String,
    pub width: f32,
    coastal: bool,
    pub bounds: [[f32; 2]; 2], // min/max XZ, from actual spline samples
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
    /// The bundled default uses the same registry and format as other disk circuits.
    pub fn scenic() -> Self {
        register_file(std::path::Path::new("data/locations/scenic.track.json"))
            .expect("cannot load Coastal Highway track definition")
            .0
    }

    pub fn index(self) -> usize {
        match self {
            Self::Compact => 0,
            Self::Engomi => 1,
            Self::File(index) => 2 + index,
        }
    }

    pub fn track(self) -> std::sync::Arc<Track> {
        self.try_track().expect("cannot load required track")
    }

    pub fn try_track(self) -> Result<std::sync::Arc<Track>, String> {
        use std::sync::{Arc, Mutex};
        static COMPACT: OnceLock<Arc<Track>> = OnceLock::new();
        // Retain only the most recent city; sessions own their own references.
        type CityCache = Mutex<Option<(std::path::PathBuf, Arc<Track>)>>;
        static CITY: OnceLock<CityCache> = OnceLock::new();
        // Serialize construction separately: a failed builder cannot poison the cache,
        // and concurrent requests cannot allocate multiple copies of a large city.
        static BUILD: Mutex<()> = Mutex::new(());
        Ok(match self {
            Self::File(index) => {
                let definition = runtime_tracks()
                    .read()
                    .unwrap()
                    .get(index)
                    .cloned()
                    .ok_or("unknown runtime track")?;
                definition
                    .track
                    .get_or_init(|| {
                        let mut track = Track::build_width(
                            self,
                            &definition.points,
                            definition.width,
                            definition.coastal,
                        )?;
                        track.name = definition.name.to_owned();
                        Ok(std::sync::Arc::new(track))
                    })
                    .clone()?
            }

            Self::Compact => COMPACT
                .get_or_init(|| Arc::new(Track::build(self, &ROAD_POINTS)))
                .clone(),
            Self::Engomi => {
                let path = crate::city_assets::CITY_ASSET
                    .read()
                    .unwrap()
                    .directory()
                    .to_path_buf();
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
        Self::build_width(
            self.id,
            &ROAD_POINTS.map(|[x, _, z]| [x, 0.0, z]),
            self.width,
            self.coastal,
        )
        .unwrap()
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
            name: crate::city_assets::CITY_ASSET
                .read()
                .unwrap()
                .name()
                .to_owned(),
            width: 8.0,
            coastal: false,
            bounds: city.bounds,
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
        Self::build_width(id, controls, ROAD_WIDTH, false).expect("invalid built-in route")
    }
    fn build_width(
        id: TrackId,
        controls: &[[f32; 3]],
        width: f32,
        coastal: bool,
    ) -> Result<Self, String> {
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
            if !length.is_finite() || length <= 0.0001 {
                return Err("degenerate route spline segment".into());
            }
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
        let mut track = Self {
            id,
            name: "COASTAL LOOP".into(),
            width,
            coastal,
            bounds,
            points,
            distances,
            lengths,
            tangents,
            total,
            shoreline: Vec::new(),
            buckets,
            city: None,
        };
        let count = if coastal {
            (total / 8.0).ceil() as usize
        } else {
            64
        };
        track.shoreline = (0..count)
            .map(|i| {
                let sample = track.at_distance(total * i as f32 / count as f32);
                let n = sample.normal();
                let offset = track.width * 0.5 + if coastal { 22.0 } else { 10.0 };
                [
                    sample.point[0] - n[0] * offset,
                    crate::ambience::SEA_LEVEL,
                    sample.point[2] - n[1] * offset,
                ]
            })
            .collect();
        Ok(track)
    }

    pub fn is_coastal(&self) -> bool {
        self.coastal
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
        if self.id != TrackId::Compact {
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
pub fn angle(value: f32) -> f32 {
    (value + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

#[cfg(test)]
mod tests {
    #[test]
    fn runtime_circuit_loads_without_a_compiled_identity() {
        let root = tempfile::tempdir().unwrap();
        let path = root.path().join("hills.track.json");
        std::fs::write(&path, r#"{"version":1,"name":"Hills","width":11,"points":[[0,0,0],[100,2,0],[100,4,100],[0,1,100]]}"#).unwrap();
        let (id, name) = super::register_file(&path).unwrap();
        assert_eq!(name, "HILLS");
        assert!(matches!(id, super::TrackId::File(_)));
        assert_eq!(id, super::register_file(&path).unwrap().0);
        let track = id.try_track().unwrap();
        assert_eq!(track.width, 11.0);
        assert!(track.city().is_none());
        assert!(!track.is_coastal());
        assert!(std::sync::Arc::ptr_eq(&track, &id.try_track().unwrap()));
        assert!(track.length() > 400.0);
        for sample in track.centerline() {
            assert!(sample.iter().all(|v| v.is_finite()));
            let road = track.project(*sample);
            assert_eq!(road.width, 11.0);
            assert!(road.point.iter().all(|v| v.is_finite()));
        }
    }

    #[test]
    fn coastal_scenery_follows_file_data_after_renaming() {
        let root = tempfile::tempdir().unwrap();
        let path = root.path().join("renamed.track.json");
        let mut data: serde_json::Value = serde_json::from_str(
            &std::fs::read_to_string("data/locations/scenic.track.json").unwrap(),
        )
        .unwrap();
        data["name"] = "Another coast".into();
        std::fs::write(&path, serde_json::to_vec(&data).unwrap()).unwrap();
        let (id, _) = super::register_file(&path).unwrap();
        assert!(matches!(id, super::TrackId::File(_)));
        let copied = id.track();
        let original = super::TrackId::scenic().track();
        assert_ne!(copied.id, original.id);
        assert_eq!(copied.name, "ANOTHER COAST");
        assert!(copied.is_coastal());
        assert_eq!(copied.centerline(), original.centerline());
        assert_eq!(copied.shoreline(), original.shoreline());
        assert_eq!(copied.width, original.width);
    }

    #[test]
    fn runtime_circuit_rejects_invalid_definitions() {
        for data in [
            r#"{"version":2}"#,
            r#"{"version":1,"name":"bad","scenery":"unknown"}"#,
            r#"{"version":1,"name":"bad","scenery":null}"#,
            r#"{"version":1,"name":"bad","width":"wide","points":[]}"#,
            r#"{"version":1,"name":"bad","width":0,"points":[]}"#,
            r#"{"version":1,"name":"bad","points":[[0,0,0],[1,0,0],[1,0,1]]}"#,
            r#"{"version":1,"name":"bad","points":[[0,0,0],[0,1,0],[1,0,1],[2,0,2]]}"#,
            r#"{"version":1,"name":"bad","points":[[0,0,0],[20000,0,0],[20000,0,20000],[0,0,20000]]}"#,
        ] {
            let file = tempfile::NamedTempFile::new().unwrap();
            std::fs::write(file.path(), data).unwrap();
            assert!(super::register_file(file.path()).is_err());
        }
    }

    use super::*;

    #[test]
    fn route_is_closed_and_projection_follows_elevation() {
        assert!(TrackId::Compact.track().length() > 180.0);
        for distance in [0.0, 20.0, 50.0, 100.0, 160.0] {
            let sample = TrackId::Compact.track().at_distance(distance);
            let projected = TrackId::Compact.track().project(sample.lane_position(1.0));
            assert!((projected.point[1] - sample.point[1]).abs() < 0.15);
            assert!((projected.offset - 1.0).abs() < 0.15);
        }
        assert_eq!(
            TrackId::Compact.track().at_distance(0.0).point,
            TrackId::Compact
                .track()
                .at_distance(TrackId::Compact.track().length())
                .point
        );
    }

    #[test]
    fn scenic_route_has_real_scale_moderate_grades_and_smooth_hairpins() {
        let scenic = TrackId::scenic().track();
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
        let road = TrackId::scenic().track();
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
        let scenic = TrackId::scenic().track();
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
        let compact_length = TrackId::Compact.track().length();
        assert!(scenic.length() > compact_length * 7.0);
        assert_eq!(TrackId::Compact.track().length(), compact_length);
    }

    #[test]
    fn scenic_route_has_no_crossings_or_ambiguous_neighbouring_roads() {
        let route = TrackId::scenic().track();
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

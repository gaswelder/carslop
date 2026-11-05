//! Offline OSM street network. Geometry, collision, recovery and minimap share it.
use crate::track::{RoadSample, CAR_RADIUS};
use std::collections::HashMap;

const CELL: f32 = 32.0;
#[derive(Debug)]
pub struct Street {
    pub ends: [usize; 2],
    way: Option<i64>,
    pub width: f32,
    pub oneway: i8,
    pub length: f32,
    pub distance: f32,
    pub path: Vec<[f32; 3]>,
    path_distances: Vec<f32>,
}
#[derive(Debug)]
pub struct Building {
    pub id: u64,
    pub height: f32,
    pub base_y: f32,
    pub polygon: Vec<[f32; 2]>,
}
#[derive(Debug)]
pub struct City {
    origin: Option<[f64; 2]>,
    pub nodes: Vec<[f32; 3]>,
    pub streets: Vec<Street>,
    pub buildings: Vec<Building>,
    pub furniture: Vec<crate::traffic::Furniture>,
    pub bounds: [[f32; 2]; 2],
    pub length: f32,
    pub landscape: crate::landscape::Landscape,
    pub elevation: Option<crate::elevation::HeightField>,
    pub surfaces: Option<crate::street_surface::StreetSurface>,
    street_names: HashMap<i64, String>,
    building_names: HashMap<u64, String>,
    road_grid: HashMap<(i32, i32), Vec<usize>>,
    building_grid: HashMap<(i32, i32), Vec<usize>>,
}
fn cell(p: [f32; 2]) -> (i32, i32) {
    ((p[0] / CELL).floor() as i32, (p[1] / CELL).floor() as i32)
}
fn insert(grid: &mut HashMap<(i32, i32), Vec<usize>>, bounds: [[f32; 2]; 2], id: usize) {
    let a = cell(bounds[0]);
    let b = cell(bounds[1]);
    for x in a.0..=b.0 {
        for z in a.1..=b.1 {
            grid.entry((x, z)).or_default().push(id);
        }
    }
}
fn bounds(points: impl Iterator<Item = [f32; 2]>, pad: f32) -> [[f32; 2]; 2] {
    let mut result = [[f32::INFINITY; 2], [f32::NEG_INFINITY; 2]];
    for p in points {
        for i in 0..2 {
            result[0][i] = result[0][i].min(p[i] - pad);
            result[1][i] = result[1][i].max(p[i] + pad);
        }
    }
    result
}
fn nearest_edge(p: [f32; 2], a: [f32; 2], b: [f32; 2]) -> [f32; 2] {
    let d = [b[0] - a[0], b[1] - a[1]];
    let t = (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1])
        / (d[0] * d[0] + d[1] * d[1]).max(0.0001))
    .clamp(0.0, 1.0);
    [a[0] + d[0] * t, a[1] + d[1] * t]
}
impl Building {
    pub fn contains(&self, p: [f32; 2]) -> bool {
        let mut inside = false;
        for i in 0..self.polygon.len() {
            let a = self.polygon[i];
            let b = self.polygon[(i + 1) % self.polygon.len()];
            if (a[1] > p[1]) != (b[1] > p[1])
                && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]
            {
                inside = !inside;
            }
        }
        inside
    }
    fn nearest(&self, p: [f32; 2]) -> ([f32; 2], [f32; 2], f32) {
        let mut best = ([0.0; 2], [0.0; 2], f32::INFINITY);
        for i in 0..self.polygon.len() {
            let a = self.polygon[i];
            let b = self.polygon[(i + 1) % self.polygon.len()];
            let q = nearest_edge(p, a, b);
            let distance = (p[0] - q[0]).hypot(p[1] - q[1]);
            if distance < best.2 {
                let len = (b[0] - a[0]).hypot(b[1] - a[1]).max(0.0001);
                best = (q, [(b[1] - a[1]) / len, -(b[0] - a[0]) / len], distance);
            }
        }
        best
    }
}

/// Read just way labels, without building the full OSM geometry again.
struct OsmLabels {
    streets: HashMap<i64, String>,
    buildings: HashMap<u64, String>,
}
#[cfg(test)]
fn read_street_names(source: impl std::io::BufRead) -> Result<HashMap<i64, String>, String> {
    read_osm_labels(source).map(|labels| labels.streets)
}
fn read_osm_labels(source: impl std::io::BufRead) -> Result<OsmLabels, String> {
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

impl City {
    pub fn engomi() -> Result<Self, String> {
        let source = crate::city_assets::try_read("city.txt")?;
        let mut city = Self::parse(&source)?;
        match crate::package::Reader::open(&crate::city_assets::directory().join("source.osm")) {
            Ok(source) => match read_osm_labels(std::io::BufReader::new(source)) {
                Ok(labels) => {
                    city.street_names = labels.streets;
                    city.building_names = labels.buildings;
                }
                Err(error) => log::warn!("Cannot load street names: {error}"),
            },
            Err(error) => log::warn!("Cannot open street names: {error}"),
        }
        let (mut surfaces, paths, checksums) = crate::street_surface::StreetSurface::load_city(
            crate::city_assets::directory(),
            &source,
            city.streets.len(),
        )?;
        let manifest: serde_json::Value =
            serde_json::from_str(&crate::city_assets::try_read("manifest.json")?)
                .map_err(|e| format!("invalid location manifest: {e}"))?;
        if let Some(height) = manifest.get("pavement_height_metres") {
            let height = height
                .as_f64()
                .filter(|h| (0.08..=0.25).contains(h))
                .ok_or("pavement_height_metres must be between 0.08 and 0.25")?;
            surfaces.pavement_height = height as f32;
        }
        city.length = 0.0;
        city.road_grid.clear();
        for (i, (street, path)) in city.streets.iter_mut().zip(paths).enumerate() {
            street.path = path;
            street.path_distances = vec![0.0];
            street.length = 0.0;
            for pair in street.path.windows(2) {
                let length = (pair[1][0] - pair[0][0]).hypot(pair[1][2] - pair[0][2]);
                if !length.is_finite() || length <= 0.0001 {
                    return Err("degenerate compiled road path".into());
                }
                street.length += length;
                street.path_distances.push(street.length);
            }
            street.distance = city.length;
            city.length += street.length;
            insert(
                &mut city.road_grid,
                bounds(street.path.iter().map(|p| [p[0], p[2]]), 32.0),
                i,
            );
        }
        city.furniture = crate::traffic::load_with_checksums(
            &crate::city_assets::try_read("furniture.txt")?,
            checksums,
        )?;
        let field = crate::elevation::HeightField::load(
            &crate::city_assets::try_read("elevation/heightfield.txt")?,
            &source,
        )?;
        for p in &mut city.nodes {
            p[1] = field.height([p[0], p[2]]);
        }
        for street in &mut city.streets {
            for p in &mut street.path {
                p[1] = field.height([p[0], p[2]]);
            }
        }
        for feature in &mut city.furniture {
            feature.position[1] = field.height([feature.position[0], feature.position[2]]);
        }
        for building in &mut city.buildings {
            building.base_y = (0..building.polygon.len())
                .flat_map(|i| {
                    field.segment(
                        building.polygon[i],
                        building.polygon[(i + 1) % building.polygon.len()],
                    )
                })
                .map(|p| p[1])
                .fold(f32::NEG_INFINITY, f32::max)
                + 0.02;
        }
        city.landscape = crate::landscape::Landscape::load_with_checksums(
            &crate::city_assets::try_read("landscape.txt")?,
            checksums,
        )?;
        let imagery = crate::city_assets::directory().join("imagery-trees.txt");
        match crate::package::read_to_string(&imagery) {
            Ok(text) => city
                .landscape
                .append_imagery_with_checksums(&text, checksums)?,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => return Err(format!("cannot read imagery trees: {e}")),
        }
        city.elevation = Some(field);
        city.surfaces = Some(surfaces);
        Ok(city)
    }
    pub(crate) fn parse(text: &str) -> Result<Self, String> {
        let mut city = Self {
            origin: None,
            nodes: Vec::new(),
            streets: Vec::new(),
            buildings: Vec::new(),
            furniture: Vec::new(),
            bounds: [[0.0; 2]; 2],
            length: 0.0,
            surfaces: None,
            elevation: None,
            landscape: crate::landscape::Landscape::default(),
            street_names: HashMap::new(),
            building_names: HashMap::new(),
            road_grid: HashMap::new(),
            building_grid: HashMap::new(),
        };
        let mut lines = text.lines();
        if !matches!(lines.next(), Some("CITY 1" | "CITY 2")) {
            return Err("unsupported city format".into());
        }
        for line in lines {
            let fields: Vec<_> = line.split_whitespace().collect();
            let f = |i: usize| -> Result<f32, String> {
                let v: f32 = fields
                    .get(i)
                    .ok_or("missing coordinate")?
                    .parse()
                    .map_err(|_| "invalid coordinate")?;
                if v.is_finite() {
                    Ok(v)
                } else {
                    Err("nonfinite coordinate".into())
                }
            };
            let n = |i: usize| -> Result<usize, String> {
                fields
                    .get(i)
                    .ok_or("missing index")?
                    .parse()
                    .map_err(|_| "invalid index".into())
            };
            match fields.first().copied() {
                Some("origin") => {
                    let latitude: f64 = fields
                        .get(1)
                        .ok_or("missing latitude")?
                        .parse()
                        .map_err(|_| "invalid latitude")?;
                    let longitude: f64 = fields
                        .get(2)
                        .ok_or("missing longitude")?
                        .parse()
                        .map_err(|_| "invalid longitude")?;
                    if !(-90.0..=90.0).contains(&latitude) || !(-180.0..=180.0).contains(&longitude)
                    {
                        return Err("invalid geographic origin".into());
                    }
                    city.origin = Some([latitude, longitude]);
                }
                Some("bounds") => city.bounds = [[f(1)?, f(2)?], [f(3)?, f(4)?]],
                Some("n") => city.nodes.push([f(1)?, 0.0, f(2)?]),
                Some("r") => {
                    let ends = [n(1)?, n(2)?];
                    if ends.iter().any(|i| *i >= city.nodes.len()) {
                        return Err("road index out of range".into());
                    }
                    let a = city.nodes[ends[0]];
                    let b = city.nodes[ends[1]];
                    let length = (b[0] - a[0]).hypot(b[2] - a[2]);
                    let width = f(3)?;
                    if length < 0.1 || !(5.0..=30.0).contains(&width) {
                        return Err("invalid road dimensions".into());
                    }
                    let oneway = fields
                        .get(4)
                        .ok_or("missing direction")?
                        .parse::<i8>()
                        .map_err(|_| "invalid direction")?;
                    if !(-1..=1).contains(&oneway) {
                        return Err("invalid direction".into());
                    }
                    city.streets.push(Street {
                        ends,
                        way: fields.get(5).and_then(|id| id.parse().ok()),
                        width,
                        oneway,
                        length,
                        distance: city.length,
                        path: vec![a, b],
                        path_distances: vec![0.0, length],
                    });
                    city.length += length;
                }
                Some("b") => {
                    let id = fields
                        .get(1)
                        .ok_or("missing building id")?
                        .parse()
                        .map_err(|_| "invalid building id")?;
                    let height = f(2)?;
                    let count = n(4)?;
                    if count < 3
                        || fields.len() != 5 + count * 2
                        || !(2.0..=100.0).contains(&height)
                    {
                        return Err("invalid building dimensions".into());
                    }
                    let polygon = (0..count)
                        .map(|i| Ok([f(5 + i * 2)?, f(6 + i * 2)?]))
                        .collect::<Result<Vec<_>, String>>()?;
                    city.buildings.push(Building {
                        id,
                        height,
                        base_y: 0.0,
                        polygon,
                    });
                }
                _ => return Err("unknown city record".into()),
            }
        }
        if city.streets.is_empty()
            || city.bounds[0]
                .iter()
                .zip(city.bounds[1])
                .any(|(a, b)| *a >= b)
        {
            return Err("empty city or invalid bounds".into());
        }
        for (i, r) in city.streets.iter().enumerate() {
            insert(
                &mut city.road_grid,
                bounds(
                    r.ends.iter().map(|n| {
                        let p = city.nodes[*n];
                        [p[0], p[2]]
                    }),
                    32.0,
                ),
                i,
            );
        }
        for (i, b) in city.buildings.iter().enumerate() {
            insert(
                &mut city.building_grid,
                bounds(b.polygon.iter().copied(), CAR_RADIUS + 1.0),
                i,
            );
        }
        Ok(city)
    }
    pub fn height(&self, p: [f32; 2]) -> f32 {
        self.elevation.as_ref().map_or(0.0, |f| f.height(p))
    }
    /// Vehicle origin uses the asphalt's 5 cm reference. Sidewalks and other
    /// surfaces retain their rendered heights rather than burying the wheels.
    pub fn drive_height(&self, p: [f32; 2]) -> f32 {
        if self.surfaces.is_none() {
            return self.height(p);
        }
        let lift = self
            .surfaces
            .as_ref()
            .and_then(|s| s.paving_height(p))
            .unwrap_or_else(|| {
                if self.landscape.style(p).is_some() {
                    0.005
                } else {
                    -0.02
                }
            });
        self.height(p) + lift - 0.05
    }
    pub fn paved(&self, p: [f32; 2]) -> bool {
        if let Some(s) = &self.surfaces {
            if let Some(h) = s.paving_height(p) {
                return h != 0.08;
            }
        }
        matches!(self.landscape.style(p), Some(3..=5))
    }
    pub fn ground_segment(&self, a: [f32; 2], b: [f32; 2]) -> Vec<[f32; 3]> {
        self.elevation.as_ref().map_or_else(
            || vec![[a[0], 0.0, a[1]], [b[0], 0.0, b[1]]],
            |f| f.segment(a, b),
        )
    }
    pub fn sample(&self, index: usize, t: f32) -> RoadSample {
        let r = &self.streets[index];
        let distance = t.clamp(0.0, 1.0) * r.length;
        let index = r
            .path_distances
            .partition_point(|d| *d < distance)
            .saturating_sub(1)
            .min(r.path.len() - 2);
        let a = r.path[index];
        let b = r.path[index + 1];
        let length = r.path_distances[index + 1] - r.path_distances[index];
        let t = ((distance - r.path_distances[index]) / length).clamp(0.0, 1.0);
        let mut point = std::array::from_fn(|i| a[i] + (b[i] - a[i]) * t);
        point[1] = self.height([point[0], point[2]]);
        RoadSample {
            point,
            tangent: [(b[0] - a[0]) / length, (b[2] - a[2]) / length],
            distance: r.distance + distance,
            offset: 0.0,
            width: r.width,
        }
    }
    pub fn at_distance(&self, distance: f32) -> RoadSample {
        let d = distance.rem_euclid(self.length);
        let i = self
            .streets
            .partition_point(|r| r.distance <= d)
            .saturating_sub(1);
        self.sample(
            i,
            ((d - self.streets[i].distance) / self.streets[i].length).clamp(0.0, 1.0),
        )
    }
    fn street_ids_in(&self, bounds: [[f32; 2]; 2]) -> Vec<usize> {
        let a = cell(bounds[0]);
        let b = cell(bounds[1]);
        let mut ids = Vec::new();
        for x in a.0..=b.0 {
            for z in a.1..=b.1 {
                if let Some(values) = self.road_grid.get(&(x, z)) {
                    ids.extend(values);
                }
            }
        }
        ids.sort_unstable();
        ids.dedup();
        ids
    }
    pub fn map_segments_in(&self, bounds: [[f32; 2]; 2]) -> Vec<([f32; 3], [f32; 3])> {
        self.street_ids_in(bounds)
            .into_iter()
            .flat_map(|i| self.streets[i].path.windows(2).map(|p| (p[0], p[1])))
            .collect()
    }
    fn candidates(&self, p: [f32; 3]) -> Box<dyn Iterator<Item = usize> + '_> {
        if let Some(ids) = self.road_grid.get(&cell([p[0], p[2]])) {
            return Box::new(ids.iter().copied());
        }
        for radius in [
            1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 128.0, 256.0, 512.0, 1024.0,
        ] {
            let d = radius * CELL;
            let ids = self.street_ids_in([[p[0] - d, p[2] - d], [p[0] + d, p[2] + d]]);
            if !ids.is_empty() {
                return Box::new(ids.into_iter());
            }
        }
        Box::new(0..self.streets.len())
    }
    fn projected(&self, index: usize, p: [f32; 3]) -> (f32, RoadSample) {
        let r = &self.streets[index];
        let point = [p[0], p[2]];
        r.path
            .windows(2)
            .enumerate()
            .map(|(i, pair)| {
                let a = [pair[0][0], pair[0][2]];
                let b = [pair[1][0], pair[1][2]];
                let q = nearest_edge(point, a, b);
                let along = (q[0] - a[0]).hypot(q[1] - a[1]);
                let mut s = self.sample(index, (r.path_distances[i] + along) / r.length);
                let delta = [p[0] - s.point[0], p[2] - s.point[2]];
                s.offset = delta[0] * s.normal()[0] + delta[1] * s.normal()[1];
                (delta[0].hypot(delta[1]), s)
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .unwrap()
    }
    pub fn project(&self, p: [f32; 3]) -> RoadSample {
        self.project_street(p).1
    }
    fn project_street(&self, p: [f32; 3]) -> (usize, RoadSample) {
        let origin = cell([p[0], p[2]]);
        let mut seen = std::collections::HashSet::new();
        let mut best: Option<(f32, usize, RoadSample)> = None;
        let limit = (self
            .bounds
            .iter()
            .flat_map(|b| [(b[0] - p[0]).abs(), (b[1] - p[2]).abs()])
            .fold(0.0_f32, f32::max)
            / CELL)
            .ceil() as i32
            + 2;
        for radius in 0..=limit {
            for dx in -radius..=radius {
                for dz in -radius..=radius {
                    if dx.abs() != radius && dz.abs() != radius {
                        continue;
                    }
                    if let Some(ids) = self.road_grid.get(&(origin.0 + dx, origin.1 + dz)) {
                        for &i in ids {
                            if seen.insert(i) {
                                let candidate = self.projected(i, p);
                                if best.is_none_or(|b| candidate.0 < b.0) {
                                    best = Some((candidate.0, i, candidate.1));
                                }
                            }
                        }
                    }
                }
            }
            // Indexed path bounds carry a 32 m margin, so unvisited cells
            // cannot hide a closer road once this distance is reached.
            if best.is_some_and(|b| b.0 <= (radius + 1) as f32 * CELL) {
                let (_, index, road) = best.unwrap();
                return (index, road);
            }
        }
        let (_, index, road) = best.expect("city contains roads");
        (index, road)
    }
    pub fn geographic_origin(&self) -> Option<[f64; 2]> {
        self.origin
    }

    pub fn google_maps_url(&self, position: [f32; 3]) -> Option<String> {
        let [latitude, longitude] = self.origin?;
        let longitude_scale = latitude.to_radians().cos();
        if longitude_scale.abs() < 1e-9 || !position[0].is_finite() || !position[2].is_finite() {
            return None;
        }
        // Invert the OSM compiler's local metres projection (east +X, south +Z).
        let latitude_at_car = latitude - (position[2] as f64 / 6371000.0).to_degrees();
        let longitude_at_car =
            longitude + (position[0] as f64 / (6371000.0 * longitude_scale)).to_degrees();
        Some(format!("https://www.google.com/maps/search/?api=1&query={latitude_at_car:.7},{longitude_at_car:.7}"))
    }

    /// Label the closest road only while the car is on or beside its surface.
    pub fn street_name(&self, p: [f32; 3]) -> &str {
        let (index, road) = self.project_street(p);
        if (p[0] - road.point[0]).hypot(p[2] - road.point[2]) > road.width * 0.5 + 5.0 {
            return "";
        }
        self.streets[index]
            .way
            .and_then(|id| self.street_names.get(&id))
            .map_or("UNNAMED ROAD", String::as_str)
    }

    /// Signed distance among nearby footprints; infinity when none is nearby.
    pub fn building_clearance(&self, p: [f32; 2]) -> f32 {
        self.building_grid
            .get(&cell(p))
            .map_or(f32::INFINITY, |ids| {
                ids.iter()
                    .map(|i| {
                        let b = &self.buildings[*i];
                        b.nearest(p).2 * if b.contains(p) { -1.0 } else { 1.0 }
                    })
                    .fold(f32::INFINITY, f32::min)
            })
    }
    fn clear_for_car(&self, p: [f32; 3]) -> bool {
        if self.surfaces.is_some() && self.road_clearance(p) > -CAR_RADIUS - 0.15 {
            return false;
        }
        self.building_clearance([p[0], p[2]]) > CAR_RADIUS + 0.15
    }
    pub fn recovery(&self, p: [f32; 3]) -> RoadSample {
        // Service roads may lead into garages. Never recover inside a footprint.
        self.candidates(p)
            .flat_map(|i| {
                let r = &self.streets[i];
                let s = self.projected(i, p).1;
                [-12.0, -6.0, -3.0, 0.0, 3.0, 6.0, 12.0]
                    .into_iter()
                    .map(move |offset| {
                        self.sample(
                            i,
                            ((s.distance - r.distance + offset) / r.length).clamp(0.0, 1.0),
                        )
                    })
            })
            .filter(|s| self.clear_for_car(s.point))
            .min_by(|a, b| {
                let distance = |s: &RoadSample| (s.point[0] - p[0]).hypot(s.point[2] - p[2]);
                distance(a).total_cmp(&distance(b))
            })
            .unwrap_or_else(|| self.spawn())
    }
    /// Distance to the union of street capsules, including shared junction nodes.
    pub fn road_clearance(&self, p: [f32; 3]) -> f32 {
        if let Some(surfaces) = &self.surfaces {
            return surfaces.clearance([p[0], p[2]]);
        }
        self.candidates(p)
            .map(|i| {
                let (d, s) = self.projected(i, p);
                d - s.width * 0.5
            })
            .fold(f32::INFINITY, f32::min)
    }
    pub fn spawn(&self) -> RoadSample {
        for (i, r) in self.streets.iter().enumerate() {
            for t in [0.5, 0.25, 0.75] {
                let mut s = self.sample(i, t);
                if self.clear_for_car(s.point) {
                    if r.oneway == -1 {
                        s.tangent = s.tangent.map(|v| -v);
                    }
                    return s;
                }
            }
        }
        panic!("city contains no safe spawn");
    }
    pub fn constrain(&self, position: &mut [f32; 3], velocity: &mut [f32; 2]) -> f32 {
        let mut impact: f32 = 0.0;
        if self.surfaces.is_none() {
            let best = self
                .candidates(*position)
                .map(|i| {
                    let (d, s) = self.projected(i, *position);
                    (d - (s.width * 0.5 - CAR_RADIUS), d, s)
                })
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .unwrap();

            if best.0 > 0.0 && best.1 > 0.0001 {
                let n = [
                    (position[0] - best.2.point[0]) / best.1,
                    (position[2] - best.2.point[2]) / best.1,
                ];
                position[0] -= n[0] * best.0;
                position[2] -= n[1] * best.0;
                impact = deflect(velocity, n);
            }
        }
        // Check footprints, not bounding boxes: concave courtyards/streets remain open.
        if let Some(ids) = self.building_grid.get(&cell([position[0], position[2]])) {
            for &i in ids {
                let b = &self.buildings[i];
                let p = [position[0], position[2]];
                let (q, outward, distance) = b.nearest(p);
                let inside = b.contains(p);
                if inside || distance < CAR_RADIUS {
                    let n = if inside || distance < 0.0001 {
                        outward
                    } else {
                        [(p[0] - q[0]) / distance, (p[1] - q[1]) / distance]
                    };
                    position[0] = q[0] + n[0] * (CAR_RADIUS + 0.01);
                    position[2] = q[1] + n[1] * (CAR_RADIUS + 0.01);
                    impact = impact.max(deflect(velocity, [-n[0], -n[1]]));
                }
            }
        }
        // The saved extract is a finite driving area. Low curbs are crossable;
        // the crop boundary remains solid so the car cannot leave terrain data.
        for (axis, world_axis) in [(0, 0), (1, 2)] {
            let value = position[world_axis];
            position[world_axis] = value.clamp(
                self.bounds[0][axis] + CAR_RADIUS,
                self.bounds[1][axis] - CAR_RADIUS,
            );
            if position[world_axis] != value {
                let mut normal = [0.0; 2];
                normal[axis] = (value - position[world_axis]).signum();
                impact = impact.max(deflect(velocity, normal));
            }
        }
        position[1] = self.drive_height([position[0], position[2]]) + 0.48;
        impact
    }
    pub fn building_name(&self, id: u64) -> Option<&str> {
        self.building_names.get(&id).map(String::as_str)
    }
    pub fn nearby_building_labels(&self, eye: [f32; 3]) -> Vec<(u64, &str, [f32; 3], f32)> {
        let mut ids = std::collections::HashSet::new();
        let a = cell([eye[0] - 180.0, eye[2] - 180.0]);
        let b = cell([eye[0] + 180.0, eye[2] + 180.0]);
        for x in a.0..=b.0 {
            for z in a.1..=b.1 {
                if let Some(items) = self.building_grid.get(&(x, z)) {
                    ids.extend(items.iter().copied());
                }
            }
        }
        let mut labels: Vec<_> = ids
            .into_iter()
            .filter_map(|i| {
                let building = &self.buildings[i];
                let name = self.building_name(building.id)?;
                let n = building.polygon.len() as f32;
                let x = building.polygon.iter().map(|p| p[0]).sum::<f32>() / n;
                let z = building.polygon.iter().map(|p| p[1]).sum::<f32>() / n;
                let distance = (x - eye[0]).hypot(z - eye[2]);
                (distance < 180.0).then_some((
                    building.id,
                    name,
                    [x, building.base_y + building.height + 1.5, z],
                    distance,
                ))
            })
            .collect();
        labels.sort_by(|a, b| a.3.total_cmp(&b.3).then(a.0.cmp(&b.0)));
        labels.truncate(32);
        labels
    }
    pub fn building_label_visible(&self, eye: [f32; 3], anchor: [f32; 3], target: u64) -> bool {
        let steps = ((eye[0] - anchor[0]).hypot(eye[2] - anchor[2]) / 1.0)
            .ceil()
            .max(1.0) as usize;
        for step in 1..steps {
            let t = step as f32 / steps as f32;
            let p: [f32; 3] = std::array::from_fn(|i| eye[i] + (anchor[i] - eye[i]) * t);
            if self
                .building_grid
                .get(&cell([p[0], p[2]]))
                .is_some_and(|ids| {
                    ids.iter().any(|i| {
                        let b = &self.buildings[*i];
                        b.id != target && p[1] < b.base_y + b.height && b.contains([p[0], p[2]])
                    })
                })
                || p[1] < self.height([p[0], p[2]])
            {
                return false;
            }
        }
        true
    }

    pub fn camera_eye(&self, anchor: [f32; 3], eye: [f32; 3]) -> [f32; 3] {
        let length = (eye[0] - anchor[0]).hypot(eye[2] - anchor[2]);
        let steps = (length / 0.35).ceil().max(1.0) as usize;
        let mut safe = anchor;
        for step in 1..=steps {
            let t = step as f32 / steps as f32;
            let p = std::array::from_fn(|i| anchor[i] + (eye[i] - anchor[i]) * t);
            let blocked = self
                .building_grid
                .get(&cell([p[0], p[2]]))
                .is_some_and(|ids| {
                    ids.iter().any(|i| {
                        let b = &self.buildings[*i];
                        p[1] < b.base_y + b.height + 0.5
                            && (b.contains([p[0], p[2]]) || b.nearest([p[0], p[2]]).2 < 0.5)
                    })
                });
            if blocked || p[1] < self.height([p[0], p[2]]) + 0.4 {
                return safe;
            }
            safe = p;
        }
        eye
    }
}
pub(crate) fn deflect(velocity: &mut [f32; 2], normal: [f32; 2]) -> f32 {
    let outward = velocity[0] * normal[0] + velocity[1] * normal[1];
    if outward > 0.0 {
        // Friction is limited by the normal impact impulse. A fixed percentage
        // of total speed on every contact makes light, repeated scrapes stop cars.
        let tangent = [
            velocity[0] - normal[0] * outward,
            velocity[1] - normal[1] * outward,
        ];
        let speed = tangent[0].hypot(tangent[1]);
        let retained = if speed > 0.0 {
            (1.0 - 0.06 * outward / speed).max(0.0)
        } else {
            0.0
        };
        for i in 0..2 {
            velocity[i] = tangent[i] * retained - normal[i] * outward * 0.08;
        }
        outward
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    #[ignore = "loads the full selected location; run alone for startup profiling"]
    fn location_startup_profile() {
        let started = std::time::Instant::now();
        let city = City::engomi().unwrap();
        let surfaces = city.surfaces.as_ref().unwrap();
        eprintln!(
            "City load: {:.3}s; {} roads, {} buildings, {} triangles, {} curbs, {} trees",
            started.elapsed().as_secs_f64(),
            city.streets.len(),
            city.buildings.len(),
            surfaces.triangles.len(),
            surfaces.curbs.len(),
            city.landscape.trees.len()
        );
    }

    #[test]
    fn repeated_light_barrier_contacts_preserve_sliding_speed() {
        // Exercise a rotated wall so the response cannot depend on world axes.
        let normal = [0.6, 0.8];
        let tangent = [-0.8, 0.6];
        let mut speed = 20.0;
        for _ in 0..120 {
            let mut velocity = std::array::from_fn(|i| tangent[i] * speed + normal[i] * 0.01);
            let impact = deflect(&mut velocity, normal);
            assert!((impact - 0.01).abs() < 0.00001);
            assert!(velocity[0] * normal[0] + velocity[1] * normal[1] <= 0.0);
            speed = velocity[0] * tangent[0] + velocity[1] * tangent[1];
        }
        assert!(speed > 19.9, "light scrapes drained sliding speed: {speed}");
    }

    #[test]
    fn barrier_response_rebounds_head_on_and_leaves_departing_cars_alone() {
        let mut velocity = [10.0, 0.0];
        assert_eq!(deflect(&mut velocity, [1.0, 0.0]), 10.0);
        assert!((velocity[0] + 0.8).abs() < 0.00001);
        assert_eq!(velocity[1], 0.0);
        let departing = velocity;
        assert_eq!(deflect(&mut velocity, [1.0, 0.0]), 0.0);
        assert_eq!(velocity, departing);
    }

    const CROSS:&str="CITY 1\norigin 0 0\nbounds -50 -50 50 50\nn -40 0\nn 0 0\nn 40 0\nn 0 -40\nn 0 40\nr 0 1 8 0 1\nr 1 2 8 0 1\nr 3 1 8 0 2\nr 1 4 8 0 2\nb 1 10 1 4 8 8 18 8 18 18 8 18\n";
    #[test]
    fn building_labels_use_names_not_plot_references_and_respect_occlusion() {
        let source = br#"<osm><way id="1"><tag k="building" v="yes"/><tag k="name" v="Local"/><tag k="name:en" v="City Hall"/></way><way id="2"><tag k="building" v="yes"/><tag k="ref" v="PLOT 2"/></way></osm>"#;
        let labels = read_osm_labels(&source[..]).unwrap();
        assert_eq!(labels.buildings[&1], "City Hall");
        assert!(!labels.buildings.contains_key(&2));
        let mut city = City::parse(CROSS).unwrap();
        city.building_names = labels.buildings;
        let eye = [0.0, 2.0, -20.0];
        let labels = city.nearby_building_labels(eye);
        assert_eq!(labels.len(), 1);
        assert_eq!(labels[0].1, "City Hall");
        assert!(city.building_label_visible(eye, labels[0].2, 1));
        assert!(!city.building_label_visible([13.0, 2.0, 0.0], [13.0, 2.0, 30.0], 999));
    }

    #[test]
    fn google_maps_url_inverts_the_osm_projection() {
        let city =
            City::parse(&CROSS.replace("origin 0 0", "origin 35.1654321 33.3256789")).unwrap();
        assert_eq!(
            city.google_maps_url([0.0, 100.0, 0.0]).unwrap(),
            "https://www.google.com/maps/search/?api=1&query=35.1654321,33.3256789"
        );
        for [latitude, longitude] in [[35.17_f64, 33.34_f64], [35.15, 33.31]] {
            let x = (longitude - 33.3256789).to_radians()
                * 6371000.0
                * 35.1654321_f64.to_radians().cos();
            let z = -(latitude - 35.1654321).to_radians() * 6371000.0;
            let url = city.google_maps_url([x as f32, 0.0, z as f32]).unwrap();
            assert_eq!(
                url,
                format!(
                    "https://www.google.com/maps/search/?api=1&query={latitude:.7},{longitude:.7}"
                )
            );
        }
        assert!(city.google_maps_url([f32::NAN, 0.0, 0.0]).is_none());
        let no_origin = City::parse(&CROSS.replace("origin 0 0\n", "")).unwrap();
        assert!(no_origin.google_maps_url([0.0; 3]).is_none());
    }

    #[test]
    fn street_labels_follow_the_car_and_preserve_unnamed_roads() {
        let mut city = City::parse(CROSS).unwrap();
        let osm = br#"<osm>
            <way id="1"><tag k="name" v="Local name"/><tag k="name:en" v="Main &amp; First"/></way>
            <way id="2"><tag k="ref" v="B12"/></way>
            <way id="3"><tag k="name" v="   "/></way>
        </osm>"#;
        city.street_names = read_street_names(&osm[..]).unwrap();
        assert_eq!(city.street_name([-20.0, 0.5, 2.0]), "Main & First");
        assert_eq!(city.street_name([2.0, 0.5, 20.0]), "B12");
        assert_eq!(city.street_name([25.0, 0.5, 25.0]), "");
        city.street_names.remove(&2);
        assert_eq!(city.street_name([2.0, 0.5, 20.0]), "UNNAMED ROAD");
        assert!(!city.street_names.contains_key(&3));
        let local = "<osm><way id=\"1\"><tag k=\"name\" v=\"Οδός\"/></way></osm>";
        assert_eq!(read_street_names(local.as_bytes()).unwrap()[&1], "Οδός");
        assert!(read_street_names(&b"<osm><way id='1'"[..]).is_err());
    }

    #[test]
    fn branches_are_open_and_dead_ends_do_not_join_other_streets() {
        let city = City::parse(CROSS).unwrap();
        for i in 0..4 {
            for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
                let s = city.sample(i, t);
                let mut p = s.lane_position(0.0);
                let before = p;
                city.constrain(&mut p, &mut [0.0; 2]);
                assert_eq!(p, before);
            }
        }
        assert!(city.road_clearance([25.0, 0.0, 25.0]) > 10.0);
        let mut p = [0.0, 0.48, 45.0];
        city.constrain(&mut p, &mut [0.0, 10.0]);
        assert!(p[2] <= 42.61);
    }
    #[test]
    fn building_contacts_and_camera_respect_actual_footprints() {
        let collision =
            City::parse(&CROSS.replace("8 8 18 8 18 18 8 18", "8 -4 18 -4 18 4 8 4")).unwrap();
        let mut position = [10.0, 0.48, 0.0];
        let mut velocity = [10.0, 0.0];
        assert!(collision.constrain(&mut position, &mut velocity) > 0.0);
        assert!(!collision.buildings[0].contains([position[0], position[2]]));
        let city = City::parse(CROSS).unwrap();
        let b = &city.buildings[0];
        assert!(b.contains([10.0, 10.0]));
        assert!(!b.contains([7.0, 10.0]));
        let eye = city.camera_eye([0.0, 1.0, 10.0], [25.0, 4.0, 10.0]);
        assert!(eye[0] < 7.6);
        assert_eq!(
            city.camera_eye([0.0, 12.0, 10.0], [25.0, 14.0, 10.0]),
            [25.0, 14.0, 10.0]
        );
    }
    #[test]
    fn engomi_ground_contact_props_and_foundations_share_elevation() {
        let city_track = crate::track::TrackId::Engomi.track();
        let city = city_track.city().unwrap();
        let mut min = f32::INFINITY;
        let mut max = f32::NEG_INFINITY;
        for i in 0..city.streets.len() {
            for t in [0.0, 0.37, 1.0] {
                let sample = city.sample(i, t);
                min = min.min(sample.point[1]);
                max = max.max(sample.point[1]);
                assert!(
                    (sample.point[1] - city.height([sample.point[0], sample.point[2]])).abs()
                        < 0.001
                );
                let mut p = sample.lane_position(0.7);
                city.constrain(&mut p, &mut [0.0; 2]);
                assert!((p[1] - city.drive_height([p[0], p[2]]) - 0.48).abs() < 0.001);
            }
        }
        assert!(max - min > 20.0 && max - min < 1000.0);
        for prop in &city.furniture {
            assert!(
                (prop.position[1] - city.height([prop.position[0], prop.position[2]])).abs()
                    < 0.001
            );
        }
        for building in &city.buildings {
            for i in 0..building.polygon.len() {
                for p in city.ground_segment(
                    building.polygon[i],
                    building.polygon[(i + 1) % building.polygon.len()],
                ) {
                    assert!(building.base_y >= p[1]);
                }
            }
        }
    }
    #[test]
    fn sidewalks_and_parking_are_drivable_at_rendered_height_and_crop_stays_solid() {
        let city_track = crate::track::TrackId::Engomi.track();
        let city = city_track.city().unwrap();
        let mut checked = 0;
        for triangle in &city.surfaces.as_ref().unwrap().triangles {
            if !matches!(triangle.style, 5 | 6) {
                continue;
            }
            let q = [
                triangle.points.iter().map(|p| p[0]).sum::<f32>() / 3.0,
                triangle.points.iter().map(|p| p[1]).sum::<f32>() / 3.0,
            ];
            // Sliver centroids can round onto shared edges at city coordinates.
            // Sample pavement interiors, outside the 2 mm edge tolerance.
            if (0..3).any(|i| {
                let a = triangle.points[i];
                let b = triangle.points[(i + 1) % 3];
                let edge = nearest_edge(q, a, b);
                (q[0] - edge[0]).hypot(q[1] - edge[1]) < 0.02
            }) || city.building_clearance(q) < CAR_RADIUS + 0.1
            {
                continue;
            }
            let mut p = [q[0], 0.0, q[1]];
            let mut velocity = [0.0; 2];
            assert_eq!(city.constrain(&mut p, &mut velocity), 0.0);
            assert!((p[0] - q[0]).hypot(p[2] - q[1]) < 0.001);
            assert!(
                (p[1]
                    - city.height(q)
                    - (0.48 + city.surfaces.as_ref().unwrap().pavement_height - 0.05))
                    .abs()
                    < 0.001,
                "q={q:?} offset={} style={}",
                p[1] - city.height(q),
                triangle.style
            );
            assert!(city.paved(q));
            checked += 1;
            if checked == 20 {
                break;
            }
        }
        assert_eq!(checked, 20);
        let mut parking = 0;
        for triangle in &city.landscape.ground {
            if triangle.style != 3 {
                continue;
            }
            let q = [
                triangle.points.iter().map(|p| p[0]).sum::<f32>() / 3.0,
                triangle.points.iter().map(|p| p[1]).sum::<f32>() / 3.0,
            ];
            // Land cover is layered underneath streets; test exposed parking.
            if city.surfaces.as_ref().unwrap().paving_height(q).is_some()
                || city.building_clearance(q) < CAR_RADIUS + 0.1
            {
                continue;
            }
            let mut p = [q[0], 0.0, q[1]];
            city.constrain(&mut p, &mut [0.0; 2]);
            assert!((p[0] - q[0]).hypot(p[2] - q[1]) < 0.001);
            assert!(city.paved(q));
            parking += 1;
            if parking == 20 {
                break;
            }
        }
        assert_eq!(parking, 20);
        let mut p = [city.bounds[1][0] + 10.0, 0.0, 0.0];
        let mut velocity = [10.0, 0.0];
        assert!(city.constrain(&mut p, &mut velocity) > 0.0);
        assert!(p[0] <= city.bounds[1][0] - CAR_RADIUS && velocity[0] <= 0.0);
    }

    #[test]
    fn expanded_city_has_connected_streets_across_the_original_extract_edges() {
        let city_track = crate::track::TrackId::Engomi.track();
        let city = city_track.city().unwrap();
        let old_north = -0.006 * std::f32::consts::PI / 180.0 * 6_371_000.0;
        let old_east =
            0.008 * std::f32::consts::PI / 180.0 * 6_371_000.0 * 35.162_f32.to_radians().cos();
        assert!(city.bounds[0][1] < old_north - 800.0);
        assert!(city.bounds[1][0] > old_east + 600.0);
        for (axis, boundary) in [(2, old_north), (0, old_east)] {
            let crossings = city
                .streets
                .iter()
                .filter(|r| {
                    r.path
                        .windows(2)
                        .any(|p| (p[0][axis] - boundary) * (p[1][axis] - boundary) < 0.0)
                })
                .count();
            assert!(
                crossings >= 5,
                "missing connected roads across former map edge on axis {axis}: {crossings}"
            );
        }
    }

    #[test]
    fn urban_map_covers_centres_on_both_sides_and_queries_only_nearby_streets() {
        let city_track = crate::track::TrackId::Engomi.track();
        let city = city_track.city().unwrap();
        assert!(city.streets.len() > 100_000 && city.buildings.len() > 70_000);
        for (name, lat, lon) in [
            ("Nicosia", 35.1746503, 33.3638783),
            ("Strovolos", 35.1328993, 33.3454972),
            ("Lakatamia", 35.1200946, 33.3078496),
            ("Aglandjia", 35.1474032, 33.4002217),
            ("Gonyeli", 35.2164170, 33.3058171),
            ("Taskinkoy", 35.2070072, 33.3457502),
        ] {
            let p = [
                ((lon - 33.32_f64).to_radians() * 6_371_000.0 * 35.162_f64.to_radians().cos())
                    as f32,
                0.0,
                (-(lat - 35.162_f64).to_radians() * 6_371_000.0) as f32,
            ];
            let road = city.project(p);
            assert!(
                (road.point[0] - p[0]).hypot(road.point[2] - p[2]) < 250.0,
                "missing public surface roads near {name}"
            );
        }
        let bounds = [[-200.0, -130.0], [200.0, 130.0]];
        let intersects = |a: [f32; 3], b: [f32; 3]| {
            a[0].min(b[0]) <= bounds[1][0]
                && a[0].max(b[0]) >= bounds[0][0]
                && a[2].min(b[2]) <= bounds[1][1]
                && a[2].max(b[2]) >= bounds[0][1]
        };
        let nearby = city.map_segments_in(bounds);
        assert!(nearby.len() < city.streets.len() / 10);
        let actual = nearby.iter().filter(|(a, b)| intersects(*a, *b)).count();
        let all = city
            .streets
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
        let city = City::parse(CROSS).unwrap();
        for p in [
            [125.0, 0.0, 175.0],
            [-200.0, 0.0, -300.0],
            [10.0, 0.0, 1000.0],
        ] {
            let indexed = city.project(p);
            let brute = (0..city.streets.len())
                .map(|i| city.projected(i, p))
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .unwrap()
                .1;
            assert!(
                (indexed.point[0] - brute.point[0]).hypot(indexed.point[2] - brute.point[2])
                    < 0.001
            );
        }
    }

    #[test]
    fn invalid_map_records_fail_clearly() {
        assert!(City::parse("CITY 2").is_err());
        assert!(City::parse("CITY 1\nn NaN 0").is_err());
        assert!(City::parse("CITY 1\nr 0 1 8 0 1").is_err());
    }
    #[test]
    fn compiled_street_paths_render_and_collide_on_the_same_surface() {
        let city_track = crate::track::TrackId::Engomi.track();
        let city = city_track.city().unwrap();
        let surfaces = city.surfaces.as_ref().unwrap();
        let mut styles = [false; 8];
        for triangle in &surfaces.triangles {
            styles[triangle.style] = true;
        }
        assert!(styles.into_iter().all(|present| present));
        for i in 0..city.streets.len() {
            for t in [0.0, 0.5, 1.0] {
                let road = city.sample(i, t);
                assert!(
                    surfaces.contains([road.point[0], road.point[2]]),
                    "street {i} at {t} is outside the rendered road"
                );
            }
        }
        let mut adjacency = vec![Vec::new(); city.nodes.len()];
        for (i, r) in city.streets.iter().enumerate() {
            for n in r.ends {
                adjacency[n].push(i);
            }
        }
        for (n, arms) in adjacency.iter().enumerate() {
            if arms.len() == 2 {
                let endpoint = |i: usize| {
                    let r = &city.streets[i];
                    if r.ends[0] == n {
                        r.path[0]
                    } else {
                        *r.path.last().unwrap()
                    }
                };
                let a = endpoint(arms[0]);
                let b = endpoint(arms[1]);
                assert!(
                    (a[0] - b[0]).hypot(a[2] - b[2]) < 0.001,
                    "curve join disconnected"
                );
                assert!(
                    (a[0] - city.nodes[n][0]).hypot(a[2] - city.nodes[n][2]) <= 0.601,
                    "curve moved a mapped bend too far"
                );
            }
        }
    }

    #[test]
    fn bundled_nicosia_retains_real_scale_networks_and_buildings() {
        let city_track = crate::track::TrackId::Engomi.track();
        let city = city_track.city().unwrap();
        assert!(city.nodes.len() > 1000 && city.buildings.len() > 1000);
        assert!(city.bounds[1][0] - city.bounds[0][0] > 1400.0);
        let mut visited = std::collections::HashSet::new();
        let mut todo = vec![city.streets[0].ends[0]];
        let mut neighbors = vec![Vec::new(); city.nodes.len()];
        for r in &city.streets {
            neighbors[r.ends[0]].push(r.ends[1]);
            neighbors[r.ends[1]].push(r.ends[0]);
        }
        while let Some(i) = todo.pop() {
            if visited.insert(i) {
                todo.extend(neighbors[i].iter().copied());
            }
        }
        assert!(
            visited.len() > city.nodes.len() / 2,
            "the main network should remain coherent"
        );
        assert!(
            visited.len() < city.nodes.len(),
            "disconnected surface networks must not be discarded or joined by invented roads"
        );
        let building = &city.buildings[0];
        let p = building.polygon[0];
        let recovered = city.recovery([p[0], 0.48, p[1]]);
        assert!(city.clear_for_car(recovered.point));
        let s = city.spawn();
        assert!(city.road_clearance(s.point) < 0.0);
        assert!(city
            .buildings
            .iter()
            .all(|b| !b.contains([s.point[0], s.point[2]])));
        for r in &city.streets {
            for n in r.ends {
                let p = city.nodes[n];
                assert!(city
                    .project(p)
                    .point
                    .iter()
                    .zip(p)
                    .all(|(a, b)| (a - b).abs() < 0.61));
            }
        }
    }
}

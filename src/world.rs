use crate::city_assets;
use crate::geometry::{BoundingBoxXZ, LatLon, XYZ, XZ};
use crate::landscape;
use crate::track;

mod barriers;
mod buildings;
mod preparation;
mod raised_roads;
mod roads;
pub mod spatial;
use barriers::Barriers;
use buildings::Buildings;
pub use preparation::CompiledLocation;
use raised_roads::RaisedRoads;
use roads::RoadNetwork;
use std::collections::HashMap;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BarrierKind {
    Wall,
    Fence,
    ChainLink,
    WoodFence,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Barrier {
    pub source_id: u64,
    pub kind: BarrierKind,
    pub ends: [XZ<f32>; 2],
    pub height_meters: f32,
    pub width_meters: f32,
}

/// Road-space triangles with height offsets; terrain remains an independent surface.
#[derive(Debug, Clone, PartialEq)]
pub struct RoadRise {
    pub points: [XYZ; 3],
}

/// Prepared runtime world. Constructed by asset loading or authored circuit preparation.
#[derive(Debug)]
pub struct Location {
    pub id: LocationId,
    assets: Option<city_assets::CityAsset>,
    pub display_name: String,

    /// Defines the location's size, in meters.
    bounding_box: BoundingBoxXZ<f32>,
    /// Geographic position corresponding to local X = 0, Z = 0, when known.
    pub origin: Option<LatLon>,

    /// Turns the flat area into slopes and hills.
    elevation: Option<HeightField>,

    roads: RoadNetwork,
    ground: landscape::GroundCover,
    buildings: Buildings,
    barriers: Barriers,
    raised_roads: RaisedRoads,

    /// Traffic lights and road signs and stuff like that.
    pub furniture: Vec<Furniture>,

    //
    //
    //
    /// Street labels keyed by source OSM way ID.
    street_names: HashMap<i64, String>,
    /// Building labels keyed by source building ID.
    building_names: HashMap<u64, String>,
    storefront_names: HashMap<u64, Vec<String>>,
}

/// Regular terrain grid whose cells are triangulated for consistent height and contact queries.
#[derive(Debug, PartialEq)]
pub struct HeightField {
    /// Number of samples along increasing X.
    pub columns: usize,
    /// Number of samples along increasing Z.
    pub rows: usize,
    /// Local [X, Z] position of the first sample in metres.
    pub origin: XZ<f32>,
    /// Spacing between adjacent samples along either axis in metres.
    pub step: f32,
    /// Row-major Y elevations in metres, indexed by row * columns + column.
    pub values: Vec<f32>,
}

/// Prepared road centreline and metadata at runtime precision.
#[derive(Clone, Debug)]
pub struct Road {
    /// Start and end indices into Location::road_nodes.
    pub ends: [usize; 2],
    /// Source OSM way ID, or a generated way ID for an authored circuit.
    pub osm_way_id: Option<i64>,
    /// Full road width at the start of this segment in metres.
    road_width_start_meters: f32,
    /// Full road width at the end; equals the start width for constant-width roads.
    road_width_end_meters: f32,
    /// Permitted travel: 0 for both directions, 1 from start to end, -1 in reverse.
    pub direction: i8,
    /// Horizontal centreline length in metres; populated at runtime.
    pub length: f32,
    /// Cumulative street offset in metres; continuous arc length for an ordered circuit.
    pub distance: f32,
    /// Centreline samples in local metres; populated at runtime.
    pub path: Vec<XYZ>,
    /// Cumulative horizontal distances from the first path sample; populated at runtime.
    path_distances: Vec<f32>,
}

/// Position and local road frame returned by road sampling or nearest-road projection.
#[derive(Debug, Clone, Copy)]
pub struct RoadSample {
    /// Road centreline position in local [X, Y, Z] metres.
    pub point: XYZ,
    /// Unit [X, Z] direction along the road.
    pub tangent: XZ<f32>,
    /// Cumulative distance through roads in storage order, in horizontal metres.
    pub distance: f32,
    /// Signed lateral distance in metres along [-tangent Z, tangent X]; zero for direct sampling.
    pub offset: f32,
    /// Full road width in metres at this position.
    pub width: f32,
}

/// Compiled edge between the road and its surroundings.
#[derive(Debug, PartialEq)]
pub struct Curb {
    /// Class of the nearest road.
    pub style: CurbStyle,
    /// Edge endpoints in local [X, Z] metres.
    pub ends: [XZ<f32>; 2],
    /// Unit [X, Z] normal pointing into the road surface.
    pub inward: XZ<f32>,
}

/// Nearest road class used for curb appearance.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum CurbStyle {
    MajorRoad = 0,
    Road = 1,
    ServiceRoad = 2,
}

/// Tree placement and dimensions: f64 during compilation, f32 at runtime by default.
#[derive(Debug, PartialEq)]
pub struct Tree<T = f32> {
    /// Source or generated tree identifier.
    pub id: u64,
    /// Local position in metres, using the same precision as the dimensions.
    pub coords: XZ<T>,
    /// Tree height above the ground in metres.
    pub height_meters: T,
    /// Estimated crown radius in metres.
    pub crown_radius_meters: T,
}

/// Building geometry: f64 during compilation, f32 at runtime by default.
#[derive(Debug)]
pub struct Building<T = f32> {
    /// Source building ID, or a generated ID for authored scenery.
    pub id: u64,
    /// Footprint vertices in local [X, Z] metres, with an implicit closing edge.
    pub ground_outline: Vec<XZ<T>>,
    /// Building height above base_y in metres.
    pub height_meters: T,
    /// World Y elevation of the base; zero until fitted to terrain at load time.
    pub base_y: T,
    /// Whether height came from an explicit height tag rather than an estimate.
    pub tagged: bool,
}

/// Traffic control or sign placed beside a road.
#[derive(Debug)]
pub struct Furniture {
    /// Source OSM node ID identifying the control.
    pub _source_id: u64,
    /// Traffic light or sign category.
    pub kind: FurnitureKind,
    /// Base position in local [X, Y, Z] metres, fitted to terrain during city loading.
    pub coords: XYZ,
    /// Unit [X, Z] direction of the traffic controlled by this feature.
    pub travel: XZ<f32>,
    /// Posted speed in km/h for speed signs; otherwise unused.
    pub value: u16,
    /// Traffic-light cycle offset in seconds, in the range 0 through 49.
    pub phase: u8,
}

/// Category of mapped roadside control.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FurnitureKind {
    Signal,
    Stop,
    Yield,
    Speed,
}

/// Ground or street triangle draped onto terrain for rendering and collision.
#[derive(Debug, PartialEq)]
pub struct Triangle {
    pub material: Material,
    pub usage: TriangleUsage,
    /// Triangle vertices in local X/Z metres; street surfaces use counterclockwise winding.
    pub points: [XZ<f32>; 3],
}

/// Appearance shared by mapped ground cover and street surfaces.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Material {
    Grass,
    Park,
    Scrub,
    Asphalt,
    Paving,
    Concrete,
    BareGround,
    Sports,
    Water,
    Paint,
}

/// Geometry's purpose and elevation, independent of its material.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TriangleUsage {
    Ground,
    Road(CurbStyle),
    Gutter,
    Marking,
    Pavement(CurbStyle),
}

//
//---------------------------------------------
//

impl TriangleUsage {
    pub fn is_road(self) -> bool {
        matches!(self, Self::Road(_) | Self::Gutter)
    }

    pub fn support_height(self, pavement_height: f32) -> f32 {
        match self {
            Self::Ground => 0.005,
            Self::Pavement(CurbStyle::ServiceRoad) => 0.08,
            Self::Pavement(_) => pavement_height,
            _ => 0.05,
        }
    }
}

impl Building {
    pub fn contains(&self, p: XZ<f32>) -> bool {
        let mut inside = false;
        for i in 0..self.ground_outline.len() {
            let a = self.ground_outline[i];
            let b = self.ground_outline[(i + 1) % self.ground_outline.len()];
            if (a.z > p.z) != (b.z > p.z) && p.x < (b.x - a.x) * (p.z - a.z) / (b.z - a.z) + a.x {
                inside = !inside;
            }
        }
        inside
    }
    pub fn nearest(&self, p: XZ<f32>) -> (XZ<f32>, XZ<f32>, f32) {
        let mut best = (XZ::<f32>::default(), XZ::<f32>::default(), f32::INFINITY);
        for i in 0..self.ground_outline.len() {
            let a = self.ground_outline[i];
            let b = self.ground_outline[(i + 1) % self.ground_outline.len()];
            let q = spatial::nearest_edge(p, a, b);
            let distance = (p.x - q.x).hypot(p.z - q.z);
            if distance < best.2 {
                let len = (b.x - a.x).hypot(b.z - a.z).max(0.0001);
                best = (
                    (q),
                    XZ {
                        x: (b.z - a.z) / len,
                        z: -(b.x - a.x) / len,
                    },
                    distance,
                );
            }
        }
        best
    }
}

impl Location {
    fn empty() -> Self {
        Self {
            roads: RoadNetwork::empty(),
            id: LocationId::Compact,
            assets: None,
            display_name: String::new(),
            origin: None,
            buildings: Buildings::new(Vec::new()),
            barriers: Barriers::new(Vec::new()),
            raised_roads: RaisedRoads::new(Vec::new()),
            furniture: Vec::new(),
            bounding_box: BoundingBoxXZ::default(),
            elevation: None,
            ground: landscape::GroundCover::new(Vec::new(), Vec::new()),
            street_names: HashMap::new(),
            building_names: HashMap::new(),
            storefront_names: HashMap::new(),
        }
    }
}

impl Location {
    pub fn height(&self, p: XZ<f32>) -> f32 {
        self.elevation.as_ref().map_or_else(
            || {
                if self.road_lines().is_empty() {
                    0.0
                } else {
                    self.project(XYZ {
                        x: p.x,
                        y: 0.0,
                        z: p.z,
                    })
                    .point
                    .y
                }
            },
            |field| field.height(p),
        )
    }
    pub fn ground_segment(&self, a: XZ<f32>, b: XZ<f32>) -> Vec<XYZ> {
        self.elevation.as_ref().map_or_else(
            || vec![a.at_height(0.0), b.at_height(0.0)],
            |f| f.segment(a, b),
        )
    }
    /// Distance to the union of street capsules, including shared junction nodes.
    pub fn road_clearance(&self, p: XYZ) -> f32 {
        if self.has_compiled_roads() {
            return self.surface_clearance(XZ { x: p.x, z: p.z });
        }
        self.candidates(p)
            .map(|i| {
                let (d, s) = self.projected(i, p);
                d - s.width * 0.5
            })
            .fold(f32::INFINITY, f32::min)
    }
    pub fn building_name(&self, id: u64) -> Option<&str> {
        self.building_names.get(&id).map(String::as_str)
    }

    pub fn storefront_names(&self, id: u64) -> Vec<&str> {
        let mut names: Vec<_> = self
            .storefront_names
            .get(&id)
            .into_iter()
            .flatten()
            .map(String::as_str)
            .collect();
        if let Some(name) = self.building_name(id) {
            if !names.iter().any(|n| n.eq_ignore_ascii_case(name)) {
                names.insert(0, name);
            }
        }
        names
    }

    pub fn terrain_bounds(&self) -> BoundingBoxXZ<f32> {
        if let Some(field) = self.elevation.as_ref() {
            BoundingBoxXZ {
                min: XZ {
                    x: field.origin.x,
                    z: field.origin.z,
                },
                max: XZ {
                    x: field.origin.x + (field.columns - 1) as f32 * field.step,
                    z: field.origin.z + (field.rows - 1) as f32 * field.step,
                },
            }
        } else {
            let bounds = self.bounding_box;
            BoundingBoxXZ {
                min: XZ {
                    x: bounds.min.x - track::TERRAIN_PADDING,
                    z: bounds.min.z - track::TERRAIN_PADDING,
                },
                max: XZ {
                    x: bounds.max.x + track::TERRAIN_PADDING,
                    z: bounds.max.z + track::TERRAIN_PADDING,
                },
            }
        }
    }

    /// Ground rise per horizontal metre, excluding discrete curb impacts.
    pub fn ground_gradient(&self, position: XYZ) -> XZ<f32> {
        let height = |x, z| self.height(XZ { x, z }) + self.road_rise(XZ { x, z });
        let XYZ { x, z, .. } = position;
        XZ {
            x: height(x + 0.5, z) - height(x - 0.5, z),
            z: height(x, z + 0.5) - height(x, z - 0.5),
        }
    }
}

impl RoadSample {
    pub fn normal(self) -> XZ<f32> {
        XZ {
            x: -self.tangent.z,
            z: self.tangent.x,
        }
    }
    /// Centreline position shifted laterally, retaining the road elevation.
    pub fn lane_position(self, offset: f32) -> XYZ {
        let normal = self.normal();
        XYZ {
            x: self.point.x + normal.x * offset,
            y: self.point.y,
            z: self.point.z + normal.z * offset,
        }
    }
    pub fn heading(self) -> f32 {
        self.tangent.x.atan2(self.tangent.z)
    }
}

impl Road {
    /// Create a constant-width, two-way road before its runtime path is sampled.
    fn new(ends: [usize; 2], width_meters: f32) -> Self {
        Self {
            ends,
            osm_way_id: None,
            road_width_start_meters: width_meters,
            road_width_end_meters: width_meters,
            direction: 0,
            length: 0.0,
            distance: 0.0,
            path: Vec::new(),
            path_distances: Vec::new(),
        }
    }
}

/// Identity of a built-in circuit, generated asset directory, or authored track file.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum LocationId {
    /// Built-in compact test circuit.
    Compact,
    /// Canonical generated asset directory; independent of all other loaded worlds.
    Asset(std::path::PathBuf),
    /// Index into the registry of authored track files.
    File(usize),
}

impl Location {
    pub fn assets(&self) -> Option<&city_assets::CityAsset> {
        self.assets.as_ref()
    }
}

impl Location {
    pub fn elevation(&self) -> Option<&HeightField> {
        self.elevation.as_ref()
    }
    pub fn ground_triangles(&self) -> &[Triangle] {
        self.ground.triangles()
    }
    pub fn trees(&self) -> &[Tree] {
        self.ground.trees()
    }
    pub fn ground_material(&self, p: XZ<f32>) -> Option<Material> {
        self.ground.material(p)
    }
}

impl Location {
    pub fn bounding_box(&self) -> BoundingBoxXZ<f32> {
        self.bounding_box
    }
}

#[cfg(test)]
pub use roads::test_support;

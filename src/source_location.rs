//! Compiler geometry and parsed CITY records, without runtime layers or indexes.
//! Importers and generators keep f64 geometry. CITY parsing deliberately retains
//! the existing f32 quantization; writers retain their existing decimal precision.
use crate::geometry::{BoundingBoxXZ, LatLon, XYZ, XZ};
use crate::world::Building;
use geo::{Intersects, MultiPolygon, Point};

#[derive(Debug)]
pub struct SourceLocation<T = f64> {
    pub bounding_box: BoundingBoxXZ<T>,
    pub origin: Option<LatLon>,
    pub boundary: Option<MultiPolygon<f64>>,
    pub road_nodes: Vec<XYZ<T>>,
    pub road_lines: Vec<SourceRoad<T>>,
    pub buildings: Vec<Building<T>>,
}

/// Road metadata at source precision; path lengths and indexes belong to preparation.
#[derive(Clone, Debug)]
pub struct SourceRoad<T = f64> {
    pub ends: [usize; 2],
    pub osm_way_id: Option<i64>,
    pub road_width_start_meters: T,
    pub direction: i8,
    pub kind: usize,
    pub lanes: usize,
}

impl<T: Copy> SourceRoad<T> {
    pub fn new(ends: [usize; 2], width: T) -> Self {
        Self {
            ends,
            osm_way_id: None,
            road_width_start_meters: width,
            direction: 0,
            kind: 1,
            lanes: 2,
        }
    }
}

impl<T: Default> SourceLocation<T> {
    pub fn empty() -> Self {
        Self {
            bounding_box: BoundingBoxXZ::default(),
            origin: None,
            boundary: None,
            road_nodes: Vec::new(),
            road_lines: Vec::new(),
            buildings: Vec::new(),
        }
    }
}

impl SourceLocation<f64> {
    /// Project a geographic position into local metres; requires a geographic origin.
    pub fn metres(&self, p: LatLon) -> XZ {
        let origin = self
            .origin
            .expect("geographic projection requires a location origin");
        XZ {
            x: (p.lon - origin.lon).to_radians() * 6371000.0 * origin.lat.to_radians().cos(),
            z: -(p.lat - origin.lat).to_radians() * 6371000.0,
        }
    }
    pub fn contains(&self, p: XZ) -> bool {
        p.x >= self.bounding_box.min.x
            && p.x <= self.bounding_box.max.x
            && p.z >= self.bounding_box.min.z
            && p.z <= self.bounding_box.max.z
            && self
                .boundary
                .as_ref()
                .is_none_or(|b| b.intersects(&Point::new(p.x, p.z)))
    }
}

impl SourceLocation<f32> {
    pub fn parse(text: &str) -> Result<Self, String> {
        crate::asset_codecs::city::parse(text)
    }
}

/// Compatibility for old asset folders; build inputs never enter stored Location.
pub fn read_legacy_labels(
    directory: &std::path::Path,
) -> Result<crate::location::Labels, String> {
    let labels = match std::fs::File::open(directory.join("source.osm")) {
        Ok(file) => match crate::city::read_osm_labels(std::io::BufReader::new(file)) {
            Ok(labels) => labels,
            Err(e) => {
                log::warn!("Cannot load legacy labels: {e}");
                return Ok(crate::location::Labels::default());
            }
        },
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            return Ok(crate::location::Labels::default())
        }
        Err(e) => return Err(format!("source.osm labels: {e}")),
    };
    let mut streets: Vec<_> = labels.streets.into_iter().collect();
    let mut buildings: Vec<_> = labels.buildings.into_iter().collect();
    streets.sort();
    buildings.sort();
    Ok(crate::location::Labels { streets, buildings })
}

/// Build-stage adapter: reduce source tags to playable labels on every rebuild.
pub fn write_labels(directory: &std::path::Path) -> crate::error::Result<()> {
    std::fs::write(
        directory.join("labels.json"),
        serde_json::to_vec(&read_legacy_labels(directory).map_err(crate::error::invalid)?)?,
    )?;
    Ok(())
}

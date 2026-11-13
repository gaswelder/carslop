//! Controlled construction of compiled and authored runtime worlds.
use super::*;
use crate::authoring::Circuit;

/// A world with compiled street surfaces and all required bake layers installed.
/// Read-only access prevents a caller from removing layers before scene baking.
#[derive(Debug)]
pub struct CompiledLocation(Location);
impl std::ops::Deref for CompiledLocation {
    type Target = Location;
    fn deref(&self) -> &Location {
        &self.0
    }
}
impl CompiledLocation {
    /// Consume the bake capability and share the prepared world with simulation.
    fn into_location(self) -> Location {
        self.0
    }
}

impl Location {
    pub fn load(assets: &crate::city_assets::CityAsset) -> Result<Self, String> {
        CompiledLocation::load(assets).map(CompiledLocation::into_location)
    }
}

impl CompiledLocation {
    /// Parse and prepare every required generated layer before exposing a bake input.
    /// Landscape and furniture may be header-only. Labels, imagery trees, and
    /// decorations are optional; this constructor does not consult global selection.
    pub fn load(assets: &crate::city_assets::CityAsset) -> Result<Self, String> {
        assets
            .resolved()
            .map_err(|e| e.to_string())
            .and_then(|assets| Location::load_assets(&assets))
            .map(Self)
            .map_err(|error| {
                format!(
                    "cannot prepare location {}: {error}",
                    assets.directory().display()
                )
            })
    }
}

impl Location {
    /// Install parsed (already quantized) CITY geometry into a runtime draft.
    /// Only controlled preparation in this module can publish that draft.

    fn load_assets(assets: &crate::city_assets::CityAsset) -> Result<Self, String> {
        let stored = crate::location::Location::read(assets.directory())?;
        let mut city = Self::empty();
        city.display_name = stored.name.replace(['_', '-'], " ").to_ascii_uppercase();
        city.origin = stored.origin;
        city.bounding_box = stored.extent;
        city.street_names = stored.labels.streets.into_iter().collect();
        city.building_names = stored.labels.buildings.into_iter().collect();
        for (id, name) in stored.labels.storefronts {
            city.storefront_names.entry(id).or_default().push(name);
        }
        let field = stored.elevation;
        let (surfaces, paths) = crate::street_surface::StreetSurface::from_geometry(
            stored.streets,
            stored.pavement_height_meters,
        )?;
        city.roads = RoadNetwork::from_stored(stored.road_nodes, stored.roads)
            .with_compiled(surfaces, paths, &field)?;
        city.buildings = Buildings::new(
            stored
                .buildings
                .into_iter()
                .map(|b| Building {
                    id: b.id,
                    ground_outline: b.outline,
                    height_meters: b.height_meters,
                    base_y: 0.0,
                    tagged: false,
                })
                .collect(),
        )
        .with_terrain(&field);
        city.furniture = stored
            .furniture
            .into_iter()
            .map(|f| Furniture {
                _source_id: f.id,
                kind: f.kind,
                coords: f.position.at_height(field.height(f.position)),
                travel: f.travel,
                value: f.value,
                phase: f.phase,
            })
            .collect();
        city.ground = crate::landscape::GroundCover::new(stored.ground, stored.trees);
        city.elevation = Some(field);
        city.id = LocationId::Asset(assets.directory().to_path_buf());
        city.assets = Some(assets.clone());
        Ok(city)
    }
}

impl Location {
    pub fn prepare_authored(circuit: &Circuit, id: LocationId) -> Result<Self, String> {
        circuit.validate()?;
        let mut location = Location::from_circuit(circuit, id == LocationId::Compact)?;
        location.id = id;
        location.display_name = "COASTAL LOOP".into();
        let terrain_bounds = location.terrain_bounds();
        location.elevation = Some(HeightField::generate(terrain_bounds, |position| {
            let road = location.project(XYZ {
                x: position.x,
                y: 0.0,
                z: position.z,
            });
            let beyond =
                (road.offset.abs() - road.width * 0.5 - crate::track::OFFROAD_MARGIN).max(0.0);
            let t = (beyond / (crate::track::TERRAIN_PADDING - crate::track::OFFROAD_MARGIN))
                .clamp(0.0, 1.0);
            let blend = t * t * (3.0 - 2.0 * t);
            let outer = if circuit.coastal {
                crate::elevation::SEA_LEVEL - 0.4
            } else {
                road.point.y - 2.0
            };
            road.point.y + (outer - road.point.y) * blend
        }));
        let scenery = crate::scene::authored_scenery(&location, circuit.coastal);
        location.buildings = Buildings::new(scenery.buildings);
        let ground = crate::scene::bake_authored_landscape(&location, circuit.coastal);
        location.ground = crate::landscape::GroundCover::new(ground, scenery.trees);
        Ok(location)
    }
    /// Sample an authored closed spline into the ordinary indexed street representation.
    fn from_circuit(circuit: &Circuit, compact: bool) -> Result<Self, String> {
        let mut points = Vec::new();
        let mut widths = Vec::new();
        let controls = &circuit.control_points;
        let n = controls.len();
        for segment in 0..n {
            let a = controls[(segment + n - 1) % n];
            let b = controls[segment];
            let c = controls[(segment + 1) % n];
            let d = controls[(segment + 2) % n];
            let steps = if compact {
                16
            } else {
                (((c.position.x - b.position.x).hypot(c.position.z - b.position.z)).ceil() as usize)
                    .max(16)
            };
            for i in 0..steps {
                let t = i as f32 / steps as f32;
                let blend = t * t * (3.0 - 2.0 * t);
                widths.push(b.width + (c.width - b.width) * blend);
                points.push(XYZ::from_fn(|axis| {
                    0.5 * (2.0 * b.position.component(axis)
                        + (-a.position.component(axis) + c.position.component(axis)) * t
                        + (2.0 * a.position.component(axis) - 5.0 * b.position.component(axis)
                            + 4.0 * c.position.component(axis)
                            - d.position.component(axis))
                            * t
                            * t
                        + (-a.position.component(axis) + 3.0 * b.position.component(axis)
                            - 3.0 * c.position.component(axis)
                            + d.position.component(axis))
                            * t
                            * t
                            * t)
                }));
            }
        }
        let mut city = Self::empty();
        city.bounding_box = crate::world::spatial::bounds(points.iter().map(|p| p.xz()), 0.0);
        city.roads = RoadNetwork::authored(points, widths)?;
        Ok(city)
    }
}

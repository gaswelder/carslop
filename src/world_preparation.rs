//! Controlled construction of compiled and authored runtime worlds.

use crate::build_authoring;
use crate::build_authoring::Circuit;
use crate::model_location;
use crate::model_world;
use crate::model_world::{Building, Furniture, LocationId, WorldLocation};
use crate::render_scene;
use crate::storage_city_assets;
use crate::util_geometry;
use crate::util_geometry::XYZ;
use crate::world_barriers;
use crate::world_buildings;
use crate::world_elevation;
use crate::world_landscape;
use crate::world_locations;
use crate::world_map_details;
use crate::world_raised_roads;
use crate::world_roads;
use crate::world_spatial;
use crate::world_street_surface;

/// A world with compiled street surfaces and all required bake layers installed.
/// Read-only access prevents a caller from removing layers before scene baking.
#[derive(Debug)]
pub struct CompiledLocation(WorldLocation);
impl std::ops::Deref for CompiledLocation {
    type Target = WorldLocation;
    fn deref(&self) -> &WorldLocation {
        &self.0
    }
}

/// Consume the bake capability and share the prepared world with simulation.
fn into_location(compiled_location: CompiledLocation) -> WorldLocation {
    compiled_location.0
}

pub fn world_location_load(
    assets: &storage_city_assets::CityAsset,
) -> Result<WorldLocation, String> {
    compiled_location_load(assets).map(into_location)
}

/// Parse and prepare every required generated layer before exposing a bake input.
/// Landscape and furniture may be header-only. Labels, imagery trees, and
/// decorations are optional; this constructor does not consult global selection.
pub fn compiled_location_load(
    assets: &storage_city_assets::CityAsset,
) -> Result<CompiledLocation, String> {
    storage_city_assets::resolved(assets)
        .map_err(|e| e.to_string())
        .and_then(|assets| load_assets(&assets))
        .map(CompiledLocation)
        .map_err(|error| {
            format!(
                "cannot prepare location {}: {error}",
                storage_city_assets::directory(assets).display()
            )
        })
}

/// Install parsed (already quantized) CITY geometry into a runtime draft.
/// Only controlled preparation in this module can publish that draft.

fn load_assets(assets: &storage_city_assets::CityAsset) -> Result<WorldLocation, String> {
    let stored = model_location::location_read(storage_city_assets::directory(assets))?;
    let mut city = model_world::empty();
    city.display_name = stored.name;
    city.origin = stored.origin;
    city.bounding_box = stored.extent;
    city.street_names = stored.labels.streets.into_iter().collect();
    city.building_names = stored.labels.buildings.into_iter().collect();
    for (id, name) in stored.labels.storefronts {
        city.storefront_names.entry(id).or_default().push(name);
    }
    let field = stored.elevation;
    let (surfaces, paths) =
        world_street_surface::from_geometry(stored.streets, stored.pavement_height_meters)?;
    city.roads = world_roads::with_compiled(
        world_roads::from_stored(stored.road_nodes, stored.roads),
        surfaces,
        paths,
        &field,
    )?;
    city.buildings = world_buildings::with_terrain(
        world_buildings::new(
            stored
                .buildings
                .into_iter()
                .map(|b| Building {
                    id: b.id,
                    ground_outline: b.outline,
                    height_meters: b.height_meters,
                    base_y: 0.0,
                    tagged: false,
                    holes: b.holes,
                    min_height_meters: b.min_height_meters,
                })
                .collect(),
        ),
        &field,
    );
    for entrance in stored.entrances {
        city.entrances
            .entry(entrance.building_id)
            .or_default()
            .push(entrance.position);
    }
    city.furniture = stored
        .furniture
        .into_iter()
        .map(|f| Furniture {
            _source_id: f.id,
            kind: f.kind,
            coords: util_geometry::at_height(
                f.position,
                world_elevation::height(&field, f.position),
            ),
            travel: f.travel,
            value: f.value,
            phase: f.phase,
        })
        .collect();
    city.details = world_map_details::new(stored.details);
    city.barriers = world_barriers::new(stored.barriers);
    city.raised_roads = world_raised_roads::new(stored.road_rises);
    city.ground = world_landscape::new(stored.ground, stored.trees);
    city.elevation = Some(field);
    city.id = LocationId::Asset(storage_city_assets::directory(assets).to_path_buf());
    city.assets = Some(assets.clone());
    Ok(city)
}

pub fn prepare_authored(circuit: &Circuit, id: LocationId) -> Result<WorldLocation, String> {
    build_authoring::validate(circuit)?;
    let mut location = from_circuit(circuit, id == LocationId::Compact)?;
    location.id = id;
    location.display_name = "COASTAL LOOP".into();
    let terrain_bounds = world_elevation::terrain_bounds(&location);
    location.elevation = Some(world_elevation::generate(terrain_bounds, |position| {
        let road = world_roads::project(
            &location,
            XYZ {
                x: position.x,
                y: 0.0,
                z: position.z,
            },
        );
        let beyond =
            (road.offset.abs() - road.width * 0.5 - world_locations::OFFROAD_MARGIN).max(0.0);
        let t = (beyond / (world_locations::TERRAIN_PADDING - world_locations::OFFROAD_MARGIN))
            .clamp(0.0, 1.0);
        let blend = t * t * (3.0 - 2.0 * t);
        let outer = if circuit.coastal {
            world_elevation::SEA_LEVEL - 0.4
        } else {
            road.point.y - 2.0
        };
        road.point.y + (outer - road.point.y) * blend
    }));
    let scenery = render_scene::authored_scenery(&location, circuit.coastal);
    location.buildings = world_buildings::new(scenery.buildings);
    let ground = render_scene::bake_authored_landscape(&location, circuit.coastal);
    location.ground = world_landscape::new(ground, scenery.trees);
    Ok(location)
}
/// Sample an authored closed spline into the ordinary indexed street representation.
fn from_circuit(circuit: &Circuit, compact: bool) -> Result<WorldLocation, String> {
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
            points.push(util_geometry::xyz_from_fn(|axis| {
                0.5 * (2.0 * util_geometry::xyz_component(b.position, axis)
                    + (-util_geometry::xyz_component(a.position, axis)
                        + util_geometry::xyz_component(c.position, axis))
                        * t
                    + (2.0 * util_geometry::xyz_component(a.position, axis)
                        - 5.0 * util_geometry::xyz_component(b.position, axis)
                        + 4.0 * util_geometry::xyz_component(c.position, axis)
                        - util_geometry::xyz_component(d.position, axis))
                        * t
                        * t
                    + (-util_geometry::xyz_component(a.position, axis)
                        + 3.0 * util_geometry::xyz_component(b.position, axis)
                        - 3.0 * util_geometry::xyz_component(c.position, axis)
                        + util_geometry::xyz_component(d.position, axis))
                        * t
                        * t
                        * t)
            }));
        }
    }
    let mut city = model_world::empty();
    city.bounding_box = world_spatial::bounds(points.iter().map(|p| util_geometry::xz(*p)), 0.0);
    city.roads = world_roads::authored(points, widths)?;
    Ok(city)
}

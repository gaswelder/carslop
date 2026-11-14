//! The persisted, playable location. No spatial indexes, terrain fitting, OSM
//! objects, compiler provenance, or derived query bounds live in these records.
use crate::aerial;
use crate::asset_codecs;
use crate::city_assets;
use crate::error;
use crate::geometry::{BoundingBoxXZ, LatLon, XZ};
use crate::source_location;
use crate::street_surface;
use crate::world;
use crate::world::{Curb, FurnitureKind, HeightField, Tree, Triangle};
use std::{
    fs,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
};

#[derive(Debug)]
pub struct Location {
    pub name: String,
    pub origin: Option<LatLon>,
    /// Explicit authored extent from CITY; query bounds are derived by the game.
    pub extent: BoundingBoxXZ<f32>,
    pub road_nodes: Vec<XZ<f32>>,
    pub roads: Vec<Road>,
    pub buildings: Vec<Building>,
    pub streets: Streets,
    pub pavement_height_meters: f32,
    pub elevation: HeightField,
    pub ground: Vec<Triangle>,
    pub trees: Vec<Tree>,
    pub furniture: Vec<Furniture>,
    pub barriers: Vec<world::Barrier>,
    pub road_rises: Vec<world::RoadRise>,
    pub labels: Labels,
    /// Large render payloads stay on disk and are streamed when writing.
    media: Media,
}

#[derive(Debug, PartialEq)]
pub struct Road {
    pub ends: [usize; 2],
    pub way_id: Option<i64>,
    pub width_meters: f32,
    pub direction: i8,
}

#[derive(Debug, PartialEq)]
pub struct Building {
    pub id: u64,
    pub outline: Vec<XZ<f32>>,
    pub height_meters: f32,
}

#[derive(Debug, PartialEq)]
pub struct Furniture {
    pub id: u64,
    pub kind: FurnitureKind,
    pub position: XZ<f32>,
    pub travel: XZ<f32>,
    pub value: u16,
    pub phase: u8,
}

#[derive(Debug, PartialEq)]
pub struct Streets {
    pub paths: Vec<Vec<XZ<f32>>>,
    pub triangles: Vec<Triangle>,
    pub curbs: Vec<Curb>,
}

#[derive(Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Labels {
    pub streets: Vec<(i64, String)>,
    pub buildings: Vec<(u64, String)>,
    #[serde(default)]
    pub storefronts: Vec<(u64, String)>,
}

/// References to immutable, already compiled payloads, not a source/build folder.
/// These remain disk-backed to avoid retaining a whole city scene or tile set.
#[derive(Debug, Default)]
struct Media {
    scene: Option<Payload>,
    aerial: Vec<Payload>,
    // Own temporary compiled payloads until the location is dropped.
    workspace: Option<tempfile::TempDir>,
}
#[derive(Debug)]
struct Payload {
    source: PathBuf,
    relative: PathBuf,
}
impl Payload {
    fn read(root: &Path, relative: PathBuf) -> Result<Self, String> {
        let source = root
            .join(&relative)
            .canonicalize()
            .map_err(|e| format!("{}: {e}", relative.display()))?;
        Ok(Self { source, relative })
    }
    fn write(&self, root: &Path) -> Result<(), String> {
        let target = root.join(&self.relative);
        if target.canonicalize().ok().as_ref() == Some(&self.source) {
            return Ok(());
        }
        if let Some(parent) = target.parent() {
            fs::create_dir_all(parent).map_err(|e| e.to_string())?;
        }
        fs::copy(&self.source, target).map_err(|e| e.to_string())?;
        Ok(())
    }
}

impl Location {
    pub fn from_compilation(workspace: tempfile::TempDir) -> Result<Self, String> {
        let mut location = Self::read(workspace.path())?;
        location.media.workspace = Some(workspace);
        Ok(location)
    }

    pub fn read(dirpath: impl AsRef<Path>) -> Result<Self, String> {
        let directory = dirpath.as_ref();
        Self::read_inner(directory).map_err(|e| format!("location {}: {e}", directory.display()))
    }

    fn read_inner(directory: &Path) -> Result<Self, String> {
        let assets = city_assets::CityAsset::new(directory.to_path_buf());
        let source = assets.get_city().map_err(|e| format!("city.txt: {e}"))?;
        let streets =
            street_surface::binary::load_geometry(directory, source.road_lines.len())
                .map_err(|e| format!("streets.bin: {e}"))?;
        let manifest = assets
            .get_manifest()
            .map_err(|e| format!("manifest.json: {e}"))?;
        let pavement_height_meters = match manifest.get("pavement_height_metres") {
            Some(v) => v
                .as_f64()
                .filter(|h| (0.08..=0.25).contains(h))
                .ok_or("manifest.json: pavement_height_metres must be between 0.08 and 0.25")?
                as f32,
            None => street_surface::DEFAULT_PAVEMENT_HEIGHT,
        };
        let (ground, trees) = assets.get_landscape_data()?;
        let mut media = Media::default();
        if directory.join("scene.pack").exists() {
            media.scene = Some(Payload::read(directory, "scene.pack".into())?);
        }
        // Keep only the catalog, tiles, and attribution used by the renderer.
        // Service requests and download/build metadata are not location data.
        if directory.join("aerial/tiles.txt").is_file() {
            let catalog = aerial::tile_filenames(
                &fs::read_to_string(directory.join("aerial/tiles.txt"))
                    .map_err(|e| e.to_string())?,
            )?;
            let mut files = vec![PathBuf::from("aerial/tiles.txt")];
            for name in catalog {
                files.push(PathBuf::from("aerial").join(name));
            }
            if directory.join("aerial/source.json").exists() {
                files.push("aerial/source.json".into());
            }
            files.sort();
            files.dedup();
            media.aerial = files
                .into_iter()
                .map(|file| Payload::read(directory, file))
                .collect::<Result<_, _>>()?;
        }
        Ok(Self {
            name: manifest
                .get("name")
                .and_then(|n| n.as_str())
                .unwrap_or(assets.name())
                .to_owned(),
            origin: source.origin,
            extent: source.bounding_box,
            road_nodes: source.road_nodes.into_iter().map(|p| p.xz()).collect(),
            roads: source
                .road_lines
                .into_iter()
                .map(|r| Road {
                    ends: r.ends,
                    way_id: r.osm_way_id,
                    width_meters: r.road_width_start_meters,
                    direction: r.direction,
                })
                .collect(),
            buildings: source
                .buildings
                .into_iter()
                .map(|b| Building {
                    id: b.id,
                    outline: b.ground_outline,
                    height_meters: b.height_meters,
                })
                .collect(),
            streets,
            pavement_height_meters,
            elevation: assets
                .get_heightfield()
                .map_err(|e| format!("elevation/heightfield.txt: {e}"))?,
            ground,
            trees,
            furniture: assets
                .get_furniture()
                .map_err(|e| format!("furniture.txt: {e}"))?
                .into_iter()
                .map(|f| Furniture {
                    id: f._source_id,
                    kind: f.kind,
                    position: f.coords.xz(),
                    travel: f.travel,
                    value: f.value,
                    phase: f.phase,
                })
                .collect(),
            barriers: match fs::read_to_string(directory.join("barriers.txt")) {
                Ok(text) => asset_codecs::barriers::parse(&text)
                    .map_err(|e| format!("barriers.txt: {e}"))?,
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => Vec::new(),
                Err(e) => return Err(format!("barriers.txt: {e}")),
            },
            road_rises: match fs::read_to_string(directory.join("road-rises.txt")) {
                Ok(text) => asset_codecs::road_rises::parse(&text)
                    .map_err(|e| format!("road-rises.txt: {e}"))?,
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => Vec::new(),
                Err(e) => return Err(format!("road-rises.txt: {e}")),
            },
            labels: read_labels(directory)?,
            media,
        })
    }

    pub fn write(&self, dirpath: impl AsRef<Path>) -> Result<(), String> {
        let directory = dirpath.as_ref();
        self.write_inner(directory)
            .map_err(|e| format!("location {}: {e}", directory.display()))
    }

    fn write_inner(&self, directory: &Path) -> Result<(), String> {
        fs::create_dir_all(directory.join("elevation")).map_err(|e| e.to_string())?;
        let write = || -> error::Result<()> {
            let mut city = BufWriter::new(fs::File::create(directory.join("city.txt"))?);
            writeln!(city, "CITY 2")?;
            if let Some(p) = self.origin {
                writeln!(city, "origin {} {}", p.lat, p.lon)?;
            }
            writeln!(
                city,
                "bounds {} {} {} {}",
                self.extent.min.x, self.extent.min.z, self.extent.max.x, self.extent.max.z
            )?;
            for p in &self.road_nodes {
                writeln!(city, "n {} {}", p.x, p.z)?;
            }
            for r in &self.roads {
                writeln!(
                    city,
                    "r {} {} {} {} {}",
                    r.ends[0],
                    r.ends[1],
                    r.width_meters,
                    r.direction,
                    r.way_id
                        .map(|id| id.to_string())
                        .unwrap_or_else(|| "-".into())
                )?;
            }
            for b in &self.buildings {
                write!(city, "b {} {} 0 {}", b.id, b.height_meters, b.outline.len())?;
                for p in &b.outline {
                    write!(city, " {} {}", p.x, p.z)?;
                }
                writeln!(city)?;
            }
            city.flush()?;
            fs::write(
                directory.join("manifest.json"),
                serde_json::to_vec_pretty(&serde_json::json!({
                    "name": self.name, "pavement_height_metres": self.pavement_height_meters,
                }))?,
            )?;
            fs::write(
                directory.join("labels.json"),
                serde_json::to_vec(&self.labels)?,
            )?;
            // Serialize at full f32 precision; compiler writers deliberately quantize.
            let mut elevation = BufWriter::new(fs::File::create(
                directory.join("elevation/heightfield.txt"),
            )?);
            let f = &self.elevation;
            writeln!(
                elevation,
                "ELEVATION 1\ngrid {} {} {} {} {} 0",
                f.columns, f.rows, f.origin.x, f.origin.z, f.step
            )?;
            for row in f.values.chunks(f.columns) {
                for v in row {
                    write!(elevation, "{v} ")?;
                }
                writeln!(elevation)?;
            }
            elevation.flush()?;
            let mut landscape = asset_codecs::landscape::Writer::new(
                BufWriter::new(fs::File::create(directory.join("landscape.txt"))?),
                asset_codecs::landscape::Precision::Authored,
            )?;
            for t in &self.ground {
                if t.usage != world::TriangleUsage::Ground {
                    return Err(error::invalid(
                        "ground layer contains street geometry",
                    ));
                }
                landscape.ground(t.material, t.points)?;
            }
            for t in &self.trees {
                landscape.tree(t)?;
            }
            landscape.flush()?;
            let mut furniture = BufWriter::new(fs::File::create(directory.join("furniture.txt"))?);
            writeln!(furniture, "FURNITURE 3")?;
            for f in &self.furniture {
                let kind = match f.kind {
                    FurnitureKind::Signal => "signal",
                    FurnitureKind::Stop => "stop",
                    FurnitureKind::Yield => "yield",
                    FurnitureKind::Speed => "speed_limit",
                };
                writeln!(
                    furniture,
                    "{} {kind} {} {} {} {} {} {}",
                    f.id, f.position.x, f.position.z, f.travel.x, f.travel.z, f.value, f.phase
                )?;
            }
            furniture.flush()?;
            asset_codecs::barriers::write(
                &mut BufWriter::new(fs::File::create(directory.join("barriers.txt"))?),
                &self.barriers,
            )?;
            asset_codecs::road_rises::write(
                &mut BufWriter::new(fs::File::create(directory.join("road-rises.txt"))?),
                &self.road_rises,
            )?;
            street_surface::binary::write_geometry(
                &directory.join("streets.bin"),
                &self.streets,
            )?;
            Ok(())
        };
        write().map_err(|e| e.to_string())?;
        // Imagery trees are merged into the ordinary vegetation layer on read.
        remove_optional(&directory.join("imagery-trees.txt"))?;
        if let Some(scene) = &self.media.scene {
            scene.write(directory)?;
        } else {
            remove_optional(&directory.join("scene.pack"))?;
        }
        // Remove only files referenced by the previous renderer catalog.
        let previous_tiles = fs::read_to_string(directory.join("aerial/tiles.txt"))
            .ok()
            .and_then(|text| aerial::tile_filenames(&text).ok())
            .unwrap_or_default();
        let previous_files = previous_tiles
            .into_iter()
            .map(|name| PathBuf::from("aerial").join(name))
            .chain([
                PathBuf::from("aerial/tiles.txt"),
                PathBuf::from("aerial/source.json"),
            ]);
        for relative in previous_files {
            if !self.media.aerial.iter().any(|p| p.relative == relative) {
                remove_optional(&directory.join(relative))?;
            }
        }
        for payload in &self.media.aerial {
            payload.write(directory)?;
        }
        Ok(())
    }
}

fn remove_optional(path: &Path) -> Result<(), String> {
    match fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(e) => Err(format!("{}: {e}", path.display())),
    }
}

fn read_labels(directory: &Path) -> Result<Labels, String> {
    match fs::read(directory.join("labels.json")) {
        Ok(bytes) => serde_json::from_slice(&bytes).map_err(|e| format!("labels.json: {e}")),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            source_location::read_legacy_labels(directory)
        }
        Err(e) => Err(format!("labels.json: {e}")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::city_assets;
    use crate::test_support;
    use crate::world;

    fn assert_data_equal(a: &Location, b: &Location) {
        assert_eq!(a.name, b.name);
        assert_eq!(a.origin, b.origin);
        assert_eq!(a.extent, b.extent);
        assert_eq!(a.road_nodes, b.road_nodes);
        assert_eq!(a.roads, b.roads);
        assert_eq!(a.buildings, b.buildings);
        assert_eq!(a.streets, b.streets);
        assert_eq!(a.pavement_height_meters, b.pavement_height_meters);
        assert_eq!(a.elevation, b.elevation);
        assert_eq!(a.ground, b.ground);
        assert_eq!(a.trees, b.trees);
        assert_eq!(a.furniture, b.furniture);
        assert_eq!(a.barriers, b.barriers);
        assert_eq!(a.road_rises, b.road_rises);
        assert_eq!(a.labels, b.labels);
    }

    #[test]
    fn stored_location_round_trip_is_lossless_without_build_inputs() {
        for populated in [false, true] {
            let input = tempfile::tempdir().unwrap();
            let output = tempfile::tempdir().unwrap();
            test_support::stored_location(input.path(), populated);
            fs::write(input.path().join("source.osm"), r#"<osm><way id="1"><tag k="name" v="Road &amp; Square"/><tag k="building" v="yes"/></way></osm>"#).unwrap();
            fs::write(input.path().join("source.track.json"), b"build-only").unwrap();
            let mut location = Location::read(input.path()).unwrap();
            location.name = "Round trip — Δρόμος".into();
            location.pavement_height_meters = 0.17345679;
            location.roads[0].way_id = None;
            location.roads[0].width_meters = 8.123457;
            location.elevation.values[0] = 7.1234565;
            if populated {
                location.road_rises.push(world::RoadRise {
                    points: [
                        XZ { x: 0., z: 0. }.at_height(0.),
                        XZ { x: 1., z: 0. }.at_height(0.1234567),
                        XZ { x: 0., z: 1. }.at_height(0.),
                    ],
                });
                location.barriers.push(world::Barrier {
                    source_id: 17,
                    kind: world::BarrierKind::ChainLink,
                    ends: [XZ { x: -5.1234565, z: 10. }, XZ { x: 5., z: 10. }],
                    height_meters: 1.8765432,
                    width_meters: 0.1234567,
                });
                location.buildings[0].height_meters = 5.1234565;
                location.furniture[0].position.x = 5.1234565;
                location.trees[0].coords.x = -5.1234565;
            }
            location.write(output.path()).unwrap();
            let reread = Location::read(output.path()).unwrap();
            assert_data_equal(&location, &reread);
            assert!(!output.path().join("source.osm").exists());
            assert!(!output.path().join("source.track.json").exists());
            assert!(!output.path().join("imagery-trees.txt").exists());
            let manifest: serde_json::Value =
                serde_json::from_slice(&fs::read(output.path().join("manifest.json")).unwrap())
                    .unwrap();
            assert_eq!(manifest.as_object().unwrap().len(), 2);
            let world = world::Location::load(&city_assets::CityAsset::new(
                output.path().to_path_buf(),
            ))
            .unwrap();
            assert!(world.has_compiled_roads());
            assert!(reread.labels.streets.contains(&(1, "Road & Square".into())));
            assert!(reread
                .labels
                .buildings
                .contains(&(1, "Road & Square".into())));
            assert_eq!(world.road_lines()[0].path.len(), 3);
            assert!(world.road_lines()[0].length > 20.0);
            assert!(world.road_lines().iter().all(|r| r.path[0].y != 0.0));
            // Writing in place neither duplicates the optional trees nor rounds data.
            reread.write(output.path()).unwrap();
            assert_data_equal(&location, &Location::read(output.path()).unwrap());
        }
    }

    #[test]
    fn render_payloads_round_trip_and_overwrite_removes_stale_optional_layers() {
        let input = tempfile::tempdir().unwrap();
        let output = tempfile::tempdir().unwrap();
        test_support::stored_location(input.path(), true);
        fs::write(input.path().join("scene.pack"), b"opaque compiled scene").unwrap();
        fs::create_dir(input.path().join("aerial")).unwrap();
        fs::write(
            input.path().join("aerial/tiles.txt"),
            "AERIAL 1\nsize 64 1\ntile -10 -10 30 30 tile.rgb\n",
        )
        .unwrap();
        fs::write(input.path().join("aerial/tile.rgb"), b"opaque tile").unwrap();
        fs::write(
            input.path().join("aerial/source.json"),
            br#"{"provider":"Cyprus"}"#,
        )
        .unwrap();
        fs::write(input.path().join("aerial/service.json"), b"build-only").unwrap();
        let mut location = Location::read(input.path()).unwrap();
        location.write(output.path()).unwrap();
        for file in [
            "scene.pack",
            "aerial/tiles.txt",
            "aerial/tile.rgb",
            "aerial/source.json",
        ] {
            assert_eq!(
                fs::read(input.path().join(file)).unwrap(),
                fs::read(output.path().join(file)).unwrap()
            );
        }
        assert!(!output.path().join("aerial/service.json").exists());
        assert_eq!(Location::read(output.path()).unwrap().media.aerial.len(), 3);
        location.write(input.path()).unwrap();
        location.media = Media::default();
        fs::write(output.path().join("imagery-trees.txt"), "stale").unwrap();
        location.write(output.path()).unwrap();
        let reread = Location::read(output.path()).unwrap();
        assert!(reread.media.scene.is_none());
        assert!(reread.media.aerial.is_empty());
        assert!(!output.path().join("aerial/tile.rgb").exists());
        assert_data_equal(&location, &reread);
    }
}

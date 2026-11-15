use crate::aerial;
use crate::codec_furniture;
use crate::codec_landscape;
use crate::elevation;
use crate::geometry::XZ;
use crate::landscape;
use crate::source_location::SourceLocation;
use crate::track;
use crate::traffic;
use crate::world;
use crate::world::HeightField;
use std::path::{Path, PathBuf};

pub const LOCATIONS_DIRECTORY: &str = "data/locations";

#[derive(Clone, Debug)]
pub struct CityAsset {
    directory: PathBuf,
    name: String,
    aerial_attribution: Option<&'static str>,
}

impl CityAsset {
    pub fn new(directory: PathBuf) -> Self {
        let name = Self::load_name(&directory);
        let aerial_attribution = aerial::attribution_label(&directory);
        Self {
            directory,
            name,
            aerial_attribution,
        }
    }

    pub fn resolved(&self) -> std::io::Result<Self> {
        Ok(Self::new(self.directory.canonicalize()?))
    }
    pub fn aerial_attribution(&self) -> Option<&'static str> {
        self.aerial_attribution
    }

    pub fn directory(&self) -> &Path {
        &self.directory
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    fn load_name(directory: &Path) -> String {
        directory
            .file_name()
            .unwrap_or_default()
            .to_string_lossy()
            .into_owned()
    }

    pub fn get_manifest(&self) -> Result<serde_json::Value, String> {
        serde_json::from_str(&self.read_text("manifest.json")?)
            .map_err(|e| format!("invalid location manifest: {e}"))
    }

    /// Load the road graph and building footprints; other getters supply its layers.
    pub fn get_city(&self) -> Result<SourceLocation<f32>, String> {
        SourceLocation::<f32>::parse(&self.read_text("city.txt")?)
    }

    // Returns the city's furniture - signs and traffic lights.
    pub fn get_furniture(&self) -> Result<Vec<traffic::Furniture>, String> {
        codec_furniture::parse(&self.read_text("furniture.txt")?)
    }

    pub fn get_heightfield(&self) -> Result<elevation::HeightField, String> {
        elevation::HeightField::load(&self.read_text("elevation/heightfield.txt")?)
    }

    pub fn get_landscape_data(&self) -> Result<(Vec<world::Triangle>, Vec<world::Tree>), String> {
        use landscape::Triangle;
        let mut ground_triangles = Vec::new();
        let mut trees = Vec::new();
        for (file, imagery) in [("landscape.txt", false), ("imagery-trees.txt", true)] {
            let input = match std::fs::File::open(self.directory.join(file)) {
                Ok(input) => std::io::BufReader::new(input),
                Err(e) if imagery && e.kind() == std::io::ErrorKind::NotFound => continue,
                Err(e) => return Err(format!("cannot read city asset {file}: {e}")),
            };
            let mut file_ground = Vec::new();
            let mut file_trees = Vec::new();
            codec_landscape::read::<f32>(input, |record| {
                match record {
                    codec_landscape::Record::Ground { material, points } => {
                        file_ground.push(Triangle {
                            material,
                            usage: world::TriangleUsage::Ground,
                            points,
                        })
                    }
                    codec_landscape::Record::Tree(tree) => file_trees.push(tree),
                }
                Ok(())
            })?;
            if imagery {
                if !file_ground.is_empty() {
                    return Err("imagery supplement must contain only trees".into());
                }
                landscape::append_imagery_trees(&mut trees, file_trees)?;
            } else {
                ground_triangles = file_ground;
                trees = file_trees;
            }
        }
        Ok((ground_triangles, trees))
    }

    fn read_text(&self, name: &str) -> Result<String, String> {
        std::fs::read_to_string(self.directory.join(name))
            .map_err(|e| format!("cannot read city asset {name}: {e}"))
    }
}

#[derive(Clone, Debug)]
pub struct MenuLocation {
    pub name: String,
    pub track: track::LocationId,
    pub assets: Option<CityAsset>,
}
/// Discover canonical identities and small metadata without decoding world geometry.
fn discover(root: &Path) -> std::io::Result<Vec<MenuLocation>> {
    let mut locations = Vec::new();
    if !root.exists() {
        return Ok(locations);
    }
    let mut paths = std::fs::read_dir(root)?
        .map(|e| e.map(|e| e.path()))
        .collect::<std::io::Result<Vec<_>>>()?;
    paths.sort();
    for path in paths {
        if !path.is_dir() || !path.join("city.txt").is_file() {
            continue;
        }
        let name = path.file_name();
        let Some(name) = name.and_then(|s| s.to_str()) else {
            continue;
        };
        if name.starts_with('.') {
            continue;
        }
        locations.push(MenuLocation::from_assets(CityAsset::new(path))?);
    }
    Ok(locations)
}
pub fn menu_locations() -> Vec<MenuLocation> {
    match discover(Path::new(LOCATIONS_DIRECTORY)) {
        Ok(locations) => locations,
        Err(error) => {
            log::warn!("Cannot list locations: {error}");
            Vec::new()
        }
    }
}

impl HeightField {
    fn load(text: &str) -> Result<Self, String> {
        let mut lines = text.lines();
        let header: Vec<_> = lines
            .next()
            .ok_or("missing elevation header")?
            .split_whitespace()
            .collect();
        if header.len() < 2 || header[0] != "ELEVATION" || header[1] != "1" {
            return Err("unsupported elevation header".into());
        }
        let fields: Vec<_> = lines
            .next()
            .ok_or("missing grid")?
            .split_whitespace()
            .collect();
        if fields.len() != 7 || fields[0] != "grid" {
            return Err("invalid elevation grid".into());
        }
        let number = |i: usize| -> Result<f32, String> {
            let v: f32 = fields[i].parse().map_err(|_| "invalid grid number")?;
            if v.is_finite() {
                Ok(v)
            } else {
                Err("nonfinite grid number".into())
            }
        };
        let columns = fields[1].parse::<usize>().map_err(|_| "invalid columns")?;
        let rows = fields[2].parse::<usize>().map_err(|_| "invalid rows")?;
        let origin = XZ {
            x: number(3)?,
            z: number(4)?,
        };
        let step = number(5)?;
        number(6)?;
        if !(2..=4096).contains(&columns)
            || !(2..=4096).contains(&rows)
            || !(1.0..=100.0).contains(&step)
        {
            return Err("invalid grid dimensions".into());
        }
        let values = lines
            .flat_map(str::split_whitespace)
            .map(|s| {
                let v: f32 = s.parse().map_err(|_| "invalid elevation value")?;
                if v.is_finite() && v.abs() < 10000.0 {
                    Ok(v)
                } else {
                    Err("invalid elevation range")
                }
            })
            .collect::<Result<Vec<_>, _>>()?;
        if values.len() != columns * rows {
            return Err("incomplete elevation grid".into());
        }
        Ok(Self {
            columns,
            rows,
            origin: (origin),
            step,
            values,
        })
    }
}

impl MenuLocation {
    pub fn from_assets(assets: CityAsset) -> std::io::Result<Self> {
        let assets = assets.resolved()?;
        Ok(Self {
            name: assets.name().into(),
            track: world::LocationId::Asset(assets.directory().to_path_buf()),
            assets: Some(assets),
        })
    }
}

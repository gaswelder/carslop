use crate::model;
use crate::model::source_location::SourceLocation;
use crate::storage;
use crate::util;
use crate::util::error::invalid;
use crate::util::error::Result;
use crate::util::geometry::BoundingBoxXZ;
use crate::util::geometry::XZ;
use std::{fs, path::PathBuf};

/// Tree centre read from a `t` record in landscape.txt for placement checks.
pub struct LandscapeItem {
    pub id: u64,
    pub coords: XZ,
}

pub fn read_landscape_file(path: PathBuf) -> Result<Vec<LandscapeItem>> {
    let mut items = Vec::new();
    storage::codec_landscape::read::<f64>(
        std::io::BufReader::new(fs::File::open(path)?),
        |record| {
            if let storage::codec_landscape::Record::Tree(tree) = record {
                items.push(LandscapeItem {
                    id: tree.id,
                    coords: tree.coords,
                });
            }
            Ok(())
        },
    )
    .map_err(invalid)?;
    Ok(items)
}

pub struct WriteCityResult {
    pub city: SourceLocation<f32>,
}

pub fn write_city_txt(
    location_map: &SourceLocation<f64>,
    path_city_txt: &PathBuf,
) -> Result<WriteCityResult> {
    use std::io::{BufWriter, Write};
    let mut file = BufWriter::new(fs::File::create(path_city_txt)?);
    storage::codec_city::write(
        &mut file,
        location_map,
        storage::codec_city::Precision::Imported,
    )?;
    file.flush()?;
    let mut city =
        model::source_location::parse(&fs::read_to_string(path_city_txt)?).map_err(invalid)?;
    city.origin = location_map.origin;
    city.bounding_box = BoundingBoxXZ {
        min: util::geometry::xz_map(location_map.bounding_box.min, |v| v as f32),
        max: util::geometry::xz_map(location_map.bounding_box.max, |v| v as f32),
    };
    Ok(WriteCityResult { city })
}

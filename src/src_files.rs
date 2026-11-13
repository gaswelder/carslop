use crate::asset_codecs;
use crate::geometry::XZ;
use crate::{
    error::{invalid, Result},
    source_location::SourceLocation,
};
use std::{fs, path::PathBuf};

/// Tree centre read from a `t` record in landscape.txt for placement checks.
pub struct LandscapeItem {
    pub coords: XZ,
}

pub fn read_landscape_file(path: PathBuf) -> Result<Vec<LandscapeItem>> {
    let mut items = Vec::new();
    asset_codecs::landscape::read::<f64>(
        std::io::BufReader::new(fs::File::open(path)?),
        |record| {
            if let asset_codecs::landscape::Record::Tree(tree) = record {
                items.push(LandscapeItem {
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
    asset_codecs::city::write(
        &mut file,
        location_map,
        asset_codecs::city::Precision::Imported,
    )?;
    file.flush()?;
    let city =
        SourceLocation::<f32>::parse(&fs::read_to_string(path_city_txt)?).map_err(invalid)?;
    Ok(WriteCityResult { city })
}

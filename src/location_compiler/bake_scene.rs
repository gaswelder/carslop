//! Compile render geometry from prepared location assets.
use crate::scene_archive::{write_spooled, Archive};
use std::path::Path;

pub(crate) fn bake_scene(directory: &Path) -> std::io::Result<()> {
    let assets = crate::city_assets::CityAsset::new(directory.to_owned());
    let city = crate::city::City::load(&assets)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
    let path = directory.join("scene.pack");
    let started = std::time::Instant::now();
    println!(
        "Generating city geometry with {} trees...",
        city.landscape.trees.len()
    );
    let mut layers = Vec::new();
    let decorations = directory.join("decorations.pack");
    let mut decorations = if decorations.is_file() {
        Some(Archive::open(&decorations)?)
    } else {
        None
    };
    for layer in 0..3 {
        println!("Generating layer {layer} with bounded geometry buffers...");
        let mut spool = crate::scene::city_layer_spooled(&city, layer, directory)?;
        if let Some(archive) = &mut decorations {
            for entry in 0..archive.entries.len() {
                let mut builder = crate::chunks::ChunkBuilder::default();
                for tri in archive.read_layer(entry, layer)?.as_chunks::<3>().0 {
                    builder.push(*tri);
                }
                for chunk in builder.finish() {
                    spool.append(chunk)?;
                }
            }
        }
        layers.push(spool);
    }
    println!("Writing city render archive...");
    write_spooled(&path, &mut layers)?;
    let archive = Archive::open(&path)?;
    let counts: [usize; 3] =
        std::array::from_fn(|layer| archive.entries.iter().map(|e| e.layers[layer].count).sum());
    println!("Baked {} chunks: {} ground, {} detailed building, {} distant building vertices; {:.1} MiB; {:.2}s",archive.entries.len(),counts[0],counts[1],counts[2],std::fs::metadata(&path)?.len() as f32/1048576.0,started.elapsed().as_secs_f32());
    drop(archive);
    Ok(())
}

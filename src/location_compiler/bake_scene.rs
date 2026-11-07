//! Compile render geometry from prepared location assets.
use crate::scene_archive::{write_spooled, Archive};
use std::path::Path;

pub(crate) fn bake_scene(
    city: &crate::city::City,
    manifest_path: &Path,
    decorations_path: &Path,
    output_path: &Path,
    temporary_directory: &Path,
) -> std::io::Result<()> {
    let manifest = serde_json::from_slice(&std::fs::read(manifest_path)?)?;
    let boundary = super::boundary::Boundary::from_manifest(&manifest)
        .map_err(|e| std::io::Error::other(e.to_string()))?;
    let boundary = if let Some(boundary) = boundary {
        let origin = city
            .geographic_origin()
            .ok_or_else(|| std::io::Error::other("boundary needs a geographic origin"))?;
        Some(boundary.local(origin))
    } else {
        None
    };
    let started = std::time::Instant::now();
    println!(
        "Generating city geometry with {} trees...",
        city.landscape.trees.len()
    );
    let mut layers = Vec::new();
    let mut decorations = if decorations_path.is_file() {
        Some(Archive::open(decorations_path)?)
    } else {
        None
    };
    for layer in 0..3 {
        println!("Generating layer {layer} with bounded geometry buffers...");
        let mut spool =
            crate::scene::city_layer_spooled(city, layer, temporary_directory, boundary.as_ref())?;
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
    write_spooled(output_path, &mut layers)?;
    let archive = Archive::open(output_path)?;
    let counts: [usize; 3] =
        std::array::from_fn(|layer| archive.entries.iter().map(|e| e.layers[layer].count).sum());
    println!("Baked {} chunks: {} ground, {} detailed building, {} distant building vertices; {:.1} MiB; {:.2}s",archive.entries.len(),counts[0],counts[1],counts[2],std::fs::metadata(output_path)?.len() as f32/1048576.0,started.elapsed().as_secs_f32());
    drop(archive);
    Ok(())
}

//! Compile render geometry from prepared location assets.
use crate::chunks;
use crate::scene;
use crate::world;
use crate::{
    osm::OSMBoundary,
    scene_archive::{write_spooled, Archive},
};
use std::path::Path;

pub fn build_scene(
    city: &world::CompiledLocation,
    boundary: Option<&OSMBoundary>,
    decorations_path: &Path,
    output_path: &Path,
    temporary_directory: &Path,
) -> std::io::Result<()> {
    //
    // convert boundary to polygon
    //
    let boundary_polygon = if let Some(boundary) = boundary {
        let origin = city
            .origin
            .ok_or_else(|| std::io::Error::other("boundary needs a geographic origin"))?;
        Some(boundary.local(origin))
    } else {
        None
    };

    println!("Generating city geometry");

    //
    //
    //
    let mut layers = Vec::new();

    //
    // load decorations
    //
    let mut decorations = if decorations_path.is_file() {
        Some(Archive::open(decorations_path)?)
    } else {
        None
    };

    for layer in 0..3 {
        println!("Generating layer {layer} with bounded geometry buffers...");

        //
        // create spool?
        //
        let mut spool =
            scene::city_layer_spooled(city, layer, temporary_directory, boundary_polygon.as_ref())?;

        //
        //
        //
        if let Some(archive) = &mut decorations {
            //
            // for each decoration from the archive
            //
            for entry in 0..archive.entries.len() {
                //
                // make a builder
                //
                let mut builder = chunks::ChunkBuilder::default();

                //
                // push triangles into builder
                //
                for tri in archive.read_layer(entry, layer)?.as_chunks::<3>().0 {
                    builder.push(*tri);
                }

                //
                // ?
                //
                for chunk in builder.finish() {
                    spool.append(chunk)?;
                }
            }
        }
        layers.push(spool);
    }
    println!("writing location");
    write_spooled(output_path, &mut layers)?;
    Ok(())
}

//! Compile render geometry from prepared location assets.

use crate::build_osm;
use crate::render_chunks;
use crate::render_scene;
use crate::render_scene_archive;
use crate::world_preparation;
use crate::{build_osm::OSMArea, render_scene_archive::write_spooled};
use std::path::Path;

pub fn build_scene(
    city: &world_preparation::CompiledLocation,
    boundary: Option<&OSMArea>,
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
        Some(build_osm::local(boundary, origin))
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
        Some(render_scene_archive::open(decorations_path)?)
    } else {
        None
    };

    for layer in 0..3 {
        println!("Generating layer {layer} with bounded geometry buffers...");

        //
        // create spool?
        //
        let mut spool = render_scene::city_layer_spooled(
            city,
            layer,
            temporary_directory,
            boundary_polygon.as_ref(),
        )?;

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
                let mut builder = render_chunks::ChunkBuilder::default();

                //
                // push triangles into builder
                //
                for tri in render_scene_archive::read_layer(archive, entry, layer)?
                    .as_chunks::<3>()
                    .0
                {
                    render_chunks::push(&mut builder, *tri);
                }

                //
                // ?
                //
                for chunk in render_chunks::finish(builder) {
                    render_scene_archive::append(&mut spool, chunk)?;
                }
            }
        }
        layers.push(spool);
    }
    println!("writing location");
    write_spooled(output_path, &mut layers)?;
    Ok(())
}

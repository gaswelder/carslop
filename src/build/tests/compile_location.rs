use super::compile_location;
use crate::build;
use crate::util::geometry;
use flate2::write::GzEncoder;
use flate2::Compression;
use std::fs;
use std::io::Write;

#[test]
fn compilation_resumes_from_location_workspace_and_repairs_missing_scene() {
    let root = tempfile::tempdir().unwrap();
    let downloads = root.path();
    let area = build::osm::ImportArea {
        latitude: 35.162,
        longitude: 33.32,
        width_km: 0.1,
        height_km: 0.1,
    };
    let boundary = build::osm::geometry(&area).unwrap();
    let blocks =
        build::imagery::blocks(geometry::bounding_box_to_array(build::osm::bbox(&boundary)));
    let images = downloads.join("satellite_images");
    fs::create_dir(&images).unwrap();
    let side = build::imagery::CORE + 2 * build::imagery::PAD;
    let image = image::RgbImage::from_pixel(side, side, image::Rgb([110, 90, 70]));
    for block in &blocks {
        image
            .save(build::imagery::image_path(&images, block))
            .unwrap();
    }
    let manifest = build::download_location::DownloadedSources {
        name: build::osm::name(&area),
        area,
        height_tiles: vec!["N35E033".into()],
        imagery_blocks: blocks,
    };
    fs::write(
        downloads.join("manifest.json"),
        serde_json::to_vec(&manifest).unwrap(),
    )
    .unwrap();
    fs::write(
        downloads.join("source.osm"),
        r#"<osm version="0.6">
        <bounds minlat="35.1615" minlon="33.3194" maxlat="35.1625" maxlon="33.3206"/>
        <node id="1" lat="35.162" lon="33.3197"/>
        <node id="2" lat="35.162" lon="33.3203"/>
        <way id="3"><nd ref="1"/><nd ref="2"/><tag k="highway" v="residential"/></way>
        </osm>"#,
    )
    .unwrap();
    fs::create_dir(downloads.join("elevation_data")).unwrap();
    let mut elevation = GzEncoder::new(
        fs::File::create(downloads.join("elevation_data/N35E033.hgt.gz")).unwrap(),
        Compression::fast(),
    );
    elevation.write_all(&vec![0; 1201 * 1201 * 2]).unwrap();
    elevation.finish().unwrap();
    build::infer_trees::add_trees(downloads).unwrap();
    let location = compile_location(downloads).unwrap();
    assert!(!location.roads.is_empty());
    drop(location);
    let scene = downloads.join("tmp/compiled/scene.pack");
    let receipts: Vec<_> = ["trees.json", "assets.json", "imagery.json", "scene.json"]
        .iter()
        .map(|name| downloads.join("tmp").join(name))
        .collect();
    let timestamps: Vec<_> = receipts
        .iter()
        .map(|path| fs::metadata(path).unwrap().modified().unwrap())
        .collect();
    let scene_time = fs::metadata(&scene).unwrap().modified().unwrap();
    build::infer_trees::add_trees(downloads).unwrap();
    compile_location(downloads).unwrap();
    for (receipt, timestamp) in receipts.iter().zip(&timestamps) {
        assert_eq!(
            fs::metadata(receipt).unwrap().modified().unwrap(),
            *timestamp
        );
    }
    assert_eq!(
        fs::metadata(&scene).unwrap().modified().unwrap(),
        scene_time
    );
    fs::remove_file(&scene).unwrap();
    compile_location(downloads).unwrap();
    assert!(scene.is_file());
    for (receipt, timestamp) in receipts.iter().zip(&timestamps).take(3) {
        assert_eq!(
            fs::metadata(receipt).unwrap().modified().unwrap(),
            *timestamp
        );
    }
}

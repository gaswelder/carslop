use super::*;

#[test]
fn equator_and_prime_meridian_use_global_xyz_indices() {
    let grid = blocks([-0.001, -0.001, 0.001, 0.001]);
    assert_eq!(grid.len(), 4);
    assert_eq!(block_name(&grid[0]), "15_16383_16383");
    assert_eq!(block_name(&grid[3]), "15_16384_16384");
    assert_eq!(tile_name(&grid[0], 3, 3), "17_65535_65535.jpg");
    assert_eq!(tile_name(&grid[3], 0, 0), "17_65536_65536.jpg");
    let pad = PAD as f64 * RES;
    assert!((grid[3].extent[0] + pad).abs() < 1e-8);
    assert!((grid[3].extent[3] - pad).abs() < 1e-8);
}

#[test]
fn overlapping_crops_share_block_names_and_extents() {
    let first = blocks([35.16, 33.31, 35.161, 33.311]);
    let second = blocks([35.159, 33.309, 35.162, 33.312]);
    for block in first {
        let matching = second
            .iter()
            .find(|b| b.x == block.x && b.y == block.y)
            .unwrap();
        assert_eq!(block.extent, matching.extent);
        assert_eq!(
            image_path(Path::new("images"), &block),
            image_path(Path::new("images"), matching)
        );
        let restored: Block = serde_json::from_slice(&serde_json::to_vec(&block).unwrap()).unwrap();
        assert_eq!(block_name(&restored), block_name(&block));
    }
}

#[test]
fn tile_cache_tracks_source_image_and_geographic_origin() {
    let root = tempfile::tempdir().unwrap();
    let work = root.path().join("work");
    let aerial = root.path().join("aerial");
    fs::create_dir(&aerial).unwrap();
    let block = blocks([35.16, 33.31, 35.161, 33.311]).remove(0);
    let source = image_path(root.path(), &block);
    let side = CORE + 2 * PAD;
    image::RgbImage::from_pixel(side, side, image::Rgb([110, 90, 70]))
        .save(&source)
        .unwrap();
    let first = process_tiles(root.path(), &work, &aerial, &block, [35.162, 33.32]).unwrap();
    let metadata = work.join(format!("block-{}.json", block_name(&block)));
    let timestamp = fs::metadata(&metadata).unwrap().modified().unwrap();
    assert_eq!(
        first,
        process_tiles(root.path(), &work, &aerial, &block, [35.162, 33.32]).unwrap()
    );
    assert_eq!(
        fs::metadata(&metadata).unwrap().modified().unwrap(),
        timestamp
    );
    let moved = process_tiles(root.path(), &work, &aerial, &block, [35.163, 33.32]).unwrap();
    assert_ne!(moved.tiles[0].bounds, first.tiles[0].bounds);
    image::RgbImage::from_pixel(side, side, image::Rgb([255, 255, 255]))
        .save(&source)
        .unwrap();
    let blank = process_tiles(root.path(), &work, &aerial, &block, [35.163, 33.32]).unwrap();
    assert!(blank.tiles.is_empty());
}

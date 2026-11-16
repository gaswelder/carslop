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

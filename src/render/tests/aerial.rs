use super::*;
use crate::render;
use crate::storage;
use crate::test;
#[test]
fn two_imagery_workers_and_attributions_stay_with_their_asset_directories() {
    let root = tempfile::tempdir().unwrap();
    let first = test::support::generated_location(&root.path().join("first"), 7.0, "Cyprus DLS");
    let second =
        test::support::generated_location(&root.path().join("second"), 13.0, "arcgisonline.com");
    let mut workers = Vec::new();
    for (assets, expected) in [(&first, 7), (&second, 13)] {
        let directory = storage::city_assets::directory(assets).join("aerial");
        let catalog = storage::aerial::parse(
            &std::fs::read_to_string(storage::city_assets::directory(assets).join("aerial.txt"))
                .unwrap(),
        )
        .unwrap();
        let loader = render::aerial::new(directory, catalog);
        render::aerial::request(&loader, std::iter::once(0));
        workers.push((loader, expected));
    }
    for (loader, expected) in workers {
        let (id, pixels) = loader
            .ready
            .recv_timeout(std::time::Duration::from_secs(3))
            .unwrap();
        assert_eq!(id, 0);
        let pixels = pixels.unwrap();
        assert_eq!(pixels.len(), 66 * 66 * 3);
        assert!(pixels.iter().all(|&value| value == expected));
    }
    assert!(storage::city_assets::aerial_attribution(&(first))
        .unwrap()
        .contains("CYPRUS DLS"));
    assert!(storage::city_assets::aerial_attribution(&(second))
        .unwrap()
        .contains("ESRI"));
}

#[test]
fn nearby_selection_is_bounded() {
    let c=storage::aerial::parse("AERIAL 1\nsize 512 8\ntile 0 0 100 100 a.rgb 00000000\ntile 100 0 200 100 b.rgb 00000000\n").unwrap();
    assert_eq!(
        wanted(
            &c,
            XYZ {
                x: 150.0,
                y: 0.0,
                z: 50.0
            },
            1
        ),
        vec![1]
    );
    assert!(capacity(&c) * layer_bytes(c.size + 2 * c.gutter) <= GPU_BUDGET);
    assert!(wanted(
        &c,
        XYZ {
            x: 2000.0,
            y: 0.0,
            z: 0.0
        },
        32
    )
    .is_empty());
    assert!(
        storage::aerial::read_tile(std::path::Path::new("/missing-aerial-test"), &c, 0).is_err()
    );
}
#[test]
fn city_tiles_fit_nearby_coverage_within_the_gpu_budget() {
    let mut text = String::from("AERIAL 1\nsize 512 8\n");
    for z in -6..=6 {
        for x in -6..=6 {
            text.push_str(&format!(
                "tile {} {} {} {} {x}_{z}.rgb 00000000\n",
                x as f32 * 249.,
                z as f32 * 249.,
                (x + 1) as f32 * 249.,
                (z + 1) as f32 * 249.
            ));
        }
    }
    let catalog = storage::aerial::parse(&text).unwrap();
    let slots = capacity(&catalog);
    for x in [0., 62., 124., 248.] {
        for z in [0., 62., 124., 248.] {
            let all = wanted(&catalog, XYZ { x: x, y: 0., z: z }, usize::MAX);
            assert!(
                all.len() <= slots,
                "nearby city coverage requires {} slots, only {slots} available",
                all.len()
            );
        }
    }
    assert!(slots * layer_bytes(catalog.size + 2 * catalog.gutter) <= GPU_BUDGET);
}
#[test]
fn jpeg_worker_decodes_validates_dimensions_and_reports_errors() {
    let dir = tempfile::tempdir().unwrap();
    let mut bytes = Vec::new();
    image::codecs::jpeg::JpegEncoder::new_with_quality(&mut bytes, 80)
        .encode(
            &vec![127; 66 * 66 * 3],
            66,
            66,
            image::ExtendedColorType::Rgb8,
        )
        .unwrap();
    std::fs::write(dir.path().join("a.jpg"), &bytes).unwrap();
    let catalog = storage::aerial::parse(&format!(
        "AERIAL 1\nsize 64 1\ntile 0 0 10 10 a.jpg {:08x}\ntile 10 0 20 10 missing.jpg 00000000\n",
        0
    ))
    .unwrap();
    let loader = render::aerial::new(dir.path().to_path_buf(), catalog.clone());
    render::aerial::request(&loader, [0, 1].into_iter());
    let (id, result) = loader
        .ready
        .recv_timeout(std::time::Duration::from_secs(5))
        .unwrap();
    assert_eq!(id, 0);
    assert!(result.unwrap().iter().all(|v| v.abs_diff(127) <= 2));
    let (id, result) = loader
        .ready
        .recv_timeout(std::time::Duration::from_secs(5))
        .unwrap();
    assert_eq!(id, 1);
    assert!(result.is_err());
    let mut wrong_size = catalog.clone();
    wrong_size.gutter = 2;
    assert!(storage::aerial::read_tile(dir.path(), &wrong_size, 0).is_err());
    std::fs::write(dir.path().join("a.jpg"), &bytes[..30]).unwrap();
    assert!(storage::aerial::read_tile(dir.path(), &catalog, 0).is_err());
}

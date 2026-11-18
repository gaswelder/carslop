use super::{cache_path, copy_download, new};
use std::fs;

#[test]
fn cached_download_preserves_unchanged_files_and_repairs_changed_files() {
    let root = tempfile::tempdir().unwrap();
    let downloader = new(ureq::Agent::new_with_defaults(), root.path());
    let url = "https://example.invalid/fixture";
    let cached = cache_path(&downloader, url);
    let target = root.path().join("source");
    fs::write(&cached, "original").unwrap();
    copy_download(&downloader, url, &target).unwrap();
    let timestamp = fs::metadata(&target).unwrap().modified().unwrap();
    copy_download(&downloader, url, &target).unwrap();
    assert_eq!(
        fs::metadata(&target).unwrap().modified().unwrap(),
        timestamp
    );
    fs::write(&target, "modified").unwrap();
    copy_download(&downloader, url, &target).unwrap();
    assert_eq!(fs::read_to_string(&target).unwrap(), "original");
    fs::remove_file(&target).unwrap();
    copy_download(&downloader, url, &target).unwrap();
    assert_eq!(fs::read_to_string(&target).unwrap(), "original");
}

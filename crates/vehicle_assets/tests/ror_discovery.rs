use std::{fs, fs::File};

use vehicle_assets::ror::{AssetKind, discover_entries};
use zip::{ZipWriter, write::SimpleFileOptions};

#[test]
fn discovery_filters_by_kind_and_sorts_full_archive_names() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("mixed.zip");
    let mut zip = ZipWriter::new(File::create(&path).unwrap());
    let options = SimpleFileOptions::default();
    for name in [
        "variants/z.truck",
        "map/Facility.TERRN2",
        "readme.txt",
        "variants/a.TRUCK",
        "other/a.truck",
        "preview.truck.png",
    ] {
        zip.start_file(name, options).unwrap();
    }
    zip.add_directory("directory.truck/", options).unwrap();
    zip.finish().unwrap();

    assert_eq!(
        discover_entries(&path, AssetKind::Vehicle).unwrap(),
        ["other/a.truck", "variants/a.TRUCK", "variants/z.truck"]
    );
    assert_eq!(
        discover_entries(&path, AssetKind::Terrain).unwrap(),
        ["map/Facility.TERRN2"]
    );
}

#[test]
fn an_archive_without_definitions_has_no_candidates() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("empty.zip");
    ZipWriter::new(File::create(&path).unwrap())
        .finish()
        .unwrap();

    assert!(
        discover_entries(&path, AssetKind::Terrain)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn a_corrupt_archive_error_names_the_file() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("corrupt.zip");
    fs::write(&path, "invalid ZIP contents").unwrap();

    let error = discover_entries(&path, AssetKind::Terrain).unwrap_err();
    assert!(error.to_string().contains(&path.display().to_string()));
    assert!(std::error::Error::source(&error).is_some());
}

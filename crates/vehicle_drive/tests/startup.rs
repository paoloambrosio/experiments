use std::{
    fs,
    path::Path,
    process::{Command, Output, Stdio},
};

use tempfile::TempDir;
use zip::{ZipWriter, write::SimpleFileOptions};

const ASSETS: &str = "\
[assets]
terrain = 'packs/CM sample.zip'
vehicle = 'packs/cars.zip'
";

fn fixture() -> TempDir {
    let dir = tempfile::tempdir().unwrap();
    let packs = dir.path().join("settings/packs");
    fs::create_dir_all(&packs).unwrap();
    write_zip(&packs.join("CM sample.zip"), &["map/CM.terrn2"]);
    write_zip(
        &packs.join("cars.zip"),
        &["variants/tuned.truck", "variants/base.truck"],
    );
    dir
}

fn write_zip(path: &Path, names: &[&str]) {
    let mut zip = ZipWriter::new(fs::File::create(path).unwrap());
    for name in names {
        zip.start_file(*name, SimpleFileOptions::default()).unwrap();
    }
    zip.finish().unwrap();
}

fn drive(dir: &Path, args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_vehicle_drive"))
        .current_dir(dir)
        .args(args)
        .stdin(Stdio::null())
        .output()
        .unwrap()
}

#[test]
fn paths_resolve_from_the_config_parent_and_explicit_entry_selects() {
    let dir = fixture();
    fs::write(
        dir.path().join("settings/drive.toml"),
        format!("{ASSETS}vehicle_entry = 'variants/base.truck'\n"),
    )
    .unwrap();

    let output = drive(dir.path(), &["--config", "settings/drive.toml"]);
    assert!(output.status.success(), "{:?}", output);
    let summary = String::from_utf8(output.stdout).unwrap();
    assert!(summary.contains("CM sample.zip -> \"map/CM.terrn2\""));
    assert!(summary.contains("cars.zip -> \"variants/base.truck\""));
    assert!(output.stderr.is_empty());
}

#[test]
fn default_config_lists_ambiguous_candidates_when_unattended() {
    let dir = fixture();
    let settings = dir.path().join("settings");
    fs::write(settings.join("config.toml"), ASSETS).unwrap();

    let output = drive(&settings, &[]);
    assert!(!output.status.success());
    assert!(output.stdout.is_empty());
    let error = String::from_utf8(output.stderr).unwrap();
    assert!(error.contains("vehicle_entry"));
    assert!(error.contains("[\"variants/base.truck\", \"variants/tuned.truck\"]"));
}

#[test]
fn invalid_selection_reports_the_field_entry_and_archive() {
    let dir = fixture();
    let settings = dir.path().join("settings");
    fs::write(
        settings.join("config.toml"),
        format!("{ASSETS}vehicle_entry = 'base.truck'\n"),
    )
    .unwrap();

    let output = drive(&settings, &[]);
    assert!(!output.status.success());
    let error = String::from_utf8(output.stderr).unwrap();
    assert!(error.contains("[assets].vehicle_entry \"base.truck\""));
    assert!(error.contains("cars.zip"));
    assert!(error.contains("variants/base.truck"));
}

#[test]
fn missing_config_points_to_the_example() {
    let dir = fixture();
    let output = drive(dir.path(), &[]);
    assert!(!output.status.success());
    let error = String::from_utf8(output.stderr).unwrap();
    assert!(error.contains("cannot read config"));
    assert!(error.contains("config.example.toml"));
}

#[test]
fn missing_archive_names_the_config_field_and_path() {
    let dir = fixture();
    let settings = dir.path().join("settings");
    fs::write(
        settings.join("config.toml"),
        ASSETS.replace("CM sample.zip", "missing.zip"),
    )
    .unwrap();

    let output = drive(&settings, &[]);
    assert!(!output.status.success());
    let error = String::from_utf8(output.stderr).unwrap();
    assert!(error.contains("[assets].terrain"));
    assert!(error.contains("missing.zip"));
}

#[test]
fn terrain_view_reaches_import_with_only_terrain_configured() {
    let dir = fixture();
    let settings = dir.path().join("settings");
    fs::write(
        settings.join("config.toml"),
        "[assets]\nterrain = 'packs/CM sample.zip'\n",
    )
    .unwrap();

    // The ZIP has a discoverable definition but no geometry configuration.
    // Import must fail before any window opens.
    let output = drive(&settings, &["--view-terrain"]);
    assert!(!output.status.success());
    let error = String::from_utf8(output.stderr).unwrap();
    assert!(error.contains("map/CM.terrn2"), "{error}");
    assert!(error.contains("missing GeometryConfig"), "{error}");
}

#[test]
fn vehicle_view_requires_a_vehicle_archive() {
    let dir = fixture();
    let settings = dir.path().join("settings");
    fs::write(
        settings.join("config.toml"),
        "[assets]\nterrain = 'packs/CM sample.zip'\n",
    )
    .unwrap();

    let output = drive(&settings, &["--view-vehicle"]);
    assert!(!output.status.success());
    let error = String::from_utf8(output.stderr).unwrap();
    assert!(error.contains("missing [assets].vehicle"), "{error}");
}

#[test]
fn selects_only_one_view_mode_at_a_time() {
    let dir = fixture();
    let output = drive(dir.path(), &["--view-terrain", "--view-vehicle"]);
    assert!(!output.status.success());
    let error = String::from_utf8(output.stderr).unwrap();
    assert!(error.contains("cannot be used with"), "{error}");
}

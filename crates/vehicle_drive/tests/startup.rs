use std::process::{Command, Output, Stdio};

fn command(args: &[&str]) -> Output {
    let cwd = tempfile::tempdir().unwrap();
    Command::new(env!("CARGO_BIN_EXE_vehicle_drive"))
        .current_dir(cwd.path())
        .args(args)
        .stdin(Stdio::null())
        .output()
        .unwrap()
}

#[test]
fn help_lists_the_human_driving_modes_without_requiring_a_config() {
    let output = command(&["--help"]);
    assert!(output.status.success());
    let help = String::from_utf8(output.stdout).unwrap();
    for mode in [
        "--drive",
        "--view-terrain",
        "--view-vehicle",
        "--crash-replay",
    ] {
        assert!(help.contains(mode), "{help}");
    }
    assert!(help.contains("--config"));
    assert!(!help.contains("-report"));
}

#[test]
fn rejects_retired_reports_and_positional_asset_paths() {
    for args in [
        &["--drive-report"][..],
        &["--alignment-report"],
        &["--crash-report"],
        &["--crash-target", "wall"],
        &["arbitrary.zip"],
    ] {
        let output = command(args);
        assert_eq!(output.status.code(), Some(2), "{args:?}: {output:?}");
        assert!(String::from_utf8_lossy(&output.stderr).contains("unexpected argument"));
    }
}

#[test]
fn manual_config_errors_identify_the_file() {
    let output = command(&["--drive", "--config", "missing.toml"]);
    assert_eq!(output.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&output.stderr).contains("cannot read config missing.toml"));
}

#[test]
fn unattended_import_lists_eligible_definitions() {
    let cwd = tempfile::tempdir().unwrap();
    let directory = cwd.path().join("assets");
    std::fs::create_dir(&directory).unwrap();
    for (name, bytes) in [
        (
            "track.zip",
            include_bytes!("../../vehicle_assets/reference/track.zip").as_slice(),
        ),
        (
            "car.zip",
            include_bytes!("../../vehicle_assets/reference/car.zip").as_slice(),
        ),
    ] {
        std::fs::write(directory.join(name), bytes).unwrap();
    }
    std::fs::write(
        directory.join("local.toml"),
        "[assets]\nterrain = 'track.zip'\nvehicle = 'car.zip'\n",
    )
    .unwrap();
    let output = Command::new(env!("CARGO_BIN_EXE_vehicle_drive"))
        .current_dir(cwd.path())
        .args(["--drive", "--config", "assets/local.toml"])
        .stdin(Stdio::null())
        .output()
        .unwrap();
    assert_eq!(output.status.code(), Some(1));
    let error = String::from_utf8_lossy(&output.stderr);
    assert!(
        error.contains("vehicle_entry") && error.contains("car-wide.truck"),
        "{error}"
    );
    assert!(String::from_utf8_lossy(&output.stdout).contains("track.terrn2"));
}

#[test]
fn crash_replay_rejects_manual_config() {
    let output = command(&["--crash-replay", "--config", "unused.toml"]);
    assert_eq!(output.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&output.stderr).contains("cannot be used with"));
}

#[test]
fn selects_only_one_driving_mode_at_a_time() {
    for modes in [
        ["--view-terrain", "--view-vehicle"],
        ["--drive", "--view-terrain"],
        ["--drive", "--view-vehicle"],
        ["--crash-replay", "--drive"],
        ["--crash-replay", "--view-terrain"],
        ["--crash-replay", "--view-vehicle"],
    ] {
        let output = command(&modes);
        assert_eq!(output.status.code(), Some(2));
        assert!(String::from_utf8_lossy(&output.stderr).contains("cannot be used with"));
    }
}

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
    assert!(!help.contains("--config"));
    assert!(!help.contains("-report"));
}

#[test]
fn rejects_retired_reports_and_external_asset_paths() {
    for args in [
        &["--drive-report"][..],
        &["--alignment-report"],
        &["--crash-report"],
        &["--crash-target", "wall"],
        &["--config", "arbitrary.toml"],
        &["arbitrary.zip"],
    ] {
        let output = command(args);
        assert_eq!(output.status.code(), Some(2), "{args:?}: {output:?}");
        assert!(String::from_utf8_lossy(&output.stderr).contains("unexpected argument"));
    }
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

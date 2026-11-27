use std::process::Command;

#[test]
fn reference_reports_run_without_a_local_asset_configuration() {
    let cwd = tempfile::tempdir().unwrap();
    for (flag, checkpoints) in [
        (
            "--drive-report",
            ["Ramp ramp:", "Imported drive checks passed."],
        ),
        (
            "--alignment-report",
            ["Upright alignment", "Authored alignment"],
        ),
        (
            "--crash-report",
            ["First body contact", "reset replay matched"],
        ),
    ] {
        let output = Command::new(env!("CARGO_BIN_EXE_vehicle_lab"))
            .current_dir(cwd.path())
            .arg(flag)
            .output()
            .unwrap();
        let stdout = String::from_utf8_lossy(&output.stdout);
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert!(
            output.status.success(),
            "{flag} failed:\n{stdout}\n{stderr}"
        );
        for checkpoint in checkpoints {
            assert!(
                stdout.contains(checkpoint),
                "{flag} missed {checkpoint}:\n{stdout}"
            );
        }
    }
}

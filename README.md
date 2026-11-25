# Bevy Vehicle Physics

A Rust workspace for soft-body vehicle physics in Bevy and Avian. Replay the 1,000 kg reference car launching, turning, and braking on four live node-and-beam tyres, or study an individual tyre on a moving belt and over a step or kerb.

```sh
cargo run -p vehicle_lab
cargo run -p vehicle_lab -- --handling-lab
cargo run -p vehicle_lab -- --handling-report
cargo run -p vehicle_lab -- --grip-lab
cargo run -p vehicle_lab -- --grip-report
cargo run -p vehicle_lab -- --corner-lab
cargo run -p vehicle_lab -- --corner-report
cargo run -p vehicle_lab -- --puncture-lab
cargo run -p vehicle_lab -- --puncture-report
cargo run -p vehicle_lab -- --impact-lab
cargo run -p vehicle_lab -- --impact-report
cargo run -p vehicle_lab -- --rolling-lab
cargo run -p vehicle_lab -- --rolling-report
cargo run -p vehicle_lab -- --inflation-lab
cargo run -p vehicle_lab -- --drop-report
```

Regular lab builds use compiler optimisations. Add `--no-vsync` to run without the display's refresh cap where supported, or `--benchmark` to time preparation and a real-time replay in the lab window. [Performance controls and measurement](docs/user/README.md#performance)

- `vehicle_physics` advances the tyre with a windowless solver. Its optional Avian integration provides the reference car assembly, suspension, steering, drive torque, and braking.
- `vehicle_lab` runs whole-car handling replays, the loaded suspension corner, wheel drop, moving-belt rig, kerb strike, puncture comparison, and fixed-bead tread press with visual labs and headless reports.
- `vehicle_assets` defines shared terrain and vehicle geometry for importers and viewers. [Imported terrain and vehicle roadmap](docs/drive-roadmap.md)
- `vehicle_drive` selects Rigs of Rods definitions from ZIP archives and opens imported terrain for visual and collision inspection. [Drive setup](docs/user/README.md#select-a-terrain-and-vehicle) · [Terrain viewer](docs/user/README.md#view-an-imported-terrain)

See the [user guide](docs/user/README.md) for setup, controls, and library usage. The [devlog](docs/devlog/README.md) records the development steps and the vehicle physics roadmap.

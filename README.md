# Bevy Vehicle Physics

A Rust workspace for soft-body vehicle physics in Bevy and Avian. Drop a reinforced node-and-beam tyre, measure loaded tyre grip on a moving belt, watch a sidewall tear let air escape, or tow a loaded suspension corner over a step.

```sh
cargo run -p vehicle_lab
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

- `vehicle_physics` generates and advances the tyre with a windowless solver. Its optional Avian integration couples the tyre to a dynamic rim and provides a suspension corner for a rigid chassis.
- `vehicle_lab` runs the loaded suspension corner, wheel drop, moving-belt rig, kerb strike, puncture comparison, and fixed-bead tread press with interactive views and headless reports.

See the [user guide](docs/user/README.md) for setup, controls, and library usage. The [devlog](docs/devlog/README.md) records the development steps and the tyre roadmap.

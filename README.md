# Bevy Vehicle Physics

A Rust workspace for soft-body vehicle physics in Bevy and Avian. Drop a reinforced node-and-beam tyre to compare rebound and the contact patch across four gauge pressures, or run it on a moving belt to measure grip under drive, braking, and lateral loads.

```sh
cargo run -p vehicle_lab
cargo run -p vehicle_lab -- --rolling-lab
cargo run -p vehicle_lab -- --rolling-report
cargo run -p vehicle_lab -- --inflation-lab
cargo run -p vehicle_lab -- --drop-report
```

- `vehicle_physics` generates and advances the tyre with a windowless solver. Its optional Avian integration couples the moving bead rings to a dynamic rim.
- `vehicle_lab` runs the wheel drop, moving-belt rig, and fixed-bead tread press with interactive views and headless reports.

See the [user guide](docs/user/README.md) for setup, controls, and library usage. The [devlog](docs/devlog/README.md) records the development steps and the tyre roadmap.

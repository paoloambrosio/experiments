# Bevy Vehicle Physics

A Rust workspace for soft-body vehicle physics in Bevy and Avian. Drop a reinforced node-and-beam tyre to compare rebound across four gauge pressures, measure grip on a moving belt, or strike a kerb and inspect the damage after unloading.

```sh
cargo run -p vehicle_lab
cargo run -p vehicle_lab -- --impact-lab
cargo run -p vehicle_lab -- --impact-report
cargo run -p vehicle_lab -- --rolling-lab
cargo run -p vehicle_lab -- --rolling-report
cargo run -p vehicle_lab -- --inflation-lab
cargo run -p vehicle_lab -- --drop-report
```

- `vehicle_physics` generates and advances the tyre with a windowless solver. Its optional Avian integration couples the moving bead rings to a dynamic rim.
- `vehicle_lab` runs the wheel drop, moving-belt rig, kerb strike, and fixed-bead tread press with interactive views and headless reports.

See the [user guide](docs/user/README.md) for setup, controls, and library usage. The [devlog](docs/devlog/README.md) records the development steps and the tyre roadmap.

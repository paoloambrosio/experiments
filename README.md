# Bevy Vehicle Physics

A Rust workspace for soft-body vehicle physics in Bevy and Avian. The lab drops a reinforced node-and-beam tyre mounted on a rigid rim. Compare rebound, axle height, and the contact patch across four gauge pressures.

```sh
cargo run -p vehicle_lab
cargo run -p vehicle_lab -- --inflation-lab
cargo run -p vehicle_lab -- --drop-report
```

- `vehicle_physics` generates and advances the tyre with a windowless solver. Its optional Avian integration couples the moving bead rings to a dynamic rim.
- `vehicle_lab` runs the wheel drop and the fixed-bead inflation and tread press, with headless reports for both experiments.

See the [user guide](docs/user/README.md) for setup, controls, and library usage. The [devlog](docs/devlog/README.md) records the development steps and the tyre roadmap.

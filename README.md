# Bevy Vehicle Physics

A Rust workspace for soft-body vehicle physics in Bevy, targeting Avian for rigid-body integration. The lab shows a node-and-beam tyre with a reinforced tread band inflating from zero gauge pressure, then deforming and rebounding under a tread press with both bead rings fixed.

```sh
cargo run -p vehicle_lab
```

- `vehicle_physics` generates tyre topology and advances the carcass, tread reinforcement, and sealed air chamber with a windowless solver.
- `vehicle_lab` visualises inflation and compares how the tyre responds to the same tread load at different pressures.

See the [user guide](docs/user/README.md) for setup, controls, and library usage. The [devlog](docs/devlog/README.md) records the development steps and the tyre roadmap.

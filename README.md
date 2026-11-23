# Bevy Vehicle Physics

A Rust workspace for soft-body vehicle physics in Bevy, targeting Avian for rigid-body integration. The lab shows a node-and-beam tyre deforming under a tread load and rebounding against a fixed bead fixture.

```sh
cargo run -p vehicle_lab
```

- `vehicle_physics` generates the tyre topology and advances the elastic carcass with a windowless solver.
- `vehicle_lab` visualises the tyre and runs the tread-load experiment interactively or as a headless report.

See the [user guide](docs/user/README.md) for setup, controls, and library usage. The [devlog](docs/devlog/README.md) records the development steps and the tyre roadmap.

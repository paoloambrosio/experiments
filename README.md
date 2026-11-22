# Bevy Vehicle Physics

A Rust workspace for soft-body vehicle physics in Bevy, targeting Avian for rigid-body integration. The first milestone generates and visualises the rest geometry of a node-and-beam tyre. Elastic deformation is the next milestone.

```sh
cargo run -p vehicle_lab
```

- `vehicle_physics` generates reusable tyre geometry and provides the foundation for the deformable model.
- `vehicle_lab` is a Bevy application for visualising that geometry and running future physics experiments.

See the [user guide](docs/user/README.md) for setup, controls, and library usage. The [devlog](docs/devlog/README.md) records the development steps and the tyre roadmap.

# Bevy Vehicle Physics

A Rust workspace for soft-body vehicle physics in Bevy and Avian. Replay the 1,000 kg reference car launching, turning, and braking on four live node-and-beam tyres, or study an individual tyre on a moving belt and over a step, kerb, or triangle ramp.

```sh
cargo run -p vehicle_lab
cargo run -p vehicle_lab -- --handling-lab
cargo run -p vehicle_lab -- --handling-report
cargo run -p vehicle_lab -- --assembly-report
cargo run -p vehicle_lab -- --chassis-crash-report
cargo run -p vehicle_lab -- --static-contact-report
cargo run -p vehicle_lab -- --coupled-chassis-report
cargo run --release -p vehicle_drive -- --drive
cargo run -p vehicle_drive -- --crash-replay
cargo run -p vehicle_lab -- --drive-report
cargo run --release -p vehicle_lab -- --deformable-drive-report
cargo run -p vehicle_lab -- --alignment-report
cargo run -p vehicle_lab -- --crash-report
cargo run -p vehicle_lab -- --grip-lab
cargo run -p vehicle_lab -- --grip-report
cargo run -p vehicle_lab -- --corner-lab
cargo run -p vehicle_lab -- --corner-report
cargo run -p vehicle_lab -- --surface-report
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

- `vehicle_physics` advances deformable tyres and chassis with a windowless solver. The core supports analytic surfaces, triangle roads, and swept contact with static obstacles. Its optional Avian integration provides a configurable four-wheel assembly, authored wheel alignment, suspension, steering, drive torque, and braking.
- `vehicle_lab` runs visual experiments and headless reports for tyre loading, grip, handling, and impacts. Its imported-car reports use the bundled reference car and test track.
- `vehicle_assets` defines shared terrain geometry and vehicle assets with rigid-body and soft-body inputs. It includes the project-authored Rigs of Rods car and test track.
- `vehicle_drive` drives a live deformable car on the bundled track or loads your own RoR archives through `--config`. It also provides terrain and parked car viewers and a textured frontal crash replay. [Driving and inspection](docs/user/README.md#drive-on-the-reference-track)
- `vehicle_scene` shares imported vehicle setup, collision geometry, spawn placement, and simulation stepping across the driving app and lab.

See the [user guide](docs/user/README.md) for setup, controls, and library usage. The [devlog](docs/devlog/README.md) records the development steps and the vehicle physics roadmap.

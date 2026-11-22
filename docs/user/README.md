# Vehicle physics user guide

## Run the lab

Install a Rust toolchain and the native build and graphics prerequisites for Bevy. This workspace uses Bevy 0.20.1 and targets Avian 0.8.0 for later rigid-body integration. [Bevy setup guide](https://bevy.org/learn/quick-start/getting-started/setup/)

Run this command from the workspace root.

```sh
cargo run -p vehicle_lab
```

The initial view shows all 160 rest nodes and 544 beams in the default tyre. White dots mark nodes, with larger dots on either side of the seam where the final segment connects to the first. The panel shows the total topology counts and each beam family's current visibility.

| Control | Action |
| --- | --- |
| Left mouse button + drag | Orbit around the tyre |
| Scroll | Zoom in or out |
| **R** | Reset the camera orientation and zoom |
| **1** | Toggle circumferential beams (mint) |
| **2** | Toggle cross-section beams (amber) |
| **3** | Toggle diagonal beams (violet) |

## Generate a tyre in Rust

Add `vehicle_physics` as a dependency using the path to `crates/vehicle_physics`. Use `TyreTopology::generate` to create the rest geometry.

```rust
use vehicle_physics::tyre::{TyreConfig, TyreConfigError, TyreTopology};

fn make_tyre() -> Result<TyreTopology, TyreConfigError> {
    TyreTopology::generate(&TyreConfig {
        segments: 48,
        ..TyreConfig::default()
    })
}
```

| Setting | Default | Meaning |
| --- | ---: | --- |
| `segments` | 32 | Nodes around each ring, with a minimum of 3 |
| `bead_radius` | 0.20 m | Radius where the tyre seats on the rim |
| `shoulder_radius` | 0.32 m | Radius at each edge of the tread |
| `tread_radius` | 0.33 m | Radius along the centre of the tread |
| `bead_width` | 0.17 m | Full distance between bead rings |
| `shoulder_width` | 0.20 m | Full distance between shoulder rings |

Dimensions must be positive and finite. The bead radius must be smaller than the shoulder radius, and the tread radius must be at least the shoulder radius. The shoulder width must be at least the bead width. Invalid inputs return a `TyreConfigError`.

Positions are in tyre-local metres with the axle along X and the first segment on positive Y. Read nodes with `nodes()` and beams with `beams()`. Each beam holds two indices into the node slice, its family, and its rest length in metres.

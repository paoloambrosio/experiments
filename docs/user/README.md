# Vehicle physics user guide

## Run the lab

Install a Rust toolchain and the native build and graphics prerequisites for Bevy. This workspace uses Bevy 0.20.1 and targets Avian 0.8.0 for later rigid-body integration. [Bevy setup guide](https://bevy.org/learn/quick-start/getting-started/setup/)

Run this command from the workspace root.

```sh
cargo run -p vehicle_lab
```

The default tyre starts at rest with 160 nodes and 544 beams. Larger dots mark the seam where the last segment connects to the first. Three pink nodes mark the load patch, and grey crosses mark their rest positions. The panel shows the phase, elapsed simulation time, applied load, and signed deflection. Positive deflection means the tread moved inward.

| Control | Action |
| --- | --- |
| **Enter** | Start a fresh replay. Apply 3 N inward for 0.5 s and observe rebound through 2 s |
| Hold **Space** | Press the patch with 3 N. Release the key to let it rebound. Pressing during replay starts a fresh manual run |
| **Backspace** | Reset the tyre to rest |
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

## Advance the elastic tyre

Create an `ElasticTyre` from the topology. Pass one tyre-local force vector in newtons per node and a timestep in seconds. Positions and velocities are in metres and metres per second. Both bead rings are fixed at rest. The solver uses the supplied forces and axial beam forces. Supply gravity as an external force if the experiment needs it.

```rust
use vehicle_physics::elastic_tyre::{ElasticTyre, ElasticTyreConfig};

let mut tyre = ElasticTyre::new(make_tyre()?, ElasticTyreConfig::default())?;
let mut forces = vec![[0.0; 3].into(); tyre.positions().len()];
// make_tyre() above creates 48 segments, so the tread-centre ring starts at 2 × 48.
forces[2 * 48] = [0.0, -1.0, 0.0].into();
tyre.step(1.0 / 960.0, &forces)?;
```

Read `positions()` and `velocities()` in topology order. Call `reset()` to restore the rest shape and clear velocities. A failed step preserves the previous state.

`ElasticTyreConfig` defaults to 8 kg divided equally among all nodes. The coefficients below are illustrative values for the 32-segment lab tyre. Choose a small enough timestep for the node mass and beam coefficients, especially when changing the mesh resolution.

| Beam family | Stiffness (N/m) | Damping (N·s/m) |
| --- | ---: | ---: |
| Circumferential | 12,000 | 4 |
| Cross-section | 20,000 | 6 |
| Diagonal | 8,000 | 3 |

## Compare timesteps without a window

```sh
cargo run -p vehicle_lab -- --elastic-report
```

The report runs the same timed replay at 960, 1,920, and 3,840 substeps per second. It shows peak inward deflection, deflection at release, outward rebound, and residual deflection. It returns an error if the tread fails to rebound past rest, the residual exceeds 0.25 mm, or the finer timestep pair fails to reduce the trace difference below 0.10 mm.

# Vehicle physics user guide

## Run the lab

Install a Rust toolchain and the native build and graphics prerequisites for Bevy. This workspace uses Bevy 0.20.1 and targets Avian 0.8.0 for later rigid-body integration. [Bevy setup guide](https://bevy.org/learn/quick-start/getting-started/setup/)

Run this command from the workspace root.

```sh
cargo run -p vehicle_lab
```

The default tyre starts sealed at 0 kPa gauge with 160 nodes, 544 beams, and a reinforced tread band. Gauge pressure measures the difference from ambient pressure. Press **P** to choose a higher target and watch the tyre inflate from zero. The lab holds the target while the tyre settles, then seals the chamber. Once it is ready, the grey crosses mark the unloaded positions of the three pink tread nodes. The panel shows target and live gauge pressure, volume, load, and deflection from that baseline. Positive deflection means the tread moved inward. **Peak press** records the largest inward deflection under load until the next run or reset. The solver runs at 3,840 substeps per second for the stiffer tread band.

| Control | Action |
| --- | --- |
| **P** | Cycle through 0, 10, 30, and 60 kPa gauge. Each selection starts from zero |
| **Enter** | Start a fresh replay. Apply 3 N inward for 0.5 s and observe rebound through 2 s |
| Hold **Space** | Press the patch with 3 N. Release the key to let it rebound. Pressing during replay starts a fresh manual run |
| **Backspace** | Restore the sealed baseline or restart preparation for the selected target |
| Left mouse button + drag | Orbit around the tyre |
| Scroll | Zoom in or out |
| **R** | Reset the camera orientation and zoom |
| **1** | Toggle circumferential beams (mint) |
| **2** | Toggle cross-section beams (amber) |
| **3** | Toggle diagonal beams (violet) |
| **4** | Toggle the chamber wireframe (blue carcass, orange rim closure) |

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

## Use a sealed air chamber

```rust
use vehicle_physics::{
    elastic_tyre::{ElasticTyre, ElasticTyreConfig},
    pressure::PressureConfig,
};

let mut tyre = ElasticTyre::new_inflated(
    make_tyre()?,
    ElasticTyreConfig::default(),
    PressureConfig::default(),
)?;
tyre.set_gauge_pressure(30_000.0)?;
let reading = tyre.chamber_reading().expect("sealed tyre");
```

`PressureConfig::default()` uses 101,325 Pa absolute ambient pressure and 0 Pa initial gauge pressure. `set_gauge_pressure` changes the gas charge at the current volume. Each subsequent `step` conserves that charge and updates volume in m³ and gauge pressure in Pa. Read the chamber mesh with `pressure_triangles()`. Triangles use tyre-local node indices and face out from the cavity. Call `reset()` to restore the constructor's geometry and initial gas charge. To restore a settled state, keep a clone of the settled tyre, as the lab does.

## Reinforce the tread band

Call `with_tread_reinforcement` on an elastic tyre to fit a stiffer band across the tread-centre and shoulder rings. The same configuration works with the vented and sealed constructors.

```rust
use vehicle_physics::tread_reinforcement::TreadReinforcementConfig;

let mut tyre = tyre.with_tread_reinforcement(TreadReinforcementConfig::default())?;
```

The defaults multiply tread and shoulder hoop stiffness by 20 and multiply the tread-to-shoulder beam stiffness by 2. A 30,000 N/m spring with 5 N·s/m damping at each segment controls the tread centre's distance from the shoulder midpoint. The lab uses these values on the default 32-segment mesh. Choose a timestep appropriate for your mesh, mass, and stiffness.

## Run the headless comparisons

```sh
cargo run -p vehicle_lab -- --radial-report
```

The radial report compares the original and reinforced tyres across all four settled pressures. It prints maximum crown and shoulder radius growth, the largest signed shoulder width change, chamber volume, seal time, and peak deflection under the same 3 N press. It also repeats the reinforced case at 7,680 substeps per second and checks agreement with the lab timestep.

```sh
cargo run -p vehicle_lab -- --inflation-report
```

The inflation report reproduces the original milestone 003 carcass. It compares the vented baseline and all four sealed presets at 960 and 1,920 substeps per second. It prints the settled pressure and volume, time to seal, peak inward deflection, deflection at release, and final residual. Each press is measured from its own unloaded baseline. The report checks that the positive presets settle and stiffen in pressure order, and that the peak deflections agree within 0.10 mm across the two rates.

```sh
cargo run -p vehicle_lab -- --elastic-report
```

The elastic report reproduces the vented milestone 002 replay at 960, 1,920, and 3,840 substeps per second. It shows peak inward deflection, deflection at release, outward rebound, and residual deflection. It returns an error if the tread fails to rebound past rest, the residual exceeds 0.25 mm, or the finer timestep pair fails to reduce the trace difference below 0.10 mm.

# Vehicle physics user guide

## Run the lab

Install a Rust toolchain and the native build and graphics prerequisites for Bevy. This workspace uses Bevy 0.20.1 and Avian 0.8.0. [Bevy setup guide](https://bevy.org/learn/quick-start/getting-started/setup/)

Run this command from the workspace root.

```sh
cargo run -p vehicle_lab
```

### Wheel drop

The default lab starts with the reinforced tyre sealed at **0 kPa gauge** and held 100 mm above the floor. The rim and tyre have a combined mass of 20 kg and fall under their own weight. The panel retains the first rebound rise, settled axle height, projected contact patch, and mean floor force. Pink markers show where nodes are pushing against the floor. The pink outline shows the projection of tread triangles that intersect the floor.

| Control | Action |
| --- | --- |
| **Enter** | Drop the prepared wheel or replay the same drop |
| **P** | Cycle through 0, 10, 30, and 60 kPa gauge and prepare a fresh wheel |
| **Backspace** | Restore the selected wheel to release height or restart pressure preparation |
| Left mouse button + drag | Orbit around the wheel |
| Scroll | Zoom in or out |
| **R** | Reset the camera orientation and zoom |

### Rolling and grip

```sh
cargo run -p vehicle_lab -- --rolling-lab
```

The rig seals the reinforced tyre at **30 kPa gauge**, settles the 20 kg wheel on a stationary belt, then starts the belt at **−2 m/s along Z**. The axle stays upright at a fixed Z position. It can rise, move laterally along X, and spin about X. Each replay starts from the same free-rolling state.

| Control | Action |
| --- | --- |
| **1** | Select free rolling |
| **2** | Select +10 N·m drive torque |
| **3** | Select −10 N·m braking torque |
| **4** | Select +30 N lateral axle load |
| **Enter** | Run or replay the selected case once the baseline is ready |
| **Backspace** | Return to the free-rolling baseline or restart preparation |
| Left mouse button + drag | Orbit around the wheel |
| Scroll | Zoom in or out |
| **R** | Reset the camera orientation and zoom |

The panel shows live readings during the run and the mean over two complete revolutions at the end. Blue and coral axle arrows show the tyre force transmitted along Z and X, respectively. Positive longitudinal slip means the tyre spins faster than free rolling at the same belt speed. Positive slip angle means the axle moves toward +X relative to the belt.

### Kerb impact and damage

```sh
cargo run -p vehicle_lab -- --impact-lab
```

The lab launches the reinforced tyre at 30 kPa initial gauge along +Z toward a 60 mm kerb with a 10 mm rounded leading edge. The wheel coasts through the strike. The lab then holds the rim fixed and lets the tyre relax without road contact or gravity. The chamber stays sealed throughout.

| Control | Action |
| --- | --- |
| **1**, **2**, **3** | Select 2, 4, or 6 m/s launch speed |
| **Enter** | Strike or replay at the selected speed |
| **Backspace** | Restore the prepared wheel or restart inflation |
| Left mouse button + drag | Orbit around the wheel |
| Scroll | Zoom in or out |
| **R** | Reset the camera orientation and zoom |

Gold marks yielded beams. Red segments with a visible gap mark broken beams. The white overlay shows the original tread-centre profile against the recovered tyre. The panel reports the actual strike speed, peak inward displacement, residual displacement after unloading, and damage counts.

### Inflation and tread press

```sh
cargo run -p vehicle_lab -- --inflation-lab
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

## Mount a tyre on a rigid rim

`MountedTyre::new` takes an elastic tyre, a `RimMotion` and an optional `FlatFloor`. Mounting transforms the tyre's current deformation and velocities into world space and preserves the sealed gas charge. World +Y is up. The axle is along the rim's local +X. The drop bench uses `FlatFloor::new(0.0, 35_000.0, 150.0)`. Stiffness is in N/m and damping is in N·s/m per tread node. The bench uses three times the default axial beam damping to settle the wheel after impact.

With the `vehicle_physics` feature `avian`, install the adapter after Avian's physics plugins and spawn the rim on the same entity as its mounted tyre. For the default reinforced tyre, the drop lab runs physics at 60 ticks/s with 64 substeps per tick.

```rust
use avian3d::prelude::{PhysicsPlugins, SubstepCount};
use bevy::prelude::{Fixed, Time};
use vehicle_physics::avian_tyre::{AvianTyre, AvianTyrePlugin};

app.add_plugins((PhysicsPlugins::default(), AvianTyrePlugin))
    .insert_resource(Time::<Fixed>::from_hz(60.0))
    .insert_resource(SubstepCount(64));
app.world_mut().spawn(AvianTyre::rim_bundle(mounted, 12.0)?);
```

The bundle includes the bare rim mass plus the mass and inertia of both bead rings. The adapter advances the free tyre nodes using Avian's gravity and substep duration, applies the bead wrench to the rigid body, then moves the beads to the updated pose. Read world positions and velocities through `AvianTyre::tyre()` and check `AvianTyre::error()` for a failed step. `MountedTyre::footprint()` returns the clipped polygons and their world Z and X spans in metres.

For a caller-managed rigid body, call `MountedTyre::step` with the substep duration and gravity to get force and torque about the rim centre. Apply that wrench and the rim's gravity, then call `sync_rim` with the updated pose and velocities. The rigid body should include the bead mass and inertia returned by `rim_mass_properties`.

### Enable grip on a flat floor

`FlatFloor::new` creates a stationary, frictionless floor. Set a dimensionless coefficient and a positive transition speed (m/s) to enable grip. A moving surface takes a world-space velocity in m/s with a zero Y component.

```rust
use bevy::math::DVec3;
use vehicle_physics::mounted_tyre::FlatFloor;

let belt = FlatFloor::new(0.0, 35_000.0, 150.0)?
    .with_friction(0.8, 0.1)?
    .with_surface_velocity(DVec3::NEG_Z * 2.0)?;
// mounted is an existing MountedTyre.
mounted.set_floor(Some(belt));
```

`AvianTyre::set_floor` also updates the surface for subsequent substeps. `MountedStep::floor_force_n` gives total upward support, `floor_tangential_force` gives the horizontal floor force on the tyre, and `rim_force` gives the bead force on the rim. All are in newtons and use world axes. At a supported node the horizontal force opposes its motion relative to the belt, with a shared longitudinal and lateral limit of the coefficient times that node's normal force.

### Enable axial damage and kerb contact

```rust
use vehicle_physics::{
    beam_damage::BeamDamageConfig,
    kerb::Kerb,
    mounted_tyre::MountedTyre,
};

// tyre, rim, and floor come from the examples above.
let material = BeamDamageConfig::new(0.10, 0.09, 0.35, 0.12)?;
let mounted = MountedTyre::new(tyre.with_beam_damage(material), rim, Some(floor))?
    .with_kerb(Kerb::new(0.0, 0.06, 0.01)?);
```

The material arguments are tensile yield, compressive yield, tensile fracture, and compressive fracture. They are positive dimensionless strain limits. `beam_damage()` on either tyre returns states in topology beam order. Each state reports its stress-free length, permanent length change, yield history, and fracture status. `reset()` on the fixed-bead tyre restores its undamaged state.

`Kerb::new` takes the leading face's world Z, the rise above the floor, and the edge radius, all in metres. The top extends along +Z and the cross-section spans world X. Kerb contact uses the floor's stiffness, damping, and friction coefficients. `MountedStep::kerb_force` gives the world-space reaction on the tyre and `kerb_normal_force_n` gives the sum of normal force magnitudes, both in newtons. `into_fixed_bead_tyre()` returns the tyre in the rim's local frame for unloaded measurements, preserving its deformation, damage history, and gas charge.

## Run the headless comparisons

```sh
cargo run -p vehicle_lab -- --impact-report
cargo run -p vehicle_lab -- --impact-report --samples
```

The impact report runs three kerb strikes and a flat-road control at 3,840 and 7,680 substeps/s. It reports actual strike speed, inward displacement, residual displacement, damage counts, and broken beam indices. `--samples` adds lab-rate coast traces at 60 Hz and one pair of tyre-local positions per node for the original and unloaded profiles. CSV positions and displacements are in metres.

```sh
cargo run -p vehicle_lab -- --rolling-report
cargo run -p vehicle_lab -- --rolling-report --samples
```

The rolling report measures spin, signed slip ratio, slip angle, normal support, and floor and axle forces along Z and X for all four load cases. It checks the expected slip and force directions and compares the means at 3,840 and 7,680 substeps/s. `--samples` adds tagged CSV rows at 60 Hz from the 3,840 substeps/s runs, with fractional slip ratio and slip angle in degrees.

```sh
cargo run -p vehicle_lab -- --drop-report
```

The drop report compares release-to-contact time, first rebound rise, settled axle height, footprint length and width, mean floor force, and maximum tread penetration at 3,840 and 7,680 substeps/s. Add `--samples` after `--drop-report` for tagged CSV rows with axle height over time, the settled bottom cross-section, and the projected footprint polygons at the lab rate.

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

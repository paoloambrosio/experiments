# Vehicle physics user guide

## Drive on the reference track

```sh
cargo run -p vehicle_drive -- --drive
```

The app opens the project-authored car and test track bundled with the executable. Run without flags to start driving. The chase view shows speed and follows the car as its four deformable tyres roll over the track. In the starting chase view, the ramp is to the right and the wall is farther ahead to the left.

| Key | Action |
| --- | --- |
| **W** | Apply forward throttle |
| **S** | Apply service brake |
| **A / D** | Steer left / right |
| **R** | Reset the car and chase camera to the start |
| **C** | Toggle terrain collision edges |

The drive uses authored camber and toe, fixed first-gear torque, and an imported suspension at each wheel. Only the tyres contact the track in this interactive mode. Startup diagnostics list the derived mass, suspension, tyre, and solver settings.

## Inspect the track or parked car

```sh
cargo run -p vehicle_drive -- --view-terrain
cargo run -p vehicle_drive -- --view-vehicle
```

In the terrain viewer, move with **WASD** and **Q / E**. Hold the right mouse button and drag to look around. In the parked-car viewer, hold the right mouse button and drag to orbit, scroll to zoom, and press **B** to switch between textured and structural views. The structural view shows nodes, beams, wheel outlines, and axle directions. In either viewer, **Home** restores the camera and **C** toggles the collision overlay.

## Run reports on the reference car and track

```sh
cargo run -p vehicle_lab -- --drive-report
cargo run -p vehicle_lab -- --alignment-report
cargo run -p vehicle_lab -- --crash-report
```

The drive report checks four-wheel support, acceleration, steering, braking, rim clearance, and loaded contact on the ramp. The alignment report compares upright and authored wheel alignment on the same level straight. It measures tread load centres in the final second of settling and mean absolute lateral force per wheel while accelerating from 2 to 4 m/s. Positive load-centre offset points left, positive camber leans the wheel top outward, and positive toe points the wheel inward.

The crash report launches the deformable car toward the bundled wall at 8 m/s. The control removes the wall from chassis collision and tyre support. The report compares matched runs, measures chassis contact and nose shortening, checks for at least 1 mm of permanent beam-length change beyond the control, and repeats the strike after reset. With the current fixture, the chassis hits the wall at 7.52 m/s and retains 13.42 mm of beam-length change beyond the control.

See the [reference asset source and regeneration command](../../crates/vehicle_assets/reference/README.md).

## Crash test a deformable chassis

Run the headless wall-strike replay from the workspace root.

```sh
cargo run -p vehicle_lab -- --chassis-crash-report
```

The synthetic chassis runs light and hard frontal strikes, then an offset strike from both sides. The report prints nose and cabin shortening in millimetres, rebound speed, yaw at wall release, beam yielding, and wall impulse. It fails if the checks or deterministic reset replay fail. [Core 015 fixture and measurements](../devlog/015-soft-body-chassis-impacts/README.md)

Run the static contact report to test a narrow post striking the bumper between its nodes and a loaded tyre striking a wall before the chassis. The report prints the first contact feature, impulses, residual contact depth, and permanent nose shortening. [Core 016 fixtures and measurements](../devlog/016-static-world-contact/README.md)

```sh
cargo run -p vehicle_lab -- --static-contact-report
```

Run the coupled chassis report for a reference car with a deformable frame and four live wheels. It measures settled load and sag, driving and braking at two solver rates, and impact transfer in both directions through the suspension. It also replays a one-second lateral wall strike at each rate and measures penetration directly from the deformed tyre and chassis geometry. [Coupled chassis fixtures](../devlog/017-deformable-chassis-suspension/README.md) and [sustained wall contact measurements](../devlog/018-sustained-tyre-contact/README.md).

```sh
cargo run -p vehicle_lab -- --coupled-chassis-report
```

## Use imported asset geometry

Use the `vehicle_assets` crate to represent terrain meshes, height samples, and vehicle and wheel geometry in memory. `VehicleAsset::physics` exposes optional rigid-body and structural inputs in SI units. Call `TerrainAsset::validate()` or `VehicleAsset::validate()` before consuming these values. Validation errors identify the field and any relevant element indices. Positions and dimensions use metres, with +Y up, +Z forward, and +X to vehicle left.

The Rigs of Rods importer exposes node masses and upright assignments, ordinary and compression-support beam properties, contact panels and nodes, and the placement and influence nodes for each deformable mesh instance. Chassis and upright masses partition the authored node mass. The rim and tyre mass is supplied separately for each wheel. The structural data is available through `VehicleAsset::physics.structure`. The drive app currently uses the rigid chassis and soft-body tyres.

## Spawn the reference assembly

Enable the core crate's `avian` feature. Install Avian's `PhysicsPlugins`, `AvianTyrePlugin`, and `AvianSuspensionPlugin`. The reference uses gravity along −Y at 9.81 m/s², a 60 Hz fixed clock, and 64 substeps per tick. Prepare the sealed tyre at the matching substep rate and cache it for resets.

```rust
use vehicle_physics::reference_car::{ReferenceCar, ReferenceTyre};

let mut prepared = ReferenceTyre::new(3840);
while !prepared.advance()? {}
let car = ReferenceCar::spawn(world, &prepared.tyre)?;
// car.wheels is ordered front left, front right, rear left, rear right.
// To reset, remove the assembly and spawn again with the cached tyre.
car.despawn(world);
```

The chassis starts at the origin in X and Z with +Y up, +Z forward, and +X to the left. Each wheel has its own deformable tyre and gas charge. [Wheel steering and torque commands](#steer-drive-and-brake-the-wheels)

## Spawn a configured four-wheel vehicle

Build a `VehicleSpec` with the chassis mass, inertia-box dimensions, centre of mass, and four wheel specifications. Start from the reference specification when you want its parameters as a template.

```rust
use bevy::prelude::{Quat, Vec3};
use vehicle_physics::{reference_car, vehicle_assembly::{VehiclePose, VehicleRoad}};

let spec = reference_car::specification();
let prepared = spec.prepare(3840)?;
let pose = VehiclePose {
    origin_m: Vec3::new(5.0, 0.52, -2.0),
    rotation: Quat::from_rotation_y(0.3),
};
let car = prepared.spawn(world, pose, &VehicleRoad::Flat(reference_car::road()))?;
let [left, right] = car.geometry.front_steer(0.1).expect("valid steering angle");
// Apply these angles to the front spin joints with WheelControl.
car.despawn(world);
```

The pose places the vehicle-local origin. The chassis entity sits at the transformed centre of mass, and each wheel starts at its transformed local centre. Create each `WheelAlignment` from a unit vehicle-local axle pointing left and pass the array to `prepared.with_wheel_alignments()` in FL, FR, RL, RR order. Preparation defaults to upright wheels. The caller sets the start height using the road and inflated tyres. For tilted wheels, transform the tyre-local nodes from `prepared.unloaded_tread_positions_m()` by the wheel and vehicle rotations before checking road clearance. Mount offsets come from the wheel centres and initial suspension lengths. Use `VehicleRoad::Triangles` to share a static triangle road across the four tyres. Reuse the prepared specification across spawns. [Triangle road setup](#use-a-static-triangle-road)

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

### Whole-car handling

```sh
cargo run -p vehicle_lab -- --handling-lab
```

The flat-road lab replays a 1,000 kg car on four sealed 150 kPa tyres. Each run settles the car for four seconds before launching. The road arrows and panel show the mean support of each wheel over the current physics tick. A red arrow and the **LOCKED** label mark sustained lock. The path is blue under drive and orange under braking, with a grid at 5 m intervals.

| Control | Action |
| --- | --- |
| **1**, **2**, **3** | Select the moderate stop from 4 m/s, moderate stop from 8 m/s, or limit stop from 8 m/s |
| **4**, **5**, **6**, **7** | Select left and right turns at 4 m/s, then left and right turns at 8 m/s |
| **8**, **9** | Select moderate or limit braking in the 8 m/s left turn |
| **Left**, **Right** | Cycle through the nine cases |
| **Enter** | Replay the selected case from rest |
| **Backspace** | Clear the replay or restart tyre preparation |
| Left mouse button + drag | Orbit around the chassis |
| Scroll | Zoom in or out |
| **R** | Reset the camera orientation and zoom |

Add `--benchmark` to replay case 9 automatically with all four tyres stepping live. [Measured handling results](../devlog/010-scripted-whole-car-handling/README.md)

### Loaded tyre grip

```sh
cargo run -p vehicle_lab -- --grip-lab
```

The lab measures drive, braking, lateral, and combined forces on the sealed 150 kPa light-car tyre. The axle moves vertically as the tread deforms. For each load and speed the lab first lets the wheel spin freely to measure its effective rolling radius, then uses that state to start each replay. Road and axle arrows show horizontal force in world coordinates. A positive slip angle means the belt has a velocity component toward −X and the tyre develops a force toward −X.

| Control | Action |
| --- | --- |
| **1**, **2**, **3** | Select 50%, 100%, or 150% of the nominal 2,452.5 N load |
| **V** | Toggle the belt between 2 and 8 m/s |
| **Left**, **Right** | Cycle through the 19 slip cases |
| **Enter** | Replay the selected case from the saved calibration |
| **Backspace** | Restore the calibration or restart preparation |
| Left mouse button + drag | Orbit around the tyre |
| Scroll | Zoom in or out |
| **R** | Reset the camera orientation and zoom |

The default case is straight rolling at nominal load and 2 m/s. The readout shows live forces during the replay and the final mean over two revolutions. The benchmark uses nominal load, 8 m/s, +10% longitudinal slip, and +5° slip angle.

### Loaded wheel and suspension

```sh
cargo run -p vehicle_lab -- --corner-lab
```

The rig prepares and seals the light-car tyre at **150 kPa initial gauge**. The **230 kg** sled and **20 kg** unsprung assembly represent one corner of an evenly loaded 1,000 kg car. Each replay spends four seconds settling under gravity, then accelerates to 2 m/s. The step case crosses a 20 mm rise with an 8 mm rounded edge. The triangle course crosses a coplanar seam at Z = 2 m, climbs 100 mm over one metre, and continues on a level platform. The panel shows axle height and body centre height above the original floor, spring compression from its free length, and the vertical road reaction.

| Control | Action |
| --- | --- |
| **1** | Select the flat-road control |
| **2** | Select the rounded step |
| **3** | Select the triangle ramp |
| **Enter** | Start a fresh replay of the selected case once inflation finishes |
| **Backspace** | Reset the replay or restart inflation |
| Left mouse button + drag | Orbit around the corner |
| Scroll | Zoom in or out |
| **R** | Reset the camera orientation and zoom |

Run the same tow against analytic flat, triangulated flat, and triangulated ramp surfaces without opening a window.

```sh
cargo run -p vehicle_lab -- --surface-report
cargo run -p vehicle_lab -- --surface-report --samples
```

The report checks settled support, the coplanar seam, ramp traversal, and solver-rate agreement. With `--samples`, standard output contains 60 Hz CSV data from all three cases at both solver rates. The summary goes to standard error.

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

### Sidewall puncture

```sh
cargo run -p vehicle_lab -- --puncture-lab
```

Watch the wheel strike the kerb and coast on top for five seconds. The panel shows live gauge pressure, opening area, net air lost, and loaded axle height above the kerb top. Pink outlines mark panels with an effective opening. Gold and red still indicate yielded and broken beams.

| Control | Action |
| --- | --- |
| **1** | Select the 6 m/s strike with sidewall tearing enabled |
| **2** | Select the sealed 6 m/s comparison |
| **3** | Select the 4 m/s yielding comparison |
| **Enter** | Launch or replay the selected case |
| **Backspace** | Restore the prepared tyre or restart inflation |
| Left mouse button + drag | Orbit around the wheel |
| Scroll | Zoom in or out |
| **R** | Reset the camera orientation and zoom |

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

### Performance

`cargo run -p vehicle_lab` uses an optimised development profile with debug symbols and assertions. Normal windows sync to the display. Add `--no-vsync` after the Cargo separator to run an interactive lab without that cap on supported backends.

```sh
cargo run -p vehicle_lab -- --puncture-lab --no-vsync
```

Use `--benchmark` for an automated windowed run. It requests no VSync, warms the renderer for two seconds, measures preparation and one second of ready/idle time, then starts a replay and exits when it finishes. Leave the window at its default size and avoid changing controls during the run.

```sh
cargo run -p vehicle_lab -- --benchmark
cargo run -p vehicle_lab -- --grip-lab --benchmark
cargo run -p vehicle_lab -- --corner-lab --benchmark
cargo run -p vehicle_lab -- --rolling-lab --benchmark
cargo run -p vehicle_lab -- --impact-lab --benchmark
cargo run -p vehicle_lab -- --puncture-lab --benchmark
cargo run -p vehicle_lab -- --inflation-lab --benchmark
```

Read average FPS alongside the 95th-percentile and longest frame times. The fixed-update number is the average CPU wall time per 60 Hz tick, including the tyre's 64 substeps when active. A fixed clock reading near `1.000x` means the simulation clock kept pace with wall time. Add Cargo's `--release` before the separator to compare against a release build.

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

### Attach a suspension corner

Enable the same `avian` feature and install `AvianSuspensionPlugin` alongside `AvianTyrePlugin` after Avian's physics plugins. Create a chassis rigid body and prepare a `MountedTyre` with its centre below the mount, at a distance within the suspension travel limits. Align the rim with the chassis at spawn. Chassis-local +Y points up and +Z points forward. The wheel spins about its local +X.

```rust
use bevy::prelude::Vec3;
use vehicle_physics::avian_suspension::{CornerConfig, Suspension, WheelCorner};

// chassis is an existing Avian rigid body. mounted is the tyre and rim pose.
let corner = WheelCorner::spawn(app.world_mut(), chassis, mounted, CornerConfig {
    mount: Vec3::new(0.0, -0.28, 0.0),
    suspension: Suspension {
        stiffness_n_per_m: 30_000.0,
        damping_ns_per_m: 2_000.0,
        free_length_m: 0.25,
        min_length_m: 0.06,
        max_length_m: 0.29,
    },
    upright_mass_kg: 3.0,
    bare_rim_mass_kg: 9.0,
})?;
```

The adapter creates a hub, a rim entity carrying `AvianTyre`, a travel-limited prismatic joint, and a spin hinge. The spring and damper exchange force between the chassis mount and hub at each substep. In this example the 3 kg hub, 9 kg bare rim, and 8 kg tyre add up to 20 kg. Read the tyre and contact forces from `AvianTyre` on `corner.rim`. The caller owns the returned entities and both joints.

### Steer, drive, and brake the wheels

Update `WheelControl` on each corner's spin joint before the Avian physics step. Positive steer rotates +Z towards +X around the hub's local +Y, and positive drive spins the rim about the steered +X axis. The brake torque capacity is a nonnegative value. The brake opposes relative axle spin and caps its impulse at the amount required to cancel spin.

```rust
use vehicle_physics::avian_suspension::{WheelControl, ackermann_steer};

// front_left and front_right are WheelCorner values. +X is vehicle left.
let [left, right] = ackermann_steer(2.5, 1.5, 6.0_f32.to_radians()).unwrap();
for (corner, steer_rad) in [(front_left, left), (front_right, right)] {
    let mut control = app.world_mut().get_mut::<WheelControl>(corner.spin_joint).unwrap();
    *control = WheelControl { steer_rad, drive_nm: 0.0, brake_nm: 250.0 };
}
```

The Ackermann helper takes wheelbase and track in metres, plus an equivalent steer in radians. It gives the front wheel angles in +X, −X order. Use zero steer at the rear and positive drive torque at the desired driven wheels. All three fields default to zero.

### Use a static triangle road

Build a `TriangleRoad` from zero-based triangle indices and world-space vertices in metres, with +Y up. Pass it to `MountedTyre::set_triangle_road` or `AvianTyre::set_triangle_road`. Give each tyre a clone of the same `Arc` to share the spatial index.

```rust
use std::sync::Arc;
use vehicle_physics::triangle_road::TriangleRoad;

// vertices and triangles contain the world-space road geometry.
let road = Arc::new(TriangleRoad::new(&vertices, &triangles, 0.05)?);
mounted.set_triangle_road(Arc::clone(&road), floor.contact_parameters());
```

The third constructor argument is the maximum accepted vertical penetration in metres. Choose it above the expected tread penetration and below the gap between stacked road levels. Both face windings work. Tread support comes from the top of finite road faces. Use `triangle_reaction()` for the last successful substep’s normal and tangential force vectors and peak normal penetration. Use `active_contacts()` for the tread node indices that would receive positive support at the current pose. Call `set_floor` to return to the analytic surface.

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

### Enable sidewall punctures

Enable axial damage on an inflated `ElasticTyre` first, then configure sidewall openings before mounting.

```rust
use vehicle_physics::puncture::PunctureConfig;

// tyre is an inflated ElasticTyre with axial damage enabled.
let tyre = tyre.with_puncture(PunctureConfig::new(0.001, 0.65)?)?;
```

The arguments are the maximum opening as a fraction of a panel's original area and the orifice discharge coefficient. Both are dimensionless. `torn_panels()` identifies the open panels by side and segment and returns their topology node indices and opening areas in m². `puncture_reading()` reports the total area and mass flow in kg/s, with positive flow leaving the tyre. `chamber_air_mass_kg()` also works with a sealed chamber. These readings are available on fixed-bead and mounted tyres. Reinflating changes the air mass and preserves the openings. Resetting the fixed-bead tyre restores the intact sidewalls and initial air charge.

## Run the headless comparisons

`cargo run -p vehicle_lab -- --assembly-report` compares reference-car loads, sag, and stopping distances with a second four-wheel assembly. The second car uses different tyre sizes and tracks at each axle and drives on a triangle floor.

```sh
cargo run -p vehicle_lab -- --handling-report
cargo run -p vehicle_lab -- --handling-report --samples
```

The handling report checks nine cases at 3,840 and 7,680 substeps/s. Filter `handling` rows for per-case launch, turning, and braking results. Lock times are seconds from brake onset. `handling_comparison` gives the signed fraction of steady path curvature retained under braking. `--samples` adds `handling_chassis` and `handling_wheel` rows at 60 Hz, with road forces averaged over each tick's substeps. World +Y is up, initial forward is +Z, and initial left is +X. Wheel loads are ordered front left, front right, rear left, rear right. [Fixture, metrics, and measured paths](../devlog/010-scripted-whole-car-handling/README.md)

```sh
cargo run -p vehicle_lab -- --grip-report
cargo run -p vehicle_lab -- --grip-report --samples
```

The grip report sweeps 19 slip cases at three vertical loads and two belt speeds. It repeats the sweep at 3,840 and 7,680 substeps/s. Filter rows by their first CSV field. `calibration` gives effective radius and free-rolling drag as a positive magnitude. `grip` gives the commanded and measured slip, spin, and belt velocity, along with support and signed forces. `--samples` adds `grip_trace` rows sampled at 60 Hz through the target ramp and settling period. Longitudinal force follows +Z and lateral force follows +X. [Fixture and measured curves](../devlog/009-loaded-tyre-grip/README.md)

```sh
cargo run -p vehicle_lab -- --corner-report
cargo run -p vehicle_lab -- --corner-report --samples
```

The corner report applies increasing static axle loads to the original tyre and compares the new light-car preset at 2,452.5 N. It checks static support and sag, clearance through the step, and recovery to the raised surface. It repeats the two tow runs at 3,840 and 7,680 substeps/s. The `--samples` flag adds tagged 60 Hz traces with positions and compression in metres and forces in newtons. [Measured results and configuration](../devlog/008-loaded-wheel-and-suspension/README.md)

```sh
cargo run -p vehicle_lab -- --puncture-report
cargo run -p vehicle_lab -- --puncture-report --samples
```

The puncture report compares all three coasting cases at 3,840 and 7,680 substeps/s. It reports each increase in total opening, final pressure and air loss, and mean loaded axle height and vertical road support over the final second. The fixed-volume check compares the initial flow through one opening with the flow through two equal openings. `--samples` adds tagged CSV for tear events, 60 Hz traces with contact flags, and final profiles at both rates. Profile coordinates use world axes, with X and Z relative to the axle and Y relative to the kerb top.

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

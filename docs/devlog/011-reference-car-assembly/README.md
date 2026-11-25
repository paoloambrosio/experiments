# 011 — Reference car assembly

The 1,000 kg reference car from 010 is now available through the core crate's `reference_car` module with the `avian` feature. A second caller can assemble the car that produced the handling measurements without copying its mount geometry, suspension settings, mass distribution, or road contact parameters. [Geometry and measured baseline](../010-scripted-whole-car-handling/README.md)

## Prepare one tyre, mount four copies

`ReferenceTyre::new(rate)` starts the light-car material and tread reinforcement at zero gauge pressure. Each call to `advance()` advances one fixed-bead inflation substep. The shared inflation routine ramps gauge pressure to 150 kPa over one second. It holds that pressure until every node has travelled slower than 0.1 mm/s for a quarter second. Once preparation completes, the caller can keep the sealed shape and gas charge for subsequent assemblies.

`ReferenceCar::spawn(world, &prepared.tyre)` creates a 920 kg chassis and four independent 20 kg corners. World +Y is up, +Z is forward, and +X points to vehicle left. The helper returns the chassis and the front-left, front-right, rear-left, and rear-right corners in that order. Each corner exposes its spin joint for commands and its mounted tyre for measurements. `despawn` removes the four assemblies, their joints, and the chassis.

## Regression checks

```sh
cargo test --workspace
cargo run -p vehicle_lab -- --corner-report
cargo run -p vehicle_lab -- --grip-report
cargo run -p vehicle_lab -- --handling-report
```

The corner report checks static support against 2,452.5 N per wheel and suspension sag against 75.21 mm. The grip report exercises 228 loaded-tyre cases. The handling report repeats nine whole-car manoeuvres at 3,840 and 7,680 substeps/s. All three reports passed with the shared preparation and assembly. At 3,840 substeps/s the straight stop from 7.95 m/s still measures 10.94 m with moderate braking and 4.08 m with limit braking.

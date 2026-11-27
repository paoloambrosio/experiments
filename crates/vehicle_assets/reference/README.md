# Reference car and test track

`car.zip` contains the four-wheel reference car and a variant with wider tyres. `track.zip` contains the level track, ramp, and wall. Both are authored in this repository. The driving app and lab reports embed these archives and load them with the Rigs of Rods importer.

Edit the [generator](../examples/build_reference.rs) to change the truck records, track layout, meshes, or textures. Rebuild both archives from the workspace root and include them with any fixture source change.

```sh
cargo run -p vehicle_assets --example build_reference
cargo test -p vehicle_assets
```

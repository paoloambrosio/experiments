# Devlog

Each entry documents a development step in the `vehicle_physics` core crate. Supporting diagrams, media, and deep dives live beside the corresponding entry.

## Completed

- [001 — Tyre topology](001-tyre-topology/README.md) builds and inspects the five-ring rest geometry.
- [002 — Elastic carcass](002-elastic-carcass/README.md) measures tread deflection and rebound with the beads fixed.
- [003 — Inflation and tread reinforcement](003-inflation/README.md) compares tread deflection at four gauge pressures and limits crown growth with a stiffer tread band.
- [004 — Rim and ground contact](004-rim-and-ground-contact/README.md) couples the tyre to an Avian rim and measures rebound, settled axle height, and the footprint on a flat floor.
- [005 — Rolling and grip](005-rolling-and-grip/README.md) adds tread friction and measures slip and axle force on a moving belt under drive, braking, and lateral loads.
- [006 — Impact and damage](006-impact-and-damage/README.md) measures temporary deflection, permanent set, and beam fracture in a coasting-wheel kerb strike.
- [007 — Puncture and pressure loss](007-puncture-and-pressure-loss/README.md) maps sidewall fractures to growing openings and measures air loss, loaded shape, and support against a sealed replay.
- [008 — Loaded wheel and suspension](008-loaded-wheel-and-suspension/README.md) tests the tyre under a 1,000 kg car's corner load and measures suspension travel over a rounded step.
- [009 — Loaded tyre grip](009-loaded-tyre-grip/README.md) measures longitudinal, lateral, and combined slip at three loads and two speeds.
- [010 — Scripted whole-car handling](010-scripted-whole-car-handling/README.md) measures launch, turns, stopping distances, load transfer, and wheel lock with four live deformable tyres.
- [011 — Reference car assembly](011-reference-car-assembly/README.md) extracts the sealed reference tyre and four-corner chassis assembly into the core crate and checks them against the loaded-wheel and whole-car baselines.
- [012 — Tyre contact on driving surfaces](012-tyre-contact-on-driving-surfaces/README.md) adds static triangle roads and measures a loaded wheel crossing a coplanar seam and climbing a ramp.
- [013 — Data-driven rigid vehicle assembly](013-data-driven-rigid-vehicle-assembly/README.md) uses one assembly path for the reference car and a second car with staggered tyres and axle tracks.
- [014 — Authored wheel alignment](014-authored-wheel-alignment/README.md) preserves rest camber and toe and measures their effect on tread loading and straight-line lateral forces.
- [015 — Soft-body chassis impacts](015-soft-body-chassis-impacts/README.md) measures elastic rebound, permanent nose crush, and captured wall impulse in full-width and offset strikes.
- [016 — Soft-body contact with static obstacles](016-static-world-contact/README.md) measures panel contact with a narrow post and a wheel-first side impact on a loaded tread.
- [017 — Deformable chassis and suspension](017-deformable-chassis-suspension/README.md) couples four live wheels to a structural frame and measures how impacts travel through its mounts.
- [018 — Sustained tyre contact with static obstacles](018-sustained-tyre-contact/README.md) follows a wheel-first strike for one second and checks the final bead path after suspension coupling.

These core milestones support the [imported terrain and vehicle roadmap](../drive-roadmap.md).

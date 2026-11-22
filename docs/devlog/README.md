# Devlog

Each entry documents one development step. Supporting diagrams, media, and deep dives live beside the corresponding entry.

## Completed

- [001 — Tyre topology](001-tyre-topology/README.md) builds and inspects the five-ring rest geometry.

## Planned

| Milestone | Implementation and experiment |
| --- | --- |
| 002 — Elastic carcass | Assign node masses, fix the bead nodes to a fixture, and apply equal and opposite axial spring and damping forces at beam endpoints. Advance on fixed substeps. Use a controlled tread load to measure deflection and rebound, then reduce the timestep to check that the results converge. |
| 003 — Inflation | Build a consistently oriented, closed pressure surface including a closure against the rim. Use the enclosed volume to update pressure and distribute forces over the surface nodes. Compare deflection at several inflation pressures under the same load. |
| 004 — Rim and ground contact | Couple the beads to an Avian rigid rim, returning the tyre's reactions to the rim as force and torque. Resolve normal contact with a flat floor. Drop the wheel and measure rebound, settled axle height, and the contact patch. |
| 005 — Rolling and grip | Add tangential contact forces and apply drive torque, braking, and lateral load to the wheel. Measure slip and the corresponding forces at the axle, using rolling at a steady speed as a baseline. |
| 006 — Impact and damage | Add beam yielding and breakage to distinguish elastic deflection from lasting damage. Strike the same kerb at repeatable speeds and record peak deformation, unloaded shape, and broken beams. Use the experiment to identify which node-and-beam behaviours are reusable for the vehicle structure. |


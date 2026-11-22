# Bevy Vehicle Physics

Build a reusable soft-body vehicle physics library for Bevy and Avian. Use node-and-beam structures so tyres and vehicle structures can deform on impact, with Rigs of Rods and BeamNG as references for the behaviour. Start with the tyre.

- Put reusable model and simulation code in the `vehicle_physics` core crate. Keep the lab's rendering, input, and experiment setup in `vehicle_lab`.
- Keep the model testable without a window. Use SI units and document the coordinate conventions of public APIs.
- Document each completed development step in its own numbered directory under `docs/devlog`, such as `docs/devlog/001-tyre-topology/README.md`. Keep that entry's media and optional deep dives in the same directory. Explain the reasoning, what changed, and how to reproduce the checks so the project can be followed as a learning exercise.
- Use SVG for geometry, dimensions, and diagrams where placement matters. Keep SVG files beside the document and embed them with relative Markdown image links and descriptive alt text. Use Mermaid code blocks in Markdown for flows, sequences, and relationships that suit automatic layout. Avoid ASCII diagrams in project documentation.
- Update the [devlog index and roadmap](docs/devlog/README.md) when adding an entry or changing upcoming milestones.
- Keep setup, API usage, configuration, and lab controls current in [the user documentation](docs/user/README.md). Keep the root README current as the project entry point.

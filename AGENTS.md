# Bevy Vehicle Physics

Build a reusable soft-body vehicle physics library for Bevy and Avian. Use node-and-beam structures so tyres and vehicle structures can deform on impact, with Rigs of Rods and BeamNG as references for the behaviour. Start with the tyre.

- Put reusable model and simulation code in the `vehicle_physics` core crate. Keep the lab's rendering, input, and experiment setup in `vehicle_lab`.
- Keep the model testable without a window. Use SI units and document the coordinate conventions of public APIs.
- Use `docs/devlog` exclusively for changes to the `vehicle_physics` core crate. Give each completed core development step a numbered directory with its own media, reasoning, and reproduction checks. Document application setup, controls, scenes, and camera behaviour in `docs/user`. Application-only work receives no devlog entry or number.
- Use SVG for geometry, dimensions, and diagrams where placement matters. Keep SVG files beside the document and embed them with relative Markdown image links and descriptive alt text. Use Mermaid code blocks in Markdown for flows, sequences, and relationships that suit automatic layout. Avoid ASCII diagrams in project documentation.
- Update the [devlog index and roadmap](docs/devlog/README.md) when adding an entry or changing upcoming milestones.
- Keep setup, API usage, configuration, and lab controls current in [the user documentation](docs/user/README.md). Keep the root README current as the project entry point.

use bevy::math::DVec3;
use vehicle_assets::{NodeId, StructuralBeamKind, VehicleAsset, WheelPhysics};
use vehicle_physics::{
    beam_damage::BeamDamageConfig,
    chassis::{BeamCoefficients, ChassisBeamKind, ChassisBeamSpec, ChassisNode},
    deformable_vehicle::{StructureSpec, SuspensionMount},
    static_collision::ContactFeature,
    vehicle_assembly::{VehicleSpec, WHEEL_NAMES},
};

#[derive(Clone, Copy)]
enum NodeOwner {
    Chassis(usize),
    Upright(usize),
}

#[derive(Clone)]
pub struct SourceIndices {
    pub nodes: Vec<usize>,
    pub beams: Vec<usize>,
    pub panels: Vec<usize>,
}

impl SourceIndices {
    pub fn feature(&self, feature: ContactFeature) -> String {
        match feature {
            ContactFeature::Node(i) => format!("chassis node {}", self.nodes[i]),
            ContactFeature::Panel(i) => format!("body panel {}", self.panels[i]),
            ContactFeature::PanelEdge { panel, edge } => {
                format!("body panel {} edge {edge}", self.panels[panel])
            }
        }
    }
}

#[derive(Default)]
pub struct ConversionCounts {
    pub upright_internal: [usize; 4],
    pub suspension_links: [usize; 4],
    pub cross_upright: usize,
    pub unavailable_beams: Vec<usize>,
    pub unavailable_panels: Vec<usize>,
    pub upright_contact_nodes: usize,
    pub support_extension_limits: usize,
    pub ground_exclusions: usize,
}

pub struct ImportedStructure {
    pub structure: StructureSpec,
    pub specification: VehicleSpec,
    pub source: SourceIndices,
    pub counts: ConversionCounts,
    pub mounts: [[usize; 3]; 4],
}

pub fn assemble(
    asset: &VehicleAsset,
    wheels: [(usize, WheelPhysics); 4],
    mut specification: VehicleSpec,
) -> Result<ImportedStructure, String> {
    asset.validate().map_err(|error| error.to_string())?;
    let authored = asset
        .physics
        .structure
        .as_ref()
        .ok_or("missing authored chassis structure")?;
    let mut corner_for_wheel = vec![None; asset.wheels.len()];
    for (corner, (wheel, _)) in wheels.iter().enumerate() {
        corner_for_wheel[*wheel] = Some(corner);
    }
    let mut nodes = Vec::new();
    let mut owners = Vec::new();
    let mut source = SourceIndices {
        nodes: vec![],
        beams: vec![],
        panels: vec![],
    };
    let mut upright_mass = [0.0; 4];
    let mut counts = ConversionCounts::default();
    for (i, (&position, node)) in asset.nodes.iter().zip(&authored.nodes).enumerate() {
        let mass = f64::from(
            node.mass_kg
                .filter(|&mass| mass > 0.0)
                .ok_or_else(|| format!("structural node {i} has no positive authored mass"))?,
        );
        if let Some(wheel) = node.upright {
            let corner = corner_for_wheel[wheel]
                .ok_or_else(|| format!("node {i} belongs to an unassigned upright"))?;
            upright_mass[corner] += mass;
            owners.push(NodeOwner::Upright(corner));
        } else {
            owners.push(NodeOwner::Chassis(nodes.len()));
            source.nodes.push(i);
            nodes.push(ChassisNode {
                mass_kg: mass,
                position_m: position.as_dvec3(),
                velocity_mps: DVec3::ZERO,
            });
            counts.ground_exclusions += usize::from(node.excludes_ground_contact);
        }
    }
    let mass = nodes.iter().map(|node| node.mass_kg).sum::<f64>();
    let moment = nodes
        .iter()
        .map(|node| node.mass_kg * node.position_m)
        .sum::<DVec3>();
    if mass <= 0.0 || (mass - f64::from(specification.chassis.mass_kg)).abs() > 0.002 {
        return Err(format!(
            "sprung node mass {mass:.6} kg disagrees with imported chassis mass {}",
            specification.chassis.mass_kg
        ));
    }
    specification.chassis.mass_kg = mass as f32;
    specification.chassis.centre_of_mass_m = (moment / mass).as_vec3();
    let mut mounts = [SuspensionMount {
        nodes: [0; 3],
        offset_m: DVec3::ZERO,
    }; 4];
    let mut mount_sources = [[0; 3]; 4];
    let mut beams = Vec::new();
    for (i, (&edge, authored_beam)) in asset.edges.iter().zip(&authored.beams).enumerate() {
        let Some(beam) = authored_beam else {
            counts.unavailable_beams.push(i);
            continue;
        };
        let endpoints = match edge.map(|id| owners[id.0]) {
            [NodeOwner::Chassis(a), NodeOwner::Chassis(b)] => [a, b],
            [NodeOwner::Upright(a), NodeOwner::Upright(b)] if a == b => {
                counts.upright_internal[a] += 1;
                continue;
            }
            [NodeOwner::Upright(_), NodeOwner::Upright(_)] => {
                counts.cross_upright += 1;
                continue;
            }
            [NodeOwner::Upright(i), _] | [_, NodeOwner::Upright(i)] => {
                counts.suspension_links[i] += 1;
                continue;
            }
        };
        let [a, b] = endpoints.map(|i| nodes[i].position_m);
        let length = a.distance(b);
        let stiffness = f64::from(beam.stiffness_n_per_m);
        if length == 0.0 || stiffness == 0.0 || beam.deformation_threshold_n == 0.0 {
            counts.unavailable_beams.push(i);
            continue;
        }
        let strain = f64::from(beam.deformation_threshold_n) / (stiffness * length);
        let material = BeamDamageConfig::yield_only(strain, strain)
            .map_err(|error| format!("source beam {i}: {error}"))?;
        let kind = match beam.kind {
            StructuralBeamKind::Bilateral => ChassisBeamKind::Bilateral,
            StructuralBeamKind::CompressionSupport {
                extension_limit_ratio,
            } => {
                counts.support_extension_limits += usize::from(extension_limit_ratio.is_some());
                ChassisBeamKind::CompressionSupport
            }
        };
        source.beams.push(i);
        beams.push(ChassisBeamSpec {
            endpoints,
            coefficients: BeamCoefficients {
                stiffness,
                damping: f64::from(beam.damping_ns_per_m),
            },
            kind,
            material: Some(material),
        });
    }
    let chassis_node = |id: NodeId| match owners[id.0] {
        NodeOwner::Chassis(i) => Some(i),
        NodeOwner::Upright(_) => None,
    };
    for (corner, (_, detail)) in wheels.into_iter().enumerate() {
        let name = WHEEL_NAMES[corner];
        let wheel = specification.wheels[corner]
            .as_mut()
            .expect("drive validates four wheels");
        if (f64::from(wheel.upright_mass_kg) - upright_mass[corner]).abs() > 0.001 {
            return Err(format!(
                "{name}: upright mass differs from the sum of its authored nodes"
            ));
        }
        let attachment = detail
            .attachment
            .ok_or_else(|| format!("{name}: no unique authored shock attachment"))?;
        for node in [
            attachment.axle_nodes[0],
            attachment.axle_nodes[1],
            attachment.lower_node,
        ] {
            if !matches!(owners[node.0], NodeOwner::Upright(i) if i == corner) {
                return Err(format!(
                    "{name}: axle or lower shock node {} belongs to a different body",
                    node.0
                ));
            }
        }
        let upper = chassis_node(attachment.upper_node).ok_or_else(|| {
            format!(
                "{name}: upper shock node {} is on an upright",
                attachment.upper_node.0
            )
        })?;
        let mount = mount_nodes(upper, &nodes, &beams).ok_or_else(|| {
            format!(
                "{name}: no non-collinear sprung neighbours for upper shock node {}",
                attachment.upper_node.0
            )
        })?;
        mount_sources[corner] = mount.map(|i| source.nodes[i]);
        mounts[corner] = SuspensionMount {
            nodes: mount,
            offset_m: (wheel.centre_m + bevy::prelude::Vec3::Y * wheel.initial_length_m).as_dvec3()
                - nodes[upper].position_m,
        };
    }
    let mut collision_panels = Vec::new();
    for (i, &panel) in authored.collision_triangles.iter().enumerate() {
        if let [Some(a), Some(b), Some(c)] = panel.map(chassis_node) {
            let origin = nodes[a].position_m;
            if (nodes[b].position_m - origin)
                .cross(nodes[c].position_m - origin)
                .length_squared()
                > 0.0
            {
                source.panels.push(i);
                collision_panels.push([a, b, c]);
                continue;
            }
        }
        counts.unavailable_panels.push(i);
    }
    let collision_nodes: Vec<_> = authored
        .contact_nodes
        .iter()
        .copied()
        .filter_map(chassis_node)
        .collect();
    counts.upright_contact_nodes = authored.contact_nodes.len() - collision_nodes.len();
    if beams.is_empty() || collision_panels.is_empty() {
        return Err("the imported structure needs sprung beams and body collision panels".into());
    }
    Ok(ImportedStructure {
        structure: StructureSpec {
            nodes,
            beams,
            collision_nodes,
            collision_panels,
            mounts,
        },
        specification,
        source,
        counts,
        mounts: mount_sources,
    })
}

fn mount_nodes(
    upper: usize,
    nodes: &[ChassisNode],
    beams: &[ChassisBeamSpec],
) -> Option<[usize; 3]> {
    let mut neighbours = Vec::new();
    for beam in beams {
        if beam.kind != ChassisBeamKind::Bilateral {
            continue;
        }
        let [a, b] = beam.endpoints;
        if a == upper {
            neighbours.push(b);
        } else if b == upper {
            neighbours.push(a);
        }
    }
    neighbours.sort_unstable();
    neighbours.dedup();
    let origin = nodes[upper].position_m;
    let mut best = None;
    let mut largest = 0.0;
    for (i, &a) in neighbours.iter().enumerate() {
        for &b in &neighbours[i + 1..] {
            let x = nodes[a].position_m - origin;
            let y = nodes[b].position_m - origin;
            let area = x.cross(y).length();
            if area > 1e-3 * x.length() * y.length() && area > largest {
                largest = area;
                best = Some([upper, a, b]);
            }
        }
    }
    best
}

/// Use a padded explicit spring/damper stability bound for the imported graph.
pub(super) fn substeps(
    structure: &StructureSpec,
    ticks_per_second: u32,
    minimum: u32,
) -> Result<u32, String> {
    let mut sums = vec![[0.0; 2]; structure.nodes.len()];
    for beam in &structure.beams {
        for i in beam.endpoints {
            sums[i][0] += beam.coefficients.stiffness;
            sums[i][1] += beam.coefficients.damping;
        }
    }
    let [stiffness, damping] = sums
        .into_iter()
        .zip(&structure.nodes)
        .fold([0.0_f64; 2], |limits, (sums, node)| {
            std::array::from_fn(|i| limits[i].max(2.0 * sums[i] / node.mass_kg))
        });
    // For a scalar mode, semi-implicit Euler needs k*dt² + 2*c*dt < 4.
    // Use half the limiting dt and round up to a whole number of substeps.
    let rate = damping + damping.hypot(2.0 * stiffness.sqrt());
    let substeps = (rate / f64::from(ticks_per_second)).ceil();
    if !substeps.is_finite() || substeps > f64::from(u32::MAX / ticks_per_second) {
        return Err(
            "authored beam coefficients and masses require an unrepresentable solver rate".into(),
        );
    }
    Ok((substeps as u32).max(minimum))
}

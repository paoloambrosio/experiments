use std::collections::{HashMap, HashSet};

use crate::{NodeId, StructuralBeam, StructuralBeamKind};

use super::{node_number, number};

const DEFAULT_BEAM_VALUES: [f32; 4] = [9_000_000.0, 12_000.0, 400_000.0, 1_000_000.0];

pub(super) struct BeamDefaults {
    values: [f32; 4],
    scales: [f32; 4],
    plasticity: f32,
    plasticity_authored: bool,
    advanced: bool,
}

impl Default for BeamDefaults {
    fn default() -> Self {
        Self {
            values: DEFAULT_BEAM_VALUES,
            scales: [1.0; 4],
            plasticity: 0.0,
            plasticity_authored: false,
            advanced: false,
        }
    }
}

impl BeamDefaults {
    pub(super) fn read(&mut self, fields: &[&str], advanced: bool) -> Result<(), String> {
        if fields.is_empty() {
            return Err("expected beam stiffness".into());
        }
        for ((value, field), default) in self.values.iter_mut().zip(fields).zip(DEFAULT_BEAM_VALUES)
        {
            let parsed = number(field)?;
            *value = if parsed < 0.0 { default } else { parsed };
        }
        if let Some(diameter) = fields.get(4) {
            number(diameter)?;
        }
        if let Some(field) = fields.get(6) {
            let plasticity = number(field)?;
            self.plasticity = plasticity.max(0.0);
            self.plasticity_authored |= plasticity >= 0.0;
        }
        self.advanced = advanced;
        Ok(())
    }

    pub(super) fn read_scale(&mut self, fields: &[&str]) -> Result<(), String> {
        let [spring, damper, deformation, breaking] = fields else {
            return Err("expected four beam scale factors".into());
        };
        let scales = [
            number(spring)?,
            number(damper)?,
            number(deformation)?,
            number(breaking)?,
        ];
        if scales.iter().any(|&scale| scale < 0.0) {
            return Err("beam scale factors must be nonnegative".into());
        }
        self.scales = scales;
        Ok(())
    }

    pub(super) fn beam(&self, fields: &[&str]) -> Result<Result<StructuralBeam, String>, String> {
        let flags = fields.first().copied().unwrap_or("");
        if flags.chars().any(|flag| !matches!(flag, 'n' | 'i' | 's')) {
            return Ok(Err(format!("unsupported beam options {flags:?}")));
        }
        if self.plasticity > 1.0 {
            return Ok(Err(format!(
                "unsupported beam plasticity {}",
                self.plasticity
            )));
        }
        let kind = if flags.contains('s') {
            let limit = fields.get(1).map(|value| number(value)).transpose()?;
            if fields.len() > 2 || limit.is_some_and(|value| value <= 0.0) {
                return Err("expected an optional positive support beam extension limit".into());
            }
            StructuralBeamKind::CompressionSupport {
                extension_limit_ratio: limit,
            }
        } else {
            if fields.len() > 1 {
                return Err("unexpected fields after beam options".into());
            }
            StructuralBeamKind::Bilateral
        };

        let [stiffness, damping, mut deformation, breaking] = self.values;
        // Legacy defaults clamp the yield threshold before scaling. Authored
        // advanced defaults can get below the creak floor by specifying plasticity.
        if !self.advanced {
            deformation = deformation.max(DEFAULT_BEAM_VALUES[2]);
        }
        if !self.plasticity_authored {
            deformation = deformation.max(100_000.0);
        }
        let [spring_scale, damp_scale, deform_scale, break_scale] = self.scales;
        let effective = [
            stiffness * spring_scale,
            damping * damp_scale,
            deformation * deform_scale,
            breaking * break_scale,
        ];
        if effective.iter().any(|value| !value.is_finite()) {
            return Err("scaled beam properties must be finite".into());
        }
        let [
            stiffness_n_per_m,
            damping_ns_per_m,
            deformation_threshold_n,
            breaking_threshold_n,
        ] = effective;
        Ok(Ok(StructuralBeam {
            kind,
            stiffness_n_per_m,
            damping_ns_per_m,
            deformation_threshold_n,
            breaking_threshold_n,
            plasticity: self.plasticity,
        }))
    }

    pub(super) fn passive_command(
        &self,
        flags: &str,
    ) -> Result<Result<StructuralBeam, String>, String> {
        if flags.contains('r') {
            return Ok(Err("unsupported tension-only command beam".into()));
        }
        if flags
            .chars()
            .any(|flag| !matches!(flag, 'n' | 'i' | 'c' | 'f' | 'p' | 'o'))
        {
            return Ok(Err(format!("unsupported command beam options {flags:?}")));
        }
        // Without a command input the hydraulic link keeps its spawn length.
        // It still carries axial load using the authored beam defaults.
        self.beam(&[])
    }
}

pub(super) fn influence_nodes(
    fields: &[&str],
    nodes: &HashMap<u32, NodeId>,
) -> Result<Vec<NodeId>, String> {
    if fields.is_empty() {
        return Err("expected at least one influence node or range".into());
    }
    let mut seen = HashSet::new();
    let mut influences = Vec::new();
    for field in fields {
        let (first, last) = match field.split_once('-') {
            Some((first, last)) => (node_number(first)?, node_number(last)?),
            None => {
                let id = node_number(field)?;
                (id, id)
            }
        };
        if first > last {
            return Err(format!("descending node range {field:?}"));
        }
        for id in first..=last {
            let node = *nodes.get(&id).ok_or_else(|| format!("unknown node {id}"))?;
            if seen.insert(node) {
                influences.push(node);
            }
        }
    }
    Ok(influences)
}

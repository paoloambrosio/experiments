//! Effective sidewall opening areas and isothermal air exchange with the atmosphere.

use std::collections::BTreeMap;

use bevy_math::DVec3;

use crate::{beam_damage::BeamDamage, elastic_tyre::ElasticTyreError, tyre::TyreTopology};

/// Fixed air temperature used to convert isothermal gas charge to mass, in kelvins.
pub const AIR_TEMPERATURE_K: f64 = 293.15;
/// Specific gas constant of dry air, in J/(kg·K).
pub const AIR_GAS_CONSTANT: f64 = 287.05;
pub(crate) const AIR_RT: f64 = AIR_GAS_CONSTANT * AIR_TEMPERATURE_K;
const AIR_GAMMA: f64 = 1.4;

#[derive(Debug, Clone, Copy)]
pub struct PunctureConfig {
    opening_fraction: f64,
    discharge_coefficient: f64,
}

impl PunctureConfig {
    /// Both arguments are dimensionless and in (0, 1]. The opening fraction scales
    /// each panel's original area to its maximum effective opening.
    pub fn new(
        opening_fraction: f64,
        discharge_coefficient: f64,
    ) -> Result<Self, ElasticTyreError> {
        if !(0.0 < opening_fraction
            && opening_fraction <= 1.0
            && 0.0 < discharge_coefficient
            && discharge_coefficient <= 1.0)
        {
            return Err(ElasticTyreError::InvalidPuncture);
        }
        Ok(Self {
            opening_fraction,
            discharge_coefficient,
        })
    }

    /// Instantaneous flow in kg/s, positive out of the tyre. Area is in m² and
    /// both pressures are absolute Pa. Uses the choked limit at low pressure ratios.
    pub fn outward_mass_flow_kg_s(
        self,
        opening_area_m2: f64,
        chamber_pa: f64,
        ambient_pa: f64,
    ) -> Result<f64, ElasticTyreError> {
        if !opening_area_m2.is_finite() || opening_area_m2 < 0.0 {
            return Err(ElasticTyreError::InvalidPuncture);
        }
        if ![chamber_pa, ambient_pa]
            .iter()
            .all(|p| p.is_finite() && *p > 0.0)
        {
            return Err(ElasticTyreError::InvalidPressure);
        }
        if opening_area_m2 == 0.0 || chamber_pa == ambient_pa {
            return Ok(0.0);
        }
        let upstream = chamber_pa.max(ambient_pa);
        let downstream = chamber_pa.min(ambient_pa);
        let critical_log_ratio = (2.0 / (AIR_GAMMA + 1.0)).ln() * AIR_GAMMA / (AIR_GAMMA - 1.0);
        let log_ratio = ((downstream - upstream) / upstream)
            .ln_1p()
            .max(critical_log_ratio);
        // The exp_m1 form retains the pressure difference near equalization.
        let pressure_term = (2.0 / AIR_GAMMA * log_ratio).exp()
            * (-((AIR_GAMMA - 1.0) / AIR_GAMMA * log_ratio).exp_m1());
        let magnitude = self.discharge_coefficient
            * opening_area_m2
            * (2.0 * AIR_GAMMA / (AIR_RT * (AIR_GAMMA - 1.0)) * pressure_term).sqrt()
            * upstream;
        if !magnitude.is_finite() {
            return Err(ElasticTyreError::InvalidPressure);
        }
        Ok(magnitude.copysign(chamber_pa - ambient_pa))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sidewall {
    /// The negative-X side of the tyre in its local frame.
    Left,
    /// The positive-X side of the tyre in its local frame.
    Right,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SidewallPanelId {
    pub side: Sidewall,
    /// Angular segment index of the first edge, in topology order.
    pub segment: usize,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TornPanel {
    pub id: SidewallPanelId,
    /// Panel corners in perimeter order, indexing the tyre node array.
    pub nodes: [usize; 4],
    /// Persistent effective airflow area in m².
    pub opening_area_m2: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PunctureReading {
    pub total_open_area_m2: f64,
    /// Actual mass exchanged per second in the last physics substep, positive outward.
    pub outward_mass_flow_kg_s: f64,
}

#[derive(Debug, Clone)]
struct SidewallPanel {
    id: SidewallPanelId,
    nodes: [usize; 4],
    /// Boundaries at the two angular edges, then the two diagonals.
    beams: [usize; 4],
    max_opening_m2: f64,
}

#[derive(Debug, Clone)]
pub(crate) struct Puncture {
    pub(crate) config: PunctureConfig,
    panels: Vec<SidewallPanel>,
    pub(crate) torn_panels: Vec<TornPanel>,
    next_torn_panels: Vec<TornPanel>,
    pub(crate) reading: PunctureReading,
}

impl Puncture {
    pub(crate) fn new(config: PunctureConfig, topology: &TyreTopology, rest: &[DVec3]) -> Self {
        let beams: BTreeMap<_, _> = topology
            .beams()
            .iter()
            .enumerate()
            .map(|(index, beam)| {
                let [a, b] = beam.endpoints;
                ((a.min(b), a.max(b)), index)
            })
            .collect();
        let segments = rest.len() / 5;
        let mut panels = Vec::with_capacity(2 * segments);
        for (side, ring) in [(Sidewall::Left, 0), (Sidewall::Right, 3)] {
            for segment in 0..segments {
                let a = ring * segments + segment;
                let b = ring * segments + (segment + 1) % segments;
                let (c, d) = (a + segments, b + segments);
                let triangle_area = |a: usize, b: usize, c: usize| {
                    (rest[b] - rest[a]).cross(rest[c] - rest[a]).length() * 0.5
                };
                // Use the same mirrored triangulation as the pressure mesh.
                let area = match side {
                    Sidewall::Left => triangle_area(a, b, d) + triangle_area(a, d, c),
                    Sidewall::Right => triangle_area(a, b, c) + triangle_area(b, d, c),
                };
                panels.push(SidewallPanel {
                    id: SidewallPanelId { side, segment },
                    nodes: [a, b, d, c],
                    beams: [
                        beams[&(a, c)],
                        beams[&(b, d)],
                        beams[&(a, d)],
                        beams[&(b, c)],
                    ],
                    max_opening_m2: config.opening_fraction * area,
                });
            }
        }
        Self {
            config,
            panels,
            torn_panels: Vec::new(),
            next_torn_panels: Vec::new(),
            reading: PunctureReading {
                total_open_area_m2: 0.0,
                outward_mass_flow_kg_s: 0.0,
            },
        }
    }

    pub(crate) fn stage(&mut self, damage: &[BeamDamage]) -> f64 {
        self.next_torn_panels.clear();
        let mut total = 0.0;
        for panel in &self.panels {
            let [a, b, c, d] = panel.beams.map(|index| u32::from(damage[index].broken()));
            let sixths = a + b + 2 * (c + d);
            if sixths != 0 {
                let area = panel.max_opening_m2 * (f64::from(sixths) / 6.0);
                self.next_torn_panels.push(TornPanel {
                    id: panel.id,
                    nodes: panel.nodes,
                    opening_area_m2: area,
                });
                total += area;
            }
        }
        total
    }

    pub(crate) fn commit(&mut self, area: f64, outward_mass_flow_kg_s: f64) {
        std::mem::swap(&mut self.torn_panels, &mut self.next_torn_panels);
        self.reading = PunctureReading {
            total_open_area_m2: area,
            outward_mass_flow_kg_s,
        };
    }

    pub(crate) fn reset(&mut self) {
        self.next_torn_panels.clear();
        self.commit(0.0, 0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{beam_damage::BeamDamageConfig, tyre::TyreConfig};

    #[test]
    fn panel_weights_match_beam_endpoints_on_both_sides_and_across_the_seam() {
        let topology = TyreTopology::generate(&TyreConfig::default()).unwrap();
        let rest: Vec<_> = topology
            .nodes()
            .iter()
            .map(|node| node.rest_position.as_dvec3())
            .collect();
        let config = PunctureConfig::new(0.001, 0.65).unwrap();
        let mut puncture = Puncture::new(config, &topology, &rest);
        let mut damage = vec![BeamDamage::new(1.0).unwrap(); topology.beams().len()];
        let material = BeamDamageConfig::new(0.02, 0.02, 0.10, 0.10).unwrap();
        let broken = damage[0].at_length(1.15, material).unwrap();
        damage[0] = broken; // Bead hoop is outside the sidewall rule.
        damage[1] = damage[1].at_length(1.04, material).unwrap();
        assert!(damage[1].yielded());
        assert_eq!(puncture.stage(&damage), 0.0);

        for (index, side, corners) in [
            (31, Sidewall::Left, [31, 0, 32, 63]),
            (63, Sidewall::Right, [127, 96, 128, 159]),
        ] {
            let panel = puncture.panels[index].clone();
            assert_eq!(panel.id, SidewallPanelId { side, segment: 31 });
            assert_eq!(panel.nodes, corners);
            let [a, b, d, c] = corners;
            for (beam, mut endpoints) in
                panel
                    .beams
                    .into_iter()
                    .zip([[a, c], [b, d], [a, d], [b, c]])
            {
                let mut actual = topology.beams()[beam].endpoints;
                actual.sort_unstable();
                endpoints.sort_unstable();
                assert_eq!(actual, endpoints);
            }
            damage[panel.beams[1]] = broken;
            let area = puncture.stage(&damage);
            let torn = &puncture.next_torn_panels;
            assert_eq!(torn.len(), 2);
            assert_eq!(torn[0].id, SidewallPanelId { side, segment: 0 });
            assert_eq!(torn[1].id, panel.id);
            assert!((torn[1].opening_area_m2 - panel.max_opening_m2 / 6.0).abs() < 1e-15);

            damage[panel.beams[2]] = broken;
            damage[panel.beams[3]] = broken;
            let grown = puncture.stage(&damage);
            assert!((grown - area - panel.max_opening_m2 * (2.0 / 3.0)).abs() < 1e-15);
            puncture.commit(grown, 0.0);
            assert_eq!(puncture.stage(&damage), grown);
            assert_eq!(puncture.torn_panels, puncture.next_torn_panels);
            for beam in panel.beams {
                damage[beam].reset();
            }
        }

        let [left, right] = [puncture.panels[0].beams[2], puncture.panels[32].beams[3]];
        damage[left] = broken;
        let single = puncture.stage(&damage);
        damage[right] = broken;
        let both = puncture.stage(&damage);
        assert_eq!(puncture.next_torn_panels.len(), 2);
        assert!((both - 2.0 * single).abs() < 1e-15);
    }

    #[test]
    fn orifice_flow_scales_with_area_and_limits_sonic_flow_in_both_directions() {
        let config = PunctureConfig::new(0.001, 0.65).unwrap();
        let upstream = 131_325.0;
        let area = 2.0e-6;
        let flow = |a, inside, outside| config.outward_mass_flow_kg_s(a, inside, outside).unwrap();
        assert_eq!(flow(0.0, upstream, 101_325.0), 0.0);
        assert_eq!(flow(area, upstream, upstream), 0.0);
        let outward = flow(area, upstream, 101_325.0);
        assert!(outward > 0.0);
        assert_eq!(flow(area, 101_325.0, upstream), -outward);
        assert_eq!(flow(2.0 * area, upstream, 101_325.0), 2.0 * outward);
        let critical = (2.0 / (AIR_GAMMA + 1.0)).powf(AIR_GAMMA / (AIR_GAMMA - 1.0));
        let sonic = 0.65
            * area
            * upstream
            * (AIR_GAMMA / AIR_RT).sqrt()
            * (2.0 / (AIR_GAMMA + 1.0)).powf((AIR_GAMMA + 1.0) / (2.0 * (AIR_GAMMA - 1.0)));
        for ratio in [critical, critical - 0.10, critical + 1e-6] {
            assert!((flow(area, upstream, upstream * ratio) / sonic - 1.0).abs() < 1e-10);
        }
        assert!(flow(area, upstream, upstream * (critical + 0.10)) < sonic);
    }
}

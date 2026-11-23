//! Closed pressure surface and isothermal gas state for a tyre chamber.

use bevy_math::DVec3;

use crate::{elastic_tyre::ElasticTyreError, tyre::TyreTopology};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PressureConfig {
    /// Ambient absolute pressure in Pa.
    pub ambient_pressure_pa: f64,
    /// Initial chamber pressure relative to ambient, in Pa.
    pub initial_gauge_pressure_pa: f64,
}

impl Default for PressureConfig {
    fn default() -> Self {
        Self {
            ambient_pressure_pa: 101_325.0,
            initial_gauge_pressure_pa: 0.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PressureTriangle {
    /// Indices into the tyre's node array, wound outward from the chamber.
    pub nodes: [usize; 3],
    /// This triangle closes the chamber between the two bead rings.
    pub rim_closure: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChamberReading {
    /// Current air volume in m³.
    pub volume_m3: f64,
    /// Current chamber pressure relative to ambient, in Pa.
    pub gauge_pressure_pa: f64,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct GasState {
    reading: ChamberReading,
    charge: f64,
}

#[derive(Debug, Clone)]
pub(crate) struct PressureChamber {
    pub(crate) triangles: Vec<PressureTriangle>,
    ambient_pressure_pa: f64,
    initial: GasState,
    pub(crate) current: GasState,
}

impl PressureChamber {
    pub(crate) fn new(
        topology: &TyreTopology,
        positions: &[DVec3],
        config: PressureConfig,
    ) -> Result<Self, ElasticTyreError> {
        let triangles = triangulate(topology);
        let volume = signed_volume(&triangles, positions)?;
        let initial = Self::charge_at(
            volume,
            config.ambient_pressure_pa,
            config.initial_gauge_pressure_pa,
        )?;
        Ok(Self {
            triangles,
            ambient_pressure_pa: config.ambient_pressure_pa,
            initial,
            current: initial,
        })
    }

    fn charge_at(
        volume_m3: f64,
        ambient: f64,
        gauge_pressure_pa: f64,
    ) -> Result<GasState, ElasticTyreError> {
        let absolute = ambient + gauge_pressure_pa;
        let charge = absolute * volume_m3;
        if !ambient.is_finite() || ambient <= 0.0 || !charge.is_finite() || charge <= 0.0 {
            return Err(ElasticTyreError::InvalidPressure);
        }
        Ok(GasState {
            reading: ChamberReading {
                volume_m3,
                gauge_pressure_pa,
            },
            charge,
        })
    }

    pub(crate) fn reading(&self) -> ChamberReading {
        self.current.reading
    }

    pub(crate) fn set_gauge_pressure(
        &mut self,
        gauge_pressure_pa: f64,
    ) -> Result<(), ElasticTyreError> {
        self.current = Self::charge_at(
            self.current.reading.volume_m3,
            self.ambient_pressure_pa,
            gauge_pressure_pa,
        )?;
        Ok(())
    }

    pub(crate) fn reset(&mut self) {
        self.current = self.initial;
    }

    pub(crate) fn at_positions(&self, positions: &[DVec3]) -> Result<GasState, ElasticTyreError> {
        let volume_m3 = signed_volume(&self.triangles, positions)?;
        if volume_m3 == self.current.reading.volume_m3 {
            return Ok(self.current);
        }
        let absolute = self.current.charge / volume_m3;
        if !absolute.is_finite() || absolute <= 0.0 {
            return Err(ElasticTyreError::InvalidPressure);
        }
        Ok(GasState {
            reading: ChamberReading {
                volume_m3,
                gauge_pressure_pa: absolute - self.ambient_pressure_pa,
            },
            charge: self.current.charge,
        })
    }

    pub(crate) fn add_forces(&self, positions: &[DVec3], forces: &mut [DVec3]) {
        let gauge = self.current.reading.gauge_pressure_pa;
        if gauge != 0.0 {
            add_pressure_forces(&self.triangles, gauge, positions, forces);
        }
    }
}

pub(crate) fn triangulate(topology: &TyreTopology) -> Vec<PressureTriangle> {
    // The tyre stores five equal-length rings from left bead to right bead.
    let segments = topology.nodes().len() / 5;
    let mut triangles = Vec::with_capacity(10 * segments);
    for ring in 0..5 {
        let rim_closure = ring == 4;
        for segment in 0..segments {
            let next_segment = (segment + 1) % segments;
            let next_ring = (ring + 1) % 5;
            let (a, b) = (ring * segments + segment, ring * segments + next_segment);
            let (c, d) = (
                next_ring * segments + segment,
                next_ring * segments + next_segment,
            );
            // Reversing the diagonal across the centreline mirrors the two sidewalls.
            let panel = if ring < 2 {
                [[a, b, d], [a, d, c]]
            } else {
                [[a, b, c], [b, d, c]]
            };
            triangles.extend(panel.map(|nodes| PressureTriangle { nodes, rim_closure }));
        }
    }
    triangles
}

fn area_vector([a, b, c]: [DVec3; 3]) -> DVec3 {
    (b - a).cross(c - a) * 0.5
}

fn signed_volume(
    triangles: &[PressureTriangle],
    positions: &[DVec3],
) -> Result<f64, ElasticTyreError> {
    let origin = positions[0];
    let mut sum = 0.0;
    for (index, face) in triangles.iter().enumerate() {
        let vertices = face.nodes.map(|node| positions[node]);
        let area = area_vector(vertices);
        if !area.is_finite() || area == DVec3::ZERO {
            return Err(ElasticTyreError::InvalidPressureTriangle(index));
        }
        sum += (vertices[0] - origin).dot(area);
    }
    let volume = sum / 3.0;
    if !volume.is_finite() || volume <= 0.0 {
        return Err(ElasticTyreError::InvalidChamberVolume);
    }
    Ok(volume)
}

fn add_pressure_forces(
    triangles: &[PressureTriangle],
    gauge_pressure_pa: f64,
    positions: &[DVec3],
    forces: &mut [DVec3],
) {
    for face in triangles {
        let share = area_vector(face.nodes.map(|node| positions[node])) * (gauge_pressure_pa / 3.0);
        for node in face.nodes {
            forces[node] += share;
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{collections::BTreeMap, f64::consts::TAU};

    use super::*;
    use crate::tyre::TyreConfig;

    #[test]
    fn surface_is_closed_and_consistently_wound_at_seam_and_rim() {
        for segments in [3, 32] {
            let topology = TyreTopology::generate(&TyreConfig {
                segments,
                ..TyreConfig::default()
            })
            .unwrap();
            let triangles = triangulate(&topology);
            assert_eq!(triangles.len(), 10 * segments);
            assert_eq!(
                triangles.iter().filter(|face| face.rim_closure).count(),
                2 * segments
            );
            let mut edges = BTreeMap::new();
            for face in triangles {
                let [a, b, c] = face.nodes;
                for (start, end) in [(a, b), (b, c), (c, a)] {
                    let (count, winding) = edges
                        .entry((start.min(end), start.max(end)))
                        .or_insert((0, 0));
                    *count += 1;
                    *winding += if start < end { 1 } else { -1 };
                }
            }
            assert!(edges.into_values().all(|edge| edge == (2, 0)));
        }
    }

    #[test]
    fn volume_matches_polygonal_frusta_and_is_translation_invariant() {
        for segments in [3, 32] {
            let config = TyreConfig {
                segments,
                ..TyreConfig::default()
            };
            let topology = TyreTopology::generate(&config).unwrap();
            let triangles = triangulate(&topology);
            let mut positions: Vec<_> = topology
                .nodes()
                .iter()
                .map(|node| node.rest_position.as_dvec3())
                .collect();
            let volume = signed_volume(&triangles, &positions).unwrap();

            // Integrate regular polygon areas along X. The signed band widths
            // subtract the bead overhangs and the cylinder inside the rim strip.
            let polygon_factor = segments as f64 / 2.0 * (TAU / segments as f64).sin();
            let (bead, shoulder, tread) = (
                f64::from(config.bead_radius),
                f64::from(config.shoulder_radius),
                f64::from(config.tread_radius),
            );
            let (bead_width, shoulder_width) = (
                f64::from(config.bead_width),
                f64::from(config.shoulder_width),
            );
            let integral = |r: f64, s: f64| (r * r + r * s + s * s) / 3.0;
            let expected = polygon_factor
                * ((bead_width - shoulder_width) * integral(bead, shoulder)
                    + shoulder_width * integral(shoulder, tread)
                    - bead_width * bead * bead);
            assert!((volume - expected).abs() < 1e-8);
            positions
                .iter_mut()
                .for_each(|position| *position += DVec3::new(100.0, -50.0, 70.0));
            assert!((signed_volume(&triangles, &positions).unwrap() - volume).abs() < 1e-13);

            let mut inverted = triangles;
            inverted.iter_mut().for_each(|face| face.nodes.swap(0, 1));
            assert_eq!(
                signed_volume(&inverted, &positions),
                Err(ElasticTyreError::InvalidChamberVolume)
            );
        }
    }

    #[test]
    fn complete_surface_balances_uniform_pressure() {
        let topology = TyreTopology::generate(&TyreConfig::default()).unwrap();
        let positions: Vec<_> = topology
            .nodes()
            .iter()
            .map(|node| node.rest_position.as_dvec3())
            .collect();
        let chamber = PressureChamber::new(
            &topology,
            &positions,
            PressureConfig {
                initial_gauge_pressure_pa: 30_000.0,
                ..PressureConfig::default()
            },
        )
        .unwrap();
        let mut forces = vec![DVec3::ZERO; positions.len()];
        chamber.add_forces(&positions, &mut forces);
        assert!(forces.iter().map(|force| force.length()).sum::<f64>() > 1.0);
        assert!(forces.iter().sum::<DVec3>().length() < 1e-9);
        let torque: DVec3 = positions
            .iter()
            .zip(forces)
            .map(|(position, force)| position.cross(force))
            .sum();
        assert!(torque.length() < 1e-9);
    }

    #[test]
    fn triangle_shares_pressure_force_equally() {
        let positions = [DVec3::ZERO, DVec3::X, DVec3::Y];
        let mut forces = [DVec3::ZERO; 3];
        let triangle = PressureTriangle {
            nodes: [0, 1, 2],
            rim_closure: false,
        };
        add_pressure_forces(&[triangle], 12.0, &positions, &mut forces);
        assert_eq!(forces, [DVec3::Z * 2.0; 3]);
    }

    #[test]
    fn compression_from_zero_gauge_preserves_absolute_pressure_times_volume() {
        let topology = TyreTopology::generate(&TyreConfig::default()).unwrap();
        let mut positions: Vec<_> = topology
            .nodes()
            .iter()
            .map(|node| node.rest_position.as_dvec3())
            .collect();
        let config = PressureConfig::default();
        let chamber = PressureChamber::new(&topology, &positions, config).unwrap();
        let initial_volume = chamber.reading().volume_m3;
        let mut forces = vec![DVec3::ZERO; positions.len()];
        chamber.add_forces(&positions, &mut forces);
        assert!(forces.iter().all(|force| *force == DVec3::ZERO));
        positions.iter_mut().for_each(|position| *position *= 0.9);
        let reading = chamber.at_positions(&positions).unwrap().reading;
        assert!(reading.gauge_pressure_pa > 0.0);
        assert!((reading.volume_m3 - initial_volume * 0.9_f64.powi(3)).abs() < 1e-13);
        let expected = config.ambient_pressure_pa * (0.9_f64.powi(-3) - 1.0);
        assert!((reading.gauge_pressure_pa - expected).abs() < 1e-8);
    }
}

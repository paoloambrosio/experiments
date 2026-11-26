//! Rate-independent axial yielding and fracture for node-and-beam structures.

use std::{error::Error, fmt};

/// Positive, dimensionless engineering strain limits measured against the original length.
#[derive(Debug, Clone, Copy)]
pub struct BeamDamageConfig {
    tension_yield: f64,
    compression_yield: f64,
    fracture_strains: Option<[f64; 2]>,
}

#[derive(Debug, Clone, Copy)]
pub struct InvalidBeamDamageConfig;

impl fmt::Display for InvalidBeamDamageConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("beam strain limits must be finite and positive, with fracture above yield and compressive fracture at most one")
    }
}

impl Error for InvalidBeamDamageConfig {}

impl BeamDamageConfig {
    pub fn new(
        tension_yield: f64,
        compression_yield: f64,
        tension_fracture: f64,
        compression_fracture: f64,
    ) -> Result<Self, InvalidBeamDamageConfig> {
        let mut config = Self::yield_only(tension_yield, compression_yield)?;
        if !tension_fracture.is_finite()
            || !compression_fracture.is_finite()
            || tension_fracture <= tension_yield
            || compression_fracture <= compression_yield
            || compression_fracture > 1.0
        {
            return Err(InvalidBeamDamageConfig);
        }
        config.fracture_strains = Some([tension_fracture, compression_fracture]);
        Ok(config)
    }

    /// Allow permanent axial deformation while keeping the beam connected.
    pub fn yield_only(
        tension_yield: f64,
        compression_yield: f64,
    ) -> Result<Self, InvalidBeamDamageConfig> {
        if [tension_yield, compression_yield]
            .into_iter()
            .any(|strain| !strain.is_finite() || strain <= 0.0)
        {
            return Err(InvalidBeamDamageConfig);
        }
        Ok(Self {
            tension_yield,
            compression_yield,
            fracture_strains: None,
        })
    }

    fn fractures(self, strain: f64) -> bool {
        self.fracture_strains
            .is_some_and(|[tension, compression]| strain >= tension || strain <= -compression)
    }
}

/// Material history for one beam. Store this alongside the beam's endpoints and stiffness.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BeamDamage {
    original_length_m: f64,
    stress_free_length_m: f64,
    yielded: bool,
    broken: bool,
}

impl BeamDamage {
    /// Start with an undamaged beam. The original length must be positive and finite.
    pub fn new(original_length_m: f64) -> Option<Self> {
        (original_length_m.is_finite() && original_length_m > 0.0).then_some(Self {
            original_length_m,
            stress_free_length_m: original_length_m,
            yielded: false,
            broken: false,
        })
    }

    pub fn stress_free_length_m(self) -> f64 {
        self.stress_free_length_m
    }

    pub fn permanent_length_change_m(self) -> f64 {
        self.stress_free_length_m - self.original_length_m
    }

    pub fn yielded(self) -> bool {
        self.yielded
    }

    pub fn broken(self) -> bool {
        self.broken
    }

    /// Return the updated material state for a finite, nonnegative beam length in metres.
    /// A caller can stage this value and commit it only when the physics step succeeds.
    pub fn at_length(mut self, length_m: f64, config: BeamDamageConfig) -> Option<Self> {
        if !length_m.is_finite() || length_m < 0.0 {
            return None;
        }
        if self.broken {
            return Some(self);
        }
        let original = self.original_length_m;
        if config.fractures((length_m - original) / original) {
            self.broken = true;
        } else {
            let elastic_change = length_m - self.stress_free_length_m;
            let elastic_limit = elastic_change.clamp(
                -config.compression_yield * original,
                config.tension_yield * original,
            );
            if elastic_change != elastic_limit {
                self.stress_free_length_m = length_m - elastic_limit;
                self.yielded = true;
            }
        }
        Some(self)
    }

    pub(crate) fn reset(&mut self) {
        *self = Self::new(self.original_length_m).expect("validated original length");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn beam_unloads_elastically_and_keeps_plastic_set_in_both_directions() {
        let config = BeamDamageConfig::new(0.05, 0.05, 0.25, 0.25).unwrap();
        let initial = BeamDamage::new(1.0).unwrap();
        let elastic = initial.at_length(1.025, config).unwrap();
        assert_eq!(elastic.at_length(1.0, config), Some(initial));
        for (length, stress_free) in [(1.125, 1.075), (0.875, 0.925)] {
            let loaded = initial.at_length(length, config).unwrap();
            assert!(loaded.yielded());
            assert!((loaded.stress_free_length_m() - stress_free).abs() < 1e-12);
            assert_eq!(loaded.at_length(stress_free, config), Some(loaded));
            let broken = loaded.at_length(1.30, config).unwrap();
            assert!(broken.broken());
            assert_eq!(broken.at_length(1.0, config), Some(broken));
        }
    }
}

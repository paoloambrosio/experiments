//! Fixed-bead inflation used to prepare a sealed tyre before mounting.
use std::{error::Error, fmt};

use bevy_math::DVec3;

use crate::{
    elastic_tyre::{ElasticTyre, ElasticTyreConfig, ElasticTyreError},
    pressure::PressureConfig,
    tread_reinforcement::TreadReinforcementConfig,
    tyre::{TyreConfig, TyreConfigError, TyreTopology},
};

/// The five-ring rest shape and material of one sealed tyre.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TyreSpec {
    pub geometry: TyreConfig,
    pub material: ElasticTyreConfig,
    pub reinforcement: TreadReinforcementConfig,
    /// Ambient absolute pressure in Pa.
    pub ambient_pressure_pa: f64,
    /// Target gauge pressure in Pa. Preparation starts at zero gauge pressure.
    pub gauge_pressure_pa: f64,
}

#[derive(Debug)]
pub enum TyrePreparationError {
    InvalidRate(u32),
    InvalidAmbientPressure,
    InvalidGaugePressure,
    Geometry(TyreConfigError),
    Material(ElasticTyreError),
    Reinforcement(ElasticTyreError),
    Inflation(Box<dyn Error>),
}

impl fmt::Display for TyrePreparationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidRate(rate) => {
                write!(f, "unsupported tyre inflation rate {rate} substeps/s")
            }
            Self::InvalidAmbientPressure => {
                f.write_str("tyre.ambient_pressure_pa must be positive and finite")
            }
            Self::InvalidGaugePressure => f.write_str(
                "tyre.gauge_pressure_pa must be nonnegative with finite absolute pressure",
            ),
            Self::Geometry(error) => write!(f, "tyre.geometry: {error}"),
            Self::Material(error) => write!(f, "tyre.material or chamber: {error}"),
            Self::Reinforcement(error) => write!(f, "tyre.reinforcement: {error}"),
            Self::Inflation(error) => write!(f, "tyre inflation: {error}"),
        }
    }
}

impl Error for TyrePreparationError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Geometry(error) => Some(error),
            Self::Material(error) | Self::Reinforcement(error) => Some(error),
            Self::Inflation(error) => Some(error.as_ref()),
            _ => None,
        }
    }
}

impl TyreSpec {
    /// Inflate at the given substeps per second and return the sealed, fixed-bead tyre.
    pub fn prepare(self, rate: u32) -> Result<ElasticTyre, TyrePreparationError> {
        let (mut tyre, mut fill) = self.begin(rate)?;
        while !fill.advance(&mut tyre)? {}
        Ok(tyre)
    }

    pub(crate) fn begin(
        self,
        rate: u32,
    ) -> Result<(ElasticTyre, TyreInflation), TyrePreparationError> {
        if !SUPPORTED_RATES.contains(&rate) {
            return Err(TyrePreparationError::InvalidRate(rate));
        }
        if !self.ambient_pressure_pa.is_finite() || self.ambient_pressure_pa <= 0.0 {
            return Err(TyrePreparationError::InvalidAmbientPressure);
        }
        if !self.gauge_pressure_pa.is_finite()
            || self.gauge_pressure_pa < 0.0
            || !(self.ambient_pressure_pa + self.gauge_pressure_pa).is_finite()
        {
            return Err(TyrePreparationError::InvalidGaugePressure);
        }
        let topology =
            TyreTopology::generate(&self.geometry).map_err(TyrePreparationError::Geometry)?;
        let forces = vec![DVec3::ZERO; topology.nodes().len()];
        let pressure = PressureConfig {
            ambient_pressure_pa: self.ambient_pressure_pa,
            initial_gauge_pressure_pa: 0.0,
        };
        let tyre = ElasticTyre::new_inflated(topology, self.material, pressure)
            .map_err(TyrePreparationError::Material)?
            .with_tread_reinforcement(self.reinforcement)
            .map_err(TyrePreparationError::Reinforcement)?;
        Ok((
            tyre,
            TyreInflation {
                fill: Inflation::new(rate),
                forces,
                gauge_pressure_pa: self.gauge_pressure_pa,
                sealed: false,
            },
        ))
    }
}

/// Radius in metres along local -Y for a prepared, unloaded upright tyre.
pub fn unloaded_radius(tyre: &ElasticTyre) -> f64 {
    -tyre
        .positions()
        .iter()
        .map(|p| p.y)
        .fold(f64::INFINITY, f64::min)
}

pub(crate) struct TyreInflation {
    fill: Inflation,
    forces: Vec<DVec3>,
    gauge_pressure_pa: f64,
    sealed: bool,
}

impl TyreInflation {
    pub(crate) fn advance(&mut self, tyre: &mut ElasticTyre) -> Result<bool, TyrePreparationError> {
        if !self.sealed {
            self.sealed = self
                .fill
                .advance(tyre, self.gauge_pressure_pa, &self.forces)
                .map_err(TyrePreparationError::Inflation)?;
        }
        Ok(self.sealed)
    }

    #[cfg(feature = "avian")]
    pub(crate) fn is_ready(&self) -> bool {
        self.sealed
    }

    #[cfg(feature = "avian")]
    pub(crate) fn seconds(&self) -> f64 {
        self.fill.seconds()
    }
}

const SUPPORTED_RATES: std::ops::RangeInclusive<u32> = 4..=u32::MAX / 30;

/// Ramp gauge pressure over one second and hold it until all nodes have settled.
pub struct Inflation {
    rate: u32,
    steps: u32,
    slow_steps: u32,
}
impl Inflation {
    /// Solver rate in substeps per second.
    pub fn new(rate: u32) -> Self {
        assert!(
            SUPPORTED_RATES.contains(&rate),
            "unsupported inflation rate"
        );
        Self {
            rate,
            steps: 0,
            slow_steps: 0,
        }
    }
    pub fn seconds(&self) -> f64 {
        f64::from(self.steps) / f64::from(self.rate)
    }
    /// Advance one substep at the target gauge pressure in pascals. Stop calling
    /// when this returns true to seal the gas charge. Forces are per node in newtons.
    pub fn advance(
        &mut self,
        tyre: &mut ElasticTyre,
        target: f64,
        forces: &[DVec3],
    ) -> Result<bool, Box<dyn std::error::Error>> {
        let next = self.steps + 1;
        let fraction = f64::from(next.min(self.rate)) / f64::from(self.rate);
        // The valve regulates pressure during the step. Restore the setpoint at the new
        // volume as well, so the displayed pressure and the final sealed charge agree.
        tyre.set_gauge_pressure(target * fraction)?;
        tyre.step(1.0 / f64::from(self.rate), forces)?;
        tyre.set_gauge_pressure(target * fraction)?;
        self.steps = next;

        // The fixed beads' velocities are zero, so they don't affect this test.
        if next >= self.rate
            && tyre
                .velocities()
                .iter()
                .all(|velocity| velocity.length_squared() < 0.0001_f64.powi(2))
        {
            self.slow_steps += 1;
        } else {
            self.slow_steps = 0;
        }
        if self.slow_steps >= self.rate / 4 {
            let actual = tyre
                .chamber_reading()
                .expect("sealed chamber")
                .gauge_pressure_pa;
            if (actual - target).abs() > 100.0 {
                return Err(format!(
                    "sealed pressure {actual:.0} Pa missed the {target:.0} Pa target"
                )
                .into());
            }
            return Ok(true);
        } else if next >= 30 * self.rate {
            return Err(format!(
                "tyre at {:.0} kPa failed to settle within 30 s",
                target / 1000.0
            )
            .into());
        }
        Ok(false)
    }
}

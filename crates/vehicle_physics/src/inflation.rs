//! Fixed-bead inflation used to prepare a sealed tyre before mounting.
use crate::elastic_tyre::ElasticTyre;
use bevy_math::DVec3;

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
            (4..=u32::MAX / 30).contains(&rate),
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

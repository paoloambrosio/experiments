use bevy::math::DVec3;
use vehicle_physics::{
    elastic_tyre::{ElasticTyre, ElasticTyreConfig, ElasticTyreError},
    pressure::PressureConfig,
    tread_reinforcement::TreadReinforcementConfig,
    tyre::{RingRole, TyreConfig, TyreTopology},
};

pub const REFERENCE_SUBSTEPS_PER_SECOND: u32 = 960;
pub const RADIAL_SUBSTEPS_PER_SECOND: u32 = 3_840;
pub const TICKS_PER_SECOND: u32 = 60;
pub const LOAD_NEWTONS: f64 = 3.0;
pub const PRESETS_PA: [f64; 4] = [0.0, 10_000.0, 30_000.0, 60_000.0];

pub struct Experiment {
    pub tyre: ElasticTyre,
    pub segments: usize,
    pub patch: [usize; 3],
    substeps_per_second: u32,
    completed_steps: u32,
    peak_inward_mm: f64,
    external_forces: Vec<DVec3>,
    baseline: Option<ElasticTyre>,
    target_pressure_pa: Option<f64>,
    preparation_steps: u32,
    slow_steps: u32,
}

impl Experiment {
    pub fn new(substeps_per_second: u32) -> Self {
        Self::build(
            substeps_per_second,
            None,
            None,
            ElasticTyreConfig::default(),
        )
    }

    pub fn inflated(substeps_per_second: u32, target_pa: f64) -> Self {
        Self::build(
            substeps_per_second,
            Some(target_pa),
            None,
            ElasticTyreConfig::default(),
        )
    }

    pub fn radial(substeps_per_second: u32, target_pa: f64) -> Self {
        Self::radial_with_config(substeps_per_second, target_pa, ElasticTyreConfig::default())
    }

    pub fn radial_with_config(
        substeps_per_second: u32,
        target_pa: f64,
        elastic_config: ElasticTyreConfig,
    ) -> Self {
        Self::reinforced(
            substeps_per_second,
            target_pa,
            elastic_config,
            TreadReinforcementConfig::default(),
        )
    }

    pub fn reinforced(
        substeps_per_second: u32,
        target_pa: f64,
        elastic_config: ElasticTyreConfig,
        reinforcement: TreadReinforcementConfig,
    ) -> Self {
        Self::build(
            substeps_per_second,
            Some(target_pa),
            Some(reinforcement),
            elastic_config,
        )
    }

    fn build(
        substeps_per_second: u32,
        target_pressure_pa: Option<f64>,
        reinforcement: Option<TreadReinforcementConfig>,
        elastic_config: ElasticTyreConfig,
    ) -> Self {
        let config = TyreConfig::default();
        let topology = TyreTopology::generate(&config).expect("default tyre geometry");
        let tread_start = topology
            .nodes()
            .iter()
            .position(|node| node.ring == RingRole::TreadCentre)
            .expect("tread centre ring");
        let patch = [
            tread_start + config.segments - 1,
            tread_start,
            tread_start + 1,
        ];
        let external_forces = vec![DVec3::ZERO; topology.nodes().len()];
        let tyre = if target_pressure_pa.is_some() {
            ElasticTyre::new_inflated(topology, elastic_config, PressureConfig::default())
        } else {
            ElasticTyre::new(topology, elastic_config)
        }
        .expect("default elastic tyre");
        let tyre = if let Some(reinforcement) = reinforcement {
            tyre.with_tread_reinforcement(reinforcement)
                .expect("default tread reinforcement")
        } else {
            tyre
        };
        let baseline = if target_pressure_pa.unwrap_or(0.0) == 0.0 {
            Some(tyre.clone())
        } else {
            None
        };
        Self {
            tyre,
            segments: config.segments,
            patch,
            substeps_per_second,
            completed_steps: 0,
            peak_inward_mm: 0.0,
            external_forces,
            baseline,
            target_pressure_pa,
            preparation_steps: 0,
            slow_steps: 0,
        }
    }

    pub fn reset(&mut self) {
        self.tyre.clone_from(
            self.baseline
                .as_ref()
                .expect("prepare the tyre before pressing"),
        );
        self.completed_steps = 0;
        self.peak_inward_mm = 0.0;
    }

    pub fn is_ready(&self) -> bool {
        self.baseline.is_some()
    }

    pub fn substeps_per_tick(&self) -> u32 {
        self.substeps_per_second / TICKS_PER_SECOND
    }

    pub fn preparation_seconds(&self) -> f64 {
        f64::from(self.preparation_steps) / f64::from(self.substeps_per_second)
    }

    pub fn patch_reference_positions(&self) -> Option<[DVec3; 3]> {
        self.baseline
            .as_ref()
            .map(|baseline| self.patch.map(|node| baseline.positions()[node]))
    }

    pub fn prepare_step(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let target = self.target_pressure_pa.expect("positive-pressure preset");
        let next = self.preparation_steps + 1;
        let fraction =
            f64::from(next.min(self.substeps_per_second)) / f64::from(self.substeps_per_second);
        // The valve regulates pressure during the step. Restore the setpoint at the new
        // volume as well, so the displayed pressure and the final sealed charge agree.
        self.tyre.set_gauge_pressure(target * fraction)?;
        self.tyre.step(
            1.0 / f64::from(self.substeps_per_second),
            &self.external_forces,
        )?;
        self.tyre.set_gauge_pressure(target * fraction)?;
        self.preparation_steps = next;

        // The fixed beads' velocities are zero, so they don't affect this test.
        if next >= self.substeps_per_second
            && self
                .tyre
                .velocities()
                .iter()
                .all(|velocity| velocity.length_squared() < 0.0001_f64.powi(2))
        {
            self.slow_steps += 1;
        } else {
            self.slow_steps = 0;
        }
        if self.slow_steps >= self.substeps_per_second / 4 {
            let actual = self
                .tyre
                .chamber_reading()
                .expect("sealed chamber")
                .gauge_pressure_pa;
            if (actual - target).abs() > 100.0 {
                return Err(format!(
                    "sealed pressure {actual:.0} Pa missed the {target:.0} Pa target"
                )
                .into());
            }
            self.baseline = Some(self.tyre.clone());
        } else if next >= 30 * self.substeps_per_second {
            return Err(format!(
                "tyre at {:.0} kPa failed to settle within 30 s",
                target / 1000.0
            )
            .into());
        }
        Ok(())
    }

    pub fn replay_load(&self) -> bool {
        self.completed_steps < self.substeps_per_second / 2
    }

    pub fn replay_finished(&self) -> bool {
        self.completed_steps >= 2 * self.substeps_per_second
    }

    pub fn elapsed_seconds(&self) -> f64 {
        f64::from(self.completed_steps) / f64::from(self.substeps_per_second)
    }

    pub fn advance(&mut self, load: bool) -> Result<(), ElasticTyreError> {
        let force = if load {
            -DVec3::Y * (LOAD_NEWTONS / 3.0)
        } else {
            DVec3::ZERO
        };
        for node in self.patch {
            self.external_forces[node] = force;
        }
        self.tyre.step(
            1.0 / f64::from(self.substeps_per_second),
            &self.external_forces,
        )?;
        self.completed_steps = self.completed_steps.saturating_add(1);
        if load {
            self.peak_inward_mm = self.peak_inward_mm.max(self.deflection_mm());
        }
        Ok(())
    }

    pub fn peak_inward_mm(&self) -> f64 {
        self.peak_inward_mm
    }

    pub fn deflection_mm(&self) -> f64 {
        let Some(reference) = self.patch_reference_positions() else {
            return 0.0;
        };
        self.patch
            .into_iter()
            .zip(reference)
            .map(|(index, reference)| reference.y - self.tyre.positions()[index].y)
            .sum::<f64>()
            * (1_000.0 / 3.0)
    }
}

pub(crate) struct Measurement {
    samples_mm: Vec<f64>,
    pub(crate) peak_inward_mm: f64,
    pub(crate) release_mm: f64,
    outward_rebound_mm: f64,
    pub(crate) residual_mm: f64,
}

pub(crate) fn measure(experiment: &mut Experiment) -> Result<Measurement, ElasticTyreError> {
    experiment.reset();
    let mut samples_mm = vec![0.0];
    let mut release_mm = 0.0;
    let mut outward_rebound_mm: f64 = 0.0;
    let sample_period = experiment.substeps_per_tick();
    while !experiment.replay_finished() {
        let load = experiment.replay_load();
        experiment.advance(load)?;
        let deflection_mm = experiment.deflection_mm();
        if load {
            release_mm = deflection_mm;
        } else {
            outward_rebound_mm = outward_rebound_mm.max(-deflection_mm);
        }
        if experiment.completed_steps.is_multiple_of(sample_period) {
            samples_mm.push(deflection_mm);
        }
    }
    Ok(Measurement {
        samples_mm,
        peak_inward_mm: experiment.peak_inward_mm(),
        release_mm,
        outward_rebound_mm,
        residual_mm: experiment.deflection_mm(),
    })
}

fn max_sample_difference(a: &Measurement, b: &Measurement) -> f64 {
    a.samples_mm
        .iter()
        .zip(&b.samples_mm)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f64::max)
}

pub fn report() -> Result<(), Box<dyn std::error::Error>> {
    let rates = [
        REFERENCE_SUBSTEPS_PER_SECOND,
        2 * REFERENCE_SUBSTEPS_PER_SECOND,
        4 * REFERENCE_SUBSTEPS_PER_SECOND,
    ];
    let measurements = rates.map(|rate| measure(&mut Experiment::new(rate)));
    let [coarse, medium, fine] = measurements;
    let measurements = [coarse?, medium?, fine?];
    println!("Elastic carcass | 3 N inward for 0.5 s | observe until 2 s");
    println!("substeps/s    peak inward mm    at release mm    outward rebound mm    residual mm");
    for (rate, result) in rates.into_iter().zip(&measurements) {
        println!(
            "{rate:>10}    {:>14.4}    {:>13.4}    {:>18.4}    {:>11.4}",
            result.peak_inward_mm, result.release_mm, result.outward_rebound_mm, result.residual_mm,
        );
    }
    let coarse_difference = max_sample_difference(&measurements[0], &measurements[1]);
    let fine_difference = max_sample_difference(&measurements[1], &measurements[2]);
    println!("Max difference at matching 60 Hz samples");
    println!("  {} / {}   {coarse_difference:.4} mm", rates[0], rates[1]);
    println!("  {} / {}   {fine_difference:.4} mm", rates[1], rates[2]);
    if fine_difference >= coarse_difference || fine_difference >= 0.10 {
        return Err(
            "refining the timestep must reduce the trace difference to below 0.10 mm".into(),
        );
    }
    for (rate, result) in rates.into_iter().zip(&measurements) {
        if result.release_mm <= 0.0
            || result.outward_rebound_mm <= 0.0
            || result.residual_mm.abs() > 0.25
        {
            return Err(format!("{rate} substeps/s failed the rebound or residual check").into());
        }
    }
    println!("Rebound, residual, and timestep checks passed.");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn replay_meets_acceptance_checks() {
        report().unwrap();
    }

    #[test]
    fn restoring_sealed_baseline_repeats_replay() {
        let mut experiment = Experiment::inflated(REFERENCE_SUBSTEPS_PER_SECOND, 10_000.0);
        while !experiment.is_ready() {
            experiment.prepare_step().unwrap();
        }
        let reading = experiment.tyre.chamber_reading();
        let positions = experiment.tyre.positions().to_vec();
        let velocities = experiment.tyre.velocities().to_vec();
        let first = measure(&mut experiment).unwrap();
        let second = measure(&mut experiment).unwrap();
        assert_eq!(first.samples_mm, second.samples_mm);
        assert_eq!(first.peak_inward_mm, second.peak_inward_mm);
        experiment.reset();
        assert_eq!(experiment.tyre.chamber_reading(), reading);
        assert_eq!(experiment.tyre.positions(), positions);
        assert_eq!(experiment.tyre.velocities(), velocities);
        assert_eq!(experiment.deflection_mm(), 0.0);
    }

    #[test]
    fn fill_ramps_then_seals_at_target() {
        let mut experiment = Experiment::inflated(REFERENCE_SUBSTEPS_PER_SECOND, 30_000.0);
        let start = experiment.tyre.chamber_reading().unwrap();
        assert_eq!(start.gauge_pressure_pa, 0.0);
        while !experiment.is_ready() {
            let reading = experiment.tyre.chamber_reading().unwrap();
            let seconds = experiment.preparation_seconds();
            assert!((reading.gauge_pressure_pa - 30_000.0 * seconds.min(1.0)).abs() < 1e-9);
            if experiment.preparation_steps.is_multiple_of(32) {
                println!(
                    "TRACE {:.6} {:.6} {:.6}",
                    seconds,
                    reading.gauge_pressure_pa / 1000.0,
                    reading.volume_m3 * 1000.0
                );
            }
            experiment.prepare_step().unwrap();
        }
        let reading = experiment.tyre.chamber_reading().unwrap();
        assert!((reading.gauge_pressure_pa - 30_000.0).abs() <= 100.0);
        assert!(reading.volume_m3 > start.volume_m3);
        println!(
            "TRACE {:.6} {:.6} {:.6}",
            experiment.preparation_seconds(),
            reading.gauge_pressure_pa / 1000.0,
            reading.volume_m3 * 1000.0
        );
    }
}

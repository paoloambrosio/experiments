use bevy::math::DVec3;
use vehicle_physics::{
    elastic_tyre::{ElasticTyre, ElasticTyreConfig, ElasticTyreError},
    tyre::{RingRole, TyreConfig, TyreTopology},
};

pub const LAB_SUBSTEPS_PER_SECOND: u32 = 960;
pub const TICKS_PER_SECOND: u32 = 60;
pub const LOAD_NEWTONS: f64 = 3.0;

pub struct Experiment {
    pub tyre: ElasticTyre,
    pub segments: usize,
    pub patch: [usize; 3],
    substeps_per_second: u32,
    completed_steps: u32,
    external_forces: Vec<DVec3>,
}

impl Experiment {
    pub fn new(substeps_per_second: u32) -> Self {
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
        Self {
            tyre: ElasticTyre::new(topology, ElasticTyreConfig::default())
                .expect("default elastic tyre"),
            segments: config.segments,
            patch,
            substeps_per_second,
            completed_steps: 0,
            external_forces,
        }
    }

    pub fn reset(&mut self) {
        self.tyre.reset();
        self.completed_steps = 0;
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
        Ok(())
    }

    pub fn deflection_mm(&self) -> f64 {
        self.patch
            .into_iter()
            .map(|index| {
                f64::from(self.tyre.topology().nodes()[index].rest_position.y)
                    - self.tyre.positions()[index].y
            })
            .sum::<f64>()
            * (1_000.0 / 3.0)
    }
}

struct Measurement {
    samples_mm: Vec<f64>,
    peak_inward_mm: f64,
    release_mm: f64,
    outward_rebound_mm: f64,
    residual_mm: f64,
}

fn measure(substeps_per_second: u32) -> Result<Measurement, ElasticTyreError> {
    let mut experiment = Experiment::new(substeps_per_second);
    let mut samples_mm = vec![0.0];
    let mut peak_inward_mm: f64 = 0.0;
    let mut release_mm = 0.0;
    let mut outward_rebound_mm: f64 = 0.0;
    let sample_period = substeps_per_second / TICKS_PER_SECOND;
    while !experiment.replay_finished() {
        let load = experiment.replay_load();
        experiment.advance(load)?;
        let deflection_mm = experiment.deflection_mm();
        if load {
            peak_inward_mm = peak_inward_mm.max(deflection_mm);
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
        peak_inward_mm,
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
        LAB_SUBSTEPS_PER_SECOND,
        2 * LAB_SUBSTEPS_PER_SECOND,
        4 * LAB_SUBSTEPS_PER_SECOND,
    ];
    let measurements = rates.map(measure);
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
}

use vehicle_physics::{
    mounted_tyre::{MountedStep, MountedTyre},
    pressure::PressureConfig,
    puncture::PunctureConfig,
};

use crate::{
    elastic_experiment::TICKS_PER_SECOND,
    impact_experiment::{KERB_RADIUS_M, KERB_RISE_M, damage_counts},
};

pub const OBSERVATION_S: u32 = 5;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PunctureCase {
    Tearing,
    Sealed,
    Yielding,
}

impl PunctureCase {
    pub const ALL: [Self; 3] = [Self::Tearing, Self::Sealed, Self::Yielding];

    pub fn name(self) -> &'static str {
        match self {
            Self::Tearing => "6 m/s tear",
            Self::Sealed => "6 m/s sealed",
            Self::Yielding => "4 m/s yield",
        }
    }

    pub fn speed_mps(self) -> f64 {
        match self {
            Self::Tearing | Self::Sealed => 6.0,
            Self::Yielding => 4.0,
        }
    }
}

pub fn material() -> PunctureConfig {
    PunctureConfig::new(0.001, 0.65).expect("puncture bench settings")
}

#[derive(Clone, Copy)]
pub struct Sample {
    pub time_s: f64,
    pub gauge_pa: f64,
    pub volume_m3: f64,
    pub mass_kg: f64,
    pub flow_kg_s: f64,
    pub opening_m2: f64,
    pub speed_mps: f64,
    pub axle_y_m: f64,
    pub support_n: f64,
    pub over_top: bool,
    pub airborne: bool,
    pub yielded: usize,
    pub broken: usize,
}

pub struct TearEvent {
    pub time_s: f64,
    pub opening_m2: f64,
    pub panels: usize,
}

pub struct PunctureRun {
    pub case: PunctureCase,
    pub last: Sample,
    pub samples: Vec<Sample>,
    pub events: Vec<TearEvent>,
    pub contact: Option<(f64, f64)>,
    pub final_tyre: Option<MountedTyre>,
    pub max_mass_change_kg: f64,
    pub initial_mass_kg: f64,
    pub final_support_sum_n: f64,
    pub loaded_height_sum_m: f64,
    pub loaded_steps: u32,
    contact_step: Option<u32>,
    steps: u32,
}

impl Sample {
    fn from_wheel(time_s: f64, tyre: &MountedTyre, step: MountedStep) -> Self {
        let chamber = tyre
            .chamber_reading()
            .expect("the puncture bench has a chamber");
        let puncture = tyre.puncture_reading();
        let rim = tyre.rim();
        Self {
            time_s,
            gauge_pa: chamber.gauge_pressure_pa,
            volume_m3: chamber.volume_m3,
            mass_kg: tyre
                .chamber_air_mass_kg()
                .expect("the puncture bench has air mass"),
            flow_kg_s: puncture.map_or(0.0, |p| p.outward_mass_flow_kg_s),
            opening_m2: puncture.map_or(0.0, |p| p.total_open_area_m2),
            speed_mps: rim.linear_velocity.z,
            axle_y_m: rim.centre.y,
            support_n: step.floor_force_n + step.kerb_force.y,
            over_top: tyre
                .positions()
                .iter()
                .all(|point| point.z >= KERB_RADIUS_M),
            airborne: step.floor_force_n + step.kerb_normal_force_n == 0.0,
            yielded: 0,
            broken: 0,
        }
    }

    pub fn loaded_height_m(self) -> Option<f64> {
        (self.over_top && !self.airborne).then_some(self.axle_y_m - KERB_RISE_M)
    }
}

impl PunctureRun {
    pub fn new(case: PunctureCase, tyre: &MountedTyre) -> Self {
        let initial = Sample::from_wheel(0.0, tyre, MountedStep::default());
        Self {
            case,
            last: initial,
            samples: vec![initial],
            events: Vec::new(),
            contact: None,
            final_tyre: None,
            max_mass_change_kg: 0.0,
            initial_mass_kg: initial.mass_kg,
            final_support_sum_n: 0.0,
            loaded_height_sum_m: 0.0,
            loaded_steps: 0,
            contact_step: None,
            steps: 0,
        }
    }

    pub fn record(
        &mut self,
        tyre: &MountedTyre,
        step: MountedStep,
        rate: u32,
    ) -> Result<bool, &'static str> {
        let frequency = f64::from(rate);
        if self.contact_step.is_none() && step.kerb_normal_force_n > 0.0 {
            self.contact_step = Some(self.steps);
            self.contact = Some((f64::from(self.steps) / frequency, self.last.speed_mps));
        }
        self.steps += 1;
        let mut sample = Sample::from_wheel(f64::from(self.steps) / frequency, tyre, step);
        let absolute = sample.gauge_pa + PressureConfig::default().ambient_pressure_pa;
        if !(absolute.is_finite() && absolute > 0.0 && sample.mass_kg.is_finite()) {
            return Err("invalid chamber pressure or mass");
        }
        self.max_mass_change_kg = self
            .max_mass_change_kg
            .max((sample.mass_kg - self.initial_mass_kg).abs());
        if sample.opening_m2 > self.last.opening_m2 {
            self.events.push(TearEvent {
                time_s: sample.time_s,
                opening_m2: sample.opening_m2,
                panels: tyre.torn_panels().len(),
            });
        }
        let elapsed_steps = self.contact_step.map_or(0, |start| self.steps - start);
        if elapsed_steps > (OBSERVATION_S - 1) * rate {
            self.final_support_sum_n += sample.support_n;
            if let Some(height) = sample.loaded_height_m() {
                self.loaded_height_sum_m += height;
                self.loaded_steps += 1;
            }
        }
        let complete = elapsed_steps == OBSERVATION_S * rate;
        if complete || self.steps.is_multiple_of(rate / TICKS_PER_SECOND) {
            (sample.yielded, sample.broken) =
                damage_counts(tyre.beam_damage().expect("enabled damage"));
            self.samples.push(sample);
        } else {
            sample.yielded = self.last.yielded;
            sample.broken = self.last.broken;
        }
        self.last = sample;
        if complete {
            self.final_tyre = Some(tyre.clone());
        } else if self.contact_step.is_none() && self.steps >= 2 * rate {
            return Err("wheel did not reach the kerb");
        }
        Ok(complete)
    }
}

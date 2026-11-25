use std::error::Error;

use vehicle_physics::elastic_tyre::ElasticTyre;

use crate::{
    corner_experiment::{
        CORNER_TICKS, CornerRoad, CornerRun, CornerSample, RAMP_END_M, RAMP_RISE_M, RAMP_START_M,
        SAG_M, SEAM_Z_M, SETTLE_TICKS,
    },
    elastic_experiment::TICKS_PER_SECOND,
    loaded_tyre::{self, NOMINAL_LOAD_N, RATES},
};

pub fn report(samples: bool) -> Result<(), Box<dyn Error>> {
    if samples {
        println!(
            "case,rate,time_s,z_m,axle_y_m,body_y_m,compression_m,support_n,spin_rad_s,clearance_m,triangle_penetration_m,triangle_normal_y_n,triangle_normal_z_n,triangle_friction_y_n,triangle_friction_z_n"
        );
    }
    let mut reference_travel = None;
    for rate in RATES {
        let tyre = loaded_tyre::prepare(rate, true)?;
        let control = run_case(rate, &tyre, CornerRoad::Flat, samples)?;
        let flat = run_case(rate, &tyre, CornerRoad::TriangleFlat, samples)?;
        let ramp = run_case(rate, &tyre, CornerRoad::Ramp, samples)?;
        let rise = ramp.final_height - control.final_height;
        let seam_delta = ramp
            .trace
            .iter()
            .zip(&control.trace)
            .filter(|(ramp, _)| (ramp.position.z - SEAM_Z_M).abs() < 0.25)
            .map(|(ramp, control)| (ramp.support - control.support).abs())
            .reduce(f64::max)
            .ok_or("tow did not reach the coplanar seam")?;
        if (flat.static_height - control.static_height).abs() > 0.002
            || (ramp.static_height - control.static_height).abs() > 0.002
            || (flat.final_height - control.final_height).abs() > 0.002
            || (rise - RAMP_RISE_M).abs() > 0.003
            || seam_delta > 0.5
        {
            return Err(format!("{rate} Hz road height or coplanar seam check failed").into());
        }
        let climbed = ramp.trace.iter().any(|sample| {
            (RAMP_START_M..RAMP_END_M).contains(&sample.position.z)
                && sample.normal_force.z < -1.0
                && sample.spin > 1.0
        });
        let reached_platform = ramp.trace.last().unwrap().position.z > RAMP_END_M + 1.0;
        if !climbed || !reached_platform {
            return Err(
                format!("{rate} Hz corner did not roll up the ramp onto the platform").into(),
            );
        }
        if let Some(travel) = reference_travel {
            let difference: f64 = ramp.peak_travel - travel;
            if difference.abs() > 0.005 {
                return Err(format!("{rate} Hz peak suspension travel changed beyond 5 mm").into());
            }
        }
        reference_travel = Some(ramp.peak_travel);
        eprintln!(
            "{rate} Hz: platform axle rise {rise:.5} m, peak difference at the seam {seam_delta:.3} N"
        );
    }
    eprintln!("Triangle road support, rolling, ramp clearance, and timestep checks passed.");
    Ok(())
}

struct ResultSummary {
    trace: Vec<CornerSample>,
    static_height: f64,
    final_height: f64,
    peak_travel: f64,
}

fn run_case(
    rate: u32,
    tyre: &ElasticTyre,
    road: CornerRoad,
    samples: bool,
) -> Result<ResultSummary, Box<dyn Error>> {
    let mut run = CornerRun::new(rate, tyre, road)?;
    let mut trace = Vec::with_capacity(CORNER_TICKS as usize);
    for tick in 1..=CORNER_TICKS {
        let s = run
            .advance()
            .map_err(|error| format!("{rate} Hz {} at tick {tick}: {error}", road.name()))?;
        if samples {
            println!(
                "{},{rate},{:.6},{:.6},{:.6},{:.6},{:.6},{:.3},{:.6},{:.6},{:.6},{:.3},{:.3},{:.3},{:.3}",
                road.name(),
                s.time,
                s.position.z,
                s.position.y,
                s.body_y,
                s.compression,
                s.support,
                s.spin,
                s.clearance,
                s.penetration,
                s.normal_force.y,
                s.normal_force.z,
                s.tangential_force.y,
                s.tangential_force.z,
            );
        }
        trace.push(s);
    }
    let second = TICKS_PER_SECOND as usize;
    let settle = SETTLE_TICKS as usize;
    let stationary = &trace[settle - second..settle];
    let platform = &trace[trace.len() - second..];
    let mean = |window: &[CornerSample], read: fn(&CornerSample) -> f64| {
        window.iter().map(read).sum::<f64>() / window.len() as f64
    };
    let static_height = mean(stationary, |s| s.position.y);
    let final_height = mean(platform, |s| s.position.y);
    let static_support = mean(stationary, |s| s.support);
    let final_support = mean(platform, |s| s.support);
    let peak_travel = trace[settle..]
        .iter()
        .map(|s| (s.compression - SAG_M).abs())
        .fold(0.0, f64::max);
    let clearance = trace
        .iter()
        .map(|s| s.clearance)
        .fold(f64::INFINITY, f64::min);
    let penetration = trace.iter().map(|s| s.penetration).fold(0.0, f64::max);
    eprintln!(
        "{rate} Hz {}: settled {:.1} N at {:.5} m, final {:.1} N at {:.5} m, peak travel {:.5} m, minimum clearance {:.5} m, peak triangle penetration {:.5} m",
        road.name(),
        static_support,
        static_height,
        final_support,
        final_height,
        peak_travel,
        clearance,
        penetration,
    );
    if (static_support - NOMINAL_LOAD_N).abs() > 0.02 * NOMINAL_LOAD_N
        || (final_support - NOMINAL_LOAD_N).abs() > 0.02 * NOMINAL_LOAD_N
        || (mean(stationary, |s| s.compression) - SAG_M).abs() > 0.003
        || platform
            .iter()
            .any(|s| (s.position.y - final_height).abs() > 0.003)
        || clearance <= 0.0
        || trace.iter().any(|s| s.hub_error > 0.001)
    {
        return Err(format!(
            "{rate} Hz {} failed support, settling, or clearance checks",
            road.name()
        )
        .into());
    }
    Ok(ResultSummary {
        trace,
        static_height,
        final_height,
        peak_travel,
    })
}

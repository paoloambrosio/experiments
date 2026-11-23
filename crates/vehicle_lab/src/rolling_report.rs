use std::time::Duration;

use bevy::{prelude::*, time::TimeUpdateStrategy};

use crate::{
    drop_experiment::{TIMEOUT_S, WHEEL_WEIGHT_N},
    elastic_experiment::{RADIAL_SUBSTEPS_PER_SECOND, TICKS_PER_SECOND},
    rolling_experiment::{self, LoadCase, Phase, ROLL_TIMEOUT_S, RollingBench, Sample},
};

pub fn report(samples: bool) -> Result<(), Box<dyn std::error::Error>> {
    let mut reference = Vec::new();
    let mut traces = Vec::new();
    for rate in [RADIAL_SUBSTEPS_PER_SECOND, 2 * RADIAL_SUBSTEPS_PER_SECOND] {
        let mut app = App::new();
        let frame = Duration::from_secs_f64(1.0 / f64::from(TICKS_PER_SECOND));
        app.add_plugins((MinimalPlugins, TransformPlugin))
            .insert_resource(TimeUpdateStrategy::ManualDuration(frame));
        rolling_experiment::install(&mut app, rate);
        app.finish();
        app.cleanup();
        advance_until(&mut app, Phase::Ready, 32 + TIMEOUT_S + ROLL_TIMEOUT_S)
            .map_err(|error| format!("{rate} Hz preparation: {error}"))?;
        let radius = app.world().resource::<RollingBench>().radius_m.unwrap();
        println!("{rate} substeps/s, 30 kPa initial gauge, effective radius {radius:.5} m");
        println!(
            "{:14} {:>8} {:>8} {:>9} {:>8} {:>8} {:>8} {:>8} {:>8}",
            "Case",
            "rad/s",
            "Slip %",
            "Angle °",
            "Load N",
            "Floor Z",
            "Axle Z",
            "Floor X",
            "Axle X",
        );
        for (index, case) in LoadCase::ALL.into_iter().enumerate() {
            app.world_mut().resource_mut::<RollingBench>().replay(case);
            advance_until(&mut app, Phase::Complete, 1 + ROLL_TIMEOUT_S)
                .map_err(|error| format!("{rate} Hz {}: {error}", case.name()))?;
            let bench = app.world().resource::<RollingBench>();
            let mean = bench.mean.expect("completed rolling mean");
            let slip = mean.slip_ratio(radius);
            let angle_deg = mean.slip_angle_rad.to_degrees();
            println!(
                "{:14} {:8.3} {:+8.3} {:+9.3} {:8.2} {:+8.2} {:+8.2} {:+8.2} {:+8.2}",
                case.name(),
                mean.spin_rad_s,
                100.0 * slip,
                angle_deg,
                mean.floor_force_n.y,
                mean.floor_force_n.z,
                mean.axle_force_n.z,
                mean.floor_force_n.x,
                mean.axle_force_n.x,
            );
            check_case(case, mean, radius)
                .map_err(|error| format!("{rate} Hz {}: {error}", case.name()))?;
            if rate == RADIAL_SUBSTEPS_PER_SECOND {
                reference.push((slip, angle_deg, mean.axle_force_n));
                if samples {
                    traces.push((case, radius, bench.trace.clone()));
                }
            } else {
                let (reference_slip, reference_angle, reference_force) = reference[index];
                let force_delta = mean.axle_force_n - reference_force;
                if (slip - reference_slip).abs() > 0.01
                    || (angle_deg - reference_angle).abs() > 0.5
                    || force_delta.xz().length() > 5.0
                {
                    return Err(format!(
                        "{} differs across rates by more than 0.01 slip ratio, 0.5° slip angle, or 5 N axle force",
                        case.name()
                    )
                    .into());
                }
            }
        }
        println!();
    }
    println!("Support, slip, force direction, and timestep checks passed.");
    if samples {
        println!(
            "trace,case,time_s,spin_rad_s,slip_ratio,slip_angle_deg,normal_n,floor_z_n,axle_z_n,floor_x_n,axle_x_n"
        );
        for (case, radius, trace) in traces {
            for (time, sample) in trace {
                println!(
                    "trace,{},{time:.8},{:.8},{:.8},{:.8},{:.8},{:.8},{:.8},{:.8},{:.8}",
                    case.name(),
                    sample.spin_rad_s,
                    sample.slip_ratio(radius),
                    sample.slip_angle_rad.to_degrees(),
                    sample.floor_force_n.y,
                    sample.floor_force_n.z,
                    sample.axle_force_n.z,
                    sample.floor_force_n.x,
                    sample.axle_force_n.x,
                );
            }
        }
    }
    Ok(())
}

fn advance_until(app: &mut App, target: Phase, timeout_s: u32) -> Result<(), String> {
    for _ in 0..timeout_s * TICKS_PER_SECOND {
        app.update();
        let bench = app.world().resource::<RollingBench>();
        if let Some(error) = &bench.error {
            return Err(error.clone());
        }
        if bench.is_at(target) {
            return Ok(());
        }
    }
    Err(format!("did not reach {target:?} on the fixed clock"))
}

fn check_case(case: LoadCase, mean: Sample, radius_m: f64) -> Result<(), &'static str> {
    if (mean.floor_force_n.y - WHEEL_WEIGHT_N).abs() > 0.02 * WHEEL_WEIGHT_N {
        return Err("mean normal support differs from the wheel weight by more than 2%");
    }
    let slip = mean.slip_ratio(radius_m);
    let signs_are_correct = match case {
        LoadCase::Free => slip.abs() <= 0.01,
        LoadCase::Drive => slip > 0.0 && mean.axle_force_n.z > 0.0 && mean.floor_force_n.z > 0.0,
        LoadCase::Brake => slip < 0.0 && mean.axle_force_n.z < 0.0 && mean.floor_force_n.z < 0.0,
        LoadCase::Lateral => {
            mean.slip_angle_rad > 0.0 && mean.axle_force_n.x < 0.0 && mean.floor_force_n.x < 0.0
        }
    };
    if !signs_are_correct {
        return Err("mean slip or force failed the expected direction check");
    }
    Ok(())
}

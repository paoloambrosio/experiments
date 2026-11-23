use std::time::Duration;

use bevy::{prelude::*, time::TimeUpdateStrategy};

use crate::{
    elastic_experiment::{RADIAL_SUBSTEPS_PER_SECOND, TICKS_PER_SECOND},
    impact_experiment::{self, ImpactBench, Phase, SPEEDS_MPS, damage_counts},
};

#[derive(Clone, Copy)]
struct ResultRow {
    peak_m: f64,
    residual_m: f64,
    yielded: usize,
    broken: usize,
}

pub fn report(samples: bool) -> Result<(), Box<dyn std::error::Error>> {
    let mut rows = Vec::new();
    let mut passed = true;
    for rate in [RADIAL_SUBSTEPS_PER_SECOND, 2 * RADIAL_SUBSTEPS_PER_SECOND] {
        println!("{rate} substeps/s, 30 kPa initial gauge");
        let mut app = App::new();
        let frame = Duration::from_secs_f64(1.0 / f64::from(TICKS_PER_SECOND));
        app.add_plugins((MinimalPlugins, TransformPlugin))
            .insert_resource(TimeUpdateStrategy::ManualDuration(frame));
        impact_experiment::install(&mut app, rate);
        app.finish();
        app.cleanup();
        advance_until(&mut app, Phase::Ready, 32)
            .map_err(|error| format!("{rate} Hz preparation: {error}"))?;
        println!(
            "Run         Launch m/s  Contact ms   Strike m/s   Peak mm   Residual mm   Yielded   Broken"
        );
        for (index, (speed, control)) in SPEEDS_MPS
            .into_iter()
            .map(|speed| (speed, false))
            .chain([(SPEEDS_MPS[2], true)])
            .enumerate()
        {
            app.world_mut()
                .resource_mut::<ImpactBench>()
                .launch(speed, control);
            let label = if control { "Flat control" } else { "Kerb" };
            advance_until(&mut app, Phase::Complete, 12)
                .map_err(|error| format!("{rate} Hz {label}, {speed} m/s: {error}"))?;
            let bench = app.world().resource::<ImpactBench>();
            let recovered = bench.unloaded.as_ref().expect("unloaded tyre");
            let states = recovered.beam_damage().expect("enabled damage");
            let (yielded, broken) = damage_counts(states);
            let row = ResultRow {
                peak_m: bench.run.peak_inward_m,
                residual_m: bench.run.residual_m.expect("settled tyre"),
                yielded,
                broken,
            };
            let strike = if let Some((time, speed)) = bench.run.contact {
                format!("{:10.2}   {:10.3}", time * 1000.0, speed)
            } else {
                format!("{:>10}   {:>10}", "-", "-")
            };
            println!(
                "{label:12} {:9.2}   {strike}   {:7.2}   {:11.3}   {:7}   {:6}",
                speed,
                row.peak_m * 1000.0,
                row.residual_m * 1000.0,
                yielded,
                broken,
            );
            if broken > 0 {
                let indices: Vec<_> = states
                    .iter()
                    .enumerate()
                    .filter_map(|(i, state)| state.broken().then_some(i))
                    .collect();
                println!("  Broken beam indices {indices:?}");
            }
            let is_expected_regime = match index {
                0 | 3 => yielded == 0 && broken == 0 && row.residual_m <= 0.0001,
                1 => yielded > 0 && broken == 0 && row.residual_m > 0.0001,
                2 => broken > 0 && broken <= states.len() / 20 && row.residual_m > 0.0001,
                _ => unreachable!(),
            };
            if !is_expected_regime
                || !row.peak_m.is_finite()
                || !row.residual_m.is_finite()
                || bench.run.contact.is_some() == control
            {
                eprintln!(
                    "{rate} Hz {label} at {speed} m/s missed its expected damage regime or contact check"
                );
                passed = false;
            }
            if rate == RADIAL_SUBSTEPS_PER_SECOND {
                rows.push(row);
                if samples {
                    print_samples(bench, label);
                }
            } else {
                let reference = rows[index];
                println!(
                    "  Finer minus lab rate   peak {:+.3} mm   residual {:+.3} mm   yielded {:+}   broken {:+}",
                    (row.peak_m - reference.peak_m) * 1000.0,
                    (row.residual_m - reference.residual_m) * 1000.0,
                    row.yielded as isize - reference.yielded as isize,
                    row.broken as isize - reference.broken as isize,
                );
            }
        }
        println!();
    }
    if !passed {
        return Err("kerb impact acceptance checks failed".into());
    }
    println!("Impact, recovery, control, and damage regime checks passed at both rates.");
    Ok(())
}

fn advance_until(app: &mut App, phase: Phase, timeout_s: u32) -> Result<(), String> {
    for _ in 0..timeout_s * TICKS_PER_SECOND {
        app.update();
        let bench = app.world().resource::<ImpactBench>();
        if let Some(error) = &bench.error {
            return Err(error.clone());
        }
        if bench.is_at(phase) {
            return Ok(());
        }
    }
    Err(format!("did not reach {phase:?} on the fixed clock"))
}

fn print_samples(bench: &ImpactBench, label: &str) {
    let speed = bench.run.speed_mps;
    println!("trace,run,launch_mps,time_s,forward_mps,kerb_normal_n,inward_m,yielded,broken");
    for sample in &bench.run.samples {
        println!(
            "trace,{label},{speed},{:.8},{:.8},{:.8},{:.8},{},{}",
            sample.time_s,
            sample.forward_speed_mps,
            sample.kerb_force_n,
            sample.inward_m,
            sample.yielded,
            sample.broken,
        );
    }
    let baseline = bench.baseline.as_ref().expect("prepared tyre");
    let recovered = bench.unloaded.as_ref().expect("unloaded tyre");
    println!(
        "profile,run,launch_mps,node,original_x_m,original_y_m,original_z_m,unloaded_x_m,unloaded_y_m,unloaded_z_m"
    );
    for (index, (a, b)) in baseline
        .positions()
        .iter()
        .zip(recovered.positions())
        .enumerate()
    {
        println!(
            "profile,{label},{speed},{index},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9}",
            a.x, a.y, a.z, b.x, b.y, b.z,
        );
    }
}

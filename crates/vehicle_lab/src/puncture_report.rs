use std::time::Duration;

use bevy::{prelude::*, time::TimeUpdateStrategy};
use vehicle_physics::pressure::PressureConfig;

use crate::{
    drop_experiment::WHEEL_WEIGHT_N,
    elastic_experiment::{RADIAL_SUBSTEPS_PER_SECOND, TICKS_PER_SECOND},
    impact_experiment::{self, ImpactBench, KERB_RISE_M, Phase},
    puncture_experiment::{self, OBSERVATION_S, PunctureCase, PunctureRun},
};

#[derive(Clone, Copy)]
struct Summary {
    pressure_pa: f64,
    loaded_height_m: f64,
}

pub fn report(samples: bool) -> Result<(), Box<dyn std::error::Error>> {
    let mut reference = Vec::new();
    let mut passed = true;
    for rate in [RADIAL_SUBSTEPS_PER_SECOND, 2 * RADIAL_SUBSTEPS_PER_SECOND] {
        let mut app = App::new();
        let frame = Duration::from_secs_f64(1.0 / f64::from(TICKS_PER_SECOND));
        app.add_plugins((MinimalPlugins, TransformPlugin))
            .insert_resource(TimeUpdateStrategy::ManualDuration(frame));
        impact_experiment::install(&mut app, rate);
        app.finish();
        app.cleanup();
        advance_until(&mut app, Phase::Ready, 32 * TICKS_PER_SECOND)?;
        println!("Puncture bench at {rate} substeps/s");
        let mut torn_summary = None;
        for (index, case) in PunctureCase::ALL.into_iter().enumerate() {
            app.world_mut()
                .resource_mut::<ImpactBench>()
                .launch_puncture(case);
            advance_until(
                &mut app,
                Phase::Complete,
                (OBSERVATION_S + 3) * TICKS_PER_SECOND,
            )
            .map_err(|error| format!("{rate} Hz {}: {error}", case.name()))?;
            let run = app
                .world()
                .resource::<ImpactBench>()
                .puncture
                .as_ref()
                .expect("completed puncture run");
            let summary = summarize(rate, run)?;
            match case {
                PunctureCase::Tearing => {
                    torn_summary = Some(summary);
                    if rate == RADIAL_SUBSTEPS_PER_SECOND {
                        fixed_volume_comparison(run)?;
                    }
                    if run.last.opening_m2 <= 0.0 || run.initial_mass_kg <= run.last.mass_kg {
                        eprintln!("{rate} Hz tear case produced no opening or air loss");
                        passed = false;
                    }
                }
                PunctureCase::Sealed => {
                    let torn = torn_summary.expect("tear case precedes sealed case");
                    if torn.pressure_pa >= summary.pressure_pa
                        || torn.loaded_height_m >= summary.loaded_height_m
                    {
                        eprintln!(
                            "{rate} Hz tear case did not finish lower in pressure and loaded axle height"
                        );
                        passed = false;
                    }
                    passed &= check_control(rate, run, run.last.broken > 0);
                }
                PunctureCase::Yielding => {
                    passed &=
                        check_control(rate, run, run.last.yielded > 0 && run.last.broken == 0);
                }
            }
            let mean_support = run.final_support_sum_n / f64::from(rate);
            if (mean_support - WHEEL_WEIGHT_N).abs() > 0.05 * WHEEL_WEIGHT_N {
                eprintln!("{rate} Hz {} did not support the wheel weight", case.name());
                passed = false;
            }
            if rate == RADIAL_SUBSTEPS_PER_SECOND {
                reference.push(summary);
            } else {
                let prior = reference[index];
                let pressure_difference = (summary.pressure_pa - prior.pressure_pa) / 1000.0;
                let height_difference = (summary.loaded_height_m - prior.loaded_height_m) * 1000.0;
                println!(
                    "  Finer minus lab rate   pressure {pressure_difference:+.3} kPa   loaded height {height_difference:+.3} mm"
                );
                if pressure_difference.abs() > 0.5 || height_difference.abs() > 2.0 {
                    eprintln!("{} missed the resolution check", case.name());
                    passed = false;
                }
            }
            if samples {
                print_samples(rate, run);
            }
        }
    }
    if !passed {
        return Err("puncture acceptance checks failed".into());
    }
    println!("Opening, air loss, controls, support, and timestep checks passed at both rates.");
    Ok(())
}

fn advance_until(app: &mut App, phase: Phase, ticks: u32) -> Result<(), String> {
    for _ in 0..ticks {
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

fn summarize(rate: u32, run: &PunctureRun) -> Result<Summary, &'static str> {
    if run.loaded_steps == 0 {
        return Err("no loaded samples on the kerb top in the final second");
    }
    let loaded_height_m = run.loaded_height_sum_m / f64::from(run.loaded_steps);
    let (contact_s, strike_mps) = run.contact.expect("first contact");
    println!(
        "{}   contact {:.2} ms at {:.3} m/s   final {:.3} kPa   area {:.3} mm²   air lost {:.3} g   mean height {:.3} mm   support {:.2} N   yielded {}   broken {}",
        run.case.name(),
        contact_s * 1000.0,
        strike_mps,
        run.last.gauge_pa / 1000.0,
        run.last.opening_m2 * 1.0e6,
        (run.initial_mass_kg - run.last.mass_kg) * 1000.0,
        loaded_height_m * 1000.0,
        run.final_support_sum_n / f64::from(rate),
        run.last.yielded,
        run.last.broken,
    );
    for event in &run.events {
        println!(
            "  Tear at +{:.2} ms   {:.3} mm² across {} panels",
            (event.time_s - contact_s) * 1000.0,
            event.opening_m2 * 1.0e6,
            event.panels,
        );
    }
    Ok(Summary {
        pressure_pa: run.last.gauge_pa,
        loaded_height_m,
    })
}

fn check_control(rate: u32, run: &PunctureRun, expected_damage: bool) -> bool {
    let valid = expected_damage
        && run.events.is_empty()
        && run.last.opening_m2 == 0.0
        && run.max_mass_change_kg == 0.0;
    if !valid {
        eprintln!(
            "{rate} Hz {} missed its closed-chamber or beam damage check",
            run.case.name()
        );
    }
    valid
}

fn fixed_volume_comparison(run: &PunctureRun) -> Result<(), Box<dyn std::error::Error>> {
    let tyre = run.final_tyre.as_ref().expect("captured loaded tyre");
    let panel = tyre
        .torn_panels()
        .first()
        .ok_or("the tear case has no opening")?;
    let area = panel.opening_area_m2;
    let ambient = PressureConfig::default().ambient_pressure_pa;
    let config = puncture_experiment::material();
    let flow = config.outward_mass_flow_kg_s(area, ambient + 30_000.0, ambient)?;
    let doubled = config.outward_mass_flow_kg_s(2.0 * area, ambient + 30_000.0, ambient)?;
    println!(
        "  Fixed volume, initially 30 kPa gauge   one {:.3} mm² panel {:.4} g/s   two equal panels {:.4} g/s",
        area * 1.0e6,
        flow * 1000.0,
        doubled * 1000.0
    );
    if flow <= 0.0 || (doubled / flow - 2.0).abs() > 1e-12 {
        return Err("doubling the opening did not double the initial mass flow".into());
    }
    Ok(())
}

fn print_samples(rate: u32, run: &PunctureRun) {
    let case = run.case.name();
    let (contact_s, _) = run.contact.expect("first contact");
    println!("event,rate,case,after_contact_s,opening_m2,panels");
    for event in &run.events {
        println!(
            "event,{rate},{case},{:.9},{:.12},{}",
            event.time_s - contact_s,
            event.opening_m2,
            event.panels
        );
    }
    println!(
        "trace,rate,case,time_s,after_contact_s,gauge_pa,volume_m3,mass_kg,outward_kg_s,opening_m2,forward_mps,axle_y_m,support_n,over_top,airborne,yielded,broken"
    );
    for s in &run.samples {
        println!(
            "trace,{rate},{case},{:.9},{:.9},{:.6},{:.12},{:.12},{:.12},{:.12},{:.6},{:.9},{:.6},{},{},{},{}",
            s.time_s,
            s.time_s - contact_s,
            s.gauge_pa,
            s.volume_m3,
            s.mass_kg,
            s.flow_kg_s,
            s.opening_m2,
            s.speed_mps,
            s.axle_y_m,
            s.support_n,
            s.over_top,
            s.airborne,
            s.yielded,
            s.broken,
        );
    }
    let tyre = run.final_tyre.as_ref().expect("captured loaded tyre");
    let rim = tyre.rim();
    println!("profile,rate,case,node,road_x_m,road_y_m,road_z_m");
    for (node, point) in tyre.positions().iter().enumerate() {
        println!(
            "profile,{rate},{case},{node},{:.9},{:.9},{:.9}",
            point.x - rim.centre.x,
            point.y - KERB_RISE_M,
            point.z - rim.centre.z,
        );
    }
}

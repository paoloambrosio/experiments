use std::time::Duration;

use bevy::{prelude::*, time::TimeUpdateStrategy};
use vehicle_physics::{avian_tyre::AvianTyre, mounted_tyre::MountedTyre};

use crate::{
    drop_experiment::{self, CLEARANCE_M, DropBench, GRAVITY_MPS2, TIMEOUT_S},
    elastic_experiment::{PRESETS_PA, RADIAL_SUBSTEPS_PER_SECOND, TICKS_PER_SECOND},
};

pub fn report(samples: bool) -> Result<(), Box<dyn std::error::Error>> {
    println!(
        "Preset    Rate   Contact   Rebound   Axle height   Patch L × W     Floor force   Max depth    Settled"
    );
    println!(
        "  kPa     Hz       ms        mm          mm            mm              N          mm           s"
    );
    for (preset, target_pa) in PRESETS_PA.into_iter().enumerate() {
        let mut reference = None;
        for rate in [RADIAL_SUBSTEPS_PER_SECOND, 2 * RADIAL_SUBSTEPS_PER_SECOND] {
            let app = completed_drop(preset, rate)?;
            let bench = app.world().resource::<DropBench>();
            let result = bench.run.settled.as_ref().expect("settled drop");
            let contact = bench.run.contact_time_s.ok_or("no floor contact")?;
            let rebound = bench.run.rebound_m.ok_or("no first rebound peak")?;
            println!(
                "{:5.0} {:7} {:9.2} {:9.2} {:12.2} {:8.2} × {:6.2} {:11.2} {:11.2} {:10.2}",
                target_pa / 1000.0,
                rate,
                contact * 1000.0,
                rebound * 1000.0,
                result.axle_height_m * 1000.0,
                result.patch_length_m * 1000.0,
                result.patch_width_m * 1000.0,
                result.mean_floor_force_n,
                bench.run.max_penetration_m * 1000.0,
                result.time_s,
            );
            let ballistic_s = (2.0 * CLEARANCE_M / f64::from(GRAVITY_MPS2)).sqrt();
            if (contact - ballistic_s).abs() > 0.002 {
                return Err(format!("first contact at {target_pa} Pa and {rate} Hz missed the ballistic prediction by more than 2 ms").into());
            }
            if let Some((height, rise)) = reference {
                let height_delta: f64 = result.axle_height_m - height;
                let rise_delta: f64 = rebound - rise;
                if height_delta.abs() > 0.001 || rise_delta.abs() > 0.005 {
                    return Err(format!("drop at {target_pa} Pa differs across rates by more than 1 mm in settled axle height or 5 mm in rebound").into());
                }
            } else {
                reference = Some((result.axle_height_m, rebound));
                if samples {
                    let wheel = app
                        .world()
                        .get::<AvianTyre>(bench.wheel.expect("dropped wheel"))
                        .expect("mounted tyre");
                    print_samples(target_pa / 1000.0, &bench.run.trace, wheel.tyre());
                }
            }
        }
    }
    println!("Contact time, settling, and timestep checks passed.");
    Ok(())
}

fn completed_drop(preset: usize, rate: u32) -> Result<App, Box<dyn std::error::Error>> {
    let mut app = report_app(preset, rate);
    // Allow the existing pressure preparation timeout, followed by the drop's own timeout.
    for _ in 0..(32 + TIMEOUT_S) * TICKS_PER_SECOND {
        app.update();
        let bench = app.world().resource::<DropBench>();
        if let Some(error) = &bench.error {
            return Err(format!(
                "{:.0} kPa at {rate} Hz: {error}",
                PRESETS_PA[preset] / 1000.0
            )
            .into());
        }
        if bench.run.settled.is_some() {
            return Ok(app);
        }
    }
    Err("drop bench did not complete on the fixed clock".into())
}

fn report_app(preset: usize, rate: u32) -> App {
    let mut app = App::new();
    let frame = Duration::from_secs_f64(1.0 / f64::from(TICKS_PER_SECOND));
    app.add_plugins((MinimalPlugins, TransformPlugin))
        .insert_resource(TimeUpdateStrategy::ManualDuration(frame));
    drop_experiment::install(&mut app, preset, rate, true);
    app.finish();
    app.cleanup();
    app
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_rim_and_tread_accelerate_together_before_contact() {
        let mut app = report_app(0, RADIAL_SUBSTEPS_PER_SECOND);
        for _ in 0..6 {
            app.update();
        }
        let bench = app.world().resource::<DropBench>();
        let wheel = app.world().get::<AvianTyre>(bench.wheel.unwrap()).unwrap();
        assert!(bench.error.is_none());
        assert!(bench.run.contact_time_s.is_none());
        let (time, _) = bench.run.trace.last().unwrap();
        assert!(*time > 0.0);
        let rim = wheel.tyre().rim();
        assert!((rim.linear_velocity.y + f64::from(GRAVITY_MPS2) * time).abs() < 3.0e-5);
        for velocity in wheel.tyre().velocities() {
            assert!((*velocity - rim.linear_velocity).length() < 3.0e-5);
        }
    }
}

fn print_samples(pressure_kpa: f64, trace: &[(f64, f64)], tyre: &MountedTyre) {
    println!("trace,kPa,time_s,axle_height_m");
    for (time, height) in trace {
        println!("trace,{pressure_kpa},{time:.8},{height:.8}");
    }
    let segments = tyre.positions().len() / 5;
    println!("section,kPa,x_m,y_m");
    for ring in 0..5 {
        let point = tyre.positions()[ring * segments + segments / 2];
        println!("section,{pressure_kpa},{:.8},{:.8}", point.x, point.y);
    }
    println!("patch,kPa,polygon,x_m,z_m");
    if let Some(footprint) = tyre.footprint() {
        for (index, polygon) in footprint.polygons.iter().enumerate() {
            for point in polygon {
                println!("patch,{pressure_kpa},{index},{:.8},{:.8}", point.x, point.z);
            }
        }
    }
}

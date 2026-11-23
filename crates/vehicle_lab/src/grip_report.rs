use std::error::Error;

use crate::{
    elastic_experiment::TICKS_PER_SECOND,
    grip_experiment::{self, LOADS_N, Preparation, SPEEDS_MPS, SlipCase, slip_ratio},
    loaded_tyre::{self, RATES},
    rolling_experiment::Sample,
};

pub fn report(samples: bool) -> Result<(), Box<dyn Error>> {
    println!(
        "calibration,rate_hz,load_n,speed_mps,radius_m,spin_rad_s,support_n,road_drag_n,axle_drag_n"
    );
    println!(
        "grip,rate_hz,load_n,speed_mps,target_slip_pct,target_angle_deg,slip_pct,angle_deg,spin_rad_s,belt_x_mps,belt_z_mps,support_n,road_long_n,road_lat_n,axle_long_n,axle_lat_n,road_long_per_support,road_lat_per_support,target_spin_rad_s,target_belt_x_mps,target_belt_z_mps"
    );
    if samples {
        println!(
            "grip_trace,rate_hz,load_n,speed_mps,target_slip_pct,target_angle_deg,time_s,slip_pct,angle_deg,spin_rad_s,belt_x_mps,belt_z_mps,support_n,road_long_n,road_lat_n,axle_long_n,axle_lat_n"
        );
    }
    let mut reference: Vec<Sample> = Vec::new();
    for rate in RATES {
        let tyre = loaded_tyre::prepare(rate, true)
            .map_err(|error| format!("{rate} Hz inflation: {error}"))?;
        let mut index = 0;
        for load in LOADS_N {
            for speed in SPEEDS_MPS {
                let context = format!("{rate} Hz, {load:.2} N, {speed:.0} m/s");
                let mut preparation = Preparation::new(tyre.clone(), rate, load, speed)
                    .map_err(|error| format!("{context} mount: {error}"))?;
                let calibration = loop {
                    if let Some(calibration) = preparation
                        .advance()
                        .map_err(|error| format!("{context} free rolling: {error}"))?
                    {
                        break calibration;
                    }
                };
                let free = calibration.mean;
                let radius = calibration.radius_m;
                println!(
                    "calibration,{rate},{load:.2},{speed:.0},{radius:.8},{:.6},{:.4},{:.4},{:.4}",
                    free.spin_rad_s,
                    free.floor_force_n.y,
                    -free.floor_force_n.z,
                    -free.axle_force_n.z,
                );
                for case in SlipCase::ALL {
                    let mut run = calibration.replay(rate, speed, case);
                    let mean = loop {
                        let result = run
                            .advance()
                            .map_err(|error| format!("{context}, {case}: {error}"))?;
                        let step = run.step_count();
                        if samples && step.is_multiple_of(rate / TICKS_PER_SECOND) {
                            let time = f64::from(step) / f64::from(rate);
                            let belt = run.belt_velocity();
                            let sample = run.latest();
                            println!(
                                "grip_trace,{rate},{load:.2},{speed:.0},{},{},{time:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.4},{:.4},{:.4},{:.4},{:.4}",
                                case.percent,
                                case.angle_deg,
                                100.0 * slip_ratio(sample.spin_rad_s, belt, radius),
                                sample.slip_angle_rad.to_degrees(),
                                sample.spin_rad_s,
                                belt.x,
                                belt.z,
                                sample.floor_force_n.y,
                                sample.floor_force_n.z,
                                sample.floor_force_n.x,
                                sample.axle_force_n.z,
                                sample.axle_force_n.x,
                            );
                        }
                        if let Some(mean) = result {
                            break mean;
                        }
                    };
                    let belt = run.belt_velocity();
                    let slip = slip_ratio(mean.spin_rad_s, belt, radius);
                    let angle = mean.slip_angle_rad.to_degrees();
                    grip_experiment::validate_case(mean, &calibration, case)
                        .map_err(|error| format!("{context}, {case}: {error}"))?;
                    if (slip - case.ratio()).abs() > 1e-9
                        || (angle - f64::from(case.angle_deg)).abs() > 1e-9
                    {
                        return Err(format!("{context}, {case}: measured slip {slip:.6} and angle {angle:.6}° missed the targets").into());
                    }
                    if rate == RATES[0] {
                        reference.push(mean);
                    } else {
                        let previous = reference[index];
                        for (quantity, delta) in [
                            (
                                "road longitudinal",
                                mean.floor_force_n.z - previous.floor_force_n.z,
                            ),
                            (
                                "road lateral",
                                mean.floor_force_n.x - previous.floor_force_n.x,
                            ),
                            (
                                "axle longitudinal",
                                mean.axle_force_n.z - previous.axle_force_n.z,
                            ),
                            (
                                "axle lateral",
                                mean.axle_force_n.x - previous.axle_force_n.x,
                            ),
                        ] {
                            if delta.abs() > 0.05 * load {
                                return Err(format!("{context}, {case}: {quantity} changed by {delta:+.1} N between rates").into());
                            }
                        }
                    }
                    let support = mean.floor_force_n.y;
                    let target_belt = case.belt(speed);
                    let target_spin = case.commanded_spin(target_belt, radius);
                    println!(
                        "grip,{rate},{load:.2},{speed:.0},{},{},{:.6},{angle:.6},{:.6},{:.6},{:.6},{support:.4},{:.4},{:.4},{:.4},{:.4},{:.8},{:.8},{target_spin:.6},{:.6},{:.6}",
                        case.percent,
                        case.angle_deg,
                        100.0 * slip,
                        mean.spin_rad_s,
                        belt.x,
                        belt.z,
                        mean.floor_force_n.z,
                        mean.floor_force_n.x,
                        mean.axle_force_n.z,
                        mean.axle_force_n.x,
                        mean.floor_force_n.z / support,
                        mean.floor_force_n.x / support,
                        target_belt.x,
                        target_belt.z,
                    );
                    index += 1;
                }
            }
        }
    }
    println!("All 228 loaded grip cases passed the support, force, slip, and timestep checks.");
    Ok(())
}

use std::error::Error;

use vehicle_physics::elastic_tyre::ElasticTyre;

use crate::{
    corner_experiment::SAG_M,
    handling_experiment::{
        Brake, CASES, COM_HEIGHT, Case, HandlingRun, Phase, Results, Sample, WHEEL_NAMES,
    },
    loaded_tyre::{self, NOMINAL_LOAD_N, RATES},
};

pub fn report(samples: bool) -> Result<(), Box<dyn Error>> {
    println!(
        "handling,rate,case,launch_s,straight_drift_m,static_com_m,static_fl_n,static_fr_n,static_rl_n,static_rr_n,sag_fl_m,sag_fr_m,sag_rl_m,sag_rr_m,min_clearance_m,radius_m,yaw_rad_s,lateral_accel_m_s2,roll_rad,turn_fl_n,turn_fr_n,turn_rl_n,turn_rr_n,brake_entry_m_s,stop_m,stop_s,mean_decel_m_s2,brake_fl_n,brake_fr_n,brake_rl_n,brake_rr_n,brake_curve_1_m,brake_yaw_rad_s,lock_fl_s,lock_fr_s,lock_rl_s,lock_rr_s"
    );
    if samples {
        println!(
            "handling_chassis,rate,case,time_s,phase,x_m,y_m,z_m,vx_m_s,vy_m_s,vz_m_s,speed_m_s,heading_rad,yaw_rad_s,lateral_accel_m_s2,roll_rad"
        );
        println!(
            "handling_wheel,rate,case,time_s,wheel,fx_n,fy_n,fz_n,sag_m,clearance_m,steer_rad,spin_rad_s,forward_m_s,locked"
        );
    }

    let mut baseline: Option<Vec<Results>> = None;
    let mut failures = Vec::new();
    for rate in RATES {
        let tyre = loaded_tyre::prepare(rate, true)?;
        let mut results = Vec::new();
        for case in CASES {
            let result = run_case(rate, &tyre, case, samples)
                .map_err(|error| format!("{rate} substeps/s {}: {error}", case.id))?;
            print_result(rate, case, &result);
            for failure in validate(case, &result) {
                failures.push(format!("{rate} substeps/s {}: {failure}", case.id));
            }
            results.push(result);
        }
        for (left, right) in [(3, 4), (5, 6)] {
            if !within_five_percent(results[left].radius, results[right].radius) {
                failures.push(format!(
                    "{rate} substeps/s {} and {} radii differ by more than 5%",
                    CASES[left].id, CASES[right].id
                ));
            }
        }
        if results[1].stop_distance <= results[0].stop_distance {
            failures.push(format!(
                "{rate} substeps/s 8 m/s moderate stop must exceed 4 m/s stop"
            ));
        }
        let (moderate, limit) = (&results[7], &results[8]);
        if (moderate.brake_entry_speed - limit.brake_entry_speed).abs() > 1e-6
            || !within_five_percent(moderate.radius, limit.radius)
        {
            failures.push(format!(
                "{rate} substeps/s turning brake branches had different entry conditions"
            ));
        }
        println!(
            "handling_comparison,{rate},left-8,moderate_curvature_fraction,{:.6},limit_curvature_fraction,{:.6}",
            moderate.brake_curvature * moderate.radius,
            limit.brake_curvature * limit.radius,
        );
        if let Some(reference) = baseline.as_ref() {
            compare_rates(reference, &results, &mut failures);
        }
        baseline = Some(results);
    }
    if !failures.is_empty() {
        return Err(failures.join("\n").into());
    }
    eprintln!("All 18 handling runs passed static, manoeuvre, and solver-rate checks.");
    Ok(())
}

pub(crate) fn run_case(
    rate: u32,
    tyre: &ElasticTyre,
    case: Case,
    samples: bool,
) -> Result<Results, Box<dyn Error>> {
    let mut run = HandlingRun::new(rate, tyre, case)?;
    while run.phase != Phase::Done {
        let sample = run.advance()?;
        if samples {
            print_sample(rate, case, sample);
        }
    }
    Ok(run.results)
}

fn validate(case: Case, r: &Results) -> Vec<&'static str> {
    let mut failures = Vec::new();
    if r.static_loads
        .iter()
        .any(|load| (load - NOMINAL_LOAD_N).abs() > 0.02 * NOMINAL_LOAD_N)
        || r.static_sag.iter().any(|sag| (sag - SAG_M).abs() > 0.003)
        || (r.static_height - f64::from(COM_HEIGHT)).abs() > 0.002
    {
        failures.push("static support, sag, or centre-of-mass height outside tolerance");
    }
    if r.min_clearance <= 0.0 || !r.min_clearance.is_finite() {
        failures.push("invalid rim clearance");
    }
    if case.turn != 0.0 {
        let [fl, fr, rl, rr] = r.turn_loads;
        let outside_load_delta = (fr + rr - fl - rl) * f64::from(case.turn);
        if !r.radius.is_finite()
            || r.radius <= 0.0
            || r.yaw * f64::from(case.turn) <= 0.0
            || r.lateral_accel * f64::from(case.turn) <= 0.0
            || outside_load_delta <= 0.0
        {
            failures.push("turn direction, radius, or outside load transfer invalid");
        }
    }
    if let Some(brake) = case.brake {
        let [fl, fr, rl, rr] = r.brake_loads;
        if !r.stop_distance.is_finite()
            || r.stop_distance <= 0.0
            || fl + fr <= r.static_loads[0] + r.static_loads[1]
            || rl + rr >= r.static_loads[2] + r.static_loads[3]
            || !r.brake_curvature.is_finite()
        {
            failures.push("stopping distance, braking interval, or forward load transfer invalid");
        }
        if r.lock_time.iter().any(Option::is_some) != (brake == Brake::Limit) {
            failures.push("wheel lock did not match the brake capacity");
        }
    }
    failures
}

fn compare_rates(reference: &[Results], results: &[Results], failures: &mut Vec<String>) {
    for (i, (base, high)) in reference.iter().zip(results).enumerate() {
        let case = CASES[i];
        if case.turn != 0.0 && !within_five_percent(base.radius, high.radius) {
            failures.push(format!(
                "{} radius changed by more than 5% between rates",
                case.id
            ));
        }
        if case.brake.is_some() && !within_five_percent(base.stop_distance, high.stop_distance) {
            failures.push(format!(
                "{} stop distance changed by more than 5% between rates",
                case.id
            ));
        }
    }
}

fn within_five_percent(a: f64, b: f64) -> bool {
    a.is_finite() && b.is_finite() && (a - b).abs() <= 0.05 * a.abs().max(b.abs())
}

fn print_result(rate: u32, case: Case, r: &Results) {
    let fields = [
        r.launch_time,
        r.drift,
        r.static_height,
        r.static_loads[0],
        r.static_loads[1],
        r.static_loads[2],
        r.static_loads[3],
        r.static_sag[0],
        r.static_sag[1],
        r.static_sag[2],
        r.static_sag[3],
        r.min_clearance,
    ];
    let mut row = format!("handling,{rate},{}", case.id);
    append_fields(&mut row, fields.into_iter().map(Some));
    append_fields(
        &mut row,
        [
            r.radius,
            r.yaw,
            r.lateral_accel,
            r.roll,
            r.turn_loads[0],
            r.turn_loads[1],
            r.turn_loads[2],
            r.turn_loads[3],
        ]
        .into_iter()
        .map(|value| (case.turn != 0.0).then_some(value)),
    );
    append_fields(
        &mut row,
        [
            r.brake_entry_speed,
            r.stop_distance,
            r.stop_time,
            (r.brake_entry_speed - 0.1) / r.stop_time,
            r.brake_loads[0],
            r.brake_loads[1],
            r.brake_loads[2],
            r.brake_loads[3],
            r.brake_curvature,
            r.brake_yaw,
        ]
        .into_iter()
        .map(|value| case.brake.is_some().then_some(value)),
    );
    append_fields(&mut row, r.lock_time);
    println!("{row}");
}

fn append_fields(row: &mut String, fields: impl IntoIterator<Item = Option<f64>>) {
    use std::fmt::Write;
    for field in fields {
        row.push(',');
        if let Some(value) = field {
            write!(row, "{value:.6}").unwrap();
        }
    }
}

fn print_sample(rate: u32, case: Case, sample: Sample) {
    use bevy::prelude::Vec3;

    let id = case.id;
    let p = sample.position;
    let v = sample.velocity;
    let heading = sample.rotation * Vec3::Z;
    println!(
        "handling_chassis,{rate},{id},{:.6},{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
        sample.time,
        sample.phase.label(),
        p.x,
        p.y,
        p.z,
        v.x,
        v.y,
        v.z,
        sample.speed,
        heading.x.atan2(heading.z),
        sample.yaw_rate,
        sample.lateral_accel,
        sample.roll,
    );
    for (name, w) in WHEEL_NAMES.into_iter().zip(sample.wheels) {
        println!(
            "handling_wheel,{rate},{id},{:.6},{name},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{}",
            sample.time,
            w.road.x,
            w.road.y,
            w.road.z,
            w.sag,
            w.clearance,
            w.steer,
            w.spin,
            w.forward_speed,
            w.locked,
        );
    }
}

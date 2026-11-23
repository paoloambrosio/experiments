use bevy::math::DVec3;

use crate::elastic_experiment::{Experiment, PRESETS_PA, RADIAL_SUBSTEPS_PER_SECOND, measure};

#[derive(Clone, Copy)]
struct ResultRow {
    crown_growth_mm: f64,
    shoulder_growth_mm: f64,
    width_change_mm: f64,
    volume_l: f64,
    peak_inward_mm: f64,
    seal_seconds: f64,
    profile: [[f64; 2]; 5],
}

fn radius(position: DVec3) -> f64 {
    position.y.hypot(position.z)
}

fn sample(mut experiment: Experiment) -> Result<ResultRow, Box<dyn std::error::Error>> {
    while !experiment.is_ready() {
        experiment.prepare_step()?;
    }
    let n = experiment.segments;
    let positions = experiment.tyre.positions();
    let rest = experiment.tyre.topology().nodes();
    let growth = |index: usize| {
        1_000.0 * (radius(positions[index]) - radius(rest[index].rest_position.as_dvec3()))
    };
    let crown_growth_mm = (2 * n..3 * n).map(growth).fold(f64::NEG_INFINITY, f64::max);
    let shoulder_growth_mm = (n..2 * n)
        .chain(3 * n..4 * n)
        .map(growth)
        .fold(f64::NEG_INFINITY, f64::max);
    let width_change_mm = (0..n)
        .map(|segment| {
            let left = n + segment;
            let right = 3 * n + segment;
            let rest_width =
                f64::from(rest[right].rest_position.x) - f64::from(rest[left].rest_position.x);
            1_000.0 * (positions[right].x - positions[left].x - rest_width)
        })
        .max_by(|a, b| a.abs().total_cmp(&b.abs()))
        .expect("populated shoulder rings");
    let profile = std::array::from_fn(|ring| {
        let point = positions[ring * n];
        [1_000.0 * point.x, 1_000.0 * radius(point)]
    });
    let volume_l = 1_000.0
        * experiment
            .tyre
            .chamber_reading()
            .expect("sealed tyre")
            .volume_m3;
    let seal_seconds = experiment.preparation_seconds();
    let peak_inward_mm = measure(&mut experiment)?.peak_inward_mm;
    Ok(ResultRow {
        crown_growth_mm,
        shoulder_growth_mm,
        width_change_mm,
        volume_l,
        peak_inward_mm,
        seal_seconds,
        profile,
    })
}

fn compare(rate: u32, reinforced: bool) -> Result<Vec<ResultRow>, Box<dyn std::error::Error>> {
    let model = if reinforced { "reinforced" } else { "original" };
    let mut rows = Vec::with_capacity(PRESETS_PA.len());
    for pressure in PRESETS_PA {
        let experiment = if reinforced {
            Experiment::radial(rate, pressure)
        } else {
            Experiment::inflated(rate, pressure)
        };
        let row = sample(experiment).map_err(|error| {
            format!(
                "{model} at {:.0} kPa and {rate} substeps/s: {error}",
                pressure / 1_000.0
            )
        })?;
        println!(
            "{model:>10}  {rate:>10}  {:>6.0}  {:>10.3}  {:>11.3}  {:>9.3}  {:>9.3}  {:>7.3}  {:>8.4}",
            pressure / 1_000.0,
            row.crown_growth_mm,
            row.shoulder_growth_mm,
            row.width_change_mm,
            row.volume_l,
            row.seal_seconds,
            row.peak_inward_mm,
        );
        rows.push(row);
    }
    Ok(rows)
}

pub fn report() -> Result<(), Box<dyn std::error::Error>> {
    println!("Tread reinforcement | settled unloaded shape | peak under a 3 N press");
    println!(
        "     model  substeps/s     kPa    crown mm  shoulder mm   width mm   volume L   seal s   peak mm"
    );
    let rate = RADIAL_SUBSTEPS_PER_SECOND;
    let original = compare(rate, false)?;
    let reinforced = compare(rate, true)?;
    let finer = compare(2 * rate, true)?;
    let (crown_error, press_error) =
        reinforced
            .iter()
            .zip(&finer)
            .fold((0.0_f64, 0.0_f64), |(crown, press), (coarse, fine)| {
                (
                    crown.max((coarse.crown_growth_mm - fine.crown_growth_mm).abs()),
                    press.max((coarse.peak_inward_mm - fine.peak_inward_mm).abs()),
                )
            });
    let original_60 = original.last().expect("60 kPa preset");
    let reinforced_60 = reinforced.last().expect("60 kPa preset");
    println!(
        "60 kPa cross-sections at segment zero in (X mm, radius mm), from left bead to right bead"
    );
    println!("  Original   {:.3?}", original_60.profile);
    println!("  Reinforced {:.3?}", reinforced_60.profile);
    println!(
        "Max timestep differences: crown radius {crown_error:.6} mm, peak press {press_error:.6} mm"
    );
    if reinforced_60.crown_growth_mm > 15.0 || reinforced_60.width_change_mm.abs() > 8.0 {
        return Err(
            "reinforced crown growth or shoulder width change exceeded the 60 kPa limit".into(),
        );
    }
    if crown_error > 0.5 || press_error > 0.10 {
        return Err("reinforced carcass exceeded the timestep comparison limits".into());
    }
    println!("Reinforced tread shape and timestep checks passed.");
    Ok(())
}

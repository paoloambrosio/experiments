use crate::elastic_experiment::{Experiment, PRESETS_PA, REFERENCE_SUBSTEPS_PER_SECOND, measure};

pub fn report() -> Result<(), Box<dyn std::error::Error>> {
    println!("Inflation | 3 N inward for 0.5 s | observe until 2 s");
    println!(
        "substeps/s   preset  gauge kPa   volume L   seal s    peak mm   release mm   residual mm"
    );
    let mut previous_peaks: Option<[f64; 5]> = None;
    for (pass, rate) in [
        REFERENCE_SUBSTEPS_PER_SECOND,
        2 * REFERENCE_SUBSTEPS_PER_SECOND,
    ]
    .into_iter()
    .enumerate()
    {
        let rest_volume = Experiment::inflated(rate, 0.0)
            .tyre
            .chamber_reading()
            .expect("sealed rest chamber")
            .volume_m3;
        let mut peaks = [0.0; 5];
        for (index, preset) in std::iter::once(None)
            .chain(PRESETS_PA.map(Some))
            .enumerate()
        {
            let mut experiment = match preset {
                Some(target) => Experiment::inflated(rate, target),
                None => Experiment::new(rate),
            };
            while !experiment.is_ready() {
                experiment.prepare_step()?;
            }
            let label = preset.map_or_else(
                || "vented".into(),
                |target| format!("{:.0} kPa", target / 1000.0),
            );
            let (gauge_pa, volume_m3) = experiment
                .tyre
                .chamber_reading()
                .map_or((0.0, rest_volume), |reading| {
                    (reading.gauge_pressure_pa, reading.volume_m3)
                });
            let measurement = measure(&mut experiment)?;
            peaks[index] = measurement.peak_inward_mm;
            println!(
                "{rate:>10} {label:>8} {:>10.3} {:>10.3} {:>8.3} {:>10.4} {:>12.4} {:>13.4}",
                gauge_pa / 1000.0,
                volume_m3 * 1000.0,
                experiment.preparation_seconds(),
                measurement.peak_inward_mm,
                measurement.release_mm,
                measurement.residual_mm,
            );
            if preset.is_none() {
                // Milestone 002 published these three vented measurements at both rates.
                let expected = [[4.6028, 3.2900, 0.0260], [4.6020, 3.2910, 0.0142]][pass];
                if [
                    measurement.peak_inward_mm,
                    measurement.release_mm,
                    measurement.residual_mm,
                ]
                .into_iter()
                .zip(expected)
                .any(|(value, expected)| (value - expected).abs() > 0.001)
                {
                    return Err(format!("vented measurements changed at {rate} substeps/s").into());
                }
            }
        }
        println!(
            "  Sealed 0 kPa minus vented peak: {:+.4} mm",
            peaks[1] - peaks[0]
        );
        if peaks[2..].windows(2).any(|pair| pair[0] <= pair[1]) {
            return Err(format!(
                "expected decreasing peaks across positive presets at {rate} substeps/s"
            )
            .into());
        }
        if let Some(coarse) = previous_peaks {
            let max_difference = peaks
                .iter()
                .zip(coarse)
                .map(|(fine, coarse)| (fine - coarse).abs())
                .fold(0.0, f64::max);
            println!("Maximum per-preset peak difference across rates: {max_difference:.4} mm");
            if max_difference > 0.10 {
                return Err(
                    "peak deflection changed by more than 0.10 mm when halving the timestep".into(),
                );
            }
        }
        previous_peaks = Some(peaks);
    }
    println!("Inflation, deflection, and timestep checks passed.");
    Ok(())
}

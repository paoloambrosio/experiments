use std::{error::Error, sync::Arc};

use avian3d::prelude::*;
use bevy::{math::DVec3, prelude::*};
use vehicle_physics::{
    avian_suspension::WheelControl,
    avian_tyre::AvianTyre,
    reference_car,
    triangle_road::TriangleRoad,
    tyre::{RingRole, TyreConfig},
    vehicle_assembly::{
        ChassisSpec, VehicleAssembly, VehiclePose, VehicleRoad, VehicleSpec, WHEEL_NAMES, WheelSpec,
    },
};

use crate::{
    corner_experiment::physics_app,
    elastic_experiment::TICKS_PER_SECOND,
    handling_experiment::CASES,
    handling_report,
    loaded_tyre::{self, G, NOMINAL_LOAD_N, RATES},
};

fn synthetic_spec() -> VehicleSpec {
    let wheels = std::array::from_fn(|i| {
        let front = i < 2;
        let side = if i % 2 == 0 { 1.0 } else { -1.0 };
        let mut tyre = reference_car::tyre_spec();
        let (centre_m, initial_length_m, stiffness_n_per_m, upright_mass_kg, bare_rim_mass_kg) =
            if front {
                tyre.geometry = TyreConfig {
                    bead_radius: 0.185,
                    shoulder_radius: 0.30,
                    tread_radius: 0.31,
                    bead_width: 0.15,
                    shoulder_width: 0.18,
                    ..tyre.geometry
                };
                tyre.material.total_mass = 7.5;
                (Vec3::new(side * 0.775, 0.0, 1.35), 0.17, 32_000.0, 3.5, 8.5)
            } else {
                tyre.geometry = TyreConfig {
                    bead_radius: 0.215,
                    shoulder_radius: 0.34,
                    tread_radius: 0.35,
                    bead_width: 0.185,
                    shoulder_width: 0.22,
                    ..tyre.geometry
                };
                tyre.material.total_mass = 9.0;
                (
                    Vec3::new(side * 0.84, 0.04, -1.45),
                    0.18,
                    36_000.0,
                    4.0,
                    10.5,
                )
            };
        Some(WheelSpec {
            centre_m,
            initial_length_m,
            suspension: vehicle_physics::avian_suspension::Suspension {
                stiffness_n_per_m,
                ..reference_car::SUSPENSION
            },
            upright_mass_kg,
            bare_rim_mass_kg,
            tyre,
        })
    });
    VehicleSpec {
        chassis: ChassisSpec {
            mass_kg: 1010.0,
            size_m: Vec3::new(1.65, 0.52, 3.9),
            centre_of_mass_m: Vec3::new(0.035, 0.20, 0.14),
        },
        wheels,
    }
}

fn triangle_floor() -> Result<VehicleRoad, Box<dyn Error>> {
    let vertices = [
        DVec3::new(-40.0, 0.0, -40.0),
        DVec3::new(-40.0, 0.0, 40.0),
        DVec3::new(40.0, 0.0, 40.0),
        DVec3::new(40.0, 0.0, -40.0),
    ];
    Ok(VehicleRoad::Triangles {
        road: Arc::new(TriangleRoad::new(&vertices, &[[0, 1, 2], [0, 2, 3]], 0.05)?),
        contact: reference_car::road().contact_parameters(),
    })
}

pub fn report() -> Result<(), Box<dyn Error>> {
    for rate in RATES {
        compare_reference(rate)?;
        run_synthetic(rate)?;
    }
    println!(
        "Reference regression, triangle-road support, placement, steering, and rear drive checks passed."
    );
    Ok(())
}

fn compare_reference(rate: u32) -> Result<(), Box<dyn Error>> {
    let tyre = loaded_tyre::prepare(rate, true)?;
    for (id, baseline) in [("stop-8", 10.94), ("limit-stop-8", 4.08)] {
        let case = CASES.into_iter().find(|case| case.id == id).unwrap();
        let result = handling_report::run_case(rate, &tyre, case, false)?;
        if result
            .static_loads
            .iter()
            .any(|&load| !near(load, NOMINAL_LOAD_N, 0.02 * NOMINAL_LOAD_N))
            || result
                .static_sag
                .iter()
                .any(|&sag| !near(sag, reference_car::SAG_M, 0.003))
            || !near(result.stop_distance, baseline, baseline * 0.02)
        {
            return Err(format!("{rate} Hz reference {id} missed the 011 baseline").into());
        }
        println!(
            "{rate} Hz reference {id}: corner loads {:.1?} N, sag {:.2?} mm, stop {:.3} m from {:.3} m/s",
            result.static_loads,
            result.static_sag.map(|sag| sag * 1000.0),
            result.stop_distance,
            result.brake_entry_speed,
        );
    }
    Ok(())
}

fn run_synthetic(rate: u32) -> Result<(), Box<dyn Error>> {
    let spec = synthetic_spec();
    let prepared = spec.prepare(rate)?;
    let wheels = spec.wheels.map(Option::unwrap);
    let radii = prepared.unloaded_radii_m();
    let geometry = prepared.geometry();
    for (actual, expected) in [
        (geometry.wheelbase_m, 2.80),
        (geometry.front_track_m, 1.55),
        (geometry.rear_track_m, 1.68),
    ] {
        if !near(f64::from(actual), expected, 1e-6) {
            return Err("synthetic wheelbase or track did not match its specification".into());
        }
    }
    for (radius, wheel) in radii.into_iter().zip(wheels) {
        if !near(radius, f64::from(wheel.tyre.geometry.tread_radius), 0.025) {
            return Err("synthetic tyre did not inflate close to its specified rest radius".into());
        }
    }
    if radii[2] - radii[0] < 0.02 {
        return Err("the synthetic axles did not retain distinct inflated tyre sizes".into());
    }
    let height = radii
        .into_iter()
        .zip(wheels)
        .map(|(radius, wheel)| radius as f32 - wheel.centre_m.y)
        .fold(f32::NEG_INFINITY, f32::max)
        + 0.002;
    let pose = VehiclePose {
        origin_m: Vec3::new(12.0, height, -8.0),
        rotation: Quat::from_rotation_y(0.37),
    };
    let mut app = physics_app(rate);
    let before = app.world().entities().count_spawned();
    let car = prepared.spawn(app.world_mut(), pose, &triangle_floor()?)?;
    verify_placement(app.world(), &car, pose, spec)?;

    let one_second = TICKS_PER_SECOND;
    let mut loads = [0.0; 4];
    let mut clearance = f64::INFINITY;
    for tick in 0..(4 * one_second) {
        app.update();
        let (support, rim_clearance) = read_road(app.world(), &car, wheels)?;
        clearance = clearance.min(rim_clearance);
        if tick >= 3 * one_second {
            for (sum, reading) in loads.iter_mut().zip(support) {
                *sum += reading / f64::from(one_second);
            }
        }
    }
    let weight = G
        * (f64::from(spec.chassis.mass_kg)
            + wheels
                .iter()
                .map(|wheel| {
                    f64::from(wheel.upright_mass_kg)
                        + wheel.bare_rim_mass_kg
                        + wheel.tyre.material.total_mass
                })
                .sum::<f64>());
    let total = loads.iter().sum();
    if !near(total, weight, 0.02 * weight) || loads.iter().any(|&load| load <= 0.0) {
        return Err(
            format!("synthetic triangle support {loads:?} N did not carry {weight:.1} N").into(),
        );
    }

    let [left, right] = car
        .geometry
        .front_steer(6.0_f32.to_radians())
        .ok_or("invalid steering geometry")?;
    if !(left > right && right > 0.0) {
        return Err("front steer signs or inside-wheel angle were incorrect".into());
    }
    let controls = [left, right, 0.0, 0.0].map(|steer_rad| WheelControl {
        steer_rad,
        ..default()
    });
    let start = app.world().get::<Position>(car.chassis).unwrap().0;
    for (i, (corner, mut control)) in car.wheels.iter().zip(controls).enumerate() {
        if i >= 2 {
            control.drive_nm = 95.0;
        }
        *app.world_mut()
            .get_mut::<WheelControl>(corner.spin_joint)
            .unwrap() = control;
    }
    for _ in 0..(2 * one_second) {
        app.update();
        let (_, rim_clearance) = read_road(app.world(), &car, wheels)?;
        clearance = clearance.min(rim_clearance);
    }
    for (i, corner) in car.wheels.iter().enumerate() {
        let world = app.world();
        let joint = world.get::<RevoluteJoint>(corner.spin_joint).unwrap();
        let expected = Quat::from_rotation_y(controls[i].steer_rad);
        let control = world.get::<WheelControl>(corner.spin_joint).unwrap();
        let actual = joint.local_basis1().unwrap_or(Quat::IDENTITY);
        if !actual.abs_diff_eq(expected, 1e-6)
            || control.drive_nm != if i >= 2 { 95.0 } else { 0.0 }
        {
            return Err(format!(
                "{} steering basis or drive command did not match the axle selection",
                WHEEL_NAMES[i]
            )
            .into());
        }
    }
    let end = app.world().get::<Position>(car.chassis).unwrap().0;
    let forward = pose.rotation * Vec3::Z;
    let distance = (end - start).dot(forward);
    let speed = app
        .world()
        .get::<LinearVelocity>(car.chassis)
        .unwrap()
        .dot(forward);
    if !(distance > 0.25 && speed > 0.25 && clearance > 0.0) {
        return Err(format!(
            "synthetic car failed the driven check at {distance:.3} m and {speed:.3} m/s"
        )
        .into());
    }
    println!(
        "{rate} Hz synthetic: inflated radii {:.2?} mm, loads {:.1?} N ({total:.1} / {weight:.1} N), clearance {:.2} mm, driven {:.3} m at {:.3} m/s",
        radii.map(|r| r * 1000.0),
        loads,
        clearance * 1000.0,
        distance,
        speed,
    );
    car.despawn(app.world_mut());
    if app.world().entities().count_spawned() != before {
        return Err("vehicle despawn left entities in the world".into());
    }
    Ok(())
}

fn verify_placement(
    world: &World,
    car: &VehicleAssembly,
    pose: VehiclePose,
    spec: VehicleSpec,
) -> Result<(), Box<dyn Error>> {
    let transform = |p| pose.origin_m + pose.rotation * p;
    let position = world.get::<Position>(car.chassis).unwrap().0;
    if !position.abs_diff_eq(transform(spec.chassis.centre_of_mass_m), 1e-5) {
        return Err("chassis centre did not transform from the vehicle pose".into());
    }
    for (i, (corner, wheel)) in car
        .wheels
        .iter()
        .zip(spec.wheels.map(Option::unwrap))
        .enumerate()
    {
        let expected = transform(wheel.centre_m);
        let tyre = world.get::<AvianTyre>(corner.rim).unwrap().tyre();
        if !world
            .get::<Position>(corner.hub)
            .unwrap()
            .0
            .abs_diff_eq(expected, 1e-5)
            || !tyre.rim().centre.as_vec3().abs_diff_eq(expected, 1e-5)
            || !(tyre.rim().rotation.as_quat() * Vec3::X).abs_diff_eq(pose.rotation * Vec3::X, 1e-5)
        {
            return Err(format!(
                "{} centre or axle did not transform from the vehicle pose",
                WHEEL_NAMES[i]
            )
            .into());
        }
        let geometry = wheel.tyre.geometry;
        for (ring, radius, width) in [
            (
                RingRole::RightBead,
                geometry.bead_radius,
                geometry.bead_width,
            ),
            (
                RingRole::RightShoulder,
                geometry.shoulder_radius,
                geometry.shoulder_width,
            ),
            (RingRole::TreadCentre, geometry.tread_radius, 0.0),
        ] {
            let rest = tyre
                .topology()
                .nodes()
                .iter()
                .find(|node| node.ring == ring)
                .unwrap()
                .rest_position;
            if !rest.abs_diff_eq(Vec3::new(width * 0.5, radius, 0.0), 1e-6) {
                return Err(format!(
                    "{} tyre did not retain the specified {ring:?} geometry",
                    WHEEL_NAMES[i]
                )
                .into());
            }
        }
    }
    Ok(())
}

fn read_road(
    world: &World,
    car: &VehicleAssembly,
    wheels: [WheelSpec; 4],
) -> Result<([f64; 4], f64), Box<dyn Error>> {
    let mut loads = [0.0; 4];
    let mut clearance = f64::INFINITY;
    for (i, corner) in car.wheels.iter().enumerate() {
        let wheel = world.get::<AvianTyre>(corner.rim).unwrap();
        if let Some(error) = wheel.error() {
            return Err(format!("{} tyre step failed with {error}", WHEEL_NAMES[i]).into());
        }
        let tyre = wheel.tyre();
        let reaction = tyre.triangle_reaction();
        loads[i] = reaction.normal_force.y + reaction.tangential_force.y;
        clearance =
            clearance.min(tyre.rim().centre.y - f64::from(wheels[i].tyre.geometry.bead_radius));
    }
    Ok((loads, clearance))
}

fn near(actual: f64, expected: f64, tolerance: f64) -> bool {
    (actual - expected).abs() <= tolerance
}

#[cfg(test)]
mod tests {
    use super::*;
    use vehicle_physics::vehicle_assembly::AssemblyError;

    #[test]
    fn invalid_vehicle_data_never_spawns_entities() {
        let spec = synthetic_spec();
        let mut world = World::new();
        world.spawn_empty();
        let count = world.entities().count_spawned();
        let road = triangle_floor().unwrap();
        let mut try_spawn = |spec: VehicleSpec, pose: VehiclePose| {
            let result = spec
                .prepare(RATES[0])
                .and_then(|prepared| prepared.spawn(&mut world, pose, &road));
            assert_eq!(world.entities().count_spawned(), count);
            result.err().expect("invalid assembly")
        };

        let mut bad = spec;
        bad.wheels[3] = None;
        assert!(matches!(
            try_spawn(bad, VehiclePose::default()),
            AssemblyError::MissingWheel("RR")
        ));

        let mut bad = spec;
        bad.wheels[0].as_mut().unwrap().tyre.geometry.tread_radius = 0.0;
        let error = try_spawn(bad, VehiclePose::default());
        assert!(matches!(error, AssemblyError::Tyre { wheel: "FL", .. }));
        assert!(error.to_string().contains("tyre.geometry"));

        let mut bad = spec;
        bad.wheels[2].as_mut().unwrap().initial_length_m = 0.35;
        assert!(matches!(
            try_spawn(bad, VehiclePose::default()),
            AssemblyError::InvalidWheel {
                wheel: "RL",
                field: "initial_length_m outside suspension travel"
            }
        ));

        let pose = VehiclePose {
            rotation: Quat::from_xyzw(0.0, 0.0, 0.0, 0.0),
            ..default()
        };
        assert!(matches!(
            try_spawn(spec, pose),
            AssemblyError::InvalidPose("rotation")
        ));
    }
}

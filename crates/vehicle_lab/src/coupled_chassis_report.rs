use std::{error::Error, sync::Arc};

use avian3d::prelude::*;
use bevy::{math::DVec3, prelude::*};
use vehicle_physics::{
    avian_suspension::WheelControl,
    avian_tyre::{AvianTyre, TyreSubstepComplete},
    beam_damage::BeamDamageConfig,
    chassis::{BeamCoefficients, ChassisBeamSpec, ChassisNode},
    deformable_vehicle::{
        DeformableBody, DeformableVehicle, DeformableVehiclePlugin, PreparedDeformableVehicle,
        StructureSpec, SuspensionMount,
    },
    reference_car::{
        self, COM_HEIGHT, G, MOUNTS, NOMINAL_LOAD_N, ReferenceTyre, SAG_M, SUSPENSION,
    },
    static_collision::{ContactFeature, StaticCollisionMesh, StaticContact},
    triangle_road::TriangleRoad,
    vehicle_assembly::{VehiclePose, VehicleRoad, WheelAlignment},
};

use crate::corner_experiment::physics_app_with;

const FACE: [[usize; 3]; 2] = [[0, 1, 2], [0, 2, 3]];
const HZ: usize = 60;
const RATES: [u32; 2] = [3840, 7680];
const MASS_KG: f64 = 920.0;

pub fn report() -> Result<(), Box<dyn Error>> {
    let mut baseline = None;
    for rate in RATES {
        let mut prepared = ReferenceTyre::new(rate);
        while !prepared.advance()? {}
        let road = road()?;
        let vehicle = fixture([WheelAlignment::default(); 4])?;
        if baseline.is_none() {
            let inertia = vehicle.inertia_kg_m2;
            println!(
                "Reference structure | 920 kg sprung | inertia {:.2?} kg·m²",
                [inertia.x_axis.x, inertia.y_axis.y, inertia.z_axis.z]
            );
        }
        let mut bench = Bench::new(rate);
        bench.spawn(&vehicle, &prepared, &road, DVec3::ZERO, None)?;
        let result = handling(&mut bench)?;
        println!(
            "{rate} Hz | support {:.1?} N | mean sag {:.2} mm | driven {:.3} m | turn {:.3} rad | stop {:.3} m | peak travel {:.2} mm",
            result.support,
            result.sag * 1000.0,
            result.travel,
            result.turn,
            result.stop,
            result.peak_travel * 1000.0
        );
        if let Some(first) = baseline {
            compare_rates(first, result)?;
        }
        baseline = Some(result);
        if rate == RATES[0] {
            alignment(&mut bench, &prepared, &road)?;
            impacts(&mut bench, &vehicle, &prepared, &road)?;
        }
    }
    Ok(())
}

fn fixture(alignment: [WheelAlignment; 4]) -> Result<PreparedDeformableVehicle, Box<dyn Error>> {
    let nodes: Vec<_> = [1.8, 1.25, -1.25, -1.8]
        .into_iter()
        .enumerate()
        .flat_map(|(i, z)| {
            let (x, mass) = if i == 0 || i == 3 {
                (0.62, 40.0)
            } else {
                (0.67, 75.0)
            };
            [(x, -0.25), (x, 0.25), (-x, -0.25), (-x, 0.25)].map(|(x, y)| ChassisNode {
                mass_kg: mass,
                position_m: DVec3::new(x, y, z),
                velocity_mps: DVec3::ZERO,
            })
        })
        .collect();
    let mounts = [[4, 5, 6], [6, 7, 4], [8, 9, 10], [10, 11, 8]].map(|indices| {
        let i = match indices[0] {
            4 => 0,
            6 => 1,
            8 => 2,
            _ => 3,
        };
        SuspensionMount {
            nodes: indices,
            offset_m: MOUNTS[i].as_dvec3() - nodes[indices[0]].position_m,
        }
    });
    let mut beams = Vec::new();
    let frame = BeamCoefficients {
        stiffness: 2_800_000.0,
        damping: 9_000.0,
    };
    let nose = BeamCoefficients {
        stiffness: 590_000.0,
        damping: 3_000.0,
    };
    let yield_nose = BeamDamageConfig::yield_only(0.09, 0.065)?;
    for start in [0, 4, 8, 12] {
        for i in start..start + 4 {
            for j in i + 1..start + 4 {
                beams.push(ChassisBeamSpec {
                    endpoints: [i, j],
                    coefficients: frame,
                    material: None,
                });
            }
            if start < 12 {
                for j in start + 4..start + 8 {
                    let coefficients = if start == 0 { nose } else { frame };
                    beams.push(ChassisBeamSpec {
                        endpoints: [i, j],
                        coefficients,
                        material: (start == 0).then_some(yield_nose),
                    });
                }
            }
        }
    }
    Ok(PreparedDeformableVehicle::new(
        reference_car::specification(),
        StructureSpec {
            nodes,
            beams,
            mounts,
            collision_nodes: (0..16).collect(),
            collision_panels: vec![[0, 1, 3], [0, 3, 2], [12, 14, 15], [12, 15, 13]],
        },
        alignment,
    )?)
}

fn road() -> Result<VehicleRoad, Box<dyn Error>> {
    let floor = [
        DVec3::new(-60.0, 0.0, -60.0),
        DVec3::new(60.0, 0.0, -60.0),
        DVec3::new(60.0, 0.0, 60.0),
        DVec3::new(-60.0, 0.0, 60.0),
    ];
    Ok(VehicleRoad::Triangles {
        road: Arc::new(TriangleRoad::new(&floor, &FACE, 0.06)?),
        contact: reference_car::road().contact_parameters(),
    })
}

struct Bench {
    app: App,
    vehicle: Option<DeformableVehicle>,
}

impl Bench {
    fn new(rate: u32) -> Self {
        let app = physics_app_with(rate, |app| {
            app.add_plugins(DeformableVehiclePlugin)
                .init_resource::<Trace>()
                .add_systems(SubstepSchedule, measure.after(TyreSubstepComplete));
        });
        Self { app, vehicle: None }
    }

    fn spawn(
        &mut self,
        vehicle: &PreparedDeformableVehicle,
        tyre: &ReferenceTyre,
        road: &VehicleRoad,
        velocity: DVec3,
        obstacles: Option<Arc<StaticCollisionMesh>>,
    ) -> Result<(), Box<dyn Error>> {
        let world = self.app.world_mut();
        if let Some(old) = self.vehicle.take() {
            old.despawn(world);
        }
        let car = vehicle.spawn(
            world,
            [&tyre.tyre; 4],
            VehiclePose {
                origin_m: Vec3::Y * COM_HEIGHT,
                rotation: Quat::IDENTITY,
            },
            velocity,
            road,
            obstacles,
        )?;
        world.insert_resource(Trace {
            monitored: Some((car.body, car.wheels.map(|wheel| wheel.rim))),
            ..Default::default()
        });
        self.vehicle = Some(car);
        Ok(())
    }

    fn step(&mut self, steer: f32, drive: f32, brake: f32) -> Result<(), Box<dyn Error>> {
        let car = self.vehicle.as_ref().unwrap();
        let [left, right] = car.geometry.front_steer(steer).unwrap();
        let world = self.app.world_mut();
        for (i, (wheel, steer_rad)) in car.wheels.iter().zip([left, right, 0.0, 0.0]).enumerate() {
            *world.get_mut::<WheelControl>(wheel.spin_joint).unwrap() = WheelControl {
                steer_rad,
                drive_nm: if i >= 2 { drive } else { 0.0 },
                brake_nm: brake,
            };
        }
        self.app.update();
        if let Some(error) = self.body().error() {
            return Err(format!("at substep {}: {error}", self.trace().steps).into());
        }
        Ok(())
    }

    fn body(&self) -> &DeformableBody {
        self.app
            .world()
            .get(self.vehicle.as_ref().unwrap().body)
            .unwrap()
    }
    fn trace(&self) -> &Trace {
        self.app.world().resource::<Trace>()
    }
    fn centre(&self) -> DVec3 {
        self.body()
            .chassis()
            .nodes()
            .iter()
            .map(|n| n.position_m * n.mass_kg)
            .sum::<DVec3>()
            / MASS_KG
    }
    fn velocity(&self) -> DVec3 {
        momentum(self.body()) / MASS_KG
    }
    fn heading(&self) -> f64 {
        let nodes = self.body().chassis().nodes();
        let direction =
            nodes[4].position_m + nodes[6].position_m - nodes[8].position_m - nodes[10].position_m;
        direction.x.atan2(direction.z)
    }
}

#[derive(Default, Resource)]
struct Trace {
    monitored: Option<(Entity, [Entity; 4])>,
    steps: usize,
    road_n: [f64; 4],
    peak_travel: f64,
    p: Option<DVec3>,
    max_balance_ns: f64,
    first_chassis: Option<(usize, StaticContact)>,
    crash_loads_n: [f64; 4],
    last_chassis_step: usize,
    first_wheel: Option<(usize, usize, f64)>,
    obstacle_impulse_ns: DVec3,
    wheel_transfer: Option<(f64, f64)>,
    body_transfer: Option<(usize, f64, f64)>,
    front_speed_mps: Option<f64>,
}

fn measure(
    time: Res<Time>,
    gravity: Res<Gravity>,
    mut trace: ResMut<Trace>,
    bodies: Query<&DeformableBody>,
    tyres: Query<&AvianTyre>,
) {
    let Some((entity, rims)) = trace.monitored else {
        return;
    };
    let Ok(body) = bodies.get(entity) else {
        return;
    };
    if body.error().is_some() {
        return;
    }
    let mounts = body.mount_reactions();
    let chassis_hit = body.obstacle_reaction();
    trace.steps += 1;
    let step = trace.steps;
    trace.obstacle_impulse_ns += chassis_hit.impulse_ns;
    let mut front_speed = 0.0;
    for (i, rim) in rims.into_iter().enumerate() {
        let tyre = tyres.get(rim).unwrap().tyre();
        if i < 2 {
            front_speed += 0.5 * tyre.rim().linear_velocity.z;
        }
        trace.road_n[i] = tyre.triangle_reaction().normal_force.y;
        trace.peak_travel = trace
            .peak_travel
            .max((mounts[i].length_m - (f64::from(SUSPENSION.free_length_m) - SAG_M)).abs());
        if tyre.obstacle_reaction().first.is_some() {
            let floor_load = trace.road_n[i];
            trace.first_wheel.get_or_insert((step, i, floor_load));
        }
    }
    if let Some(hit) = chassis_hit.first {
        if trace.first_chassis.is_none() {
            trace.first_chassis = Some((step, hit));
            trace.crash_loads_n = trace.road_n;
        }
        trace.last_chassis_step = step;
    }
    let front_delta = trace
        .front_speed_mps
        .replace(front_speed)
        .map_or(0.0, |previous| front_speed - previous);
    let p = momentum(body);
    if let Some(previous) = trace.p.replace(p) {
        let mount_impulse: DVec3 = mounts.iter().map(|r| r.impulse_ns).sum();
        trace.max_balance_ns = trace.max_balance_ns.max(
            (p - previous
                - mount_impulse
                - chassis_hit.impulse_ns
                - gravity.0.as_dvec3() * (MASS_KG * time.delta_secs_f64()))
            .length(),
        );
        if let Some((_, wheel, _)) = trace.first_wheel
            && trace.first_chassis.is_none()
            && mounts[wheel].impulse_ns.x < -0.05
            && (p - previous).x < -0.05
            && trace.wheel_transfer.is_none()
        {
            trace.wheel_transfer = Some((mounts[wheel].impulse_ns.x, (p - previous).x));
        }
        if trace.first_chassis.is_some()
            && trace.first_wheel.is_none()
            && trace.body_transfer.is_none()
        {
            let reaction_z = mounts[0].impulse_ns.z + mounts[1].impulse_ns.z;
            if reaction_z > 0.15 && front_delta < -0.001 {
                trace.body_transfer = Some((step, reaction_z, front_delta));
            }
        }
    }
}

fn momentum(body: &DeformableBody) -> DVec3 {
    body.chassis()
        .nodes()
        .iter()
        .map(|n| n.velocity_mps * n.mass_kg)
        .sum()
}

#[derive(Clone, Copy, Debug)]
struct Handling {
    support: [f64; 4],
    sag: f64,
    travel: f64,
    turn: f64,
    stop: f64,
    peak_travel: f64,
}

fn handling(bench: &mut Bench) -> Result<Handling, Box<dyn Error>> {
    let mut totals = [0.0; 4];
    let mut sag = 0.0;
    for tick in 0..4 * HZ {
        bench.step(0.0, 0.0, 0.0)?;
        if tick >= 3 * HZ {
            for (i, (sum, mount)) in totals
                .iter_mut()
                .zip(bench.body().mount_reactions())
                .enumerate()
            {
                *sum += bench.trace().road_n[i];
                sag += f64::from(SUSPENSION.free_length_m) - mount.length_m;
            }
        }
    }
    let support = totals.map(|load| load / HZ as f64);
    let sag = sag / (4 * HZ) as f64;
    if (support.iter().sum::<f64>() - 1000.0 * G).abs() > 0.01 * 1000.0 * G
        || support
            .iter()
            .any(|&n| (n - NOMINAL_LOAD_N).abs() > 0.03 * NOMINAL_LOAD_N)
        || (sag - SAG_M).abs() > 0.005
        || bench
            .body()
            .chassis()
            .beam_damage()
            .any(|beam| beam.yielded())
    {
        return Err(format!("settle failed with support {support:?} N and sag {sag:.5} m").into());
    }
    let start = bench.centre();
    let mut integral = 0.0_f64;
    // Reference launch controller, followed by the 6-degree left input.
    for tick in 0..5 * HZ {
        let error = 4.0 - bench.velocity().length();
        if error < 0.25 {
            integral = (integral + 42.0 * error / HZ as f64).clamp(0.0, 100.0);
        }
        let drive = (220.0 * error + integral).clamp(0.0, 400.0) as f32;
        let steer = 6.0_f32.to_radians()
            * ((tick as f32 - 3.0 * HZ as f32) / (0.5 * HZ as f32)).clamp(0.0, 1.0);
        bench.step(steer, drive, 0.0)?;
    }
    let travel = (bench.centre() - start).z;
    let turn = bench.heading();
    let brake_speed = bench.velocity().length();
    let mut stop = 0.0;
    let mut previous = bench.centre();
    for _ in 0..4 * HZ {
        bench.step(6.0_f32.to_radians(), 0.0, 250.0)?;
        let current = bench.centre();
        stop += (current - previous).with_y(0.0).length();
        previous = current;
        if bench.velocity().length() < 0.1 {
            break;
        }
    }
    if travel < 10.0
        || turn < 0.1
        || !(3.5..=4.5).contains(&brake_speed)
        || bench.velocity().length() > 0.1
        || !(0.5..=6.0).contains(&stop)
        || bench.trace().peak_travel > 0.12
    {
        return Err(format!("handling failed with travel {travel:.3} m, heading {turn:.3} rad, brake entry {brake_speed:.3} m/s, stop {stop:.3} m, final speed {:.3} m/s and travel {:.3} m",
            bench.velocity().length(), bench.trace().peak_travel).into());
    }
    Ok(Handling {
        support,
        sag,
        travel,
        turn,
        stop,
        peak_travel: bench.trace().peak_travel,
    })
}

fn compare_rates(a: Handling, b: Handling) -> Result<(), Box<dyn Error>> {
    if (a.travel - b.travel).abs() > 0.25
        || (a.turn - b.turn).abs() > 0.035
        || (a.stop - b.stop).abs() > 0.25
        || (a.peak_travel - b.peak_travel).abs() > 0.005
    {
        return Err(format!("handling varies across rates: {a:?} vs {b:?}").into());
    }
    Ok(())
}

fn alignment(
    bench: &mut Bench,
    tyre: &ReferenceTyre,
    road: &VehicleRoad,
) -> Result<(), Box<dyn Error>> {
    let axles = [
        Vec3::new(1.0, 0.035, 0.025),
        Vec3::new(1.0, -0.035, -0.025),
        Vec3::new(1.0, 0.020, 0.012),
        Vec3::new(1.0, -0.020, -0.012),
    ]
    .map(Vec3::normalize);
    let alignments = axles.map(|axle| WheelAlignment::from_axle(axle).unwrap());
    bench.spawn(&fixture(alignments)?, tyre, road, DVec3::ZERO, None)?;
    for _ in 0..2 * HZ {
        bench.step(0.0, 0.0, 0.0)?;
    }
    let steer = 6.0_f32.to_radians();
    for tick in 1..=HZ {
        bench.step(steer * (tick as f32 / (HZ / 2) as f32).min(1.0), 150.0, 0.0)?;
    }
    let car = bench.vehicle.as_ref().unwrap();
    let [left, right] = car.geometry.front_steer(steer).unwrap();
    let steering = [left, right, 0.0, 0.0];
    let world = bench.app.world();
    let mut max_error = 0.0_f64;
    for (i, wheel) in car.wheels.iter().enumerate() {
        let (_, frame) = bench.body().mount_frame(i).unwrap();
        let axle = world
            .get::<AvianTyre>(wheel.rim)
            .unwrap()
            .tyre()
            .rim()
            .rotation
            * DVec3::X;
        let local = frame.conjugate() * axle;
        let expected =
            bevy::math::DQuat::from_rotation_y(f64::from(steering[i])) * axles[i].as_dvec3();
        max_error = max_error.max(local.angle_between(expected).abs());
    }
    println!(
        "Authored alignment | maximum mount-relative axle error {:.4}°",
        max_error.to_degrees()
    );
    if max_error > 0.005 {
        return Err("authored alignment did not follow the mount".into());
    }
    Ok(())
}

fn rectangle(points: [DVec3; 4]) -> Result<Arc<StaticCollisionMesh>, Box<dyn Error>> {
    Ok(Arc::new(StaticCollisionMesh::new(&points, &FACE)?))
}

fn impacts(
    bench: &mut Bench,
    vehicle: &PreparedDeformableVehicle,
    tyre: &ReferenceTyre,
    road: &VehicleRoad,
) -> Result<(), Box<dyn Error>> {
    let wall = rectangle([
        DVec3::new(0.877, 0.08, 0.85),
        DVec3::new(0.877, 0.08, 1.65),
        DVec3::new(0.877, 0.65, 1.65),
        DVec3::new(0.877, 0.65, 0.85),
    ])?;
    bench.spawn(vehicle, tyre, road, 2.5 * DVec3::X, Some(wall))?;
    bench.step(0.0, 0.0, 0.0)?;
    let trace = bench.trace();
    let (wheel_step, wheel, load) = trace
        .first_wheel
        .ok_or("wheel missed the side wall in its first tick")?;
    let (mount, delta_p) = trace
        .wheel_transfer
        .ok_or("wheel strike did not transfer momentum before chassis contact")?;
    if load <= 0.0 || trace.max_balance_ns > 1e-7 {
        return Err(format!(
            "side strike lost support or impulse balance: {load} N, {} N·s",
            trace.max_balance_ns
        )
        .into());
    }
    println!(
        "Wheel-first strike | corner {} at step {wheel_step} | floor {load:.1} N | mount {mount:.3} N·s | chassis Δp {delta_p:.3} N·s | balance {:.2e} N·s",
        wheel + 1,
        trace.max_balance_ns
    );
    let post = |x: f64| {
        rectangle([
            DVec3::new(x - 0.06, 0.0, 1.98),
            DVec3::new(x + 0.06, 0.0, 1.98),
            DVec3::new(x + 0.06, 0.62, 1.98),
            DVec3::new(x - 0.06, 0.62, 1.98),
        ])
    };
    let centre_post = post(0.0)?;
    let mut first = None;
    for _ in 0..2 {
        bench.spawn(
            vehicle,
            tyre,
            road,
            8.0 * DVec3::Z,
            Some(centre_post.clone()),
        )?;
        let initial = nose_length(bench);
        let mut crush_sum = 0.0;
        for tick in 0..3 * HZ {
            bench.step(0.0, 0.0, 0.0)?;
            if tick >= 3 * HZ - HZ / 4 {
                crush_sum += initial - nose_length(bench);
            }
        }
        let trace = bench.trace();
        let (step, hit) = trace.first_chassis.ok_or("bumper missed the centre post")?;
        let (transfer, impulse, wheel_delta) = trace
            .body_transfer
            .ok_or("body strike did not slow the front wheels through their mounts")?;
        let crush = crush_sum / (HZ / 4) as f64;
        if !matches!(
            hit.feature,
            ContactFeature::Panel(_) | ContactFeature::PanelEdge { .. }
        ) || crush < 0.005
            || trace.first_wheel.is_some_and(|(step, ..)| step <= transfer)
            || trace.last_chassis_step >= trace.steps.saturating_sub(RATES[0] as usize / 4)
        {
            return Err(format!("post check failed at {step}, hit {hit:?}, crush {crush:.5} m, wheel contact {:?}, transfer step {transfer}", trace.first_wheel).into());
        }
        let result = (
            step,
            transfer,
            impulse,
            wheel_delta,
            trace.crash_loads_n,
            trace.obstacle_impulse_ns,
            crush,
        );
        if let Some(previous) = first {
            if result != previous {
                return Err(format!("post replay diverged: {previous:?} vs {result:?}").into());
            }
        } else {
            println!(
                "Bumper-first strike | {:?} at step {step} | front mounts {impulse:.3} N·s and wheel Δv {wheel_delta:.4} m/s at step {transfer} | obstacle {:.2?} N·s | residual crush {:.2} mm",
                hit.feature,
                trace.obstacle_impulse_ns.to_array(),
                crush * 1000.0
            );
        }
        first = Some(result);
    }
    bench.spawn(vehicle, tyre, road, 8.0 * DVec3::Z, Some(post(1.15)?))?;
    for _ in 0..HZ / 2 {
        bench.step(0.0, 0.0, 0.0)?;
    }
    if bench.trace().first_chassis.is_some() || bench.trace().first_wheel.is_some() {
        return Err("offset post struck the vehicle".into());
    }
    println!("Offset post | zero obstacle impulse | centred replay matched");
    Ok(())
}

fn nose_length(bench: &Bench) -> f64 {
    let nodes = bench.body().chassis().nodes();
    (0..4)
        .map(|i| nodes[i].position_m.distance(nodes[i + 4].position_m))
        .sum::<f64>()
        / 4.0
}

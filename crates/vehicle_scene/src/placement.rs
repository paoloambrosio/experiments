use bevy::prelude::Vec3;
use std::ops::Range;

use vehicle_assets::{Heightfield, TerrainAsset, TriangleMesh, VehicleAsset, ror::TerrainImport};

/// Positions a vehicle root at a terrain start with its authored rest pose intact.
pub fn park(terrain: &TerrainImport, vehicle: &VehicleAsset) -> Result<Vec3, String> {
    let hint = start_hint(terrain)?;
    let geometry = collision_geometry(&terrain.terrain);

    let support = |point: Vec3| support_below(&geometry.triangles, point.with_y(hint.y));
    support(hint).ok_or_else(|| {
        format!(
            "no supporting surface at terrain start ({:.3}, {:.3})",
            hint.x, hint.z
        )
    })?;

    let origin = Vec3::new(hint.x, 0.0, hint.z);
    let mut lift = f32::NEG_INFINITY;
    for (index, wheel) in vehicle.wheels.iter().enumerate() {
        // Evaluate the lowest tread points across the tyre width. The axle may
        // have authored camber, while the vehicle root stays upright.
        let down = (Vec3::NEG_Y + wheel.axle * wheel.axle.y).normalize_or_zero();
        for side in [-1.0, 0.0, 1.0] {
            let point =
                wheel.centre + down * wheel.outer_radius + wheel.axle * (side * wheel.width * 0.5);
            let surface = support(origin + point)
                .ok_or_else(|| format!("no supporting terrain beneath wheel {}", index + 1))?;
            lift = lift.max(surface - point.y);
        }
    }
    if !lift.is_finite() {
        return Err("the selected vehicle has no wheels for placement".into());
    }
    Ok(Vec3::new(origin.x, lift, origin.z))
}

pub fn start_hint(terrain: &TerrainImport) -> Result<Vec3, String> {
    Ok(if let Some(hint) = terrain.start_position {
        hint
    } else {
        let grid = terrain
            .terrain
            .heightfield
            .as_ref()
            .ok_or("terrain has neither a start position nor a heightfield centre")?;
        grid.origin
            + Vec3::new(
                (grid.samples_x - 1) as f32 * grid.spacing_x * 0.5,
                f32::INFINITY,
                (grid.samples_z - 1) as f32 * grid.spacing_z * 0.5,
            )
    })
}

pub struct CollisionGeometry {
    pub triangles: Vec<[Vec3; 3]>,
    pub objects: Vec<(String, Range<usize>)>,
}

pub fn collision_geometry(terrain: &TerrainAsset) -> CollisionGeometry {
    let ground = terrain.heightfield.as_ref().map(triangulate);
    let mut triangles = Vec::new();
    if let Some(mesh) = &ground {
        triangles.extend(
            mesh.triangles
                .iter()
                .map(|indices| indices.map(|id| mesh.vertices[id as usize])),
        );
    }
    let mut objects = Vec::new();
    for instance in &terrain.objects {
        if let Some(id) = instance.collision_mesh {
            let mesh = &terrain.meshes[id.0];
            let start = triangles.len();
            triangles.extend(mesh.triangles.iter().map(|indices| {
                indices.map(|id| instance.transform_point(mesh.vertices[id as usize]))
            }));
            objects.push((instance.name.clone(), start..triangles.len()));
        }
    }

    CollisionGeometry { triangles, objects }
}

pub fn support_below(triangles: &[[Vec3; 3]], point: Vec3) -> Option<f32> {
    triangles
        .iter()
        .filter_map(|triangle| height_at(*triangle, point))
        .filter(|height| *height <= point.y)
        .max_by(f32::total_cmp)
}

pub fn height_at([a, b, c]: [Vec3; 3], point: Vec3) -> Option<f32> {
    let cross_xz = |u: Vec3, v: Vec3| u.x * v.z - u.z * v.x;
    let (ab, ac, ap) = (b - a, c - a, point - a);
    let det = cross_xz(ab, ac);
    if det.abs() < 1e-8 {
        return None;
    }
    let u = cross_xz(ap, ac) / det;
    let v = cross_xz(ab, ap) / det;
    let epsilon = 1e-5;
    (u >= -epsilon && v >= -epsilon && u + v <= 1.0 + epsilon).then_some(a.y + u * ab.y + v * ac.y)
}

pub fn triangulate(grid: &Heightfield) -> TriangleMesh {
    let vertices = (0..grid.samples_z)
        .flat_map(|z| {
            (0..grid.samples_x)
                .map(move |x| grid.sample_position(x, z).expect("validated heightfield"))
        })
        .collect();
    let mut triangles = Vec::new();
    for z in 0..grid.samples_z - 1 {
        for x in 0..grid.samples_x - 1 {
            let a = (z * grid.samples_x + x) as u32;
            let b = a + 1;
            let c = a + grid.samples_x as u32;
            let d = c + 1;
            // OGRE alternates the diagonal on consecutive terrain rows.
            if z.is_multiple_of(2) {
                triangles.extend([[a, c, d], [a, d, b]]);
            } else {
                triangles.extend([[a, c, b], [b, c, d]]);
            }
        }
    }
    TriangleMesh {
        vertices,
        triangles,
        normals: None,
        uvs: None,
    }
}

#[cfg(test)]
mod tests {
    use bevy::prelude::Quat;
    use vehicle_assets::{MeshAppearance, MeshId, MeshInstance, WheelGeometry};

    use super::*;

    #[test]
    fn parks_above_the_collision_floor_and_below_the_roof() {
        let plane = TriangleMesh {
            vertices: vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 10.0),
                Vec3::new(10.0, 0.0, 10.0),
                Vec3::new(10.0, 0.0, 0.0),
            ],
            triangles: vec![[0, 1, 2], [0, 2, 3]],
            normals: None,
            uvs: None,
        };
        let instance = |name: &str, y| MeshInstance {
            name: name.into(),
            translation: Vec3::Y * y,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
            visual_mesh: None,
            collision_mesh: Some(MeshId(0)),
        };
        let mut terrain = TerrainImport {
            terrain: TerrainAsset {
                name: "fixture".into(),
                heightfield: Some(Heightfield {
                    origin: Vec3::ZERO,
                    samples_x: 2,
                    samples_z: 2,
                    spacing_x: 10.0,
                    spacing_z: 10.0,
                    heights: vec![0.0; 4],
                }),
                meshes: vec![plane],
                objects: vec![instance("floor", 2.0), instance("roof", 12.0)],
            },
            start_position: Some(Vec3::new(4.0, 8.0, 5.0)),
            appearance: MeshAppearance::default(),
            warnings: Vec::new(),
        };
        let vehicle = VehicleAsset {
            physics: Default::default(),
            name: "fixture".into(),
            meshes: Vec::new(),
            objects: Vec::new(),
            nodes: vec![Vec3::new(0.1, 1.0, 0.2)],
            edges: Vec::new(),
            wheels: vec![WheelGeometry {
                centre: Vec3::new(0.5, 1.0, 0.25),
                axle: Vec3::X,
                outer_radius: 0.3,
                width: 0.2,
                rim_radius: Some(0.15),
            }],
        };
        assert!(
            park(&terrain, &vehicle)
                .unwrap()
                .abs_diff_eq(Vec3::new(4.0, 1.3, 5.0), 1e-6)
        );
        assert_eq!(vehicle.nodes[0], Vec3::new(0.1, 1.0, 0.2));

        terrain.start_position = None;
        terrain.terrain.objects.clear();
        assert!(
            park(&terrain, &vehicle)
                .unwrap()
                .abs_diff_eq(Vec3::new(5.0, -0.7, 5.0), 1e-6)
        );

        terrain.start_position = Some(Vec3::new(30.0, 8.0, 30.0));
        assert!(
            park(&terrain, &vehicle)
                .unwrap_err()
                .contains("no supporting surface")
        );
    }
}

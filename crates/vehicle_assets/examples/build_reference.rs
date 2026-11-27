use std::{error::Error, fmt::Write as _, fs::File, io::Write, path::Path};

use bevy_math::Vec3;
use image::{ExtendedColorType, ImageEncoder, codecs::png::PngEncoder};
use zip::{ZipWriter, write::SimpleFileOptions};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
type Quad = [Vec3; 4];

fn main() -> Result<()> {
    let output = Path::new(env!("CARGO_MANIFEST_DIR")).join("reference");
    std::fs::create_dir_all(&output)?;
    write_car(&output.join("car.zip"))?;
    write_track(&output.join("track.zip"))?;
    Ok(())
}

fn entry(zip: &mut ZipWriter<File>, name: &str, bytes: impl AsRef<[u8]>) -> Result<()> {
    zip.start_file(name, SimpleFileOptions::default())?;
    zip.write_all(bytes.as_ref())?;
    Ok(())
}

fn png(width: u32, height: u32, pixels: &[u8]) -> Result<Vec<u8>> {
    let mut bytes = Vec::new();
    PngEncoder::new(&mut bytes).write_image(pixels, width, height, ExtendedColorType::Rgba8)?;
    Ok(bytes)
}

fn chunk(output: &mut Vec<u8>, id: u16, payload: &[u8]) {
    output.extend(id.to_le_bytes());
    output.extend(((payload.len() + 6) as u32).to_le_bytes());
    output.extend(payload);
}

// OGRE 1.41 static geometry with one section per material and flat face normals.
fn mesh(sections: &[(&str, &[Quad])]) -> Vec<u8> {
    let mut mesh = vec![0];
    for &(material, quads) in sections {
        let mut submesh = format!("{material}\n").into_bytes();
        submesh.push(0); // Each section owns its vertex buffer.
        submesh.extend((6 * quads.len() as u32).to_le_bytes());
        submesh.push(0); // Indices are u16.
        let mut vertices = Vec::new();
        for (i, &[a, b, c, d]) in quads.iter().enumerate() {
            for index in [0, 1, 2, 0, 2, 3] {
                submesh.extend((4 * i as u16 + index).to_le_bytes());
            }
            let normal = (b - a).cross(c - a).normalize();
            for (point, uv) in
                [a, b, c, d]
                    .into_iter()
                    .zip([[0., 0.], [1., 0.], [1., 1.], [0., 1.]])
            {
                for value in point
                    .to_array()
                    .into_iter()
                    .chain(normal.to_array())
                    .chain(uv)
                {
                    vertices.extend(value.to_le_bytes());
                }
            }
        }
        let mut geometry = (4 * quads.len() as u32).to_le_bytes().to_vec();
        let mut declaration = Vec::new();
        for (kind, semantic, offset) in [(2u16, 1u16, 0u16), (2, 4, 12), (1, 7, 24)] {
            let element: Vec<_> = [0, kind, semantic, offset, 0]
                .into_iter()
                .flat_map(u16::to_le_bytes)
                .collect();
            chunk(&mut declaration, 0x5110, &element);
        }
        chunk(&mut geometry, 0x5100, &declaration);
        let mut buffer = vec![0, 0, 32, 0];
        chunk(&mut buffer, 0x5210, &vertices);
        chunk(&mut geometry, 0x5200, &buffer);
        chunk(&mut submesh, 0x5000, &geometry);
        chunk(&mut submesh, 0x4010, &4u16.to_le_bytes());
        chunk(&mut mesh, 0x4000, &submesh);
    }
    let mut bytes = b"\x00\x10[MeshSerializer_v1.41]\n".to_vec();
    chunk(&mut bytes, 0x3000, &mesh);
    bytes
}

fn section(x: f32, low: f32, high: f32, z: f32) -> Quad {
    [
        Vec3::new(-x, low, z),
        Vec3::new(x, low, z),
        Vec3::new(x, high, z),
        Vec3::new(-x, high, z),
    ]
}

fn sides(rear: Quad, front: Quad) -> [Quad; 4] {
    std::array::from_fn(|i| {
        let next = (i + 1) % 4;
        [rear[i], rear[next], front[next], front[i]]
    })
}

const CORNERS: [(f32, f32); 4] = [(-0.75, -1.24), (0.75, 1.24), (0.75, -1.24), (-0.75, 1.24)];

fn truck(wide: bool) -> String {
    let name = if wide {
        "Reference Car Wide"
    } else {
        "Reference Car"
    };
    let mut text = format!("{name}\nglobals\n0,0\nminimass\n2\nnodes\n");
    let rear = section(0.68, 0.36, 0.72, -1.82);
    let bulkhead = section(0.68, 0.36, 0.72, 1.18);
    let nose = section(0.64, 0.38, 0.62, 2.0);
    let mut node = |id, p: Vec3, mass| {
        // RoR truck coordinates are -X forward, +Y up, +Z left.
        writeln!(text, "{id},{:.4},{:.4},{:.4},l {mass}", -p.z, p.y, p.x).unwrap();
    };
    for (i, point) in rear.into_iter().chain(bulkhead).chain(nose).enumerate() {
        let mass = if i < 8 { 78 } else { 14 };
        node(i, point, mass);
    }
    for (i, (x, z)) in CORNERS.into_iter().enumerate() {
        let i = 12 + 3 * i;
        let side = x.signum();
        let toe = if z > 0.0 { 0.002 } else { 0.0 };
        node(i, Vec3::new(x - side * 0.10, 0.36 + 0.004, z - toe), 3);
        node(i + 1, Vec3::new(x + side * 0.10, 0.36 - 0.004, z + toe), 3);
        node(i + 2, Vec3::new(x - side * 0.10, 0.76, z - toe), 12);
    }
    node(24, Vec3::new(-0.62, 0.85, 0.0), 10);
    node(25, Vec3::new(0.62, 0.85, 0.0), 10);
    text.push_str("beams\nenable_advanced_deformation\nset_beam_defaults 800000,900,60000,500000,0.04,tracks/beam,0.1\n");
    for a in 0..8 {
        for b in a + 1..8 {
            writeln!(text, "{a},{b}").unwrap();
        }
    }
    for (i, (x, z)) in CORNERS.into_iter().enumerate() {
        let axle = 12 + 3 * i;
        let mount = axle + 2;
        let end = if z > 0.0 { 4 } else { 0 };
        let other_end = 4 - end;
        let side = usize::from(x > 0.0);
        for chassis in [end + side, end + 2, end + 3, other_end + side] {
            writeln!(text, "{mount},{chassis}").unwrap();
        }
        writeln!(text, "{axle},{}\n{},{}", axle + 1, axle + 1, end + side).unwrap();
    }
    for node in [24, 25] {
        for anchor in [2, 3, 6, 7] {
            writeln!(text, "{node},{anchor}").unwrap();
        }
    }
    writeln!(text, "24,25").unwrap();
    text.push_str("set_beam_defaults 500000,2500,38000,500000,0.04,tracks/beam,0.1\n");
    for a in 4..12 {
        for b in (a + 1).max(8)..12 {
            let flags = if (a, b) == (4, 8) { ",s" } else { "" };
            writeln!(text, "{a},{b}{flags}").unwrap();
        }
    }
    text.push_str("shocks\n");
    for i in 0..4 {
        let axle = 12 + 3 * i;
        writeln!(text, "{},{axle},32000,2200,0.5,0.35,1.18,i", axle + 2).unwrap();
    }
    text.push_str("engine\n900,6200,110,3.8,-3,0,3.2\nbrakes\n550\nmeshwheels2\n");
    for (i, (x, z)) in CORNERS.into_iter().enumerate() {
        let axle = 12 + 3 * i;
        let side = if x > 0.0 { 'l' } else { 'r' };
        let driven = usize::from(z > 0.0);
        let (radius, rim, width) = match (wide, z > 0.0) {
            (true, true) => (0.33, 0.20, 0.225),
            (true, false) => (0.33, 0.20, 0.245),
            (false, _) => (0.31, 0.19, 0.205),
        };
        writeln!(text, "{radius},{rim},{width},16,{},{axle},9999,1,{driven},0,32,1000,100,{side},rim.mesh,tread", axle + 1).unwrap();
    }
    text.push_str("submesh\ncab\n8,9,10,c\n8,10,11,c\n7,6,25,c\n7,25,24,c\ncontacters\n8\n9\n10\n11\nflexbodies\n0,1,4,0,0,0,0,0,0,body.mesh\nforset 0-11,24,25\nmanagedmaterials\npaint flexmesh_standard paint.png\nglass flexmesh_transparent glass.png\nalloy mesh_standard alloy.png\ntread mesh_standard rubber.png\nend\n");
    text
}

fn write_car(path: &Path) -> Result<()> {
    let mut zip = ZipWriter::new(File::create(path)?);
    entry(&mut zip, "car.truck", truck(false))?;
    entry(&mut zip, "car-wide.truck", truck(true))?;
    let offset = Vec3::new(0.68, -0.36, 1.82);
    let local = |quad: Quad| quad.map(|point| point + offset);
    let rear = section(0.68, 0.36, 0.72, -1.82);
    let nose = section(0.64, 0.38, 0.62, 2.0);
    let mut paint = sides(rear, nose).map(local).to_vec();
    paint.extend([local([rear[3], rear[2], rear[1], rear[0]]), local(nose)]);
    let cabin_rear = section(0.53, 0.71, 0.71, -1.20);
    let cabin_front = section(0.53, 0.71, 0.71, 0.92);
    let roof_rear = section(0.43, 1.19, 1.19, -0.73);
    let roof_front = section(0.43, 1.19, 1.19, 0.35);
    paint.push(local([
        roof_rear[0],
        roof_front[0],
        roof_front[1],
        roof_rear[1],
    ]));
    let glass = [
        [cabin_front[0], cabin_front[1], roof_front[1], roof_front[0]],
        [cabin_rear[1], cabin_rear[0], roof_rear[0], roof_rear[1]],
        [cabin_rear[0], cabin_front[0], roof_front[0], roof_rear[0]],
        [cabin_front[1], cabin_rear[1], roof_rear[1], roof_front[1]],
    ]
    .map(local);
    entry(
        &mut zip,
        "body.mesh",
        mesh(&[("paint", &paint), ("glass", &glass)]),
    )?;
    let rim: Vec<_> = (0..16)
        .map(|i| {
            let point = |j: u32, x: f32| {
                let angle = j as f32 * std::f32::consts::TAU / 16.0;
                Vec3::new(x, 0.18 * angle.cos(), 0.18 * angle.sin())
            };
            [
                Vec3::new(0.092, 0., 0.),
                point(i, 0.092),
                point(i + 1, 0.092),
                point(i + 1, 0.083),
            ]
        })
        .collect();
    entry(&mut zip, "rim.mesh", mesh(&[("alloy", &rim)]))?;
    entry(
        &mut zip,
        "paint.png",
        png(
            2,
            2,
            &[
                24, 100, 151, 255, 24, 100, 151, 255, 40, 160, 198, 255, 40, 160, 198, 255,
            ],
        )?,
    )?;
    for (name, rgba) in [
        ("glass.png", [40, 70, 86, 206]),
        ("alloy.png", [172, 180, 188, 255]),
        ("rubber.png", [30, 33, 37, 255]),
    ] {
        entry(&mut zip, name, png(1, 1, &rgba)?)?;
    }
    zip.finish()?;
    Ok(())
}

fn write_track(path: &Path) -> Result<()> {
    let mut zip = ZipWriter::new(File::create(path)?);
    entry(
        &mut zip,
        "track.terrn2",
        "[General]\nName = Reference Test Track\nStartPosition = 34,3,10\nGeometryConfig = track.otc\n[Objects]\ntrack.tobj=\n",
    )?;
    entry(
        &mut zip,
        "track.otc",
        "Pages_X=0\nPages_Y=0\nWorldSizeX=90\nWorldSizeZ=130\nWorldSizeY=1\nHeightmapImage.0.0=level.png\n",
    )?;
    entry(&mut zip, "level.png", png(2, 2, &[0, 0, 0, 255].repeat(4))?)?;
    entry(
        &mut zip,
        "track.tobj",
        "16,0,21,0,0,0,ramp\n62,0,60,0,8,0,wall\n34,0,13,0,0,0,start-line\n",
    )?;
    entry(
        &mut zip,
        "ramp.odef",
        "ramp.mesh\n1,1,1\nstandard\nbeginmesh\nmesh ramp.mesh\nendmesh\nend\n",
    )?;
    entry(
        &mut zip,
        "wall.odef",
        "wall.mesh\n1,1,1\nstandard\nbeginmesh\nmesh wall-contact.mesh\nendmesh\nend\n",
    )?;
    entry(
        &mut zip,
        "start-line.odef",
        "start-line.mesh\n1,1,1\nstandard\nend\n",
    )?;
    let lane = |x, y, z| Vec3::new(x, y, z);
    let surface = |x: f32, z0, y0, z1, y1| {
        [
            lane(-x, y0, z0),
            lane(-x, y1, z1),
            lane(x, y1, z1),
            lane(x, y0, z0),
        ]
    };
    let ramp = [
        surface(3.5, 0.0, 0.0, 5.0, 0.55),
        surface(3.5, 5.0, 0.55, 7.0, 0.55),
        surface(3.5, 7.0, 0.55, 12.0, 0.0),
    ];
    let wall = section(4.0, 0.0, 2.5, 0.0);
    entry(&mut zip, "ramp.mesh", mesh(&[("asphalt", &ramp)]))?;
    entry(&mut zip, "wall.mesh", mesh(&[("barrier", &[wall])]))?;
    entry(
        &mut zip,
        "wall-contact.mesh",
        mesh(&[("collision", &[wall])]),
    )?;
    entry(
        &mut zip,
        "start-line.mesh",
        mesh(&[("line", &[surface(3.5, 0.0, 0.01, 0.35, 0.01)])]),
    )?;
    entry(
        &mut zip,
        "track.material",
        "material asphalt\n{\n technique\n {\n  pass\n  {\n   diffuse 0.20 0.22 0.25 1\n  }\n }\n}\nmaterial barrier\n{\n technique\n {\n  pass\n  {\n   cull_hardware none\n   texture_unit\n   {\n    texture barrier.png\n   }\n  }\n }\n}\nmaterial line\n{\n technique\n {\n  pass\n  {\n   diffuse 0.85 0.87 0.90 1\n  }\n }\n}\n",
    )?;
    entry(
        &mut zip,
        "barrier.png",
        png(
            2,
            2,
            &[
                225, 164, 33, 255, 42, 43, 46, 255, 42, 43, 46, 255, 225, 164, 33, 255,
            ],
        )?,
    )?;
    zip.finish()?;
    Ok(())
}

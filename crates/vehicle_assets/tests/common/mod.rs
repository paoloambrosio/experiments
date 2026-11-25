fn chunk(output: &mut Vec<u8>, id: u16, payload: &[u8]) {
    output.extend_from_slice(&id.to_le_bytes());
    output.extend_from_slice(&((payload.len() + 6) as u32).to_le_bytes());
    output.extend_from_slice(payload);
}

pub fn triangle_mesh(version: &str, materials: &[&str], height: f32) -> Vec<u8> {
    let mut mesh = vec![0];
    for (section, material) in materials.iter().enumerate() {
        let mut submesh = material.as_bytes().to_vec();
        submesh.extend_from_slice(&[b'\n', 0]);
        submesh.extend_from_slice(&3u32.to_le_bytes());
        let wide = section > 0;
        submesh.push(u8::from(wide));
        for index in [0u32, 1, 2] {
            let bytes = index.to_le_bytes();
            submesh.extend_from_slice(&bytes[..if wide { 4 } else { 2 }]);
        }

        let mut geometry = 3u32.to_le_bytes().to_vec();
        let mut declaration = Vec::new();
        for (ty, semantic, offset) in [(2u16, 1u16, 0u16), (2, 4, 12), (1, 7, 24)] {
            let element: Vec<_> = [0, ty, semantic, offset, 0]
                .into_iter()
                .flat_map(u16::to_le_bytes)
                .collect();
            chunk(&mut declaration, 0x5110, &element);
        }
        chunk(&mut geometry, 0x5100, &declaration);
        let mut buffer = vec![0, 0, 32, 0];
        let mut vertices = Vec::new();
        for [x, y, z] in [
            [1.0f32, 0.0, height],
            [2.0, 0.0, height],
            [1.0, 1.0, height],
        ] {
            let values = [x, y, z, 0.0, 0.0, 1.0, x - 1.0, y];
            vertices.extend(values.into_iter().flat_map(f32::to_le_bytes));
        }
        chunk(&mut buffer, 0x5210, &vertices);
        chunk(&mut geometry, 0x5200, &buffer);
        chunk(&mut submesh, 0x5000, &geometry);
        chunk(&mut submesh, 0x4010, &4u16.to_le_bytes());
        chunk(&mut mesh, 0x4000, &submesh);
    }
    let mut bytes = 0x1000u16.to_le_bytes().to_vec();
    bytes.extend_from_slice(format!("[MeshSerializer_v{version}]\n").as_bytes());
    chunk(&mut bytes, 0x3000, &mesh);
    bytes
}

// SPDX-License-Identifier: MIT

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use crate::{ExportError, ExportPart};

pub fn write_obj(
    parts: &[ExportPart],
    obj_path: &Path,
    mtl_path: &Path,
) -> Result<(), ExportError> {
    let mut obj = BufWriter::new(File::create(obj_path)?);
    let mut mtl = BufWriter::new(File::create(mtl_path)?);
    let mtl_name = mtl_path
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("assembly.mtl");
    writeln!(obj, "# SPDX-License-Identifier: MIT")?;
    writeln!(obj, "mtllib {mtl_name}")?;
    let mut vertex_offset = 1_u64;
    for (index, part) in parts.iter().enumerate() {
        let material = format!("part_{index}");
        writeln!(mtl, "newmtl {material}")?;
        writeln!(
            mtl,
            "Kd {:.6} {:.6} {:.6}",
            part.color_rgba[0], part.color_rgba[1], part.color_rgba[2]
        )?;
        writeln!(mtl, "d {:.6}", part.color_rgba[3])?;
        writeln!(mtl)?;
        writeln!(obj, "o {}", part.name)?;
        writeln!(obj, "usemtl {material}")?;
        for vertex in &part.mesh.vertices {
            let vertex = part.static_pose.transform_point(*vertex);
            writeln!(obj, "v {:.9} {:.9} {:.9}", vertex[0], vertex[1], vertex[2])?;
        }
        for triangle in &part.mesh.triangles {
            writeln!(
                obj,
                "f {} {} {}",
                u64::from(triangle[0]) + vertex_offset,
                u64::from(triangle[1]) + vertex_offset,
                u64::from(triangle[2]) + vertex_offset
            )?;
        }
        vertex_offset +=
            u64::try_from(part.mesh.vertices.len()).map_err(|_| ExportError::MeshTooLarge)?;
    }
    Ok(())
}

pub fn write_binary_stl(parts: &[ExportPart], path: &Path) -> Result<(), ExportError> {
    let triangle_count = parts
        .iter()
        .try_fold(0_u32, |count, part| {
            let triangles = u32::try_from(part.mesh.triangles.len()).ok()?;
            count.checked_add(triangles)
        })
        .ok_or(ExportError::MeshTooLarge)?;
    let mut writer = BufWriter::new(File::create(path)?);
    let mut header = [0_u8; 80];
    let label = b"Gimbal prototype; units=millimetres; STL is compatibility output";
    header[..label.len()].copy_from_slice(label);
    writer.write_all(&header)?;
    writer.write_all(&triangle_count.to_le_bytes())?;
    for part in parts {
        for triangle in &part.mesh.triangles {
            let a = part
                .static_pose
                .transform_point(part.mesh.vertices[triangle[0] as usize]);
            let b = part
                .static_pose
                .transform_point(part.mesh.vertices[triangle[1] as usize]);
            let c = part
                .static_pose
                .transform_point(part.mesh.vertices[triangle[2] as usize]);
            let normal = normal(a, b, c);
            for value in normal.into_iter().chain(a).chain(b).chain(c) {
                writer.write_all(&(value as f32).to_le_bytes())?;
            }
            writer.write_all(&0_u16.to_le_bytes())?;
        }
    }
    Ok(())
}

fn normal(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> [f64; 3] {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let cross = [
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    ];
    let length = (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt();
    if length <= f64::EPSILON {
        [0.0, 0.0, 0.0]
    } else {
        [cross[0] / length, cross[1] / length, cross[2] / length]
    }
}

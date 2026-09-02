// SPDX-License-Identifier: MIT

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use crate::{ExportError, ExportPart};

/// In-memory Wavefront OBJ geometry and its companion material library.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EncodedObj {
    pub obj: Vec<u8>,
    pub mtl: Vec<u8>,
}

/// Encodes an assembly as Wavefront OBJ/MTL without filesystem access.
///
/// `mtl_name` is written to the OBJ `mtllib` directive and should be the
/// filename used when the returned `mtl` bytes are eventually persisted.
pub fn encode_obj(parts: &[ExportPart], mtl_name: &str) -> Result<EncodedObj, ExportError> {
    let mut obj = Vec::new();
    let mut mtl = Vec::new();
    encode_obj_into(parts, mtl_name, &mut obj, &mut mtl)?;
    Ok(EncodedObj { obj, mtl })
}

/// Writes the bytes produced by [`encode_obj`] to an OBJ/MTL file pair.
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
    encode_obj_into(parts, mtl_name, &mut obj, &mut mtl)
}

fn encode_obj_into(
    parts: &[ExportPart],
    mtl_name: &str,
    obj: &mut impl Write,
    mtl: &mut impl Write,
) -> Result<(), ExportError> {
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

/// Encodes an assembly as unitless binary STL compatibility output.
///
/// Vertex coordinates remain in millimetres and the header records that
/// convention. 3MF remains the canonical FDM output because STL has no unit
/// field.
pub fn encode_binary_stl(parts: &[ExportPart]) -> Result<Vec<u8>, ExportError> {
    let mut bytes = Vec::new();
    encode_binary_stl_into(parts, &mut bytes)?;
    Ok(bytes)
}

/// Writes the bytes produced by [`encode_binary_stl`] to a file.
pub fn write_binary_stl(parts: &[ExportPart], path: &Path) -> Result<(), ExportError> {
    let mut writer = BufWriter::new(File::create(path)?);
    encode_binary_stl_into(parts, &mut writer)
}

fn encode_binary_stl_into(
    parts: &[ExportPart],
    writer: &mut impl Write,
) -> Result<(), ExportError> {
    let triangle_count = parts
        .iter()
        .try_fold(0_u32, |count, part| {
            let triangles = u32::try_from(part.mesh.triangles.len()).ok()?;
            count.checked_add(triangles)
        })
        .ok_or(ExportError::MeshTooLarge)?;
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

#[cfg(test)]
mod tests {
    use super::*;
    use gimbal_core::{
        Assembly, Body, ComponentDefinition, ComponentRole, DatumSet, FeatureBuilder, FrameGraph,
        Length, Manufacturing, Primitive3, RigidTransform, TriangleMesh,
    };

    fn part() -> ExportPart {
        let mut builder = FeatureBuilder::new();
        let unit = Length::positive_mm(1.0).unwrap();
        let solid = builder.primitive(Primitive3::Box {
            x: unit,
            y: unit,
            z: unit,
            centered: true,
        });
        let mut assembly = Assembly::new();
        let definition = assembly.add_definition(ComponentDefinition {
            name: "triangle".to_string(),
            role: ComponentRole::Cockpit,
            body: Body::Solid(solid),
            manufacturing: Manufacturing::Fdm,
            color_rgba: [0.25, 0.5, 0.75, 1.0],
            datums: DatumSet::default(),
        });
        let frames = FrameGraph::new();
        ExportPart {
            name: "triangle".to_string(),
            definition,
            mesh: TriangleMesh {
                vertices: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
                triangles: vec![[0, 1, 2]],
            },
            manufacturing: Manufacturing::Fdm,
            frame: frames.world(),
            local_pose: RigidTransform::IDENTITY,
            static_pose: RigidTransform::translated(1.0, 2.0, 3.0),
            color_rgba: [0.25, 0.5, 0.75, 1.0],
            semantics: Default::default(),
        }
    }

    #[test]
    fn obj_encoder_matches_filesystem_wrapper() {
        let parts = [part()];
        let encoded = encode_obj(&parts, "assembly.mtl").unwrap();
        let directory =
            std::env::temp_dir().join(format!("gimbal-export-{}-obj-encoder", std::process::id()));
        std::fs::create_dir_all(&directory).unwrap();
        let obj_path = directory.join("assembly.obj");
        let mtl_path = directory.join("assembly.mtl");

        write_obj(&parts, &obj_path, &mtl_path).unwrap();

        assert_eq!(std::fs::read(&obj_path).unwrap(), encoded.obj);
        assert_eq!(std::fs::read(&mtl_path).unwrap(), encoded.mtl);
        std::fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn binary_stl_encoder_matches_filesystem_wrapper() {
        let parts = [part()];
        let encoded = encode_binary_stl(&parts).unwrap();
        let path = std::env::temp_dir().join(format!(
            "gimbal-export-{}-stl-encoder.stl",
            std::process::id()
        ));

        write_binary_stl(&parts, &path).unwrap();

        assert_eq!(std::fs::read(&path).unwrap(), encoded);
        assert_eq!(encoded.len(), 84 + 50);
        std::fs::remove_file(path).unwrap();
    }
}

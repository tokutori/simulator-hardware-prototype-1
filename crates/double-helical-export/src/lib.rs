// SPDX-License-Identifier: MIT

use std::fmt::Write as _;
use std::fs::File;
use std::io::{BufWriter, Write as _};
use std::path::Path;

use double_helical_core::{GearPose, TriangleMesh};
use thiserror::Error;
use zip::write::SimpleFileOptions;
use zip::{CompressionMethod, DateTime, ZipWriter};

const CONTENT_TYPES_XML: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>"#;

const ROOT_RELATIONSHIPS_XML: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>"#;

#[derive(Clone, Copy, Debug)]
pub struct ExportPart<'a> {
    pub name: &'a str,
    pub mesh: &'a TriangleMesh,
    pub pose: GearPose,
    pub color_rgb: [f64; 3],
}

#[derive(Debug, Error)]
pub enum ExportError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("mesh is too large for this output format")]
    MeshTooLarge,
    #[error("3MF error: {0}")]
    ThreeMf(String),
    #[error("at least one mesh part is required")]
    EmptyAssembly,
}

pub fn write_binary_stl(parts: &[ExportPart<'_>], path: &Path) -> Result<(), ExportError> {
    if parts.is_empty() {
        return Err(ExportError::EmptyAssembly);
    }
    let triangle_count = parts
        .iter()
        .try_fold(0_u32, |count, part| {
            let triangles = u32::try_from(part.mesh.triangles.len()).ok()?;
            count.checked_add(triangles)
        })
        .ok_or(ExportError::MeshTooLarge)?;
    let mut writer = BufWriter::new(File::create(path)?);
    let mut header = [0_u8; 80];
    let label = b"Double-helical gear experiment; units=millimetres";
    header[..label.len()].copy_from_slice(label);
    writer.write_all(&header)?;
    writer.write_all(&triangle_count.to_le_bytes())?;
    for part in parts {
        for triangle in &part.mesh.triangles {
            let a = transform(part.mesh.vertices[triangle[0] as usize], part.pose);
            let b = transform(part.mesh.vertices[triangle[1] as usize], part.pose);
            let c = transform(part.mesh.vertices[triangle[2] as usize], part.pose);
            let normal = normal(a, b, c);
            for value in normal.into_iter().chain(a).chain(b).chain(c) {
                writer.write_all(&(value as f32).to_le_bytes())?;
            }
            writer.write_all(&0_u16.to_le_bytes())?;
        }
    }
    Ok(())
}

pub fn write_obj(
    parts: &[ExportPart<'_>],
    obj_path: &Path,
    mtl_path: &Path,
) -> Result<(), ExportError> {
    if parts.is_empty() {
        return Err(ExportError::EmptyAssembly);
    }
    let mut obj = BufWriter::new(File::create(obj_path)?);
    let mut mtl = BufWriter::new(File::create(mtl_path)?);
    let mtl_name = mtl_path
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("gear-set.mtl");
    writeln!(obj, "# SPDX-License-Identifier: MIT")?;
    writeln!(obj, "# Units: millimetres")?;
    writeln!(obj, "mtllib {mtl_name}")?;
    let mut vertex_offset = 1_u64;
    for (index, part) in parts.iter().enumerate() {
        let material = format!("gear_{index}");
        writeln!(mtl, "newmtl {material}")?;
        writeln!(
            mtl,
            "Kd {:.6} {:.6} {:.6}",
            part.color_rgb[0], part.color_rgb[1], part.color_rgb[2]
        )?;
        writeln!(mtl)?;
        writeln!(obj, "o {}", part.name)?;
        writeln!(obj, "usemtl {material}")?;
        for vertex in &part.mesh.vertices {
            let vertex = transform(*vertex, part.pose);
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

pub fn write_3mf(parts: &[ExportPart<'_>], path: &Path) -> Result<(), ExportError> {
    if parts.is_empty() {
        return Err(ExportError::EmptyAssembly);
    }
    let model_xml = model_xml(parts)?;
    let file = File::create(path)?;
    let mut archive = ZipWriter::new(file);
    let options = SimpleFileOptions::default()
        .compression_method(CompressionMethod::Deflated)
        .last_modified_time(DateTime::default());
    write_zip_entry(
        &mut archive,
        "[Content_Types].xml",
        CONTENT_TYPES_XML,
        options,
    )?;
    write_zip_entry(&mut archive, "_rels/.rels", ROOT_RELATIONSHIPS_XML, options)?;
    write_zip_entry(&mut archive, "3D/3dmodel.model", &model_xml, options)?;
    archive
        .finish()
        .map_err(|error| ExportError::ThreeMf(error.to_string()))?;
    Ok(())
}

fn model_xml(parts: &[ExportPart<'_>]) -> Result<String, ExportError> {
    let mut xml = String::from(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"><resources>"#,
    );
    for (index, part) in parts.iter().enumerate() {
        let id = u32::try_from(index + 1).map_err(|_| ExportError::MeshTooLarge)?;
        write!(
            xml,
            "<object id=\"{id}\" type=\"model\" name=\"{}\"><mesh><vertices>",
            escape_xml_attribute(part.name)
        )
        .expect("writing to a String cannot fail");
        for vertex in &part.mesh.vertices {
            let vertex = transform(*vertex, part.pose);
            write!(
                xml,
                "<vertex x=\"{}\" y=\"{}\" z=\"{}\"/>",
                vertex[0], vertex[1], vertex[2]
            )
            .expect("writing to a String cannot fail");
        }
        xml.push_str("</vertices><triangles>");
        for triangle in &part.mesh.triangles {
            write!(
                xml,
                "<triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>",
                triangle[0], triangle[1], triangle[2]
            )
            .expect("writing to a String cannot fail");
        }
        xml.push_str("</triangles></mesh></object>");
    }
    xml.push_str("</resources><build>");
    for index in 0..parts.len() {
        let id = u32::try_from(index + 1).map_err(|_| ExportError::MeshTooLarge)?;
        write!(xml, "<item objectid=\"{id}\"/>").expect("writing to a String cannot fail");
    }
    xml.push_str("</build></model>");
    Ok(xml)
}

fn write_zip_entry(
    archive: &mut ZipWriter<File>,
    name: &str,
    contents: &str,
    options: SimpleFileOptions,
) -> Result<(), ExportError> {
    archive
        .start_file(name, options)
        .map_err(|error| ExportError::ThreeMf(error.to_string()))?;
    archive.write_all(contents.as_bytes())?;
    Ok(())
}

fn transform(vertex: [f64; 3], pose: GearPose) -> [f64; 3] {
    let radians = pose.rotation_z_deg.to_radians();
    let cosine = radians.cos();
    let sine = radians.sin();
    [
        cosine * vertex[0] - sine * vertex[1] + pose.translation_mm[0],
        sine * vertex[0] + cosine * vertex[1] + pose.translation_mm[1],
        vertex[2] + pose.translation_mm[2],
    ]
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

fn escape_xml_attribute(value: &str) -> String {
    let mut escaped = String::with_capacity(value.len());
    for character in value.chars() {
        match character {
            '&' => escaped.push_str("&amp;"),
            '<' => escaped.push_str("&lt;"),
            '>' => escaped.push_str("&gt;"),
            '"' => escaped.push_str("&quot;"),
            '\'' => escaped.push_str("&apos;"),
            _ => escaped.push(character),
        }
    }
    escaped
}

#[cfg(test)]
mod tests {
    use std::io::Read;

    use super::*;

    fn triangle() -> TriangleMesh {
        TriangleMesh {
            vertices: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            triangles: vec![[0, 1, 2]],
        }
    }

    fn part<'a>(mesh: &'a TriangleMesh) -> ExportPart<'a> {
        ExportPart {
            name: "test & fixture",
            mesh,
            pose: GearPose {
                translation_mm: [2.0, 3.0, 4.0],
                rotation_z_deg: 90.0,
            },
            color_rgb: [0.2, 0.4, 0.8],
        }
    }

    #[test]
    fn writes_deterministic_millimetre_3mf() {
        let path = std::env::temp_dir().join(format!(
            "double-helical-export-{}-minimal.3mf",
            std::process::id()
        ));
        let mesh = triangle();
        write_3mf(&[part(&mesh)], &path).unwrap();
        let first = std::fs::read(&path).unwrap();
        write_3mf(&[part(&mesh)], &path).unwrap();
        assert_eq!(first, std::fs::read(&path).unwrap());

        let file = File::open(&path).unwrap();
        let mut archive = zip::ZipArchive::new(file).unwrap();
        let mut model = String::new();
        archive
            .by_name("3D/3dmodel.model")
            .unwrap()
            .read_to_string(&mut model)
            .unwrap();
        assert!(model.contains("unit=\"millimeter\""));
        assert!(model.contains("name=\"test &amp; fixture\""));
        assert!(model.contains("x=\"2\" y=\"3\" z=\"4\""));
        drop(archive);
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn stl_triangle_count_matches_parts() {
        let path = std::env::temp_dir().join(format!(
            "double-helical-export-{}-minimal.stl",
            std::process::id()
        ));
        let mesh = triangle();
        write_binary_stl(&[part(&mesh), part(&mesh)], &path).unwrap();
        let bytes = std::fs::read(&path).unwrap();
        assert_eq!(u32::from_le_bytes(bytes[80..84].try_into().unwrap()), 2);
        assert_eq!(bytes.len(), 84 + 2 * 50);
        std::fs::remove_file(path).unwrap();
    }
}

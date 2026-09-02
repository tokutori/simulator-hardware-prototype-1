// SPDX-License-Identifier: MIT

use std::fmt::Write as _;
use std::io::{Cursor, Seek, Write as IoWrite};
use std::path::Path;

use gimbal_core::TriangleMesh;
use zip::write::SimpleFileOptions;
use zip::{CompressionMethod, DateTime, ZipWriter};

use crate::{ExportError, ExportPart};

const CONTENT_TYPES_XML: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>"#;

const ROOT_RELATIONSHIPS_XML: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>"#;

struct ThreeMfObject {
    name: String,
    vertices: Vec<[f64; 3]>,
    triangles: Vec<[u32; 3]>,
}

/// Encodes an assembly as a deterministic, millimetre-based 3MF package.
pub fn encode_3mf(parts: &[ExportPart]) -> Result<Vec<u8>, ExportError> {
    let objects = parts
        .iter()
        .map(|part| ThreeMfObject {
            name: part.name.clone(),
            vertices: part
                .mesh
                .vertices
                .iter()
                .map(|vertex| part.static_pose.transform_point(*vertex))
                .collect(),
            triangles: part.mesh.triangles.clone(),
        })
        .collect::<Vec<_>>();
    encode_package(&objects)
}

/// Writes the bytes produced by [`encode_3mf`] to a file.
pub fn write_3mf(parts: &[ExportPart], path: &Path) -> Result<(), ExportError> {
    std::fs::write(path, encode_3mf(parts)?)?;
    Ok(())
}

/// Encodes one local-coordinate mesh as a deterministic 3MF package.
pub fn encode_mesh_3mf(name: &str, mesh: &TriangleMesh) -> Result<Vec<u8>, ExportError> {
    encode_package(&[ThreeMfObject {
        name: name.to_string(),
        vertices: mesh.vertices.clone(),
        triangles: mesh.triangles.clone(),
    }])
}

/// Writes the bytes produced by [`encode_mesh_3mf`] to a file.
pub fn write_mesh_3mf(name: &str, mesh: &TriangleMesh, path: &Path) -> Result<(), ExportError> {
    std::fs::write(path, encode_mesh_3mf(name, mesh)?)?;
    Ok(())
}

fn encode_package(objects: &[ThreeMfObject]) -> Result<Vec<u8>, ExportError> {
    if objects.is_empty() {
        return Err(ExportError::ThreeMf(
            "a 3MF package must contain at least one object".to_string(),
        ));
    }
    let model_xml = model_xml(objects)?;
    let mut archive = ZipWriter::new(Cursor::new(Vec::new()));
    let options = SimpleFileOptions::default()
        .compression_method(CompressionMethod::Deflated)
        .last_modified_time(DateTime::default());
    write_entry(
        &mut archive,
        "[Content_Types].xml",
        CONTENT_TYPES_XML,
        options,
    )?;
    write_entry(&mut archive, "_rels/.rels", ROOT_RELATIONSHIPS_XML, options)?;
    write_entry(&mut archive, "3D/3dmodel.model", &model_xml, options)?;
    let cursor = archive
        .finish()
        .map_err(|error| ExportError::ThreeMf(error.to_string()))?;
    Ok(cursor.into_inner())
}

fn write_entry<W: IoWrite + Seek>(
    archive: &mut ZipWriter<W>,
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

fn model_xml(objects: &[ThreeMfObject]) -> Result<String, ExportError> {
    let mut xml = String::from(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"><resources>"#,
    );
    for (index, object) in objects.iter().enumerate() {
        let id = u32::try_from(index + 1)
            .map_err(|_| ExportError::ThreeMf("too many 3MF objects".to_string()))?;
        write!(
            xml,
            "<object id=\"{id}\" type=\"model\" name=\"{}\"><mesh><vertices>",
            escape_xml_attribute(&object.name)
        )
        .expect("writing to a String cannot fail");
        for vertex in &object.vertices {
            write!(
                xml,
                "<vertex x=\"{}\" y=\"{}\" z=\"{}\"/>",
                vertex[0], vertex[1], vertex[2]
            )
            .expect("writing to a String cannot fail");
        }
        xml.push_str("</vertices><triangles>");
        for triangle in &object.triangles {
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
    for index in 0..objects.len() {
        let id = u32::try_from(index + 1)
            .map_err(|_| ExportError::ThreeMf("too many 3MF build items".to_string()))?;
        write!(xml, "<item objectid=\"{id}\"/>").expect("writing to a String cannot fail");
    }
    xml.push_str("</build></model>");
    Ok(xml)
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
    use std::io::{Cursor, Read};

    use super::*;

    #[test]
    fn writes_minimal_millimetre_3mf_package() {
        let path =
            std::env::temp_dir().join(format!("gimbal-export-{}-minimal.3mf", std::process::id()));
        let mesh = TriangleMesh {
            vertices: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            triangles: vec![[0, 1, 2]],
        };
        let encoded = encode_mesh_3mf("test & fixture", &mesh).unwrap();
        write_mesh_3mf("test & fixture", &mesh, &path).unwrap();
        let first_bytes = std::fs::read(&path).unwrap();
        assert_eq!(first_bytes, encoded);
        write_mesh_3mf("test & fixture", &mesh, &path).unwrap();
        assert_eq!(first_bytes, std::fs::read(&path).unwrap());

        let mut archive = zip::ZipArchive::new(Cursor::new(encoded)).unwrap();
        assert!(archive.by_name("[Content_Types].xml").is_ok());
        assert!(archive.by_name("_rels/.rels").is_ok());
        let mut model = String::new();
        archive
            .by_name("3D/3dmodel.model")
            .unwrap()
            .read_to_string(&mut model)
            .unwrap();
        assert!(model.contains("unit=\"millimeter\""));
        assert!(model.contains("name=\"test &amp; fixture\""));
        assert!(model.contains("<triangle v1=\"0\" v2=\"1\" v3=\"2\"/>"));
        drop(archive);
        std::fs::remove_file(path).unwrap();
    }
}

// SPDX-License-Identifier: MIT

use std::io::Cursor;
use std::path::Path;

use chrono::{Local, NaiveDate, TimeZone, Utc};
use dxf::entities::{Entity, EntityType, LwPolyline};
use dxf::enums::{AcadVersion, DrawingUnits, Units};
use dxf::tables::Layer;
use dxf::{Drawing, LwPolylineVertex};
use gimbal_core::Point2;

use crate::ExportError;

/// Encodes one closed CUT-layer profile as an R2013 millimetre DXF.
pub fn encode_dxf_profile(points: &[Point2]) -> Result<Vec<u8>, ExportError> {
    encode_dxf_sheet_profile(points, &[])
}

/// Encodes an outer contour and zero or more cutouts as an R2013 millimetre DXF.
pub fn encode_dxf_sheet_profile(
    outer: &[Point2],
    cutouts: &[&[Point2]],
) -> Result<Vec<u8>, ExportError> {
    if outer.len() < 3 || cutouts.iter().any(|points| points.len() < 3) {
        return Err(ExportError::InvalidProfile);
    }
    let mut drawing = Drawing::new();
    normalize_generated_header(&mut drawing)?;
    drawing.header.version = AcadVersion::R2013;
    drawing.header.default_drawing_units = Units::Millimeters;
    drawing.header.drawing_units = DrawingUnits::Metric;

    let layer = Layer {
        name: "CUT".to_string(),
        ..Default::default()
    };
    drawing.add_layer(layer);

    for points in core::iter::once(outer).chain(cutouts.iter().copied()) {
        let mut polyline = LwPolyline::default();
        polyline.set_is_closed(true);
        polyline
            .vertices
            .extend(points.iter().map(|point| LwPolylineVertex {
                x: point.x,
                y: point.y,
                ..Default::default()
            }));
        let mut entity = Entity::new(EntityType::LwPolyline(polyline));
        entity.common.layer = "CUT".to_string();
        drawing.add_entity(entity);
    }
    let mut bytes = Vec::new();
    drawing
        .save(&mut bytes)
        .map_err(|error| ExportError::Dxf(error.to_string()))?;
    let expected_vertices = core::iter::once(outer.len())
        .chain(cutouts.iter().map(|points| points.len()))
        .collect::<Vec<_>>();
    validate_dxf_bytes(&bytes, &expected_vertices)?;
    Ok(bytes)
}

/// Writes the bytes produced by [`encode_dxf_profile`] to a file.
pub fn write_dxf_profile(points: &[Point2], path: &Path) -> Result<(), ExportError> {
    std::fs::write(path, encode_dxf_profile(points)?)?;
    Ok(())
}

/// Writes the bytes produced by [`encode_dxf_sheet_profile`] to a file.
pub fn write_dxf_sheet_profile(
    outer: &[Point2],
    cutouts: &[&[Point2]],
    path: &Path,
) -> Result<(), ExportError> {
    std::fs::write(path, encode_dxf_sheet_profile(outer, cutouts)?)?;
    Ok(())
}

fn normalize_generated_header(drawing: &mut Drawing) -> Result<(), ExportError> {
    let date = NaiveDate::from_ymd_opt(2000, 1, 1)
        .and_then(|date| date.and_hms_opt(0, 0, 0))
        .ok_or(ExportError::InvalidDxf("fixed DXF date is invalid"))?;
    drawing.header.creation_date = Local
        .from_local_datetime(&date)
        .single()
        .ok_or(ExportError::InvalidDxf("fixed local DXF date is ambiguous"))?;
    drawing.header.update_date = drawing.header.creation_date;
    drawing.header.creation_date_universal = Utc.from_utc_datetime(&date);
    drawing.header.update_date_universal = drawing.header.creation_date_universal;
    drawing.header.fingerprint_guid = uuid::Uuid::nil();
    drawing.header.version_guid = uuid::Uuid::nil();
    Ok(())
}

#[cfg(test)]
fn validate_dxf_profile(path: &Path, expected_vertices: &[usize]) -> Result<(), ExportError> {
    let bytes = std::fs::read(path)?;
    validate_dxf_bytes(&bytes, expected_vertices)
}

fn validate_dxf_bytes(bytes: &[u8], expected_vertices: &[usize]) -> Result<(), ExportError> {
    let mut reader = Cursor::new(bytes);
    let drawing =
        Drawing::load(&mut reader).map_err(|error| ExportError::Dxf(error.to_string()))?;
    if drawing.header.default_drawing_units != Units::Millimeters {
        return Err(ExportError::InvalidDxf("INSUNITS is not millimetres"));
    }
    let entities = drawing.entities().collect::<Vec<_>>();
    if entities.len() != expected_vertices.len()
        || entities.iter().any(|entity| entity.common.layer != "CUT")
    {
        return Err(ExportError::InvalidDxf(
            "expected one CUT entity per sheet contour",
        ));
    }
    for (entity, expected_vertices) in entities.iter().zip(expected_vertices) {
        match &entity.specific {
            EntityType::LwPolyline(polyline)
                if polyline.is_closed() && polyline.vertices.len() == *expected_vertices => {}
            _ => {
                return Err(ExportError::InvalidDxf(
                    "CUT entity is not the expected closed polyline",
                ));
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn writes_and_reloads_closed_millimetre_profile() {
        let path =
            std::env::temp_dir().join(format!("gimbal-export-{}-profile.dxf", std::process::id()));
        let points = [
            Point2 { x: 0.0, y: 0.0 },
            Point2 { x: 10.0, y: 0.0 },
            Point2 { x: 10.0, y: 5.0 },
            Point2 { x: 0.0, y: 5.0 },
        ];
        let encoded = encode_dxf_profile(&points).unwrap();
        write_dxf_profile(&points, &path).unwrap();
        assert_eq!(std::fs::read(&path).unwrap(), encoded);
        validate_dxf_profile(&path, &[points.len()]).unwrap();
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn writes_and_reloads_outer_and_cutout_contours() {
        let path = std::env::temp_dir().join(format!(
            "gimbal-export-{}-sheet-with-hole.dxf",
            std::process::id()
        ));
        let outer = [
            Point2 { x: 0.0, y: 0.0 },
            Point2 { x: 20.0, y: 0.0 },
            Point2 { x: 20.0, y: 10.0 },
            Point2 { x: 0.0, y: 10.0 },
        ];
        let cutout = [
            Point2 { x: 8.0, y: 4.0 },
            Point2 { x: 12.0, y: 4.0 },
            Point2 { x: 12.0, y: 6.0 },
            Point2 { x: 8.0, y: 6.0 },
        ];
        let encoded = encode_dxf_sheet_profile(&outer, &[&cutout]).unwrap();
        write_dxf_sheet_profile(&outer, &[&cutout], &path).unwrap();
        assert_eq!(std::fs::read(&path).unwrap(), encoded);
        validate_dxf_profile(&path, &[outer.len(), cutout.len()]).unwrap();
        std::fs::remove_file(path).unwrap();
    }
}

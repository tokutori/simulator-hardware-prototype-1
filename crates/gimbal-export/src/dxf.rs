// SPDX-License-Identifier: MIT

use std::path::Path;

use dxf::entities::{Entity, EntityType, LwPolyline};
use dxf::enums::{AcadVersion, DrawingUnits, Units};
use dxf::tables::Layer;
use dxf::{Drawing, LwPolylineVertex};
use gimbal_core::Point2;

use crate::ExportError;

pub fn write_dxf_profile(points: &[Point2], path: &Path) -> Result<(), ExportError> {
    write_dxf_sheet_profile(points, &[], path)
}

pub fn write_dxf_sheet_profile(
    outer: &[Point2],
    cutouts: &[&[Point2]],
    path: &Path,
) -> Result<(), ExportError> {
    if outer.len() < 3 || cutouts.iter().any(|points| points.len() < 3) {
        return Err(ExportError::InvalidProfile);
    }
    let mut drawing = Drawing::new();
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
    drawing
        .save_file(path)
        .map_err(|error| ExportError::Dxf(error.to_string()))?;
    let expected_vertices = core::iter::once(outer.len())
        .chain(cutouts.iter().map(|points| points.len()))
        .collect::<Vec<_>>();
    validate_dxf_profile(path, &expected_vertices)
}

fn validate_dxf_profile(path: &Path, expected_vertices: &[usize]) -> Result<(), ExportError> {
    let drawing = Drawing::load_file(path).map_err(|error| ExportError::Dxf(error.to_string()))?;
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
        write_dxf_profile(&points, &path).unwrap();
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
        write_dxf_sheet_profile(&outer, &[&cutout], &path).unwrap();
        validate_dxf_profile(&path, &[outer.len(), cutout.len()]).unwrap();
        std::fs::remove_file(path).unwrap();
    }
}

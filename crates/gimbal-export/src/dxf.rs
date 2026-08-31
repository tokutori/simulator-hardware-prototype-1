// SPDX-License-Identifier: MIT

use std::path::Path;

use dxf::entities::{Entity, EntityType, LwPolyline};
use dxf::enums::{AcadVersion, DrawingUnits, Units};
use dxf::tables::Layer;
use dxf::{Drawing, LwPolylineVertex};
use gimbal_core::Point2;

use crate::ExportError;

pub fn write_dxf_profile(points: &[Point2], path: &Path) -> Result<(), ExportError> {
    if points.len() < 3 {
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
    drawing
        .save_file(path)
        .map_err(|error| ExportError::Dxf(error.to_string()))?;
    validate_dxf_profile(path, points.len())
}

fn validate_dxf_profile(path: &Path, expected_vertices: usize) -> Result<(), ExportError> {
    let drawing = Drawing::load_file(path).map_err(|error| ExportError::Dxf(error.to_string()))?;
    if drawing.header.default_drawing_units != Units::Millimeters {
        return Err(ExportError::InvalidDxf("INSUNITS is not millimetres"));
    }
    let entities = drawing.entities().collect::<Vec<_>>();
    if entities.len() != 1 || entities[0].common.layer != "CUT" {
        return Err(ExportError::InvalidDxf("expected exactly one CUT entity"));
    }
    match &entities[0].specific {
        EntityType::LwPolyline(polyline)
            if polyline.is_closed() && polyline.vertices.len() == expected_vertices =>
        {
            Ok(())
        }
        _ => Err(ExportError::InvalidDxf(
            "CUT entity is not the expected closed polyline",
        )),
    }
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
        validate_dxf_profile(&path, points.len()).unwrap();
        std::fs::remove_file(path).unwrap();
    }
}

// SPDX-License-Identifier: MIT

use crate::config::LoadedConfig;
use crate::manifest::{optional_artifact_paths, staged_artifact_manifest};
use crate::output::{prepare_staging_output, publish_staging_output};
use crate::validate::{
    require_valid_assembly, validate_assembly, validation_report_json, write_validation_report_to,
};
use geared_gimbal_design::build_prototype;
use gimbal_core::{Angle, Body, Manufacturing, PitchRollCommand, RegionNode, TriangleMesh};
use gimbal_export::{
    AnimationParameters, ExportPart, ExportSemantics, write_3mf, write_animated_gltf,
    write_binary_stl, write_dxf_sheet_profile, write_mesh_3mf, write_obj,
};
use gimbal_kernel_manifold::{Evaluator, ValidationProfile};
use serde_json::json;
use std::error::Error;
use std::fs;
use std::path::Path;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum GenerationMode {
    Validated,
    PreviewOnly,
}

pub(crate) fn generate(
    workspace: &Path,
    loaded: &LoadedConfig,
    mode: GenerationMode,
) -> Result<(), Box<dyn Error>> {
    let design = build_prototype(&loaded.parameters)
        .map_err(|error| format!("prototype design rejected: {error:?}"))?;
    let validation_report = if mode == GenerationMode::Validated {
        let report = validate_assembly(&design, ValidationProfile::EXACT_STATIC)?;
        if let Err(error) = require_valid_assembly(&report) {
            crate::validate::write_validation_report(workspace, &design, &report)?;
            return Err(error);
        }
        Some(report)
    } else {
        None
    };
    let output = prepare_staging_output(workspace)?;
    if let Some(report) = &validation_report {
        write_validation_report_to(&output, &design, report)?;
    }
    let model_dir = output.join("model");
    let animation_dir = output.join("animation");
    let preview_dir = output.join("preview");
    let fdm_dir = output.join("fabrication").join("fdm");
    let laser_dir = output.join("fabrication").join("laser");
    for directory in [
        &model_dir,
        &animation_dir,
        &preview_dir,
        &fdm_dir,
        &laser_dir,
    ] {
        fs::create_dir_all(directory)?;
    }

    let mut evaluator = Evaluator::new(&design.graph);
    let mut definition_meshes =
        Vec::<TriangleMesh>::with_capacity(design.assembly.definitions().len());
    let mut definition_manifest = Vec::new();
    let mut fabrication_artifacts = Vec::new();
    for (definition_index, definition) in design.assembly.definitions().iter().enumerate() {
        let solid = definition.body.assembly_solid();
        let mesh = evaluator.mesh(solid)?;
        let metrics = evaluator.metrics(solid)?;
        let quantity = design
            .assembly
            .instances()
            .iter()
            .filter(|instance| instance.definition.index() == definition_index)
            .count();
        definition_manifest.push(json!({
            "name": definition.name,
            "role": format!("{:?}", definition.role),
            "manufacturing": manufacturing_name(definition.manufacturing),
            "quantity": quantity,
            "vertices": metrics.vertices,
            "triangles": metrics.triangles,
            "volume_mm3": metrics.volume_mm3,
            "surface_area_mm2": metrics.surface_area_mm2
        }));
        match (mode, definition.manufacturing) {
            (GenerationMode::PreviewOnly, _) => {}
            (GenerationMode::Validated, Manufacturing::Fdm) => {
                let path = fdm_dir.join(format!("{}.3mf", definition.name));
                write_mesh_3mf(&definition.name, &mesh, &path)?;
                fabrication_artifacts.push(path);
            }
            (GenerationMode::Validated, Manufacturing::LaserCut) => {
                let Body::Sheet { outer, cutouts, .. } = &definition.body else {
                    return Err(format!(
                        "laser definition {:?} does not retain a nominal 2D profile",
                        definition.name
                    )
                    .into());
                };
                let Some(RegionNode::Polygon(outer_points)) = design.graph.region(*outer) else {
                    return Err("laser profile references an unknown region".into());
                };
                let cutout_points = cutouts
                    .iter()
                    .map(|cutout| match design.graph.region(*cutout) {
                        Some(RegionNode::Polygon(points)) => Ok(points.as_slice()),
                        None => Err("laser cutout references an unknown region"),
                    })
                    .collect::<Result<Vec<_>, _>>()?;
                let path = laser_dir.join(format!("{}.dxf", definition.name));
                write_dxf_sheet_profile(outer_points, &cutout_points, &path)?;
                fabrication_artifacts.push(path);
            }
            (GenerationMode::Validated, Manufacturing::Purchased) => {}
        }
        definition_meshes.push(mesh);
    }
    let zero_pose = design
        .kinematics
        .pose(PitchRollCommand {
            pitch: Angle::degrees(0.0).expect("zero angle is valid"),
            roll: Angle::degrees(0.0).expect("zero angle is valid"),
        })
        .map_err(|error| format!("zero pose rejected: {error:?}"))?;
    let mut export_parts = Vec::with_capacity(design.assembly.instances().len());
    let mut instance_manifest = Vec::with_capacity(design.assembly.instances().len());
    for (instance_index, instance) in design.assembly.instances().iter().enumerate() {
        let definition = design
            .assembly
            .definition(instance.definition)
            .ok_or("assembly referenced an unknown definition")?;
        let frame_pose = zero_pose
            .frame(instance.frame)
            .ok_or("assembly referenced an unknown frame")?;
        let static_pose = frame_pose.compose(instance.local_pose);
        instance_manifest.push(json!({
            "id": instance_index,
            "name": instance.name,
            "definition": definition.name,
            "role": format!("{:?}", definition.role),
            "location": {
                "side": instance.location.side.map(|value| value.as_str()),
                "longitudinal_end": instance.location.longitudinal_end.map(|value| value.as_str()),
                "vertical_end": instance.location.vertical_end.map(|value| value.as_str()),
                "ordinal": instance.location.ordinal,
            },
            "frame": instance.frame.index(),
            "static_translation_mm": static_pose.translation,
        }));
        export_parts.push(ExportPart {
            name: instance.name.clone(),
            definition: instance.definition,
            mesh: definition_meshes[instance.definition.index()].clone(),
            manufacturing: definition.manufacturing,
            frame: instance.frame,
            local_pose: instance.local_pose,
            static_pose,
            color_rgba: definition.color_rgba,
            semantics: ExportSemantics {
                role: format!("{:?}", definition.role),
                side: instance
                    .location
                    .side
                    .map(|value| value.as_str().to_string()),
                longitudinal_end: instance
                    .location
                    .longitudinal_end
                    .map(|value| value.as_str().to_string()),
                vertical_end: instance
                    .location
                    .vertical_end
                    .map(|value| value.as_str().to_string()),
                ordinal: instance.location.ordinal,
            },
        });
    }

    let three_mf_path = model_dir.join("assembly.3mf");
    let stl_path = model_dir.join("assembly.stl");
    let obj_path = model_dir.join("assembly.obj");
    let mtl_path = model_dir.join("assembly.mtl");
    let gltf_path = animation_dir.join("gimbal-motion.gltf");
    let bin_path = animation_dir.join("gimbal-motion.bin");
    write_3mf(&export_parts, &three_mf_path)?;
    write_binary_stl(&export_parts, &stl_path)?;
    write_obj(&export_parts, &obj_path, &mtl_path)?;
    write_animated_gltf(
        &export_parts,
        &design.kinematics,
        AnimationParameters {
            pitch_limit_degrees: loaded.parameters.motion.pitch_limit.as_degrees(),
            roll_limit_degrees: loaded.parameters.motion.roll_limit.as_degrees(),
            duration_seconds: 6.0,
            sample_count: 73,
        },
        &gltf_path,
        &bin_path,
    )?;

    let mut artifact_paths = vec![
        three_mf_path,
        stl_path,
        obj_path,
        mtl_path,
        gltf_path,
        bin_path,
    ];
    artifact_paths.extend(fabrication_artifacts);
    for optional in optional_artifact_paths(&output) {
        if optional.is_file() {
            artifact_paths.push(optional);
        }
    }
    let artifacts = staged_artifact_manifest(&output, &artifact_paths)?;
    let sector = &loaded.parameters.pitch_sector.sector;
    let gearbox_stage_ratio = design.pitch_gearbox_pair.ratio();
    let pitch_distribution_ratio = loaded.parameters.pitch_gearbox.small_gear.teeth() as f64
        / loaded.parameters.pitch_gearbox.distribution_gear.teeth() as f64;
    let validation_json = validation_report
        .as_ref()
        .map(|report| validation_report_json(&design, report))
        .unwrap_or_else(|| {
            json!({
                "valid": false,
                "complete": false,
                "preview_only": true,
                "reason": "mechanical assembly validation was intentionally not run for intermediate visualization"
            })
        });
    let manifest = json!({
        "schema_version": 3,
        "project": "pitch-roll cockpit attitude simulator prototype",
        "units": "millimeter",
        "status": if mode == GenerationMode::Validated {
            "validated unpowered concept geometry only; not load-rated"
        } else {
            "intermediate preview only; mechanically invalid or unvalidated; not for fabrication"
        },
        "preview_only": mode == GenerationMode::PreviewOnly,
        "geometry": {
            "reference_outside_diameter_mm": sector.external_reference().outside_diameter(),
            "external_reference_teeth": sector.external_reference().teeth(),
            "internal_reference_teeth": sector.internal_reference().teeth(),
            "sector_half_angle_deg": sector.half_angle().as_degrees(),
            "physical_sector_count": 4,
            "carrier_count": 2,
            "contact_unit_count": 4,
            "drive_pinions_per_unit": 2,
            "drive_pinion_count": 8,
            "retention_encoder_pinion_count": 4,
            "pitch_gearbox_count": 4,
            "roll_gearbox_count": 2,
            "cockpit_length_mm": loaded.parameters.cockpit.length.mm(),
            "cockpit_suspension_drop_mm": loaded.parameters.cockpit.suspension_drop.mm(),
            "roll_shaft_diameter_mm": loaded.parameters.roll_axis.shaft_radius.mm() * 2.0,
            "roll_bearing_outer_diameter_mm": loaded.parameters.roll_axis.bearing_outer_radius.mm() * 2.0,
            "roll_bearing_width_mm": loaded.parameters.roll_axis.bearing_width.mm(),
            "roll_gearbox_support_half_span_mm": loaded.parameters.roll_axis.gearbox_support_half_span.mm(),
            "upper_rail_height_mm": loaded.parameters.frame.upper_rail_height.mm(),
            "lower_rail_depth_mm": loaded.parameters.frame.lower_rail_depth.mm(),
            "moving_carrier_half_span_mm": loaded.parameters.frame.moving_carrier_half_span.mm(),
            "moving_carrier_height_mm": loaded.parameters.frame.moving_carrier_height.mm(),
            "moving_carrier_inboard_offset_mm": loaded.parameters.frame.moving_carrier_inboard_offset.mm(),
            "moving_carrier_member_width_mm": loaded.parameters.frame.moving_carrier_member_width.mm(),
            "fixed_crossmember_width_mm": loaded.parameters.frame.fixed_crossmember_width.mm(),
            "fixed_crossmember_station_mm": loaded.parameters.frame.fixed_crossmember_station.mm(),
            "fixed_rail_length_mm": loaded.parameters.frame.fixed_rail_length.mm(),
            "fixed_rail_depth_mm": loaded.parameters.frame.fixed_rail_depth.mm(),
            "floor_top_below_axis_mm": loaded.parameters.frame.floor_top_below_axis.mm(),
            "pitch_contact_outboard_support_plate_offset_mm": loaded.parameters.contact_unit.outboard_support_plate_offset.mm(),
            "pitch_gearbox_near_plate_inboard_offset_mm": loaded.parameters.pitch_gearbox.near_plate_inboard_offset.mm(),
            "pitch_gearbox_gear_plane_inboard_offset_mm": loaded.parameters.pitch_gearbox.gear_plane_inboard_offset.mm(),
            "pitch_gearbox_far_plate_inboard_offset_mm": loaded.parameters.pitch_gearbox.far_plate_inboard_offset.mm()
        },
        "ratios": {
            "pitch_drive_pinion_to_sector_reference": design.pitch_drive_pair.ratio(),
            "pitch_encoder_pinion_to_sector_reference": design.pitch_encoder_pair.ratio(),
            "pitch_branch_to_distribution": pitch_distribution_ratio,
            "pitch_gearbox_per_stage": gearbox_stage_ratio,
            "pitch_input_shaft_to_carrier": pitch_distribution_ratio * gearbox_stage_ratio * gearbox_stage_ratio * design.pitch_drive_pair.ratio(),
            "roll_pinion_to_cockpit": design.roll_pair.ratio(),
            "roll_input_shaft_to_cockpit": gearbox_stage_ratio * gearbox_stage_ratio * design.roll_pair.ratio()
        },
        "motion": {
            "pitch_limit_deg": loaded.parameters.motion.pitch_limit.as_degrees(),
            "roll_limit_deg": loaded.parameters.motion.roll_limit.as_degrees(),
            "yaw_degrees_of_freedom": 0
        },
        "validation": validation_json,
        "process_profiles": {
            "fdm_material": loaded.fdm_material.as_str(),
            "fdm_hole_compensation_mm": loaded.fdm_hole_compensation_mm,
            "laser_material": loaded.laser_material,
            "laser_kerf_mm": loaded.laser_kerf_mm,
            "laser_bed_mm": loaded.laser_bed_mm
        },
        "definitions": definition_manifest,
        "instances": instance_manifest,
        "artifacts": artifacts,
        "unverified": [
            "declared mechanism intent: pitch sectors fixed to world and contact units move with pitch",
            "declared mechanism intent: roll gearboxes are below the roll axis and move with pitch",
            "declared mechanism intent: roll shaft is continuous and the lower frame bears directly on the floor",
            "motor and encoder bodies are intentionally omitted from this prototype",
            "load capacity, stiffness, fatigue life, and tooth contact stress",
            "motor, encoder, bearing, brake, and emergency-stop selection",
            "leaf-spring rate and preload",
            "real manufactured backlash, runout, and load sharing",
            "production 2 m / 50-80 kg safety design"
        ]
    });
    fs::write(
        output.join("manifest.json"),
        serde_json::to_vec_pretty(&manifest)?,
    )?;
    let published_output = publish_staging_output(workspace)?;
    println!(
        "generated {} component definitions and {} instances in {} ({mode:?})",
        design.assembly.definitions().len(),
        export_parts.len(),
        published_output.display()
    );
    Ok(())
}

fn manufacturing_name(manufacturing: Manufacturing) -> &'static str {
    match manufacturing {
        Manufacturing::Fdm => "fdm",
        Manufacturing::LaserCut => "laser-cut",
        Manufacturing::Purchased => "purchased",
    }
}

// SPDX-License-Identifier: MIT

mod config;

use std::error::Error;
use std::fs;
use std::path::{Component, Path, PathBuf};

use config::LoadedConfig;
use gimbal_core::{
    Angle, Body, Manufacturing, NonNegativeLength, NumericalTolerance, PitchRollCommand,
    PositiveArea, PositiveLength, PositiveVolume, PrototypeDesign, RegionNode, TriangleMesh,
    build_prototype,
};
use gimbal_export::{
    AnimationParameters, ExportPart, sha256_file, write_3mf, write_animated_gltf, write_binary_stl,
    write_dxf_sheet_profile, write_mesh_3mf, write_obj,
};
use gimbal_kernel_manifold::{
    AssemblyValidator, Evaluator, UnrelatedProximityPolicy, ValidationIssueKind,
    ValidationProgress, ValidationReport, ValidationScope, ValidatorSettings,
};
use serde_json::{Value, json};

fn main() {
    if let Err(error) = run() {
        eprintln!("error: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let command = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "generate".to_string());
    let workspace = std::env::current_dir()?;
    let loaded = config::load(
        &workspace.join("parameters.toml"),
        &workspace.join("fabrication.toml"),
    )?;
    match command.as_str() {
        "generate" => generate(&workspace, &loaded, GenerationMode::Validated),
        "generate-preview" => generate(&workspace, &loaded, GenerationMode::PreviewOnly),
        "validate" => validate(&workspace, &loaded, ValidationScope::StructuralFast),
        "validate-full" => validate(&workspace, &loaded, ValidationScope::Full),
        "refresh-manifest" => refresh_manifest(&workspace),
        "clean-output" => clean_output(&workspace),
        unknown => Err(format!(
            "unknown command {unknown:?}; expected generate, generate-preview, validate, validate-full, refresh-manifest, or clean-output"
        )
        .into()),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum GenerationMode {
    Validated,
    PreviewOnly,
}

fn validate(
    workspace: &Path,
    loaded: &LoadedConfig,
    scope: ValidationScope,
) -> Result<(), Box<dyn Error>> {
    let design = build_prototype(&loaded.parameters)
        .map_err(|error| format!("prototype design rejected: {error:?}"))?;
    let report = validate_assembly(&design, scope)?;
    write_validation_report(workspace, &design, &report)?;
    for validated in &report.definitions {
        let definition = design
            .assembly
            .definition(validated.definition)
            .ok_or("validation report referenced an unknown definition")?;
        println!(
            "validated definition {:<38} {:>8} triangles {:>12.2} mm^3",
            definition.name, validated.metrics.triangles, validated.metrics.volume_mm3
        );
    }
    println!(
        "validation complete ({:?}): {} definitions checked ({} skipped), {} pair candidates, {} errors, {} warnings",
        report.scope,
        report.definitions.len(),
        report.skipped_definitions.len(),
        report.broad_phase_candidates,
        report.error_count(),
        report.warning_count(),
    );
    require_valid_assembly(&report)?;
    Ok(())
}

fn generate(
    workspace: &Path,
    loaded: &LoadedConfig,
    mode: GenerationMode,
) -> Result<(), Box<dyn Error>> {
    let design = build_prototype(&loaded.parameters)
        .map_err(|error| format!("prototype design rejected: {error:?}"))?;
    let validation_report = if mode == GenerationMode::Validated {
        let report = validate_assembly(&design, ValidationScope::Full)?;
        write_validation_report(workspace, &design, &report)?;
        require_valid_assembly(&report)?;
        Some(report)
    } else {
        None
    };
    let output = workspace.join("output");
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
    let artifacts = artifact_manifest(workspace, &artifact_paths)?;
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
    println!(
        "generated {} component definitions and {} instances in {} ({mode:?})",
        design.assembly.definitions().len(),
        export_parts.len(),
        output.display()
    );
    Ok(())
}

fn validate_assembly(
    design: &PrototypeDesign,
    scope: ValidationScope,
) -> Result<ValidationReport, Box<dyn Error>> {
    let zero_pose = design
        .kinematics
        .pose(PitchRollCommand {
            pitch: Angle::degrees(0.0).expect("zero angle is valid"),
            roll: Angle::degrees(0.0).expect("zero angle is valid"),
        })
        .map_err(|error| format!("zero pose rejected: {error:?}"))?;
    let settings = ValidatorSettings {
        scope,
        numerical_tolerance: NumericalTolerance {
            linear_epsilon: PositiveLength::mm(1.0e-6).expect("validator epsilon is positive"),
            area_epsilon: PositiveArea::square_mm(1.0e-8).expect("validator epsilon is positive"),
            volume_epsilon: PositiveVolume::cubic_mm(1.0e-7)
                .expect("validator epsilon is positive"),
        },
        unrelated_proximity_threshold: NonNegativeLength::mm(0.05)
            .expect("validator threshold is non-negative"),
        unrelated_proximity_policy: UnrelatedProximityPolicy::Warning,
    };
    AssemblyValidator::new(&design.graph, &design.assembly, &zero_pose, settings)
        .validate_with_progress(|progress| match progress {
            ValidationProgress::BroadPhaseComplete { candidates } => {
                eprintln!("assembly validation: {candidates} broad-phase pair candidates");
            }
            ValidationProgress::PairCheck {
                current,
                total,
                pair,
            } if current == 1 || current % 10 == 0 || current == total => {
                eprintln!(
                    "assembly validation: checking pair {current}/{total} ({:?}, {:?})",
                    pair.first, pair.second
                );
            }
            ValidationProgress::PairCheck { .. } => {}
        })
        .map_err(Into::into)
}

fn require_valid_assembly(report: &ValidationReport) -> Result<(), Box<dyn Error>> {
    if report.is_valid() {
        Ok(())
    } else {
        Err(format!(
            "assembly validation failed with {} errors; see output/validation-report.json",
            report.error_count()
        )
        .into())
    }
}

fn write_validation_report(
    workspace: &Path,
    design: &PrototypeDesign,
    report: &ValidationReport,
) -> Result<(), Box<dyn Error>> {
    let output = workspace.join("output");
    fs::create_dir_all(&output)?;
    let bytes = serde_json::to_vec_pretty(&validation_report_json(design, report))?;
    fs::write(output.join("validation-report.json"), &bytes)?;
    let scoped_name = match report.scope {
        ValidationScope::StructuralFast => "validation-report-structural.json",
        ValidationScope::Full => "validation-report-full.json",
    };
    fs::write(output.join(scoped_name), bytes)?;
    Ok(())
}

fn validation_report_json(design: &PrototypeDesign, report: &ValidationReport) -> Value {
    let pair_json = |pair: gimbal_core::ComponentInstancePair| {
        let identity_json = |id: gimbal_core::ComponentInstanceId| {
            let identity = design
                .assembly
                .component_identity(id)
                .expect("validation pair references an inserted instance");
            json!({
                "instance_id": id.index(),
                "role": format!("{:?}", identity.role),
                "location": {
                    "side": identity.location.side.map(|value| value.as_str()),
                    "longitudinal_end": identity.location.longitudinal_end.map(|value| value.as_str()),
                    "vertical_end": identity.location.vertical_end.map(|value| value.as_str()),
                    "ordinal": identity.location.ordinal,
                }
            })
        };
        json!({
            "first": identity_json(pair.first),
            "second": identity_json(pair.second),
        })
    };
    let issues = report
        .issues
        .iter()
        .map(|issue| {
            let (code, measurement) = match issue.kind {
                ValidationIssueKind::DuplicateComponentIdentity { .. } => {
                    ("duplicate_component_identity", Value::Null)
                }
                ValidationIssueKind::UnexpectedInterference {
                    intersection_volume_mm3,
                } => (
                    "unexpected_interference",
                    json!({ "intersection_volume_mm3": intersection_volume_mm3 }),
                ),
                ValidationIssueKind::PotentialStructuralInterference {
                    proxy_aabb_overlap_mm3,
                } => (
                    "potential_structural_interference",
                    json!({ "proxy_aabb_overlap_mm3": proxy_aabb_overlap_mm3 }),
                ),
                ValidationIssueKind::SurfaceContactSeparation {
                    distance_mm,
                    allowed_mm,
                } => (
                    "surface_contact_separation",
                    json!({ "distance_mm": distance_mm, "allowed_mm": allowed_mm }),
                ),
                ValidationIssueKind::SurfaceContactNormalMismatch {
                    error_radians,
                    allowed_radians,
                } => (
                    "surface_contact_normal_mismatch",
                    json!({
                        "error_radians": error_radians,
                        "allowed_radians": allowed_radians
                    }),
                ),
                ValidationIssueKind::SurfaceContactAreaInsufficient {
                    contact_area_mm2,
                    minimum_area_mm2,
                } => (
                    "surface_contact_area_insufficient",
                    json!({
                        "contact_area_mm2": contact_area_mm2,
                        "minimum_area_mm2": minimum_area_mm2
                    }),
                ),
                ValidationIssueKind::FastenerHoleAxisSeparation {
                    distance_mm,
                    allowed_mm,
                } => (
                    "fastener_hole_axis_separation",
                    json!({ "distance_mm": distance_mm, "allowed_mm": allowed_mm }),
                ),
                ValidationIssueKind::FastenerHoleAxisMismatch {
                    error_radians,
                    allowed_radians,
                } => (
                    "fastener_hole_axis_mismatch",
                    json!({
                        "error_radians": error_radians,
                        "allowed_radians": allowed_radians
                    }),
                ),
                ValidationIssueKind::FastenerHoleRadiusMismatch {
                    first_radius_mm,
                    second_radius_mm,
                    expected_radius_mm,
                    allowed_mm,
                } => (
                    "fastener_hole_radius_mismatch",
                    json!({
                        "first_radius_mm": first_radius_mm,
                        "second_radius_mm": second_radius_mm,
                        "expected_radius_mm": expected_radius_mm,
                        "allowed_mm": allowed_mm
                    }),
                ),
                ValidationIssueKind::FastenerSeatNormalMismatch {
                    error_radians,
                    allowed_radians,
                } => (
                    "fastener_seat_normal_mismatch",
                    json!({
                        "error_radians": error_radians,
                        "allowed_radians": allowed_radians
                    }),
                ),
                ValidationIssueKind::FastenerGripLengthMismatch {
                    actual_mm,
                    expected_mm,
                    allowed_mm,
                } => (
                    "fastener_grip_length_mismatch",
                    json!({
                        "actual_mm": actual_mm,
                        "expected_mm": expected_mm,
                        "allowed_mm": allowed_mm
                    }),
                ),
                ValidationIssueKind::FastenerHardwareAxisSeparation {
                    hardware,
                    distance_mm,
                    allowed_mm,
                } => (
                    "fastener_hardware_axis_separation",
                    json!({
                        "hardware_instance_id": hardware.index(),
                        "distance_mm": distance_mm,
                        "allowed_mm": allowed_mm
                    }),
                ),
                ValidationIssueKind::FastenerHardwareAxisMismatch {
                    hardware,
                    error_radians,
                    allowed_radians,
                } => (
                    "fastener_hardware_axis_mismatch",
                    json!({
                        "hardware_instance_id": hardware.index(),
                        "error_radians": error_radians,
                        "allowed_radians": allowed_radians
                    }),
                ),
                ValidationIssueKind::FastenerHardwareContactMismatch {
                    first,
                    second,
                    separation_mm,
                    normal_error_radians,
                    allowed_mm,
                    allowed_radians,
                } => (
                    "fastener_hardware_contact_mismatch",
                    json!({
                        "first_instance_id": first.index(),
                        "second_instance_id": second.index(),
                        "separation_mm": separation_mm,
                        "normal_error_radians": normal_error_radians,
                        "allowed_mm": allowed_mm,
                        "allowed_radians": allowed_radians
                    }),
                ),
                ValidationIssueKind::FastenerThreadEngagementInsufficient {
                    actual_mm,
                    minimum_mm,
                } => (
                    "fastener_thread_engagement_insufficient",
                    json!({ "actual_mm": actual_mm, "minimum_mm": minimum_mm }),
                ),
                ValidationIssueKind::FastenerBoltProtrusionInsufficient {
                    actual_mm,
                    minimum_mm,
                } => (
                    "fastener_bolt_protrusion_insufficient",
                    json!({ "actual_mm": actual_mm, "minimum_mm": minimum_mm }),
                ),
                ValidationIssueKind::UnspecifiedProximity {
                    gap_mm,
                    threshold_mm,
                } => (
                    "unspecified_proximity",
                    json!({
                        "gap_mm": gap_mm,
                        "threshold_mm": threshold_mm
                    }),
                ),
            };
            json!({
                "severity": format!("{:?}", issue.severity).to_lowercase(),
                "code": code,
                "pair": issue.pair.map(pair_json),
                "relation_id": issue.relation.map(|relation| relation.index()),
                "measurement": measurement,
            })
        })
        .collect::<Vec<_>>();
    let skipped_definitions = report
        .skipped_definitions
        .iter()
        .map(|id| {
            let definition = design
                .assembly
                .definition(*id)
                .expect("validation report references an inserted definition");
            json!({
                "definition_id": id.index(),
                "name": definition.name,
                "role": format!("{:?}", definition.role),
            })
        })
        .collect::<Vec<_>>();
    json!({
        "complete": true,
        "preview_only": false,
        "valid": report.is_valid(),
        "scope": match report.scope {
            ValidationScope::StructuralFast => "structural-fast",
            ValidationScope::Full => "full",
        },
        "definition_count": report.definitions.len(),
        "skipped_definition_count": report.skipped_definitions.len(),
        "skipped_definitions": skipped_definitions,
        "skipped_instance_count": report.skipped_instances.len(),
        "total_instance_pairs": report.total_instance_pairs,
        "eligible_instance_pairs": report.eligible_instance_pairs,
        "broad_phase_candidates": report.broad_phase_candidates,
        "unrelated_proximity_checks": report.unrelated_proximity_checks,
        "skipped_relation_checks": report.skipped_relation_checks,
        "pair_checks": report.pair_checks.len(),
        "error_count": report.error_count(),
        "warning_count": report.warning_count(),
        "issues": issues,
    })
}

fn clean_output(workspace: &Path) -> Result<(), Box<dyn Error>> {
    let output = workspace.join("output");
    let canonical_workspace = workspace.canonicalize()?;
    if output.exists() {
        let canonical_output = output.canonicalize()?;
        if canonical_output.parent() != Some(canonical_workspace.as_path())
            || canonical_output.file_name().and_then(|name| name.to_str()) != Some("output")
        {
            return Err("refusing to remove an unexpected output path".into());
        }
        fs::remove_dir_all(&canonical_output)?;
        println!("removed {}", canonical_output.display());
    }
    Ok(())
}

fn manufacturing_name(manufacturing: Manufacturing) -> &'static str {
    match manufacturing {
        Manufacturing::Fdm => "fdm",
        Manufacturing::LaserCut => "laser-cut",
        Manufacturing::Purchased => "purchased",
    }
}

fn optional_artifact_paths(output: &Path) -> Vec<PathBuf> {
    [
        "model/gimbal-prototype.blend",
        "preview/isometric.png",
        "preview/top-z.png",
        "preview/left-side-minus-y.png",
        "preview/front-plus-x.png",
        "preview/drive-unit-detail.png",
        "preview/pitch-gearbox-detail.png",
        "preview/roll-gearbox-detail.png",
        "preview/pitch-sector-reinforcement-detail.png",
        "preview/gimbal-motion.mp4",
        "preview/pitch-gearbox-motion.mp4",
        "preview/roll-gearbox-motion.mp4",
        "validation-report.json",
        "validation-report-structural.json",
        "validation-report-full.json",
    ]
    .into_iter()
    .map(|relative| output.join(relative))
    .collect()
}

fn artifact_manifest(
    workspace: &Path,
    artifact_paths: &[PathBuf],
) -> Result<Vec<Value>, Box<dyn Error>> {
    artifact_paths
        .iter()
        .map(|path| {
            let relative = path.strip_prefix(workspace).unwrap_or(path);
            Ok(json!({
                "path": relative.to_string_lossy().replace('\\', "/"),
                "bytes": fs::metadata(path)?.len(),
                "sha256": sha256_file(path)?
            }))
        })
        .collect()
}

fn refresh_manifest(workspace: &Path) -> Result<(), Box<dyn Error>> {
    let output = workspace.join("output");
    let manifest_path = output.join("manifest.json");
    let mut manifest: Value = serde_json::from_slice(&fs::read(&manifest_path)?)?;
    let mut paths = Vec::<PathBuf>::new();
    for artifact in manifest
        .get("artifacts")
        .and_then(Value::as_array)
        .ok_or("manifest has no artifact list")?
    {
        let relative = artifact
            .get("path")
            .and_then(Value::as_str)
            .ok_or("manifest artifact has no path")?;
        let relative_path = Path::new(relative);
        if relative_path.is_absolute()
            || relative_path.components().any(|component| {
                matches!(
                    component,
                    Component::ParentDir | Component::RootDir | Component::Prefix(_)
                )
            })
            || !relative_path.starts_with("output")
        {
            return Err(format!("unsafe artifact path in manifest: {relative:?}").into());
        }
        let path = workspace.join(relative_path);
        if path.is_file() && !paths.contains(&path) {
            paths.push(path);
        }
    }
    for path in optional_artifact_paths(&output) {
        if path.is_file() && !paths.contains(&path) {
            paths.push(path);
        }
    }
    manifest["artifacts"] = Value::Array(artifact_manifest(workspace, &paths)?);
    fs::write(&manifest_path, serde_json::to_vec_pretty(&manifest)?)?;
    println!("refreshed {} artifact hashes", paths.len());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use gimbal_core::{
        AssemblyRelation, ComponentIdentity, ComponentInstance, ComponentInstanceId,
        ComponentLocation, ComponentRole, LongitudinalEnd, PrototypeDesign, RigidTransform, Side,
        VerticalEnd,
    };
    use gimbal_kernel_manifold::GeometryEvaluationMode;
    use std::collections::HashMap;

    fn load_configuration() -> config::LoadedConfig {
        let workspace = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        config::load(
            &workspace.join("parameters.toml"),
            &workspace.join("fabrication.toml"),
        )
        .expect("repository parameters must be valid")
    }

    fn load_design() -> PrototypeDesign {
        let loaded = load_configuration();
        build_prototype(&loaded.parameters).expect("repository design must be valid")
    }

    fn is_fastener_validation_issue(kind: ValidationIssueKind) -> bool {
        matches!(
            kind,
            ValidationIssueKind::FastenerHoleAxisSeparation { .. }
                | ValidationIssueKind::FastenerHoleAxisMismatch { .. }
                | ValidationIssueKind::FastenerHoleRadiusMismatch { .. }
                | ValidationIssueKind::FastenerSeatNormalMismatch { .. }
                | ValidationIssueKind::FastenerGripLengthMismatch { .. }
                | ValidationIssueKind::FastenerHardwareAxisSeparation { .. }
                | ValidationIssueKind::FastenerHardwareAxisMismatch { .. }
                | ValidationIssueKind::FastenerHardwareContactMismatch { .. }
                | ValidationIssueKind::FastenerThreadEngagementInsufficient { .. }
                | ValidationIssueKind::FastenerBoltProtrusionInsufficient { .. }
        )
    }

    fn command(pitch: f64, roll: f64) -> PitchRollCommand {
        PitchRollCommand {
            pitch: Angle::degrees(pitch).expect("finite pitch"),
            roll: Angle::degrees(roll).expect("finite roll"),
        }
    }

    const fn singleton(role: ComponentRole) -> ComponentIdentity {
        ComponentIdentity {
            role,
            location: ComponentLocation::new(),
        }
    }

    const fn located(role: ComponentRole, location: ComponentLocation) -> ComponentIdentity {
        ComponentIdentity { role, location }
    }

    fn selected_instance(
        design: &PrototypeDesign,
        identity: ComponentIdentity,
    ) -> &ComponentInstance {
        let mut matches = design
            .assembly
            .instances_with_role(identity.role)
            .filter(|(_, instance)| instance.location == identity.location);
        let (_, instance) = matches
            .next()
            .unwrap_or_else(|| panic!("missing component identity {identity:?}"));
        assert!(
            matches.next().is_none(),
            "component identity is not unique: {identity:?}"
        );
        instance
    }

    fn instance_pose(
        design: &PrototypeDesign,
        identity: ComponentIdentity,
        pitch: f64,
        roll: f64,
    ) -> RigidTransform {
        let instance = selected_instance(design, identity);
        design
            .kinematics
            .pose(command(pitch, roll))
            .expect("command within limits")
            .frame(instance.frame)
            .expect("instance frame exists")
            .compose(instance.local_pose)
    }

    fn instance_solid(
        design: &PrototypeDesign,
        identity: ComponentIdentity,
    ) -> gimbal_core::SolidId {
        let instance = selected_instance(design, identity);
        design
            .assembly
            .definition(instance.definition)
            .expect("instance definition exists")
            .body
            .assembly_solid()
    }

    fn instance_solid_by_id(
        design: &PrototypeDesign,
        instance_id: ComponentInstanceId,
    ) -> gimbal_core::SolidId {
        let instance = design
            .assembly
            .instance(instance_id)
            .expect("instance id exists");
        design
            .assembly
            .definition(instance.definition)
            .expect("instance definition exists")
            .body
            .assembly_solid()
    }

    fn instance_pose_by_id(
        design: &PrototypeDesign,
        instance_id: ComponentInstanceId,
        pitch: f64,
        roll: f64,
    ) -> RigidTransform {
        let instance = design
            .assembly
            .instance(instance_id)
            .expect("instance id exists");
        design
            .kinematics
            .pose(command(pitch, roll))
            .expect("command within limits")
            .frame(instance.frame)
            .expect("instance frame exists")
            .compose(instance.local_pose)
    }

    fn count_role(design: &PrototypeDesign, role: ComponentRole) -> usize {
        design.assembly.instances_with_role(role).count()
    }

    #[test]
    fn repository_design_has_the_required_reused_components() {
        let design = load_design();
        let identity_collisions = design.assembly.component_identity_collisions();
        assert!(
            identity_collisions.is_empty(),
            "component semantic identities must be unique: {identity_collisions:#?}"
        );
        assert_eq!(count_role(&design, ComponentRole::PitchSector), 4);
        assert_eq!(count_role(&design, ComponentRole::PitchDrivePinion), 8);
        assert_eq!(count_role(&design, ComponentRole::PitchRetentionPinion), 4);
        assert_eq!(count_role(&design, ComponentRole::RollDrivenGear), 2);
        assert_eq!(count_role(&design, ComponentRole::RollInputPinion), 2);
        for side in [Side::Left, Side::Right] {
            for end in [LongitudinalEnd::Front, LongitudinalEnd::Rear] {
                assert!(
                    design
                        .assembly
                        .instances_with_role(ComponentRole::PitchSector)
                        .any(|(_, instance)| {
                            instance.location.side == Some(side)
                                && instance.location.longitudinal_end == Some(end)
                        })
                );
                for ordinal in [1, 2] {
                    assert!(
                        design
                            .assembly
                            .instances_with_role(ComponentRole::PitchDrivePinion)
                            .any(|(_, instance)| {
                                instance.location.side == Some(side)
                                    && instance.location.longitudinal_end == Some(end)
                                    && instance.location.ordinal == Some(ordinal)
                            })
                    );
                }
            }
        }
        assert_eq!(count_role(&design, ComponentRole::FixedCrossmember), 4);
        assert_eq!(count_role(&design, ComponentRole::FixedCarrierPost), 4);
        for end in [LongitudinalEnd::Front, LongitudinalEnd::Rear] {
            selected_instance(
                &design,
                located(
                    ComponentRole::RollGearboxSmallGear,
                    ComponentLocation::new()
                        .with_longitudinal_end(end)
                        .with_ordinal(2),
                ),
            );
        }
    }

    #[test]
    fn pitch_contact_units_use_two_spread_outer_drives_and_one_inner_retainer() {
        let design = load_design();
        let location = ComponentLocation::new()
            .with_side(Side::Left)
            .with_longitudinal_end(LongitudinalEnd::Front);
        let first = instance_pose(
            &design,
            located(ComponentRole::PitchDrivePinion, location.with_ordinal(1)),
            0.0,
            0.0,
        );
        let second = instance_pose(
            &design,
            located(ComponentRole::PitchDrivePinion, location.with_ordinal(2)),
            0.0,
            0.0,
        );
        let retainer = instance_pose(
            &design,
            located(ComponentRole::PitchRetentionPinion, location),
            0.0,
            0.0,
        );
        let radial = |pose: RigidTransform| {
            (pose.translation[0] * pose.translation[0] + pose.translation[2] * pose.translation[2])
                .sqrt()
        };
        let first_radius = radial(first);
        let second_radius = radial(second);
        let retainer_radius = radial(retainer);
        let sector_outer_pitch = load_configuration()
            .parameters
            .pitch_sector
            .sector
            .external_reference()
            .pitch_radius();
        let sector_inner_pitch = load_configuration()
            .parameters
            .pitch_sector
            .sector
            .internal_reference()
            .pitch_radius();
        assert!(first_radius > sector_outer_pitch);
        assert!(second_radius > sector_outer_pitch);
        assert!(retainer_radius < sector_inner_pitch);

        let separation = ((first.translation[0] - second.translation[0]).powi(2)
            + (first.translation[2] - second.translation[2]).powi(2))
        .sqrt();
        assert!(
            separation >= 40.0,
            "outer drive pinions need a useful load-sharing baseline; got {separation} mm"
        );
    }

    #[test]
    fn fixed_sector_load_paths_reach_the_floor_through_typed_contacts() {
        let design = load_design();
        let floor = design
            .assembly
            .instance_by_identity(singleton(ComponentRole::InstallationFloor))
            .expect("installation floor exists");
        let mut adjacency = vec![Vec::new(); design.assembly.instances().len()];
        for relation in design.assembly.relations() {
            let (first, second) = match relation {
                AssemblyRelation::SurfaceContact(contact) => {
                    (contact.first.instance, contact.second.instance)
                }
                AssemblyRelation::Fastened(joint) => {
                    (joint.first_hole.instance, joint.second_hole.instance)
                }
                AssemblyRelation::CylindricalFit(fit) => (fit.shaft.instance, fit.bore.instance),
                AssemblyRelation::GearMesh(mesh) => {
                    (mesh.first_axis.instance, mesh.second_axis.instance)
                }
            };
            adjacency[first.index()].push(second);
            adjacency[second.index()].push(first);
        }
        for (sector, _) in design
            .assembly
            .instances_with_role(ComponentRole::PitchSector)
        {
            let mut visited = vec![false; adjacency.len()];
            let mut pending = vec![sector];
            visited[sector.index()] = true;
            while let Some(current) = pending.pop() {
                for &next in &adjacency[current.index()] {
                    if !visited[next.index()] {
                        visited[next.index()] = true;
                        pending.push(next);
                    }
                }
            }
            assert!(
                visited[floor.index()],
                "pitch sector {sector:?} has no typed structural path to the floor"
            );
        }
    }

    #[test]
    fn structural_surface_contacts_do_not_use_solid_overlap() {
        let design = load_design();
        let mut evaluator = Evaluator::new(&design.graph);
        let mut checked = 0;
        let mut overlaps = Vec::new();
        for relation in design.assembly.relations() {
            let AssemblyRelation::SurfaceContact(contact) = relation else {
                continue;
            };
            let volume = evaluator
                .intersection_volume_transformed(
                    instance_solid_by_id(&design, contact.first.instance),
                    instance_pose_by_id(&design, contact.first.instance, 0.0, 0.0),
                    instance_solid_by_id(&design, contact.second.instance),
                    instance_pose_by_id(&design, contact.second.instance, 0.0, 0.0),
                )
                .expect("structural contact intersection query succeeds");
            if volume > 1.0e-7 {
                let first = design
                    .assembly
                    .instance(contact.first.instance)
                    .expect("first contact instance exists");
                let second = design
                    .assembly
                    .instance(contact.second.instance)
                    .expect("second contact instance exists");
                overlaps.push((first.name.as_str(), second.name.as_str(), volume));
            }
            checked += 1;
        }
        assert_eq!(checked, 42);
        assert!(
            overlaps.is_empty(),
            "structural surface contacts must not use solid overlap: {overlaps:#?}"
        );
    }

    #[test]
    fn fixed_structure_has_no_unintended_solid_overlap() {
        let design = load_design();
        let fixed_roles = [
            ComponentRole::PitchSector,
            ComponentRole::FixedCarrierRail,
            ComponentRole::FixedCarrierPost,
            ComponentRole::FixedCrossmember,
            ComponentRole::InstallationFloor,
        ];
        let fixed_instances = design
            .assembly
            .instances_with_ids()
            .filter_map(|(instance_id, instance)| {
                let definition = design
                    .assembly
                    .definition(instance.definition)
                    .expect("instance definition exists");
                fixed_roles
                    .contains(&definition.role)
                    .then_some(instance_id)
            })
            .collect::<Vec<_>>();
        let mut evaluator = Evaluator::new(&design.graph);
        let mut overlaps = Vec::new();
        let mut checked = 0;
        for (index, first_id) in fixed_instances.iter().copied().enumerate() {
            for second_id in fixed_instances.iter().copied().skip(index + 1) {
                let volume = evaluator
                    .intersection_volume_transformed(
                        instance_solid_by_id(&design, first_id),
                        instance_pose_by_id(&design, first_id, 0.0, 0.0),
                        instance_solid_by_id(&design, second_id),
                        instance_pose_by_id(&design, second_id, 0.0, 0.0),
                    )
                    .expect("fixed structure intersection query succeeds");
                if volume > 1.0e-7 {
                    let first = design
                        .assembly
                        .instance(first_id)
                        .expect("first fixed instance exists");
                    let second = design
                        .assembly
                        .instance(second_id)
                        .expect("second fixed instance exists");
                    overlaps.push((first.name.as_str(), second.name.as_str(), volume));
                }
                checked += 1;
            }
        }
        assert_eq!(fixed_instances.len(), 17);
        assert_eq!(checked, 136);
        assert!(
            overlaps.is_empty(),
            "fixed structure must not contain unintended solid overlap: {overlaps:#?}"
        );
    }

    #[test]
    fn structural_face_contacts_are_typed_relations() {
        let design = load_design();
        let contacts = design
            .assembly
            .relations()
            .iter()
            .filter(|relation| matches!(relation, AssemblyRelation::SurfaceContact(_)))
            .count();
        assert_eq!(contacts, 42);

        for side in [Side::Left, Side::Right] {
            for end in [LongitudinalEnd::Front, LongitudinalEnd::Rear] {
                let post = design
                    .assembly
                    .instance_by_identity(located(
                        ComponentRole::FixedCarrierPost,
                        ComponentLocation::new()
                            .with_side(side)
                            .with_longitudinal_end(end),
                    ))
                    .expect("fixed carrier post exists");
                let relation_count = design
                    .assembly
                    .relations_with_ids()
                    .filter(|(_, relation)| match relation {
                        AssemblyRelation::SurfaceContact(contact) => {
                            contact.first.instance == post || contact.second.instance == post
                        }
                        AssemblyRelation::Fastened(_) => false,
                        AssemblyRelation::CylindricalFit(fit) => {
                            fit.shaft.instance == post || fit.bore.instance == post
                        }
                        AssemblyRelation::GearMesh(mesh) => {
                            mesh.first_axis.instance == post || mesh.second_axis.instance == post
                        }
                    })
                    .count();
                assert_eq!(relation_count, 3);
            }
        }
    }

    #[test]
    fn sector_post_m3_joints_have_real_clearance_and_valid_datums() {
        let design = load_design();
        let joints = design
            .assembly
            .relations()
            .iter()
            .filter_map(|relation| {
                let AssemblyRelation::Fastened(joint) = relation else {
                    return None;
                };
                let roles = [joint.first_hole.instance, joint.second_hole.instance].map(|id| {
                    design
                        .assembly
                        .definition(
                            design
                                .assembly
                                .instance(id)
                                .expect("fastened member exists")
                                .definition,
                        )
                        .expect("fastened member definition exists")
                        .role
                });
                (roles == [ComponentRole::PitchSector, ComponentRole::FixedCarrierPost])
                    .then_some(*joint)
            })
            .collect::<Vec<_>>();
        assert_eq!(
            joints.len(),
            8,
            "each of four sector/post joints needs two bolts"
        );

        let report = validate_assembly(&design, ValidationScope::StructuralFast)
            .expect("fast assembly validation succeeds");
        assert!(
            report
                .issues
                .iter()
                .all(|issue| !is_fastener_validation_issue(issue.kind)),
            "all M3 member and hardware datums must satisfy the typed relation: {:#?}",
            report
                .issues
                .iter()
                .filter(|issue| is_fastener_validation_issue(issue.kind))
                .collect::<Vec<_>>()
        );

        let mut evaluator = Evaluator::new(&design.graph);
        for joint in joints {
            let participants = [
                joint.first_hole.instance,
                joint.second_hole.instance,
                joint.hardware.bolt.instance,
                joint.hardware.nut.instance,
                joint
                    .hardware
                    .first_washer
                    .expect("head washer exists")
                    .instance,
                joint
                    .hardware
                    .second_washer
                    .expect("nut washer exists")
                    .instance,
            ];
            for first_index in 0..participants.len() {
                for second_index in first_index + 1..participants.len() {
                    let first_id = participants[first_index];
                    let second_id = participants[second_index];
                    let volume = evaluator
                        .intersection_volume_transformed(
                            instance_solid_by_id(&design, first_id),
                            instance_pose_by_id(&design, first_id, 0.0, 0.0),
                            instance_solid_by_id(&design, second_id),
                            instance_pose_by_id(&design, second_id, 0.0, 0.0),
                        )
                        .expect("M3 joint intersection query succeeds");
                    let first_name = &design
                        .assembly
                        .instance(first_id)
                        .expect("participant exists")
                        .name;
                    let second_name = &design
                        .assembly
                        .instance(second_id)
                        .expect("participant exists")
                        .name;
                    assert!(
                        volume <= 1.0e-7,
                        "M3 joint participants {first_name} and {second_name} overlap by {volume} mm^3"
                    );
                }
            }
        }
    }

    #[test]
    fn pitch_gearbox_plates_use_real_m3_fasteners_instead_of_placeholder_rods() {
        let design = load_design();
        let joints = design
            .assembly
            .relations()
            .iter()
            .filter_map(|relation| {
                let AssemblyRelation::Fastened(joint) = relation else {
                    return None;
                };
                let first_role = design
                    .assembly
                    .definition(
                        design
                            .assembly
                            .instance(joint.first_hole.instance)
                            .expect("first fastened member exists")
                            .definition,
                    )
                    .expect("first member definition exists")
                    .role;
                let second_role = design
                    .assembly
                    .definition(
                        design
                            .assembly
                            .instance(joint.second_hole.instance)
                            .expect("second fastened member exists")
                            .definition,
                    )
                    .expect("second member definition exists")
                    .role;
                (first_role == ComponentRole::PitchContactCarriagePlate
                    && second_role == ComponentRole::PitchGearboxFarPlate)
                    .then_some(*joint)
            })
            .collect::<Vec<_>>();
        assert_eq!(
            joints.len(),
            12,
            "each of four gearboxes needs three M3 joints"
        );
        assert!(
            design
                .assembly
                .instances()
                .iter()
                .all(|instance| !instance.name.contains("_m3_tie_"))
        );
        assert_eq!(count_role(&design, ComponentRole::M3Bolt), 20);
        assert_eq!(count_role(&design, ComponentRole::M3Nut), 20);
        assert_eq!(count_role(&design, ComponentRole::M3Washer), 40);

        let report = validate_assembly(&design, ValidationScope::StructuralFast)
            .expect("fast assembly validation succeeds");
        assert!(
            report
                .issues
                .iter()
                .all(|issue| !is_fastener_validation_issue(issue.kind)),
            "pitch gearbox hardware placement must satisfy the typed M3 relation: {:#?}",
            report
                .issues
                .iter()
                .filter(|issue| is_fastener_validation_issue(issue.kind))
                .collect::<Vec<_>>()
        );

        let mut evaluator = Evaluator::new(&design.graph);
        for joint in joints {
            let participants = [
                joint.first_hole.instance,
                joint.second_hole.instance,
                joint.hardware.bolt.instance,
                joint.hardware.nut.instance,
                joint
                    .hardware
                    .first_washer
                    .expect("head washer exists")
                    .instance,
                joint
                    .hardware
                    .second_washer
                    .expect("nut washer exists")
                    .instance,
            ];
            for first_index in 0..participants.len() {
                for second_index in first_index + 1..participants.len() {
                    let first_id = participants[first_index];
                    let second_id = participants[second_index];
                    let volume = evaluator
                        .intersection_volume_transformed(
                            instance_solid_by_id(&design, first_id),
                            instance_pose_by_id(&design, first_id, 0.0, 0.0),
                            instance_solid_by_id(&design, second_id),
                            instance_pose_by_id(&design, second_id, 0.0, 0.0),
                        )
                        .expect("pitch gearbox fastener intersection query succeeds");
                    assert!(
                        volume <= 1.0e-7,
                        "pitch gearbox M3 participants overlap by {volume} mm^3"
                    );
                }
            }
        }
    }

    #[test]
    fn fixed_rack_stays_still_while_pinion_unit_orbits() {
        let design = load_design();
        let rack = located(
            ComponentRole::PitchSector,
            ComponentLocation::new()
                .with_side(Side::Left)
                .with_longitudinal_end(LongitudinalEnd::Front),
        );
        let pinion = located(
            ComponentRole::PitchDrivePinion,
            ComponentLocation::new()
                .with_side(Side::Left)
                .with_longitudinal_end(LongitudinalEnd::Front)
                .with_ordinal(1),
        );
        let floor = singleton(ComponentRole::InstallationFloor);
        let rack_zero = instance_pose(&design, rack, 0.0, 0.0);
        let rack_pitch = instance_pose(&design, rack, 20.0, 0.0);
        assert_eq!(rack_zero, rack_pitch);

        let pinion_zero = instance_pose(&design, pinion, 0.0, 0.0);
        let pinion_pitch = instance_pose(&design, pinion, 20.0, 0.0);
        assert_ne!(pinion_zero.translation, pinion_pitch.translation);
        let radius_zero = pinion_zero.translation[0].hypot(pinion_zero.translation[2]);
        let radius_pitch = pinion_pitch.translation[0].hypot(pinion_pitch.translation[2]);
        assert!((radius_zero - radius_pitch).abs() < 1.0e-8);

        let floor_zero = instance_pose(&design, floor, 0.0, 0.0);
        let floor_pitch = instance_pose(&design, floor, 20.0, 0.0);
        assert_eq!(floor_zero, floor_pitch);
    }

    #[test]
    fn pitch_drive_and_roll_mechanism_travel_as_one_moving_body() {
        let design = load_design();
        let moving_components = [
            located(
                ComponentRole::PitchContactCarriagePlate,
                ComponentLocation::new()
                    .with_side(Side::Left)
                    .with_longitudinal_end(LongitudinalEnd::Front),
            ),
            located(
                ComponentRole::RollGearboxPlate,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Front)
                    .with_ordinal(1),
            ),
            singleton(ComponentRole::RollShaft),
            singleton(ComponentRole::Cockpit),
        ];
        for component in moving_components {
            let zero = instance_pose(&design, component, 0.0, 0.0);
            let pitched = instance_pose(&design, component, 20.0, 0.0);
            assert_ne!(zero, pitched, "{component:?} must follow pitch");
        }

        let rack = located(
            ComponentRole::PitchSector,
            ComponentLocation::new()
                .with_side(Side::Left)
                .with_longitudinal_end(LongitudinalEnd::Front),
        );
        let rack_zero = instance_pose(&design, rack, 0.0, 0.0);
        let rack_pitched = instance_pose(&design, rack, 20.0, 0.0);
        assert_eq!(rack_zero, rack_pitched, "the ground rack must remain fixed");

        let roll_shaft = singleton(ComponentRole::RollShaft);
        let roll_gearbox_plate = located(
            ComponentRole::RollGearboxPlate,
            ComponentLocation::new()
                .with_longitudinal_end(LongitudinalEnd::Front)
                .with_ordinal(1),
        );
        let roll_shaft_zero = instance_pose(&design, roll_shaft, 0.0, 0.0);
        let roll_shaft_pitched = instance_pose(&design, roll_shaft, 20.0, 0.0);
        let roll_gearbox_zero = instance_pose(&design, roll_gearbox_plate, 0.0, 0.0);
        let roll_gearbox_pitched = instance_pose(&design, roll_gearbox_plate, 20.0, 0.0);
        assert!(
            (distance(roll_shaft_zero, roll_gearbox_zero)
                - distance(roll_shaft_pitched, roll_gearbox_pitched))
            .abs()
                < 1.0e-8
        );
    }

    #[test]
    fn pitch_gearboxes_are_between_the_two_sector_planes() {
        let design = load_design();
        let mut far_plate_y = [0.0; 2];
        for (side_index, side) in [Side::Left, Side::Right].into_iter().enumerate() {
            for end in [LongitudinalEnd::Front, LongitudinalEnd::Rear] {
                let location = ComponentLocation::new()
                    .with_side(side)
                    .with_longitudinal_end(end);
                let sector_y = instance_pose(
                    &design,
                    located(ComponentRole::PitchSector, location),
                    0.0,
                    0.0,
                )
                .translation[1];
                let outboard_plate_y = instance_pose(
                    &design,
                    located(ComponentRole::PitchContactOutboardPlate, location),
                    0.0,
                    0.0,
                )
                .translation[1];
                let near_plate_y = instance_pose(
                    &design,
                    located(ComponentRole::PitchContactCarriagePlate, location),
                    0.0,
                    0.0,
                )
                .translation[1];
                let far_y = instance_pose(
                    &design,
                    located(ComponentRole::PitchGearboxFarPlate, location),
                    0.0,
                    0.0,
                )
                .translation[1];
                let input_gear_y = instance_pose(
                    &design,
                    located(
                        ComponentRole::PitchGearboxSmallGear,
                        location.with_ordinal(5),
                    ),
                    0.0,
                    0.0,
                )
                .translation[1];

                assert!(outboard_plate_y.abs() > sector_y.abs());
                assert!(near_plate_y.abs() < sector_y.abs());
                assert!(input_gear_y.abs() < near_plate_y.abs());
                assert!(far_y.abs() < input_gear_y.abs());
                far_plate_y[side_index] = far_y;
            }
        }
        assert!(
            far_plate_y[1] - far_plate_y[0] > load_configuration().parameters.cockpit.width.mm(),
            "the opposing inboard gearbox plates must leave a central cockpit corridor"
        );
    }

    #[test]
    fn moving_carrier_and_roll_bearing_supports_avoid_the_cockpit_underbody() {
        let design = load_design();
        for (_, instance) in design
            .assembly
            .instances_with_role(ComponentRole::PitchCradleLongitudinalRail)
        {
            let pose = design
                .kinematics
                .pose(command(0.0, 0.0))
                .expect("zero pose is valid")
                .frame(instance.frame)
                .expect("instance frame exists")
                .compose(instance.local_pose);
            assert!(
                pose.translation[2] > 0.0,
                "upper moving-carrier support {} must not occupy the cockpit underside",
                instance.name
            );
        }
        let cockpit_half_length = load_configuration().parameters.cockpit.length.mm() * 0.5;
        for (_, instance) in design
            .assembly
            .instances_with_role(ComponentRole::RollBearingCarrierEnd)
        {
            let pose = design
                .kinematics
                .pose(command(0.0, 0.0))
                .expect("zero pose is valid")
                .frame(instance.frame)
                .expect("instance frame exists")
                .compose(instance.local_pose);
            assert!(
                pose.translation[0].abs() > cockpit_half_length,
                "roll bearing carrier end {} must remain beyond the cockpit end plane",
                instance.name
            );
        }
        assert!(
            design
                .assembly
                .instances()
                .iter()
                .all(|instance| !instance.name.contains("moving_crossbar")),
            "the obsolete cockpit-underbody crossbar must remain absent"
        );
    }

    #[test]
    fn retention_supports_are_not_fixed_to_the_outer_frame() {
        let design = load_design();
        for component in [
            located(
                ComponentRole::RetentionBearingBlock,
                ComponentLocation::new()
                    .with_side(Side::Left)
                    .with_longitudinal_end(LongitudinalEnd::Front),
            ),
            located(
                ComponentRole::RetentionLeafSpring,
                ComponentLocation::new()
                    .with_side(Side::Right)
                    .with_longitudinal_end(LongitudinalEnd::Rear)
                    .with_ordinal(1),
            ),
        ] {
            let zero = instance_pose(&design, component, 0.0, 0.0);
            let pitched = instance_pose(&design, component, 20.0, 0.0);
            assert_ne!(
                zero, pitched,
                "{component:?} must travel with the pitch unit"
            );
        }
    }

    #[test]
    fn base_frame_contacts_floor_and_roll_gearboxes_are_below_axis() {
        let design = load_design();
        let floor = instance_pose(
            &design,
            singleton(ComponentRole::InstallationFloor),
            0.0,
            0.0,
        );
        let lower_rail = instance_pose(
            &design,
            located(
                ComponentRole::FixedCarrierRail,
                ComponentLocation::new()
                    .with_side(Side::Left)
                    .with_vertical_end(VerticalEnd::Lower),
            ),
            0.0,
            0.0,
        );
        let floor_top = floor.translation[2] + 5.0;
        let rail_bottom = lower_rail.translation[2] - 4.0;
        assert!((floor_top - rail_bottom).abs() < 1.0e-8);

        for end in [LongitudinalEnd::Front, LongitudinalEnd::Rear] {
            let location = ComponentLocation::new().with_longitudinal_end(end);
            let driven = instance_pose(
                &design,
                located(ComponentRole::RollDrivenGear, location),
                0.0,
                0.0,
            );
            let input = instance_pose(
                &design,
                located(
                    ComponentRole::RollGearboxSmallGear,
                    location.with_ordinal(2),
                ),
                0.0,
                0.0,
            );
            let plate = instance_pose(
                &design,
                located(ComponentRole::RollGearboxPlate, location.with_ordinal(1)),
                0.0,
                0.0,
            );
            assert!(input.translation[2] < driven.translation[2]);
            assert!(plate.translation[2] < driven.translation[2]);
        }
    }

    #[test]
    fn cockpit_is_suspended_and_gravity_has_a_restoring_direction() {
        let design = load_design();
        let shaft = instance_pose(&design, singleton(ComponentRole::RollShaft), 0.0, 0.0);
        let cockpit = singleton(ComponentRole::Cockpit);
        let cockpit_zero = instance_pose(&design, cockpit, 0.0, 0.0);
        let cockpit_rolled = instance_pose(&design, cockpit, 0.0, 35.0);
        assert!(cockpit_zero.translation[2] < shaft.translation[2]);
        assert!(cockpit_zero.translation[2] < cockpit_rolled.translation[2]);
    }

    #[test]
    fn pitch_pinion_spin_includes_orbit_about_the_fixed_rack() {
        let design = load_design();
        let pitch = 1.0_f64;
        let drive = instance_pose(
            &design,
            located(
                ComponentRole::PitchDrivePinion,
                ComponentLocation::new()
                    .with_side(Side::Left)
                    .with_longitudinal_end(LongitudinalEnd::Front)
                    .with_ordinal(1),
            ),
            pitch,
            0.0,
        );
        let encoder = instance_pose(
            &design,
            located(
                ComponentRole::PitchRetentionPinion,
                ComponentLocation::new()
                    .with_side(Side::Left)
                    .with_longitudinal_end(LongitudinalEnd::Front),
            ),
            pitch,
            0.0,
        );
        let drive_angle = quaternion_y_degrees(drive.rotation);
        let encoder_angle = quaternion_y_degrees(encoder.rotation);
        let expected_drive = pitch * (1.0 + design.pitch_drive_pair.ratio());
        let expected_encoder = pitch * (1.0 - design.pitch_encoder_pair.ratio());
        assert!((drive_angle - expected_drive).abs() < 1.0e-6);
        assert!((encoder_angle - expected_encoder).abs() < 1.0e-6);
    }

    #[test]
    fn moving_assembly_clears_the_floor_over_the_command_envelope() {
        let design = load_design();
        let floor = singleton(ComponentRole::InstallationFloor);
        let floor_pose = instance_pose(&design, floor, 0.0, 0.0);
        let mut evaluator =
            Evaluator::with_mode(&design.graph, GeometryEvaluationMode::StructuralProxy);
        let floor_mesh = evaluator
            .mesh(instance_solid(&design, floor))
            .expect("floor mesh evaluates");
        let floor_top_z = floor_mesh
            .vertices
            .iter()
            .map(|vertex| floor_pose.transform_point(*vertex)[2])
            .fold(f64::NEG_INFINITY, f64::max);

        let moving_instances = design
            .assembly
            .instances_with_ids()
            .filter_map(|(id, instance)| {
                let definition = design
                    .assembly
                    .definition(instance.definition)
                    .expect("instance definition exists");
                if definition.role.has_high_detail_gear_geometry() {
                    return None;
                }
                let zero = instance_pose_by_id(&design, id, 0.0, 0.0);
                let pitched = instance_pose_by_id(&design, id, 1.0, 0.0);
                let rolled = instance_pose_by_id(&design, id, 0.0, 1.0);
                (zero != pitched || zero != rolled).then_some((id, instance))
            })
            .collect::<Vec<_>>();
        assert!(
            !moving_instances.is_empty(),
            "the mechanism must contain moving instances"
        );

        let mut meshes = HashMap::new();
        for (instance_id, _) in &moving_instances {
            let solid = instance_solid_by_id(&design, *instance_id);
            if let std::collections::hash_map::Entry::Vacant(entry) = meshes.entry(solid) {
                entry.insert(evaluator.mesh(solid).expect("moving mesh evaluates"));
            }
        }

        let required_clearance_mm = 5.0;
        for pitch in [-20.0, 0.0, 20.0] {
            for roll in [-35.0, 0.0, 35.0] {
                for (instance_id, instance) in &moving_instances {
                    let pose = instance_pose_by_id(&design, *instance_id, pitch, roll);
                    let minimum_z = meshes[&instance_solid_by_id(&design, *instance_id)]
                        .vertices
                        .iter()
                        .map(|vertex| pose.transform_point(*vertex)[2])
                        .fold(f64::INFINITY, f64::min);
                    let clearance_mm = minimum_z - floor_top_z;
                    assert!(
                        clearance_mm >= required_clearance_mm - 1.0e-7,
                        "{} has only {clearance_mm} mm floor clearance at pitch={pitch}, roll={roll}; required {required_clearance_mm} mm",
                        instance.name
                    );
                }
            }
        }
    }

    #[test]
    fn central_pinion_keepout_has_no_obsolete_16_mm_sector_backbone() {
        let design = load_design();
        let mut evaluator = Evaluator::new(&design.graph);
        let sector = located(
            ComponentRole::PitchSector,
            ComponentLocation::new()
                .with_side(Side::Left)
                .with_longitudinal_end(LongitudinalEnd::Front),
        );
        let sector_mesh = evaluator
            .mesh(instance_solid(&design, sector))
            .expect("unreinforced sector evaluates to a manifold mesh");
        let keepout_vertices = sector_mesh
            .vertices
            .iter()
            .filter(|vertex| vertex[2].abs() < 39.0)
            .collect::<Vec<_>>();
        assert!(
            !keepout_vertices.is_empty(),
            "sector mesh must cross the central pinion keep-out"
        );
        let minimum_y = keepout_vertices
            .iter()
            .map(|vertex| vertex[1])
            .fold(f64::INFINITY, f64::min);
        let maximum_y = keepout_vertices
            .iter()
            .map(|vertex| vertex[1])
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            maximum_y - minimum_y <= 8.01,
            "obsolete 16 mm sector backbone is still present"
        );
    }

    #[test]
    fn shortened_cockpit_clears_pitch_frame_roll_supports() {
        let design = load_design();
        let cockpit = singleton(ComponentRole::Cockpit);
        let cockpit_solid = instance_solid(&design, cockpit);
        let fixed_to_pitch_frame = [
            located(
                ComponentRole::RollBearingCarrierEnd,
                ComponentLocation::new().with_longitudinal_end(LongitudinalEnd::Front),
            ),
            located(
                ComponentRole::RollBearingCarrierEnd,
                ComponentLocation::new().with_longitudinal_end(LongitudinalEnd::Rear),
            ),
            located(
                ComponentRole::MovingDriveMountArm,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Front)
                    .with_ordinal(1),
            ),
            located(
                ComponentRole::MovingDriveMountArm,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Front)
                    .with_ordinal(2),
            ),
            located(
                ComponentRole::MovingDriveMountArm,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Rear)
                    .with_ordinal(1),
            ),
            located(
                ComponentRole::MovingDriveMountArm,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Rear)
                    .with_ordinal(2),
            ),
        ];
        let mut evaluator = Evaluator::new(&design.graph);
        for pitch in [-20.0, 0.0, 20.0] {
            for roll in [-35.0, 0.0, 35.0] {
                let cockpit_pose = instance_pose(&design, cockpit, pitch, roll);
                for support in fixed_to_pitch_frame {
                    let volume = evaluator
                        .intersection_volume_transformed(
                            cockpit_solid,
                            cockpit_pose,
                            instance_solid(&design, support),
                            instance_pose(&design, support, pitch, roll),
                        )
                        .expect("cockpit clearance query succeeds");
                    assert!(
                        volume <= 1.0e-7,
                        "cockpit intersects {support:?} by {volume} mm^3 at pitch={pitch}, roll={roll}"
                    );
                }
            }
        }
    }

    #[test]
    fn gearbox_plates_clear_their_gears_and_shafts() {
        let design = load_design();
        let mut evaluator = Evaluator::new(&design.graph);
        let right_front = ComponentLocation::new()
            .with_side(Side::Right)
            .with_longitudinal_end(LongitudinalEnd::Front);
        let roll_front = ComponentLocation::new().with_longitudinal_end(LongitudinalEnd::Front);
        let groups: &[(ComponentIdentity, &[ComponentIdentity])] = &[
            (
                located(ComponentRole::PitchContactCarriagePlate, right_front),
                &[
                    located(ComponentRole::PitchDrivePinion, right_front.with_ordinal(1)),
                    located(ComponentRole::PitchDrivePinion, right_front.with_ordinal(2)),
                    located(
                        ComponentRole::PitchGearboxSmallGear,
                        right_front.with_ordinal(1),
                    ),
                    located(
                        ComponentRole::PitchGearboxSmallGear,
                        right_front.with_ordinal(2),
                    ),
                    located(
                        ComponentRole::PitchGearboxDistributionGear,
                        right_front.with_ordinal(3),
                    ),
                    located(
                        ComponentRole::PitchGearboxLargeGear,
                        right_front.with_ordinal(1),
                    ),
                    located(
                        ComponentRole::PitchGearboxSmallGear,
                        right_front.with_ordinal(4),
                    ),
                    located(
                        ComponentRole::PitchGearboxLargeGear,
                        right_front.with_ordinal(2),
                    ),
                    located(
                        ComponentRole::PitchGearboxSmallGear,
                        right_front.with_ordinal(5),
                    ),
                ],
            ),
            (
                located(ComponentRole::PitchContactOutboardPlate, right_front),
                &[
                    located(ComponentRole::PitchDrivePinion, right_front.with_ordinal(1)),
                    located(ComponentRole::PitchDrivePinion, right_front.with_ordinal(2)),
                    located(ComponentRole::PitchRetentionPinion, right_front),
                    located(ComponentRole::PitchDriveShaft, right_front.with_ordinal(1)),
                    located(ComponentRole::PitchDriveShaft, right_front.with_ordinal(2)),
                    located(ComponentRole::PitchRetentionShaft, right_front),
                ],
            ),
            (
                located(ComponentRole::PitchGearboxFarPlate, right_front),
                &[
                    located(
                        ComponentRole::PitchGearboxLargeGear,
                        right_front.with_ordinal(1),
                    ),
                    located(
                        ComponentRole::PitchGearboxSmallGear,
                        right_front.with_ordinal(4),
                    ),
                    located(
                        ComponentRole::PitchGearboxLargeGear,
                        right_front.with_ordinal(2),
                    ),
                    located(
                        ComponentRole::PitchGearboxSmallGear,
                        right_front.with_ordinal(5),
                    ),
                    located(
                        ComponentRole::PitchGearboxShaft,
                        right_front.with_ordinal(1),
                    ),
                    located(
                        ComponentRole::PitchGearboxShaft,
                        right_front.with_ordinal(2),
                    ),
                    located(
                        ComponentRole::PitchGearboxShaft,
                        right_front.with_ordinal(3),
                    ),
                ],
            ),
            (
                located(ComponentRole::RollGearboxPlate, roll_front.with_ordinal(1)),
                &[
                    located(ComponentRole::RollInputPinion, roll_front),
                    located(
                        ComponentRole::RollGearboxLargeGear,
                        roll_front.with_ordinal(1),
                    ),
                    located(
                        ComponentRole::RollGearboxSmallGear,
                        roll_front.with_ordinal(1),
                    ),
                    located(
                        ComponentRole::RollGearboxLargeGear,
                        roll_front.with_ordinal(2),
                    ),
                    located(
                        ComponentRole::RollGearboxSmallGear,
                        roll_front.with_ordinal(2),
                    ),
                    located(ComponentRole::RollGearboxShaft, roll_front.with_ordinal(1)),
                    located(ComponentRole::RollGearboxShaft, roll_front.with_ordinal(2)),
                    located(ComponentRole::RollGearboxShaft, roll_front.with_ordinal(3)),
                ],
            ),
        ];
        for (plate, parts) in groups {
            for part in *parts {
                let volume = evaluator
                    .intersection_volume_transformed(
                        instance_solid(&design, *plate),
                        instance_pose(&design, *plate, 0.0, 0.0),
                        instance_solid(&design, *part),
                        instance_pose(&design, *part, 0.0, 0.0),
                    )
                    .expect("gearbox interference query succeeds");
                assert!(
                    volume <= 1.0e-7,
                    "{plate:?} intersects {part:?} by {volume} mm^3"
                );
            }
        }
    }

    #[test]
    fn upper_carrier_and_roll_mounts_do_not_use_solid_overlap() {
        let design = load_design();
        let mut evaluator = Evaluator::new(&design.graph);
        let mut pairs = Vec::new();
        for end in [LongitudinalEnd::Front, LongitudinalEnd::Rear] {
            let end_location = ComponentLocation::new().with_longitudinal_end(end);
            let tie = located(ComponentRole::RollBearingCarrierEnd, end_location);
            for side in [Side::Left, Side::Right] {
                pairs.push((
                    tie,
                    located(
                        ComponentRole::PitchGearboxFarPlate,
                        ComponentLocation::new()
                            .with_side(side)
                            .with_longitudinal_end(end),
                    ),
                ));
            }
            let hub = located(ComponentRole::RollDrivenHub, end_location);
            for ordinal in [1, 2] {
                let arm = located(
                    ComponentRole::MovingDriveMountArm,
                    end_location.with_ordinal(ordinal),
                );
                pairs.push((hub, arm));
                for plate_ordinal in [1, 2] {
                    pairs.push((
                        located(
                            ComponentRole::RollGearboxPlate,
                            end_location.with_ordinal(plate_ordinal),
                        ),
                        arm,
                    ));
                }
            }
        }
        for (first, second) in pairs {
            let volume = evaluator
                .intersection_volume_transformed(
                    instance_solid(&design, first),
                    instance_pose(&design, first, 0.0, 0.0),
                    instance_solid(&design, second),
                    instance_pose(&design, second, 0.0, 0.0),
                )
                .expect("structural interference query succeeds");
            assert!(
                volume <= 1.0e-7,
                "{first:?} intersects {second:?} by {volume} mm^3"
            );
        }
    }

    fn quaternion_y_degrees(rotation: [f64; 4]) -> f64 {
        2.0 * rotation[1].atan2(rotation[3]) * 180.0 / core::f64::consts::PI
    }

    fn distance(a: RigidTransform, b: RigidTransform) -> f64 {
        let dx = a.translation[0] - b.translation[0];
        let dy = a.translation[1] - b.translation[1];
        let dz = a.translation[2] - b.translation[2];
        dx.hypot(dy).hypot(dz)
    }
}

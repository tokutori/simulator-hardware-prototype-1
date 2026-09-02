// SPDX-License-Identifier: MIT

pub mod config;

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
    AssemblyValidator, Evaluator, GeometryFidelity, MotionCoverage, RelationValidationStatus,
    UnrelatedProximityPolicy, ValidationIssueKind, ValidationProfile, ValidationProgress,
    ValidationReport, ValidatorSettings,
};
use serde_json::{Value, json};

pub fn run() -> Result<(), Box<dyn Error>> {
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
        "validate" => validate(&workspace, &loaded, ValidationProfile::STRUCTURAL_STATIC),
        "validate-full" => validate(&workspace, &loaded, ValidationProfile::EXACT_STATIC),
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
    profile: ValidationProfile,
) -> Result<(), Box<dyn Error>> {
    let design = build_prototype(&loaded.parameters)
        .map_err(|error| format!("prototype design rejected: {error:?}"))?;
    let report = validate_assembly(&design, profile)?;
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
        report.profile,
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
        let report = validate_assembly(&design, ValidationProfile::EXACT_STATIC)?;
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
    println!(
        "generated {} component definitions and {} instances in {} ({mode:?})",
        design.assembly.definitions().len(),
        export_parts.len(),
        output.display()
    );
    Ok(())
}

pub fn validate_assembly(
    design: &PrototypeDesign,
    profile: ValidationProfile,
) -> Result<ValidationReport, Box<dyn Error>> {
    let zero_pose = design
        .kinematics
        .pose(PitchRollCommand {
            pitch: Angle::degrees(0.0).expect("zero angle is valid"),
            roll: Angle::degrees(0.0).expect("zero angle is valid"),
        })
        .map_err(|error| format!("zero pose rejected: {error:?}"))?;
    let settings = ValidatorSettings {
        profile,
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
    let scoped_name = match report.profile.geometry {
        GeometryFidelity::StructuralProxy => "validation-report-structural.json",
        GeometryFidelity::Exact => "validation-report-full.json",
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
    let relation_checks = report
        .relation_checks
        .iter()
        .map(|check| {
            let relation = &design.assembly.relations()[check.relation.index()];
            let kind = match relation {
                gimbal_core::AssemblyRelation::SurfaceContact(_) => "surface-contact",
                gimbal_core::AssemblyRelation::Fastened(_) => "fastened",
                gimbal_core::AssemblyRelation::CylindricalFit(_) => "cylindrical-fit",
                gimbal_core::AssemblyRelation::GearMesh(_) => "gear-mesh",
            };
            let status = match check.status {
                RelationValidationStatus::Validated => "validated",
                RelationValidationStatus::Failed => "failed",
                RelationValidationStatus::SkippedByScope => "skipped-by-scope",
                RelationValidationStatus::Unsupported => "unsupported",
            };
            json!({
                "relation_id": check.relation.index(),
                "kind": kind,
                "status": status,
            })
        })
        .collect::<Vec<_>>();
    json!({
        "complete": report.is_complete(),
        "preview_only": false,
        "valid": report.is_valid(),
        "profile": {
            "geometry_fidelity": match report.profile.geometry {
                GeometryFidelity::StructuralProxy => "structural-proxy",
                GeometryFidelity::Exact => "exact",
            },
            "motion_coverage": match report.profile.motion {
                MotionCoverage::StaticPose => "static-pose",
            },
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
        "relation_checks": relation_checks,
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

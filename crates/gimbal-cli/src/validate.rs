// SPDX-License-Identifier: MIT

use crate::config::LoadedConfig;
use geared_gimbal_design::{PrototypeDesign, build_prototype};
use gimbal_core::{
    Angle, NonNegativeLength, NumericalTolerance, PitchRollCommand, PositiveArea, PositiveLength,
    PositiveVolume,
};
use gimbal_kernel_manifold::{
    AssemblyValidator, GeometryFidelity, MotionCoverage, RelationValidationStatus,
    UnrelatedProximityPolicy, ValidationIssueKind, ValidationPlan, ValidationProfile,
    ValidationProgress, ValidationReport, ValidatorSettings,
};
use serde_json::{Value, json};
use std::error::Error;
use std::fs;
use std::path::Path;

pub(crate) fn validate(
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
    let plan = match profile.geometry {
        GeometryFidelity::Exact => ValidationPlan::all(profile),
        GeometryFidelity::StructuralProxy => ValidationPlan::include_only(
            profile,
            design
                .assembly
                .definitions_with_ids()
                .filter_map(|(id, definition)| {
                    (!definition.role.has_high_detail_gear_geometry()).then_some(id)
                }),
        ),
    };
    let settings = ValidatorSettings {
        plan,
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

pub(crate) fn require_valid_assembly(report: &ValidationReport) -> Result<(), Box<dyn Error>> {
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

pub(crate) fn write_validation_report(
    workspace: &Path,
    design: &PrototypeDesign,
    report: &ValidationReport,
) -> Result<(), Box<dyn Error>> {
    let output = workspace.join("output");
    write_validation_report_to(&output, design, report)
}

pub(crate) fn write_validation_report_to(
    output: &Path,
    design: &PrototypeDesign,
    report: &ValidationReport,
) -> Result<(), Box<dyn Error>> {
    fs::create_dir_all(output)?;
    let bytes = serde_json::to_vec_pretty(&validation_report_json(design, report))?;
    fs::write(output.join("validation-report.json"), &bytes)?;
    let scoped_name = match report.profile.geometry {
        GeometryFidelity::StructuralProxy => "validation-report-structural.json",
        GeometryFidelity::Exact => "validation-report-full.json",
    };
    fs::write(output.join(scoped_name), bytes)?;
    Ok(())
}

pub(crate) fn validation_report_json(design: &PrototypeDesign, report: &ValidationReport) -> Value {
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
                ValidationIssueKind::PlaneClearanceSeparationMismatch {
                    actual_mm,
                    target_mm,
                    allowed_mm,
                } => (
                    "plane_clearance_separation_mismatch",
                    json!({
                        "actual_mm": actual_mm,
                        "target_mm": target_mm,
                        "allowed_mm": allowed_mm
                    }),
                ),
                ValidationIssueKind::PlaneClearanceNormalMismatch {
                    error_radians,
                    allowed_radians,
                } => (
                    "plane_clearance_normal_mismatch",
                    json!({
                        "error_radians": error_radians,
                        "allowed_radians": allowed_radians
                    }),
                ),
                ValidationIssueKind::PlaneClearanceAreaInsufficient {
                    overlap_area_mm2,
                    minimum_area_mm2,
                } => (
                    "plane_clearance_area_insufficient",
                    json!({
                        "overlap_area_mm2": overlap_area_mm2,
                        "minimum_area_mm2": minimum_area_mm2
                    }),
                ),
                ValidationIssueKind::CylindricalFitAxisSeparation {
                    distance_mm,
                    allowed_mm,
                } => (
                    "cylindrical_fit_axis_separation",
                    json!({ "distance_mm": distance_mm, "allowed_mm": allowed_mm }),
                ),
                ValidationIssueKind::CylindricalFitAxisMismatch {
                    error_radians,
                    allowed_radians,
                } => (
                    "cylindrical_fit_axis_mismatch",
                    json!({
                        "error_radians": error_radians,
                        "allowed_radians": allowed_radians
                    }),
                ),
                ValidationIssueKind::CylindricalFitClearanceMismatch {
                    actual_radial_clearance_mm,
                    target_radial_clearance_mm,
                    allowed_mm,
                } => (
                    "cylindrical_fit_clearance_mismatch",
                    json!({
                        "actual_radial_clearance_mm": actual_radial_clearance_mm,
                        "target_radial_clearance_mm": target_radial_clearance_mm,
                        "allowed_mm": allowed_mm
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
                gimbal_core::AssemblyRelation::PlaneClearance(_) => "plane-clearance",
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

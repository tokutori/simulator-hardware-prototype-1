// SPDX-License-Identifier: MIT

use gimbal_core::{
    AssemblyRelationId, ComponentDefinitionId, ComponentIdentity, ComponentInstanceId,
    ComponentInstancePair,
};

use super::ValidationProfile;
use crate::SolidMetrics;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ValidationSeverity {
    Warning,
    Error,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ValidationIssueKind {
    DuplicateComponentIdentity {
        identity: ComponentIdentity,
    },
    UnexpectedInterference {
        intersection_volume_mm3: f64,
    },
    PotentialStructuralInterference {
        proxy_aabb_overlap_mm3: f64,
    },
    SurfaceContactSeparation {
        distance_mm: f64,
        allowed_mm: f64,
    },
    SurfaceContactNormalMismatch {
        error_radians: f64,
        allowed_radians: f64,
    },
    SurfaceContactAreaInsufficient {
        contact_area_mm2: f64,
        minimum_area_mm2: f64,
    },
    PlaneClearanceSeparationMismatch {
        actual_mm: f64,
        target_mm: f64,
        allowed_mm: f64,
    },
    PlaneClearanceNormalMismatch {
        error_radians: f64,
        allowed_radians: f64,
    },
    PlaneClearanceAreaInsufficient {
        overlap_area_mm2: f64,
        minimum_area_mm2: f64,
    },
    CylindricalFitOriginSeparation {
        distance_mm: f64,
        allowed_mm: f64,
    },
    CylindricalFitAxisMismatch {
        error_radians: f64,
        allowed_radians: f64,
    },
    CylindricalFitClearanceMismatch {
        actual_radial_clearance_mm: f64,
        target_radial_clearance_mm: f64,
        allowed_mm: f64,
    },
    FastenerHoleAxisSeparation {
        distance_mm: f64,
        allowed_mm: f64,
    },
    FastenerHoleAxisMismatch {
        error_radians: f64,
        allowed_radians: f64,
    },
    FastenerHoleRadiusMismatch {
        first_radius_mm: f64,
        second_radius_mm: f64,
        expected_radius_mm: f64,
        allowed_mm: f64,
    },
    FastenerSeatNormalMismatch {
        error_radians: f64,
        allowed_radians: f64,
    },
    FastenerGripLengthMismatch {
        actual_mm: f64,
        expected_mm: f64,
        allowed_mm: f64,
    },
    FastenerHardwareAxisSeparation {
        hardware: ComponentInstanceId,
        distance_mm: f64,
        allowed_mm: f64,
    },
    FastenerHardwareAxisMismatch {
        hardware: ComponentInstanceId,
        error_radians: f64,
        allowed_radians: f64,
    },
    FastenerHardwareContactMismatch {
        first: ComponentInstanceId,
        second: ComponentInstanceId,
        separation_mm: f64,
        normal_error_radians: f64,
        allowed_mm: f64,
        allowed_radians: f64,
    },
    FastenerThreadEngagementInsufficient {
        actual_mm: f64,
        minimum_mm: f64,
    },
    FastenerBoltProtrusionInsufficient {
        actual_mm: f64,
        minimum_mm: f64,
    },
    UnspecifiedProximity {
        gap_mm: f64,
        threshold_mm: f64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ValidationIssue {
    pub severity: ValidationSeverity,
    pub pair: Option<ComponentInstancePair>,
    pub relation: Option<AssemblyRelationId>,
    pub kind: ValidationIssueKind,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DefinitionValidation {
    pub definition: ComponentDefinitionId,
    pub metrics: SolidMetrics,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PairValidation {
    pub pair: ComponentInstancePair,
    pub relation_ids: Vec<AssemblyRelationId>,
    pub method: PairCheckMethod,
    pub intersection_volume_mm3: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PairCheckMethod {
    StructuralProxyAabb,
    ExactSolid,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RelationValidationStatus {
    Validated,
    Failed,
    SkippedByScope,
    Unsupported,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RelationValidation {
    pub relation: AssemblyRelationId,
    pub status: RelationValidationStatus,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ValidationReport {
    pub profile: ValidationProfile,
    pub definitions: Vec<DefinitionValidation>,
    pub skipped_definitions: Vec<ComponentDefinitionId>,
    pub skipped_instances: Vec<ComponentInstanceId>,
    pub total_instance_pairs: usize,
    pub eligible_instance_pairs: usize,
    pub broad_phase_candidates: usize,
    pub unrelated_proximity_checks: usize,
    pub skipped_relation_checks: usize,
    pub relation_checks: Vec<RelationValidation>,
    pub pair_checks: Vec<PairValidation>,
    pub issues: Vec<ValidationIssue>,
}

impl ValidationReport {
    pub fn is_complete(&self) -> bool {
        self.relation_checks.iter().all(|check| {
            matches!(
                check.status,
                RelationValidationStatus::Validated
                    | RelationValidationStatus::Failed
                    | RelationValidationStatus::SkippedByScope
            )
        })
    }

    pub fn is_valid(&self) -> bool {
        self.is_complete()
            && !self
                .issues
                .iter()
                .any(|issue| issue.severity == ValidationSeverity::Error)
    }

    pub fn error_count(&self) -> usize {
        self.issues
            .iter()
            .filter(|issue| issue.severity == ValidationSeverity::Error)
            .count()
    }

    pub fn warning_count(&self) -> usize {
        self.issues
            .iter()
            .filter(|issue| issue.severity == ValidationSeverity::Warning)
            .count()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ValidationProgress {
    BroadPhaseComplete {
        candidates: usize,
    },
    PairCheck {
        current: usize,
        total: usize,
        pair: ComponentInstancePair,
    },
}

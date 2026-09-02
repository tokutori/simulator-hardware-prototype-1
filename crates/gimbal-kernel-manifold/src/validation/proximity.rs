// SPDX-License-Identifier: MIT

use super::{
    AssemblyValidator, GeometryFidelity, InstanceGeometry, UnrelatedProximityPolicy,
    ValidationIssue, ValidationIssueKind, ValidationSeverity,
};

impl AssemblyValidator<'_> {
    pub(super) fn validate_unrelated_proximity(
        &self,
        instances: &[Option<InstanceGeometry>],
        linear_epsilon: f64,
        issues: &mut Vec<ValidationIssue>,
    ) -> usize {
        let severity = match self.settings.unrelated_proximity_policy {
            UnrelatedProximityPolicy::Ignore => return 0,
            UnrelatedProximityPolicy::Warning => ValidationSeverity::Warning,
            UnrelatedProximityPolicy::Error => ValidationSeverity::Error,
        };
        let threshold_mm = self.settings.unrelated_proximity_threshold.as_mm();
        let search_length = threshold_mm + linear_epsilon;
        let mut checks = 0;
        for pair in self.assembly.unrelated_instance_pairs() {
            let Some(first) = &instances[pair.first.index()] else {
                continue;
            };
            let Some(second) = &instances[pair.second.index()] else {
                continue;
            };
            if first
                .bounds
                .has_interior_overlap(second.bounds, linear_epsilon)
                || !first.bounds.is_within_gap(second.bounds, search_length)
            {
                continue;
            }
            checks += 1;
            let gap_mm = match self.settings.profile.geometry {
                GeometryFidelity::StructuralProxy => first.bounds.minimum_gap(second.bounds),
                GeometryFidelity::Exact => first.solid.min_gap(&second.solid, search_length),
            };
            if gap_mm <= search_length {
                issues.push(ValidationIssue {
                    severity,
                    pair: Some(pair),
                    relation: None,
                    kind: ValidationIssueKind::UnspecifiedProximity {
                        gap_mm,
                        threshold_mm,
                    },
                });
            }
        }
        checks
    }
}

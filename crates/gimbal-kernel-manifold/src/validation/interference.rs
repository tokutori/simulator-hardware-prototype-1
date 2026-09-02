// SPDX-License-Identifier: MIT

use super::{
    Aabb3, AssemblyValidator, DefinitionValidation, GeometryFidelity, InstanceGeometry,
    PairCheckMethod, PairValidation, RelationValidationStatus, ValidationError, ValidationIssue,
    ValidationIssueKind, ValidationProgress, ValidationReport, ValidationSeverity,
    ValidatorSettings,
};
use crate::{Evaluator, GeometryEvaluationMode, transform};
use gimbal_core::{Assembly, AssemblyPose, ComponentInstancePair, FeatureGraph};
use manifold_rust::types::{BooleanEngine, Error as ManifoldStatus};
use rayon::prelude::*;
use std::panic::{AssertUnwindSafe, catch_unwind};

impl<'a> AssemblyValidator<'a> {
    pub const fn new(
        graph: &'a FeatureGraph,
        assembly: &'a Assembly,
        pose: &'a AssemblyPose,
        settings: ValidatorSettings,
    ) -> Self {
        Self {
            graph,
            assembly,
            pose,
            settings,
        }
    }

    pub fn validate(&self) -> Result<ValidationReport, ValidationError> {
        self.validate_with_progress(|_| {})
    }

    pub fn validate_with_progress(
        &self,
        mut progress: impl FnMut(ValidationProgress),
    ) -> Result<ValidationReport, ValidationError> {
        let evaluation_mode = match self.settings.profile.geometry {
            GeometryFidelity::StructuralProxy => GeometryEvaluationMode::StructuralProxy,
            GeometryFidelity::Exact => GeometryEvaluationMode::Robust,
        };
        let mut evaluator = Evaluator::with_mode(self.graph, evaluation_mode);
        let mut definitions = Vec::new();
        let mut skipped_definitions = Vec::new();
        for (definition_id, definition) in self.assembly.definitions_with_ids() {
            if self.settings.profile.geometry.includes(definition.role) {
                definitions.push(DefinitionValidation {
                    definition: definition_id,
                    metrics: evaluator.metrics(definition.body.assembly_solid())?,
                });
            } else {
                skipped_definitions.push(definition_id);
            }
        }

        let mut issues = self
            .assembly
            .component_identity_collisions()
            .into_iter()
            .map(|collision| ValidationIssue {
                severity: ValidationSeverity::Error,
                pair: Some(ComponentInstancePair {
                    first: collision.first,
                    second: collision.second,
                }),
                relation: None,
                kind: ValidationIssueKind::DuplicateComponentIdentity {
                    identity: collision.identity,
                },
            })
            .collect::<Vec<_>>();

        let mut skipped_instances = Vec::new();
        let instances = self
            .assembly
            .instances_with_ids()
            .map(|(id, instance)| {
                let definition = self
                    .assembly
                    .definition(instance.definition)
                    .expect("inserted instance references a validated definition");
                if !self.settings.profile.geometry.includes(definition.role) {
                    skipped_instances.push(id);
                    return Ok(None);
                }
                let frame_pose = self
                    .pose
                    .frame(instance.frame)
                    .ok_or(ValidationError::MissingFrame(id))?;
                let world_pose = frame_pose.compose(instance.local_pose);
                let solid = transform(
                    evaluator.evaluate(definition.body.assembly_solid())?,
                    world_pose,
                );
                let bounds = Aabb3::from_manifold(&solid);
                Ok(Some(InstanceGeometry {
                    solid,
                    bounds,
                    world_pose,
                }))
            })
            .collect::<Result<Vec<Option<InstanceGeometry>>, ValidationError>>()?;

        let relation_checks = self.validate_relations(&instances, &mut issues);
        let skipped_relation_checks = relation_checks
            .iter()
            .filter(|check| check.status == RelationValidationStatus::SkippedByScope)
            .count();

        let linear_epsilon = self.settings.numerical_tolerance.linear_epsilon.as_mm();
        let volume_epsilon = self
            .settings
            .numerical_tolerance
            .volume_epsilon
            .as_cubic_mm();
        let total_instance_pairs = self.assembly.instance_pairs().count();
        let eligible_instance_pairs = self
            .assembly
            .instance_pairs()
            .filter(|pair| {
                instances[pair.first.index()].is_some() && instances[pair.second.index()].is_some()
            })
            .count();
        let candidate_pairs = self
            .assembly
            .instance_pairs()
            .filter(|pair| {
                let Some(first) = &instances[pair.first.index()] else {
                    return false;
                };
                let Some(second) = &instances[pair.second.index()] else {
                    return false;
                };
                first
                    .bounds
                    .has_interior_overlap(second.bounds, linear_epsilon)
            })
            .collect::<Vec<_>>();
        let broad_phase_candidates = candidate_pairs.len();
        progress(ValidationProgress::BroadPhaseComplete {
            candidates: broad_phase_candidates,
        });
        let mut pair_checks = Vec::new();
        const PAIR_CHUNK_SIZE: usize = 32;
        for (chunk_index, chunk) in candidate_pairs.chunks(PAIR_CHUNK_SIZE).enumerate() {
            let evaluated = match self.settings.profile.geometry {
                GeometryFidelity::StructuralProxy => chunk
                    .iter()
                    .map(|pair| {
                        let first = instances[pair.first.index()]
                            .as_ref()
                            .expect("candidate pair contains included instances");
                        let second = instances[pair.second.index()]
                            .as_ref()
                            .expect("candidate pair contains included instances");
                        (*pair, first.bounds.interior_overlap_volume(second.bounds))
                    })
                    .collect::<Vec<_>>(),
                GeometryFidelity::Exact => chunk
                    .par_iter()
                    .map(|pair| {
                        let first = instances[pair.first.index()]
                            .as_ref()
                            .expect("candidate pair contains included instances");
                        let second = instances[pair.second.index()]
                            .as_ref()
                            .expect("candidate pair contains included instances");
                        if first.solid.min_gap(&second.solid, linear_epsilon) > 0.0 {
                            return Ok((*pair, 0.0));
                        }
                        let intersection = catch_unwind(AssertUnwindSafe(|| {
                            first
                                .solid
                                .intersection_with_engine(&second.solid, BooleanEngine::Robust)
                        }))
                        .map_err(|_| ValidationError::PairPanicked(*pair))?;
                        if intersection.status() != ManifoldStatus::NoError {
                            return Err(ValidationError::PairIntersection {
                                pair: *pair,
                                status: intersection.status(),
                            });
                        }
                        Ok((*pair, intersection.volume()))
                    })
                    .collect::<Result<Vec<_>, ValidationError>>()?,
            };
            for (pair, intersection_volume_mm3) in evaluated {
                if intersection_volume_mm3 <= volume_epsilon {
                    continue;
                }
                let relation_ids = self
                    .assembly
                    .relations_between(pair)
                    .map(|(id, _)| id)
                    .collect::<Vec<_>>();
                let method = match self.settings.profile.geometry {
                    GeometryFidelity::StructuralProxy => PairCheckMethod::StructuralProxyAabb,
                    GeometryFidelity::Exact => PairCheckMethod::ExactSolid,
                };
                pair_checks.push(PairValidation {
                    pair,
                    relation_ids,
                    method,
                    intersection_volume_mm3,
                });
                let kind = match self.settings.profile.geometry {
                    GeometryFidelity::StructuralProxy => {
                        ValidationIssueKind::PotentialStructuralInterference {
                            proxy_aabb_overlap_mm3: intersection_volume_mm3,
                        }
                    }
                    GeometryFidelity::Exact => ValidationIssueKind::UnexpectedInterference {
                        intersection_volume_mm3,
                    },
                };
                issues.push(ValidationIssue {
                    severity: ValidationSeverity::Error,
                    pair: Some(pair),
                    relation: None,
                    kind,
                });
            }
            let completed = ((chunk_index + 1) * PAIR_CHUNK_SIZE).min(broad_phase_candidates);
            let pair = candidate_pairs[completed - 1];
            progress(ValidationProgress::PairCheck {
                current: completed,
                total: broad_phase_candidates,
                pair,
            });
        }

        let unrelated_proximity_checks =
            self.validate_unrelated_proximity(&instances, linear_epsilon, &mut issues);

        Ok(ValidationReport {
            profile: self.settings.profile,
            definitions,
            skipped_definitions,
            skipped_instances,
            total_instance_pairs,
            eligible_instance_pairs,
            broad_phase_candidates,
            unrelated_proximity_checks,
            skipped_relation_checks,
            relation_checks,
            pair_checks,
            issues,
        })
    }
}

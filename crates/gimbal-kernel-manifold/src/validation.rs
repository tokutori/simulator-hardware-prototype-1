// SPDX-License-Identifier: MIT

use gimbal_core::{
    Assembly, AssemblyPose, AssemblyRelation, AssemblyRelationId, ComponentDefinitionId,
    ComponentIdentity, ComponentInstanceId, ComponentInstancePair, DatumEndpoint, FeatureGraph,
    NumericalTolerance, PlaneDatum,
};
use manifold_rust::manifold::Manifold;
use manifold_rust::types::{BooleanEngine, Error as ManifoldStatus};
use rayon::prelude::*;
use std::panic::{AssertUnwindSafe, catch_unwind};
use thiserror::Error;

use crate::{Evaluator, KernelError, SolidMetrics, transform};

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
    SurfaceContactSeparation {
        distance_mm: f64,
        allowed_mm: f64,
    },
    SurfaceContactNormalMismatch {
        error_radians: f64,
        allowed_radians: f64,
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
    pub intersection_volume_mm3: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ValidationReport {
    pub definitions: Vec<DefinitionValidation>,
    pub total_instance_pairs: usize,
    pub broad_phase_candidates: usize,
    pub exact_pair_checks: Vec<PairValidation>,
    pub issues: Vec<ValidationIssue>,
}

impl ValidationReport {
    pub fn is_valid(&self) -> bool {
        !self
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ValidatorSettings {
    pub numerical_tolerance: NumericalTolerance,
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

#[derive(Debug, Error)]
pub enum ValidationError {
    #[error(transparent)]
    Kernel(#[from] KernelError),
    #[error("instance {0:?} references a frame that is absent from the assembly pose")]
    MissingFrame(ComponentInstanceId),
    #[error("geometry kernel returned {status:?} while checking instance pair {pair:?}")]
    PairIntersection {
        pair: ComponentInstancePair,
        status: ManifoldStatus,
    },
    #[error("geometry kernel panicked while checking instance pair {0:?}")]
    PairPanicked(ComponentInstancePair),
}

pub struct AssemblyValidator<'a> {
    graph: &'a FeatureGraph,
    assembly: &'a Assembly,
    pose: &'a AssemblyPose,
    settings: ValidatorSettings,
}

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
        let mut evaluator = Evaluator::new(self.graph);
        let definitions = self
            .assembly
            .definitions_with_ids()
            .map(|(definition_id, definition)| {
                Ok(DefinitionValidation {
                    definition: definition_id,
                    metrics: evaluator.metrics(definition.body.assembly_solid())?,
                })
            })
            .collect::<Result<Vec<_>, KernelError>>()?;

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

        let instances = self
            .assembly
            .instances_with_ids()
            .map(|(id, instance)| {
                let definition = self
                    .assembly
                    .definition(instance.definition)
                    .expect("inserted instance references a validated definition");
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
                Ok(InstanceGeometry {
                    solid,
                    bounds,
                    world_pose,
                })
            })
            .collect::<Result<Vec<_>, ValidationError>>()?;

        self.validate_surface_contacts(&instances, &mut issues);

        let linear_epsilon = self.settings.numerical_tolerance.linear_epsilon.as_mm();
        let volume_epsilon = self
            .settings
            .numerical_tolerance
            .volume_epsilon
            .as_cubic_mm();
        let total_instance_pairs = self.assembly.instance_pairs().count();
        let candidate_pairs = self
            .assembly
            .instance_pairs()
            .filter(|pair| {
                let first = &instances[pair.first.index()];
                let second = &instances[pair.second.index()];
                first
                    .bounds
                    .has_interior_overlap(second.bounds, linear_epsilon)
            })
            .collect::<Vec<_>>();
        let broad_phase_candidates = candidate_pairs.len();
        progress(ValidationProgress::BroadPhaseComplete {
            candidates: broad_phase_candidates,
        });
        let mut exact_pair_checks = Vec::new();
        const PAIR_CHUNK_SIZE: usize = 32;
        for (chunk_index, chunk) in candidate_pairs.chunks(PAIR_CHUNK_SIZE).enumerate() {
            let evaluated = chunk
                .par_iter()
                .map(|pair| {
                    let first = &instances[pair.first.index()];
                    let second = &instances[pair.second.index()];
                    if first.solid.min_gap(&second.solid, linear_epsilon) > 0.0 {
                        return Ok((*pair, None));
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
                    Ok((*pair, Some(intersection.volume())))
                })
                .collect::<Result<Vec<_>, ValidationError>>()?;
            for (pair, intersection_volume) in evaluated {
                let Some(intersection_volume_mm3) = intersection_volume else {
                    continue;
                };
                let relation_ids = self
                    .assembly
                    .relations_between(pair)
                    .map(|(id, _)| id)
                    .collect::<Vec<_>>();
                exact_pair_checks.push(PairValidation {
                    pair,
                    relation_ids,
                    intersection_volume_mm3,
                });
                if intersection_volume_mm3 > volume_epsilon {
                    issues.push(ValidationIssue {
                        severity: ValidationSeverity::Error,
                        pair: Some(pair),
                        relation: None,
                        kind: ValidationIssueKind::UnexpectedInterference {
                            intersection_volume_mm3,
                        },
                    });
                }
            }
            let completed = ((chunk_index + 1) * PAIR_CHUNK_SIZE).min(broad_phase_candidates);
            let pair = candidate_pairs[completed - 1];
            progress(ValidationProgress::PairCheck {
                current: completed,
                total: broad_phase_candidates,
                pair,
            });
        }

        Ok(ValidationReport {
            definitions,
            total_instance_pairs,
            broad_phase_candidates,
            exact_pair_checks,
            issues,
        })
    }

    fn validate_surface_contacts(
        &self,
        instances: &[InstanceGeometry],
        issues: &mut Vec<ValidationIssue>,
    ) {
        for (relation_id, relation) in self.assembly.relations_with_ids() {
            let relation = *relation;
            let AssemblyRelation::SurfaceContact(contact) = relation else {
                continue;
            };
            let pair = ComponentInstancePair {
                first: contact.first.instance,
                second: contact.second.instance,
            };
            let first = world_plane(contact.first, self.assembly, instances);
            let second = world_plane(contact.second, self.assembly, instances);
            let delta = subtract(second.origin, first.origin);
            let distance_mm = dot(first.normal, delta).abs();
            let allowed_mm = contact.tolerance.linear.as_mm();
            if distance_mm > allowed_mm {
                issues.push(ValidationIssue {
                    severity: ValidationSeverity::Error,
                    pair: Some(pair),
                    relation: Some(relation_id),
                    kind: ValidationIssueKind::SurfaceContactSeparation {
                        distance_mm,
                        allowed_mm,
                    },
                });
            }
            let opposed_dot = (-dot(first.normal, second.normal)).clamp(-1.0, 1.0);
            let error_radians = opposed_dot.acos();
            let allowed_radians = contact.tolerance.angular.as_radians();
            if error_radians > allowed_radians {
                issues.push(ValidationIssue {
                    severity: ValidationSeverity::Error,
                    pair: Some(pair),
                    relation: Some(relation_id),
                    kind: ValidationIssueKind::SurfaceContactNormalMismatch {
                        error_radians,
                        allowed_radians,
                    },
                });
            }
        }
    }
}

struct InstanceGeometry {
    solid: Manifold,
    bounds: Aabb3,
    world_pose: gimbal_core::RigidTransform,
}

#[derive(Clone, Copy)]
struct WorldPlane {
    origin: [f64; 3],
    normal: [f64; 3],
}

fn world_plane(
    endpoint: DatumEndpoint<PlaneDatum>,
    assembly: &Assembly,
    instances: &[InstanceGeometry],
) -> WorldPlane {
    let instance = assembly
        .instance(endpoint.instance)
        .expect("relation endpoint was validated when inserted");
    let definition = assembly
        .definition(instance.definition)
        .expect("inserted instance references a definition");
    let plane = definition
        .datums
        .get(endpoint.datum)
        .expect("relation datum kind was validated when inserted");
    let pose = instances[endpoint.instance.index()].world_pose;
    WorldPlane {
        origin: pose.transform_point(plane.origin.coordinates_mm()),
        normal: pose.transform_vector(plane.normal.components()),
    }
}

fn subtract(lhs: [f64; 3], rhs: [f64; 3]) -> [f64; 3] {
    [lhs[0] - rhs[0], lhs[1] - rhs[1], lhs[2] - rhs[2]]
}

fn dot(lhs: [f64; 3], rhs: [f64; 3]) -> f64 {
    lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2]
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Aabb3 {
    minimum: [f64; 3],
    maximum: [f64; 3],
}

impl Aabb3 {
    fn from_manifold(solid: &Manifold) -> Self {
        let bounds = solid.bounding_box();
        Self {
            minimum: [bounds.min.x, bounds.min.y, bounds.min.z],
            maximum: [bounds.max.x, bounds.max.y, bounds.max.z],
        }
    }

    fn has_interior_overlap(self, other: Self, epsilon: f64) -> bool {
        (0..3).all(|axis| {
            self.maximum[axis].min(other.maximum[axis])
                - self.minimum[axis].max(other.minimum[axis])
                > epsilon
        })
    }
}

#[cfg(test)]
mod tests {
    use gimbal_core::{
        Angle, AssemblyRelation, Body, ComponentDefinition, ComponentInstance, ComponentLocation,
        ComponentRole, DatumEndpoint, DatumSet, EngineeringTolerance, FeatureBuilder, FrameGraph,
        Kinematics, Manufacturing, NonNegativeAngle, NonNegativeLength, PitchRollCommand,
        PlaneDatum, Point3, PositiveLength, PositiveVolume, Primitive3, RigidTransform,
        SurfaceContact, UnitVector3,
    };

    use super::*;

    fn settings() -> ValidatorSettings {
        ValidatorSettings {
            numerical_tolerance: NumericalTolerance {
                linear_epsilon: PositiveLength::mm(1.0e-6).expect("positive epsilon"),
                volume_epsilon: PositiveVolume::cubic_mm(1.0e-9).expect("positive epsilon"),
            },
        }
    }

    #[test]
    fn overlapping_instances_fail_but_face_contact_does_not() {
        let mut builder = FeatureBuilder::new();
        let cube = builder.primitive(Primitive3::Box {
            x: gimbal_core::Length::positive_mm(10.0).expect("positive length"),
            y: gimbal_core::Length::positive_mm(10.0).expect("positive length"),
            z: gimbal_core::Length::positive_mm(10.0).expect("positive length"),
            centered: true,
        });
        let graph = builder.finish();
        let frames = FrameGraph::new();
        let world = frames.world();
        let kinematics = Kinematics::new(
            frames,
            Angle::degrees(1.0).expect("finite limit"),
            Angle::degrees(1.0).expect("finite limit"),
        );
        let pose = kinematics
            .pose(PitchRollCommand {
                pitch: Angle::degrees(0.0).expect("finite angle"),
                roll: Angle::degrees(0.0).expect("finite angle"),
            })
            .expect("zero pose");

        let report_for_offset = |offset: f64| {
            let mut assembly = Assembly::new();
            let definition = assembly.add_definition(ComponentDefinition {
                name: "cube".into(),
                role: ComponentRole::FixedCrossmember,
                body: Body::Solid(cube),
                manufacturing: Manufacturing::Purchased,
                color_rgba: [1.0; 4],
                datums: DatumSet::new(),
            });
            for (ordinal, x) in [(1, 0.0), (2, offset)] {
                assembly.add_instance(ComponentInstance {
                    name: format!("cube_{ordinal}"),
                    definition,
                    frame: world,
                    local_pose: RigidTransform::translated(x, 0.0, 0.0),
                    location: ComponentLocation::new().with_ordinal(ordinal),
                });
            }
            AssemblyValidator::new(&graph, &assembly, &pose, settings())
                .validate()
                .expect("validation query succeeds")
        };

        let overlap = report_for_offset(9.0);
        assert!(!overlap.is_valid());
        assert_eq!(overlap.error_count(), 1);
        assert!(matches!(
            overlap.issues[0].kind,
            ValidationIssueKind::UnexpectedInterference {
                intersection_volume_mm3
            } if (intersection_volume_mm3 - 100.0).abs() < 1.0e-8
        ));

        let face_contact = report_for_offset(10.0);
        assert!(face_contact.is_valid());
        assert_eq!(face_contact.broad_phase_candidates, 0);
    }

    #[test]
    fn surface_contact_validates_semantic_planes_with_engineering_tolerance() {
        let mut builder = FeatureBuilder::new();
        let cube = builder.primitive(Primitive3::Box {
            x: gimbal_core::Length::positive_mm(10.0).expect("positive length"),
            y: gimbal_core::Length::positive_mm(10.0).expect("positive length"),
            z: gimbal_core::Length::positive_mm(10.0).expect("positive length"),
            centered: true,
        });
        let graph = builder.finish();
        let frames = FrameGraph::new();
        let world = frames.world();
        let kinematics = Kinematics::new(
            frames,
            Angle::degrees(1.0).expect("finite limit"),
            Angle::degrees(1.0).expect("finite limit"),
        );
        let pose = kinematics
            .pose(PitchRollCommand {
                pitch: Angle::degrees(0.0).expect("finite angle"),
                roll: Angle::degrees(0.0).expect("finite angle"),
            })
            .expect("zero pose");

        let report_for_offset = |offset: f64| {
            let mut first_datums = DatumSet::new();
            let first_plane = first_datums.add(
                "contact_plane".into(),
                PlaneDatum {
                    origin: Point3::from_mm([5.0, 0.0, 0.0]).expect("finite point"),
                    normal: UnitVector3::new([1.0, 0.0, 0.0]).expect("valid normal"),
                },
            );
            let mut second_datums = DatumSet::new();
            let second_plane = second_datums.add(
                "contact_plane".into(),
                PlaneDatum {
                    origin: Point3::from_mm([-5.0, 0.0, 0.0]).expect("finite point"),
                    normal: UnitVector3::new([-1.0, 0.0, 0.0]).expect("valid normal"),
                },
            );
            let mut assembly = Assembly::new();
            let first_definition = assembly.add_definition(ComponentDefinition {
                name: "first_cube".into(),
                role: ComponentRole::FixedCrossmember,
                body: Body::Solid(cube),
                manufacturing: Manufacturing::Purchased,
                color_rgba: [1.0; 4],
                datums: first_datums,
            });
            let second_definition = assembly.add_definition(ComponentDefinition {
                name: "second_cube".into(),
                role: ComponentRole::FixedCarrierRail,
                body: Body::Solid(cube),
                manufacturing: Manufacturing::Purchased,
                color_rgba: [1.0; 4],
                datums: second_datums,
            });
            let first = assembly.add_instance(ComponentInstance {
                name: "first_cube".into(),
                definition: first_definition,
                frame: world,
                local_pose: RigidTransform::IDENTITY,
                location: ComponentLocation::new(),
            });
            let second = assembly.add_instance(ComponentInstance {
                name: "second_cube".into(),
                definition: second_definition,
                frame: world,
                local_pose: RigidTransform::translated(offset, 0.0, 0.0),
                location: ComponentLocation::new(),
            });
            assembly
                .add_relation(AssemblyRelation::SurfaceContact(SurfaceContact {
                    first: DatumEndpoint::new(first, first_plane),
                    second: DatumEndpoint::new(second, second_plane),
                    tolerance: EngineeringTolerance {
                        linear: NonNegativeLength::mm(0.01).expect("valid tolerance"),
                        angular: NonNegativeAngle::degrees(0.1).expect("valid tolerance"),
                    },
                }))
                .expect("valid contact relation");
            AssemblyValidator::new(&graph, &assembly, &pose, settings())
                .validate()
                .expect("validation query succeeds")
        };

        assert!(report_for_offset(10.0).is_valid());
        let separated = report_for_offset(10.1);
        assert!(!separated.is_valid());
        assert!(separated.issues.iter().any(|issue| matches!(
            issue.kind,
            ValidationIssueKind::SurfaceContactSeparation { distance_mm, .. }
                if (distance_mm - 0.1).abs() < 1.0e-8
        )));
    }
}

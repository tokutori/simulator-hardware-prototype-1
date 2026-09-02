// SPDX-License-Identifier: MIT

use gimbal_core::{
    Assembly, AssemblyPose, AssemblyRelation, AssemblyRelationId, ComponentDefinitionId,
    ComponentIdentity, ComponentInstanceId, ComponentInstancePair, ComponentRole, DatumEndpoint,
    FeatureGraph, NonNegativeLength, NumericalTolerance, PlaneDatum,
};
use manifold_rust::manifold::Manifold;
use manifold_rust::types::{BooleanEngine, Error as ManifoldStatus};
use rayon::prelude::*;
use std::panic::{AssertUnwindSafe, catch_unwind};
use thiserror::Error;

use crate::{Evaluator, GeometryEvaluationMode, KernelError, SolidMetrics, transform};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ValidationSeverity {
    Warning,
    Error,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UnrelatedProximityPolicy {
    Ignore,
    Warning,
    Error,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ValidationScope {
    StructuralFast,
    Full,
}

impl ValidationScope {
    const fn includes(self, role: ComponentRole) -> bool {
        match self {
            Self::Full => true,
            Self::StructuralFast => match role {
                ComponentRole::PitchSector
                | ComponentRole::PitchDrivePinion
                | ComponentRole::PitchRetentionPinion
                | ComponentRole::PitchGearboxSmallGear
                | ComponentRole::PitchGearboxDistributionGear
                | ComponentRole::PitchGearboxLargeGear
                | ComponentRole::RollDrivenGear
                | ComponentRole::RollInputPinion
                | ComponentRole::RollGearboxSmallGear
                | ComponentRole::RollGearboxLargeGear => false,
                ComponentRole::FixedCarrierRail
                | ComponentRole::FixedCarrierPost
                | ComponentRole::FixedCrossmember
                | ComponentRole::PitchCradleLongitudinalRail
                | ComponentRole::RollBearingCarrierEnd
                | ComponentRole::InstallationFloor
                | ComponentRole::PitchDriveFlange
                | ComponentRole::PitchRetentionFlange
                | ComponentRole::PitchDriveShaft
                | ComponentRole::PitchRetentionShaft
                | ComponentRole::PitchContactOutboardPlate
                | ComponentRole::PitchContactCarriagePlate
                | ComponentRole::PitchGearboxFarPlate
                | ComponentRole::PitchGearboxShaft
                | ComponentRole::PitchGearboxTieRod
                | ComponentRole::RetentionLeafSpring
                | ComponentRole::RetentionBearingBlock
                | ComponentRole::Cockpit
                | ComponentRole::CockpitHanger
                | ComponentRole::CockpitShaftKey
                | ComponentRole::RollShaft
                | ComponentRole::RollDrivenHub
                | ComponentRole::RollDrivenKey
                | ComponentRole::RollGearboxShaft
                | ComponentRole::RollBearing
                | ComponentRole::RollGearboxPlate
                | ComponentRole::RollGearboxMount
                | ComponentRole::MovingDriveMountArm => true,
            },
        }
    }
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

#[derive(Clone, Debug, PartialEq)]
pub struct ValidationReport {
    pub scope: ValidationScope,
    pub definitions: Vec<DefinitionValidation>,
    pub skipped_definitions: Vec<ComponentDefinitionId>,
    pub skipped_instances: Vec<ComponentInstanceId>,
    pub total_instance_pairs: usize,
    pub eligible_instance_pairs: usize,
    pub broad_phase_candidates: usize,
    pub unrelated_proximity_checks: usize,
    pub skipped_relation_checks: usize,
    pub pair_checks: Vec<PairValidation>,
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
    pub scope: ValidationScope,
    pub numerical_tolerance: NumericalTolerance,
    pub unrelated_proximity_threshold: NonNegativeLength,
    pub unrelated_proximity_policy: UnrelatedProximityPolicy,
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
        let evaluation_mode = match self.settings.scope {
            ValidationScope::StructuralFast => GeometryEvaluationMode::StructuralProxy,
            ValidationScope::Full => GeometryEvaluationMode::Robust,
        };
        let mut evaluator = Evaluator::with_mode(self.graph, evaluation_mode);
        let mut definitions = Vec::new();
        let mut skipped_definitions = Vec::new();
        for (definition_id, definition) in self.assembly.definitions_with_ids() {
            if self.settings.scope.includes(definition.role) {
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
                if !self.settings.scope.includes(definition.role) {
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

        let skipped_relation_checks = self.validate_surface_contacts(&instances, &mut issues);

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
            let evaluated = match self.settings.scope {
                ValidationScope::StructuralFast => chunk
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
                ValidationScope::Full => chunk
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
                let method = match self.settings.scope {
                    ValidationScope::StructuralFast => PairCheckMethod::StructuralProxyAabb,
                    ValidationScope::Full => PairCheckMethod::ExactSolid,
                };
                pair_checks.push(PairValidation {
                    pair,
                    relation_ids,
                    method,
                    intersection_volume_mm3,
                });
                let kind = match self.settings.scope {
                    ValidationScope::StructuralFast => {
                        ValidationIssueKind::PotentialStructuralInterference {
                            proxy_aabb_overlap_mm3: intersection_volume_mm3,
                        }
                    }
                    ValidationScope::Full => ValidationIssueKind::UnexpectedInterference {
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
            scope: self.settings.scope,
            definitions,
            skipped_definitions,
            skipped_instances,
            total_instance_pairs,
            eligible_instance_pairs,
            broad_phase_candidates,
            unrelated_proximity_checks,
            skipped_relation_checks,
            pair_checks,
            issues,
        })
    }

    fn validate_unrelated_proximity(
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
            let gap_mm = match self.settings.scope {
                ValidationScope::StructuralFast => first.bounds.minimum_gap(second.bounds),
                ValidationScope::Full => first.solid.min_gap(&second.solid, search_length),
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

    fn validate_surface_contacts(
        &self,
        instances: &[Option<InstanceGeometry>],
        issues: &mut Vec<ValidationIssue>,
    ) -> usize {
        let mut skipped = 0;
        for (relation_id, relation) in self.assembly.relations_with_ids() {
            let relation = *relation;
            let AssemblyRelation::SurfaceContact(contact) = relation else {
                continue;
            };
            let pair = ComponentInstancePair {
                first: contact.first.instance,
                second: contact.second.instance,
            };
            let (Some(first_geometry), Some(second_geometry)) = (
                instances[pair.first.index()].as_ref(),
                instances[pair.second.index()].as_ref(),
            ) else {
                skipped += 1;
                continue;
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
            let normals_match = error_radians <= allowed_radians;
            if !normals_match {
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
            if normals_match {
                let contact_area_mm2 = contact_area(
                    &first_geometry.solid,
                    &second_geometry.solid,
                    first,
                    second,
                    self.settings.numerical_tolerance.linear_epsilon.as_mm(),
                );
                let minimum_area_mm2 = contact.minimum_contact_area.as_square_mm();
                if contact_area_mm2
                    + self
                        .settings
                        .numerical_tolerance
                        .area_epsilon
                        .as_square_mm()
                    < minimum_area_mm2
                {
                    issues.push(ValidationIssue {
                        severity: ValidationSeverity::Error,
                        pair: Some(pair),
                        relation: Some(relation_id),
                        kind: ValidationIssueKind::SurfaceContactAreaInsufficient {
                            contact_area_mm2,
                            minimum_area_mm2,
                        },
                    });
                }
            }
        }
        skipped
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
    instances: &[Option<InstanceGeometry>],
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
    let pose = instances[endpoint.instance.index()]
        .as_ref()
        .expect("surface contact endpoint is included in this validation scope")
        .world_pose;
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

fn contact_area(
    first: &Manifold,
    second: &Manifold,
    first_plane: WorldPlane,
    second_plane: WorldPlane,
    probe_depth: f64,
) -> f64 {
    let to_plane = world_to_plane_transform(first_plane);
    let first = transform(first.clone(), to_plane);
    let second = transform(second.clone(), to_plane);
    let signed_separation = dot(
        first_plane.normal,
        subtract(second_plane.origin, first_plane.origin),
    );
    let first_section = first.slice(-probe_depth);
    let second_section = second.slice(signed_separation + probe_depth);
    first_section.intersection(&second_section).area()
}

fn world_to_plane_transform(plane: WorldPlane) -> gimbal_core::RigidTransform {
    let rotation = quaternion_between(plane.normal, [0.0, 0.0, 1.0]);
    let rotation_only = gimbal_core::RigidTransform {
        translation: [0.0; 3],
        rotation,
    };
    let rotated_origin = rotation_only.transform_point(plane.origin);
    gimbal_core::RigidTransform {
        translation: [-rotated_origin[0], -rotated_origin[1], -rotated_origin[2]],
        rotation,
    }
}

fn quaternion_between(from: [f64; 3], to: [f64; 3]) -> [f64; 4] {
    let direction_dot = dot(from, to).clamp(-1.0, 1.0);
    if direction_dot < -1.0 + 1.0e-12 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    let cross = [
        from[1] * to[2] - from[2] * to[1],
        from[2] * to[0] - from[0] * to[2],
        from[0] * to[1] - from[1] * to[0],
    ];
    let quaternion = [cross[0], cross[1], cross[2], 1.0 + direction_dot];
    let norm = (quaternion[0] * quaternion[0]
        + quaternion[1] * quaternion[1]
        + quaternion[2] * quaternion[2]
        + quaternion[3] * quaternion[3])
        .sqrt();
    [
        quaternion[0] / norm,
        quaternion[1] / norm,
        quaternion[2] / norm,
        quaternion[3] / norm,
    ]
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

    fn interior_overlap_volume(self, other: Self) -> f64 {
        (0..3)
            .map(|axis| {
                (self.maximum[axis].min(other.maximum[axis])
                    - self.minimum[axis].max(other.minimum[axis]))
                .max(0.0)
            })
            .product()
    }

    fn is_within_gap(self, other: Self, gap: f64) -> bool {
        (0..3).all(|axis| {
            self.minimum[axis] <= other.maximum[axis] + gap
                && other.minimum[axis] <= self.maximum[axis] + gap
        })
    }

    fn minimum_gap(self, other: Self) -> f64 {
        let squared = (0..3)
            .map(|axis| {
                let axis_gap = (self.minimum[axis] - other.maximum[axis])
                    .max(other.minimum[axis] - self.maximum[axis])
                    .max(0.0);
                axis_gap * axis_gap
            })
            .sum::<f64>();
        squared.sqrt()
    }
}

#[cfg(test)]
mod tests {
    use gimbal_core::{
        Angle, AssemblyRelation, Body, ComponentDefinition, ComponentInstance, ComponentLocation,
        ComponentRole, DatumEndpoint, DatumSet, EngineeringTolerance, FeatureBuilder, FrameGraph,
        Kinematics, Manufacturing, NonNegativeAngle, NonNegativeLength, PitchRollCommand,
        PlaneDatum, Point3, PositiveArea, PositiveLength, PositiveVolume, Primitive3,
        RigidTransform, SurfaceContact, UnitVector3,
    };

    use super::*;

    fn settings() -> ValidatorSettings {
        ValidatorSettings {
            scope: ValidationScope::Full,
            numerical_tolerance: NumericalTolerance {
                linear_epsilon: PositiveLength::mm(1.0e-6).expect("positive epsilon"),
                area_epsilon: PositiveArea::square_mm(1.0e-9).expect("positive epsilon"),
                volume_epsilon: PositiveVolume::cubic_mm(1.0e-9).expect("positive epsilon"),
            },
            unrelated_proximity_threshold: NonNegativeLength::mm(0.05)
                .expect("non-negative threshold"),
            unrelated_proximity_policy: UnrelatedProximityPolicy::Warning,
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
        assert_eq!(face_contact.warning_count(), 1);
        assert!(matches!(
            face_contact.issues[0].kind,
            ValidationIssueKind::UnspecifiedProximity { gap_mm, .. }
                if gap_mm.abs() < 1.0e-8
        ));

        let numerical_noise = report_for_offset(10.0 - 5.0e-7);
        assert!(numerical_noise.is_valid());
        assert_eq!(numerical_noise.broad_phase_candidates, 0);

        let engineering_interference = report_for_offset(10.0 - 2.0e-6);
        assert!(!engineering_interference.is_valid());
        assert!(engineering_interference.issues.iter().any(|issue| matches!(
            issue.kind,
            ValidationIssueKind::UnexpectedInterference {
                intersection_volume_mm3
            } if intersection_volume_mm3 > settings()
                .numerical_tolerance
                .volume_epsilon
                .as_cubic_mm()
        )));
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

        let report_for_offset = |offset: f64, minimum_contact_area_mm2: f64| {
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
                    minimum_contact_area: PositiveArea::square_mm(minimum_contact_area_mm2)
                        .expect("positive contact area"),
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

        assert!(report_for_offset(10.0, 90.0).is_valid());
        let insufficient_area = report_for_offset(10.0, 101.0);
        assert!(!insufficient_area.is_valid());
        assert!(insufficient_area.issues.iter().any(|issue| matches!(
            issue.kind,
            ValidationIssueKind::SurfaceContactAreaInsufficient {
                contact_area_mm2,
                minimum_area_mm2,
            } if (contact_area_mm2 - 100.0).abs() < 1.0e-8
                && (minimum_area_mm2 - 101.0).abs() < 1.0e-8
        )));

        let separated = report_for_offset(10.1, 90.0);
        assert!(!separated.is_valid());
        assert!(separated.issues.iter().any(|issue| matches!(
            issue.kind,
            ValidationIssueKind::SurfaceContactSeparation { distance_mm, .. }
                if (distance_mm - 0.1).abs() < 1.0e-8
        )));
    }

    #[test]
    fn structural_fast_skips_high_detail_gears_and_uses_proxy_checks() {
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
        let mut assembly = Assembly::new();
        let gear = assembly.add_definition(ComponentDefinition {
            name: "high_detail_gear".into(),
            role: ComponentRole::PitchDrivePinion,
            body: Body::Solid(cube),
            manufacturing: Manufacturing::Purchased,
            color_rgba: [1.0; 4],
            datums: DatumSet::new(),
        });
        let structure = assembly.add_definition(ComponentDefinition {
            name: "structure".into(),
            role: ComponentRole::FixedCrossmember,
            body: Body::Solid(cube),
            manufacturing: Manufacturing::Purchased,
            color_rgba: [1.0; 4],
            datums: DatumSet::new(),
        });
        assembly.add_instance(ComponentInstance {
            name: "gear".into(),
            definition: gear,
            frame: world,
            local_pose: RigidTransform::IDENTITY,
            location: ComponentLocation::new(),
        });
        for (ordinal, x) in [(1, 0.0), (2, 9.0)] {
            assembly.add_instance(ComponentInstance {
                name: format!("structure_{ordinal}"),
                definition: structure,
                frame: world,
                local_pose: RigidTransform::translated(x, 0.0, 0.0),
                location: ComponentLocation::new().with_ordinal(ordinal),
            });
        }
        let mut structural_settings = settings();
        structural_settings.scope = ValidationScope::StructuralFast;
        let report = AssemblyValidator::new(&graph, &assembly, &pose, structural_settings)
            .validate()
            .expect("structural validation succeeds");

        assert_eq!(report.skipped_definitions, vec![gear]);
        assert_eq!(report.skipped_instances.len(), 1);
        assert_eq!(report.eligible_instance_pairs, 1);
        assert_eq!(report.error_count(), 1);
        assert!(matches!(
            report.issues[0].kind,
            ValidationIssueKind::PotentialStructuralInterference {
                proxy_aabb_overlap_mm3
            } if (proxy_aabb_overlap_mm3 - 100.0).abs() < 1.0e-8
        ));
        assert_eq!(
            report.pair_checks[0].method,
            PairCheckMethod::StructuralProxyAabb
        );
    }
}

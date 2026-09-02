// SPDX-License-Identifier: MIT

use super::{
    AssemblyValidator, InstanceGeometry, RelationValidation, RelationValidationStatus,
    ValidationIssue, ValidationIssueKind, ValidationSeverity,
};
use crate::transform;
use gimbal_core::{
    Assembly, AssemblyRelation, AssemblyRelationId, AxisDatum, ComponentInstanceId,
    ComponentInstancePair, CylinderDatum, CylindricalFit, DatumEndpoint, FastenedJoint, PlaneDatum,
    SurfaceContact,
};
use manifold_rust::manifold::Manifold;

impl AssemblyValidator<'_> {
    pub(super) fn validate_relations(
        &self,
        instances: &[Option<InstanceGeometry>],
        issues: &mut Vec<ValidationIssue>,
    ) -> Vec<RelationValidation> {
        self.assembly
            .relations_with_ids()
            .map(|(relation_id, relation)| {
                let status = match *relation {
                    AssemblyRelation::SurfaceContact(contact) => {
                        self.validate_surface_contact(relation_id, contact, instances, issues)
                    }
                    AssemblyRelation::Fastened(joint) => {
                        self.validate_fastened_joint(relation_id, joint, instances, issues)
                    }
                    AssemblyRelation::CylindricalFit(fit) => {
                        self.validate_cylindrical_fit(relation_id, fit, instances, issues)
                    }
                    AssemblyRelation::GearMesh(mesh) => unsupported_relation_status(
                        mesh.first_axis.instance,
                        mesh.second_axis.instance,
                        instances,
                    ),
                };
                RelationValidation {
                    relation: relation_id,
                    status,
                }
            })
            .collect()
    }

    fn validate_cylindrical_fit(
        &self,
        relation_id: AssemblyRelationId,
        fit: CylindricalFit,
        instances: &[Option<InstanceGeometry>],
        issues: &mut Vec<ValidationIssue>,
    ) -> RelationValidationStatus {
        let initial_issue_count = issues.len();
        let pair = ComponentInstancePair {
            first: fit.shaft.instance,
            second: fit.bore.instance,
        };
        if instances[pair.first.index()].is_none() || instances[pair.second.index()].is_none() {
            return RelationValidationStatus::SkippedByScope;
        }

        let shaft = world_cylinder(fit.shaft, self.assembly, instances);
        let bore = world_cylinder(fit.bore, self.assembly, instances);
        let allowed_mm = fit.tolerance.linear.as_mm();
        let origin_distance_mm = magnitude(subtract(bore.origin, shaft.origin));
        if origin_distance_mm > allowed_mm {
            issues.push(ValidationIssue {
                severity: ValidationSeverity::Error,
                pair: Some(pair),
                relation: Some(relation_id),
                kind: ValidationIssueKind::CylindricalFitOriginSeparation {
                    distance_mm: origin_distance_mm,
                    allowed_mm,
                },
            });
        }

        let axis_error_radians = dot(shaft.direction, bore.direction)
            .abs()
            .clamp(-1.0, 1.0)
            .acos();
        let allowed_radians = fit.tolerance.angular.as_radians();
        if axis_error_radians > allowed_radians {
            issues.push(ValidationIssue {
                severity: ValidationSeverity::Error,
                pair: Some(pair),
                relation: Some(relation_id),
                kind: ValidationIssueKind::CylindricalFitAxisMismatch {
                    error_radians: axis_error_radians,
                    allowed_radians,
                },
            });
        }

        let actual_radial_clearance_mm = bore.radius_mm - shaft.radius_mm;
        let target_radial_clearance_mm = fit.target_radial_clearance.as_mm();
        if (actual_radial_clearance_mm - target_radial_clearance_mm).abs() > allowed_mm {
            issues.push(ValidationIssue {
                severity: ValidationSeverity::Error,
                pair: Some(pair),
                relation: Some(relation_id),
                kind: ValidationIssueKind::CylindricalFitClearanceMismatch {
                    actual_radial_clearance_mm,
                    target_radial_clearance_mm,
                    allowed_mm,
                },
            });
        }

        relation_status_after(initial_issue_count, issues)
    }

    fn validate_surface_contact(
        &self,
        relation_id: AssemblyRelationId,
        contact: SurfaceContact,
        instances: &[Option<InstanceGeometry>],
        issues: &mut Vec<ValidationIssue>,
    ) -> RelationValidationStatus {
        let initial_issue_count = issues.len();
        let pair = ComponentInstancePair {
            first: contact.first.instance,
            second: contact.second.instance,
        };
        let (Some(first_geometry), Some(second_geometry)) = (
            instances[pair.first.index()].as_ref(),
            instances[pair.second.index()].as_ref(),
        ) else {
            return RelationValidationStatus::SkippedByScope;
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
        relation_status_after(initial_issue_count, issues)
    }

    fn validate_fastened_joint(
        &self,
        relation_id: AssemblyRelationId,
        joint: FastenedJoint,
        instances: &[Option<InstanceGeometry>],
        issues: &mut Vec<ValidationIssue>,
    ) -> RelationValidationStatus {
        let initial_issue_count = issues.len();
        let pair = ComponentInstancePair {
            first: joint.first_hole.instance,
            second: joint.second_hole.instance,
        };
        if instances[pair.first.index()].is_none() || instances[pair.second.index()].is_none() {
            return RelationValidationStatus::SkippedByScope;
        }
        let first_hole = world_cylinder(joint.first_hole, self.assembly, instances);
        let second_hole = world_cylinder(joint.second_hole, self.assembly, instances);
        let axis_delta = subtract(second_hole.origin, first_hole.origin);
        let axis_distance_mm = magnitude(cross(axis_delta, first_hole.direction));
        let allowed_mm = joint.tolerance.linear.as_mm();
        if axis_distance_mm > allowed_mm {
            issues.push(ValidationIssue {
                severity: ValidationSeverity::Error,
                pair: Some(pair),
                relation: Some(relation_id),
                kind: ValidationIssueKind::FastenerHoleAxisSeparation {
                    distance_mm: axis_distance_mm,
                    allowed_mm,
                },
            });
        }
        let axis_error_radians = dot(first_hole.direction, second_hole.direction)
            .abs()
            .clamp(-1.0, 1.0)
            .acos();
        let allowed_radians = joint.tolerance.angular.as_radians();
        if axis_error_radians > allowed_radians {
            issues.push(ValidationIssue {
                severity: ValidationSeverity::Error,
                pair: Some(pair),
                relation: Some(relation_id),
                kind: ValidationIssueKind::FastenerHoleAxisMismatch {
                    error_radians: axis_error_radians,
                    allowed_radians,
                },
            });
        }
        let expected_radius_mm =
            joint.thread.nominal_diameter_mm() * 0.5 + joint.target_hole_radial_clearance.as_mm();
        if (first_hole.radius_mm - expected_radius_mm).abs() > allowed_mm
            || (second_hole.radius_mm - expected_radius_mm).abs() > allowed_mm
        {
            issues.push(ValidationIssue {
                severity: ValidationSeverity::Error,
                pair: Some(pair),
                relation: Some(relation_id),
                kind: ValidationIssueKind::FastenerHoleRadiusMismatch {
                    first_radius_mm: first_hole.radius_mm,
                    second_radius_mm: second_hole.radius_mm,
                    expected_radius_mm,
                    allowed_mm,
                },
            });
        }
        let first_seat = world_plane(joint.head_seat, self.assembly, instances);
        let second_seat = world_plane(joint.nut_seat, self.assembly, instances);
        let first_axis_error = dot(first_seat.normal, first_hole.direction)
            .abs()
            .clamp(-1.0, 1.0)
            .acos();
        let second_axis_error = dot(second_seat.normal, first_hole.direction)
            .abs()
            .clamp(-1.0, 1.0)
            .acos();
        let opposed_error = (-dot(first_seat.normal, second_seat.normal))
            .clamp(-1.0, 1.0)
            .acos();
        let seat_error_radians = first_axis_error.max(second_axis_error).max(opposed_error);
        if seat_error_radians > allowed_radians {
            issues.push(ValidationIssue {
                severity: ValidationSeverity::Error,
                pair: Some(pair),
                relation: Some(relation_id),
                kind: ValidationIssueKind::FastenerSeatNormalMismatch {
                    error_radians: seat_error_radians,
                    allowed_radians,
                },
            });
        }
        let actual_grip_mm = dot(
            subtract(second_seat.origin, first_seat.origin),
            first_hole.direction,
        )
        .abs();
        let expected_grip_mm = joint.grip_length.as_mm();
        if (actual_grip_mm - expected_grip_mm).abs() > allowed_mm {
            issues.push(ValidationIssue {
                severity: ValidationSeverity::Error,
                pair: Some(pair),
                relation: Some(relation_id),
                kind: ValidationIssueKind::FastenerGripLengthMismatch {
                    actual_mm: actual_grip_mm,
                    expected_mm: expected_grip_mm,
                    allowed_mm,
                },
            });
        }

        let bolt = joint.hardware.bolt;
        let nut = joint.hardware.nut;
        let bolt_axis = world_axis(
            DatumEndpoint::new(bolt.instance, bolt.axis),
            self.assembly,
            instances,
        );
        let nut_axis = world_axis(
            DatumEndpoint::new(nut.instance, nut.axis),
            self.assembly,
            instances,
        );
        let mut validate_hardware_axis = |hardware: ComponentInstanceId, axis: WorldAxis| {
            let delta = subtract(axis.origin, first_hole.origin);
            let distance_mm = magnitude(cross(delta, first_hole.direction));
            if distance_mm > allowed_mm {
                issues.push(ValidationIssue {
                    severity: ValidationSeverity::Error,
                    pair: Some(pair),
                    relation: Some(relation_id),
                    kind: ValidationIssueKind::FastenerHardwareAxisSeparation {
                        hardware,
                        distance_mm,
                        allowed_mm,
                    },
                });
            }
            let error_radians = dot(axis.direction, first_hole.direction)
                .abs()
                .clamp(-1.0, 1.0)
                .acos();
            if error_radians > allowed_radians {
                issues.push(ValidationIssue {
                    severity: ValidationSeverity::Error,
                    pair: Some(pair),
                    relation: Some(relation_id),
                    kind: ValidationIssueKind::FastenerHardwareAxisMismatch {
                        hardware,
                        error_radians,
                        allowed_radians,
                    },
                });
            }
        };
        validate_hardware_axis(bolt.instance, bolt_axis);
        validate_hardware_axis(nut.instance, nut_axis);
        for washer in [joint.hardware.first_washer, joint.hardware.second_washer]
            .into_iter()
            .flatten()
        {
            validate_hardware_axis(
                washer.instance,
                world_axis(
                    DatumEndpoint::new(washer.instance, washer.axis),
                    self.assembly,
                    instances,
                ),
            );
        }

        let bolt_under_head = world_plane(
            DatumEndpoint::new(bolt.instance, bolt.under_head_face),
            self.assembly,
            instances,
        );
        let bolt_tip = world_plane(
            DatumEndpoint::new(bolt.instance, bolt.shank_tip_face),
            self.assembly,
            instances,
        );
        let nut_bearing = world_plane(
            DatumEndpoint::new(nut.instance, nut.bearing_face),
            self.assembly,
            instances,
        );
        let nut_outer = world_plane(
            DatumEndpoint::new(nut.instance, nut.outer_face),
            self.assembly,
            instances,
        );
        let mut validate_hardware_contact =
            |first_instance: ComponentInstanceId,
             first: WorldPlane,
             second_instance: ComponentInstanceId,
             second: WorldPlane| {
                let separation_mm = magnitude(subtract(second.origin, first.origin));
                let normal_error_radians =
                    (-dot(first.normal, second.normal)).clamp(-1.0, 1.0).acos();
                if separation_mm > allowed_mm || normal_error_radians > allowed_radians {
                    issues.push(ValidationIssue {
                        severity: ValidationSeverity::Error,
                        pair: Some(pair),
                        relation: Some(relation_id),
                        kind: ValidationIssueKind::FastenerHardwareContactMismatch {
                            first: first_instance,
                            second: second_instance,
                            separation_mm,
                            normal_error_radians,
                            allowed_mm,
                            allowed_radians,
                        },
                    });
                }
            };

        if let Some(washer) = joint.hardware.first_washer {
            let member_face = world_plane(
                DatumEndpoint::new(washer.instance, washer.member_face),
                self.assembly,
                instances,
            );
            let hardware_face = world_plane(
                DatumEndpoint::new(washer.instance, washer.hardware_face),
                self.assembly,
                instances,
            );
            validate_hardware_contact(
                joint.head_seat.instance,
                first_seat,
                washer.instance,
                member_face,
            );
            validate_hardware_contact(
                washer.instance,
                hardware_face,
                bolt.instance,
                bolt_under_head,
            );
        } else {
            validate_hardware_contact(
                joint.head_seat.instance,
                first_seat,
                bolt.instance,
                bolt_under_head,
            );
        }
        if let Some(washer) = joint.hardware.second_washer {
            let member_face = world_plane(
                DatumEndpoint::new(washer.instance, washer.member_face),
                self.assembly,
                instances,
            );
            let hardware_face = world_plane(
                DatumEndpoint::new(washer.instance, washer.hardware_face),
                self.assembly,
                instances,
            );
            validate_hardware_contact(
                joint.nut_seat.instance,
                second_seat,
                washer.instance,
                member_face,
            );
            validate_hardware_contact(washer.instance, hardware_face, nut.instance, nut_bearing);
        } else {
            validate_hardware_contact(
                joint.nut_seat.instance,
                second_seat,
                nut.instance,
                nut_bearing,
            );
        }

        let seat_delta = subtract(second_seat.origin, first_seat.origin);
        let travel_sign = if dot(seat_delta, first_hole.direction) >= 0.0 {
            1.0
        } else {
            -1.0
        };
        let travel_direction = [
            first_hole.direction[0] * travel_sign,
            first_hole.direction[1] * travel_sign,
            first_hole.direction[2] * travel_sign,
        ];
        let thread_engagement_mm = dot(
            subtract(nut_outer.origin, nut_bearing.origin),
            travel_direction,
        )
        .abs();
        let minimum_engagement_mm = joint.thread.minimum_full_thread_engagement_mm();
        if thread_engagement_mm + allowed_mm < minimum_engagement_mm {
            issues.push(ValidationIssue {
                severity: ValidationSeverity::Error,
                pair: Some(pair),
                relation: Some(relation_id),
                kind: ValidationIssueKind::FastenerThreadEngagementInsufficient {
                    actual_mm: thread_engagement_mm,
                    minimum_mm: minimum_engagement_mm,
                },
            });
        }
        let bolt_reach_mm = dot(
            subtract(bolt_tip.origin, bolt_under_head.origin),
            travel_direction,
        );
        let nut_outer_distance_mm = dot(
            subtract(nut_outer.origin, bolt_under_head.origin),
            travel_direction,
        );
        let protrusion_mm = bolt_reach_mm - nut_outer_distance_mm;
        let minimum_protrusion_mm = joint.thread.nominal_pitch_mm();
        if protrusion_mm + allowed_mm < minimum_protrusion_mm {
            issues.push(ValidationIssue {
                severity: ValidationSeverity::Error,
                pair: Some(pair),
                relation: Some(relation_id),
                kind: ValidationIssueKind::FastenerBoltProtrusionInsufficient {
                    actual_mm: protrusion_mm,
                    minimum_mm: minimum_protrusion_mm,
                },
            });
        }
        relation_status_after(initial_issue_count, issues)
    }
}

fn unsupported_relation_status(
    first: ComponentInstanceId,
    second: ComponentInstanceId,
    instances: &[Option<InstanceGeometry>],
) -> RelationValidationStatus {
    if instances[first.index()].is_none() || instances[second.index()].is_none() {
        RelationValidationStatus::SkippedByScope
    } else {
        RelationValidationStatus::Unsupported
    }
}

fn relation_status_after(
    initial_issue_count: usize,
    issues: &[ValidationIssue],
) -> RelationValidationStatus {
    if issues[initial_issue_count..]
        .iter()
        .any(|issue| issue.severity == ValidationSeverity::Error)
    {
        RelationValidationStatus::Failed
    } else {
        RelationValidationStatus::Validated
    }
}

#[derive(Clone, Copy)]
struct WorldPlane {
    origin: [f64; 3],
    normal: [f64; 3],
}

#[derive(Clone, Copy)]
struct WorldCylinder {
    origin: [f64; 3],
    direction: [f64; 3],
    radius_mm: f64,
}

#[derive(Clone, Copy)]
struct WorldAxis {
    origin: [f64; 3],
    direction: [f64; 3],
}

fn world_axis(
    endpoint: DatumEndpoint<AxisDatum>,
    assembly: &Assembly,
    instances: &[Option<InstanceGeometry>],
) -> WorldAxis {
    let instance = assembly
        .instance(endpoint.instance)
        .expect("relation endpoint was validated when inserted");
    let definition = assembly
        .definition(instance.definition)
        .expect("inserted instance references a definition");
    let axis = definition
        .datums
        .get(endpoint.datum)
        .expect("relation datum kind was validated when inserted");
    let pose = instances[endpoint.instance.index()]
        .as_ref()
        .expect("fastener hardware endpoint is included in this validation scope")
        .world_pose;
    WorldAxis {
        origin: pose.transform_point(axis.origin.coordinates_mm()),
        direction: pose.transform_vector(axis.direction.components()),
    }
}

fn world_cylinder(
    endpoint: DatumEndpoint<CylinderDatum>,
    assembly: &Assembly,
    instances: &[Option<InstanceGeometry>],
) -> WorldCylinder {
    let instance = assembly
        .instance(endpoint.instance)
        .expect("relation endpoint was validated when inserted");
    let definition = assembly
        .definition(instance.definition)
        .expect("inserted instance references a definition");
    let cylinder = definition
        .datums
        .get(endpoint.datum)
        .expect("relation datum kind was validated when inserted");
    let pose = instances[endpoint.instance.index()]
        .as_ref()
        .expect("fastened endpoint is included in this validation scope")
        .world_pose;
    WorldCylinder {
        origin: pose.transform_point(cylinder.axis.origin.coordinates_mm()),
        direction: pose.transform_vector(cylinder.axis.direction.components()),
        radius_mm: cylinder.radius.as_mm(),
    }
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

fn cross(lhs: [f64; 3], rhs: [f64; 3]) -> [f64; 3] {
    [
        lhs[1] * rhs[2] - lhs[2] * rhs[1],
        lhs[2] * rhs[0] - lhs[0] * rhs[2],
        lhs[0] * rhs[1] - lhs[1] * rhs[0],
    ]
}

fn magnitude(vector: [f64; 3]) -> f64 {
    dot(vector, vector).sqrt()
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

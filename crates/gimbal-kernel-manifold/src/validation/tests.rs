use gimbal_core::{
    Angle, AssemblyRelation, AxisDatum, Body, BoltHardware, ComponentDefinition, ComponentInstance,
    ComponentLocation, ComponentRole, CylinderDatum, CylindricalFit, DatumEndpoint, DatumSet,
    EngineeringTolerance, FastenedJoint, FastenerHardware, FeatureBuilder, FrameGraph, Kinematics,
    Manufacturing, MetricThread, NonNegativeAngle, NonNegativeLength, NumericalTolerance,
    NutHardware, PitchRollCommand, PlaneDatum, Point3, PositiveArea, PositiveLength,
    PositiveVolume, Primitive3, RigidTransform, SurfaceContact, UnitVector3,
};

use super::*;

fn settings() -> ValidatorSettings {
    ValidatorSettings {
        profile: ValidationProfile::EXACT_STATIC,
        numerical_tolerance: NumericalTolerance {
            linear_epsilon: PositiveLength::mm(1.0e-6).expect("positive epsilon"),
            area_epsilon: PositiveArea::square_mm(1.0e-9).expect("positive epsilon"),
            volume_epsilon: PositiveVolume::cubic_mm(1.0e-9).expect("positive epsilon"),
        },
        unrelated_proximity_threshold: NonNegativeLength::mm(0.05).expect("non-negative threshold"),
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
        let mut assembly = Assembly::new();
        let mut first_datums = DatumSet::for_definition(assembly.next_definition_id());
        let first_plane = first_datums.add(
            "contact_plane".into(),
            PlaneDatum {
                origin: Point3::from_mm([5.0, 0.0, 0.0]).expect("finite point"),
                normal: UnitVector3::new([1.0, 0.0, 0.0]).expect("valid normal"),
            },
        );
        let first_definition = assembly.add_definition(ComponentDefinition {
            name: "first_cube".into(),
            role: ComponentRole::FixedCrossmember,
            body: Body::Solid(cube),
            manufacturing: Manufacturing::Purchased,
            color_rgba: [1.0; 4],
            datums: first_datums,
        });
        let mut second_datums = DatumSet::for_definition(assembly.next_definition_id());
        let second_plane = second_datums.add(
            "contact_plane".into(),
            PlaneDatum {
                origin: Point3::from_mm([-5.0, 0.0, 0.0]).expect("finite point"),
                normal: UnitVector3::new([-1.0, 0.0, 0.0]).expect("valid normal"),
            },
        );
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
fn unsupported_relation_prevents_a_complete_or_valid_report() {
    let mut builder = FeatureBuilder::new();
    let solid = builder.primitive(Primitive3::Box {
        x: gimbal_core::Length::positive_mm(2.0).expect("positive length"),
        y: gimbal_core::Length::positive_mm(2.0).expect("positive length"),
        z: gimbal_core::Length::positive_mm(2.0).expect("positive length"),
        centered: true,
    });
    let graph = builder.finish();
    let frames = FrameGraph::new();
    let world = frames.world();
    let pose = Kinematics::new(
        frames,
        Angle::degrees(1.0).expect("finite limit"),
        Angle::degrees(1.0).expect("finite limit"),
    )
    .pose(PitchRollCommand {
        pitch: Angle::degrees(0.0).expect("finite angle"),
        roll: Angle::degrees(0.0).expect("finite angle"),
    })
    .expect("zero pose");
    let mut assembly = Assembly::new();
    let add_definition = |assembly: &mut Assembly, name: &str, radius: f64| {
        let mut datums = DatumSet::for_definition(assembly.next_definition_id());
        let cylinder = datums.add(
            format!("{name}_cylinder"),
            CylinderDatum {
                axis: AxisDatum {
                    origin: Point3::from_mm([0.0; 3]).expect("finite point"),
                    direction: UnitVector3::new([0.0, 0.0, 1.0]).expect("valid direction"),
                },
                radius: PositiveLength::mm(radius).expect("positive radius"),
            },
        );
        let definition = assembly.add_definition(ComponentDefinition {
            name: name.into(),
            role: ComponentRole::RollShaft,
            body: Body::Solid(solid),
            manufacturing: Manufacturing::Purchased,
            color_rgba: [1.0; 4],
            datums,
        });
        (definition, cylinder)
    };
    let (shaft_definition, shaft_datum) = add_definition(&mut assembly, "shaft", 1.0);
    let (bore_definition, bore_datum) = add_definition(&mut assembly, "bore", 1.1);
    let shaft = assembly.add_instance(ComponentInstance {
        name: "shaft".into(),
        definition: shaft_definition,
        frame: world,
        local_pose: RigidTransform::translated(-5.0, 0.0, 0.0),
        location: ComponentLocation::new(),
    });
    let bore = assembly.add_instance(ComponentInstance {
        name: "bore".into(),
        definition: bore_definition,
        frame: world,
        local_pose: RigidTransform::translated(5.0, 0.0, 0.0),
        location: ComponentLocation::new().with_ordinal(1),
    });
    assembly
        .add_relation(AssemblyRelation::CylindricalFit(CylindricalFit {
            shaft: DatumEndpoint::new(shaft, shaft_datum),
            bore: DatumEndpoint::new(bore, bore_datum),
            target_radial_clearance: NonNegativeLength::mm(0.1).expect("non-negative clearance"),
            tolerance: EngineeringTolerance {
                linear: NonNegativeLength::mm(0.05).expect("valid tolerance"),
                angular: NonNegativeAngle::degrees(0.1).expect("valid tolerance"),
            },
        }))
        .expect("valid relation structure");

    let report = AssemblyValidator::new(&graph, &assembly, &pose, settings())
        .validate()
        .expect("validation query succeeds");
    assert_eq!(report.relation_checks.len(), 1);
    assert_eq!(report.relation_checks[0].relation.index(), 0);
    assert_eq!(
        report.relation_checks[0].status,
        RelationValidationStatus::Unsupported
    );
    assert!(!report.is_complete());
    assert!(!report.is_valid());
}

#[test]
fn fastened_relation_validates_hole_axes_radii_seats_and_grip() {
    let mut builder = FeatureBuilder::new();
    let member = builder.primitive(Primitive3::Box {
        x: gimbal_core::Length::positive_mm(10.0).expect("positive length"),
        y: gimbal_core::Length::positive_mm(10.0).expect("positive length"),
        z: gimbal_core::Length::positive_mm(2.0).expect("positive length"),
        centered: true,
    });
    let hardware = builder.primitive(Primitive3::Box {
        x: gimbal_core::Length::positive_mm(1.0).expect("positive length"),
        y: gimbal_core::Length::positive_mm(1.0).expect("positive length"),
        z: gimbal_core::Length::positive_mm(1.0).expect("positive length"),
        centered: true,
    });
    let hardware = builder
        .translate(
            hardware,
            gimbal_core::Translation3 {
                x: 20.0,
                y: 0.0,
                z: 0.0,
            },
        )
        .expect("hardware fixture translation");
    let graph = builder.finish();
    let frames = FrameGraph::new();
    let world = frames.world();
    let pose = Kinematics::new(
        frames,
        Angle::degrees(1.0).expect("finite limit"),
        Angle::degrees(1.0).expect("finite limit"),
    )
    .pose(PitchRollCommand {
        pitch: Angle::degrees(0.0).expect("finite angle"),
        roll: Angle::degrees(0.0).expect("finite angle"),
    })
    .expect("zero pose");

    let report_for_offset =
        |second_x: f64, second_radius: f64, hardware_x: f64, bolt_tip_z: f64| {
            let member_datums = |owner, seat_z: f64, seat_normal: [f64; 3], radius: f64| {
                let mut datums = DatumSet::for_definition(owner);
                let hole = datums.add(
                    "m3_clearance_hole".into(),
                    CylinderDatum {
                        axis: AxisDatum {
                            origin: Point3::from_mm([0.0, 0.0, 0.0]).expect("finite point"),
                            direction: UnitVector3::new([0.0, 0.0, 1.0]).expect("valid direction"),
                        },
                        radius: PositiveLength::mm(radius).expect("positive radius"),
                    },
                );
                let seat = datums.add(
                    "washer_seat".into(),
                    PlaneDatum {
                        origin: Point3::from_mm([0.0, 0.0, seat_z]).expect("finite point"),
                        normal: UnitVector3::new(seat_normal).expect("valid normal"),
                    },
                );
                (datums, hole, seat)
            };
            let mut assembly = Assembly::new();
            let (first_datums, first_hole, first_seat) =
                member_datums(assembly.next_definition_id(), -1.0, [0.0, 0.0, -1.0], 1.7);
            let first_definition = assembly.add_definition(ComponentDefinition {
                name: "first_member".into(),
                role: ComponentRole::FixedCarrierPost,
                body: Body::Solid(member),
                manufacturing: Manufacturing::Fdm,
                color_rgba: [1.0; 4],
                datums: first_datums,
            });
            let (second_datums, second_hole, second_seat) = member_datums(
                assembly.next_definition_id(),
                1.0,
                [0.0, 0.0, 1.0],
                second_radius,
            );
            let second_definition = assembly.add_definition(ComponentDefinition {
                name: "second_member".into(),
                role: ComponentRole::FixedCarrierRail,
                body: Body::Solid(member),
                manufacturing: Manufacturing::Fdm,
                color_rgba: [1.0; 4],
                datums: second_datums,
            });
            let first = assembly.add_instance(ComponentInstance {
                name: "first_member".into(),
                definition: first_definition,
                frame: world,
                local_pose: RigidTransform::translated(0.0, 0.0, -1.0),
                location: ComponentLocation::new(),
            });
            let second = assembly.add_instance(ComponentInstance {
                name: "second_member".into(),
                definition: second_definition,
                frame: world,
                local_pose: RigidTransform::translated(second_x, 0.0, 1.0),
                location: ComponentLocation::new(),
            });
            let add_hardware = |assembly: &mut Assembly,
                                name: &str,
                                role,
                                z: f64,
                                first_face_z: f64,
                                first_face_normal: [f64; 3],
                                second_face_z: f64,
                                second_face_normal: [f64; 3]| {
                let owner = assembly.next_definition_id();
                let mut datums = DatumSet::for_definition(owner);
                let axis = datums.add(
                    format!("{name}_axis"),
                    AxisDatum {
                        origin: Point3::from_mm([0.0; 3]).expect("finite point"),
                        direction: UnitVector3::new([0.0, 0.0, 1.0]).expect("valid direction"),
                    },
                );
                let first_face = datums.add(
                    format!("{name}_first_face"),
                    PlaneDatum {
                        origin: Point3::from_mm([0.0, 0.0, first_face_z]).expect("finite point"),
                        normal: UnitVector3::new(first_face_normal).expect("valid normal"),
                    },
                );
                let second_face = datums.add(
                    format!("{name}_second_face"),
                    PlaneDatum {
                        origin: Point3::from_mm([0.0, 0.0, second_face_z]).expect("finite point"),
                        normal: UnitVector3::new(second_face_normal).expect("valid normal"),
                    },
                );
                let definition = assembly.add_definition(ComponentDefinition {
                    name: name.into(),
                    role,
                    body: Body::Solid(hardware),
                    manufacturing: Manufacturing::Purchased,
                    color_rgba: [1.0; 4],
                    datums,
                });
                let instance = assembly.add_instance(ComponentInstance {
                    name: name.into(),
                    definition,
                    frame: world,
                    local_pose: RigidTransform::translated(hardware_x, 0.0, z),
                    location: ComponentLocation::new(),
                });
                (instance, axis, first_face, second_face)
            };
            let (bolt, bolt_axis, bolt_under_head, bolt_tip) = add_hardware(
                &mut assembly,
                "m3_bolt",
                ComponentRole::M3Bolt,
                -2.0,
                0.0,
                [0.0, 0.0, 1.0],
                bolt_tip_z,
                [0.0, 0.0, 1.0],
            );
            let (nut, nut_axis, nut_bearing, nut_outer) = add_hardware(
                &mut assembly,
                "m3_nut",
                ComponentRole::M3Nut,
                3.2,
                -1.2,
                [0.0, 0.0, -1.0],
                1.2,
                [0.0, 0.0, 1.0],
            );
            assembly
                .add_relation(AssemblyRelation::Fastened(FastenedJoint {
                    first_hole: DatumEndpoint::new(first, first_hole),
                    second_hole: DatumEndpoint::new(second, second_hole),
                    head_seat: DatumEndpoint::new(first, first_seat),
                    nut_seat: DatumEndpoint::new(second, second_seat),
                    hardware: FastenerHardware {
                        bolt: BoltHardware {
                            instance: bolt,
                            axis: bolt_axis,
                            under_head_face: bolt_under_head,
                            shank_tip_face: bolt_tip,
                        },
                        nut: NutHardware {
                            instance: nut,
                            axis: nut_axis,
                            bearing_face: nut_bearing,
                            outer_face: nut_outer,
                        },
                        first_washer: None,
                        second_washer: None,
                    },
                    thread: MetricThread::M3,
                    target_hole_radial_clearance: NonNegativeLength::mm(0.2)
                        .expect("non-negative clearance"),
                    grip_length: PositiveLength::mm(4.0).expect("positive grip"),
                    tolerance: EngineeringTolerance {
                        linear: NonNegativeLength::mm(0.05).expect("valid tolerance"),
                        angular: NonNegativeAngle::degrees(0.1).expect("valid tolerance"),
                    },
                }))
                .expect("valid fastened relation");
            AssemblyValidator::new(&graph, &assembly, &pose, settings())
                .validate()
                .expect("validation query succeeds")
        };

    let valid = report_for_offset(0.0, 1.7, 0.0, 8.0);
    assert!(valid.is_valid(), "{valid:#?}");
    assert_eq!(
        valid.relation_checks[0].status,
        RelationValidationStatus::Validated
    );
    let axis_error = report_for_offset(0.2, 1.7, 0.0, 8.0);
    assert_eq!(
        axis_error.relation_checks[0].status,
        RelationValidationStatus::Failed
    );
    assert!(axis_error.issues.iter().any(|issue| matches!(
        issue.kind,
        ValidationIssueKind::FastenerHoleAxisSeparation { distance_mm, .. }
            if (distance_mm - 0.2).abs() < 1.0e-8
    )));
    let radius_error = report_for_offset(0.0, 1.9, 0.0, 8.0);
    assert!(radius_error.issues.iter().any(|issue| matches!(
        issue.kind,
        ValidationIssueKind::FastenerHoleRadiusMismatch {
            second_radius_mm,
            expected_radius_mm,
            ..
        } if (second_radius_mm - 1.9).abs() < 1.0e-8
            && (expected_radius_mm - 1.7).abs() < 1.0e-8
    )));
    let hardware_axis_error = report_for_offset(0.0, 1.7, 0.2, 8.0);
    assert!(hardware_axis_error.issues.iter().any(|issue| matches!(
        issue.kind,
        ValidationIssueKind::FastenerHardwareAxisSeparation { distance_mm, .. }
            if (distance_mm - 0.2).abs() < 1.0e-8
    )));
    let short_bolt = report_for_offset(0.0, 1.7, 0.0, 6.0);
    assert!(short_bolt.issues.iter().any(|issue| matches!(
        issue.kind,
        ValidationIssueKind::FastenerBoltProtrusionInsufficient { actual_mm, .. }
            if (actual_mm + 0.4).abs() < 1.0e-8
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
    structural_settings.profile = ValidationProfile::STRUCTURAL_STATIC;
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

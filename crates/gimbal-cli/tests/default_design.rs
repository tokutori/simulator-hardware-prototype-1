use gimbal_cli::{config, validate_assembly};
use gimbal_core::*;
use gimbal_kernel_manifold::{
    Evaluator, GeometryEvaluationMode, ValidationIssueKind, ValidationProfile,
};
use std::collections::HashMap;
use std::path::Path;

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

fn selected_instance(design: &PrototypeDesign, identity: ComponentIdentity) -> &ComponentInstance {
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

fn instance_solid(design: &PrototypeDesign, identity: ComponentIdentity) -> gimbal_core::SolidId {
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

    let report = validate_assembly(&design, ValidationProfile::STRUCTURAL_STATIC)
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

    let report = validate_assembly(&design, ValidationProfile::STRUCTURAL_STATIC)
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
fn retention_flexures_are_integrated_into_the_moving_support_plates() {
    let design = load_design();
    assert!(
        design.assembly.instances().iter().all(|instance| {
            !instance.name.contains("bearing_block") && !instance.name.contains("leaf_spring")
        }),
        "obsolete rigid bearing blocks and decorative leaf springs must remain absent"
    );
    for component in [
        located(
            ComponentRole::PitchContactOutboardPlate,
            ComponentLocation::new()
                .with_side(Side::Left)
                .with_longitudinal_end(LongitudinalEnd::Front),
        ),
        located(
            ComponentRole::PitchContactCarriagePlate,
            ComponentLocation::new()
                .with_side(Side::Right)
                .with_longitudinal_end(LongitudinalEnd::Rear),
        ),
    ] {
        let zero = instance_pose(&design, component, 0.0, 0.0);
        let pitched = instance_pose(&design, component, 20.0, 0.0);
        assert_ne!(
            zero, pitched,
            "the integrated retention flexure support {component:?} must travel with the pitch unit"
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
                located(ComponentRole::PitchDriveFlange, right_front.with_ordinal(1)),
                located(ComponentRole::PitchDriveFlange, right_front.with_ordinal(2)),
                located(ComponentRole::PitchDriveFlange, right_front.with_ordinal(3)),
                located(ComponentRole::PitchDriveFlange, right_front.with_ordinal(4)),
                located(
                    ComponentRole::PitchRetentionFlange,
                    right_front.with_ordinal(1),
                ),
                located(
                    ComponentRole::PitchRetentionFlange,
                    right_front.with_ordinal(2),
                ),
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

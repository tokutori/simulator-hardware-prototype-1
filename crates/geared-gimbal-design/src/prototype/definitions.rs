// SPDX-License-Identifier: MIT

use super::*;

#[derive(Clone, Copy)]
// A complete semantic face set keeps datum indices stable as additional
// fastened joints are introduced in Phase 4.
#[allow(dead_code)]
pub(super) struct BoxPlaneDatums {
    pub(super) negative_x: DatumId<PlaneDatum>,
    pub(super) positive_x: DatumId<PlaneDatum>,
    pub(super) negative_y: DatumId<PlaneDatum>,
    pub(super) positive_y: DatumId<PlaneDatum>,
    pub(super) negative_z: DatumId<PlaneDatum>,
    pub(super) positive_z: DatumId<PlaneDatum>,
}

#[derive(Clone, Copy)]
pub(super) struct FastenerMemberDatums {
    pub(super) hole: DatumId<CylinderDatum>,
    pub(super) negative_y_seat: DatumId<PlaneDatum>,
    pub(super) positive_y_seat: DatumId<PlaneDatum>,
}

#[derive(Clone, Copy)]
pub(super) struct PostFastenerDatums {
    pub(super) hole: DatumId<CylinderDatum>,
}

#[derive(Clone, Copy)]
pub(super) struct M3BoltDefinition {
    pub(super) definition: ComponentDefinitionId,
    pub(super) axis: DatumId<AxisDatum>,
    pub(super) under_head_face: DatumId<PlaneDatum>,
    pub(super) shank_tip_face: DatumId<PlaneDatum>,
}

#[derive(Clone, Copy)]
pub(super) struct M3NutDefinition {
    pub(super) definition: ComponentDefinitionId,
    pub(super) axis: DatumId<AxisDatum>,
    pub(super) negative_x_face: DatumId<PlaneDatum>,
    pub(super) positive_x_face: DatumId<PlaneDatum>,
}

#[derive(Clone, Copy)]
pub(super) struct M3WasherDefinition {
    pub(super) definition: ComponentDefinitionId,
    pub(super) axis: DatumId<AxisDatum>,
    pub(super) negative_x_face: DatumId<PlaneDatum>,
    pub(super) positive_x_face: DatumId<PlaneDatum>,
}

#[derive(Clone, Copy)]
pub(super) struct Defined<D> {
    pub(super) id: ComponentDefinitionId,
    pub(super) datums: D,
}

#[derive(Clone, Copy)]
pub(super) struct SectorDatums {
    pub(super) mount_face: DatumId<PlaneDatum>,
    pub(super) post_fasteners: [FastenerMemberDatums; 2],
}

#[derive(Clone, Copy)]
pub(super) struct CarrierPostDatums {
    pub(super) faces: BoxPlaneDatums,
    pub(super) fasteners: [PostFastenerDatums; 2],
}

#[derive(Clone, Copy)]
pub(super) struct ContactCarriageDatums {
    pub(super) negative_y: DatumId<PlaneDatum>,
    pub(super) positive_y: DatumId<PlaneDatum>,
    pub(super) fasteners: [FastenerMemberDatums; 3],
}

#[derive(Clone, Copy)]
pub(super) struct GearboxFarPlateDatums {
    pub(super) fasteners: [FastenerMemberDatums; 3],
}

#[derive(Clone, Copy)]
pub(super) struct CarrierEndDatums {
    pub(super) rail_face: DatumId<PlaneDatum>,
    pub(super) arm_face: DatumId<PlaneDatum>,
    pub(super) bearing_bore: DatumId<CylinderDatum>,
}

#[derive(Clone, Copy)]
pub(super) struct RollShaftDatums {
    pub(super) front_bearing_surface: DatumId<CylinderDatum>,
    pub(super) rear_bearing_surface: DatumId<CylinderDatum>,
}

#[derive(Clone, Copy)]
pub(super) struct RollBearingDatums {
    pub(super) inner_bore: DatumId<CylinderDatum>,
    pub(super) outer_surface: DatumId<CylinderDatum>,
}

#[derive(Clone, Copy)]
pub(super) struct CockpitDatums {
    pub(super) top_face: DatumId<PlaneDatum>,
}

#[derive(Clone, Copy)]
pub(super) struct CockpitHangerDatums {
    pub(super) cockpit_face: DatumId<PlaneDatum>,
}

#[derive(Clone, Copy)]
pub(super) struct RollGearboxPlateDatums {
    pub(super) negative_x: DatumId<PlaneDatum>,
    pub(super) positive_x: DatumId<PlaneDatum>,
}

#[derive(Clone, Copy)]
pub(super) struct MovingArmDatums {
    pub(super) carrier_face: DatumId<PlaneDatum>,
    pub(super) plate_face: DatumId<PlaneDatum>,
}

#[derive(Clone, Copy)]
pub(super) struct FixedFrameDefinitions {
    pub(super) sector: Defined<SectorDatums>,
    pub(super) carrier_rail: Defined<BoxPlaneDatums>,
    pub(super) carrier_post: Defined<CarrierPostDatums>,
    pub(super) crossmember: Defined<BoxPlaneDatums>,
    pub(super) floor: Defined<BoxPlaneDatums>,
}

#[derive(Clone, Copy)]
pub(super) struct PitchUnitDefinitions {
    pub(super) drive_pinion: ComponentDefinitionId,
    pub(super) encoder_pinion: ComponentDefinitionId,
    pub(super) drive_flange: ComponentDefinitionId,
    pub(super) encoder_flange: ComponentDefinitionId,
    pub(super) drive_shaft: ComponentDefinitionId,
    pub(super) encoder_shaft: ComponentDefinitionId,
    pub(super) gearbox_small: ComponentDefinitionId,
    pub(super) gearbox_distribution: ComponentDefinitionId,
    pub(super) gearbox_large: ComponentDefinitionId,
    pub(super) pitch_contact_outboard_plate: ComponentDefinitionId,
    pub(super) contact_carriage_plate: Defined<ContactCarriageDatums>,
    pub(super) pitch_gearbox_far_plate: Defined<GearboxFarPlateDatums>,
    pub(super) pitch_gearbox_shaft: ComponentDefinitionId,
}

#[derive(Clone, Copy)]
pub(super) struct RollDefinitions {
    pub(super) pitch_cradle_longitudinal_rail: Defined<BoxPlaneDatums>,
    pub(super) roll_bearing_carrier_end: Defined<CarrierEndDatums>,
    pub(super) cockpit: Defined<CockpitDatums>,
    pub(super) cockpit_hanger: Defined<CockpitHangerDatums>,
    pub(super) roll_shaft: Defined<RollShaftDatums>,
    pub(super) roll_driven: ComponentDefinitionId,
    pub(super) roll_pinion: ComponentDefinitionId,
    pub(super) roll_gearbox_small: ComponentDefinitionId,
    pub(super) roll_gearbox_large: ComponentDefinitionId,
    pub(super) roll_gearbox_shaft: ComponentDefinitionId,
    pub(super) roll_bearing: Defined<RollBearingDatums>,
    pub(super) roll_gearbox_plate: Defined<RollGearboxPlateDatums>,
    pub(super) moving_drive_mount_arm: Defined<MovingArmDatums>,
}

#[derive(Clone, Copy)]
pub(super) struct HardwareDefinitions {
    pub(super) m3x20_bolt: M3BoltDefinition,
    pub(super) m3x25_bolt: M3BoltDefinition,
    pub(super) m3_nut: M3NutDefinition,
    pub(super) m3_washer: M3WasherDefinition,
}

#[derive(Clone, Copy)]
pub(super) struct Definitions {
    pub(super) fixed_frame: FixedFrameDefinitions,
    pub(super) pitch_unit: PitchUnitDefinitions,
    pub(super) roll: RollDefinitions,
    pub(super) hardware: HardwareDefinitions,
}

pub(super) fn build_definitions(
    builder: &mut FeatureBuilder,
    assembly: &mut Assembly,
    p: &PrototypeParameters,
) -> Result<Definitions, PrototypeError> {
    let fdm = Manufacturing::Fdm;
    let sector_solid = dual_sector_solid(builder, p)?;
    let mut sector_datums = DatumSet::for_definition(assembly.next_definition_id());
    let sector_mount_face = add_plane_datum(
        &mut sector_datums,
        "mounting_spine_inner_face",
        [sector_spine_inner_x(p), 0.0, 0.0],
        [-1.0, 0.0, 0.0],
    );
    let sector_post_fasteners = sector_post_fastener_datums(&mut sector_datums, p);
    let sector = assembly.add_definition(ComponentDefinition {
        name: "pitch_dual_gear_sector".to_string(),
        role: ComponentRole::PitchSector,
        body: Body::Solid(sector_solid),
        manufacturing: fdm,
        color_rgba: [0.94, 0.52, 0.08, 1.0],
        datums: sector_datums,
    });
    let (carrier_rail, carrier_rail_faces) = sheet_box_definition_with_faces(
        builder,
        assembly,
        "pitch_carrier_rail",
        p.frame.fixed_rail_length.mm(),
        p.frame.fixed_rail_depth.mm(),
        p.frame.sheet_thickness,
        definition_style(ComponentRole::FixedCarrierRail, [0.58, 0.35, 0.16, 1.0]),
    )?;
    let carrier_post_height = p.frame.upper_rail_height.mm() + p.frame.lower_rail_depth.mm()
        - p.frame.fixed_rail_depth.mm();
    let (carrier_post, carrier_post_faces, carrier_post_fasteners) =
        carrier_post_definition(builder, assembly, p, carrier_post_height)?;
    let (crossmember, crossmember_faces) = add_box_definition_with_faces(
        builder,
        assembly,
        "pitch_crossmember",
        ComponentRole::FixedCrossmember,
        [
            p.frame.fixed_crossmember_width.mm(),
            p.pitch_sector.carrier_spacing.mm() - p.frame.sheet_thickness.mm(),
            p.frame.fixed_rail_depth.mm(),
        ],
        fdm,
        [0.58, 0.35, 0.16, 1.0],
    );
    let (pitch_cradle_longitudinal_rail, pitch_cradle_longitudinal_rail_faces) =
        add_box_definition_with_faces(
            builder,
            assembly,
            "pitch_cradle_longitudinal_rail",
            ComponentRole::PitchCradleLongitudinalRail,
            [
                p.frame.moving_carrier_half_span.mm() * 2.0,
                p.frame.moving_carrier_member_width.mm(),
                p.frame.moving_carrier_member_width.mm(),
            ],
            fdm,
            [0.24, 0.27, 0.32, 1.0],
        );
    let (floor, floor_faces) = add_box_definition_with_faces(
        builder,
        assembly,
        "installation_floor_reference",
        ComponentRole::InstallationFloor,
        [400.0, 250.0, p.frame.floor_thickness.mm()],
        Manufacturing::Purchased,
        [0.16, 0.18, 0.21, 1.0],
    );
    let drive_pinion = gear_definition_y(
        builder,
        assembly,
        "pitch_drive_pinion",
        &p.contact_unit.drive_pinion,
        p.pitch_sector.face_width,
        p.contact_unit.drive_shaft_radius,
        definition_style(ComponentRole::PitchDrivePinion, [0.10, 0.43, 0.84, 1.0]),
    )?;
    let encoder_pinion = gear_definition_y(
        builder,
        assembly,
        "pitch_retention_encoder_pinion",
        &p.contact_unit.encoder_pinion,
        p.pitch_sector.face_width,
        p.contact_unit.encoder_shaft_radius,
        definition_style(ComponentRole::PitchRetentionPinion, [0.10, 0.72, 0.34, 1.0]),
    )?;
    let drive_flange = annulus_definition_y(
        builder,
        assembly,
        "drive_retention_flange",
        p.contact_unit.drive_pinion.tip_radius() + 2.0,
        p.contact_unit.drive_shaft_radius.mm(),
        p.contact_unit.flange_thickness.mm(),
        definition_style(ComponentRole::PitchDriveFlange, [0.18, 0.48, 0.90, 1.0]),
    )?;
    let encoder_flange = annulus_definition_y(
        builder,
        assembly,
        "encoder_guide_flange",
        p.contact_unit.encoder_pinion.tip_radius() + 2.5,
        p.contact_unit.encoder_shaft_radius.mm(),
        p.contact_unit.flange_thickness.mm(),
        definition_style(ComponentRole::PitchRetentionFlange, [0.18, 0.80, 0.40, 1.0]),
    )?;
    let drive_shaft = add_solid_definition(
        assembly,
        "drive_shaft",
        ComponentRole::PitchDriveShaft,
        cylinder_y(builder, p.contact_unit.drive_shaft_radius.mm() - 0.15, 28.0)?,
        Manufacturing::Purchased,
        [0.62, 0.66, 0.70, 1.0],
    );
    let encoder_shaft = add_solid_definition(
        assembly,
        "encoder_interface_shaft",
        ComponentRole::PitchRetentionShaft,
        cylinder_y(
            builder,
            p.contact_unit.encoder_shaft_radius.mm() - 0.15,
            22.0,
        )?,
        Manufacturing::Purchased,
        [0.62, 0.66, 0.70, 1.0],
    );
    let gearbox_small = gear_definition_y(
        builder,
        assembly,
        "pitch_gearbox_small_gear",
        &p.pitch_gearbox.small_gear,
        p.pitch_gearbox.gear_face_width,
        p.pitch_gearbox.shaft_radius,
        definition_style(
            ComponentRole::PitchGearboxSmallGear,
            [0.66, 0.20, 0.72, 1.0],
        ),
    )?;
    let gearbox_large = gear_definition_y(
        builder,
        assembly,
        "pitch_gearbox_large_gear",
        &p.pitch_gearbox.large_gear,
        p.pitch_gearbox.gear_face_width,
        p.pitch_gearbox.shaft_radius,
        definition_style(
            ComponentRole::PitchGearboxLargeGear,
            [0.80, 0.28, 0.70, 1.0],
        ),
    )?;
    // The splitter is the same manufactured 54-tooth part as the two driven
    // gears. Reuse its Feature DAG solid so export and validation evaluate the
    // high-detail involute body only once, while retaining a semantic role for
    // assembly relations and reports.
    let large_definition = assembly
        .definition(gearbox_large)
        .expect("gearbox large definition was just inserted")
        .clone();
    let gearbox_distribution = assembly.add_definition(ComponentDefinition {
        name: "pitch_gearbox_distribution_gear".into(),
        role: ComponentRole::PitchGearboxDistributionGear,
        body: large_definition.body,
        manufacturing: large_definition.manufacturing,
        color_rgba: [0.74, 0.22, 0.72, 1.0],
        datums: large_definition.datums,
    });
    let pitch_contact_outboard_plate = add_solid_definition(
        assembly,
        "pitch_contact_outboard_plate",
        ComponentRole::PitchContactOutboardPlate,
        pitch_contact_outboard_plate_solid(builder, p)?,
        fdm,
        [0.20, 0.22, 0.27, 1.0],
    );
    let mut contact_carriage_datums = DatumSet::for_definition(assembly.next_definition_id());
    let contact_carriage_negative_y = add_plane_datum(
        &mut contact_carriage_datums,
        "negative_y",
        [0.0, -p.pitch_gearbox.side_plate_thickness.mm() * 0.5, 0.0],
        [0.0, -1.0, 0.0],
    );
    let contact_carriage_positive_y = add_plane_datum(
        &mut contact_carriage_datums,
        "positive_y",
        [0.0, p.pitch_gearbox.side_plate_thickness.mm() * 0.5, 0.0],
        [0.0, 1.0, 0.0],
    );
    let contact_carriage_fasteners = pitch_gearbox_plate_fastener_datums(
        &mut contact_carriage_datums,
        p.pitch_gearbox.side_plate_thickness.mm(),
        "near",
    );
    let contact_carriage_plate = add_solid_definition_with_datums(
        assembly,
        "pitch_contact_carriage_plate",
        ComponentRole::PitchContactCarriagePlate,
        pitch_contact_carriage_plate_solid(builder, p)?,
        fdm,
        [0.20, 0.22, 0.27, 1.0],
        contact_carriage_datums,
    );
    let mut far_plate_datums = DatumSet::for_definition(assembly.next_definition_id());
    let pitch_gearbox_far_plate_fasteners = pitch_gearbox_plate_fastener_datums(
        &mut far_plate_datums,
        p.pitch_gearbox.side_plate_thickness.mm(),
        "far",
    );
    let pitch_gearbox_far_plate = add_solid_definition_with_datums(
        assembly,
        "pitch_gearbox_far_plate",
        ComponentRole::PitchGearboxFarPlate,
        pitch_gearbox_far_plate_solid(builder, p)?,
        fdm,
        [0.20, 0.22, 0.27, 1.0],
        far_plate_datums,
    );
    let pitch_gearbox_shaft = add_solid_definition(
        assembly,
        "pitch_gearbox_shaft",
        ComponentRole::PitchGearboxShaft,
        cylinder_y(
            builder,
            p.pitch_gearbox.shaft_radius.mm() - 0.15,
            p.pitch_gearbox.far_plate_inboard_offset.mm()
                - p.pitch_gearbox.near_plate_inboard_offset.mm()
                + p.pitch_gearbox.side_plate_thickness.mm()
                + 2.0,
        )?,
        Manufacturing::Purchased,
        [0.62, 0.66, 0.70, 1.0],
    );
    let cockpit_size = [
        p.cockpit.length.mm(),
        p.cockpit.width.mm(),
        p.cockpit.height.mm(),
    ];
    let (cockpit_datums, cockpit_faces) =
        box_plane_datums(assembly.next_definition_id(), cockpit_size);
    let cockpit = add_solid_definition_with_datums(
        assembly,
        "cockpit_body",
        ComponentRole::Cockpit,
        centered_box(builder, cockpit_size),
        fdm,
        [0.86, 0.20, 0.18, 1.0],
        cockpit_datums,
    );
    let mut cockpit_hanger_datums = DatumSet::for_definition(assembly.next_definition_id());
    let cockpit_hanger_cockpit_face = add_plane_datum(
        &mut cockpit_hanger_datums,
        "cockpit_mount_face",
        [0.0, 0.0, cockpit_top_z(p)],
        [0.0, 0.0, -1.0],
    );
    let cockpit_hanger = add_solid_definition_with_datums(
        assembly,
        "cockpit_roll_shaft_clamp_hanger",
        ComponentRole::CockpitHanger,
        cockpit_hanger_solid(builder, p)?,
        fdm,
        [0.72, 0.25, 0.20, 1.0],
        cockpit_hanger_datums,
    );
    let mut roll_shaft_datums = DatumSet::for_definition(assembly.next_definition_id());
    let front_bearing_surface = add_cylinder_datum(
        &mut roll_shaft_datums,
        "front_bearing_surface",
        [p.roll_axis.bearing_station.mm(), 0.0, 0.0],
        [1.0, 0.0, 0.0],
        p.roll_axis.shaft_radius.mm(),
    );
    let rear_bearing_surface = add_cylinder_datum(
        &mut roll_shaft_datums,
        "rear_bearing_surface",
        [-p.roll_axis.bearing_station.mm(), 0.0, 0.0],
        [1.0, 0.0, 0.0],
        p.roll_axis.shaft_radius.mm(),
    );
    let roll_shaft = add_solid_definition_with_datums(
        assembly,
        "roll_shaft",
        ComponentRole::RollShaft,
        roll_shaft_solid(builder, p)?,
        Manufacturing::Purchased,
        [0.64, 0.67, 0.70, 1.0],
        roll_shaft_datums,
    );
    let roll_driven = add_solid_definition(
        assembly,
        "roll_driven_gear",
        ComponentRole::RollDrivenGear,
        roll_driven_gear_solid(builder, p)?,
        fdm,
        [0.88, 0.72, 0.08, 1.0],
    );
    let roll_pinion = gear_definition_x(
        builder,
        assembly,
        "roll_input_pinion",
        &p.roll_axis.pinion,
        length(6.0),
        p.pitch_gearbox.shaft_radius,
        definition_style(ComponentRole::RollInputPinion, [0.96, 0.80, 0.12, 1.0]),
    )?;
    let roll_gearbox_small = gear_definition_x(
        builder,
        assembly,
        "roll_gearbox_small_gear",
        &p.pitch_gearbox.small_gear,
        p.pitch_gearbox.gear_face_width,
        p.pitch_gearbox.shaft_radius,
        definition_style(ComponentRole::RollGearboxSmallGear, [0.66, 0.20, 0.72, 1.0]),
    )?;
    let roll_gearbox_large = gear_definition_x(
        builder,
        assembly,
        "roll_gearbox_large_gear",
        &p.pitch_gearbox.large_gear,
        p.pitch_gearbox.gear_face_width,
        p.pitch_gearbox.shaft_radius,
        definition_style(ComponentRole::RollGearboxLargeGear, [0.80, 0.28, 0.70, 1.0]),
    )?;
    let roll_gearbox_shaft = add_solid_definition(
        assembly,
        "roll_gearbox_shaft",
        ComponentRole::RollGearboxShaft,
        cylinder_x(builder, p.pitch_gearbox.shaft_radius.mm() - 0.15, 25.0)?,
        Manufacturing::Purchased,
        [0.62, 0.66, 0.70, 1.0],
    );
    let mut carrier_end_datums = DatumSet::for_definition(assembly.next_definition_id());
    let tie_center_x = roll_bearing_carrier_tie_center_x(p);
    let tie_half = p.frame.moving_carrier_member_width.mm() * 0.5;
    let roll_bearing_carrier_end_rail_face = add_plane_datum(
        &mut carrier_end_datums,
        "rail_contact_face",
        [
            tie_center_x - tie_half,
            0.0,
            p.frame.moving_carrier_height.mm(),
        ],
        [-1.0, 0.0, 0.0],
    );
    let roll_bearing_carrier_end_arm_face = add_plane_datum(
        &mut carrier_end_datums,
        "arm_contact_face",
        [
            tie_center_x + tie_half,
            0.0,
            p.frame.moving_carrier_height.mm(),
        ],
        [1.0, 0.0, 0.0],
    );
    let roll_bearing_carrier_bore = add_cylinder_datum(
        &mut carrier_end_datums,
        "bearing_bore",
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        p.roll_axis.bearing_outer_radius.mm() + roll_bearing_carrier_radial_clearance_mm(),
    );
    let roll_bearing_carrier_end = add_solid_definition_with_datums(
        assembly,
        "roll_bearing_carrier_end",
        ComponentRole::RollBearingCarrierEnd,
        roll_bearing_carrier_end_solid(builder, p)?,
        fdm,
        [0.56, 0.34, 0.16, 1.0],
        carrier_end_datums,
    );
    let mut roll_bearing_datums = DatumSet::for_definition(assembly.next_definition_id());
    let roll_bearing_inner_bore = add_cylinder_datum(
        &mut roll_bearing_datums,
        "inner_bore",
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        p.roll_axis.shaft_radius.mm() + roll_bearing_inner_radial_clearance_mm(),
    );
    let roll_bearing_outer_surface = add_cylinder_datum(
        &mut roll_bearing_datums,
        "outer_surface",
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        p.roll_axis.bearing_outer_radius.mm(),
    );
    let roll_bearing = add_solid_definition_with_datums(
        assembly,
        "roll_shaft_bearing",
        ComponentRole::RollBearing,
        annulus_solid_x(
            builder,
            p.roll_axis.bearing_outer_radius.mm(),
            p.roll_axis.shaft_radius.mm() + roll_bearing_inner_radial_clearance_mm(),
            p.roll_axis.bearing_width.mm(),
        )?,
        Manufacturing::Purchased,
        [0.48, 0.52, 0.56, 1.0],
        roll_bearing_datums,
    );
    let mut roll_gearbox_plate_datums = DatumSet::for_definition(assembly.next_definition_id());
    let roll_gearbox_plate_negative_x = add_plane_datum(
        &mut roll_gearbox_plate_datums,
        "negative_x",
        [-1.5, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
    );
    let roll_gearbox_plate_positive_x = add_plane_datum(
        &mut roll_gearbox_plate_datums,
        "positive_x",
        [1.5, 0.0, 0.0],
        [1.0, 0.0, 0.0],
    );
    let roll_gearbox_plate = add_solid_definition_with_datums(
        assembly,
        "roll_gearbox_plate",
        ComponentRole::RollGearboxPlate,
        roll_gearbox_plate_solid(builder, p)?,
        fdm,
        [0.20, 0.22, 0.27, 1.0],
        roll_gearbox_plate_datums,
    );
    let mut moving_arm_datums = DatumSet::for_definition(assembly.next_definition_id());
    let moving_drive_mount_arm_carrier_face = add_plane_datum(
        &mut moving_arm_datums,
        "carrier_contact_face",
        [p.frame.moving_carrier_member_width.mm() * 0.5, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
    );
    let moving_drive_mount_arm_plate_face = add_plane_datum(
        &mut moving_arm_datums,
        "gearbox_plate_contact_face",
        [
            roll_gearbox_arm_center_local_x(p) - roll_gearbox_arm_width_x() * 0.5,
            0.0,
            roll_gearbox_support_z(p) - p.frame.moving_carrier_height.mm(),
        ],
        [-1.0, 0.0, 0.0],
    );
    let moving_drive_mount_arm = add_solid_definition_with_datums(
        assembly,
        "moving_drive_mount_arm",
        ComponentRole::MovingDriveMountArm,
        moving_drive_mount_arm_solid(builder, p)?,
        fdm,
        [0.34, 0.24, 0.16, 1.0],
        moving_arm_datums,
    );
    let m3x20_bolt = add_m3_bolt_definition(builder, assembly, 20.0)?;
    let m3x25_bolt = add_m3_bolt_definition(builder, assembly, 25.0)?;
    let m3_nut = add_m3_nut_definition(builder, assembly)?;
    let m3_washer = add_m3_washer_definition(builder, assembly)?;
    Ok(Definitions {
        fixed_frame: FixedFrameDefinitions {
            sector: Defined {
                id: sector,
                datums: SectorDatums {
                    mount_face: sector_mount_face,
                    post_fasteners: sector_post_fasteners,
                },
            },
            carrier_rail: Defined {
                id: carrier_rail,
                datums: carrier_rail_faces,
            },
            carrier_post: Defined {
                id: carrier_post,
                datums: CarrierPostDatums {
                    faces: carrier_post_faces,
                    fasteners: carrier_post_fasteners,
                },
            },
            crossmember: Defined {
                id: crossmember,
                datums: crossmember_faces,
            },
            floor: Defined {
                id: floor,
                datums: floor_faces,
            },
        },
        pitch_unit: PitchUnitDefinitions {
            drive_pinion,
            encoder_pinion,
            drive_flange,
            encoder_flange,
            drive_shaft,
            encoder_shaft,
            gearbox_small,
            gearbox_distribution,
            gearbox_large,
            pitch_contact_outboard_plate,
            contact_carriage_plate: Defined {
                id: contact_carriage_plate,
                datums: ContactCarriageDatums {
                    negative_y: contact_carriage_negative_y,
                    positive_y: contact_carriage_positive_y,
                    fasteners: contact_carriage_fasteners,
                },
            },
            pitch_gearbox_far_plate: Defined {
                id: pitch_gearbox_far_plate,
                datums: GearboxFarPlateDatums {
                    fasteners: pitch_gearbox_far_plate_fasteners,
                },
            },
            pitch_gearbox_shaft,
        },
        roll: RollDefinitions {
            pitch_cradle_longitudinal_rail: Defined {
                id: pitch_cradle_longitudinal_rail,
                datums: pitch_cradle_longitudinal_rail_faces,
            },
            roll_bearing_carrier_end: Defined {
                id: roll_bearing_carrier_end,
                datums: CarrierEndDatums {
                    rail_face: roll_bearing_carrier_end_rail_face,
                    arm_face: roll_bearing_carrier_end_arm_face,
                    bearing_bore: roll_bearing_carrier_bore,
                },
            },
            cockpit: Defined {
                id: cockpit,
                datums: CockpitDatums {
                    top_face: cockpit_faces.positive_z,
                },
            },
            cockpit_hanger: Defined {
                id: cockpit_hanger,
                datums: CockpitHangerDatums {
                    cockpit_face: cockpit_hanger_cockpit_face,
                },
            },
            roll_shaft: Defined {
                id: roll_shaft,
                datums: RollShaftDatums {
                    front_bearing_surface,
                    rear_bearing_surface,
                },
            },
            roll_driven,
            roll_pinion,
            roll_gearbox_small,
            roll_gearbox_large,
            roll_gearbox_shaft,
            roll_bearing: Defined {
                id: roll_bearing,
                datums: RollBearingDatums {
                    inner_bore: roll_bearing_inner_bore,
                    outer_surface: roll_bearing_outer_surface,
                },
            },
            roll_gearbox_plate: Defined {
                id: roll_gearbox_plate,
                datums: RollGearboxPlateDatums {
                    negative_x: roll_gearbox_plate_negative_x,
                    positive_x: roll_gearbox_plate_positive_x,
                },
            },
            moving_drive_mount_arm: Defined {
                id: moving_drive_mount_arm,
                datums: MovingArmDatums {
                    carrier_face: moving_drive_mount_arm_carrier_face,
                    plate_face: moving_drive_mount_arm_plate_face,
                },
            },
        },
        hardware: HardwareDefinitions {
            m3x20_bolt,
            m3x25_bolt,
            m3_nut,
            m3_washer,
        },
    })
}

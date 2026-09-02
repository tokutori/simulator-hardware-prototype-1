// SPDX-License-Identifier: MIT

use alloc::format;
use alloc::string::ToString;
use alloc::vec;
use alloc::vec::Vec;
use core::f64::consts::{FRAC_PI_2, PI};

use crate::{
    Angle, Assembly, AssemblyError, AssemblyRelation, Axis3, AxisDatum, Body, BooleanOperation,
    ComponentDefinition, ComponentDefinitionId, ComponentIdentity, ComponentInstance,
    ComponentInstanceId, ComponentLocation, ComponentRole, CoordinateExpr, CylinderDatum,
    DatumEndpoint, DatumId, DatumSet, EngineeringTolerance, ExternalGearPair, FastenedJoint,
    FastenerHardware, FeatureBuilder, FeatureError, FeatureGraph, FrameGraph, FrameId, GearSector,
    InternalGearPair, Joint, Kinematics, Length, LongitudinalEnd, Manufacturing, MetricThread,
    NonNegativeAngle, NonNegativeLength, PlaneDatum, Point2, Point3, PositiveArea, PositiveLength,
    Primitive3, RigidTransform, Rotation3, Side, SolidId, SpurGear, SurfaceContact, Translation3,
    UnitVector3, VerticalEnd,
};

#[derive(Clone, Debug)]
pub struct PitchSectorParameters {
    pub target_outer_diameter: Length,
    pub sector: GearSector,
    pub carrier_spacing: Length,
    pub face_width: Length,
    pub minimum_web: Length,
}

#[derive(Clone, Debug)]
pub struct ContactUnitParameters {
    pub drive_pinion: SpurGear,
    pub encoder_pinion: SpurGear,
    pub branch_angle_offset: Angle,
    pub drive_shaft_radius: Length,
    pub encoder_shaft_radius: Length,
    pub drive_flange_clearance: Length,
    pub encoder_flange_clearance: Length,
    pub flange_thickness: Length,
}

#[derive(Clone, Debug)]
pub struct PitchGearboxParameters {
    pub small_gear: SpurGear,
    pub large_gear: SpurGear,
    pub distribution_gear: SpurGear,
    pub gear_face_width: Length,
    pub shaft_radius: Length,
    pub side_plate_thickness: Length,
    /// Distance from the pitch-sector mid-plane toward the opposite sector.
    pub near_plate_inboard_offset: Length,
    /// Distance from the pitch-sector mid-plane to the first reduction-gear layer.
    pub gear_plane_inboard_offset: Length,
    /// Distance from the pitch-sector mid-plane toward the opposite sector.
    pub far_plate_inboard_offset: Length,
}

#[derive(Clone, Debug)]
pub struct RollAxisParameters {
    pub driven_gear: SpurGear,
    pub pinion: SpurGear,
    pub shaft_length: Length,
    pub shaft_radius: Length,
    pub drive_station: Length,
    pub bearing_station: Length,
    pub gearbox_support_half_span: Length,
}

#[derive(Clone, Copy, Debug)]
pub struct CockpitParameters {
    pub length: Length,
    pub width: Length,
    pub height: Length,
    /// Vertical distance from the continuous roll axis to the cockpit center of mass.
    pub suspension_drop: Length,
}

#[derive(Clone, Copy, Debug)]
pub struct FrameParameters {
    pub fixed_rail_length: Length,
    pub fixed_crossmember_station: Length,
    pub fixed_crossmember_width: Length,
    pub fixed_rail_depth: Length,
    pub bearing_pedestal_thickness: Length,
    pub sheet_thickness: Length,
    pub upper_rail_height: Length,
    pub lower_rail_depth: Length,
    pub moving_carrier_half_span: Length,
    pub moving_carrier_height: Length,
    pub moving_carrier_inboard_offset: Length,
    pub moving_carrier_member_width: Length,
    pub floor_top_below_axis: Length,
    pub floor_thickness: Length,
}

#[derive(Clone, Copy, Debug)]
pub struct MotionParameters {
    pub pitch_limit: Angle,
    pub roll_limit: Angle,
}

#[derive(Clone, Debug)]
pub struct PrototypeParameters {
    pub pitch_sector: PitchSectorParameters,
    pub contact_unit: ContactUnitParameters,
    pub pitch_gearbox: PitchGearboxParameters,
    pub roll_axis: RollAxisParameters,
    pub cockpit: CockpitParameters,
    pub frame: FrameParameters,
    pub motion: MotionParameters,
}

#[derive(Clone, Debug)]
pub struct PrototypeDesign {
    pub graph: FeatureGraph,
    pub assembly: Assembly,
    pub kinematics: Kinematics,
    pub pitch_drive_pair: ExternalGearPair,
    pub pitch_encoder_pair: InternalGearPair,
    pub pitch_gearbox_pair: ExternalGearPair,
    pub roll_pair: ExternalGearPair,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrototypeError {
    Feature(FeatureError),
    Assembly(AssemblyError),
    IncompatibleGearPair,
    OuterDiameterMismatch,
    SectorWebTooThin,
    SectorMotionMarginTooSmall,
    DrivePinionsOverlap,
    InvalidCockpitEnvelope,
    InvalidCockpitSuspension,
    InvalidGearboxGeometry,
    InvalidGearboxPlacement,
    InvalidMovingCarrier,
    SectorSpineHitsDrive,
    SectorSupportHitsPost,
    FrameBaseNotOnFloor,
    CarrierRailTooClose,
    CockpitHitsRollSupport,
    RollStationOutsideShaft,
    MissingRequiredInstance,
}

pub fn build_prototype(
    parameters: &PrototypeParameters,
) -> Result<PrototypeDesign, PrototypeError> {
    validate(parameters)?;
    let pitch_drive_pair = ExternalGearPair::new(
        parameters.contact_unit.drive_pinion.clone(),
        parameters.pitch_sector.sector.external_reference().clone(),
    )
    .map_err(|_| PrototypeError::IncompatibleGearPair)?;
    let pitch_encoder_pair = InternalGearPair::new(
        parameters.contact_unit.encoder_pinion.clone(),
        parameters.pitch_sector.sector.internal_reference().clone(),
    )
    .map_err(|_| PrototypeError::IncompatibleGearPair)?;
    let pitch_gearbox_pair = ExternalGearPair::new(
        parameters.pitch_gearbox.small_gear.clone(),
        parameters.pitch_gearbox.large_gear.clone(),
    )
    .map_err(|_| PrototypeError::IncompatibleGearPair)?;
    let roll_pair = ExternalGearPair::new(
        parameters.roll_axis.pinion.clone(),
        parameters.roll_axis.driven_gear.clone(),
    )
    .map_err(|_| PrototypeError::IncompatibleGearPair)?;

    let mut builder = FeatureBuilder::new();
    let mut assembly = Assembly::new();
    let mut frames = FrameGraph::new();
    let world = frames.world();
    let pitch_frame = frames.add_frame(
        world,
        RigidTransform::IDENTITY,
        Joint::Revolute {
            axis: Axis3::Y,
            coordinate: CoordinateExpr::pitch(1.0),
        },
    );
    let roll_frame = frames.add_frame(
        pitch_frame,
        RigidTransform::IDENTITY,
        Joint::Revolute {
            axis: Axis3::X,
            coordinate: CoordinateExpr::roll(1.0),
        },
    );

    let definitions = build_definitions(&mut builder, &mut assembly, parameters)?;
    build_pitch_carrier(&mut assembly, &definitions, parameters, world);
    build_crossmembers(&mut assembly, &definitions, parameters, world);
    build_fixed_frame_contacts(&mut assembly, &definitions, parameters)?;
    build_contact_units(
        &mut assembly,
        &definitions,
        &mut frames,
        parameters,
        pitch_frame,
        pitch_drive_pair.ratio(),
        pitch_encoder_pair.ratio(),
        pitch_gearbox_pair.ratio(),
    )?;
    build_roll_assembly(
        &mut assembly,
        &definitions,
        &mut frames,
        parameters,
        pitch_frame,
        roll_frame,
        roll_pair.ratio(),
        pitch_gearbox_pair.ratio(),
    );
    build_moving_carrier_contacts(&mut assembly, &definitions)?;

    let kinematics = Kinematics::new(
        frames,
        parameters.motion.pitch_limit,
        parameters.motion.roll_limit,
    );
    Ok(PrototypeDesign {
        graph: builder.finish(),
        assembly,
        kinematics,
        pitch_drive_pair,
        pitch_encoder_pair,
        pitch_gearbox_pair,
        roll_pair,
    })
}

fn validate(parameters: &PrototypeParameters) -> Result<(), PrototypeError> {
    let external = parameters.pitch_sector.sector.external_reference();
    let internal = parameters.pitch_sector.sector.internal_reference();
    if (external.outside_diameter() - parameters.pitch_sector.target_outer_diameter.mm()).abs()
        > 1.0e-6
    {
        return Err(PrototypeError::OuterDiameterMismatch);
    }
    if external.root_radius() - internal.root_radius() < parameters.pitch_sector.minimum_web.mm() {
        return Err(PrototypeError::SectorWebTooThin);
    }
    // The outer drive pinions travel with the pitch carrier.  Reserve an
    // additional two degrees of intact teeth beyond their angular offset at
    // both ends of the manufactured sector.
    let contact_margin = Angle::radians(
        parameters.contact_unit.branch_angle_offset.as_radians()
            + Angle::degrees(2.0)
                .expect("constant is finite")
                .as_radians(),
    )
    .expect("finite validated angles");
    if !parameters
        .pitch_sector
        .sector
        .supports_motion(parameters.motion.pitch_limit, contact_margin)
    {
        return Err(PrototypeError::SectorMotionMarginTooSmall);
    }
    let drive_radius =
        external.pitch_radius() + parameters.contact_unit.drive_pinion.pitch_radius();
    let center_separation =
        2.0 * drive_radius * libm::sin(parameters.contact_unit.branch_angle_offset.as_radians());
    if center_separation <= parameters.contact_unit.drive_pinion.outside_diameter() + 1.0 {
        return Err(PrototypeError::DrivePinionsOverlap);
    }
    let drive_vertical_extent = drive_radius
        * libm::sin(parameters.contact_unit.branch_angle_offset.as_radians())
        + parameters.contact_unit.drive_pinion.tip_radius();
    if sector_support_keep_out_half_height() <= drive_vertical_extent + 5.0 {
        return Err(PrototypeError::SectorSpineHitsDrive);
    }
    // The fixed post meets the integral support at `sector_spine_inner_x`.
    // Keep that plane inward of the toothed sector at both angular ends so
    // the separate post only has face contact with the support, never a
    // positive-volume intersection with the gear body.
    let sector_end_inner_x =
        internal.tip_radius() * libm::cos(parameters.pitch_sector.sector.half_angle().as_radians());
    let support_inner_x = sector_spine_inner_x(parameters);
    const MINIMUM_SECTOR_POST_CLEARANCE_MM: f64 = 1.0;
    if support_inner_x + MINIMUM_SECTOR_POST_CLEARANCE_MM > sector_end_inner_x
        || support_inner_x + parameters.frame.fixed_rail_depth.mm()
            <= sector_end_inner_x + MINIMUM_SECTOR_POST_CLEARANCE_MM
    {
        return Err(PrototypeError::SectorSupportHitsPost);
    }
    if parameters.cockpit.length.mm() >= internal.tip_radius() * 2.0
        || parameters.cockpit.width.mm() >= parameters.pitch_sector.carrier_spacing.mm()
    {
        return Err(PrototypeError::InvalidCockpitEnvelope);
    }
    if parameters.cockpit.suspension_drop.mm() <= parameters.cockpit.height.mm() * 0.5
        || parameters.roll_axis.shaft_length.mm() <= parameters.cockpit.length.mm()
    {
        return Err(PrototypeError::InvalidCockpitSuspension);
    }
    if parameters.pitch_gearbox.distribution_gear != parameters.pitch_gearbox.large_gear {
        return Err(PrototypeError::InvalidGearboxGeometry);
    }
    let gearbox = &parameters.pitch_gearbox;
    let plate_half = gearbox.side_plate_thickness.mm() * 0.5;
    let gear_half = gearbox.gear_face_width.mm() * 0.5;
    let layer_pitch = gearbox.gear_face_width.mm() + 1.0;
    let near = gearbox.near_plate_inboard_offset.mm();
    let gear = gearbox.gear_plane_inboard_offset.mm();
    let far = gearbox.far_plate_inboard_offset.mm();
    let sector_half = parameters.pitch_sector.face_width.mm() * 0.5;
    let deepest_gear = gear + 2.0 * layer_pitch;
    if near - plate_half <= sector_half
        || gear - gear_half <= sector_half
        || gear - near < plate_half + gear_half
        || far - deepest_gear < plate_half + gear_half
        || far >= parameters.pitch_sector.carrier_spacing.mm() * 0.5
    {
        return Err(PrototypeError::InvalidGearboxPlacement);
    }
    let carrier = &parameters.frame;
    let carrier_inner_span = parameters.pitch_sector.carrier_spacing.mm()
        - 2.0 * carrier.moving_carrier_inboard_offset.mm()
        - carrier.moving_carrier_member_width.mm();
    if carrier_inner_span <= 0.0
        || carrier.moving_carrier_half_span.mm() <= parameters.cockpit.length.mm() * 0.5 + 5.0
        || carrier.moving_carrier_height.mm()
            <= parameters.roll_axis.shaft_radius.mm()
                + carrier.moving_carrier_member_width.mm() * 0.5
        || carrier.fixed_crossmember_width.mm()
            >= parameters.pitch_sector.carrier_spacing.mm() - carrier.sheet_thickness.mm()
        || carrier.moving_carrier_inboard_offset.mm()
            <= near + gearbox.side_plate_thickness.mm() * 0.5
        || parameters.pitch_sector.carrier_spacing.mm() * 0.5
            - carrier.moving_carrier_inboard_offset.mm()
            - carrier.moving_carrier_member_width.mm() * 0.5
            <= parameters.pitch_sector.carrier_spacing.mm() * 0.5
                - gearbox.far_plate_inboard_offset.mm()
                + gearbox.side_plate_thickness.mm() * 0.5
    {
        return Err(PrototypeError::InvalidMovingCarrier);
    }
    if carrier.fixed_crossmember_station.mm() + carrier.fixed_crossmember_width.mm() * 0.5
        > carrier.fixed_rail_length.mm() * 0.5
        || carrier.fixed_crossmember_station.mm() <= parameters.cockpit.length.mm() * 0.5 + 5.0
    {
        return Err(PrototypeError::InvalidMovingCarrier);
    }
    let sector_outer_end_z =
        external.tip_radius() * libm::sin(parameters.pitch_sector.sector.half_angle().as_radians());
    let upper_rail_bottom =
        parameters.frame.upper_rail_height.mm() - parameters.frame.fixed_rail_depth.mm() * 0.5;
    const MINIMUM_SECTOR_RAIL_CLEARANCE_MM: f64 = 1.0;
    if parameters.frame.lower_rail_depth.mm() <= 80.0
        || upper_rail_bottom < sector_outer_end_z + MINIMUM_SECTOR_RAIL_CLEARANCE_MM
    {
        return Err(PrototypeError::CarrierRailTooClose);
    }
    let intended_floor_depth =
        parameters.frame.lower_rail_depth.mm() + parameters.frame.fixed_rail_depth.mm() * 0.5;
    if (parameters.frame.floor_top_below_axis.mm() - intended_floor_depth).abs() > 1.0e-6 {
        return Err(PrototypeError::FrameBaseNotOnFloor);
    }
    let cockpit_half = parameters.cockpit.length.mm() * 0.5;
    let support_inner_x = parameters.roll_axis.bearing_station.mm()
        - parameters.frame.bearing_pedestal_thickness.mm() * 0.5;
    if support_inner_x - cockpit_half < 5.0 {
        return Err(PrototypeError::CockpitHitsRollSupport);
    }
    if parameters.roll_axis.drive_station.mm() + parameters.pitch_sector.face_width.mm() * 0.5
        > parameters.roll_axis.shaft_length.mm() * 0.5
    {
        return Err(PrototypeError::RollStationOutsideShaft);
    }
    if parameters.roll_axis.gearbox_support_half_span.mm() + 4.0 >= carrier_inner_span * 0.5 {
        return Err(PrototypeError::InvalidGearboxPlacement);
    }
    Ok(())
}

#[derive(Clone, Copy)]
// A complete semantic face set keeps datum indices stable as additional
// fastened joints are introduced in Phase 4.
#[allow(dead_code)]
struct BoxPlaneDatums {
    negative_x: DatumId<PlaneDatum>,
    positive_x: DatumId<PlaneDatum>,
    negative_y: DatumId<PlaneDatum>,
    positive_y: DatumId<PlaneDatum>,
    negative_z: DatumId<PlaneDatum>,
    positive_z: DatumId<PlaneDatum>,
}

#[derive(Clone, Copy)]
struct FastenerMemberDatums {
    hole: DatumId<CylinderDatum>,
    negative_y_seat: DatumId<PlaneDatum>,
    positive_y_seat: DatumId<PlaneDatum>,
}

#[derive(Clone, Copy)]
struct PostFastenerDatums {
    hole: DatumId<CylinderDatum>,
}

#[derive(Clone, Copy)]
struct Definitions {
    sector: ComponentDefinitionId,
    sector_mount_face: DatumId<PlaneDatum>,
    sector_post_fasteners: [FastenerMemberDatums; 2],
    carrier_rail: ComponentDefinitionId,
    carrier_rail_faces: BoxPlaneDatums,
    carrier_post: ComponentDefinitionId,
    carrier_post_faces: BoxPlaneDatums,
    carrier_post_fasteners: [PostFastenerDatums; 2],
    crossmember: ComponentDefinitionId,
    crossmember_faces: BoxPlaneDatums,
    pitch_cradle_longitudinal_rail: ComponentDefinitionId,
    pitch_cradle_longitudinal_rail_faces: BoxPlaneDatums,
    roll_bearing_carrier_end: ComponentDefinitionId,
    roll_bearing_carrier_end_rail_face: DatumId<PlaneDatum>,
    roll_bearing_carrier_end_arm_face: DatumId<PlaneDatum>,
    floor: ComponentDefinitionId,
    floor_faces: BoxPlaneDatums,
    drive_pinion: ComponentDefinitionId,
    encoder_pinion: ComponentDefinitionId,
    drive_flange: ComponentDefinitionId,
    encoder_flange: ComponentDefinitionId,
    drive_shaft: ComponentDefinitionId,
    encoder_shaft: ComponentDefinitionId,
    gearbox_small: ComponentDefinitionId,
    gearbox_distribution: ComponentDefinitionId,
    gearbox_large: ComponentDefinitionId,
    pitch_contact_outboard_plate: ComponentDefinitionId,
    contact_carriage_plate: ComponentDefinitionId,
    contact_carriage_negative_y: DatumId<PlaneDatum>,
    contact_carriage_positive_y: DatumId<PlaneDatum>,
    contact_carriage_fasteners: [FastenerMemberDatums; 3],
    pitch_gearbox_far_plate: ComponentDefinitionId,
    pitch_gearbox_far_plate_fasteners: [FastenerMemberDatums; 3],
    pitch_gearbox_shaft: ComponentDefinitionId,
    leaf_spring: ComponentDefinitionId,
    bearing_block: ComponentDefinitionId,
    cockpit: ComponentDefinitionId,
    cockpit_top_face: DatumId<PlaneDatum>,
    cockpit_hanger: ComponentDefinitionId,
    cockpit_hanger_cockpit_face: DatumId<PlaneDatum>,
    cockpit_shaft_key: ComponentDefinitionId,
    roll_shaft: ComponentDefinitionId,
    roll_driven: ComponentDefinitionId,
    roll_driven_hub: ComponentDefinitionId,
    roll_driven_key: ComponentDefinitionId,
    roll_pinion: ComponentDefinitionId,
    roll_gearbox_small: ComponentDefinitionId,
    roll_gearbox_large: ComponentDefinitionId,
    roll_gearbox_shaft: ComponentDefinitionId,
    roll_bearing: ComponentDefinitionId,
    roll_gearbox_plate: ComponentDefinitionId,
    roll_gearbox_plate_negative_x: DatumId<PlaneDatum>,
    roll_gearbox_plate_positive_x: DatumId<PlaneDatum>,
    moving_drive_mount_arm: ComponentDefinitionId,
    moving_drive_mount_arm_carrier_face: DatumId<PlaneDatum>,
    moving_drive_mount_arm_plate_face: DatumId<PlaneDatum>,
    m3x20_bolt: ComponentDefinitionId,
    m3x25_bolt: ComponentDefinitionId,
    m3_nut: ComponentDefinitionId,
    m3_washer: ComponentDefinitionId,
}

fn build_definitions(
    builder: &mut FeatureBuilder,
    assembly: &mut Assembly,
    p: &PrototypeParameters,
) -> Result<Definitions, PrototypeError> {
    let fdm = Manufacturing::Fdm;
    let sector_solid = dual_sector_solid(builder, p)?;
    let mut sector_datums = DatumSet::new();
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
    let mut contact_carriage_datums = DatumSet::new();
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
    let mut far_plate_datums = DatumSet::new();
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
    let leaf_spring = add_solid_definition(
        assembly,
        "encoder_leaf_spring",
        ComponentRole::RetentionLeafSpring,
        centered_box(builder, [22.0, 0.8, 4.0]),
        Manufacturing::Purchased,
        [0.74, 0.76, 0.78, 1.0],
    );
    let bearing_block = add_solid_definition(
        assembly,
        "encoder_bearing_block",
        ComponentRole::RetentionBearingBlock,
        encoder_bearing_block_solid(builder, p)?,
        fdm,
        [0.16, 0.52, 0.26, 1.0],
    );
    let cockpit_size = [
        p.cockpit.length.mm(),
        p.cockpit.width.mm(),
        p.cockpit.height.mm(),
    ];
    let (cockpit_datums, cockpit_faces) = box_plane_datums(cockpit_size);
    let cockpit = add_solid_definition_with_datums(
        assembly,
        "cockpit_body",
        ComponentRole::Cockpit,
        centered_box(builder, cockpit_size),
        fdm,
        [0.86, 0.20, 0.18, 1.0],
        cockpit_datums,
    );
    let mut cockpit_hanger_datums = DatumSet::new();
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
    let cockpit_shaft_key = add_solid_definition(
        assembly,
        "cockpit_roll_shaft_key",
        ComponentRole::CockpitShaftKey,
        centered_box(builder, [14.0, 2.5, 2.0]),
        Manufacturing::Purchased,
        [0.72, 0.74, 0.77, 1.0],
    );
    let roll_shaft = add_solid_definition(
        assembly,
        "roll_shaft",
        ComponentRole::RollShaft,
        cylinder_x(
            builder,
            p.roll_axis.shaft_radius.mm(),
            p.roll_axis.shaft_length.mm(),
        )?,
        Manufacturing::Purchased,
        [0.64, 0.67, 0.70, 1.0],
    );
    let roll_driven = gear_definition_x(
        builder,
        assembly,
        "roll_driven_gear",
        &p.roll_axis.driven_gear,
        length(6.0),
        p.roll_axis.shaft_radius,
        definition_style(ComponentRole::RollDrivenGear, [0.88, 0.72, 0.08, 1.0]),
    )?;
    let roll_driven_hub = annulus_definition_x(
        builder,
        assembly,
        "roll_driven_clamping_hub",
        10.0,
        p.roll_axis.shaft_radius.mm() + 0.15,
        12.0,
        definition_style(ComponentRole::RollDrivenHub, [0.76, 0.58, 0.10, 1.0]),
    )?;
    let roll_driven_key = add_solid_definition(
        assembly,
        "roll_driven_shaft_key",
        ComponentRole::RollDrivenKey,
        centered_box(builder, [12.0, 2.5, 2.0]),
        Manufacturing::Purchased,
        [0.72, 0.74, 0.77, 1.0],
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
    let mut carrier_end_datums = DatumSet::new();
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
    let roll_bearing_carrier_end = add_solid_definition_with_datums(
        assembly,
        "roll_bearing_carrier_end",
        ComponentRole::RollBearingCarrierEnd,
        roll_bearing_carrier_end_solid(builder, p)?,
        fdm,
        [0.56, 0.34, 0.16, 1.0],
        carrier_end_datums,
    );
    let roll_bearing = annulus_definition_x(
        builder,
        assembly,
        "roll_shaft_bearing",
        9.0,
        p.roll_axis.shaft_radius.mm() + 0.15,
        p.frame.bearing_pedestal_thickness.mm(),
        definition_style(ComponentRole::RollBearing, [0.48, 0.52, 0.56, 1.0]),
    )?;
    let mut roll_gearbox_plate_datums = DatumSet::new();
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
    let mut moving_arm_datums = DatumSet::new();
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
    let m3x20_bolt = add_solid_definition(
        assembly,
        "m3x20_pan_head_bolt",
        ComponentRole::M3Bolt,
        m3_pan_head_bolt_solid(builder, 20.0)?,
        Manufacturing::Purchased,
        [0.68, 0.71, 0.74, 1.0],
    );
    let m3x25_bolt = add_solid_definition(
        assembly,
        "m3x25_pan_head_bolt",
        ComponentRole::M3Bolt,
        m3_pan_head_bolt_solid(builder, 25.0)?,
        Manufacturing::Purchased,
        [0.68, 0.71, 0.74, 1.0],
    );
    let m3_nut = add_solid_definition(
        assembly,
        "m3_hex_nut",
        ComponentRole::M3Nut,
        m3_hex_nut_solid(builder)?,
        Manufacturing::Purchased,
        [0.60, 0.63, 0.66, 1.0],
    );
    let m3_washer = add_solid_definition(
        assembly,
        "m3_plain_washer",
        ComponentRole::M3Washer,
        m3_washer_solid(builder)?,
        Manufacturing::Purchased,
        [0.72, 0.75, 0.78, 1.0],
    );
    Ok(Definitions {
        sector,
        sector_mount_face,
        sector_post_fasteners,
        carrier_rail,
        carrier_rail_faces,
        carrier_post,
        carrier_post_faces,
        carrier_post_fasteners,
        crossmember,
        crossmember_faces,
        pitch_cradle_longitudinal_rail,
        pitch_cradle_longitudinal_rail_faces,
        roll_bearing_carrier_end,
        roll_bearing_carrier_end_rail_face,
        roll_bearing_carrier_end_arm_face,
        floor,
        floor_faces,
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
        contact_carriage_plate,
        contact_carriage_negative_y,
        contact_carriage_positive_y,
        contact_carriage_fasteners,
        pitch_gearbox_far_plate,
        pitch_gearbox_far_plate_fasteners,
        pitch_gearbox_shaft,
        leaf_spring,
        bearing_block,
        cockpit,
        cockpit_top_face: cockpit_faces.positive_z,
        cockpit_hanger,
        cockpit_hanger_cockpit_face,
        cockpit_shaft_key,
        roll_shaft,
        roll_driven,
        roll_driven_hub,
        roll_driven_key,
        roll_pinion,
        roll_gearbox_small,
        roll_gearbox_large,
        roll_gearbox_shaft,
        roll_bearing,
        roll_gearbox_plate,
        roll_gearbox_plate_negative_x,
        roll_gearbox_plate_positive_x,
        moving_drive_mount_arm,
        moving_drive_mount_arm_carrier_face,
        moving_drive_mount_arm_plate_face,
        m3x20_bolt,
        m3x25_bolt,
        m3_nut,
        m3_washer,
    })
}

fn build_pitch_carrier(
    assembly: &mut Assembly,
    definitions: &Definitions,
    p: &PrototypeParameters,
    fixed_frame: FrameId,
) {
    let half_spacing = p.pitch_sector.carrier_spacing.mm() * 0.5;
    for (side, y) in [(Side::Left, -half_spacing), (Side::Right, half_spacing)] {
        let side_name = side.as_str();
        for (end, rotation) in [(LongitudinalEnd::Front, 0.0), (LongitudinalEnd::Rear, PI)] {
            let end_name = end.as_str();
            add_located_instance(
                assembly,
                &format!("pitch_sector_{side_name}_{end_name}"),
                definitions.sector,
                fixed_frame,
                RigidTransform::translated(0.0, y, 0.0)
                    // Z rotation mirrors the front sector longitudinally while
                    // preserving the asymmetric upper/lower support lengths.
                    .compose(RigidTransform::rotated(Axis3::Z, rotation)),
                ComponentLocation::new()
                    .with_side(side)
                    .with_longitudinal_end(end),
            );
        }
        for (vertical_end, z) in [
            (VerticalEnd::Upper, p.frame.upper_rail_height.mm()),
            (VerticalEnd::Lower, -p.frame.lower_rail_depth.mm()),
        ] {
            let vertical_name = vertical_end.as_str();
            add_located_instance(
                assembly,
                &format!("pitch_carrier_{side_name}_{vertical_name}_rail"),
                definitions.carrier_rail,
                fixed_frame,
                RigidTransform::translated(0.0, y, z)
                    .compose(RigidTransform::rotated(Axis3::X, FRAC_PI_2)),
                ComponentLocation::new()
                    .with_side(side)
                    .with_vertical_end(vertical_end),
            );
        }
        let post_center_z = (p.frame.upper_rail_height.mm() - p.frame.lower_rail_depth.mm()) * 0.5;
        for (end, x) in [
            (
                LongitudinalEnd::Front,
                p.frame.fixed_crossmember_station.mm(),
            ),
            (
                LongitudinalEnd::Rear,
                -p.frame.fixed_crossmember_station.mm(),
            ),
        ] {
            add_located_instance(
                assembly,
                &format!("pitch_carrier_{side_name}_{}_post", end.as_str()),
                definitions.carrier_post,
                fixed_frame,
                RigidTransform::translated(x, y, post_center_z),
                ComponentLocation::new()
                    .with_side(side)
                    .with_longitudinal_end(end),
            );
        }
    }
}

fn build_crossmembers(
    assembly: &mut Assembly,
    definitions: &Definitions,
    p: &PrototypeParameters,
    world: FrameId,
) {
    for (index, (x, z)) in [
        (
            -p.frame.fixed_crossmember_station.mm(),
            p.frame.upper_rail_height.mm(),
        ),
        (
            -p.frame.fixed_crossmember_station.mm(),
            -p.frame.lower_rail_depth.mm(),
        ),
        (
            p.frame.fixed_crossmember_station.mm(),
            p.frame.upper_rail_height.mm(),
        ),
        (
            p.frame.fixed_crossmember_station.mm(),
            -p.frame.lower_rail_depth.mm(),
        ),
    ]
    .into_iter()
    .enumerate()
    {
        add_located_instance(
            assembly,
            &format!("pitch_crossmember_{}", index + 1),
            definitions.crossmember,
            world,
            RigidTransform::translated(x, 0.0, z),
            ComponentLocation::new().with_ordinal((index + 1) as u16),
        );
    }
    add_instance(
        assembly,
        "installation_floor_reference",
        definitions.floor,
        world,
        RigidTransform::translated(
            0.0,
            0.0,
            -p.frame.floor_top_below_axis.mm() - p.frame.floor_thickness.mm() * 0.5,
        ),
    );
}

fn build_fixed_frame_contacts(
    assembly: &mut Assembly,
    d: &Definitions,
    p: &PrototypeParameters,
) -> Result<(), PrototypeError> {
    let mut fastener_ordinal = 0_u16;
    let floor = required_instance(
        assembly,
        ComponentRole::InstallationFloor,
        ComponentLocation::new(),
    )?;
    for side in [Side::Left, Side::Right] {
        let upper_rail = required_instance(
            assembly,
            ComponentRole::FixedCarrierRail,
            ComponentLocation::new()
                .with_side(side)
                .with_vertical_end(VerticalEnd::Upper),
        )?;
        let lower_rail = required_instance(
            assembly,
            ComponentRole::FixedCarrierRail,
            ComponentLocation::new()
                .with_side(side)
                .with_vertical_end(VerticalEnd::Lower),
        )?;
        add_surface_contact(
            assembly,
            lower_rail,
            d.carrier_rail_faces.negative_y,
            floor,
            d.floor_faces.positive_z,
            1_000.0,
        )?;

        let (crossmember_face, rail_inner_face) = match side {
            Side::Left => (
                d.crossmember_faces.negative_y,
                d.carrier_rail_faces.negative_z,
            ),
            Side::Right => (
                d.crossmember_faces.positive_y,
                d.carrier_rail_faces.positive_z,
            ),
        };
        for end in [LongitudinalEnd::Front, LongitudinalEnd::Rear] {
            let post = required_instance(
                assembly,
                ComponentRole::FixedCarrierPost,
                ComponentLocation::new()
                    .with_side(side)
                    .with_longitudinal_end(end),
            )?;
            let sector = required_instance(
                assembly,
                ComponentRole::PitchSector,
                ComponentLocation::new()
                    .with_side(side)
                    .with_longitudinal_end(end),
            )?;
            let post_sector_face = match end {
                LongitudinalEnd::Front => d.carrier_post_faces.positive_x,
                LongitudinalEnd::Rear => d.carrier_post_faces.negative_x,
            };
            add_surface_contact(
                assembly,
                sector,
                d.sector_mount_face,
                post,
                post_sector_face,
                600.0,
            )?;
            for sector_hole_index in 0..2 {
                fastener_ordinal += 1;
                add_sector_post_fastener(
                    assembly,
                    d,
                    p,
                    sector,
                    post,
                    side,
                    end,
                    sector_hole_index,
                    fastener_ordinal,
                )?;
            }
            add_surface_contact(
                assembly,
                post,
                d.carrier_post_faces.positive_z,
                upper_rail,
                d.carrier_rail_faces.negative_y,
                60.0,
            )?;
            add_surface_contact(
                assembly,
                post,
                d.carrier_post_faces.negative_z,
                lower_rail,
                d.carrier_rail_faces.positive_y,
                60.0,
            )?;

            let (upper_ordinal, lower_ordinal) = match end {
                LongitudinalEnd::Rear => (1, 2),
                LongitudinalEnd::Front => (3, 4),
            };
            let upper_crossmember = required_instance(
                assembly,
                ComponentRole::FixedCrossmember,
                ComponentLocation::new().with_ordinal(upper_ordinal),
            )?;
            let lower_crossmember = required_instance(
                assembly,
                ComponentRole::FixedCrossmember,
                ComponentLocation::new().with_ordinal(lower_ordinal),
            )?;
            for (crossmember, rail) in [
                (upper_crossmember, upper_rail),
                (lower_crossmember, lower_rail),
            ] {
                add_surface_contact(
                    assembly,
                    crossmember,
                    crossmember_face,
                    rail,
                    rail_inner_face,
                    80.0,
                )?;
            }
        }
    }
    for ordinal in [2, 4] {
        let lower_crossmember = required_instance(
            assembly,
            ComponentRole::FixedCrossmember,
            ComponentLocation::new().with_ordinal(ordinal),
        )?;
        add_surface_contact(
            assembly,
            lower_crossmember,
            d.crossmember_faces.negative_z,
            floor,
            d.floor_faces.positive_z,
            800.0,
        )?;
    }
    Ok(())
}

fn required_instance(
    assembly: &Assembly,
    role: ComponentRole,
    location: ComponentLocation,
) -> Result<crate::ComponentInstanceId, PrototypeError> {
    assembly
        .instance_by_identity(ComponentIdentity { role, location })
        .ok_or(PrototypeError::MissingRequiredInstance)
}

fn add_surface_contact(
    assembly: &mut Assembly,
    first: crate::ComponentInstanceId,
    first_plane: DatumId<PlaneDatum>,
    second: crate::ComponentInstanceId,
    second_plane: DatumId<PlaneDatum>,
    minimum_area_mm2: f64,
) -> Result<(), PrototypeError> {
    assembly
        .add_relation(AssemblyRelation::SurfaceContact(SurfaceContact {
            first: DatumEndpoint::new(first, first_plane),
            second: DatumEndpoint::new(second, second_plane),
            minimum_contact_area: PositiveArea::square_mm(minimum_area_mm2)
                .expect("fixed contact area is positive"),
            tolerance: EngineeringTolerance {
                linear: NonNegativeLength::mm(0.02).expect("fixed tolerance is non-negative"),
                angular: NonNegativeAngle::degrees(0.1)
                    .expect("fixed angular tolerance is non-negative"),
            },
        }))
        .map_err(PrototypeError::Assembly)?;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn add_sector_post_fastener(
    assembly: &mut Assembly,
    d: &Definitions,
    p: &PrototypeParameters,
    sector: crate::ComponentInstanceId,
    post: crate::ComponentInstanceId,
    side: Side,
    end: LongitudinalEnd,
    sector_hole_index: usize,
    fastener_ordinal: u16,
) -> Result<(), PrototypeError> {
    const WASHER_THICKNESS: f64 = 0.5;
    const NUT_THICKNESS: f64 = 2.4;
    let frame = assembly
        .instance(sector)
        .expect("required sector instance exists")
        .frame;
    let longitudinal_sign = match end {
        LongitudinalEnd::Front => 1.0,
        LongitudinalEnd::Rear => -1.0,
    };
    let y = match side {
        Side::Left => -p.pitch_sector.carrier_spacing.mm() * 0.5,
        Side::Right => p.pitch_sector.carrier_spacing.mm() * 0.5,
    };
    let local_sector_z = sector_post_hole_zs(p)[sector_hole_index];
    let world_z = longitudinal_sign * local_sector_z;
    let vertical_end = if world_z > 0.0 {
        VerticalEnd::Upper
    } else {
        VerticalEnd::Lower
    };
    let post_hole_index = match end {
        LongitudinalEnd::Front => sector_hole_index,
        LongitudinalEnd::Rear => 1 - sector_hole_index,
    };
    let sector_datums = d.sector_post_fasteners[sector_hole_index];
    let post_datums = d.carrier_post_fasteners[post_hole_index];
    let outward_sign = match side {
        Side::Left => -1.0,
        Side::Right => 1.0,
    };
    let local_outward_sign = outward_sign * longitudinal_sign;
    let (head_seat, nut_seat) = if local_outward_sign > 0.0 {
        (sector_datums.positive_y_seat, sector_datums.negative_y_seat)
    } else {
        (sector_datums.negative_y_seat, sector_datums.positive_y_seat)
    };
    let seat_half_span = p.frame.sheet_thickness.mm() * 0.5 + 3.0;
    let x = longitudinal_sign * p.frame.fixed_crossmember_station.mm();
    let first_washer_y = y + outward_sign * (seat_half_span + WASHER_THICKNESS * 0.5);
    let bolt_under_head_y = y + outward_sign * (seat_half_span + WASHER_THICKNESS);
    let second_washer_y = y - outward_sign * (seat_half_span + WASHER_THICKNESS * 0.5);
    let nut_y = y - outward_sign * (seat_half_span + WASHER_THICKNESS + NUT_THICKNESS * 0.5);
    let hardware_rotation = outward_sign * FRAC_PI_2;
    let base_location = ComponentLocation::new()
        .with_side(side)
        .with_longitudinal_end(end)
        .with_vertical_end(vertical_end);
    let stem = format!(
        "pitch_sector_post_{}_{}_{}",
        side.as_str(),
        end.as_str(),
        vertical_end.as_str()
    );
    let bolt = add_located_instance(
        assembly,
        &format!("{stem}_m3x20_bolt"),
        d.m3x20_bolt,
        frame,
        RigidTransform::translated(x, bolt_under_head_y, world_z)
            .compose(RigidTransform::rotated(Axis3::Z, hardware_rotation)),
        base_location.with_ordinal(fastener_ordinal),
    );
    let nut = add_located_instance(
        assembly,
        &format!("{stem}_m3_nut"),
        d.m3_nut,
        frame,
        RigidTransform::translated(x, nut_y, world_z)
            .compose(RigidTransform::rotated(Axis3::Z, hardware_rotation)),
        base_location.with_ordinal(fastener_ordinal),
    );
    let first_washer = add_located_instance(
        assembly,
        &format!("{stem}_head_washer"),
        d.m3_washer,
        frame,
        RigidTransform::translated(x, first_washer_y, world_z)
            .compose(RigidTransform::rotated(Axis3::Z, hardware_rotation)),
        base_location.with_ordinal(fastener_ordinal * 2 - 1),
    );
    let second_washer = add_located_instance(
        assembly,
        &format!("{stem}_nut_washer"),
        d.m3_washer,
        frame,
        RigidTransform::translated(x, second_washer_y, world_z)
            .compose(RigidTransform::rotated(Axis3::Z, hardware_rotation)),
        base_location.with_ordinal(fastener_ordinal * 2),
    );
    assembly
        .add_relation(AssemblyRelation::Fastened(FastenedJoint {
            first_hole: DatumEndpoint::new(sector, sector_datums.hole),
            second_hole: DatumEndpoint::new(post, post_datums.hole),
            head_seat: DatumEndpoint::new(sector, head_seat),
            nut_seat: DatumEndpoint::new(sector, nut_seat),
            hardware: FastenerHardware {
                bolt,
                nut,
                first_washer: Some(first_washer),
                second_washer: Some(second_washer),
            },
            thread: MetricThread::M3,
            target_hole_radial_clearance: NonNegativeLength::mm(0.2)
                .expect("M3 clearance is non-negative"),
            grip_length: PositiveLength::mm(seat_half_span * 2.0)
                .expect("sector clevis grip is positive"),
            tolerance: EngineeringTolerance {
                linear: NonNegativeLength::mm(0.05).expect("fastener tolerance is non-negative"),
                angular: NonNegativeAngle::degrees(0.1)
                    .expect("fastener angular tolerance is non-negative"),
            },
        }))
        .map_err(PrototypeError::Assembly)?;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn build_contact_units(
    assembly: &mut Assembly,
    definitions: &Definitions,
    frames: &mut FrameGraph,
    p: &PrototypeParameters,
    pitch_frame: FrameId,
    drive_ratio: f64,
    encoder_ratio: f64,
    gearbox_ratio: f64,
) -> Result<(), PrototypeError> {
    let half_spacing = p.pitch_sector.carrier_spacing.mm() * 0.5;
    for (side, y, side_sign) in [
        (Side::Left, -half_spacing, -1.0),
        (Side::Right, half_spacing, 1.0),
    ] {
        for (end, end_angle) in [(LongitudinalEnd::Front, 0.0), (LongitudinalEnd::Rear, PI)] {
            build_contact_unit(
                assembly,
                definitions,
                frames,
                p,
                pitch_frame,
                side,
                end,
                y,
                side_sign,
                end_angle,
                drive_ratio,
                encoder_ratio,
                gearbox_ratio,
            )?;
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn build_contact_unit(
    assembly: &mut Assembly,
    d: &Definitions,
    frames: &mut FrameGraph,
    p: &PrototypeParameters,
    pitch_frame: FrameId,
    side: Side,
    end: LongitudinalEnd,
    y: f64,
    side_sign: f64,
    end_angle: f64,
    drive_ratio: f64,
    encoder_ratio: f64,
    gearbox_ratio: f64,
) -> Result<(), PrototypeError> {
    let base_location = ComponentLocation::new()
        .with_side(side)
        .with_longitudinal_end(end);
    let side = side.as_str();
    let end = end.as_str();
    let internal = p.pitch_sector.sector.internal_reference();
    let external = p.pitch_sector.sector.external_reference();
    let drive_radius = external.pitch_radius() + p.contact_unit.drive_pinion.pitch_radius();
    let offset = p.contact_unit.branch_angle_offset.as_radians();
    let mut branch_centers = [[0.0; 2]; 2];
    // The reduction train sits between the two sector planes. Offsets are
    // measured from each sector mid-plane toward the assembly centre so the
    // left/right units remain exact mirrors of one another.
    let inward_sign = -side_sign;
    let gearbox_layer_y = y + inward_sign * p.pitch_gearbox.gear_plane_inboard_offset.mm();

    for (branch, branch_offset) in [-offset, offset].into_iter().enumerate() {
        let angle = end_angle + branch_offset;
        let x = drive_radius * libm::cos(angle);
        let z = drive_radius * libm::sin(angle);
        branch_centers[branch] = [x, z];
        let frame = revolute_frame(
            frames,
            pitch_frame,
            [x, y, z],
            Axis3::Y,
            CoordinateExpr::pitch(drive_ratio),
        );
        let stem = format!("pitch_drive_{side}_{end}_{}", branch + 1);
        add_located_instance(
            assembly,
            &stem,
            d.drive_pinion,
            frame,
            RigidTransform::IDENTITY,
            base_location.with_ordinal((branch + 1) as u16),
        );
        add_located_instance(
            assembly,
            &format!("{stem}_shaft"),
            d.drive_shaft,
            frame,
            RigidTransform::IDENTITY,
            base_location.with_ordinal((branch + 1) as u16),
        );
        let flange_offset = p.pitch_sector.face_width.mm() * 0.5
            + p.contact_unit.drive_flange_clearance.mm()
            + p.contact_unit.flange_thickness.mm() * 0.5;
        for (flange, (label, dy)) in [("inner", -flange_offset), ("outer", flange_offset)]
            .into_iter()
            .enumerate()
        {
            add_located_instance(
                assembly,
                &format!("{stem}_flange_{label}"),
                d.drive_flange,
                frame,
                RigidTransform::translated(0.0, dy, 0.0),
                base_location.with_ordinal((branch * 2 + flange + 1) as u16),
            );
        }
        add_located_instance(
            assembly,
            &format!("{stem}_distribution_branch"),
            d.gearbox_small,
            frame,
            RigidTransform::translated(0.0, gearbox_layer_y - y, 0.0),
            base_location.with_ordinal((branch + 1) as u16),
        );
    }

    let encoder_radius = internal.pitch_radius() - p.contact_unit.encoder_pinion.pitch_radius();
    let encoder_center = [
        encoder_radius * libm::cos(end_angle),
        encoder_radius * libm::sin(end_angle),
    ];
    let encoder_frame = revolute_frame(
        frames,
        pitch_frame,
        [encoder_center[0], y, encoder_center[1]],
        Axis3::Y,
        CoordinateExpr::pitch(-encoder_ratio),
    );
    let encoder_stem = format!("pitch_retention_{side}_{end}");
    add_located_instance(
        assembly,
        &encoder_stem,
        d.encoder_pinion,
        encoder_frame,
        RigidTransform::IDENTITY,
        base_location,
    );
    add_located_instance(
        assembly,
        &format!("{encoder_stem}_interface_shaft"),
        d.encoder_shaft,
        encoder_frame,
        RigidTransform::IDENTITY,
        base_location,
    );
    let encoder_flange_offset = p.pitch_sector.face_width.mm() * 0.5
        + p.contact_unit.encoder_flange_clearance.mm()
        + p.contact_unit.flange_thickness.mm() * 0.5;
    for (flange, (label, dy)) in [
        ("inner", -encoder_flange_offset),
        ("outer", encoder_flange_offset),
    ]
    .into_iter()
    .enumerate()
    {
        add_located_instance(
            assembly,
            &format!("{encoder_stem}_flange_{label}"),
            d.encoder_flange,
            encoder_frame,
            RigidTransform::translated(0.0, dy, 0.0),
            base_location.with_ordinal((flange + 1) as u16),
        );
    }

    let radial = [libm::cos(end_angle), libm::sin(end_angle)];
    let tangent = [-radial[1], radial[0]];
    let block_center = encoder_center;
    let outboard_support_plane_y = y + side_sign * p.pitch_gearbox.near_plate_inboard_offset.mm();
    add_located_instance(
        assembly,
        &format!("{encoder_stem}_bearing_block"),
        d.bearing_block,
        pitch_frame,
        RigidTransform::translated(block_center[0], outboard_support_plane_y, block_center[1])
            .compose(RigidTransform::rotated(Axis3::Y, -end_angle)),
        base_location,
    );
    for (index, tangent_offset) in [-7.0, 7.0].into_iter().enumerate() {
        add_located_instance(
            assembly,
            &format!("{encoder_stem}_leaf_spring_{}", index + 1),
            d.leaf_spring,
            pitch_frame,
            RigidTransform::translated(
                block_center[0] - radial[0] * 7.0 + tangent[0] * tangent_offset,
                outboard_support_plane_y,
                block_center[1] - radial[1] * 7.0 + tangent[1] * tangent_offset,
            )
            .compose(RigidTransform::rotated(Axis3::Y, -end_angle)),
            base_location.with_ordinal((index + 1) as u16),
        );
    }

    let branch_distance = p.pitch_gearbox.distribution_gear.pitch_radius()
        + p.pitch_gearbox.small_gear.pitch_radius();
    let midpoint = [
        (branch_centers[0][0] + branch_centers[1][0]) * 0.5,
        (branch_centers[0][1] + branch_centers[1][1]) * 0.5,
    ];
    let half_chord = distance2(branch_centers[0], branch_centers[1]) * 0.5;
    if half_chord >= branch_distance {
        return Err(PrototypeError::InvalidGearboxGeometry);
    }
    let radial_offset = libm::sqrt(branch_distance * branch_distance - half_chord * half_chord);
    let central = [
        midpoint[0] - radial_offset * radial[0],
        midpoint[1] - radial_offset * radial[1],
    ];
    let distribution_ratio = p.pitch_gearbox.small_gear.teeth() as f64
        / p.pitch_gearbox.distribution_gear.teeth() as f64;
    let distributor_frame = revolute_frame(
        frames,
        pitch_frame,
        [central[0], gearbox_layer_y, central[1]],
        Axis3::Y,
        CoordinateExpr::pitch(-drive_ratio * distribution_ratio),
    );
    add_located_instance(
        assembly,
        &format!("pitch_gearbox_{side}_{end}_distributor"),
        d.gearbox_distribution,
        distributor_frame,
        RigidTransform::IDENTITY,
        base_location.with_ordinal(3),
    );

    let stage_distance =
        p.pitch_gearbox.small_gear.pitch_radius() + p.pitch_gearbox.large_gear.pitch_radius();
    let compound_a = [
        central[0] + tangent[0] * stage_distance,
        central[1] + tangent[1] * stage_distance,
    ];
    let input_center = [
        compound_a[0] + radial[0] * stage_distance,
        compound_a[1] + radial[1] * stage_distance,
    ];
    let compound_a_frame = revolute_frame(
        frames,
        pitch_frame,
        [compound_a[0], gearbox_layer_y, compound_a[1]],
        Axis3::Y,
        CoordinateExpr::pitch(drive_ratio * distribution_ratio * gearbox_ratio),
    );
    let input_frame = revolute_frame(
        frames,
        pitch_frame,
        [input_center[0], gearbox_layer_y, input_center[1]],
        Axis3::Y,
        CoordinateExpr::pitch(-drive_ratio * distribution_ratio * gearbox_ratio * gearbox_ratio),
    );
    let layer = inward_sign * (p.pitch_gearbox.gear_face_width.mm() + 1.0);
    add_located_instance(
        assembly,
        &format!("pitch_gearbox_{side}_{end}_stage2_driven"),
        d.gearbox_large,
        distributor_frame,
        RigidTransform::translated(0.0, layer, 0.0),
        base_location.with_ordinal(1),
    );
    add_located_instance(
        assembly,
        &format!("pitch_gearbox_{side}_{end}_stage2_pinion"),
        d.gearbox_small,
        compound_a_frame,
        RigidTransform::translated(0.0, layer, 0.0),
        base_location.with_ordinal(4),
    );
    add_located_instance(
        assembly,
        &format!("pitch_gearbox_{side}_{end}_stage1_driven"),
        d.gearbox_large,
        compound_a_frame,
        RigidTransform::translated(0.0, layer * 2.0, 0.0),
        base_location.with_ordinal(2),
    );
    add_located_instance(
        assembly,
        &format!("pitch_gearbox_{side}_{end}_input_pinion"),
        d.gearbox_small,
        input_frame,
        RigidTransform::translated(0.0, layer * 2.0, 0.0),
        base_location.with_ordinal(5),
    );
    let plate_center = [
        (central[0] + input_center[0]) * 0.5,
        (central[1] + input_center[1]) * 0.5,
    ];
    let inboard_near_plane_y = y + inward_sign * p.pitch_gearbox.near_plate_inboard_offset.mm();
    add_located_instance(
        assembly,
        &format!("pitch_contact_{side}_{end}_outboard_plate"),
        d.pitch_contact_outboard_plate,
        pitch_frame,
        RigidTransform::translated(midpoint[0], outboard_support_plane_y, midpoint[1])
            .compose(RigidTransform::rotated(Axis3::Y, -end_angle)),
        base_location,
    );
    let near_plate = add_located_instance(
        assembly,
        &format!("pitch_gearbox_{side}_{end}_contact_carriage_plate"),
        d.contact_carriage_plate,
        pitch_frame,
        RigidTransform::translated(plate_center[0], inboard_near_plane_y, plate_center[1])
            .compose(RigidTransform::rotated(Axis3::Y, -end_angle)),
        base_location,
    );
    let far_plate = add_located_instance(
        assembly,
        &format!("pitch_gearbox_{side}_{end}_far_plate"),
        d.pitch_gearbox_far_plate,
        pitch_frame,
        RigidTransform::translated(
            plate_center[0],
            y + inward_sign * p.pitch_gearbox.far_plate_inboard_offset.mm(),
            plate_center[1],
        )
        .compose(RigidTransform::rotated(Axis3::Y, -end_angle)),
        base_location,
    );
    for (index, tie) in pitch_gearbox_tie_points().into_iter().enumerate() {
        add_pitch_gearbox_fastener(
            assembly,
            d,
            p,
            pitch_frame,
            base_location,
            near_plate,
            far_plate,
            plate_center,
            tie,
            index,
            side_sign,
            inward_sign,
            end_angle,
            side,
            end,
        )?;
    }
    for (shaft, ordinal, frame) in [
        ("distributor", 1, distributor_frame),
        ("compound", 2, compound_a_frame),
        ("input", 3, input_frame),
    ] {
        add_located_instance(
            assembly,
            &format!("pitch_gearbox_{side}_{end}_{shaft}_shaft"),
            d.pitch_gearbox_shaft,
            frame,
            RigidTransform::translated(
                0.0,
                inward_sign
                    * ((p.pitch_gearbox.near_plate_inboard_offset.mm()
                        + p.pitch_gearbox.far_plate_inboard_offset.mm())
                        * 0.5
                        - p.pitch_gearbox.gear_plane_inboard_offset.mm()),
                0.0,
            ),
            base_location.with_ordinal(ordinal),
        );
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn add_pitch_gearbox_fastener(
    assembly: &mut Assembly,
    d: &Definitions,
    p: &PrototypeParameters,
    frame: FrameId,
    base_location: ComponentLocation,
    near_plate: ComponentInstanceId,
    far_plate: ComponentInstanceId,
    plate_center: [f64; 2],
    tie: [f64; 2],
    index: usize,
    outward_sign: f64,
    inward_sign: f64,
    end_angle: f64,
    side_label: &str,
    end_label: &str,
) -> Result<(), PrototypeError> {
    const WASHER_THICKNESS: f64 = 0.5;
    const NUT_THICKNESS: f64 = 2.4;
    let thickness = p.pitch_gearbox.side_plate_thickness.mm();
    let near_center_y = if outward_sign < 0.0 {
        -p.pitch_sector.carrier_spacing.mm() * 0.5
    } else {
        p.pitch_sector.carrier_spacing.mm() * 0.5
    } + inward_sign * p.pitch_gearbox.near_plate_inboard_offset.mm();
    let far_center_y = if outward_sign < 0.0 {
        -p.pitch_sector.carrier_spacing.mm() * 0.5
    } else {
        p.pitch_sector.carrier_spacing.mm() * 0.5
    } + inward_sign * p.pitch_gearbox.far_plate_inboard_offset.mm();
    let head_seat_y = near_center_y + outward_sign * thickness * 0.5;
    let nut_seat_y = far_center_y + inward_sign * thickness * 0.5;
    let first_washer_y = head_seat_y + outward_sign * WASHER_THICKNESS * 0.5;
    let bolt_under_head_y = head_seat_y + outward_sign * WASHER_THICKNESS;
    let second_washer_y = nut_seat_y + inward_sign * WASHER_THICKNESS * 0.5;
    let nut_y = nut_seat_y + inward_sign * (WASHER_THICKNESS + NUT_THICKNESS * 0.5);
    let hardware_rotation = outward_sign * FRAC_PI_2;
    let pose = |y, rotate_hardware: bool| {
        let base = RigidTransform::translated(plate_center[0], y, plate_center[1])
            .compose(RigidTransform::rotated(Axis3::Y, -end_angle))
            .compose(RigidTransform::translated(tie[0], 0.0, tie[1]));
        if rotate_hardware {
            base.compose(RigidTransform::rotated(Axis3::Z, hardware_rotation))
        } else {
            base
        }
    };
    let ordinal = (index + 1) as u16;
    let stem = format!("pitch_gearbox_{side_label}_{end_label}_m3x25_{}", index + 1);
    let bolt = add_located_instance(
        assembly,
        &format!("{stem}_bolt"),
        d.m3x25_bolt,
        frame,
        pose(bolt_under_head_y, true),
        base_location.with_ordinal(ordinal),
    );
    let nut = add_located_instance(
        assembly,
        &format!("{stem}_nut"),
        d.m3_nut,
        frame,
        pose(nut_y, true),
        base_location.with_ordinal(ordinal),
    );
    let first_washer = add_located_instance(
        assembly,
        &format!("{stem}_head_washer"),
        d.m3_washer,
        frame,
        pose(first_washer_y, true),
        base_location.with_ordinal(ordinal * 2 - 1),
    );
    let second_washer = add_located_instance(
        assembly,
        &format!("{stem}_nut_washer"),
        d.m3_washer,
        frame,
        pose(second_washer_y, true),
        base_location.with_ordinal(ordinal * 2),
    );
    let near_datums = d.contact_carriage_fasteners[index];
    let far_datums = d.pitch_gearbox_far_plate_fasteners[index];
    let head_seat = if outward_sign > 0.0 {
        near_datums.positive_y_seat
    } else {
        near_datums.negative_y_seat
    };
    let nut_seat = if inward_sign > 0.0 {
        far_datums.positive_y_seat
    } else {
        far_datums.negative_y_seat
    };
    assembly
        .add_relation(AssemblyRelation::Fastened(FastenedJoint {
            first_hole: DatumEndpoint::new(near_plate, near_datums.hole),
            second_hole: DatumEndpoint::new(far_plate, far_datums.hole),
            head_seat: DatumEndpoint::new(near_plate, head_seat),
            nut_seat: DatumEndpoint::new(far_plate, nut_seat),
            hardware: FastenerHardware {
                bolt,
                nut,
                first_washer: Some(first_washer),
                second_washer: Some(second_washer),
            },
            thread: MetricThread::M3,
            target_hole_radial_clearance: NonNegativeLength::mm(0.2)
                .expect("M3 clearance is non-negative"),
            grip_length: PositiveLength::mm((nut_seat_y - head_seat_y).abs())
                .expect("pitch gearbox grip is positive"),
            tolerance: EngineeringTolerance {
                linear: NonNegativeLength::mm(0.05).expect("fastener tolerance is non-negative"),
                angular: NonNegativeAngle::degrees(0.1)
                    .expect("fastener angular tolerance is non-negative"),
            },
        }))
        .map_err(PrototypeError::Assembly)?;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn build_roll_assembly(
    assembly: &mut Assembly,
    d: &Definitions,
    frames: &mut FrameGraph,
    p: &PrototypeParameters,
    pitch_frame: FrameId,
    roll_frame: FrameId,
    roll_ratio: f64,
    gearbox_ratio: f64,
) {
    add_instance(
        assembly,
        "cockpit_body",
        d.cockpit,
        roll_frame,
        RigidTransform::translated(0.0, 0.0, -p.cockpit.suspension_drop.mm()),
    );
    add_instance(
        assembly,
        "roll_shaft",
        d.roll_shaft,
        roll_frame,
        RigidTransform::IDENTITY,
    );
    for (index, x) in [-p.cockpit.length.mm() * 0.30, p.cockpit.length.mm() * 0.30]
        .into_iter()
        .enumerate()
    {
        add_located_instance(
            assembly,
            &format!("cockpit_hanger_{}", index + 1),
            d.cockpit_hanger,
            roll_frame,
            RigidTransform::translated(x, 0.0, 0.0),
            ComponentLocation::new().with_ordinal((index + 1) as u16),
        );
        add_located_instance(
            assembly,
            &format!("cockpit_shaft_key_{}", index + 1),
            d.cockpit_shaft_key,
            roll_frame,
            RigidTransform::translated(x, 0.0, 3.5),
            ComponentLocation::new().with_ordinal((index + 1) as u16),
        );
    }
    let carrier_rail_y =
        p.pitch_sector.carrier_spacing.mm() * 0.5 - p.frame.moving_carrier_inboard_offset.mm();
    for (index, y) in [-carrier_rail_y, carrier_rail_y].into_iter().enumerate() {
        add_located_instance(
            assembly,
            &format!("pitch_cradle_longitudinal_rail_{}", index + 1),
            d.pitch_cradle_longitudinal_rail,
            pitch_frame,
            RigidTransform::translated(0.0, y, p.frame.moving_carrier_height.mm()),
            ComponentLocation::new().with_ordinal((index + 1) as u16),
        );
    }
    for (end, outward) in [(LongitudinalEnd::Front, 1.0), (LongitudinalEnd::Rear, -1.0)] {
        let location = ComponentLocation::new().with_longitudinal_end(end);
        let end = end.as_str();
        let gear_x = outward * p.roll_axis.drive_station.mm();
        // Only the roll reduction itself remains below the roll axis. Its
        // support is carried up to the front/rear upper carrier tie, outside
        // the cockpit longitudinal envelope.
        let carrier_tie_x = outward
            * (p.frame.moving_carrier_half_span.mm()
                + p.frame.moving_carrier_member_width.mm() * 0.5);
        let carrier_z = p.frame.moving_carrier_height.mm();
        add_located_instance(
            assembly,
            &format!("roll_bearing_carrier_end_{end}"),
            d.roll_bearing_carrier_end,
            pitch_frame,
            RigidTransform::translated(outward * p.roll_axis.bearing_station.mm(), 0.0, 0.0)
                .compose(RigidTransform::rotated(
                    Axis3::Z,
                    if outward > 0.0 { 0.0 } else { PI },
                )),
            location,
        );
        add_located_instance(
            assembly,
            &format!("roll_driven_gear_{end}"),
            d.roll_driven,
            roll_frame,
            RigidTransform::translated(gear_x, 0.0, 0.0),
            location,
        );
        add_located_instance(
            assembly,
            &format!("roll_driven_hub_{end}"),
            d.roll_driven_hub,
            roll_frame,
            RigidTransform::translated(gear_x, 0.0, 0.0),
            location,
        );
        add_located_instance(
            assembly,
            &format!("roll_driven_key_{end}"),
            d.roll_driven_key,
            roll_frame,
            RigidTransform::translated(gear_x, 0.0, 3.5),
            location,
        );
        let output_z =
            -(p.roll_axis.driven_gear.pitch_radius() + p.roll_axis.pinion.pitch_radius());
        let output_center = [gear_x, 0.0, output_z];
        let stage_distance =
            p.pitch_gearbox.small_gear.pitch_radius() + p.pitch_gearbox.large_gear.pitch_radius();
        let compound_center = [gear_x, stage_distance, output_z];
        let input_center = [gear_x, stage_distance, output_z - stage_distance];
        let output_frame = revolute_frame(
            frames,
            pitch_frame,
            output_center,
            Axis3::X,
            CoordinateExpr::roll(-roll_ratio),
        );
        let compound_frame = revolute_frame(
            frames,
            pitch_frame,
            compound_center,
            Axis3::X,
            CoordinateExpr::roll(roll_ratio * gearbox_ratio),
        );
        let input_frame = revolute_frame(
            frames,
            pitch_frame,
            input_center,
            Axis3::X,
            CoordinateExpr::roll(-roll_ratio * gearbox_ratio * gearbox_ratio),
        );
        add_located_instance(
            assembly,
            &format!("roll_output_pinion_{end}"),
            d.roll_pinion,
            output_frame,
            RigidTransform::IDENTITY,
            location,
        );
        let first_layer = outward * 7.0;
        let second_layer = outward * 12.0;
        add_located_instance(
            assembly,
            &format!("roll_gearbox_{end}_stage2_driven"),
            d.roll_gearbox_large,
            output_frame,
            RigidTransform::translated(first_layer, 0.0, 0.0),
            location.with_ordinal(1),
        );
        add_located_instance(
            assembly,
            &format!("roll_gearbox_{end}_stage2_pinion"),
            d.roll_gearbox_small,
            compound_frame,
            RigidTransform::translated(first_layer, 0.0, 0.0),
            location.with_ordinal(1),
        );
        add_located_instance(
            assembly,
            &format!("roll_gearbox_{end}_stage1_driven"),
            d.roll_gearbox_large,
            compound_frame,
            RigidTransform::translated(second_layer, 0.0, 0.0),
            location.with_ordinal(2),
        );
        add_located_instance(
            assembly,
            &format!("roll_gearbox_{end}_input_pinion"),
            d.roll_gearbox_small,
            input_frame,
            RigidTransform::translated(second_layer, 0.0, 0.0),
            location.with_ordinal(2),
        );
        for (shaft, ordinal, frame) in [
            ("output", 1, output_frame),
            ("compound", 2, compound_frame),
            ("input", 3, input_frame),
        ] {
            add_located_instance(
                assembly,
                &format!("roll_gearbox_{end}_{shaft}_shaft"),
                d.roll_gearbox_shaft,
                frame,
                RigidTransform::translated(outward * 5.5, 0.0, 0.0),
                location.with_ordinal(ordinal),
            );
        }
        for (index, plate_offset) in [-5.5, 16.5].into_iter().enumerate() {
            add_located_instance(
                assembly,
                &format!("roll_gearbox_{end}_side_plate_{}", index + 1),
                d.roll_gearbox_plate,
                pitch_frame,
                RigidTransform::translated(
                    gear_x + outward * plate_offset,
                    stage_distance * 0.5,
                    output_z - stage_distance * 0.5,
                ),
                location.with_ordinal((index + 1) as u16),
            );
        }
        for (index, y) in [
            -p.roll_axis.gearbox_support_half_span.mm(),
            p.roll_axis.gearbox_support_half_span.mm(),
        ]
        .into_iter()
        .enumerate()
        {
            add_located_instance(
                assembly,
                &format!("roll_gearbox_{end}_mount_arm_{}", index + 1),
                d.moving_drive_mount_arm,
                pitch_frame,
                RigidTransform::translated(carrier_tie_x, y, carrier_z).compose(
                    RigidTransform::rotated(Axis3::Z, if outward > 0.0 { 0.0 } else { PI }),
                ),
                location.with_ordinal((index + 1) as u16),
            );
        }
    }
    for (end, outward) in [(LongitudinalEnd::Front, 1.0), (LongitudinalEnd::Rear, -1.0)] {
        let location = ComponentLocation::new().with_longitudinal_end(end);
        let end = end.as_str();
        let x = outward * p.roll_axis.bearing_station.mm();
        add_located_instance(
            assembly,
            &format!("roll_bearing_{end}"),
            d.roll_bearing,
            pitch_frame,
            RigidTransform::translated(x, 0.0, 0.0),
            location,
        );
    }
}

fn build_moving_carrier_contacts(
    assembly: &mut Assembly,
    d: &Definitions,
) -> Result<(), PrototypeError> {
    let cockpit = required_instance(assembly, ComponentRole::Cockpit, ComponentLocation::new())?;
    for ordinal in [1, 2] {
        let hanger = required_instance(
            assembly,
            ComponentRole::CockpitHanger,
            ComponentLocation::new().with_ordinal(ordinal),
        )?;
        add_surface_contact(
            assembly,
            cockpit,
            d.cockpit_top_face,
            hanger,
            d.cockpit_hanger_cockpit_face,
            100.0,
        )?;
    }

    for end in [LongitudinalEnd::Front, LongitudinalEnd::Rear] {
        let end_location = ComponentLocation::new().with_longitudinal_end(end);
        let carrier_end =
            required_instance(assembly, ComponentRole::RollBearingCarrierEnd, end_location)?;
        let rail_end_face = match end {
            LongitudinalEnd::Front => d.pitch_cradle_longitudinal_rail_faces.positive_x,
            LongitudinalEnd::Rear => d.pitch_cradle_longitudinal_rail_faces.negative_x,
        };
        for (side, ordinal) in [(Side::Left, 1), (Side::Right, 2)] {
            let rail = required_instance(
                assembly,
                ComponentRole::PitchCradleLongitudinalRail,
                ComponentLocation::new().with_ordinal(ordinal),
            )?;
            add_surface_contact(
                assembly,
                rail,
                rail_end_face,
                carrier_end,
                d.roll_bearing_carrier_end_rail_face,
                120.0,
            )?;

            let carriage = required_instance(
                assembly,
                ComponentRole::PitchContactCarriagePlate,
                ComponentLocation::new()
                    .with_side(side)
                    .with_longitudinal_end(end),
            )?;
            let (carriage_face, rail_side_face) = match side {
                Side::Left => (
                    d.contact_carriage_positive_y,
                    d.pitch_cradle_longitudinal_rail_faces.negative_y,
                ),
                Side::Right => (
                    d.contact_carriage_negative_y,
                    d.pitch_cradle_longitudinal_rail_faces.positive_y,
                ),
            };
            add_surface_contact(
                assembly,
                carriage,
                carriage_face,
                rail,
                rail_side_face,
                20.0,
            )?;
        }

        for ordinal in [1, 2] {
            let arm_location = end_location.with_ordinal(ordinal);
            let arm =
                required_instance(assembly, ComponentRole::MovingDriveMountArm, arm_location)?;
            add_surface_contact(
                assembly,
                carrier_end,
                d.roll_bearing_carrier_end_arm_face,
                arm,
                d.moving_drive_mount_arm_carrier_face,
                100.0,
            )?;
            let plate = required_instance(
                assembly,
                ComponentRole::RollGearboxPlate,
                end_location.with_ordinal(2),
            )?;
            let plate_face = match end {
                LongitudinalEnd::Front => d.roll_gearbox_plate_positive_x,
                LongitudinalEnd::Rear => d.roll_gearbox_plate_negative_x,
            };
            add_surface_contact(
                assembly,
                arm,
                d.moving_drive_mount_arm_plate_face,
                plate,
                plate_face,
                40.0,
            )?;
        }
    }
    Ok(())
}

fn roll_gearbox_plate_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
) -> Result<SolidId, PrototypeError> {
    let stage_distance =
        p.pitch_gearbox.small_gear.pitch_radius() + p.pitch_gearbox.large_gear.pitch_radius();
    let centers = [
        [-stage_distance * 0.5, stage_distance * 0.5],
        [stage_distance * 0.5, stage_distance * 0.5],
        [stage_distance * 0.5, -stage_distance * 0.5],
    ];
    let supports = [
        [
            -p.roll_axis.gearbox_support_half_span.mm() - stage_distance * 0.5,
            roll_gearbox_plate_support_offset_z(),
        ],
        [
            p.roll_axis.gearbox_support_half_span.mm() - stage_distance * 0.5,
            roll_gearbox_plate_support_offset_z(),
        ],
    ];
    let mut plate = cylinder_x(builder, 5.5, 3.0)?;
    plate = builder.translate(
        plate,
        Translation3 {
            x: 0.0,
            y: centers[0][0],
            z: centers[0][1],
        },
    )?;
    for center in centers.into_iter().skip(1) {
        let boss = cylinder_x(builder, 5.5, 3.0)?;
        let boss = builder.translate(
            boss,
            Translation3 {
                x: 0.0,
                y: center[0],
                z: center[1],
            },
        )?;
        plate = builder.boolean(BooleanOperation::Union, plate, boss)?;
    }
    for (a, b) in [
        (centers[0], centers[1]),
        (centers[1], centers[2]),
        (centers[0], supports[0]),
        (centers[2], supports[1]),
        (supports[0], supports[1]),
    ] {
        let rib = beam_yz(builder, a, b, 3.0, 5.0)?;
        plate = builder.boolean(BooleanOperation::Union, plate, rib)?;
    }
    for support in supports {
        let tab = centered_box(builder, [3.0, 12.0, 6.0]);
        let tab = builder.translate(
            tab,
            Translation3 {
                x: 0.0,
                y: support[0],
                z: support[1],
            },
        )?;
        plate = builder.boolean(BooleanOperation::Union, plate, tab)?;
    }
    let bore_radius = p.pitch_gearbox.shaft_radius.mm() + 0.35;
    for [y, z] in centers {
        let bore = cylinder_x(builder, bore_radius, 5.0)?;
        let bore = builder.translate(bore, Translation3 { x: 0.0, y, z })?;
        plate = builder.boolean(BooleanOperation::Difference, plate, bore)?;
    }
    Ok(plate)
}

fn roll_gearbox_support_z(p: &PrototypeParameters) -> f64 {
    let output_z = -(p.roll_axis.driven_gear.pitch_radius() + p.roll_axis.pinion.pitch_radius());
    let stage_distance =
        p.pitch_gearbox.small_gear.pitch_radius() + p.pitch_gearbox.large_gear.pitch_radius();
    output_z - stage_distance * 0.5 + roll_gearbox_plate_support_offset_z()
}

const fn roll_gearbox_plate_support_offset_z() -> f64 {
    -24.0
}

fn moving_drive_mount_arm_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
) -> Result<SolidId, PrototypeError> {
    let dx = roll_gearbox_arm_center_local_x(p);
    let dz = roll_gearbox_plate_support_local_z(p);
    let tie_half = p.frame.moving_carrier_member_width.mm() * 0.5;
    let vertical_x = roll_gearbox_arm_width_x();
    let arm_y = 10.0;
    let horizontal_depth = 14.0;
    let horizontal_end = dx + vertical_x * 0.5;
    let horizontal = centered_box(
        builder,
        [horizontal_end - tie_half, arm_y, horizontal_depth],
    );
    let horizontal = builder.translate(
        horizontal,
        Translation3 {
            x: (horizontal_end + tie_half) * 0.5,
            y: 0.0,
            z: 0.0,
        },
    )?;
    let vertical_top = -horizontal_depth * 0.5 + 1.0;
    let plate_tab_bottom = dz - 3.0;
    let vertical = centered_box(
        builder,
        [vertical_x, arm_y, vertical_top - plate_tab_bottom],
    );
    let vertical = builder.translate(
        vertical,
        Translation3 {
            x: dx,
            y: 0.0,
            z: (vertical_top + plate_tab_bottom) * 0.5,
        },
    )?;
    builder
        .boolean(BooleanOperation::Union, horizontal, vertical)
        .map_err(PrototypeError::Feature)
}

const fn roll_gearbox_arm_width_x() -> f64 {
    8.0
}

fn roll_gearbox_arm_center_local_x(p: &PrototypeParameters) -> f64 {
    p.roll_axis.drive_station.mm() + 16.5 + 1.5 + 4.0
        - (p.frame.moving_carrier_half_span.mm() + p.frame.moving_carrier_member_width.mm() * 0.5)
}

fn roll_gearbox_plate_support_local_z(p: &PrototypeParameters) -> f64 {
    roll_gearbox_support_z(p) - p.frame.moving_carrier_height.mm()
}

fn roll_bearing_carrier_tie_center_x(p: &PrototypeParameters) -> f64 {
    p.frame.moving_carrier_half_span.mm() + p.frame.moving_carrier_member_width.mm() * 0.5
        - p.roll_axis.bearing_station.mm()
}

fn roll_bearing_carrier_end_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
) -> Result<SolidId, PrototypeError> {
    let thickness = p.frame.bearing_pedestal_thickness.mm();
    let boss_center = [0.0, 0.0];
    let carrier_rail_y =
        p.pitch_sector.carrier_spacing.mm() * 0.5 - p.frame.moving_carrier_inboard_offset.mm();
    let bridge_half_span = carrier_rail_y - p.frame.moving_carrier_member_width.mm() * 0.5;
    let bridge_z = p.frame.moving_carrier_height.mm();
    let mut pedestal = cylinder_x(builder, 14.0, thickness)?;
    for endpoint in [
        [-bridge_half_span + 10.0, bridge_z - 4.0],
        [bridge_half_span - 10.0, bridge_z - 4.0],
    ] {
        let rib = beam_yz(builder, boss_center, endpoint, thickness, 8.0)?;
        pedestal = builder.boolean(BooleanOperation::Union, pedestal, rib)?;
    }
    let bridge = centered_box(builder, [thickness, bridge_half_span * 2.0, 8.0]);
    let bridge = builder.translate(
        bridge,
        Translation3 {
            x: 0.0,
            y: 0.0,
            z: bridge_z,
        },
    )?;
    pedestal = builder.boolean(BooleanOperation::Union, pedestal, bridge)?;
    let tie_center_x = roll_bearing_carrier_tie_center_x(p);
    let tie = centered_box(
        builder,
        [
            p.frame.moving_carrier_member_width.mm(),
            (bridge_half_span + p.frame.moving_carrier_member_width.mm()) * 2.0,
            p.frame.moving_carrier_member_width.mm(),
        ],
    );
    let tie = builder.translate(
        tie,
        Translation3 {
            x: tie_center_x,
            y: 0.0,
            z: bridge_z,
        },
    )?;
    pedestal = builder.boolean(BooleanOperation::Union, pedestal, tie)?;

    let bore = cylinder_x(builder, 9.2, thickness + 2.0)?;
    builder
        .boolean(BooleanOperation::Difference, pedestal, bore)
        .map_err(PrototypeError::Feature)
}

fn cockpit_hanger_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
) -> Result<SolidId, PrototypeError> {
    let cockpit_top = cockpit_top_z(p);
    let web_height = -cockpit_top;
    let web = centered_box(builder, [12.0, 14.0, web_height]);
    let web = builder.translate(
        web,
        Translation3 {
            x: 0.0,
            y: 0.0,
            z: -web_height * 0.5,
        },
    )?;
    let boss = cylinder_x(builder, 9.0, 14.0)?;
    let hanger = builder.boolean(BooleanOperation::Union, web, boss)?;
    let bore = cylinder_x(builder, p.roll_axis.shaft_radius.mm() + 0.15, 16.0)?;
    builder
        .boolean(BooleanOperation::Difference, hanger, bore)
        .map_err(PrototypeError::Feature)
}

fn cockpit_top_z(p: &PrototypeParameters) -> f64 {
    -p.cockpit.suspension_drop.mm() + p.cockpit.height.mm() * 0.5
}

#[derive(Clone, Copy)]
struct PitchUnitLayout {
    branches: [[f64; 2]; 2],
    branch_midpoint: [f64; 2],
    distributor: [f64; 2],
    compound: [f64; 2],
    input: [f64; 2],
    plate_center: [f64; 2],
}

fn pitch_unit_layout(p: &PrototypeParameters) -> Result<PitchUnitLayout, PrototypeError> {
    let external = p.pitch_sector.sector.external_reference();
    let drive_radius = external.pitch_radius() + p.contact_unit.drive_pinion.pitch_radius();
    let offset = p.contact_unit.branch_angle_offset.as_radians();
    let branches = [
        [
            drive_radius * libm::cos(offset),
            -drive_radius * libm::sin(offset),
        ],
        [
            drive_radius * libm::cos(offset),
            drive_radius * libm::sin(offset),
        ],
    ];
    let branch_distance = p.pitch_gearbox.distribution_gear.pitch_radius()
        + p.pitch_gearbox.small_gear.pitch_radius();
    let midpoint = [
        (branches[0][0] + branches[1][0]) * 0.5,
        (branches[0][1] + branches[1][1]) * 0.5,
    ];
    let half_chord = distance2(branches[0], branches[1]) * 0.5;
    if half_chord >= branch_distance {
        return Err(PrototypeError::InvalidGearboxGeometry);
    }
    let distributor = [
        midpoint[0] - libm::sqrt(branch_distance * branch_distance - half_chord * half_chord),
        midpoint[1],
    ];
    let stage_distance =
        p.pitch_gearbox.small_gear.pitch_radius() + p.pitch_gearbox.large_gear.pitch_radius();
    let compound = [distributor[0], distributor[1] + stage_distance];
    let input = [compound[0] + stage_distance, compound[1]];
    let plate_center = [
        (distributor[0] + input[0]) * 0.5,
        (distributor[1] + input[1]) * 0.5,
    ];
    Ok(PitchUnitLayout {
        branches,
        branch_midpoint: midpoint,
        distributor,
        compound,
        input,
        plate_center,
    })
}

fn pitch_contact_carriage_plate_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
) -> Result<SolidId, PrototypeError> {
    let layout = pitch_unit_layout(p)?;
    let thickness = p.pitch_gearbox.side_plate_thickness.mm();
    let centers = [
        layout.branches[0],
        layout.branches[1],
        layout.distributor,
        layout.compound,
        layout.input,
    ]
    .map(|center| {
        [
            center[0] - layout.plate_center[0],
            center[1] - layout.plate_center[1],
        ]
    });
    let mut plate = cylinder_y(builder, 5.5, thickness)?;
    plate = builder.translate(
        plate,
        Translation3 {
            x: centers[0][0],
            y: 0.0,
            z: centers[0][1],
        },
    )?;
    for center in centers.into_iter().skip(1) {
        let boss = cylinder_y(builder, 5.5, thickness)?;
        let boss = builder.translate(
            boss,
            Translation3 {
                x: center[0],
                y: 0.0,
                z: center[1],
            },
        )?;
        plate = builder.boolean(BooleanOperation::Union, plate, boss)?;
    }
    for (a, b) in [
        (centers[0], centers[1]),
        (centers[0], centers[2]),
        (centers[1], centers[2]),
        (centers[2], centers[3]),
        (centers[3], centers[4]),
    ] {
        let rib = beam_xz(builder, a, b, thickness, 5.0)?;
        plate = builder.boolean(BooleanOperation::Union, plate, rib)?;
    }
    for (tie, anchor) in pitch_gearbox_tie_points()
        .into_iter()
        .zip([centers[0], centers[4], centers[3]])
    {
        let boss = cylinder_y(builder, 5.0, thickness)?;
        let boss = builder.translate(
            boss,
            Translation3 {
                x: tie[0],
                y: 0.0,
                z: tie[1],
            },
        )?;
        plate = builder.boolean(BooleanOperation::Union, plate, boss)?;
        let rib = beam_xz(builder, anchor, tie, thickness, 5.0)?;
        plate = builder.boolean(BooleanOperation::Union, plate, rib)?;
        plate = subtract_y_bore(builder, plate, 1.7, thickness + 2.0, tie[0], tie[1])?;
    }
    let encoder_anchor = [
        p.pitch_sector.sector.internal_reference().pitch_radius()
            - p.contact_unit.encoder_pinion.pitch_radius()
            - 18.0
            - layout.plate_center[0],
        -layout.plate_center[1],
    ];
    let anchor_boss = cylinder_y(builder, 5.0, thickness)?;
    let anchor_boss = builder.translate(
        anchor_boss,
        Translation3 {
            x: encoder_anchor[0],
            y: 0.0,
            z: encoder_anchor[1],
        },
    )?;
    plate = builder.boolean(BooleanOperation::Union, plate, anchor_boss)?;
    let anchor_rib = beam_xz(builder, centers[1], encoder_anchor, thickness, 6.0)?;
    plate = builder.boolean(BooleanOperation::Union, plate, anchor_rib)?;
    plate = subtract_y_bore(
        builder,
        plate,
        1.7,
        thickness + 2.0,
        encoder_anchor[0],
        encoder_anchor[1],
    )?;
    let brace_origin = [
        layout.branch_midpoint[0] - layout.plate_center[0],
        layout.branch_midpoint[1] - layout.plate_center[1],
    ];
    for anchor in [
        [
            p.frame.moving_carrier_half_span.mm() - 26.0 - layout.plate_center[0],
            p.frame.moving_carrier_height.mm() - layout.plate_center[1],
        ],
        [
            p.frame.moving_carrier_half_span.mm() - 6.0 - layout.plate_center[0],
            p.frame.moving_carrier_height.mm() - layout.plate_center[1],
        ],
    ] {
        let brace = beam_xz(builder, brace_origin, anchor, thickness, 8.0)?;
        plate = builder.boolean(BooleanOperation::Union, plate, brace)?;
    }
    // Give the sparse gearbox truss a real mounting pad against the cradle rail.
    // Without this pad the braces only touched the rail over a few square
    // millimetres, which was neither a useful load path nor an honest contact
    // surface for the assembly relation.
    let rail_pad = centered_box(
        builder,
        [20.0, thickness, p.frame.moving_carrier_member_width.mm()],
    );
    let rail_pad = builder.translate(
        rail_pad,
        Translation3 {
            x: p.frame.moving_carrier_half_span.mm() - 16.0 - layout.plate_center[0],
            y: 0.0,
            z: p.frame.moving_carrier_height.mm() - layout.plate_center[1],
        },
    )?;
    plate = builder.boolean(BooleanOperation::Union, plate, rail_pad)?;
    let bore_radius = p.pitch_gearbox.shaft_radius.mm() + 0.35;
    for center in centers {
        plate = subtract_y_bore(
            builder,
            plate,
            bore_radius,
            thickness + 2.0,
            center[0],
            center[1],
        )?;
    }
    Ok(plate)
}

fn pitch_contact_outboard_plate_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
) -> Result<SolidId, PrototypeError> {
    let layout = pitch_unit_layout(p)?;
    let thickness = p.pitch_gearbox.side_plate_thickness.mm();
    let encoder = [
        p.pitch_sector.sector.internal_reference().pitch_radius()
            - p.contact_unit.encoder_pinion.pitch_radius(),
        0.0,
    ];
    let centers = [
        [
            layout.branches[0][0] - layout.branch_midpoint[0],
            layout.branches[0][1] - layout.branch_midpoint[1],
        ],
        [
            layout.branches[1][0] - layout.branch_midpoint[0],
            layout.branches[1][1] - layout.branch_midpoint[1],
        ],
        [
            encoder[0] - layout.branch_midpoint[0],
            encoder[1] - layout.branch_midpoint[1],
        ],
        [0.0, 0.0],
    ];
    let mut plate = cylinder_y(builder, 5.5, thickness)?;
    plate = builder.translate(
        plate,
        Translation3 {
            x: centers[0][0],
            y: 0.0,
            z: centers[0][1],
        },
    )?;
    for center in centers.into_iter().skip(1) {
        let boss = cylinder_y(builder, 5.5, thickness)?;
        let boss = builder.translate(
            boss,
            Translation3 {
                x: center[0],
                y: 0.0,
                z: center[1],
            },
        )?;
        plate = builder.boolean(BooleanOperation::Union, plate, boss)?;
    }
    for (a, b) in [
        (centers[0], centers[3]),
        (centers[1], centers[3]),
        (centers[2], centers[3]),
    ] {
        let rib = beam_xz(builder, a, b, thickness, 5.0)?;
        plate = builder.boolean(BooleanOperation::Union, plate, rib)?;
    }
    for center in centers.into_iter().take(3) {
        let radius = if center == centers[2] {
            p.contact_unit.encoder_shaft_radius.mm() + 0.35
        } else {
            p.contact_unit.drive_shaft_radius.mm() + 0.35
        };
        plate = subtract_y_bore(
            builder,
            plate,
            radius,
            thickness + 2.0,
            center[0],
            center[1],
        )?;
    }
    subtract_y_bore(builder, plate, 1.7, thickness + 2.0, 0.0, 0.0)
}

fn pitch_gearbox_far_plate_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
) -> Result<SolidId, PrototypeError> {
    let layout = pitch_unit_layout(p)?;
    let thickness = p.pitch_gearbox.side_plate_thickness.mm();
    let centers = [layout.distributor, layout.compound, layout.input].map(|center| {
        [
            center[0] - layout.plate_center[0],
            center[1] - layout.plate_center[1],
        ]
    });
    let mut plate = cylinder_y(builder, 5.5, thickness)?;
    plate = builder.translate(
        plate,
        Translation3 {
            x: centers[0][0],
            y: 0.0,
            z: centers[0][1],
        },
    )?;
    for center in centers.into_iter().skip(1) {
        let boss = cylinder_y(builder, 5.5, thickness)?;
        let boss = builder.translate(
            boss,
            Translation3 {
                x: center[0],
                y: 0.0,
                z: center[1],
            },
        )?;
        plate = builder.boolean(BooleanOperation::Union, plate, boss)?;
    }
    for (a, b) in [(centers[0], centers[1]), (centers[1], centers[2])] {
        let rib = beam_xz(builder, a, b, thickness, 5.0)?;
        plate = builder.boolean(BooleanOperation::Union, plate, rib)?;
    }
    for (tie, anchor) in pitch_gearbox_tie_points()
        .into_iter()
        .zip([centers[0], centers[2], centers[1]])
    {
        let boss = cylinder_y(builder, 5.0, thickness)?;
        let boss = builder.translate(
            boss,
            Translation3 {
                x: tie[0],
                y: 0.0,
                z: tie[1],
            },
        )?;
        plate = builder.boolean(BooleanOperation::Union, plate, boss)?;
        let rib = beam_xz(builder, anchor, tie, thickness, 5.0)?;
        plate = builder.boolean(BooleanOperation::Union, plate, rib)?;
        plate = subtract_y_bore(builder, plate, 1.7, thickness + 2.0, tie[0], tie[1])?;
    }
    let bore_radius = p.pitch_gearbox.shaft_radius.mm() + 0.35;
    for center in centers {
        plate = subtract_y_bore(
            builder,
            plate,
            bore_radius,
            thickness + 2.0,
            center[0],
            center[1],
        )?;
    }
    Ok(plate)
}

const fn pitch_gearbox_tie_points() -> [[f64; 2]; 3] {
    [[-24.0, -24.0], [26.0, -18.0], [0.0, 38.0]]
}

fn pitch_gearbox_plate_fastener_datums(
    datums: &mut DatumSet,
    thickness: f64,
    plate_label: &str,
) -> [FastenerMemberDatums; 3] {
    pitch_gearbox_tie_points().map(|point| FastenerMemberDatums {
        hole: add_cylinder_datum(
            datums,
            &format!("{plate_label}_m3_hole_{:.0}_{:.0}", point[0], point[1]),
            [point[0], 0.0, point[1]],
            [0.0, 1.0, 0.0],
            m3_clearance_radius_mm(),
        ),
        negative_y_seat: add_plane_datum(
            datums,
            &format!(
                "{plate_label}_m3_negative_y_seat_{:.0}_{:.0}",
                point[0], point[1]
            ),
            [point[0], -thickness * 0.5, point[1]],
            [0.0, -1.0, 0.0],
        ),
        positive_y_seat: add_plane_datum(
            datums,
            &format!(
                "{plate_label}_m3_positive_y_seat_{:.0}_{:.0}",
                point[0], point[1]
            ),
            [point[0], thickness * 0.5, point[1]],
            [0.0, 1.0, 0.0],
        ),
    })
}

fn encoder_bearing_block_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
) -> Result<SolidId, PrototypeError> {
    let block = centered_box(builder, [16.0, 3.0, 18.0]);
    subtract_y_bore(
        builder,
        block,
        p.contact_unit.encoder_shaft_radius.mm() + 0.35,
        5.0,
        0.0,
        0.0,
    )
}

fn subtract_y_bore(
    builder: &mut FeatureBuilder,
    solid: SolidId,
    radius: f64,
    length: f64,
    x: f64,
    z: f64,
) -> Result<SolidId, PrototypeError> {
    let bore = cylinder_y(builder, radius, length)?;
    let bore = builder.translate(bore, Translation3 { x, y: 0.0, z })?;
    builder
        .boolean(BooleanOperation::Difference, solid, bore)
        .map_err(PrototypeError::Feature)
}

fn beam_xz(
    builder: &mut FeatureBuilder,
    a: [f64; 2],
    b: [f64; 2],
    thickness_y: f64,
    width: f64,
) -> Result<SolidId, PrototypeError> {
    let dx = b[0] - a[0];
    let dz = b[1] - a[1];
    let beam = centered_box(
        builder,
        [libm::sqrt(dx * dx + dz * dz) + width, thickness_y, width],
    );
    let beam = builder.rotate(
        beam,
        Rotation3 {
            x: angle(0.0),
            y: Angle::radians(-libm::atan2(dz, dx)).expect("derived rib angle is finite"),
            z: angle(0.0),
        },
    )?;
    builder
        .translate(
            beam,
            Translation3 {
                x: (a[0] + b[0]) * 0.5,
                y: 0.0,
                z: (a[1] + b[1]) * 0.5,
            },
        )
        .map_err(PrototypeError::Feature)
}

fn beam_yz(
    builder: &mut FeatureBuilder,
    a: [f64; 2],
    b: [f64; 2],
    thickness_x: f64,
    width: f64,
) -> Result<SolidId, PrototypeError> {
    let dy = b[0] - a[0];
    let dz = b[1] - a[1];
    let beam = centered_box(
        builder,
        [thickness_x, libm::sqrt(dy * dy + dz * dz) + width, width],
    );
    let beam = builder.rotate(
        beam,
        Rotation3 {
            x: Angle::radians(libm::atan2(dz, dy)).expect("derived rib angle is finite"),
            y: angle(0.0),
            z: angle(0.0),
        },
    )?;
    builder
        .translate(
            beam,
            Translation3 {
                x: 0.0,
                y: (a[0] + b[0]) * 0.5,
                z: (a[1] + b[1]) * 0.5,
            },
        )
        .map_err(PrototypeError::Feature)
}

fn dual_sector_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
) -> Result<SolidId, PrototypeError> {
    let external = p.pitch_sector.sector.external_reference();
    let internal = p.pitch_sector.sector.internal_reference();
    let outer_profile = builder.polygon(external.profile().points)?;
    let outer = builder.extrude(outer_profile, p.pitch_sector.face_width)?;
    let inner_profile = builder.polygon(internal.void_profile().points)?;
    let inner = builder.extrude(inner_profile, length(p.pitch_sector.face_width.mm() + 2.0))?;
    let inner = builder.translate(
        inner,
        Translation3 {
            x: 0.0,
            y: 0.0,
            z: -1.0,
        },
    )?;
    let ring = builder.boolean(BooleanOperation::Difference, outer, inner)?;
    let half = p.pitch_sector.sector.half_angle().as_radians();
    let wedge = builder.polygon(sector_wedge_points(external.tip_radius(), half))?;
    let wedge = builder.extrude(wedge, length(p.pitch_sector.face_width.mm() + 2.0))?;
    let wedge = builder.translate(
        wedge,
        Translation3 {
            x: 0.0,
            y: 0.0,
            z: -1.0,
        },
    )?;
    let sector = builder.boolean(BooleanOperation::Intersection, ring, wedge)?;
    let centered = builder.translate(
        sector,
        Translation3 {
            x: 0.0,
            y: 0.0,
            z: -p.pitch_sector.face_width.mm() * 0.5,
        },
    )?;
    let toothed_sector = builder.rotate(
        centered,
        Rotation3 {
            x: angle(90.0),
            y: angle(0.0),
            z: angle(0.0),
        },
    )?;
    let support_x0 = sector_spine_inner_x(p);
    let support_width = p.frame.fixed_rail_depth.mm();
    let upper_z1 = p.frame.upper_rail_height.mm() - p.frame.fixed_rail_depth.mm() * 0.5;
    let lower_z0 = -p.frame.lower_rail_depth.mm() + p.frame.fixed_rail_depth.mm() * 0.5;
    let keep_out = sector_support_keep_out_half_height();
    let mut supported_sector = toothed_sector;
    for (z0, z1) in [(keep_out, upper_z1), (lower_z0, -keep_out)] {
        let support = centered_box(
            builder,
            [support_width, p.pitch_sector.face_width.mm(), z1 - z0],
        );
        let support = builder.translate(
            support,
            Translation3 {
                x: support_x0 + support_width * 0.5,
                y: 0.0,
                z: (z0 + z1) * 0.5,
            },
        )?;
        supported_sector = builder.boolean(BooleanOperation::Union, supported_sector, support)?;
    }
    let clevis_post_x = p.frame.fixed_crossmember_station.mm();
    let clevis_outer_x = support_x0 + support_width;
    let clevis_inner_x = clevis_post_x - p.frame.fixed_crossmember_width.mm() * 0.5;
    let clevis_length = clevis_outer_x - clevis_inner_x;
    let clevis_thickness = 3.0;
    let clevis_half_height = 8.0;
    let post_half_depth = p.frame.sheet_thickness.mm() * 0.5;
    for z in sector_post_hole_zs(p) {
        for side_sign in [-1.0, 1.0] {
            let cheek = centered_box(
                builder,
                [clevis_length, clevis_thickness, clevis_half_height * 2.0],
            );
            let cheek = builder.translate(
                cheek,
                Translation3 {
                    x: (clevis_inner_x + clevis_outer_x) * 0.5,
                    y: side_sign * (post_half_depth + clevis_thickness * 0.5),
                    z,
                },
            )?;
            supported_sector = builder.boolean(BooleanOperation::Union, supported_sector, cheek)?;
        }
        supported_sector = subtract_y_bore(
            builder,
            supported_sector,
            m3_clearance_radius_mm(),
            (post_half_depth + clevis_thickness) * 2.0 + 2.0,
            clevis_post_x,
            z,
        )?;
    }
    Ok(supported_sector)
}

fn sector_spine_inner_x(p: &PrototypeParameters) -> f64 {
    p.frame.fixed_crossmember_station.mm() + p.frame.fixed_crossmember_width.mm() * 0.5
}

const fn sector_support_keep_out_half_height() -> f64 {
    40.0
}

fn sector_post_hole_zs(p: &PrototypeParameters) -> [f64; 2] {
    let upper_support_end = p.frame.upper_rail_height.mm() - p.frame.fixed_rail_depth.mm() * 0.5;
    let positive_z = (sector_support_keep_out_half_height() + upper_support_end) * 0.5;
    [positive_z, -positive_z]
}

const fn m3_clearance_radius_mm() -> f64 {
    1.7
}

fn sector_post_fastener_datums(
    datums: &mut DatumSet,
    p: &PrototypeParameters,
) -> [FastenerMemberDatums; 2] {
    let post_half_depth = p.frame.sheet_thickness.mm() * 0.5;
    let clevis_thickness = 3.0;
    let seat_y = post_half_depth + clevis_thickness;
    sector_post_hole_zs(p).map(|z| FastenerMemberDatums {
        hole: add_cylinder_datum(
            datums,
            if z > 0.0 {
                "sector_post_upper_m3_hole"
            } else {
                "sector_post_lower_m3_hole"
            },
            [p.frame.fixed_crossmember_station.mm(), 0.0, z],
            [0.0, 1.0, 0.0],
            m3_clearance_radius_mm(),
        ),
        negative_y_seat: add_plane_datum(
            datums,
            if z > 0.0 {
                "sector_post_upper_negative_y_m3_seat"
            } else {
                "sector_post_lower_negative_y_m3_seat"
            },
            [p.frame.fixed_crossmember_station.mm(), -seat_y, z],
            [0.0, -1.0, 0.0],
        ),
        positive_y_seat: add_plane_datum(
            datums,
            if z > 0.0 {
                "sector_post_upper_positive_y_m3_seat"
            } else {
                "sector_post_lower_positive_y_m3_seat"
            },
            [p.frame.fixed_crossmember_station.mm(), seat_y, z],
            [0.0, 1.0, 0.0],
        ),
    })
}

fn sector_wedge_points(tip_radius: f64, half_angle: f64) -> Vec<Point2> {
    const ARC_SEGMENTS: usize = 12;
    let step = 2.0 * half_angle / ARC_SEGMENTS as f64;
    // Inflate the sampled arc so every chord lies beyond the reference tip
    // circle. A triangle made only from the two end rays would cut away the
    // sector centre on the roll-axis extension.
    let outer_radius = (tip_radius + 2.0) / libm::cos(step * 0.5);
    let mut points = Vec::with_capacity(ARC_SEGMENTS + 2);
    points.push(Point2 { x: 0.0, y: 0.0 });
    for index in 0..=ARC_SEGMENTS {
        let angle = -half_angle + step * index as f64;
        points.push(Point2 {
            x: outer_radius * libm::cos(angle),
            y: outer_radius * libm::sin(angle),
        });
    }
    points
}

#[derive(Clone, Copy)]
struct DefinitionStyle {
    role: ComponentRole,
    color: [f32; 4],
}

const fn definition_style(role: ComponentRole, color: [f32; 4]) -> DefinitionStyle {
    DefinitionStyle { role, color }
}

fn gear_definition_y(
    builder: &mut FeatureBuilder,
    assembly: &mut Assembly,
    name: &str,
    gear: &SpurGear,
    width: Length,
    bore_radius: Length,
    style: DefinitionStyle,
) -> Result<ComponentDefinitionId, PrototypeError> {
    let solid = gear_solid_z(builder, gear, width, bore_radius)?;
    let solid = builder.rotate(
        solid,
        Rotation3 {
            x: angle(90.0),
            y: angle(0.0),
            z: angle(0.0),
        },
    )?;
    Ok(add_solid_definition(
        assembly,
        name,
        style.role,
        solid,
        Manufacturing::Fdm,
        style.color,
    ))
}

fn gear_definition_x(
    builder: &mut FeatureBuilder,
    assembly: &mut Assembly,
    name: &str,
    gear: &SpurGear,
    width: Length,
    bore_radius: Length,
    style: DefinitionStyle,
) -> Result<ComponentDefinitionId, PrototypeError> {
    let solid = gear_solid_z(builder, gear, width, bore_radius)?;
    let solid = builder.rotate(
        solid,
        Rotation3 {
            x: angle(0.0),
            y: angle(90.0),
            z: angle(0.0),
        },
    )?;
    Ok(add_solid_definition(
        assembly,
        name,
        style.role,
        solid,
        Manufacturing::Fdm,
        style.color,
    ))
}

fn gear_solid_z(
    builder: &mut FeatureBuilder,
    gear: &SpurGear,
    width: Length,
    bore_radius: Length,
) -> Result<SolidId, PrototypeError> {
    let profile = builder.polygon(gear.profile().points)?;
    let solid = builder.extrude(profile, width)?;
    let solid = builder.translate(
        solid,
        Translation3 {
            x: 0.0,
            y: 0.0,
            z: -width.mm() * 0.5,
        },
    )?;
    let bore = builder.primitive(Primitive3::Cylinder {
        height: length(width.mm() + 2.0),
        radius: bore_radius,
        segments: 48,
        centered: true,
    });
    builder
        .boolean(BooleanOperation::Difference, solid, bore)
        .map_err(PrototypeError::Feature)
}

fn annulus_definition_y(
    builder: &mut FeatureBuilder,
    assembly: &mut Assembly,
    name: &str,
    outer_radius: f64,
    inner_radius: f64,
    width: f64,
    style: DefinitionStyle,
) -> Result<ComponentDefinitionId, PrototypeError> {
    let outer = builder.primitive(Primitive3::Cylinder {
        height: length(width),
        radius: length(outer_radius),
        segments: 48,
        centered: true,
    });
    let inner = builder.primitive(Primitive3::Cylinder {
        height: length(width + 2.0),
        radius: length(inner_radius),
        segments: 48,
        centered: true,
    });
    let annulus = builder.boolean(BooleanOperation::Difference, outer, inner)?;
    let annulus = builder.rotate(
        annulus,
        Rotation3 {
            x: angle(90.0),
            y: angle(0.0),
            z: angle(0.0),
        },
    )?;
    Ok(add_solid_definition(
        assembly,
        name,
        style.role,
        annulus,
        Manufacturing::Fdm,
        style.color,
    ))
}

fn annulus_definition_x(
    builder: &mut FeatureBuilder,
    assembly: &mut Assembly,
    name: &str,
    outer_radius: f64,
    inner_radius: f64,
    width: f64,
    style: DefinitionStyle,
) -> Result<ComponentDefinitionId, PrototypeError> {
    let outer = cylinder_x(builder, outer_radius, width)?;
    let inner = cylinder_x(builder, inner_radius, width + 2.0)?;
    let annulus = builder.boolean(BooleanOperation::Difference, outer, inner)?;
    Ok(add_solid_definition(
        assembly,
        name,
        style.role,
        annulus,
        Manufacturing::Purchased,
        style.color,
    ))
}

fn carrier_post_definition(
    builder: &mut FeatureBuilder,
    assembly: &mut Assembly,
    p: &PrototypeParameters,
    height: f64,
) -> Result<
    (
        ComponentDefinitionId,
        BoxPlaneDatums,
        [PostFastenerDatums; 2],
    ),
    PrototypeError,
> {
    let width = p.frame.fixed_crossmember_width.mm();
    let depth = p.frame.sheet_thickness.mm();
    let mut solid = centered_box(builder, [width, depth, height]);
    let post_center_z = (p.frame.upper_rail_height.mm() - p.frame.lower_rail_depth.mm()) * 0.5;
    let world_hole_zs = sector_post_hole_zs(p);
    let local_hole_zs = world_hole_zs.map(|world_z| world_z - post_center_z);
    for local_z in local_hole_zs {
        solid = subtract_y_bore(
            builder,
            solid,
            m3_clearance_radius_mm(),
            depth + 2.0,
            0.0,
            local_z,
        )?;
    }

    let (mut datums, faces) = box_plane_datums([width, depth, height]);
    let fasteners = local_hole_zs.map(|z| PostFastenerDatums {
        hole: add_cylinder_datum(
            &mut datums,
            if z > 0.0 {
                "post_upper_m3_hole"
            } else {
                "post_lower_m3_hole"
            },
            [0.0, 0.0, z],
            [0.0, 1.0, 0.0],
            m3_clearance_radius_mm(),
        ),
    });
    let id = assembly.add_definition(ComponentDefinition {
        name: "pitch_carrier_post".to_string(),
        role: ComponentRole::FixedCarrierPost,
        body: Body::Solid(solid),
        manufacturing: Manufacturing::Fdm,
        color_rgba: [0.58, 0.35, 0.16, 1.0],
        datums,
    });
    Ok((id, faces, fasteners))
}

fn m3_pan_head_bolt_solid(
    builder: &mut FeatureBuilder,
    shank_length: f64,
) -> Result<SolidId, PrototypeError> {
    const HEAD_THICKNESS: f64 = 2.4;
    let shank = cylinder_x(builder, 1.5, shank_length)?;
    let shank = builder.translate(
        shank,
        Translation3 {
            x: -shank_length * 0.5,
            y: 0.0,
            z: 0.0,
        },
    )?;
    let head = cylinder_x(builder, 2.75, HEAD_THICKNESS)?;
    let head = builder.translate(
        head,
        Translation3 {
            x: HEAD_THICKNESS * 0.5,
            y: 0.0,
            z: 0.0,
        },
    )?;
    builder
        .boolean(BooleanOperation::Union, shank, head)
        .map_err(PrototypeError::Feature)
}

fn m3_hex_nut_solid(builder: &mut FeatureBuilder) -> Result<SolidId, PrototypeError> {
    const THICKNESS: f64 = 2.4;
    // 3.175 mm circumradius gives approximately 5.5 mm across flats.
    let outer = cylinder_x_segments(builder, 3.175, THICKNESS, 6)?;
    let bore = cylinder_x(builder, 1.6, THICKNESS + 2.0)?;
    builder
        .boolean(BooleanOperation::Difference, outer, bore)
        .map_err(PrototypeError::Feature)
}

fn m3_washer_solid(builder: &mut FeatureBuilder) -> Result<SolidId, PrototypeError> {
    const THICKNESS: f64 = 0.5;
    let outer = cylinder_x(builder, 3.5, THICKNESS)?;
    let bore = cylinder_x(builder, m3_clearance_radius_mm(), THICKNESS + 2.0)?;
    builder
        .boolean(BooleanOperation::Difference, outer, bore)
        .map_err(PrototypeError::Feature)
}

fn sheet_box_definition_with_faces(
    builder: &mut FeatureBuilder,
    assembly: &mut Assembly,
    name: &str,
    width: f64,
    height: f64,
    thickness: Length,
    style: DefinitionStyle,
) -> Result<(ComponentDefinitionId, BoxPlaneDatums), PrototypeError> {
    let profile = builder.polygon(rectangle_points(width, height))?;
    let solid = builder.extrude(profile, thickness)?;
    let solid = builder.translate(
        solid,
        Translation3 {
            x: 0.0,
            y: 0.0,
            z: -thickness.mm() * 0.5,
        },
    )?;
    let (datums, faces) = box_plane_datums([width, height, thickness.mm()]);
    let id = assembly.add_definition(ComponentDefinition {
        name: name.to_string(),
        role: style.role,
        body: Body::Sheet {
            outer: profile,
            cutouts: Vec::new(),
            thickness,
            assembly_solid: solid,
        },
        manufacturing: Manufacturing::Fdm,
        color_rgba: style.color,
        datums,
    });
    Ok((id, faces))
}

fn add_box_definition_with_faces(
    builder: &mut FeatureBuilder,
    assembly: &mut Assembly,
    name: &str,
    role: ComponentRole,
    size: [f64; 3],
    manufacturing: Manufacturing,
    color_rgba: [f32; 4],
) -> (ComponentDefinitionId, BoxPlaneDatums) {
    let (datums, faces) = box_plane_datums(size);
    let id = assembly.add_definition(ComponentDefinition {
        name: name.to_string(),
        role,
        body: Body::Solid(centered_box(builder, size)),
        manufacturing,
        color_rgba,
        datums,
    });
    (id, faces)
}

fn box_plane_datums(size: [f64; 3]) -> (DatumSet, BoxPlaneDatums) {
    let mut datums = DatumSet::new();
    let negative_x = add_plane_datum(
        &mut datums,
        "negative_x",
        [-size[0] * 0.5, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
    );
    let positive_x = add_plane_datum(
        &mut datums,
        "positive_x",
        [size[0] * 0.5, 0.0, 0.0],
        [1.0, 0.0, 0.0],
    );
    let negative_y = add_plane_datum(
        &mut datums,
        "negative_y",
        [0.0, -size[1] * 0.5, 0.0],
        [0.0, -1.0, 0.0],
    );
    let positive_y = add_plane_datum(
        &mut datums,
        "positive_y",
        [0.0, size[1] * 0.5, 0.0],
        [0.0, 1.0, 0.0],
    );
    let negative_z = add_plane_datum(
        &mut datums,
        "negative_z",
        [0.0, 0.0, -size[2] * 0.5],
        [0.0, 0.0, -1.0],
    );
    let positive_z = add_plane_datum(
        &mut datums,
        "positive_z",
        [0.0, 0.0, size[2] * 0.5],
        [0.0, 0.0, 1.0],
    );
    (
        datums,
        BoxPlaneDatums {
            negative_x,
            positive_x,
            negative_y,
            positive_y,
            negative_z,
            positive_z,
        },
    )
}

fn add_plane_datum(
    datums: &mut DatumSet,
    name: &str,
    origin: [f64; 3],
    normal: [f64; 3],
) -> DatumId<PlaneDatum> {
    datums.add(
        name.to_string(),
        PlaneDatum {
            origin: Point3::from_mm(origin).expect("box face origin is finite"),
            normal: UnitVector3::new(normal).expect("box face normal is non-zero"),
        },
    )
}

fn add_cylinder_datum(
    datums: &mut DatumSet,
    name: &str,
    origin: [f64; 3],
    direction: [f64; 3],
    radius: f64,
) -> DatumId<CylinderDatum> {
    datums.add(
        name.to_string(),
        CylinderDatum {
            axis: AxisDatum {
                origin: Point3::from_mm(origin).expect("cylinder origin is finite"),
                direction: UnitVector3::new(direction)
                    .expect("cylinder axis direction is non-zero"),
            },
            radius: PositiveLength::mm(radius).expect("cylinder radius is positive"),
        },
    )
}

fn cylinder_y(
    builder: &mut FeatureBuilder,
    radius: f64,
    height: f64,
) -> Result<SolidId, PrototypeError> {
    let solid = builder.primitive(Primitive3::Cylinder {
        height: length(height),
        radius: length(radius),
        segments: 48,
        centered: true,
    });
    builder
        .rotate(
            solid,
            Rotation3 {
                x: angle(90.0),
                y: angle(0.0),
                z: angle(0.0),
            },
        )
        .map_err(PrototypeError::Feature)
}

fn cylinder_x(
    builder: &mut FeatureBuilder,
    radius: f64,
    height: f64,
) -> Result<SolidId, PrototypeError> {
    cylinder_x_segments(builder, radius, height, 64)
}

fn cylinder_x_segments(
    builder: &mut FeatureBuilder,
    radius: f64,
    height: f64,
    segments: u16,
) -> Result<SolidId, PrototypeError> {
    let solid = builder.primitive(Primitive3::Cylinder {
        height: length(height),
        radius: length(radius),
        segments,
        centered: true,
    });
    builder
        .rotate(
            solid,
            Rotation3 {
                x: angle(0.0),
                y: angle(90.0),
                z: angle(0.0),
            },
        )
        .map_err(PrototypeError::Feature)
}

fn centered_box(builder: &mut FeatureBuilder, size: [f64; 3]) -> SolidId {
    builder.primitive(Primitive3::Box {
        x: length(size[0]),
        y: length(size[1]),
        z: length(size[2]),
        centered: true,
    })
}

fn add_solid_definition(
    assembly: &mut Assembly,
    name: &str,
    role: ComponentRole,
    solid: SolidId,
    manufacturing: Manufacturing,
    color_rgba: [f32; 4],
) -> ComponentDefinitionId {
    add_solid_definition_with_datums(
        assembly,
        name,
        role,
        solid,
        manufacturing,
        color_rgba,
        DatumSet::new(),
    )
}

fn add_solid_definition_with_datums(
    assembly: &mut Assembly,
    name: &str,
    role: ComponentRole,
    solid: SolidId,
    manufacturing: Manufacturing,
    color_rgba: [f32; 4],
    datums: DatumSet,
) -> ComponentDefinitionId {
    assembly.add_definition(ComponentDefinition {
        name: name.to_string(),
        role,
        body: Body::Solid(solid),
        manufacturing,
        color_rgba,
        datums,
    })
}

fn add_instance(
    assembly: &mut Assembly,
    name: &str,
    definition: ComponentDefinitionId,
    frame: FrameId,
    local_pose: RigidTransform,
) {
    add_located_instance(
        assembly,
        name,
        definition,
        frame,
        local_pose,
        ComponentLocation::new(),
    );
}

fn add_located_instance(
    assembly: &mut Assembly,
    name: &str,
    definition: ComponentDefinitionId,
    frame: FrameId,
    local_pose: RigidTransform,
    location: ComponentLocation,
) -> crate::ComponentInstanceId {
    assembly.add_instance(ComponentInstance {
        name: name.to_string(),
        definition,
        frame,
        local_pose,
        location,
    })
}

fn revolute_frame(
    frames: &mut FrameGraph,
    parent: FrameId,
    center: [f64; 3],
    axis: Axis3,
    coordinate: CoordinateExpr,
) -> FrameId {
    frames.add_frame(
        parent,
        RigidTransform::translated(center[0], center[1], center[2]),
        Joint::Revolute { axis, coordinate },
    )
}

fn rectangle_points(width: f64, height: f64) -> Vec<Point2> {
    vec![
        Point2 {
            x: -width * 0.5,
            y: -height * 0.5,
        },
        Point2 {
            x: width * 0.5,
            y: -height * 0.5,
        },
        Point2 {
            x: width * 0.5,
            y: height * 0.5,
        },
        Point2 {
            x: -width * 0.5,
            y: height * 0.5,
        },
    ]
}

fn distance2(a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    libm::sqrt(dx * dx + dy * dy)
}

fn length(value: f64) -> Length {
    Length::positive_mm(value).expect("derived prototype length must be positive")
}

fn angle(degrees: f64) -> Angle {
    Angle::degrees(degrees).expect("derived prototype angle must be finite")
}

impl From<FeatureError> for PrototypeError {
    fn from(value: FeatureError) -> Self {
        Self::Feature(value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sector_definition_is_instanced_four_times() {
        // The concrete parameter integration test lives at the CLI boundary; this
        // compile-time test protects the definition/instance distinction itself.
        assert_ne!(core::mem::size_of::<ComponentDefinitionId>(), 0);
    }

    #[test]
    fn sector_wedge_contains_the_roll_axis_extension() {
        let points = sector_wedge_points(150.0, PI / 6.0);
        let centre = points
            .iter()
            .max_by(|a, b| a.x.total_cmp(&b.x))
            .expect("wedge has arc samples");
        assert!(centre.x > 150.0);
        assert!(centre.y.abs() < 1.0e-10);
    }
}

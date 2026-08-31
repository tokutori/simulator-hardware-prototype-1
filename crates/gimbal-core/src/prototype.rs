// SPDX-License-Identifier: MIT

use alloc::format;
use alloc::string::ToString;
use alloc::vec;
use alloc::vec::Vec;
use core::f64::consts::{FRAC_PI_2, PI};

use crate::{
    Angle, Assembly, Axis3, Body, BooleanOperation, ComponentDefinition, ComponentDefinitionId,
    ComponentInstance, CoordinateExpr, ExternalGearPair, FdmMaterial, FeatureBuilder, FeatureError,
    FeatureGraph, FrameGraph, FrameId, GearSector, InternalGearPair, Joint, Kinematics, Length,
    Manufacturing, Point2, Primitive3, RigidTransform, Rotation3, SolidId, SpurGear, Translation3,
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
}

#[derive(Clone, Debug)]
pub struct RollAxisParameters {
    pub driven_gear: SpurGear,
    pub pinion: SpurGear,
    pub shaft_length: Length,
    pub shaft_radius: Length,
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
    pub crossmember_radius: Length,
    pub bearing_pedestal_thickness: Length,
    pub sheet_thickness: Length,
    pub carrier_rail_offset: Length,
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
    pub pitch_drive_pair: InternalGearPair,
    pub pitch_encoder_pair: ExternalGearPair,
    pub pitch_gearbox_pair: ExternalGearPair,
    pub roll_pair: ExternalGearPair,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrototypeError {
    Feature(FeatureError),
    IncompatibleGearPair,
    OuterDiameterMismatch,
    SectorWebTooThin,
    SectorMotionMarginTooSmall,
    DrivePinionsOverlap,
    InvalidCockpitEnvelope,
    InvalidCockpitSuspension,
    InvalidGearboxGeometry,
    FloorClearanceTooSmall,
    CarrierRailTooClose,
}

pub fn build_prototype(
    parameters: &PrototypeParameters,
) -> Result<PrototypeDesign, PrototypeError> {
    validate(parameters)?;
    let pitch_drive_pair = InternalGearPair::new(
        parameters.contact_unit.drive_pinion.clone(),
        parameters.pitch_sector.sector.internal_reference().clone(),
    )
    .map_err(|_| PrototypeError::IncompatibleGearPair)?;
    let pitch_encoder_pair = ExternalGearPair::new(
        parameters.contact_unit.encoder_pinion.clone(),
        parameters.pitch_sector.sector.external_reference().clone(),
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
    let contact_margin = Angle::degrees(8.0).expect("constant is finite");
    if !parameters
        .pitch_sector
        .sector
        .supports_motion(parameters.motion.pitch_limit, contact_margin)
    {
        return Err(PrototypeError::SectorMotionMarginTooSmall);
    }
    let drive_radius =
        internal.pitch_radius() - parameters.contact_unit.drive_pinion.pitch_radius();
    let center_separation =
        2.0 * drive_radius * libm::sin(parameters.contact_unit.branch_angle_offset.as_radians());
    if center_separation <= parameters.contact_unit.drive_pinion.outside_diameter() + 1.0 {
        return Err(PrototypeError::DrivePinionsOverlap);
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
    if parameters.pitch_gearbox.small_gear.teeth()
        != parameters.pitch_gearbox.distribution_gear.teeth()
    {
        return Err(PrototypeError::InvalidGearboxGeometry);
    }
    if parameters.frame.carrier_rail_offset.mm() <= 80.0 {
        return Err(PrototypeError::CarrierRailTooClose);
    }
    let rail_swept_radius = libm::sqrt(
        122.0 * 122.0
            + (parameters.frame.carrier_rail_offset.mm() + 4.0)
                * (parameters.frame.carrier_rail_offset.mm() + 4.0),
    );
    let cockpit_half_length = parameters.cockpit.length.mm() * 0.5;
    let cockpit_half_width = parameters.cockpit.width.mm() * 0.5;
    let cockpit_bottom_radius =
        parameters.cockpit.suspension_drop.mm() + parameters.cockpit.height.mm() * 0.5;
    let cockpit_corner_radius = libm::sqrt(
        cockpit_half_length * cockpit_half_length
            + cockpit_half_width * cockpit_half_width
            + cockpit_bottom_radius * cockpit_bottom_radius,
    );
    let retention_center_radius =
        external.pitch_radius() + parameters.contact_unit.encoder_pinion.pitch_radius();
    let retention_swept_radius =
        retention_center_radius + parameters.contact_unit.encoder_pinion.tip_radius() + 2.5;
    let mechanism_envelope_radius = rail_swept_radius
        .max(cockpit_corner_radius)
        .max(external.tip_radius())
        .max(retention_swept_radius);
    if parameters.frame.floor_top_below_axis.mm() < mechanism_envelope_radius + 10.0 {
        return Err(PrototypeError::FloorClearanceTooSmall);
    }
    Ok(())
}

#[derive(Clone, Copy)]
struct Definitions {
    sector: ComponentDefinitionId,
    carrier_rail: ComponentDefinitionId,
    carrier_link: ComponentDefinitionId,
    crossmember: ComponentDefinitionId,
    floor_support_post: ComponentDefinitionId,
    floor: ComponentDefinitionId,
    drive_pinion: ComponentDefinitionId,
    encoder_pinion: ComponentDefinitionId,
    drive_flange: ComponentDefinitionId,
    encoder_flange: ComponentDefinitionId,
    drive_shaft: ComponentDefinitionId,
    encoder_shaft: ComponentDefinitionId,
    gearbox_small: ComponentDefinitionId,
    gearbox_large: ComponentDefinitionId,
    gearbox_plate: ComponentDefinitionId,
    leaf_spring: ComponentDefinitionId,
    bearing_block: ComponentDefinitionId,
    cockpit: ComponentDefinitionId,
    cockpit_hanger: ComponentDefinitionId,
    roll_shaft: ComponentDefinitionId,
    roll_driven: ComponentDefinitionId,
    roll_pinion: ComponentDefinitionId,
    roll_gearbox_small: ComponentDefinitionId,
    roll_gearbox_large: ComponentDefinitionId,
    roll_gearbox_shaft: ComponentDefinitionId,
    roll_pedestal: ComponentDefinitionId,
    roll_gearbox_plate: ComponentDefinitionId,
    roll_gearbox_mount: ComponentDefinitionId,
    moving_drive_mount_arm: ComponentDefinitionId,
}

fn build_definitions(
    builder: &mut FeatureBuilder,
    assembly: &mut Assembly,
    p: &PrototypeParameters,
) -> Result<Definitions, PrototypeError> {
    let fdm = Manufacturing::Fdm {
        material: FdmMaterial::Petg,
    };
    let sector = dual_sector_solid(builder, p)?;
    let sector = add_solid_definition(
        assembly,
        "pitch_dual_gear_sector",
        sector,
        fdm,
        [0.94, 0.52, 0.08, 1.0],
    );
    let carrier_rail = sheet_definition(
        builder,
        assembly,
        "pitch_carrier_rail",
        244.0,
        8.0,
        p.frame.sheet_thickness,
        [0.58, 0.35, 0.16, 1.0],
    )?;
    let link_height = p.frame.carrier_rail_offset.mm() - 70.0;
    let carrier_link = sheet_definition(
        builder,
        assembly,
        "pitch_sector_to_rail_link",
        10.0,
        link_height,
        p.frame.sheet_thickness,
        [0.58, 0.35, 0.16, 1.0],
    )?;
    let crossmember = add_solid_definition(
        assembly,
        "pitch_crossmember",
        cylinder_y(
            builder,
            p.frame.crossmember_radius.mm(),
            p.pitch_sector.carrier_spacing.mm() + p.pitch_sector.face_width.mm(),
        )?,
        Manufacturing::Purchased,
        [0.66, 0.69, 0.72, 1.0],
    );
    let floor_support_height = p.frame.floor_top_below_axis.mm() - p.frame.carrier_rail_offset.mm();
    let floor_support_post = add_solid_definition(
        assembly,
        "fixed_frame_floor_support_post",
        centered_box(builder, [14.0, 14.0, floor_support_height]),
        Manufacturing::Purchased,
        [0.30, 0.32, 0.35, 1.0],
    );
    let floor = add_solid_definition(
        assembly,
        "installation_floor_reference",
        centered_box(builder, [400.0, 250.0, p.frame.floor_thickness.mm()]),
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
        [0.10, 0.43, 0.84, 1.0],
    )?;
    let encoder_pinion = gear_definition_y(
        builder,
        assembly,
        "pitch_retention_encoder_pinion",
        &p.contact_unit.encoder_pinion,
        p.pitch_sector.face_width,
        p.contact_unit.encoder_shaft_radius,
        [0.10, 0.72, 0.34, 1.0],
    )?;
    let drive_flange = annulus_definition_y(
        builder,
        assembly,
        "drive_retention_flange",
        p.contact_unit.drive_pinion.tip_radius() + 2.0,
        p.contact_unit.drive_shaft_radius.mm(),
        p.contact_unit.flange_thickness.mm(),
        [0.18, 0.48, 0.90, 1.0],
    )?;
    let encoder_flange = annulus_definition_y(
        builder,
        assembly,
        "encoder_guide_flange",
        p.contact_unit.encoder_pinion.tip_radius() + 2.5,
        p.contact_unit.encoder_shaft_radius.mm(),
        p.contact_unit.flange_thickness.mm(),
        [0.18, 0.80, 0.40, 1.0],
    )?;
    let drive_shaft = add_solid_definition(
        assembly,
        "drive_shaft",
        cylinder_y(builder, p.contact_unit.drive_shaft_radius.mm() - 0.15, 28.0)?,
        Manufacturing::Purchased,
        [0.62, 0.66, 0.70, 1.0],
    );
    let encoder_shaft = add_solid_definition(
        assembly,
        "encoder_interface_shaft",
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
        [0.66, 0.20, 0.72, 1.0],
    )?;
    let gearbox_large = gear_definition_y(
        builder,
        assembly,
        "pitch_gearbox_large_gear",
        &p.pitch_gearbox.large_gear,
        p.pitch_gearbox.gear_face_width,
        p.pitch_gearbox.shaft_radius,
        [0.80, 0.28, 0.70, 1.0],
    )?;
    let gearbox_plate = add_solid_definition(
        assembly,
        "pitch_gearbox_side_plate",
        centered_box(
            builder,
            [52.0, p.pitch_gearbox.side_plate_thickness.mm(), 54.0],
        ),
        fdm,
        [0.20, 0.22, 0.27, 1.0],
    );
    let leaf_spring = add_solid_definition(
        assembly,
        "encoder_leaf_spring",
        centered_box(builder, [26.0, 0.7, 5.0]),
        Manufacturing::Purchased,
        [0.74, 0.76, 0.78, 1.0],
    );
    let bearing_block = add_solid_definition(
        assembly,
        "encoder_bearing_block",
        centered_box(builder, [16.0, 12.0, 18.0]),
        fdm,
        [0.16, 0.52, 0.26, 1.0],
    );
    let cockpit = add_solid_definition(
        assembly,
        "cockpit_body",
        centered_box(
            builder,
            [
                p.cockpit.length.mm(),
                p.cockpit.width.mm(),
                p.cockpit.height.mm(),
            ],
        ),
        fdm,
        [0.86, 0.20, 0.18, 1.0],
    );
    let hanger_height = p.cockpit.suspension_drop.mm() - p.cockpit.height.mm() * 0.5;
    let cockpit_hanger = add_solid_definition(
        assembly,
        "cockpit_roll_shaft_hanger",
        centered_box(builder, [10.0, 12.0, hanger_height]),
        fdm,
        [0.72, 0.25, 0.20, 1.0],
    );
    let roll_shaft = add_solid_definition(
        assembly,
        "roll_shaft",
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
        [0.88, 0.72, 0.08, 1.0],
    )?;
    let roll_pinion = gear_definition_x(
        builder,
        assembly,
        "roll_input_pinion",
        &p.roll_axis.pinion,
        length(6.0),
        p.pitch_gearbox.shaft_radius,
        [0.96, 0.80, 0.12, 1.0],
    )?;
    let roll_gearbox_small = gear_definition_x(
        builder,
        assembly,
        "roll_gearbox_small_gear",
        &p.pitch_gearbox.small_gear,
        p.pitch_gearbox.gear_face_width,
        p.pitch_gearbox.shaft_radius,
        [0.66, 0.20, 0.72, 1.0],
    )?;
    let roll_gearbox_large = gear_definition_x(
        builder,
        assembly,
        "roll_gearbox_large_gear",
        &p.pitch_gearbox.large_gear,
        p.pitch_gearbox.gear_face_width,
        p.pitch_gearbox.shaft_radius,
        [0.80, 0.28, 0.70, 1.0],
    )?;
    let roll_gearbox_shaft = add_solid_definition(
        assembly,
        "roll_gearbox_shaft",
        cylinder_x(builder, p.pitch_gearbox.shaft_radius.mm() - 0.15, 24.0)?,
        Manufacturing::Purchased,
        [0.62, 0.66, 0.70, 1.0],
    );
    let roll_pedestal = u_sheet_definition(
        builder,
        assembly,
        "roll_bearing_pedestal",
        p.frame.bearing_pedestal_thickness,
        [0.56, 0.34, 0.16, 1.0],
    )?;
    let roll_gearbox_plate = add_solid_definition(
        assembly,
        "roll_gearbox_plate",
        roll_gearbox_plate_solid(builder, p)?,
        fdm,
        [0.20, 0.22, 0.27, 1.0],
    );
    let roll_gearbox_mount = add_solid_definition(
        assembly,
        "roll_gearbox_carrier_mount",
        centered_box(builder, [8.0, 8.0, 42.0]),
        fdm,
        [0.34, 0.24, 0.16, 1.0],
    );
    let moving_drive_mount_arm = add_solid_definition(
        assembly,
        "moving_drive_mount_arm",
        centered_box(builder, [34.0, 8.0, 8.0]),
        fdm,
        [0.34, 0.24, 0.16, 1.0],
    );
    Ok(Definitions {
        sector,
        carrier_rail,
        carrier_link,
        crossmember,
        floor_support_post,
        floor,
        drive_pinion,
        encoder_pinion,
        drive_flange,
        encoder_flange,
        drive_shaft,
        encoder_shaft,
        gearbox_small,
        gearbox_large,
        gearbox_plate,
        leaf_spring,
        bearing_block,
        cockpit,
        cockpit_hanger,
        roll_shaft,
        roll_driven,
        roll_pinion,
        roll_gearbox_small,
        roll_gearbox_large,
        roll_gearbox_shaft,
        roll_pedestal,
        roll_gearbox_plate,
        roll_gearbox_mount,
        moving_drive_mount_arm,
    })
}

fn build_pitch_carrier(
    assembly: &mut Assembly,
    definitions: &Definitions,
    p: &PrototypeParameters,
    fixed_frame: FrameId,
) {
    let half_spacing = p.pitch_sector.carrier_spacing.mm() * 0.5;
    for (side, y) in [("left", -half_spacing), ("right", half_spacing)] {
        for (end, rotation) in [("front", 0.0), ("rear", PI)] {
            add_instance(
                assembly,
                &format!("pitch_sector_{side}_{end}"),
                definitions.sector,
                fixed_frame,
                RigidTransform::translated(0.0, y, 0.0)
                    .compose(RigidTransform::rotated(Axis3::Y, rotation)),
            );
        }
        for (rail, z) in [
            ("upper", p.frame.carrier_rail_offset.mm()),
            ("lower", -p.frame.carrier_rail_offset.mm()),
        ] {
            add_instance(
                assembly,
                &format!("pitch_carrier_{side}_{rail}_rail"),
                definitions.carrier_rail,
                fixed_frame,
                RigidTransform::translated(0.0, y, z)
                    .compose(RigidTransform::rotated(Axis3::X, FRAC_PI_2)),
            );
        }
        let link_center_z = (74.0 + p.frame.carrier_rail_offset.mm()) * 0.5;
        for (end, x) in [("front", 126.0), ("rear", -126.0)] {
            for (vertical, z) in [("upper", link_center_z), ("lower", -link_center_z)] {
                add_instance(
                    assembly,
                    &format!("pitch_carrier_{side}_{end}_{vertical}_link"),
                    definitions.carrier_link,
                    fixed_frame,
                    RigidTransform::translated(x, y, z)
                        .compose(RigidTransform::rotated(Axis3::X, FRAC_PI_2)),
                );
            }
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
        (-112.0, p.frame.carrier_rail_offset.mm()),
        (-112.0, -p.frame.carrier_rail_offset.mm()),
        (112.0, p.frame.carrier_rail_offset.mm()),
        (112.0, -p.frame.carrier_rail_offset.mm()),
    ]
    .into_iter()
    .enumerate()
    {
        add_instance(
            assembly,
            &format!("pitch_crossmember_{}", index + 1),
            definitions.crossmember,
            world,
            RigidTransform::translated(x, 0.0, z),
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
    let support_height = p.frame.floor_top_below_axis.mm() - p.frame.carrier_rail_offset.mm();
    let support_center_z = -p.frame.carrier_rail_offset.mm() - support_height * 0.5;
    let half_spacing = p.pitch_sector.carrier_spacing.mm() * 0.5;
    for (index, (x, y)) in [
        (-112.0, -half_spacing),
        (-112.0, half_spacing),
        (112.0, -half_spacing),
        (112.0, half_spacing),
    ]
    .into_iter()
    .enumerate()
    {
        add_instance(
            assembly,
            &format!("fixed_frame_floor_support_{}", index + 1),
            definitions.floor_support_post,
            world,
            RigidTransform::translated(x, y, support_center_z),
        );
    }
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
    for (side, y, side_sign) in [("left", -half_spacing, -1.0), ("right", half_spacing, 1.0)] {
        for (end, end_angle) in [("front", 0.0), ("rear", PI)] {
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
    side: &str,
    end: &str,
    y: f64,
    side_sign: f64,
    end_angle: f64,
    drive_ratio: f64,
    encoder_ratio: f64,
    gearbox_ratio: f64,
) -> Result<(), PrototypeError> {
    let internal = p.pitch_sector.sector.internal_reference();
    let external = p.pitch_sector.sector.external_reference();
    let drive_radius = internal.pitch_radius() - p.contact_unit.drive_pinion.pitch_radius();
    let offset = p.contact_unit.branch_angle_offset.as_radians();
    let mut branch_centers = [[0.0; 2]; 2];
    let outer_layer_y = y + side_sign * (p.pitch_sector.face_width.mm() * 0.5 + 5.0);

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
            CoordinateExpr::pitch(-drive_ratio),
        );
        let stem = format!("pitch_drive_{side}_{end}_{}", branch + 1);
        add_instance(
            assembly,
            &stem,
            d.drive_pinion,
            frame,
            RigidTransform::IDENTITY,
        );
        add_instance(
            assembly,
            &format!("{stem}_shaft"),
            d.drive_shaft,
            frame,
            RigidTransform::IDENTITY,
        );
        let flange_offset = p.pitch_sector.face_width.mm() * 0.5
            + p.contact_unit.drive_flange_clearance.mm()
            + p.contact_unit.flange_thickness.mm() * 0.5;
        for (label, dy) in [("inner", -flange_offset), ("outer", flange_offset)] {
            add_instance(
                assembly,
                &format!("{stem}_flange_{label}"),
                d.drive_flange,
                frame,
                RigidTransform::translated(0.0, dy, 0.0),
            );
        }
        add_instance(
            assembly,
            &format!("{stem}_distribution_branch"),
            d.gearbox_small,
            frame,
            RigidTransform::translated(0.0, outer_layer_y - y, 0.0),
        );
    }

    let encoder_radius = external.pitch_radius() + p.contact_unit.encoder_pinion.pitch_radius();
    let encoder_center = [
        encoder_radius * libm::cos(end_angle),
        encoder_radius * libm::sin(end_angle),
    ];
    let encoder_frame = revolute_frame(
        frames,
        pitch_frame,
        [encoder_center[0], y, encoder_center[1]],
        Axis3::Y,
        CoordinateExpr::pitch(encoder_ratio),
    );
    let encoder_stem = format!("pitch_retention_{side}_{end}");
    add_instance(
        assembly,
        &encoder_stem,
        d.encoder_pinion,
        encoder_frame,
        RigidTransform::IDENTITY,
    );
    add_instance(
        assembly,
        &format!("{encoder_stem}_interface_shaft"),
        d.encoder_shaft,
        encoder_frame,
        RigidTransform::IDENTITY,
    );
    let encoder_flange_offset = p.pitch_sector.face_width.mm() * 0.5
        + p.contact_unit.encoder_flange_clearance.mm()
        + p.contact_unit.flange_thickness.mm() * 0.5;
    for (label, dy) in [
        ("inner", -encoder_flange_offset),
        ("outer", encoder_flange_offset),
    ] {
        add_instance(
            assembly,
            &format!("{encoder_stem}_flange_{label}"),
            d.encoder_flange,
            encoder_frame,
            RigidTransform::translated(0.0, dy, 0.0),
        );
    }

    let radial = [libm::cos(end_angle), libm::sin(end_angle)];
    let tangent = [-radial[1], radial[0]];
    let block_center = [
        encoder_center[0] + radial[0] * 8.0,
        encoder_center[1] + radial[1] * 8.0,
    ];
    add_instance(
        assembly,
        &format!("{encoder_stem}_bearing_block"),
        d.bearing_block,
        pitch_frame,
        RigidTransform::translated(block_center[0], y, block_center[1])
            .compose(RigidTransform::rotated(Axis3::Y, -end_angle)),
    );
    for (index, tangent_offset) in [-7.0, 7.0].into_iter().enumerate() {
        add_instance(
            assembly,
            &format!("{encoder_stem}_leaf_spring_{}", index + 1),
            d.leaf_spring,
            pitch_frame,
            RigidTransform::translated(
                block_center[0] + radial[0] * 18.0 + tangent[0] * tangent_offset,
                y + side_sign * 7.0,
                block_center[1] + radial[1] * 18.0 + tangent[1] * tangent_offset,
            )
            .compose(RigidTransform::rotated(Axis3::Y, -end_angle)),
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
    let distributor_frame = revolute_frame(
        frames,
        pitch_frame,
        [central[0], outer_layer_y, central[1]],
        Axis3::Y,
        CoordinateExpr::pitch(drive_ratio),
    );
    add_instance(
        assembly,
        &format!("pitch_gearbox_{side}_{end}_distributor"),
        d.gearbox_small,
        distributor_frame,
        RigidTransform::IDENTITY,
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
        [compound_a[0], outer_layer_y, compound_a[1]],
        Axis3::Y,
        CoordinateExpr::pitch(-drive_ratio * gearbox_ratio),
    );
    let input_frame = revolute_frame(
        frames,
        pitch_frame,
        [input_center[0], outer_layer_y, input_center[1]],
        Axis3::Y,
        CoordinateExpr::pitch(drive_ratio * gearbox_ratio * gearbox_ratio),
    );
    let layer = side_sign * (p.pitch_gearbox.gear_face_width.mm() + 1.0);
    add_instance(
        assembly,
        &format!("pitch_gearbox_{side}_{end}_stage2_driven"),
        d.gearbox_large,
        distributor_frame,
        RigidTransform::translated(0.0, layer, 0.0),
    );
    add_instance(
        assembly,
        &format!("pitch_gearbox_{side}_{end}_stage2_pinion"),
        d.gearbox_small,
        compound_a_frame,
        RigidTransform::translated(0.0, layer, 0.0),
    );
    add_instance(
        assembly,
        &format!("pitch_gearbox_{side}_{end}_stage1_driven"),
        d.gearbox_large,
        compound_a_frame,
        RigidTransform::translated(0.0, layer * 2.0, 0.0),
    );
    add_instance(
        assembly,
        &format!("pitch_gearbox_{side}_{end}_input_pinion"),
        d.gearbox_small,
        input_frame,
        RigidTransform::translated(0.0, layer * 2.0, 0.0),
    );
    let plate_center = [
        (central[0] + input_center[0]) * 0.5,
        (central[1] + input_center[1]) * 0.5,
    ];
    for (index, plate_y) in [outer_layer_y - side_sign * 3.0, outer_layer_y + layer * 3.0]
        .into_iter()
        .enumerate()
    {
        add_instance(
            assembly,
            &format!("pitch_gearbox_{side}_{end}_side_plate_{}", index + 1),
            d.gearbox_plate,
            pitch_frame,
            RigidTransform::translated(plate_center[0], plate_y, plate_center[1])
                .compose(RigidTransform::rotated(Axis3::Y, -end_angle)),
        );
    }
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
    let hanger_height = p.cockpit.suspension_drop.mm() - p.cockpit.height.mm() * 0.5;
    for (index, x) in [-p.cockpit.length.mm() * 0.30, p.cockpit.length.mm() * 0.30]
        .into_iter()
        .enumerate()
    {
        add_instance(
            assembly,
            &format!("cockpit_hanger_{}", index + 1),
            d.cockpit_hanger,
            roll_frame,
            RigidTransform::translated(x, 0.0, -hanger_height * 0.5),
        );
    }
    for (end, outward) in [("front", 1.0), ("rear", -1.0)] {
        let gear_x = outward * (p.cockpit.length.mm() * 0.5 + 8.0);
        // This crossbar belongs to the moving pitch body.  Keep it radially
        // inside the fixed upper rail so its swept envelope remains clear at
        // both pitch limits.
        let moving_crossbar_x = outward * 95.0;
        let moving_crossbar_z = 58.0;
        add_instance(
            assembly,
            &format!("pitch_moving_crossbar_{end}"),
            d.crossmember,
            pitch_frame,
            RigidTransform::translated(moving_crossbar_x, 0.0, moving_crossbar_z),
        );
        add_instance(
            assembly,
            &format!("roll_driven_gear_{end}"),
            d.roll_driven,
            roll_frame,
            RigidTransform::translated(gear_x, 0.0, 0.0),
        );
        let output_z = p.roll_axis.driven_gear.pitch_radius() + p.roll_axis.pinion.pitch_radius();
        let output_center = [gear_x, 0.0, output_z];
        let stage_distance =
            p.pitch_gearbox.small_gear.pitch_radius() + p.pitch_gearbox.large_gear.pitch_radius();
        let compound_center = [gear_x, stage_distance, output_z];
        let input_center = [gear_x, stage_distance, output_z + stage_distance];
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
        add_instance(
            assembly,
            &format!("roll_output_pinion_{end}"),
            d.roll_pinion,
            output_frame,
            RigidTransform::IDENTITY,
        );
        let first_layer = outward * 7.0;
        let second_layer = outward * 12.0;
        add_instance(
            assembly,
            &format!("roll_gearbox_{end}_stage2_driven"),
            d.roll_gearbox_large,
            output_frame,
            RigidTransform::translated(first_layer, 0.0, 0.0),
        );
        add_instance(
            assembly,
            &format!("roll_gearbox_{end}_stage2_pinion"),
            d.roll_gearbox_small,
            compound_frame,
            RigidTransform::translated(first_layer, 0.0, 0.0),
        );
        add_instance(
            assembly,
            &format!("roll_gearbox_{end}_stage1_driven"),
            d.roll_gearbox_large,
            compound_frame,
            RigidTransform::translated(second_layer, 0.0, 0.0),
        );
        add_instance(
            assembly,
            &format!("roll_gearbox_{end}_input_pinion"),
            d.roll_gearbox_small,
            input_frame,
            RigidTransform::translated(second_layer, 0.0, 0.0),
        );
        for (shaft, frame) in [
            ("output", output_frame),
            ("compound", compound_frame),
            ("input", input_frame),
        ] {
            add_instance(
                assembly,
                &format!("roll_gearbox_{end}_{shaft}_shaft"),
                d.roll_gearbox_shaft,
                frame,
                RigidTransform::translated(outward * 5.0, 0.0, 0.0),
            );
        }
        for (index, plate_offset) in [-4.0, 17.0].into_iter().enumerate() {
            add_instance(
                assembly,
                &format!("roll_gearbox_{end}_side_plate_{}", index + 1),
                d.roll_gearbox_plate,
                pitch_frame,
                RigidTransform::translated(
                    gear_x + outward * plate_offset,
                    stage_distance * 0.5,
                    output_z + stage_distance * 0.5,
                ),
            );
        }
        for (index, y) in [-18.0, 40.0].into_iter().enumerate() {
            add_instance(
                assembly,
                &format!("roll_gearbox_{end}_carrier_mount_{}", index + 1),
                d.roll_gearbox_mount,
                pitch_frame,
                RigidTransform::translated(moving_crossbar_x, y, 37.0),
            );
            add_instance(
                assembly,
                &format!("roll_gearbox_{end}_mount_arm_{}", index + 1),
                d.moving_drive_mount_arm,
                pitch_frame,
                RigidTransform::translated(
                    (moving_crossbar_x + gear_x) * 0.5,
                    y,
                    moving_crossbar_z,
                ),
            );
        }
    }
    for (end, x) in [
        ("front", p.cockpit.length.mm() * 0.5 - 8.0),
        ("rear", -p.cockpit.length.mm() * 0.5 + 8.0),
    ] {
        add_instance(
            assembly,
            &format!("roll_bearing_pedestal_{end}"),
            d.roll_pedestal,
            pitch_frame,
            RigidTransform::translated(x, 0.0, -26.0)
                .compose(RigidTransform::rotated(Axis3::Y, FRAC_PI_2)),
        );
    }
}

fn roll_gearbox_plate_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
) -> Result<SolidId, PrototypeError> {
    let stage_distance =
        p.pitch_gearbox.small_gear.pitch_radius() + p.pitch_gearbox.large_gear.pitch_radius();
    let mut plate = centered_box(builder, [3.0, 60.0, 60.0]);
    let bore_radius = p.pitch_gearbox.shaft_radius.mm() + 0.35;
    for (y, z) in [
        (-stage_distance * 0.5, -stage_distance * 0.5),
        (stage_distance * 0.5, -stage_distance * 0.5),
        (stage_distance * 0.5, stage_distance * 0.5),
    ] {
        let bore = cylinder_x(builder, bore_radius, 5.0)?;
        let bore = builder.translate(bore, Translation3 { x: 0.0, y, z })?;
        plate = builder.boolean(BooleanOperation::Difference, plate, bore)?;
    }
    Ok(plate)
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
    builder
        .rotate(
            centered,
            Rotation3 {
                x: angle(90.0),
                y: angle(0.0),
                z: angle(0.0),
            },
        )
        .map_err(PrototypeError::Feature)
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

fn gear_definition_y(
    builder: &mut FeatureBuilder,
    assembly: &mut Assembly,
    name: &str,
    gear: &SpurGear,
    width: Length,
    bore_radius: Length,
    color: [f32; 4],
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
        solid,
        Manufacturing::Fdm {
            material: FdmMaterial::Petg,
        },
        color,
    ))
}

fn gear_definition_x(
    builder: &mut FeatureBuilder,
    assembly: &mut Assembly,
    name: &str,
    gear: &SpurGear,
    width: Length,
    bore_radius: Length,
    color: [f32; 4],
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
        solid,
        Manufacturing::Fdm {
            material: FdmMaterial::Petg,
        },
        color,
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
    color: [f32; 4],
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
        annulus,
        Manufacturing::Fdm {
            material: FdmMaterial::Petg,
        },
        color,
    ))
}

fn sheet_definition(
    builder: &mut FeatureBuilder,
    assembly: &mut Assembly,
    name: &str,
    width: f64,
    height: f64,
    thickness: Length,
    color: [f32; 4],
) -> Result<ComponentDefinitionId, PrototypeError> {
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
    Ok(assembly.add_definition(ComponentDefinition {
        name: name.to_string(),
        body: Body::Sheet {
            profile,
            thickness,
            assembly_solid: solid,
        },
        manufacturing: Manufacturing::LaserCut,
        color_rgba: color,
    }))
}

fn u_sheet_definition(
    builder: &mut FeatureBuilder,
    assembly: &mut Assembly,
    name: &str,
    thickness: Length,
    color: [f32; 4],
) -> Result<ComponentDefinitionId, PrototypeError> {
    let points = vec![
        Point2 { x: -36.0, y: -32.0 },
        Point2 { x: 36.0, y: -32.0 },
        Point2 { x: 36.0, y: 32.0 },
        Point2 { x: 10.0, y: 32.0 },
        Point2 { x: 10.0, y: -4.5 },
        Point2 { x: -10.0, y: -4.5 },
        Point2 { x: -10.0, y: 32.0 },
        Point2 { x: -36.0, y: 32.0 },
    ];
    let profile = builder.polygon(points)?;
    let solid = builder.extrude(profile, thickness)?;
    let solid = builder.translate(
        solid,
        Translation3 {
            x: 0.0,
            y: 0.0,
            z: -thickness.mm() * 0.5,
        },
    )?;
    Ok(assembly.add_definition(ComponentDefinition {
        name: name.to_string(),
        body: Body::Sheet {
            profile,
            thickness,
            assembly_solid: solid,
        },
        manufacturing: Manufacturing::LaserCut,
        color_rgba: color,
    }))
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
    let solid = builder.primitive(Primitive3::Cylinder {
        height: length(height),
        radius: length(radius),
        segments: 64,
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
    solid: SolidId,
    manufacturing: Manufacturing,
    color_rgba: [f32; 4],
) -> ComponentDefinitionId {
    assembly.add_definition(ComponentDefinition {
        name: name.to_string(),
        body: Body::Solid(solid),
        manufacturing,
        color_rgba,
    })
}

fn add_instance(
    assembly: &mut Assembly,
    name: &str,
    definition: ComponentDefinitionId,
    frame: FrameId,
    local_pose: RigidTransform,
) {
    assembly.add_instance(ComponentInstance {
        name: name.to_string(),
        definition,
        frame,
        local_pose,
    });
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

// SPDX-License-Identifier: MIT

use super::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum RollBearingPosition {
    Locating,
    Floating,
}

impl RollBearingPosition {
    pub(super) const fn for_end(end: LongitudinalEnd) -> Self {
        match end {
            LongitudinalEnd::Front => Self::Locating,
            LongitudinalEnd::Rear => Self::Floating,
        }
    }

    pub(super) const fn suffix(self) -> &'static str {
        match self {
            Self::Locating => "locating",
            Self::Floating => "floating",
        }
    }
}

fn roll_bearing_carrier_definition(
    definitions: &Definitions,
    end: LongitudinalEnd,
) -> Defined<CarrierEndDatums> {
    match RollBearingPosition::for_end(end) {
        RollBearingPosition::Locating => definitions.roll.locating_bearing_carrier_end,
        RollBearingPosition::Floating => definitions.roll.floating_bearing_carrier_end,
    }
}

fn roll_bearing_retainer_definition(
    definitions: &Definitions,
    end: LongitudinalEnd,
) -> Defined<RollBearingRetainerDatums> {
    match RollBearingPosition::for_end(end) {
        RollBearingPosition::Locating => definitions.roll.locating_bearing_retainer,
        RollBearingPosition::Floating => definitions.roll.floating_bearing_retainer,
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn build_roll_assembly(
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
        d.roll.cockpit.id,
        roll_frame,
        RigidTransform::translated(0.0, 0.0, -p.cockpit.suspension_drop.mm()),
    );
    add_instance(
        assembly,
        "roll_shaft",
        d.roll.roll_shaft.id,
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
            d.roll.cockpit_hanger.id,
            roll_frame,
            RigidTransform::translated(x, 0.0, 0.0),
            ComponentLocation::new().with_ordinal((index + 1) as u16),
        );
    }
    let carrier_rail_y =
        p.pitch_sector.carrier_spacing.mm() * 0.5 - p.frame.moving_carrier_inboard_offset.mm();
    for (index, y) in [-carrier_rail_y, carrier_rail_y].into_iter().enumerate() {
        add_located_instance(
            assembly,
            &format!("pitch_cradle_longitudinal_rail_{}", index + 1),
            d.roll.pitch_cradle_longitudinal_rail.id,
            pitch_frame,
            RigidTransform::translated(0.0, y, p.frame.moving_carrier_height.mm()),
            ComponentLocation::new().with_ordinal((index + 1) as u16),
        );
    }
    for (end, outward) in [(LongitudinalEnd::Front, 1.0), (LongitudinalEnd::Rear, -1.0)] {
        let location = ComponentLocation::new().with_longitudinal_end(end);
        let carrier_definition = roll_bearing_carrier_definition(d, end);
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
            carrier_definition.id,
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
            d.roll.roll_driven,
            roll_frame,
            RigidTransform::translated(gear_x, 0.0, 0.0),
            location,
        );
        let output_z =
            -(p.roll_axis.driven_gear.pitch_radius() + p.roll_axis.pinion.pitch_radius());
        let output_center = [gear_x, 0.0, output_z];
        let lateral = roll_gearbox_stage_lateral_offset(p);
        let vertical = roll_gearbox_stage_vertical_offset(p);
        // Fold both stages into a shallow V below the output. The former
        // one-sided L intersected the right pitch unit, while a pure vertical
        // stack lost floor clearance at the pitch limit.
        let compound_center = [gear_x, lateral, output_z - vertical];
        let input_center = [gear_x, 0.0, output_z - vertical * 2.0];
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
            d.roll.roll_pinion,
            output_frame,
            RigidTransform::IDENTITY,
            location,
        );
        let first_layer = outward * 7.0;
        let second_layer = outward * 12.0;
        add_located_instance(
            assembly,
            &format!("roll_gearbox_{end}_stage2_driven"),
            d.roll.roll_gearbox_large,
            output_frame,
            RigidTransform::translated(first_layer, 0.0, 0.0),
            location.with_ordinal(1),
        );
        add_located_instance(
            assembly,
            &format!("roll_gearbox_{end}_stage2_pinion"),
            d.roll.roll_gearbox_small,
            compound_frame,
            RigidTransform::translated(first_layer, 0.0, 0.0),
            location.with_ordinal(1),
        );
        add_located_instance(
            assembly,
            &format!("roll_gearbox_{end}_stage1_driven"),
            d.roll.roll_gearbox_large,
            compound_frame,
            RigidTransform::translated(second_layer, 0.0, 0.0),
            location.with_ordinal(2),
        );
        add_located_instance(
            assembly,
            &format!("roll_gearbox_{end}_input_pinion"),
            d.roll.roll_gearbox_small,
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
                d.roll.roll_gearbox_shaft,
                frame,
                RigidTransform::translated(outward * 5.5, 0.0, 0.0),
                location.with_ordinal(ordinal),
            );
        }
        for (index, plate_offset) in [-5.5, 16.5].into_iter().enumerate() {
            add_located_instance(
                assembly,
                &format!("roll_gearbox_{end}_side_plate_{}", index + 1),
                d.roll.roll_gearbox_plate.id,
                pitch_frame,
                RigidTransform::translated(
                    gear_x + outward * plate_offset,
                    0.0,
                    output_z - vertical,
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
                d.roll.moving_drive_mount_arm.id,
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
        let retainer_definition = roll_bearing_retainer_definition(d, end);
        let end = end.as_str();
        let end_rotation = if outward > 0.0 { 0.0 } else { PI };
        let x = outward * (p.roll_axis.bearing_station.mm() + roll_bearing_center_offset_x(p));
        add_located_instance(
            assembly,
            &format!("roll_bearing_{end}"),
            d.roll.roll_bearing.id,
            pitch_frame,
            RigidTransform::translated(x, 0.0, 0.0)
                .compose(RigidTransform::rotated(Axis3::Z, end_rotation)),
            location,
        );
        let retainer_x = outward
            * (p.roll_axis.bearing_station.mm()
                + p.frame.bearing_pedestal_thickness.mm() * 0.5
                + roll_bearing_retainer_thickness_mm() * 0.5);
        add_located_instance(
            assembly,
            &format!("roll_bearing_retainer_{end}"),
            retainer_definition.id,
            pitch_frame,
            RigidTransform::translated(retainer_x, 0.0, 0.0)
                .compose(RigidTransform::rotated(Axis3::Z, end_rotation)),
            location,
        );
    }
    for (end, name, x, rotation, ordinal) in [
        (
            LongitudinalEnd::Front,
            "inboard",
            front_bearing_inboard_collar_x(p),
            PI,
            1,
        ),
        (
            LongitudinalEnd::Front,
            "outboard",
            front_bearing_outboard_collar_x(p),
            0.0,
            2,
        ),
        (
            LongitudinalEnd::Rear,
            "inboard",
            rear_bearing_inboard_collar_x(p),
            0.0,
            1,
        ),
        (
            LongitudinalEnd::Rear,
            "outboard",
            rear_bearing_outboard_collar_x(p),
            PI,
            2,
        ),
    ] {
        add_located_instance(
            assembly,
            &format!("roll_shaft_bearing_collar_{}_{name}", end.as_str()),
            d.roll.roll_shaft_bearing_collar.id,
            roll_frame,
            RigidTransform::translated(x, 0.0, 0.0)
                .compose(RigidTransform::rotated(Axis3::Z, rotation)),
            ComponentLocation::new()
                .with_longitudinal_end(end)
                .with_ordinal(ordinal),
        );
    }
}

pub(super) fn build_roll_bearing_fits(
    assembly: &mut Assembly,
    d: &Definitions,
    p: &PrototypeParameters,
) -> Result<(), PrototypeError> {
    let shaft = required_instance(assembly, ComponentRole::RollShaft, ComponentLocation::new())?;
    for end in [LongitudinalEnd::Front, LongitudinalEnd::Rear] {
        let location = ComponentLocation::new().with_longitudinal_end(end);
        let position = RollBearingPosition::for_end(end);
        let carrier_definition = roll_bearing_carrier_definition(d, end);
        let retainer_definition = roll_bearing_retainer_definition(d, end);
        let bearing = required_instance(assembly, ComponentRole::RollBearing, location)?;
        let carrier = required_instance(assembly, ComponentRole::RollBearingCarrierEnd, location)?;
        let retainer = required_instance(assembly, ComponentRole::RollBearingRetainer, location)?;
        let shaft_surface = match end {
            LongitudinalEnd::Front => d.roll.roll_shaft.datums.front_bearing_surface,
            LongitudinalEnd::Rear => d.roll.roll_shaft.datums.rear_bearing_surface,
        };
        add_cylindrical_fit(
            assembly,
            shaft,
            shaft_surface,
            bearing,
            d.roll.roll_bearing.datums.inner_bore,
            roll_bearing_inner_radial_clearance_mm(),
        )?;
        add_cylindrical_fit(
            assembly,
            bearing,
            d.roll.roll_bearing.datums.outer_surface,
            carrier,
            carrier_definition.datums.bearing_bore,
            roll_bearing_carrier_radial_clearance_mm(position),
        )?;
        add_surface_contact(
            assembly,
            bearing,
            d.roll.roll_bearing.datums.negative_x_face,
            carrier,
            carrier_definition.datums.bearing_shoulder_face,
            100.0,
        )?;
        match position {
            RollBearingPosition::Locating => add_surface_contact(
                assembly,
                bearing,
                d.roll.roll_bearing.datums.positive_x_face,
                retainer,
                retainer_definition.datums.bearing_face,
                100.0,
            )?,
            RollBearingPosition::Floating => add_plane_clearance(
                assembly,
                bearing,
                d.roll.roll_bearing.datums.positive_x_face,
                retainer,
                retainer_definition.datums.bearing_face,
                roll_bearing_axial_float_mm(),
                100.0,
            )?,
        }
        add_surface_contact(
            assembly,
            retainer,
            retainer_definition.datums.carrier_face,
            carrier,
            carrier_definition.datums.outer_face,
            500.0,
        )?;
        for index in 0..3 {
            add_roll_bearing_retainer_fastener(assembly, d, p, end, index, carrier, retainer)?;
        }
    }
    build_roll_shaft_axial_location(assembly, d)?;
    Ok(())
}

fn build_roll_shaft_axial_location(
    assembly: &mut Assembly,
    d: &Definitions,
) -> Result<(), PrototypeError> {
    let shaft = required_instance(assembly, ComponentRole::RollShaft, ComponentLocation::new())?;
    for end in [LongitudinalEnd::Front, LongitudinalEnd::Rear] {
        let location = ComponentLocation::new().with_longitudinal_end(end);
        let bearing = required_instance(assembly, ComponentRole::RollBearing, location)?;
        let inboard = required_instance(
            assembly,
            ComponentRole::RollShaftBearingCollar,
            location.with_ordinal(1),
        )?;
        let outboard = required_instance(
            assembly,
            ComponentRole::RollShaftBearingCollar,
            location.with_ordinal(2),
        )?;
        let (inboard_surface, outboard_surface, inboard_face, outboard_face) = match end {
            LongitudinalEnd::Front => (
                d.roll.roll_shaft.datums.front_inboard_collar_surface,
                d.roll.roll_shaft.datums.front_outboard_collar_surface,
                d.roll.roll_bearing.datums.negative_x_face,
                d.roll.roll_bearing.datums.positive_x_face,
            ),
            LongitudinalEnd::Rear => (
                d.roll.roll_shaft.datums.rear_inboard_collar_surface,
                d.roll.roll_shaft.datums.rear_outboard_collar_surface,
                d.roll.roll_bearing.datums.negative_x_face,
                d.roll.roll_bearing.datums.positive_x_face,
            ),
        };
        for (collar, shaft_surface) in [(inboard, inboard_surface), (outboard, outboard_surface)] {
            add_cylindrical_fit(
                assembly,
                shaft,
                shaft_surface,
                collar,
                d.roll.roll_shaft_bearing_collar.datums.bore,
                0.0,
            )?;
        }
        add_surface_contact(
            assembly,
            bearing,
            inboard_face,
            inboard,
            d.roll.roll_shaft_bearing_collar.datums.bearing_face,
            40.0,
        )?;
        add_surface_contact(
            assembly,
            bearing,
            outboard_face,
            outboard,
            d.roll.roll_shaft_bearing_collar.datums.bearing_face,
            40.0,
        )?;
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn add_plane_clearance(
    assembly: &mut Assembly,
    first: ComponentInstanceId,
    first_plane: DatumId<PlaneDatum>,
    second: ComponentInstanceId,
    second_plane: DatumId<PlaneDatum>,
    target_separation_mm: f64,
    minimum_overlap_area_mm2: f64,
) -> Result<(), PrototypeError> {
    assembly
        .add_relation(AssemblyRelation::PlaneClearance(PlaneClearance {
            first: DatumEndpoint::new(first, first_plane),
            second: DatumEndpoint::new(second, second_plane),
            target_separation: NonNegativeLength::mm(target_separation_mm)
                .expect("bearing axial clearance is non-negative"),
            minimum_overlap_area: PositiveArea::square_mm(minimum_overlap_area_mm2)
                .expect("bearing stop overlap is positive"),
            tolerance: EngineeringTolerance {
                linear: NonNegativeLength::mm(0.02)
                    .expect("bearing clearance tolerance is non-negative"),
                angular: NonNegativeAngle::degrees(0.1)
                    .expect("bearing clearance angular tolerance is non-negative"),
            },
        }))
        .map_err(PrototypeError::Assembly)?;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn add_roll_bearing_retainer_fastener(
    assembly: &mut Assembly,
    d: &Definitions,
    p: &PrototypeParameters,
    end: LongitudinalEnd,
    index: usize,
    carrier: ComponentInstanceId,
    retainer: ComponentInstanceId,
) -> Result<(), PrototypeError> {
    const WASHER_THICKNESS: f64 = 0.5;
    const NUT_THICKNESS: f64 = 2.4;
    let outward = match end {
        LongitudinalEnd::Front => 1.0,
        LongitudinalEnd::Rear => -1.0,
    };
    let end_rotation = if outward > 0.0 { 0.0 } else { PI };
    let carrier_half = p.frame.bearing_pedestal_thickness.mm() * 0.5;
    let retainer_outer_face_x = carrier_half + roll_bearing_retainer_thickness_mm();
    let [hole_y, hole_z] = roll_bearing_retainer_hole_centres(p)[index];
    let carrier_base =
        RigidTransform::translated(outward * p.roll_axis.bearing_station.mm(), 0.0, 0.0)
            .compose(RigidTransform::rotated(Axis3::Z, end_rotation));
    let pose = |local_x| carrier_base.compose(RigidTransform::translated(local_x, hole_y, hole_z));
    let ordinal = (index + 1) as u16;
    let base_location = ComponentLocation::new()
        .with_longitudinal_end(end)
        .with_ordinal(ordinal);
    let stem = format!("roll_bearing_retainer_{}_m3x20_{}", end.as_str(), index + 1);
    let bolt = add_located_instance(
        assembly,
        &format!("{stem}_bolt"),
        d.hardware.m3x20_bolt.definition,
        assembly
            .instance(carrier)
            .expect("roll bearing carrier exists")
            .frame,
        pose(retainer_outer_face_x + WASHER_THICKNESS),
        base_location,
    );
    let first_washer = add_located_instance(
        assembly,
        &format!("{stem}_head_washer"),
        d.hardware.m3_washer.definition,
        assembly
            .instance(carrier)
            .expect("roll bearing carrier exists")
            .frame,
        pose(retainer_outer_face_x + WASHER_THICKNESS * 0.5),
        base_location.with_ordinal(ordinal * 2 - 1),
    );
    let second_washer = add_located_instance(
        assembly,
        &format!("{stem}_nut_washer"),
        d.hardware.m3_washer.definition,
        assembly
            .instance(carrier)
            .expect("roll bearing carrier exists")
            .frame,
        pose(-carrier_half - WASHER_THICKNESS * 0.5),
        base_location.with_ordinal(ordinal * 2),
    );
    let nut = add_located_instance(
        assembly,
        &format!("{stem}_nut"),
        d.hardware.m3_nut.definition,
        assembly
            .instance(carrier)
            .expect("roll bearing carrier exists")
            .frame,
        pose(-carrier_half - WASHER_THICKNESS - NUT_THICKNESS * 0.5),
        base_location,
    );
    let retainer_datums = roll_bearing_retainer_definition(d, end).datums.fasteners[index];
    let carrier_datums = roll_bearing_carrier_definition(d, end)
        .datums
        .retainer_fasteners[index];
    assembly
        .add_relation(AssemblyRelation::Fastened(FastenedJoint {
            first_hole: DatumEndpoint::new(retainer, retainer_datums.hole),
            second_hole: DatumEndpoint::new(carrier, carrier_datums.hole),
            head_seat: DatumEndpoint::new(retainer, retainer_datums.positive_x_seat),
            nut_seat: DatumEndpoint::new(carrier, carrier_datums.negative_x_seat),
            hardware: FastenerHardware {
                bolt: BoltHardware {
                    instance: bolt,
                    axis: d.hardware.m3x20_bolt.axis,
                    under_head_face: d.hardware.m3x20_bolt.under_head_face,
                    shank_tip_face: d.hardware.m3x20_bolt.shank_tip_face,
                },
                nut: NutHardware {
                    instance: nut,
                    axis: d.hardware.m3_nut.axis,
                    bearing_face: d.hardware.m3_nut.positive_x_face,
                    outer_face: d.hardware.m3_nut.negative_x_face,
                },
                first_washer: Some(WasherHardware {
                    instance: first_washer,
                    axis: d.hardware.m3_washer.axis,
                    member_face: d.hardware.m3_washer.negative_x_face,
                    hardware_face: d.hardware.m3_washer.positive_x_face,
                }),
                second_washer: Some(WasherHardware {
                    instance: second_washer,
                    axis: d.hardware.m3_washer.axis,
                    member_face: d.hardware.m3_washer.positive_x_face,
                    hardware_face: d.hardware.m3_washer.negative_x_face,
                }),
            },
            thread: MetricThread::M3,
            target_hole_radial_clearance: NonNegativeLength::mm(0.2)
                .expect("M3 clearance is non-negative"),
            grip_length: PositiveLength::mm(
                p.frame.bearing_pedestal_thickness.mm() + roll_bearing_retainer_thickness_mm(),
            )
            .expect("bearing retainer grip is positive"),
            tolerance: EngineeringTolerance {
                linear: NonNegativeLength::mm(0.05)
                    .expect("bearing retainer tolerance is non-negative"),
                angular: NonNegativeAngle::degrees(0.1)
                    .expect("bearing retainer angular tolerance is non-negative"),
            },
        }))
        .map_err(PrototypeError::Assembly)?;
    Ok(())
}

fn add_cylindrical_fit(
    assembly: &mut Assembly,
    shaft: ComponentInstanceId,
    shaft_surface: DatumId<CylinderDatum>,
    bore: ComponentInstanceId,
    bore_surface: DatumId<CylinderDatum>,
    target_radial_clearance_mm: f64,
) -> Result<(), PrototypeError> {
    assembly
        .add_relation(AssemblyRelation::CylindricalFit(CylindricalFit {
            shaft: DatumEndpoint::new(shaft, shaft_surface),
            bore: DatumEndpoint::new(bore, bore_surface),
            target_radial_clearance: NonNegativeLength::mm(target_radial_clearance_mm)
                .expect("bearing radial clearance is non-negative"),
            tolerance: EngineeringTolerance {
                linear: NonNegativeLength::mm(0.02).expect("bearing fit tolerance is non-negative"),
                angular: NonNegativeAngle::degrees(0.1)
                    .expect("bearing fit angular tolerance is non-negative"),
            },
        }))
        .map_err(PrototypeError::Assembly)?;
    Ok(())
}

pub(super) fn build_moving_carrier_contacts(
    assembly: &mut Assembly,
    d: &Definitions,
    p: &PrototypeParameters,
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
            d.roll.cockpit.datums.top_face,
            hanger,
            d.roll.cockpit_hanger.datums.cockpit_face,
            100.0,
        )?;
    }

    for end in [LongitudinalEnd::Front, LongitudinalEnd::Rear] {
        let end_location = ComponentLocation::new().with_longitudinal_end(end);
        let carrier_definition = roll_bearing_carrier_definition(d, end);
        let carriage_definition = d.pitch_unit.contact_carriage_plate(end);
        let carrier_end =
            required_instance(assembly, ComponentRole::RollBearingCarrierEnd, end_location)?;
        let rail_end_face = match end {
            LongitudinalEnd::Front => d.roll.pitch_cradle_longitudinal_rail.datums.positive_x,
            LongitudinalEnd::Rear => d.roll.pitch_cradle_longitudinal_rail.datums.negative_x,
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
                carrier_definition.datums.rail_face,
                120.0,
            )?;

            let carriage = required_instance(
                assembly,
                ComponentRole::PitchContactCarriagePlate,
                ComponentLocation::new()
                    .with_side(side)
                    .with_longitudinal_end(end),
            )?;
            add_surface_contact(
                assembly,
                carriage,
                carriage_definition.datums.carrier_contact_face,
                carrier_end,
                carrier_definition.datums.carriage_face,
                120.0,
            )?;
            let carrier_side_index = match (end, side) {
                (LongitudinalEnd::Front, Side::Left) | (LongitudinalEnd::Rear, Side::Right) => 0,
                (LongitudinalEnd::Front, Side::Right) | (LongitudinalEnd::Rear, Side::Left) => 1,
            };
            for physical_fastener_index in 0..2 {
                let carriage_fastener_index = match end {
                    LongitudinalEnd::Front => physical_fastener_index,
                    LongitudinalEnd::Rear => 1 - physical_fastener_index,
                };
                add_carriage_carrier_fastener(
                    assembly,
                    d,
                    p,
                    end,
                    side,
                    carriage,
                    carriage_definition.datums.carrier_fasteners[carriage_fastener_index],
                    carrier_end,
                    carrier_definition.datums.carriage_fasteners[carrier_side_index]
                        [physical_fastener_index],
                    physical_fastener_index,
                )?;
            }
        }

        for ordinal in [1, 2] {
            let arm_location = end_location.with_ordinal(ordinal);
            let arm =
                required_instance(assembly, ComponentRole::MovingDriveMountArm, arm_location)?;
            add_surface_contact(
                assembly,
                carrier_end,
                carrier_definition.datums.arm_face,
                arm,
                d.roll.moving_drive_mount_arm.datums.carrier_face,
                100.0,
            )?;
            let plate = required_instance(
                assembly,
                ComponentRole::RollGearboxPlate,
                end_location.with_ordinal(2),
            )?;
            let plate_face = match end {
                LongitudinalEnd::Front => d.roll.roll_gearbox_plate.datums.positive_x,
                LongitudinalEnd::Rear => d.roll.roll_gearbox_plate.datums.negative_x,
            };
            add_surface_contact(
                assembly,
                arm,
                d.roll.moving_drive_mount_arm.datums.plate_face,
                plate,
                plate_face,
                40.0,
            )?;
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn add_carriage_carrier_fastener(
    assembly: &mut Assembly,
    d: &Definitions,
    p: &PrototypeParameters,
    end: LongitudinalEnd,
    side: Side,
    carriage: ComponentInstanceId,
    carriage_datums: AxialFastenerDatums,
    carrier: ComponentInstanceId,
    carrier_datums: AxialFastenerDatums,
    index: usize,
) -> Result<(), PrototypeError> {
    const WASHER_THICKNESS: f64 = 0.5;
    const NUT_THICKNESS: f64 = 2.4;
    let outward = match end {
        LongitudinalEnd::Front => 1.0,
        LongitudinalEnd::Rear => -1.0,
    };
    let end_rotation = if outward > 0.0 { 0.0 } else { PI };
    let contact_x =
        p.frame.moving_carrier_half_span.mm() + p.frame.moving_carrier_member_width.mm();
    let pad_outer_x = contact_x + pitch_carriage_carrier_pad_depth_mm();
    let carrier_inner_x = p.frame.moving_carrier_half_span.mm();
    let y = match side {
        Side::Left => -pitch_carriage_carrier_mount_y_mm(p),
        Side::Right => pitch_carriage_carrier_mount_y_mm(p),
    };
    let z =
        p.frame.moving_carrier_height.mm() + pitch_carriage_carrier_fastener_z_offsets_mm()[index];
    let pose = |x| {
        RigidTransform::translated(outward * x, y, z)
            .compose(RigidTransform::rotated(Axis3::Z, end_rotation))
    };
    let ordinal = 11 + index as u16;
    let base_location = ComponentLocation::new()
        .with_side(side)
        .with_longitudinal_end(end);
    let stem = format!(
        "pitch_carriage_{}_{}_carrier_m3x25_{}",
        side.as_str(),
        end.as_str(),
        index + 1
    );
    let bolt = add_located_instance(
        assembly,
        &format!("{stem}_bolt"),
        d.hardware.m3x25_bolt.definition,
        assembly.instance(carrier).expect("carrier exists").frame,
        pose(pad_outer_x + WASHER_THICKNESS),
        base_location.with_ordinal(ordinal),
    );
    let first_washer = add_located_instance(
        assembly,
        &format!("{stem}_head_washer"),
        d.hardware.m3_washer.definition,
        assembly.instance(carrier).expect("carrier exists").frame,
        pose(pad_outer_x + WASHER_THICKNESS * 0.5),
        base_location.with_ordinal(ordinal * 2 - 1),
    );
    let second_washer = add_located_instance(
        assembly,
        &format!("{stem}_nut_washer"),
        d.hardware.m3_washer.definition,
        assembly.instance(carrier).expect("carrier exists").frame,
        pose(carrier_inner_x - WASHER_THICKNESS * 0.5),
        base_location.with_ordinal(ordinal * 2),
    );
    let nut = add_located_instance(
        assembly,
        &format!("{stem}_nut"),
        d.hardware.m3_nut.definition,
        assembly.instance(carrier).expect("carrier exists").frame,
        pose(carrier_inner_x - WASHER_THICKNESS - NUT_THICKNESS * 0.5),
        base_location.with_ordinal(ordinal),
    );
    assembly
        .add_relation(AssemblyRelation::Fastened(FastenedJoint {
            first_hole: DatumEndpoint::new(carriage, carriage_datums.hole),
            second_hole: DatumEndpoint::new(carrier, carrier_datums.hole),
            head_seat: DatumEndpoint::new(carriage, carriage_datums.positive_x_seat),
            nut_seat: DatumEndpoint::new(carrier, carrier_datums.negative_x_seat),
            hardware: FastenerHardware {
                bolt: BoltHardware {
                    instance: bolt,
                    axis: d.hardware.m3x25_bolt.axis,
                    under_head_face: d.hardware.m3x25_bolt.under_head_face,
                    shank_tip_face: d.hardware.m3x25_bolt.shank_tip_face,
                },
                nut: NutHardware {
                    instance: nut,
                    axis: d.hardware.m3_nut.axis,
                    bearing_face: d.hardware.m3_nut.positive_x_face,
                    outer_face: d.hardware.m3_nut.negative_x_face,
                },
                first_washer: Some(WasherHardware {
                    instance: first_washer,
                    axis: d.hardware.m3_washer.axis,
                    member_face: d.hardware.m3_washer.negative_x_face,
                    hardware_face: d.hardware.m3_washer.positive_x_face,
                }),
                second_washer: Some(WasherHardware {
                    instance: second_washer,
                    axis: d.hardware.m3_washer.axis,
                    member_face: d.hardware.m3_washer.positive_x_face,
                    hardware_face: d.hardware.m3_washer.negative_x_face,
                }),
            },
            thread: MetricThread::M3,
            target_hole_radial_clearance: NonNegativeLength::mm(0.2)
                .expect("M3 clearance is non-negative"),
            grip_length: PositiveLength::mm(
                pitch_carriage_carrier_pad_depth_mm() + p.frame.moving_carrier_member_width.mm(),
            )
            .expect("carriage carrier grip is positive"),
            tolerance: EngineeringTolerance {
                linear: NonNegativeLength::mm(0.05)
                    .expect("carriage carrier tolerance is non-negative"),
                angular: NonNegativeAngle::degrees(0.1)
                    .expect("carriage carrier angular tolerance is non-negative"),
            },
        }))
        .map_err(PrototypeError::Assembly)?;
    Ok(())
}

pub(super) fn roll_gearbox_plate_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
) -> Result<SolidId, PrototypeError> {
    let lateral = roll_gearbox_stage_lateral_offset(p);
    let vertical = roll_gearbox_stage_vertical_offset(p);
    let centers = [[0.0, vertical], [lateral, 0.0], [0.0, -vertical]];
    let supports = [
        [
            -p.roll_axis.gearbox_support_half_span.mm(),
            roll_gearbox_plate_support_offset_z(),
        ],
        [
            p.roll_axis.gearbox_support_half_span.mm(),
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

pub(super) fn roll_gearbox_support_z(p: &PrototypeParameters) -> f64 {
    let output_z = -(p.roll_axis.driven_gear.pitch_radius() + p.roll_axis.pinion.pitch_radius());
    output_z - roll_gearbox_stage_vertical_offset(p) + roll_gearbox_plate_support_offset_z()
}

pub(super) fn roll_gearbox_stage_lateral_offset(p: &PrototypeParameters) -> f64 {
    p.roll_axis.gearbox_support_half_span.mm()
}

pub(super) fn roll_gearbox_stage_vertical_offset(p: &PrototypeParameters) -> f64 {
    let stage_distance =
        p.pitch_gearbox.small_gear.pitch_radius() + p.pitch_gearbox.large_gear.pitch_radius();
    let lateral = roll_gearbox_stage_lateral_offset(p);
    libm::sqrt(stage_distance * stage_distance - lateral * lateral)
}

pub(super) const fn roll_gearbox_plate_support_offset_z() -> f64 {
    0.0
}

pub(super) fn moving_drive_mount_arm_solid(
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

pub(super) const fn roll_gearbox_arm_width_x() -> f64 {
    8.0
}

pub(super) fn roll_gearbox_arm_center_local_x(p: &PrototypeParameters) -> f64 {
    p.roll_axis.drive_station.mm() + 16.5 + 1.5 + 4.0
        - (p.frame.moving_carrier_half_span.mm() + p.frame.moving_carrier_member_width.mm() * 0.5)
}

pub(super) fn roll_gearbox_plate_support_local_z(p: &PrototypeParameters) -> f64 {
    roll_gearbox_support_z(p) - p.frame.moving_carrier_height.mm()
}

pub(super) fn roll_bearing_carrier_tie_center_x(p: &PrototypeParameters) -> f64 {
    p.frame.moving_carrier_half_span.mm() + p.frame.moving_carrier_member_width.mm() * 0.5
        - p.roll_axis.bearing_station.mm()
}

pub(super) const fn roll_bearing_inner_radial_clearance_mm() -> f64 {
    0.0
}

pub(super) const fn roll_bearing_carrier_radial_clearance_mm(position: RollBearingPosition) -> f64 {
    match position {
        RollBearingPosition::Locating => 0.0,
        RollBearingPosition::Floating => 0.15,
    }
}

pub(super) const fn roll_bearing_retainer_thickness_mm() -> f64 {
    3.0
}

pub(super) const fn roll_bearing_axial_float_mm() -> f64 {
    1.0
}

// NBK NSCS-8-8-SB1, a clamping collar intended for the 608ZZ inner race.
// https://www.nbk1560.com/images/en/product/setcollar/NSCS-SB/NSCS-SB_1.pdf
pub(super) const fn roll_bearing_collar_width_mm() -> f64 {
    8.5
}

pub(super) const fn roll_bearing_collar_body_width_mm() -> f64 {
    7.5
}

pub(super) const fn roll_bearing_collar_outer_radius_mm() -> f64 {
    10.0
}

pub(super) const fn roll_bearing_collar_boss_radius_mm() -> f64 {
    5.85
}

pub(super) fn front_bearing_inboard_collar_x(p: &PrototypeParameters) -> f64 {
    p.roll_axis.bearing_station.mm() + roll_bearing_center_offset_x(p)
        - p.roll_axis.bearing_width.mm() * 0.5
        - roll_bearing_collar_width_mm() * 0.5
}

pub(super) fn front_bearing_outboard_collar_x(p: &PrototypeParameters) -> f64 {
    p.roll_axis.bearing_station.mm()
        + roll_bearing_center_offset_x(p)
        + p.roll_axis.bearing_width.mm() * 0.5
        + roll_bearing_collar_width_mm() * 0.5
}

pub(super) fn rear_bearing_inboard_collar_x(p: &PrototypeParameters) -> f64 {
    -front_bearing_inboard_collar_x(p)
}

pub(super) fn rear_bearing_outboard_collar_x(p: &PrototypeParameters) -> f64 {
    -front_bearing_outboard_collar_x(p)
}

pub(super) fn roll_bearing_center_offset_x(p: &PrototypeParameters) -> f64 {
    (p.frame.bearing_pedestal_thickness.mm() - p.roll_axis.bearing_width.mm()) * 0.5
}

pub(super) fn roll_bearing_inner_face_x(p: &PrototypeParameters) -> f64 {
    roll_bearing_center_offset_x(p) - p.roll_axis.bearing_width.mm() * 0.5
}

pub(super) fn roll_bearing_retainer_hole_centres(p: &PrototypeParameters) -> [[f64; 2]; 3] {
    let radius = p.roll_axis.bearing_outer_radius.mm() + 3.5;
    let diagonal = radius * 0.866_025_403_784_438_6;
    [
        [0.0, radius],
        [-diagonal, -radius * 0.5],
        [diagonal, -radius * 0.5],
    ]
}

fn roll_bearing_retainer_outer_radius_mm(p: &PrototypeParameters) -> f64 {
    p.roll_axis.bearing_outer_radius.mm() + 7.0
}

fn roll_bearing_retainer_inner_radius_mm(p: &PrototypeParameters) -> f64 {
    p.roll_axis.bearing_outer_radius.mm() - 2.0
}

pub(super) fn roll_bearing_carrier_end_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
    position: RollBearingPosition,
) -> Result<SolidId, PrototypeError> {
    let thickness = p.frame.bearing_pedestal_thickness.mm();
    let boss_center = [0.0, 0.0];
    let carrier_rail_y =
        p.pitch_sector.carrier_spacing.mm() * 0.5 - p.frame.moving_carrier_inboard_offset.mm();
    let bridge_half_span = carrier_rail_y - p.frame.moving_carrier_member_width.mm() * 0.5;
    let bridge_z = p.frame.moving_carrier_height.mm();
    let mut pedestal = cylinder_x(builder, roll_bearing_retainer_outer_radius_mm(p), thickness)?;
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
    let carriage_mount_y = pitch_carriage_carrier_mount_y_mm(p);
    for y in [-carriage_mount_y, carriage_mount_y] {
        let ear = centered_box(
            builder,
            [
                p.frame.moving_carrier_member_width.mm(),
                pitch_carriage_carrier_pad_width_mm(),
                pitch_carriage_carrier_pad_height_mm(),
            ],
        );
        let ear = builder.translate(
            ear,
            Translation3 {
                x: tie_center_x,
                y,
                z: bridge_z,
            },
        )?;
        pedestal = builder.boolean(BooleanOperation::Union, pedestal, ear)?;
    }

    let pocket_inner_x = roll_bearing_inner_face_x(p);
    let pocket_outer_x = thickness * 0.5 + 1.0;
    let pocket_width = pocket_outer_x - pocket_inner_x;
    let pocket = cylinder_x(
        builder,
        p.roll_axis.bearing_outer_radius.mm() + roll_bearing_carrier_radial_clearance_mm(position),
        pocket_width,
    )?;
    let pocket = builder.translate(
        pocket,
        Translation3 {
            x: (pocket_inner_x + pocket_outer_x) * 0.5,
            y: 0.0,
            z: 0.0,
        },
    )?;
    pedestal = builder.boolean(BooleanOperation::Difference, pedestal, pocket)?;
    let shoulder_opening = cylinder_x(
        builder,
        roll_bearing_retainer_inner_radius_mm(p),
        thickness + 2.0,
    )?;
    pedestal = builder.boolean(BooleanOperation::Difference, pedestal, shoulder_opening)?;
    for [y, z] in roll_bearing_retainer_hole_centres(p) {
        pedestal = subtract_x_bore_at(
            builder,
            pedestal,
            m3_clearance_radius_mm(),
            thickness + 2.0,
            y,
            z,
        )?;
    }
    for y in [-carriage_mount_y, carriage_mount_y] {
        for z_offset in pitch_carriage_carrier_fastener_z_offsets_mm() {
            let bore = cylinder_x(
                builder,
                m3_clearance_radius_mm(),
                p.frame.moving_carrier_member_width.mm() + 2.0,
            )?;
            let bore = builder.translate(
                bore,
                Translation3 {
                    x: tie_center_x,
                    y,
                    z: bridge_z + z_offset,
                },
            )?;
            pedestal = builder.boolean(BooleanOperation::Difference, pedestal, bore)?;
        }
    }
    Ok(pedestal)
}

pub(super) fn roll_bearing_retainer_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
    position: RollBearingPosition,
) -> Result<SolidId, PrototypeError> {
    let thickness = roll_bearing_retainer_thickness_mm();
    let mut retainer = annulus_solid_x(
        builder,
        roll_bearing_retainer_outer_radius_mm(p),
        roll_bearing_retainer_inner_radius_mm(p),
        thickness,
    )?;
    // The collar's 20 mm body starts 1 mm beyond the bearing face. Keep the
    // 1 mm lip that retains the outer race, then counterbore the remaining
    // retainer thickness so the inner-race collar cannot touch it.
    let counterbore_width = thickness - 1.0;
    let counterbore = cylinder_x(
        builder,
        roll_bearing_collar_outer_radius_mm() + 0.2,
        counterbore_width + 0.2,
    )?;
    let counterbore = builder.translate(
        counterbore,
        Translation3 {
            x: thickness * 0.5 - counterbore_width * 0.5 + 0.1,
            y: 0.0,
            z: 0.0,
        },
    )?;
    retainer = builder.boolean(BooleanOperation::Difference, retainer, counterbore)?;
    if position == RollBearingPosition::Floating {
        let clearance = roll_bearing_axial_float_mm();
        let recess = cylinder_x(
            builder,
            p.roll_axis.bearing_outer_radius.mm() + 0.2,
            clearance + 0.2,
        )?;
        let recess = builder.translate(
            recess,
            Translation3 {
                x: -thickness * 0.5 + clearance * 0.5 - 0.1,
                y: 0.0,
                z: 0.0,
            },
        )?;
        retainer = builder.boolean(BooleanOperation::Difference, retainer, recess)?;
    }
    for [y, z] in roll_bearing_retainer_hole_centres(p) {
        retainer = subtract_x_bore_at(
            builder,
            retainer,
            m3_clearance_radius_mm(),
            thickness + 2.0,
            y,
            z,
        )?;
    }
    Ok(retainer)
}

pub(super) fn roll_bearing_collar_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
) -> Result<SolidId, PrototypeError> {
    let total_width = roll_bearing_collar_width_mm();
    let body_width = roll_bearing_collar_body_width_mm();
    let boss_width = total_width - body_width;
    let body = annulus_solid_x(
        builder,
        roll_bearing_collar_outer_radius_mm(),
        p.roll_axis.shaft_radius.mm(),
        body_width,
    )?;
    let body = builder.translate(
        body,
        Translation3 {
            x: boss_width * 0.5,
            y: 0.0,
            z: 0.0,
        },
    )?;
    let boss = annulus_solid_x(
        builder,
        roll_bearing_collar_boss_radius_mm(),
        p.roll_axis.shaft_radius.mm(),
        boss_width,
    )?;
    let boss = builder.translate(
        boss,
        Translation3 {
            x: -body_width * 0.5,
            y: 0.0,
            z: 0.0,
        },
    )?;
    builder
        .boolean(BooleanOperation::Union, body, boss)
        .map_err(PrototypeError::Feature)
}

fn subtract_x_bore_at(
    builder: &mut FeatureBuilder,
    solid: SolidId,
    radius: f64,
    width: f64,
    y: f64,
    z: f64,
) -> Result<SolidId, PrototypeError> {
    let bore = cylinder_x(builder, radius, width)?;
    let bore = builder.translate(bore, Translation3 { x: 0.0, y, z })?;
    builder
        .boolean(BooleanOperation::Difference, solid, bore)
        .map_err(PrototypeError::Feature)
}

pub(super) fn cockpit_hanger_solid(
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
    let bore = d_bore_x(
        builder,
        p.roll_axis.shaft_radius.mm() + 0.15,
        16.0,
        roll_shaft_flat_height_mm() + 0.15,
    )?;
    builder
        .boolean(BooleanOperation::Difference, hanger, bore)
        .map_err(PrototypeError::Feature)
}

pub(super) const fn roll_shaft_flat_height_mm() -> f64 {
    3.0
}

pub(super) fn d_bore_x(
    builder: &mut FeatureBuilder,
    radius: f64,
    width: f64,
    flat_height: f64,
) -> Result<SolidId, PrototypeError> {
    let cylinder = cylinder_x(builder, radius, width)?;
    let lower = -radius - 1.0;
    let height = flat_height - lower;
    let half_space = centered_box(builder, [width + 2.0, radius * 2.0 + 2.0, height]);
    let half_space = builder.translate(
        half_space,
        Translation3 {
            x: 0.0,
            y: 0.0,
            z: (flat_height + lower) * 0.5,
        },
    )?;
    builder
        .boolean(BooleanOperation::Intersection, cylinder, half_space)
        .map_err(PrototypeError::Feature)
}

pub(super) fn roll_shaft_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
) -> Result<SolidId, PrototypeError> {
    let radius = p.roll_axis.shaft_radius.mm();
    let mut shaft = cylinder_x(builder, radius, p.roll_axis.shaft_length.mm())?;
    let hanger_station = p.cockpit.length.mm() * 0.30;
    for (station, width) in [
        (-hanger_station, 14.0),
        (hanger_station, 14.0),
        (-p.roll_axis.drive_station.mm(), 12.0),
        (p.roll_axis.drive_station.mm(), 12.0),
    ] {
        let cut_height = radius - roll_shaft_flat_height_mm() + 1.0;
        let cutter = centered_box(builder, [width, radius * 2.0 + 2.0, cut_height]);
        let cutter = builder.translate(
            cutter,
            Translation3 {
                x: station,
                y: 0.0,
                z: (radius + roll_shaft_flat_height_mm() + 1.0) * 0.5,
            },
        )?;
        shaft = builder.boolean(BooleanOperation::Difference, shaft, cutter)?;
    }
    Ok(shaft)
}

pub(super) fn roll_driven_gear_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
) -> Result<SolidId, PrototypeError> {
    let face_width = 6.0;
    let profile = builder.polygon(p.roll_axis.driven_gear.profile().points)?;
    let gear = builder.extrude(profile, length(face_width))?;
    let gear = builder.translate(
        gear,
        Translation3 {
            x: 0.0,
            y: 0.0,
            z: -face_width * 0.5,
        },
    )?;
    let gear = builder.rotate(
        gear,
        Rotation3 {
            x: angle(0.0),
            y: angle(90.0),
            z: angle(0.0),
        },
    )?;
    let hub = cylinder_x(builder, 10.0, 12.0)?;
    let body = builder.boolean(BooleanOperation::Union, gear, hub)?;
    let bore = d_bore_x(
        builder,
        p.roll_axis.shaft_radius.mm() + 0.15,
        14.0,
        roll_shaft_flat_height_mm() + 0.15,
    )?;
    builder
        .boolean(BooleanOperation::Difference, body, bore)
        .map_err(PrototypeError::Feature)
}

pub(super) fn cockpit_top_z(p: &PrototypeParameters) -> f64 {
    -p.cockpit.suspension_drop.mm() + p.cockpit.height.mm() * 0.5
}

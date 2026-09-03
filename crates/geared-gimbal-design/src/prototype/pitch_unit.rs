// SPDX-License-Identifier: MIT

use super::*;

#[allow(clippy::too_many_arguments)]
pub(super) fn build_contact_units(
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
    let mut branch_shafts = [None; 2];

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
            d.pitch_unit.drive_pinion,
            frame,
            RigidTransform::IDENTITY,
            base_location.with_ordinal((branch + 1) as u16),
        );
        let drive_shaft = add_located_instance(
            assembly,
            &format!("{stem}_shaft"),
            d.pitch_unit.drive_shaft.id,
            frame,
            RigidTransform::IDENTITY,
            base_location.with_ordinal((branch + 1) as u16),
        );
        branch_shafts[branch] = Some(drive_shaft);
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
                d.pitch_unit.drive_flange,
                frame,
                RigidTransform::translated(0.0, dy, 0.0),
                base_location.with_ordinal((branch * 2 + flange + 1) as u16),
            );
        }
        add_located_instance(
            assembly,
            &format!("{stem}_distribution_branch"),
            d.pitch_unit.gearbox_small,
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
        d.pitch_unit.encoder_pinion,
        encoder_frame,
        RigidTransform::IDENTITY,
        base_location,
    );
    let retention_shaft = add_located_instance(
        assembly,
        &format!("{encoder_stem}_interface_shaft"),
        d.pitch_unit.encoder_shaft.id,
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
            d.pitch_unit.encoder_flange,
            encoder_frame,
            RigidTransform::translated(0.0, dy, 0.0),
            base_location.with_ordinal((flange + 1) as u16),
        );
    }

    let radial = [libm::cos(end_angle), libm::sin(end_angle)];
    let tangent = [-radial[1], radial[0]];
    let outboard_support_plane_y =
        y + side_sign * p.contact_unit.outboard_support_plate_offset.mm();

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
        d.pitch_unit.gearbox_distribution,
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
        d.pitch_unit.gearbox_large,
        distributor_frame,
        RigidTransform::translated(0.0, layer, 0.0),
        base_location.with_ordinal(1),
    );
    add_located_instance(
        assembly,
        &format!("pitch_gearbox_{side}_{end}_stage2_pinion"),
        d.pitch_unit.gearbox_small,
        compound_a_frame,
        RigidTransform::translated(0.0, layer, 0.0),
        base_location.with_ordinal(4),
    );
    add_located_instance(
        assembly,
        &format!("pitch_gearbox_{side}_{end}_stage1_driven"),
        d.pitch_unit.gearbox_large,
        compound_a_frame,
        RigidTransform::translated(0.0, layer * 2.0, 0.0),
        base_location.with_ordinal(2),
    );
    add_located_instance(
        assembly,
        &format!("pitch_gearbox_{side}_{end}_input_pinion"),
        d.pitch_unit.gearbox_small,
        input_frame,
        RigidTransform::translated(0.0, layer * 2.0, 0.0),
        base_location.with_ordinal(5),
    );
    let plate_center = [
        (central[0] + input_center[0]) * 0.5,
        (central[1] + input_center[1]) * 0.5,
    ];
    let inboard_near_plane_y = y + inward_sign * p.pitch_gearbox.near_plate_inboard_offset.mm();
    let outboard_plate_pose =
        RigidTransform::translated(midpoint[0], outboard_support_plane_y, midpoint[1])
            .compose(RigidTransform::rotated(Axis3::Y, -end_angle));
    let outboard_plate = add_located_instance(
        assembly,
        &format!("pitch_contact_{side}_{end}_outboard_plate"),
        d.pitch_unit.pitch_contact_outboard_plate.id,
        pitch_frame,
        outboard_plate_pose,
        base_location,
    );
    let near_plate_pose =
        RigidTransform::translated(plate_center[0], inboard_near_plane_y, plate_center[1])
            .compose(RigidTransform::rotated(Axis3::Y, -end_angle));
    let near_plate = add_located_instance(
        assembly,
        &format!("pitch_gearbox_{side}_{end}_contact_carriage_plate"),
        d.pitch_unit.contact_carriage_plate.id,
        pitch_frame,
        near_plate_pose,
        base_location,
    );
    let far_plate_pose = RigidTransform::translated(
        plate_center[0],
        y + inward_sign * p.pitch_gearbox.far_plate_inboard_offset.mm(),
        plate_center[1],
    )
    .compose(RigidTransform::rotated(Axis3::Y, -end_angle));
    let far_plate = add_located_instance(
        assembly,
        &format!("pitch_gearbox_{side}_{end}_far_plate"),
        d.pitch_unit.pitch_gearbox_far_plate.id,
        pitch_frame,
        far_plate_pose,
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
    let mut gearbox_shafts = [None; 3];
    for (shaft_index, (shaft, ordinal, frame)) in [
        ("distributor", 1, distributor_frame),
        ("compound", 2, compound_a_frame),
        ("input", 3, input_frame),
    ]
    .into_iter()
    .enumerate()
    {
        let shaft_instance = add_located_instance(
            assembly,
            &format!("pitch_gearbox_{side}_{end}_{shaft}_shaft"),
            d.pitch_unit.pitch_gearbox_shaft.id,
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
        gearbox_shafts[shaft_index] = Some(shaft_instance);
    }
    let branch_shafts = branch_shafts.map(|shaft| shaft.expect("both branch shafts were created"));
    let gearbox_shafts =
        gearbox_shafts.map(|shaft| shaft.expect("all gearbox shafts were created"));
    build_pitch_bearing_supports(
        assembly,
        d,
        p,
        pitch_frame,
        base_location,
        side_sign,
        inward_sign,
        side,
        end,
        outboard_plate,
        outboard_plate_pose,
        near_plate,
        near_plate_pose,
        far_plate,
        far_plate_pose,
        branch_shafts,
        retention_shaft,
        gearbox_shafts,
    )?;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn build_pitch_bearing_supports(
    assembly: &mut Assembly,
    d: &Definitions,
    p: &PrototypeParameters,
    pitch_frame: FrameId,
    base_location: ComponentLocation,
    side_sign: f64,
    inward_sign: f64,
    side_label: &str,
    end_label: &str,
    outboard_plate: ComponentInstanceId,
    outboard_plate_pose: RigidTransform,
    near_plate: ComponentInstanceId,
    near_plate_pose: RigidTransform,
    far_plate: ComponentInstanceId,
    far_plate_pose: RigidTransform,
    branch_shafts: [ComponentInstanceId; 2],
    retention_shaft: ComponentInstanceId,
    gearbox_shafts: [ComponentInstanceId; 3],
) -> Result<(), PrototypeError> {
    let outboard_centers = pitch_contact_outboard_bearing_centers(p)?;
    let near_centers = pitch_contact_carriage_bearing_centers(p)?;
    let far_centers = pitch_gearbox_far_plate_bearing_centers(p)?;
    let outboard_face = if side_sign > 0.0 {
        d.pitch_unit.pitch_contact_outboard_plate.datums.positive_y
    } else {
        d.pitch_unit.pitch_contact_outboard_plate.datums.negative_y
    };
    let near_face = if side_sign > 0.0 {
        d.pitch_unit.contact_carriage_plate.datums.positive_y
    } else {
        d.pitch_unit.contact_carriage_plate.datums.negative_y
    };
    let far_face = if inward_sign > 0.0 {
        d.pitch_unit.pitch_gearbox_far_plate.datums.positive_y
    } else {
        d.pitch_unit.pitch_gearbox_far_plate.datums.negative_y
    };

    let outboard_shafts = [branch_shafts[0], branch_shafts[1], retention_shaft];
    let near_shafts = [
        branch_shafts[0],
        branch_shafts[1],
        gearbox_shafts[0],
        gearbox_shafts[1],
        gearbox_shafts[2],
        retention_shaft,
    ];
    let mut ordinal = 1_u16;
    for index in 0..outboard_centers.len() {
        add_pitch_bearing_support(
            assembly,
            d,
            p,
            pitch_frame,
            base_location.with_ordinal(ordinal),
            &format!(
                "pitch_bearing_{side_label}_{end_label}_outboard_{}",
                index + 1
            ),
            outboard_plate,
            outboard_plate_pose,
            outboard_centers[index],
            d.pitch_unit
                .pitch_contact_outboard_plate
                .datums
                .bearing_bores[index],
            outboard_face,
            outboard_shafts[index],
            pitch_shaft_surface(d, index, true),
            side_sign,
        )?;
        ordinal += 1;
    }
    for index in 0..near_centers.len() {
        add_pitch_bearing_support(
            assembly,
            d,
            p,
            pitch_frame,
            base_location.with_ordinal(ordinal),
            &format!("pitch_bearing_{side_label}_{end_label}_near_{}", index + 1),
            near_plate,
            near_plate_pose,
            near_centers[index],
            d.pitch_unit.contact_carriage_plate.datums.bearing_bores[index],
            near_face,
            near_shafts[index],
            pitch_shaft_surface(d, index, false),
            side_sign,
        )?;
        ordinal += 1;
    }
    for index in 0..far_centers.len() {
        add_pitch_bearing_support(
            assembly,
            d,
            p,
            pitch_frame,
            base_location.with_ordinal(ordinal),
            &format!("pitch_bearing_{side_label}_{end_label}_far_{}", index + 1),
            far_plate,
            far_plate_pose,
            far_centers[index],
            d.pitch_unit.pitch_gearbox_far_plate.datums.bearing_bores[index],
            far_face,
            gearbox_shafts[index],
            d.pitch_unit.pitch_gearbox_shaft.datums.surface,
            inward_sign,
        )?;
        ordinal += 1;
    }
    Ok(())
}

fn pitch_shaft_surface(
    d: &Definitions,
    bearing_index: usize,
    outboard: bool,
) -> DatumId<CylinderDatum> {
    if bearing_index < 2 {
        d.pitch_unit.drive_shaft.datums.surface
    } else if outboard || bearing_index == 5 {
        d.pitch_unit.encoder_shaft.datums.surface
    } else {
        d.pitch_unit.pitch_gearbox_shaft.datums.surface
    }
}

#[allow(clippy::too_many_arguments)]
fn add_pitch_bearing_support(
    assembly: &mut Assembly,
    d: &Definitions,
    p: &PrototypeParameters,
    pitch_frame: FrameId,
    location: ComponentLocation,
    name: &str,
    plate: ComponentInstanceId,
    plate_pose: RigidTransform,
    local_center: [f64; 2],
    plate_bore: DatumId<CylinderDatum>,
    plate_face: DatumId<PlaneDatum>,
    shaft: ComponentInstanceId,
    shaft_surface: DatumId<CylinderDatum>,
    flange_direction: f64,
) -> Result<(), PrototypeError> {
    let flange_offset = flange_direction * p.pitch_gearbox.flanged_bearing_flange_width.mm();
    let flip = if flange_direction > 0.0 { 0.0 } else { PI };
    let bearing_pose = plate_pose
        .compose(RigidTransform::translated(
            local_center[0],
            flange_offset,
            local_center[1],
        ))
        .compose(RigidTransform::rotated(Axis3::X, flip));
    let bearing = add_located_instance(
        assembly,
        name,
        d.pitch_unit.pitch_gearbox_bearing.id,
        pitch_frame,
        bearing_pose,
        location,
    );
    add_pitch_cylindrical_fit(
        assembly,
        shaft,
        shaft_surface,
        bearing,
        d.pitch_unit.pitch_gearbox_bearing.datums.inner_bore,
    )?;
    add_pitch_cylindrical_fit(
        assembly,
        bearing,
        d.pitch_unit.pitch_gearbox_bearing.datums.outer_surface,
        plate,
        plate_bore,
    )?;
    assembly
        .add_relation(AssemblyRelation::SurfaceContact(SurfaceContact {
            first: DatumEndpoint::new(
                bearing,
                d.pitch_unit
                    .pitch_gearbox_bearing
                    .datums
                    .flange_contact_face,
            ),
            second: DatumEndpoint::new(plate, plate_face),
            minimum_contact_area: PositiveArea::square_mm(8.0)
                .expect("bearing flange contact area is positive"),
            tolerance: EngineeringTolerance {
                linear: NonNegativeLength::mm(0.02)
                    .expect("bearing seat tolerance is non-negative"),
                angular: NonNegativeAngle::degrees(0.1)
                    .expect("bearing seat angular tolerance is non-negative"),
            },
        }))
        .map_err(PrototypeError::Assembly)?;
    Ok(())
}

fn add_pitch_cylindrical_fit(
    assembly: &mut Assembly,
    shaft: ComponentInstanceId,
    shaft_surface: DatumId<CylinderDatum>,
    bore: ComponentInstanceId,
    bore_surface: DatumId<CylinderDatum>,
) -> Result<(), PrototypeError> {
    assembly
        .add_relation(AssemblyRelation::CylindricalFit(CylindricalFit {
            shaft: DatumEndpoint::new(shaft, shaft_surface),
            bore: DatumEndpoint::new(bore, bore_surface),
            target_radial_clearance: NonNegativeLength::mm(0.0)
                .expect("nominal bearing fit is non-negative"),
            tolerance: EngineeringTolerance {
                linear: NonNegativeLength::mm(0.02).expect("bearing fit tolerance is non-negative"),
                angular: NonNegativeAngle::degrees(0.1)
                    .expect("bearing fit angular tolerance is non-negative"),
            },
        }))
        .map_err(PrototypeError::Assembly)?;
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
        d.hardware.m3x25_bolt.definition,
        frame,
        pose(bolt_under_head_y, true),
        base_location.with_ordinal(ordinal),
    );
    let nut = add_located_instance(
        assembly,
        &format!("{stem}_nut"),
        d.hardware.m3_nut.definition,
        frame,
        pose(nut_y, true),
        base_location.with_ordinal(ordinal),
    );
    let first_washer = add_located_instance(
        assembly,
        &format!("{stem}_head_washer"),
        d.hardware.m3_washer.definition,
        frame,
        pose(first_washer_y, true),
        base_location.with_ordinal(ordinal * 2 - 1),
    );
    let second_washer = add_located_instance(
        assembly,
        &format!("{stem}_nut_washer"),
        d.hardware.m3_washer.definition,
        frame,
        pose(second_washer_y, true),
        base_location.with_ordinal(ordinal * 2),
    );
    let near_datums = d.pitch_unit.contact_carriage_plate.datums.fasteners[index];
    let far_datums = d.pitch_unit.pitch_gearbox_far_plate.datums.fasteners[index];
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

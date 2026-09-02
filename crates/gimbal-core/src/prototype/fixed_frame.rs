// SPDX-License-Identifier: MIT

use super::*;

pub(super) fn build_pitch_carrier(
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
                definitions.fixed_frame.sector.id,
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
                definitions.fixed_frame.carrier_rail.id,
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
                definitions.fixed_frame.carrier_post.id,
                fixed_frame,
                RigidTransform::translated(x, y, post_center_z),
                ComponentLocation::new()
                    .with_side(side)
                    .with_longitudinal_end(end),
            );
        }
    }
}

pub(super) fn build_crossmembers(
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
            definitions.fixed_frame.crossmember.id,
            world,
            RigidTransform::translated(x, 0.0, z),
            ComponentLocation::new().with_ordinal((index + 1) as u16),
        );
    }
    add_instance(
        assembly,
        "installation_floor_reference",
        definitions.fixed_frame.floor.id,
        world,
        RigidTransform::translated(
            0.0,
            0.0,
            -p.frame.floor_top_below_axis.mm() - p.frame.floor_thickness.mm() * 0.5,
        ),
    );
}

pub(super) fn build_fixed_frame_contacts(
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
            d.fixed_frame.carrier_rail.datums.negative_y,
            floor,
            d.fixed_frame.floor.datums.positive_z,
            1_000.0,
        )?;

        let (crossmember_face, rail_inner_face) = match side {
            Side::Left => (
                d.fixed_frame.crossmember.datums.negative_y,
                d.fixed_frame.carrier_rail.datums.negative_z,
            ),
            Side::Right => (
                d.fixed_frame.crossmember.datums.positive_y,
                d.fixed_frame.carrier_rail.datums.positive_z,
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
                LongitudinalEnd::Front => d.fixed_frame.carrier_post.datums.faces.positive_x,
                LongitudinalEnd::Rear => d.fixed_frame.carrier_post.datums.faces.negative_x,
            };
            add_surface_contact(
                assembly,
                sector,
                d.fixed_frame.sector.datums.mount_face,
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
                d.fixed_frame.carrier_post.datums.faces.positive_z,
                upper_rail,
                d.fixed_frame.carrier_rail.datums.negative_y,
                60.0,
            )?;
            add_surface_contact(
                assembly,
                post,
                d.fixed_frame.carrier_post.datums.faces.negative_z,
                lower_rail,
                d.fixed_frame.carrier_rail.datums.positive_y,
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
            d.fixed_frame.crossmember.datums.negative_z,
            floor,
            d.fixed_frame.floor.datums.positive_z,
            800.0,
        )?;
    }
    Ok(())
}

pub(super) fn required_instance(
    assembly: &Assembly,
    role: ComponentRole,
    location: ComponentLocation,
) -> Result<crate::ComponentInstanceId, PrototypeError> {
    assembly
        .instance_by_identity(ComponentIdentity { role, location })
        .ok_or(PrototypeError::MissingRequiredInstance)
}

pub(super) fn add_surface_contact(
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
    let sector_datums = d.fixed_frame.sector.datums.post_fasteners[sector_hole_index];
    let post_datums = d.fixed_frame.carrier_post.datums.fasteners[post_hole_index];
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
        d.hardware.m3x20_bolt.definition,
        frame,
        RigidTransform::translated(x, bolt_under_head_y, world_z)
            .compose(RigidTransform::rotated(Axis3::Z, hardware_rotation)),
        base_location.with_ordinal(fastener_ordinal),
    );
    let nut = add_located_instance(
        assembly,
        &format!("{stem}_m3_nut"),
        d.hardware.m3_nut.definition,
        frame,
        RigidTransform::translated(x, nut_y, world_z)
            .compose(RigidTransform::rotated(Axis3::Z, hardware_rotation)),
        base_location.with_ordinal(fastener_ordinal),
    );
    let first_washer = add_located_instance(
        assembly,
        &format!("{stem}_head_washer"),
        d.hardware.m3_washer.definition,
        frame,
        RigidTransform::translated(x, first_washer_y, world_z)
            .compose(RigidTransform::rotated(Axis3::Z, hardware_rotation)),
        base_location.with_ordinal(fastener_ordinal * 2 - 1),
    );
    let second_washer = add_located_instance(
        assembly,
        &format!("{stem}_nut_washer"),
        d.hardware.m3_washer.definition,
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

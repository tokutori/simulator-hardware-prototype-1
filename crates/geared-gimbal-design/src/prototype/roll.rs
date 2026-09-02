// SPDX-License-Identifier: MIT

use super::*;

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
        d.roll.roll_shaft,
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
            d.roll.roll_bearing_carrier_end.id,
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
        let end = end.as_str();
        let x = outward * p.roll_axis.bearing_station.mm();
        add_located_instance(
            assembly,
            &format!("roll_bearing_{end}"),
            d.roll.roll_bearing,
            pitch_frame,
            RigidTransform::translated(x, 0.0, 0.0),
            location,
        );
    }
}

pub(super) fn build_moving_carrier_contacts(
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
            d.roll.cockpit.datums.top_face,
            hanger,
            d.roll.cockpit_hanger.datums.cockpit_face,
            100.0,
        )?;
    }

    for end in [LongitudinalEnd::Front, LongitudinalEnd::Rear] {
        let end_location = ComponentLocation::new().with_longitudinal_end(end);
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
                d.roll.roll_bearing_carrier_end.datums.rail_face,
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
                    d.pitch_unit.contact_carriage_plate.datums.positive_y,
                    d.roll.pitch_cradle_longitudinal_rail.datums.negative_y,
                ),
                Side::Right => (
                    d.pitch_unit.contact_carriage_plate.datums.negative_y,
                    d.roll.pitch_cradle_longitudinal_rail.datums.positive_y,
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
                d.roll.roll_bearing_carrier_end.datums.arm_face,
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

pub(super) fn roll_gearbox_plate_solid(
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

pub(super) fn roll_gearbox_support_z(p: &PrototypeParameters) -> f64 {
    let output_z = -(p.roll_axis.driven_gear.pitch_radius() + p.roll_axis.pinion.pitch_radius());
    let stage_distance =
        p.pitch_gearbox.small_gear.pitch_radius() + p.pitch_gearbox.large_gear.pitch_radius();
    output_z - stage_distance * 0.5 + roll_gearbox_plate_support_offset_z()
}

pub(super) const fn roll_gearbox_plate_support_offset_z() -> f64 {
    -24.0
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

pub(super) fn roll_bearing_carrier_end_solid(
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

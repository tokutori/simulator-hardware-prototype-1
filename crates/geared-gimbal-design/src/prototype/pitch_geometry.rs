// SPDX-License-Identifier: MIT

use super::*;

#[derive(Clone, Copy)]
pub(super) struct PitchUnitLayout {
    pub(super) branches: [[f64; 2]; 2],
    pub(super) branch_midpoint: [f64; 2],
    pub(super) distributor: [f64; 2],
    pub(super) compound: [f64; 2],
    pub(super) input: [f64; 2],
    pub(super) plate_center: [f64; 2],
}

#[derive(Clone, Copy)]
pub(super) struct CompliantSolidPair {
    pub(super) manufacturing: SolidId,
    pub(super) assembly: SolidId,
}

#[derive(Clone, Copy)]
enum FlexureState {
    Free,
    Installed,
}

pub(super) fn pitch_unit_layout(
    p: &PrototypeParameters,
) -> Result<PitchUnitLayout, PrototypeError> {
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

pub(super) fn pitch_contact_carriage_bearing_centers(
    p: &PrototypeParameters,
) -> Result<[[f64; 2]; 6], PrototypeError> {
    let layout = pitch_unit_layout(p)?;
    let local = |center: [f64; 2]| {
        [
            center[0] - layout.plate_center[0],
            center[1] - layout.plate_center[1],
        ]
    };
    Ok([
        local(layout.branches[0]),
        local(layout.branches[1]),
        local(layout.distributor),
        local(layout.compound),
        local(layout.input),
        local([
            p.pitch_sector.sector.internal_reference().pitch_radius()
                - p.contact_unit.encoder_pinion.pitch_radius(),
            0.0,
        ]),
    ])
}

pub(super) fn pitch_contact_outboard_bearing_centers(
    p: &PrototypeParameters,
) -> Result<[[f64; 2]; 3], PrototypeError> {
    let layout = pitch_unit_layout(p)?;
    let local = |center: [f64; 2]| {
        [
            center[0] - layout.branch_midpoint[0],
            center[1] - layout.branch_midpoint[1],
        ]
    };
    Ok([
        local(layout.branches[0]),
        local(layout.branches[1]),
        local([
            p.pitch_sector.sector.internal_reference().pitch_radius()
                - p.contact_unit.encoder_pinion.pitch_radius(),
            0.0,
        ]),
    ])
}

pub(super) fn pitch_gearbox_far_plate_bearing_centers(
    p: &PrototypeParameters,
) -> Result<[[f64; 2]; 3], PrototypeError> {
    let layout = pitch_unit_layout(p)?;
    Ok(
        [layout.distributor, layout.compound, layout.input].map(|center| {
            [
                center[0] - layout.plate_center[0],
                center[1] - layout.plate_center[1],
            ]
        }),
    )
}

pub(super) fn pitch_contact_carriage_plate_solids(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
) -> Result<CompliantSolidPair, PrototypeError> {
    Ok(CompliantSolidPair {
        manufacturing: pitch_contact_carriage_plate_solid(builder, p, FlexureState::Free)?,
        assembly: pitch_contact_carriage_plate_solid(builder, p, FlexureState::Installed)?,
    })
}

fn pitch_contact_carriage_plate_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
    flexure_state: FlexureState,
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
    }
    let retention_center = [
        p.pitch_sector.sector.internal_reference().pitch_radius()
            - p.contact_unit.encoder_pinion.pitch_radius()
            - layout.plate_center[0],
        -layout.plate_center[1],
    ];
    plate = add_retention_flexure(
        builder,
        plate,
        retention_center,
        centers[0],
        thickness,
        p,
        flexure_state,
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
    // Cut fastener holes after every structural union. A later flexure or
    // brace union must never refill a previously cut through-hole.
    for tie in pitch_gearbox_tie_points() {
        plate = subtract_y_bore(builder, plate, 1.7, thickness + 2.0, tie[0], tie[1])?;
    }
    let bore_radius = p.pitch_gearbox.flanged_bearing_outer_radius.mm();
    for center in pitch_contact_carriage_bearing_centers(p)? {
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

pub(super) fn pitch_contact_outboard_plate_solids(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
) -> Result<CompliantSolidPair, PrototypeError> {
    Ok(CompliantSolidPair {
        manufacturing: pitch_contact_outboard_plate_solid(builder, p, FlexureState::Free)?,
        assembly: pitch_contact_outboard_plate_solid(builder, p, FlexureState::Installed)?,
    })
}

fn pitch_contact_outboard_plate_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
    flexure_state: FlexureState,
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
    for (a, b) in [(centers[0], centers[2]), (centers[1], centers[2])] {
        let rib = beam_xz(builder, a, b, thickness, 5.0)?;
        plate = builder.boolean(BooleanOperation::Union, plate, rib)?;
    }
    let retention_center = [
        encoder[0] - layout.branch_midpoint[0],
        encoder[1] - layout.branch_midpoint[1],
    ];
    plate = add_retention_flexure(
        builder,
        plate,
        retention_center,
        centers[2],
        thickness,
        p,
        flexure_state,
    )?;
    // Every bearing seat is cut after the complete flexure and plate body is
    // unioned.  Cutting earlier would let a later bridge refill the bore.
    for center in pitch_contact_outboard_bearing_centers(p)? {
        plate = subtract_y_bore(
            builder,
            plate,
            p.pitch_gearbox.flanged_bearing_outer_radius.mm(),
            thickness + 2.0,
            center[0],
            center[1],
        )?;
    }
    subtract_y_bore(builder, plate, 1.7, thickness + 2.0, 0.0, 0.0)
}

pub(super) fn pitch_gearbox_far_plate_solid(
    builder: &mut FeatureBuilder,
    p: &PrototypeParameters,
) -> Result<SolidId, PrototypeError> {
    let thickness = p.pitch_gearbox.side_plate_thickness.mm();
    let centers = pitch_gearbox_far_plate_bearing_centers(p)?;
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
    }
    for tie in pitch_gearbox_tie_points() {
        plate = subtract_y_bore(builder, plate, 1.7, thickness + 2.0, tie[0], tie[1])?;
    }
    let bore_radius = p.pitch_gearbox.flanged_bearing_outer_radius.mm();
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

pub(super) const fn pitch_gearbox_tie_points() -> [[f64; 2]; 3] {
    [[-24.0, -24.0], [26.0, -18.0], [0.0, 38.0]]
}

pub(super) fn pitch_gearbox_plate_fastener_datums(
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

fn add_retention_flexure(
    builder: &mut FeatureBuilder,
    mut plate: SolidId,
    bearing_center: [f64; 2],
    rigid_anchor: [f64; 2],
    thickness: f64,
    p: &PrototypeParameters,
    state: FlexureState,
) -> Result<SolidId, PrototypeError> {
    let contact = &p.contact_unit;
    let length = contact.retention_flexure_length.mm();
    let beam_width = contact.retention_flexure_beam_width.mm();
    let bridge_width = contact.retention_flexure_bridge_width.mm();
    let island_radius = contact.retention_bearing_island_radius.mm();
    let beam_offset = island_radius * 0.65;
    let free_center = [
        bearing_center[0] + contact.retention_installed_deflection.as_mm(),
        bearing_center[1],
    ];
    let moving_center = match state {
        FlexureState::Free => free_center,
        FlexureState::Installed => bearing_center,
    };
    // The fixed bridge belongs to the rigid plate and therefore remains at
    // the free-state radial coordinate in both shape states.
    let fixed_bridge_center = [free_center[0], bearing_center[1] - length];

    let island = cylinder_y(builder, island_radius, thickness)?;
    let island = builder.translate(
        island,
        Translation3 {
            x: moving_center[0],
            y: 0.0,
            z: moving_center[1],
        },
    )?;
    plate = builder.boolean(BooleanOperation::Union, plate, island)?;

    for offset in [-beam_offset, beam_offset] {
        let beam = flexure_beam_xz(
            builder,
            [fixed_bridge_center[0] + offset, fixed_bridge_center[1]],
            [moving_center[0] + offset, moving_center[1]],
            thickness,
            beam_width,
        )?;
        plate = builder.boolean(BooleanOperation::Union, plate, beam)?;
    }
    let moving_bridge = beam_xz(
        builder,
        [moving_center[0] - beam_offset, moving_center[1]],
        [moving_center[0] + beam_offset, moving_center[1]],
        thickness,
        bridge_width,
    )?;
    plate = builder.boolean(BooleanOperation::Union, plate, moving_bridge)?;
    let fixed_bridge = beam_xz(
        builder,
        [bearing_center[0] - beam_offset, fixed_bridge_center[1]],
        [bearing_center[0] + beam_offset, fixed_bridge_center[1]],
        thickness,
        bridge_width,
    )?;
    plate = builder.boolean(BooleanOperation::Union, plate, fixed_bridge)?;
    let anchor_rib = beam_xz(
        builder,
        fixed_bridge_center,
        rigid_anchor,
        thickness,
        bridge_width,
    )?;
    plate = builder.boolean(BooleanOperation::Union, plate, anchor_rib)?;

    Ok(plate)
}

fn flexure_beam_xz(
    builder: &mut FeatureBuilder,
    fixed: [f64; 2],
    moving: [f64; 2],
    thickness_y: f64,
    width: f64,
) -> Result<SolidId, PrototypeError> {
    const SEGMENTS: usize = 8;
    let point = |index: usize| {
        let u = index as f64 / SEGMENTS as f64;
        let smooth = 3.0 * u * u - 2.0 * u * u * u;
        [
            fixed[0] + (moving[0] - fixed[0]) * smooth,
            fixed[1] + (moving[1] - fixed[1]) * u,
        ]
    };
    let mut beam = beam_xz(builder, point(0), point(1), thickness_y, width)?;
    for segment in 1..SEGMENTS {
        let next = beam_xz(
            builder,
            point(segment),
            point(segment + 1),
            thickness_y,
            width,
        )?;
        beam = builder.boolean(BooleanOperation::Union, beam, next)?;
    }
    Ok(beam)
}

pub(super) fn subtract_y_bore(
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

pub(super) fn beam_xz(
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

pub(super) fn beam_yz(
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

pub(super) fn dual_sector_solid(
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

pub(super) fn sector_spine_inner_x(p: &PrototypeParameters) -> f64 {
    p.frame.fixed_crossmember_station.mm() + p.frame.fixed_crossmember_width.mm() * 0.5
}

pub(super) const fn sector_support_keep_out_half_height() -> f64 {
    40.0
}

pub(super) fn sector_post_hole_zs(p: &PrototypeParameters) -> [f64; 2] {
    let upper_support_end = p.frame.upper_rail_height.mm() - p.frame.fixed_rail_depth.mm() * 0.5;
    let positive_z = (sector_support_keep_out_half_height() + upper_support_end) * 0.5;
    [positive_z, -positive_z]
}

pub(super) const fn m3_clearance_radius_mm() -> f64 {
    1.7
}

pub(super) fn sector_post_fastener_datums(
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

pub(super) fn sector_wedge_points(tip_radius: f64, half_angle: f64) -> Vec<Point2> {
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

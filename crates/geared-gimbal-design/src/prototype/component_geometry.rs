// SPDX-License-Identifier: MIT

use super::*;

#[derive(Clone, Copy)]
pub(super) struct DefinitionStyle {
    pub(super) role: ComponentRole,
    pub(super) color: [f32; 4],
}

pub(super) const fn definition_style(role: ComponentRole, color: [f32; 4]) -> DefinitionStyle {
    DefinitionStyle { role, color }
}

pub(super) fn gear_definition_y(
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

pub(super) fn gear_definition_x(
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

pub(super) fn gear_solid_z(
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

pub(super) fn annulus_definition_y(
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

pub(super) fn annulus_solid_x(
    builder: &mut FeatureBuilder,
    outer_radius: f64,
    inner_radius: f64,
    width: f64,
) -> Result<SolidId, PrototypeError> {
    let outer = cylinder_x(builder, outer_radius, width)?;
    let inner = cylinder_x(builder, inner_radius, width + 2.0)?;
    builder
        .boolean(BooleanOperation::Difference, outer, inner)
        .map_err(PrototypeError::Feature)
}

pub(super) fn carrier_post_definition(
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

    let (mut datums, faces) =
        box_plane_datums(assembly.next_definition_id(), [width, depth, height]);
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

pub(super) fn m3_pan_head_bolt_solid(
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

pub(super) fn add_m3_bolt_definition(
    builder: &mut FeatureBuilder,
    assembly: &mut Assembly,
    shank_length: f64,
) -> Result<M3BoltDefinition, PrototypeError> {
    let owner = assembly.next_definition_id();
    let mut datums = DatumSet::for_definition(owner);
    let axis = add_axis_datum(&mut datums, "bolt_axis", [0.0; 3], [1.0, 0.0, 0.0]);
    let under_head_face = add_plane_datum(
        &mut datums,
        "under_head_bearing_face",
        [0.0; 3],
        [-1.0, 0.0, 0.0],
    );
    let shank_tip_face = add_plane_datum(
        &mut datums,
        "shank_tip_face",
        [-shank_length, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
    );
    let definition = add_solid_definition_with_datums(
        assembly,
        &format!("m3x{shank_length:.0}_pan_head_bolt"),
        ComponentRole::M3Bolt,
        m3_pan_head_bolt_solid(builder, shank_length)?,
        Manufacturing::Purchased,
        [0.68, 0.71, 0.74, 1.0],
        datums,
    );
    Ok(M3BoltDefinition {
        definition,
        axis,
        under_head_face,
        shank_tip_face,
    })
}

pub(super) fn m3_hex_nut_solid(builder: &mut FeatureBuilder) -> Result<SolidId, PrototypeError> {
    const THICKNESS: f64 = 2.4;
    // 3.175 mm circumradius gives approximately 5.5 mm across flats.
    let outer = cylinder_x_segments(builder, 3.175, THICKNESS, 6)?;
    let bore = cylinder_x(builder, 1.6, THICKNESS + 2.0)?;
    builder
        .boolean(BooleanOperation::Difference, outer, bore)
        .map_err(PrototypeError::Feature)
}

pub(super) fn add_m3_nut_definition(
    builder: &mut FeatureBuilder,
    assembly: &mut Assembly,
) -> Result<M3NutDefinition, PrototypeError> {
    const THICKNESS: f64 = 2.4;
    let owner = assembly.next_definition_id();
    let mut datums = DatumSet::for_definition(owner);
    let axis = add_axis_datum(&mut datums, "nut_axis", [0.0; 3], [1.0, 0.0, 0.0]);
    let negative_x_face = add_plane_datum(
        &mut datums,
        "negative_x_bearing_face",
        [-THICKNESS * 0.5, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
    );
    let positive_x_face = add_plane_datum(
        &mut datums,
        "positive_x_bearing_face",
        [THICKNESS * 0.5, 0.0, 0.0],
        [1.0, 0.0, 0.0],
    );
    let definition = add_solid_definition_with_datums(
        assembly,
        "m3_hex_nut",
        ComponentRole::M3Nut,
        m3_hex_nut_solid(builder)?,
        Manufacturing::Purchased,
        [0.60, 0.63, 0.66, 1.0],
        datums,
    );
    Ok(M3NutDefinition {
        definition,
        axis,
        negative_x_face,
        positive_x_face,
    })
}

pub(super) fn m3_washer_solid(builder: &mut FeatureBuilder) -> Result<SolidId, PrototypeError> {
    const THICKNESS: f64 = 0.5;
    let outer = cylinder_x(builder, 3.5, THICKNESS)?;
    let bore = cylinder_x(builder, m3_clearance_radius_mm(), THICKNESS + 2.0)?;
    builder
        .boolean(BooleanOperation::Difference, outer, bore)
        .map_err(PrototypeError::Feature)
}

pub(super) fn add_m3_washer_definition(
    builder: &mut FeatureBuilder,
    assembly: &mut Assembly,
) -> Result<M3WasherDefinition, PrototypeError> {
    const THICKNESS: f64 = 0.5;
    let owner = assembly.next_definition_id();
    let mut datums = DatumSet::for_definition(owner);
    let axis = add_axis_datum(&mut datums, "washer_axis", [0.0; 3], [1.0, 0.0, 0.0]);
    let negative_x_face = add_plane_datum(
        &mut datums,
        "negative_x_face",
        [-THICKNESS * 0.5, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
    );
    let positive_x_face = add_plane_datum(
        &mut datums,
        "positive_x_face",
        [THICKNESS * 0.5, 0.0, 0.0],
        [1.0, 0.0, 0.0],
    );
    let definition = add_solid_definition_with_datums(
        assembly,
        "m3_plain_washer",
        ComponentRole::M3Washer,
        m3_washer_solid(builder)?,
        Manufacturing::Purchased,
        [0.72, 0.75, 0.78, 1.0],
        datums,
    );
    Ok(M3WasherDefinition {
        definition,
        axis,
        negative_x_face,
        positive_x_face,
    })
}

pub(super) fn sheet_box_definition_with_faces(
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
    let (datums, faces) = box_plane_datums(
        assembly.next_definition_id(),
        [width, height, thickness.mm()],
    );
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

pub(super) fn add_box_definition_with_faces(
    builder: &mut FeatureBuilder,
    assembly: &mut Assembly,
    name: &str,
    role: ComponentRole,
    size: [f64; 3],
    manufacturing: Manufacturing,
    color_rgba: [f32; 4],
) -> (ComponentDefinitionId, BoxPlaneDatums) {
    let (datums, faces) = box_plane_datums(assembly.next_definition_id(), size);
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

pub(super) fn box_plane_datums(
    owner: ComponentDefinitionId,
    size: [f64; 3],
) -> (DatumSet, BoxPlaneDatums) {
    let mut datums = DatumSet::for_definition(owner);
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

pub(super) fn add_plane_datum(
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

pub(super) fn add_axis_datum(
    datums: &mut DatumSet,
    name: &str,
    origin: [f64; 3],
    direction: [f64; 3],
) -> DatumId<AxisDatum> {
    datums.add(
        name.to_string(),
        AxisDatum {
            origin: Point3::from_mm(origin).expect("axis origin is finite"),
            direction: UnitVector3::new(direction).expect("axis direction is non-zero"),
        },
    )
}

pub(super) fn add_cylinder_datum(
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

pub(super) fn cylinder_y(
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

pub(super) fn cylinder_x(
    builder: &mut FeatureBuilder,
    radius: f64,
    height: f64,
) -> Result<SolidId, PrototypeError> {
    cylinder_x_segments(builder, radius, height, 64)
}

pub(super) fn cylinder_x_segments(
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

pub(super) fn centered_box(builder: &mut FeatureBuilder, size: [f64; 3]) -> SolidId {
    builder.primitive(Primitive3::Box {
        x: length(size[0]),
        y: length(size[1]),
        z: length(size[2]),
        centered: true,
    })
}

pub(super) fn add_solid_definition(
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

pub(super) fn add_solid_definition_with_datums(
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

pub(super) fn add_instance(
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

pub(super) fn add_located_instance(
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

pub(super) fn revolute_frame(
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

pub(super) fn rectangle_points(width: f64, height: f64) -> Vec<Point2> {
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

pub(super) fn distance2(a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    libm::sqrt(dx * dx + dy * dy)
}

pub(super) fn length(value: f64) -> Length {
    Length::positive_mm(value).expect("derived prototype length must be positive")
}

pub(super) fn angle(degrees: f64) -> Angle {
    Angle::degrees(degrees).expect("derived prototype angle must be finite")
}

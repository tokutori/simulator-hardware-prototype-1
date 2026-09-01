// SPDX-License-Identifier: MIT

use double_helical_core::{
    DoubleHelicalGear, DoubleHelicalRack, GearPose, Prototype, SpurGear, TriangleMesh,
};
use manifold_rust::cross_section::CrossSection;
use manifold_rust::linalg::{Mat3x4, Vec2, Vec3};
use manifold_rust::manifold::Manifold;
use manifold_rust::types::{BooleanEngine, Error as ManifoldStatus};
use thiserror::Error;

#[derive(Clone, Debug, PartialEq)]
pub struct PrototypeMeshes {
    pub handle_spur: TriangleMesh,
    pub handle_crank: TriangleMesh,
    pub handle_knob: TriangleMesh,
    pub reduction_compound: TriangleMesh,
    pub driven_b_compound: TriangleMesh,
    pub driven_c_compound: TriangleMesh,
    pub idler_pinion: TriangleMesh,
    pub rack: TriangleMesh,
    pub top_plate: TriangleMesh,
    pub bottom_plate: TriangleMesh,
    pub handle_upper_thrust_spacer: TriangleMesh,
    pub reduction_upper_thrust_spacer: TriangleMesh,
    pub driven_lower_thrust_spacer: TriangleMesh,
    pub idler_lower_thrust_spacer: TriangleMesh,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrototypeInterference {
    pub handle_to_reduction_large_mm3: f64,
    pub reduction_small_to_b_mm3: f64,
    pub reduction_small_to_c_mm3: f64,
    pub driven_b_pinion_to_rack_mm3: f64,
    pub driven_c_pinion_to_rack_mm3: f64,
    pub idler_pinion_to_rack_mm3: f64,
}

#[derive(Debug, Error)]
pub enum KernelError {
    #[error("geometry kernel returned {0:?}")]
    Manifold(ManifoldStatus),
    #[error("geometry operation produced an empty solid")]
    EmptySolid,
}

pub fn build_prototype(prototype: &Prototype) -> Result<PrototypeMeshes, KernelError> {
    let handle_spur = handle_shaft_spur_solid(prototype)?;
    let handle_crank = handle_crank_solid(prototype)?;
    let handle_knob = tube(
        prototype.rack().face_width().mm() + 5.0,
        prototype.journal_outer_diameter().mm() * 1.5,
        prototype.bolt_clearance_diameter().mm(),
    )?;
    let reduction_compound = reduction_compound_solid(prototype)?;
    let driven_b_compound =
        driven_compound_solid(prototype, prototype.driven_b_internal_pinion_rotation_deg())?;
    let driven_c_compound =
        driven_compound_solid(prototype, prototype.driven_c_internal_pinion_rotation_deg())?;
    let idler = double_helical_gear_solid(prototype.idler_pinion())?;
    let rack = rack_solid(prototype.rack())?;
    let top_plate = frame_plate(prototype, PlateSide::Top)?;
    let bottom_plate = frame_plate(prototype, PlateSide::Bottom)?;
    let thrust_outer = prototype.thrust_spacer_outer_diameter().mm();
    let thrust_inner = prototype.driven_pinion().bore_diameter().mm();
    let handle_upper_thrust_spacer = tube(
        prototype.handle_upper_thrust_spacer_length(),
        thrust_outer,
        thrust_inner,
    )?;
    let reduction_upper_thrust_spacer = tube(
        prototype.reduction_upper_thrust_spacer_length(),
        thrust_outer,
        thrust_inner,
    )?;
    let driven_lower_thrust_spacer = tube(
        prototype.driven_lower_thrust_spacer_length(),
        thrust_outer,
        thrust_inner,
    )?;
    let idler_lower_thrust_spacer = tube(
        prototype.idler_lower_thrust_spacer_length(),
        thrust_outer,
        thrust_inner,
    )?;
    Ok(PrototypeMeshes {
        handle_spur: mesh_from_manifold(&handle_spur),
        handle_crank: mesh_from_manifold(&handle_crank),
        handle_knob: mesh_from_manifold(&handle_knob),
        reduction_compound: mesh_from_manifold(&reduction_compound),
        driven_b_compound: mesh_from_manifold(&driven_b_compound),
        driven_c_compound: mesh_from_manifold(&driven_c_compound),
        idler_pinion: mesh_from_manifold(&idler),
        rack: mesh_from_manifold(&rack),
        top_plate: mesh_from_manifold(&top_plate),
        bottom_plate: mesh_from_manifold(&bottom_plate),
        handle_upper_thrust_spacer: mesh_from_manifold(&handle_upper_thrust_spacer),
        reduction_upper_thrust_spacer: mesh_from_manifold(&reduction_upper_thrust_spacer),
        driven_lower_thrust_spacer: mesh_from_manifold(&driven_lower_thrust_spacer),
        idler_lower_thrust_spacer: mesh_from_manifold(&idler_lower_thrust_spacer),
    })
}

pub fn prototype_interference(prototype: &Prototype) -> Result<PrototypeInterference, KernelError> {
    let handle = transform(
        spur_solid(
            prototype.handle_spur(),
            prototype.spur_face_width().mm(),
            prototype.driven_pinion().bore_diameter().mm(),
        )?,
        prototype.handle_spur_pose(),
    );
    let reduction_large = transform(
        spur_solid(
            prototype.reduction_large_spur(),
            prototype.spur_face_width().mm(),
            prototype.driven_pinion().bore_diameter().mm(),
        )?,
        prototype.reduction_large_spur_pose(),
    );
    let reduction_small = transform(
        spur_solid(
            prototype.reduction_small_spur(),
            prototype.spur_face_width().mm(),
            prototype.driven_pinion().bore_diameter().mm(),
        )?,
        prototype.reduction_small_spur_pose(),
    );
    let output_b = transform(
        spur_solid(
            prototype.output_spur(),
            prototype.spur_face_width().mm(),
            prototype.driven_pinion().bore_diameter().mm(),
        )?,
        prototype.output_b_spur_pose(),
    );
    let output_c = transform(
        spur_solid(
            prototype.output_spur(),
            prototype.spur_face_width().mm(),
            prototype.driven_pinion().bore_diameter().mm(),
        )?,
        prototype.output_c_spur_pose(),
    );
    let driven_b = transform(
        double_helical_gear_solid(prototype.driven_pinion())?,
        prototype.driven_b_pinion_pose(),
    );
    let driven_c = transform(
        double_helical_gear_solid(prototype.driven_pinion())?,
        prototype.driven_c_pinion_pose(),
    );
    let idler = transform(
        double_helical_gear_solid(prototype.idler_pinion())?,
        prototype.idler_pose(),
    );
    let rack = rack_solid(prototype.rack())?;
    Ok(PrototypeInterference {
        handle_to_reduction_large_mm3: intersection_volume(&handle, &reduction_large)?,
        reduction_small_to_b_mm3: intersection_volume(&reduction_small, &output_b)?,
        reduction_small_to_c_mm3: intersection_volume(&reduction_small, &output_c)?,
        driven_b_pinion_to_rack_mm3: intersection_volume(&driven_b, &rack)?,
        driven_c_pinion_to_rack_mm3: intersection_volume(&driven_c, &rack)?,
        idler_pinion_to_rack_mm3: intersection_volume(&idler, &rack)?,
    })
}

fn double_helical_gear_solid(gear: &DoubleHelicalGear) -> Result<Manifold, KernelError> {
    let section = gear_cross_section(gear.spur(), gear.bore_diameter().mm());
    let band_width = gear.tooth_band_width();
    let twist = gear.half_twist_degrees() * gear.lower_twist_sign();
    let divisions = i32::from(gear.slices_per_half()) - 1;

    let lower = Manifold::extrude(&section, band_width, divisions, twist, Vec2::new(1.0, 1.0))
        .rotate(0.0, 0.0, -twist)
        .translate(Vec3::new(0.0, 0.0, -gear.face_width().mm() * 0.5));
    let upper = Manifold::extrude(&section, band_width, divisions, -twist, Vec2::new(1.0, 1.0))
        .translate(Vec3::new(0.0, 0.0, gear.center_gap().mm() * 0.5));

    // Only bridge the tooth-free center relief.  Each helical band already
    // contains its own full root disc, so a full-width cylinder would create
    // a needlessly expensive and fragile boolean against every tooth face.
    let bridge_radius = gear.spur().root_radius() + 0.05;
    let bridge_section = annulus(bridge_radius * 2.0, gear.bore_diameter().mm());
    let overlap = 0.10;
    let bridge = Manifold::extrude(
        &bridge_section,
        gear.center_gap().mm() + overlap * 2.0,
        0,
        0.0,
        Vec2::new(1.0, 1.0),
    )
    .translate(Vec3::new(0.0, 0.0, -gear.center_gap().mm() * 0.5 - overlap));
    let result = lower
        .union_with_engine(&upper, BooleanEngine::Exact)
        .union_with_engine(&bridge, BooleanEngine::Exact);
    validate_solid(&result)?;
    Ok(result)
}

fn spur_solid(
    gear: &SpurGear,
    face_width_mm: f64,
    bore_diameter_mm: f64,
) -> Result<Manifold, KernelError> {
    let section = gear_cross_section(gear, bore_diameter_mm);
    let result = Manifold::extrude(&section, face_width_mm, 0, 0.0, Vec2::new(1.0, 1.0))
        .translate(Vec3::new(0.0, 0.0, -face_width_mm * 0.5));
    validate_solid(&result)?;
    Ok(result)
}

fn handle_shaft_spur_solid(prototype: &Prototype) -> Result<Manifold, KernelError> {
    let gear = spur_solid(
        prototype.handle_spur(),
        prototype.spur_face_width().mm(),
        0.10,
    )?;
    let gear_center_z = prototype.handle_spur_pose().translation_mm[2];
    let shaft_bottom =
        prototype.bottom_plate_center_z() - prototype.plate_thickness().mm() * 0.5 - gear_center_z;
    let shaft_top =
        prototype.top_plate_center_z() + prototype.plate_thickness().mm() * 0.5 - gear_center_z;
    let shaft_radius = prototype.journal_outer_diameter().mm() * 0.5;
    let shaft = Manifold::cylinder(
        shaft_top - shaft_bottom,
        shaft_radius,
        shaft_radius,
        circular_segments(shaft_radius),
    )
    .translate(Vec3::new(0.0, 0.0, shaft_bottom));
    let overlap = 0.10;
    let taper_height = prototype.handle_support_taper_height();
    let taper = Manifold::cylinder(
        taper_height + overlap,
        shaft_radius,
        prototype.handle_spur().root_radius(),
        circular_segments(prototype.handle_spur().root_radius()),
    )
    .translate(Vec3::new(0.0, 0.0, shaft_bottom));
    let drive_height =
        prototype.nut_thickness().mm() * 2.0 + prototype.axial_clearance().mm() + 1.0;
    let drive_size = prototype.journal_outer_diameter().mm();
    let square_drive = Manifold::cube(
        Vec3::new(drive_size, drive_size, drive_height + overlap),
        true,
    )
    .translate(Vec3::new(
        0.0,
        0.0,
        shaft_top + drive_height * 0.5 - overlap * 0.5,
    ));
    let result = gear
        .union_with_engine(&shaft, BooleanEngine::Exact)
        .union_with_engine(&taper, BooleanEngine::Exact)
        .union_with_engine(&square_drive, BooleanEngine::Exact);
    validate_solid(&result)?;
    Ok(result)
}

fn handle_crank_solid(prototype: &Prototype) -> Result<Manifold, KernelError> {
    let radius = prototype.handle_crank_radius().mm();
    let thickness = prototype.nut_thickness().mm();
    let arm_width = prototype.journal_outer_diameter().mm() + 2.0;
    let hub_radius = prototype.journal_outer_diameter().mm();
    let end_radius = arm_width * 0.5;
    let hub = Manifold::cylinder(
        thickness,
        hub_radius,
        hub_radius,
        circular_segments(hub_radius),
    )
    .translate(Vec3::new(0.0, 0.0, -thickness * 0.5));
    let arm = Manifold::cube(Vec3::new(radius, arm_width, thickness), true).translate(Vec3::new(
        radius * 0.5,
        0.0,
        0.0,
    ));
    let end = Manifold::cylinder(
        thickness,
        end_radius,
        end_radius,
        circular_segments(end_radius),
    )
    .translate(Vec3::new(radius, 0.0, -thickness * 0.5));
    let socket_size = prototype.journal_outer_diameter().mm() + 0.30;
    let socket = Manifold::cube(Vec3::new(socket_size, socket_size, thickness + 0.20), true);
    let knob_radius = prototype.bolt_clearance_diameter().mm() * 0.5;
    let knob_hole = Manifold::cylinder(
        thickness + 0.20,
        knob_radius,
        knob_radius,
        circular_segments(knob_radius),
    )
    .translate(Vec3::new(radius, 0.0, -thickness * 0.5 - 0.10));
    let result = hub
        .union_with_engine(&arm, BooleanEngine::Exact)
        .union_with_engine(&end, BooleanEngine::Exact)
        .difference_with_engine(&socket, BooleanEngine::Exact)
        .difference_with_engine(&knob_hole, BooleanEngine::Exact);
    validate_solid(&result)?;
    Ok(result)
}

fn rack_solid(rack: &DoubleHelicalRack) -> Result<Manifold, KernelError> {
    let polygon = rack
        .profile()
        .points
        .into_iter()
        .map(|point| Vec2::new(point.x, point.y))
        .collect::<Vec<_>>();
    let section = vec![polygon];
    let band_width = rack.tooth_band_width();
    let shear = libm::tan(rack.system().helix_angle().as_radians()) * rack.lower_shift_sign();

    let lower = Manifold::extrude(
        &section,
        band_width,
        i32::from(rack.slices_per_half()) - 1,
        0.0,
        Vec2::new(1.0, 1.0),
    )
    .transform(&shear_x_by_z(
        shear,
        -shear * band_width,
        -rack.face_width().mm() * 0.5,
    ));
    let upper = Manifold::extrude(
        &section,
        band_width,
        i32::from(rack.slices_per_half()) - 1,
        0.0,
        Vec2::new(1.0, 1.0),
    )
    .transform(&shear_x_by_z(-shear, 0.0, rack.center_gap().mm() * 0.5));
    let overlap = 0.10;
    let bridge = Manifold::cube(
        Vec3::new(
            rack.length(),
            rack.body_thickness().mm() + 0.10,
            rack.center_gap().mm() + overlap * 2.0,
        ),
        true,
    );
    let result = lower
        .union_with_engine(&upper, BooleanEngine::Exact)
        .union_with_engine(&bridge, BooleanEngine::Exact);
    validate_solid(&result)?;
    Ok(result)
}

fn driven_compound_solid(
    prototype: &Prototype,
    internal_pinion_rotation_deg: f64,
) -> Result<Manifold, KernelError> {
    let pinion = double_helical_gear_solid(prototype.driven_pinion())?;
    let lower_extension = lower_helical_extension_solid(
        prototype.driven_pinion(),
        prototype.pinion_lower_extension().mm(),
    )?;
    let extended_pinion = pinion
        .union_with_engine(&lower_extension, BooleanEngine::Exact)
        .rotate(0.0, 0.0, internal_pinion_rotation_deg);
    let output_spur = spur_solid(
        prototype.output_spur(),
        prototype.spur_face_width().mm(),
        prototype.driven_pinion().bore_diameter().mm(),
    )?
    .translate(Vec3::new(
        0.0,
        0.0,
        prototype.secondary_spur_layer_center_z(),
    ));

    let result = extended_pinion.union_with_engine(&output_spur, BooleanEngine::Exact);
    validate_solid(&result)?;
    Ok(result)
}

fn lower_helical_extension_solid(
    gear: &DoubleHelicalGear,
    extension_height: f64,
) -> Result<Manifold, KernelError> {
    let section = gear_cross_section(gear.spur(), gear.bore_diameter().mm());
    let band_width = gear.tooth_band_width();
    let twist_per_mm = gear.half_twist_degrees() * gear.lower_twist_sign() / band_width;
    let overlap = 0.10;
    let extrusion_height = extension_height + overlap;
    let extension_twist = twist_per_mm * extension_height;
    let extrusion_twist = twist_per_mm * extrusion_height;
    let divisions = ((f64::from(gear.slices_per_half() - 1) * extrusion_height / band_width).ceil()
        as i32)
        .max(1);
    let extension = Manifold::extrude(
        &section,
        extrusion_height,
        divisions,
        extrusion_twist,
        Vec2::new(1.0, 1.0),
    )
    .rotate(
        0.0,
        0.0,
        -gear.half_twist_degrees() * gear.lower_twist_sign() - extension_twist,
    )
    .translate(Vec3::new(
        0.0,
        0.0,
        -gear.face_width().mm() * 0.5 - extension_height,
    ));
    validate_solid(&extension)?;
    Ok(extension)
}

fn reduction_compound_solid(prototype: &Prototype) -> Result<Manifold, KernelError> {
    let bore = prototype.driven_pinion().bore_diameter().mm();
    let small = spur_solid(
        prototype.reduction_small_spur(),
        prototype.spur_face_width().mm(),
        bore,
    )?
    .translate(Vec3::new(
        0.0,
        0.0,
        prototype.secondary_spur_layer_center_z(),
    ));
    let large = spur_solid(
        prototype.reduction_large_spur(),
        prototype.spur_face_width().mm(),
        bore,
    )?
    .translate(Vec3::new(0.0, 0.0, prototype.primary_spur_layer_center_z()));
    let overlap = 0.10;
    let large_top =
        prototype.primary_spur_layer_center_z() + prototype.spur_face_width().mm() * 0.5;
    let small_bottom =
        prototype.secondary_spur_layer_center_z() - prototype.spur_face_width().mm() * 0.5;
    let hub = Manifold::extrude(
        &annulus(bore + 6.0, bore),
        small_bottom - large_top + overlap * 2.0,
        0,
        0.0,
        Vec2::new(1.0, 1.0),
    )
    .translate(Vec3::new(0.0, 0.0, large_top - overlap));
    let result = small
        .union_with_engine(&large, BooleanEngine::Exact)
        .union_with_engine(&hub, BooleanEngine::Exact);
    validate_solid(&result)?;
    Ok(result)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PlateSide {
    Top,
    Bottom,
}

fn frame_plate(prototype: &Prototype, side: PlateSide) -> Result<Manifold, KernelError> {
    let mut section = CrossSection::square_vec2(
        Vec2::new(prototype.plate_length().mm(), prototype.plate_width().mm()),
        true,
    )
    .translate(Vec2::new(0.0, prototype.plate_center_y()));
    let handle = prototype.handle_spur_pose();
    let reduction = prototype.reduction_pose();
    let driven_b = prototype.driven_b_pose();
    let driven_c = prototype.driven_c_pose();
    let idler = prototype.idler_pose();
    let fixed_axles = vec![
        [reduction.translation_mm[0], reduction.translation_mm[1]],
        [driven_b.translation_mm[0], driven_b.translation_mm[1]],
        [driven_c.translation_mm[0], driven_c.translation_mm[1]],
        [idler.translation_mm[0], idler.translation_mm[1]],
    ];
    let corners = prototype.corner_positions();
    let mut bolt_positions = fixed_axles.clone();
    bolt_positions.extend(corners.iter().copied());
    if side == PlateSide::Top {
        let handle_hole = CrossSection::circle(
            prototype.driven_pinion().bore_diameter().mm() * 0.5,
            circular_segments(prototype.driven_pinion().bore_diameter().mm() * 0.5),
        )
        .translate(Vec2::new(
            handle.translation_mm[0],
            handle.translation_mm[1],
        ));
        section = section.difference(&handle_hole);
    }
    for [x, y] in &bolt_positions {
        let hole = CrossSection::circle(
            prototype.bolt_clearance_diameter().mm() * 0.5,
            circular_segments(prototype.bolt_clearance_diameter().mm() * 0.5),
        )
        .translate(Vec2::new(*x, *y));
        section = section.difference(&hole);
    }
    let mut result = Manifold::extrude(
        &section.to_polygons(),
        prototype.plate_thickness().mm(),
        0,
        0.0,
        Vec2::new(1.0, 1.0),
    )
    .translate(Vec3::new(0.0, 0.0, -prototype.plate_thickness().mm() * 0.5));
    if side == PlateSide::Bottom {
        let shaft_radius = prototype.journal_outer_diameter().mm() * 0.5;
        let taper_top_radius = prototype.handle_spur().root_radius();
        let taper_height = prototype.handle_support_taper_height();
        let radial_clearance = prototype.top_socket_diameter_clearance().mm() * 0.5;
        let plate_thickness = prototype.plate_thickness().mm();
        let clearance_top_radius = shaft_radius
            + (taper_top_radius - shaft_radius) * plate_thickness / taper_height
            + radial_clearance;
        let handle_clearance = Manifold::cylinder(
            plate_thickness + 0.10,
            shaft_radius + radial_clearance,
            clearance_top_radius,
            circular_segments(clearance_top_radius),
        )
        .translate(Vec3::new(
            handle.translation_mm[0],
            handle.translation_mm[1],
            -plate_thickness * 0.5 - 0.05,
        ));
        result = result.difference_with_engine(&handle_clearance, BooleanEngine::Exact);
        let pocket_depth = prototype.nut_pocket_depth().mm();
        let pocket_radius = prototype.nut_across_flats().mm() / 3.0_f64.sqrt();
        let pocket_section = CrossSection::circle(pocket_radius, 6);
        for [x, y] in &bolt_positions {
            let pocket = Manifold::extrude(
                &pocket_section.translate(Vec2::new(*x, *y)).to_polygons(),
                pocket_depth + 0.05,
                0,
                0.0,
                Vec2::new(1.0, 1.0),
            )
            .translate(Vec3::new(
                0.0,
                0.0,
                -prototype.plate_thickness().mm() * 0.5 - 0.05,
            ));
            result = result.difference_with_engine(&pocket, BooleanEngine::Exact);
        }
        let overlap = 0.10;
        let post_height = prototype.fixed_post_length() + overlap;
        let post_center_z = prototype.plate_thickness().mm() * 0.5 - overlap + post_height * 0.5;
        for [x, y] in fixed_axles {
            let post = tube(
                post_height,
                prototype.journal_outer_diameter().mm(),
                prototype.bolt_clearance_diameter().mm(),
            )?
            .translate(Vec3::new(x, y, post_center_z));
            result = result.union_with_engine(&post, BooleanEngine::Exact);
        }
        for [x, y] in corners {
            let post = tube(
                post_height,
                prototype.journal_outer_diameter().mm() * 1.5,
                prototype.bolt_clearance_diameter().mm(),
            )?
            .translate(Vec3::new(x, y, post_center_z));
            result = result.union_with_engine(&post, BooleanEngine::Exact);
        }
    } else {
        let socket_depth = prototype.top_socket_depth().mm();
        let socket_clearance = prototype.top_socket_diameter_clearance().mm();
        for ([x, y], diameter) in fixed_axles
            .into_iter()
            .map(|position| (position, prototype.journal_outer_diameter().mm()))
            .chain(
                corners
                    .into_iter()
                    .map(|position| (position, prototype.journal_outer_diameter().mm() * 1.5)),
            )
        {
            let radius = (diameter + socket_clearance) * 0.5;
            let socket = Manifold::cylinder(
                socket_depth + 0.05,
                radius,
                radius,
                circular_segments(radius),
            )
            .translate(Vec3::new(
                x,
                y,
                -prototype.plate_thickness().mm() * 0.5 - 0.05,
            ));
            result = result.difference_with_engine(&socket, BooleanEngine::Exact);
        }
    }
    validate_solid(&result)?;
    Ok(result)
}

fn tube(height: f64, outside_diameter: f64, inside_diameter: f64) -> Result<Manifold, KernelError> {
    let result = Manifold::extrude(
        &annulus(outside_diameter, inside_diameter),
        height,
        0,
        0.0,
        Vec2::new(1.0, 1.0),
    )
    .translate(Vec3::new(0.0, 0.0, -height * 0.5));
    validate_solid(&result)?;
    Ok(result)
}

fn gear_cross_section(gear: &SpurGear, bore_diameter_mm: f64) -> Vec<Vec<Vec2>> {
    let outline = gear
        .profile()
        .points
        .into_iter()
        .map(|point| Vec2::new(point.x, point.y))
        .collect::<Vec<_>>();
    CrossSection::new(vec![outline])
        .difference(&CrossSection::circle(
            bore_diameter_mm * 0.5,
            circular_segments(bore_diameter_mm * 0.5),
        ))
        .to_polygons()
}

fn annulus(outside_diameter: f64, inside_diameter: f64) -> Vec<Vec<Vec2>> {
    CrossSection::circle(
        outside_diameter * 0.5,
        circular_segments(outside_diameter * 0.5),
    )
    .difference(&CrossSection::circle(
        inside_diameter * 0.5,
        circular_segments(inside_diameter * 0.5),
    ))
    .to_polygons()
}

fn shear_x_by_z(shear: f64, translate_x: f64, translate_z: f64) -> Mat3x4 {
    Mat3x4::from_cols(
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(shear, 0.0, 1.0),
        Vec3::new(translate_x, 0.0, translate_z),
    )
}

fn transform(solid: Manifold, pose: GearPose) -> Manifold {
    solid
        .rotate(0.0, 0.0, pose.rotation_z_deg)
        .translate(Vec3::new(
            pose.translation_mm[0],
            pose.translation_mm[1],
            pose.translation_mm[2],
        ))
}

fn intersection_volume(lhs: &Manifold, rhs: &Manifold) -> Result<f64, KernelError> {
    let result = lhs.intersection_with_engine(rhs, BooleanEngine::Exact);
    validate_status(&result)?;
    Ok(result.volume())
}

fn circular_segments(radius: f64) -> i32 {
    ((radius * 6.0).ceil() as i32).clamp(48, 256)
}

fn validate_status(solid: &Manifold) -> Result<(), KernelError> {
    if solid.status() == ManifoldStatus::NoError {
        Ok(())
    } else {
        Err(KernelError::Manifold(solid.status()))
    }
}

fn validate_solid(solid: &Manifold) -> Result<(), KernelError> {
    validate_status(solid)?;
    if solid.is_empty() || solid.volume() <= 0.0 {
        Err(KernelError::EmptySolid)
    } else {
        Ok(())
    }
}

fn mesh_from_manifold(solid: &Manifold) -> TriangleMesh {
    let mesh = solid.get_mesh_gl64(-1);
    let vertices = (0..mesh.num_vert())
        .map(|index| mesh.get_vert_pos(index))
        .collect();
    let triangles = (0..mesh.num_tri())
        .map(|index| {
            let triangle = mesh.get_tri_verts(index);
            [
                u32::try_from(triangle[0]).expect("mesh index fits u32"),
                u32::try_from(triangle[1]).expect("mesh index fits u32"),
                u32::try_from(triangle[2]).expect("mesh index fits u32"),
            ]
        })
        .collect();
    TriangleMesh {
        vertices,
        triangles,
    }
}

#[cfg(test)]
mod tests {
    use double_helical_core::{Angle, GearHand, Length, NormalGearSystem};

    use super::*;

    #[test]
    fn optimized_center_bridge_produces_closed_gear_quickly() {
        let system = NormalGearSystem::new(
            Length::positive_mm(2.0).unwrap(),
            Angle::degrees(20.0).unwrap(),
            Angle::degrees(15.0).unwrap(),
            Length::non_negative_mm(0.10).unwrap(),
            Length::positive_mm(0.08).unwrap(),
        )
        .unwrap();
        let gear = system
            .pinion(
                20,
                Length::positive_mm(18.0).unwrap(),
                Length::positive_mm(2.0).unwrap(),
                Length::positive_mm(12.4).unwrap(),
                8,
                GearHand::LeftAtLowerFace,
            )
            .unwrap();
        let solid = double_helical_gear_solid(&gear).unwrap();
        assert!(solid.volume() > 0.0);
        assert!(solid.num_tri() > 100);
    }
}

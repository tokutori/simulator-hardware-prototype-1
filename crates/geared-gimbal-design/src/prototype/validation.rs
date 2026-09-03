// SPDX-License-Identifier: MIT

use super::*;

pub(super) fn validate(parameters: &PrototypeParameters) -> Result<(), PrototypeError> {
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
    let contact = &parameters.contact_unit;
    if contact.retention_bearing_island_radius.mm() <= contact.encoder_shaft_radius.mm() + 1.5
        || contact.retention_flexure_length.mm()
            <= contact.retention_bearing_island_radius.mm() * 2.0
        || contact.retention_flexure_beam_width.mm() >= contact.retention_bearing_island_radius.mm()
        || contact.retention_flexure_bridge_width.mm() <= contact.retention_flexure_beam_width.mm()
        || contact.retention_installed_deflection.as_mm()
            > contact.retention_flexure_length.mm() * 0.1
        || contact.retention_modeled_surface_strain()
            > contact.retention_max_modeled_surface_strain.get()
    {
        return Err(PrototypeError::InvalidRetentionFlexure);
    }
    let plate_half = parameters.pitch_gearbox.side_plate_thickness.mm() * 0.5;
    let drive_flange_outer_extent = parameters.pitch_sector.face_width.mm() * 0.5
        + contact.drive_flange_clearance.mm()
        + contact.flange_thickness.mm();
    let encoder_flange_outer_extent = parameters.pitch_sector.face_width.mm() * 0.5
        + contact.encoder_flange_clearance.mm()
        + contact.flange_thickness.mm();
    const MINIMUM_OUTBOARD_PLATE_GAP_MM: f64 = 0.25;
    if contact.outboard_support_plate_offset.mm() - plate_half
        < libm::fmax(drive_flange_outer_extent, encoder_flange_outer_extent)
            + MINIMUM_OUTBOARD_PLATE_GAP_MM
    {
        return Err(PrototypeError::OutboardSupportHitsFlange);
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
    let roll_stage_distance = parameters.pitch_gearbox.small_gear.pitch_radius()
        + parameters.pitch_gearbox.large_gear.pitch_radius();
    if parameters.roll_axis.gearbox_support_half_span.mm() >= roll_stage_distance {
        return Err(PrototypeError::InvalidGearboxGeometry);
    }
    let gearbox = &parameters.pitch_gearbox;
    const PITCH_GEARBOX_BOSS_RADIUS_MM: f64 = 5.5;
    const MINIMUM_FDM_WALL_MM: f64 = 0.8;
    if gearbox.flanged_bearing_outer_radius.mm() <= gearbox.shaft_radius.mm()
        || gearbox.flanged_bearing_flange_radius.mm() <= gearbox.flanged_bearing_outer_radius.mm()
        || gearbox.flanged_bearing_flange_width.mm() >= gearbox.flanged_bearing_width.mm()
        || (gearbox.flanged_bearing_width.mm() - gearbox.side_plate_thickness.mm()).abs() > 1.0e-6
        || gearbox.flanged_bearing_flange_radius.mm() + MINIMUM_FDM_WALL_MM
            > PITCH_GEARBOX_BOSS_RADIUS_MM
        || gearbox.flanged_bearing_flange_radius.mm() + MINIMUM_FDM_WALL_MM
            > parameters.contact_unit.retention_bearing_island_radius.mm()
    {
        return Err(PrototypeError::InvalidPitchGearboxBearing);
    }
    let diagonal_degrees = gearbox.stage_diagonal_angle.as_degrees();
    if !(0.0 < diagonal_degrees && diagonal_degrees < 90.0) {
        return Err(PrototypeError::InvalidGearboxGeometry);
    }
    let bearing_centers = pitch_contact_carriage_bearing_centers(parameters)?;
    const MINIMUM_BEARING_FLANGE_GAP_MM: f64 = 0.5;
    let required_bearing_separation =
        gearbox.flanged_bearing_flange_radius.mm() * 2.0 + MINIMUM_BEARING_FLANGE_GAP_MM;
    for first in 0..bearing_centers.len() {
        for second in first + 1..bearing_centers.len() {
            if distance2(bearing_centers[first], bearing_centers[second])
                < required_bearing_separation
            {
                return Err(PrototypeError::InvalidPitchBearingSpacing);
            }
        }
    }
    let plate_half = gearbox.side_plate_thickness.mm() * 0.5;
    let gear_half = gearbox.gear_face_width.mm() * 0.5;
    let layer_pitch = gearbox.gear_face_width.mm() + 1.0;
    let near = gearbox.near_plate_inboard_offset.mm();
    let gear = gearbox.gear_plane_inboard_offset.mm();
    let far = gearbox.far_plate_inboard_offset.mm();
    let deepest_gear = gear + 2.0 * layer_pitch;
    const MINIMUM_AXIAL_GAP_MM: f64 = 0.2;
    let flange_outer_extent = libm::fmax(drive_flange_outer_extent, encoder_flange_outer_extent);
    let near_bearing_flange_outer_face =
        near - plate_half - gearbox.flanged_bearing_flange_width.mm();
    let near_plate_inner_face = near + plate_half;
    let near_bearing_body_inner_end = near - plate_half + gearbox.flanged_bearing_width.mm()
        - gearbox.flanged_bearing_flange_width.mm();
    let far_plate_inner_face = far - plate_half;
    let far_bearing_body_outer_start = far + plate_half - gearbox.flanged_bearing_width.mm()
        + gearbox.flanged_bearing_flange_width.mm();
    if near_bearing_flange_outer_face < flange_outer_extent + MINIMUM_AXIAL_GAP_MM
        || gear - gear_half < near_plate_inner_face + MINIMUM_AXIAL_GAP_MM
        || gear - gear_half < near_bearing_body_inner_end + MINIMUM_AXIAL_GAP_MM
        || far_plate_inner_face < deepest_gear + gear_half + MINIMUM_AXIAL_GAP_MM
        || far_bearing_body_outer_start < deepest_gear + gear_half + MINIMUM_AXIAL_GAP_MM
        || far >= parameters.pitch_sector.carrier_spacing.mm() * 0.5
    {
        return Err(PrototypeError::InvalidPitchAxialStack);
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
    if parameters.roll_axis.bearing_outer_radius.mm()
        <= parameters.roll_axis.shaft_radius.mm() + 2.0
        || parameters.roll_axis.bearing_width.mm() + 0.8
            > parameters.frame.bearing_pedestal_thickness.mm()
        || front_bearing_outboard_collar_x(parameters) + roll_bearing_collar_width_mm() * 0.5 + 1.0
            > parameters.roll_axis.drive_station.mm()
                - parameters.pitch_sector.face_width.mm() * 0.5
    {
        return Err(PrototypeError::InvalidRollBearing);
    }
    if parameters.roll_axis.gearbox_support_half_span.mm() + 4.0 >= carrier_inner_span * 0.5 {
        return Err(PrototypeError::InvalidGearboxPlacement);
    }
    Ok(())
}

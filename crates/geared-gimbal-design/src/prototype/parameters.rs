// SPDX-License-Identifier: MIT

use crate::{
    Angle, AssemblyError, FeatureError, GearSector, Length, PositiveLength, PositiveRatio, SpurGear,
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
    pub retention_flexure_length: Length,
    pub retention_flexure_beam_width: Length,
    pub retention_flexure_bridge_width: Length,
    pub retention_bearing_island_radius: Length,
    /// Radial travel from the unloaded printed shape to the installed mesh position.
    pub retention_installed_deflection: PositiveLength,
    /// Geometric fixed-guided beam strain ceiling; this is not a material rating.
    pub retention_max_modeled_surface_strain: PositiveRatio,
    /// Distance from the pitch-sector mid-plane toward the outside support plate.
    pub outboard_support_plate_offset: Length,
}

impl ContactUnitParameters {
    /// Euler-Bernoulli fixed-guided beam estimate, `3 t delta / L^2`.
    ///
    /// This geometric proxy does not establish spring force or fatigue life;
    /// those require the actual print material, orientation, and test coupons.
    pub fn retention_modeled_surface_strain(&self) -> f64 {
        3.0 * self.retention_flexure_beam_width.mm() * self.retention_installed_deflection.as_mm()
            / (self.retention_flexure_length.mm() * self.retention_flexure_length.mm())
    }
}

#[derive(Clone, Debug)]
pub struct PitchGearboxParameters {
    pub small_gear: SpurGear,
    pub large_gear: SpurGear,
    pub distribution_gear: SpurGear,
    pub gear_face_width: Length,
    pub shaft_radius: Length,
    /// Nominal outer radius of the purchased flanged miniature bearing.
    pub flanged_bearing_outer_radius: Length,
    pub flanged_bearing_width: Length,
    pub flanged_bearing_flange_radius: Length,
    pub flanged_bearing_flange_width: Length,
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
    pub bearing_outer_radius: Length,
    pub bearing_width: Length,
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
    InvalidPitchGearboxBearing,
    InvalidRollBearing,
    InvalidRetentionFlexure,
    OutboardSupportHitsFlange,
    InvalidMovingCarrier,
    SectorSpineHitsDrive,
    SectorSupportHitsPost,
    FrameBaseNotOnFloor,
    CarrierRailTooClose,
    CockpitHitsRollSupport,
    RollStationOutsideShaft,
    MissingRequiredInstance,
}

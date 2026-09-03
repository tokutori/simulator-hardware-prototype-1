// SPDX-License-Identifier: MIT

use std::fs;
use std::path::Path;

use geared_gimbal_design::{
    CockpitParameters, ContactUnitParameters, FrameParameters, MotionParameters,
    PitchGearboxParameters, PitchSectorParameters, PrototypeParameters, RollAxisParameters,
};
use gimbal_core::{
    Angle, FdmMaterial, GearSector, InternalGear, Length, PositiveLength, PositiveRatio, SpurGear,
};
use serde::Deserialize;
use thiserror::Error;

#[derive(Debug, Deserialize)]
struct RawParameters {
    pitch_rings: RawPitchRings,
    contact_unit: RawContactUnit,
    pitch_gearbox: RawPitchGearbox,
    roll_axis: RawRollAxis,
    cockpit: RawCockpit,
    frame: RawFrame,
    motion: RawMotion,
}

#[derive(Debug, Deserialize)]
struct RawPitchRings {
    target_outside_diameter_mm: f64,
    spacing_mm: f64,
    module_mm: f64,
    external_teeth: u16,
    internal_teeth: u16,
    pressure_angle_deg: f64,
    backlash_mm: f64,
    chord_tolerance_mm: f64,
    face_width_mm: f64,
    minimum_web_mm: f64,
    sector_half_angle_deg: f64,
}

#[derive(Debug, Deserialize)]
struct RawContactUnit {
    drive_pinion_teeth: u16,
    encoder_pinion_teeth: u16,
    branch_angle_offset_deg: f64,
    drive_shaft_diameter_mm: f64,
    encoder_shaft_diameter_mm: f64,
    drive_flange_clearance_mm: f64,
    encoder_flange_clearance_mm: f64,
    flange_thickness_mm: f64,
    retention_flexure_length_mm: f64,
    retention_flexure_beam_width_mm: f64,
    retention_flexure_bridge_width_mm: f64,
    retention_bearing_island_radius_mm: f64,
    retention_installed_deflection_mm: f64,
    retention_max_modeled_surface_strain: f64,
    outboard_support_plate_offset_mm: f64,
}

#[derive(Debug, Deserialize)]
struct RawPitchGearbox {
    module_mm: f64,
    small_teeth: u16,
    large_teeth: u16,
    distribution_teeth: u16,
    pressure_angle_deg: f64,
    backlash_mm: f64,
    chord_tolerance_mm: f64,
    gear_face_width_mm: f64,
    shaft_diameter_mm: f64,
    flanged_bearing_outer_diameter_mm: f64,
    flanged_bearing_width_mm: f64,
    flanged_bearing_flange_diameter_mm: f64,
    flanged_bearing_flange_width_mm: f64,
    side_plate_thickness_mm: f64,
    near_plate_inboard_offset_mm: f64,
    gear_plane_inboard_offset_mm: f64,
    far_plate_inboard_offset_mm: f64,
}

#[derive(Debug, Deserialize)]
struct RawRollAxis {
    module_mm: f64,
    driven_teeth: u16,
    pinion_teeth: u16,
    pressure_angle_deg: f64,
    backlash_mm: f64,
    chord_tolerance_mm: f64,
    shaft_length_mm: f64,
    shaft_diameter_mm: f64,
    bearing_outer_diameter_mm: f64,
    bearing_width_mm: f64,
    drive_station_mm: f64,
    bearing_station_mm: f64,
    gearbox_support_half_span_mm: f64,
}

#[derive(Debug, Deserialize)]
struct RawCockpit {
    length_mm: f64,
    width_mm: f64,
    height_mm: f64,
    suspension_drop_mm: f64,
}

#[derive(Debug, Deserialize)]
struct RawFrame {
    fixed_rail_length_mm: f64,
    fixed_crossmember_station_mm: f64,
    fixed_crossmember_width_mm: f64,
    fixed_rail_depth_mm: f64,
    bearing_pedestal_thickness_mm: f64,
    sheet_thickness_mm: f64,
    upper_rail_height_mm: f64,
    lower_rail_depth_mm: f64,
    moving_carrier_half_span_mm: f64,
    moving_carrier_height_mm: f64,
    moving_carrier_inboard_offset_mm: f64,
    moving_carrier_member_width_mm: f64,
    floor_top_below_axis_mm: f64,
    floor_thickness_mm: f64,
}

#[derive(Debug, Deserialize)]
struct RawMotion {
    pitch_limit_deg: f64,
    roll_limit_deg: f64,
}

#[derive(Debug, Deserialize)]
struct RawFabrication {
    fdm: RawFdm,
    laser: RawLaser,
}

#[derive(Debug, Deserialize)]
struct RawFdm {
    material: String,
    hole_compensation_mm: f64,
}

#[derive(Debug, Deserialize)]
struct RawLaser {
    material: String,
    kerf_mm: f64,
    bed_width_mm: f64,
    bed_height_mm: f64,
}

#[derive(Clone, Debug)]
pub struct LoadedConfig {
    pub parameters: PrototypeParameters,
    pub fdm_material: FdmMaterial,
    pub fdm_hole_compensation_mm: f64,
    pub laser_material: String,
    pub laser_kerf_mm: f64,
    pub laser_bed_mm: [f64; 2],
}

#[derive(Debug, Error)]
pub enum ConfigError {
    #[error("unable to read {path}: {source}")]
    Read {
        path: String,
        source: std::io::Error,
    },
    #[error("invalid TOML in {path}: {source}")]
    Toml {
        path: String,
        source: toml::de::Error,
    },
    #[error("invalid length for {0}")]
    Length(&'static str),
    #[error("invalid angle for {0}")]
    Angle(&'static str),
    #[error("invalid positive ratio for {0}")]
    Ratio(&'static str),
    #[error("invalid gear specification for {0}: {1}")]
    Gear(&'static str, gimbal_core::GearError),
    #[error("unsupported FDM material {0:?}; expected PLA, ABS, PETG, or ASA")]
    UnsupportedFdmMaterial(String),
    #[error("fabrication process values must be finite and non-negative")]
    InvalidProcessProfile,
    #[error("laser bed dimensions must be positive")]
    InvalidLaserBed,
}

pub fn load(parameters_path: &Path, fabrication_path: &Path) -> Result<LoadedConfig, ConfigError> {
    let raw: RawParameters = read_toml(parameters_path)?;
    let fabrication: RawFabrication = read_toml(fabrication_path)?;
    let fdm_material = validate_fabrication(&fabrication)?;

    let ring_module = positive(raw.pitch_rings.module_mm, "pitch_rings.module_mm")?;
    let ring_pressure = degrees(
        raw.pitch_rings.pressure_angle_deg,
        "pitch_rings.pressure_angle_deg",
    )?;
    let ring_backlash = non_negative(raw.pitch_rings.backlash_mm * 0.5, "pitch_rings.backlash_mm")?;
    let ring_tolerance = positive(
        raw.pitch_rings.chord_tolerance_mm,
        "pitch_rings.chord_tolerance_mm",
    )?;
    let external_reference = external(
        "pitch sector external reference",
        ring_module,
        raw.pitch_rings.external_teeth,
        ring_pressure,
        ring_backlash,
        ring_tolerance,
    )?;
    let internal_reference = InternalGear::new(
        ring_module,
        raw.pitch_rings.internal_teeth,
        ring_pressure,
        ring_backlash,
        ring_tolerance,
    )
    .map_err(|error| ConfigError::Gear("pitch sector internal reference", error))?;
    let sector = GearSector::new(
        external_reference,
        internal_reference,
        degrees(
            raw.pitch_rings.sector_half_angle_deg,
            "pitch_rings.sector_half_angle_deg",
        )?,
    )
    .map_err(|error| ConfigError::Gear("pitch gear sector", error))?;
    let drive_pinion = external(
        "pitch drive pinion",
        ring_module,
        raw.contact_unit.drive_pinion_teeth,
        ring_pressure,
        ring_backlash,
        ring_tolerance,
    )?;
    let encoder_pinion = external(
        "pitch retention/encoder pinion",
        ring_module,
        raw.contact_unit.encoder_pinion_teeth,
        ring_pressure,
        ring_backlash,
        ring_tolerance,
    )?;

    let gearbox_module = positive(raw.pitch_gearbox.module_mm, "pitch_gearbox.module_mm")?;
    let gearbox_pressure = degrees(
        raw.pitch_gearbox.pressure_angle_deg,
        "pitch_gearbox.pressure_angle_deg",
    )?;
    let gearbox_backlash = non_negative(
        raw.pitch_gearbox.backlash_mm * 0.5,
        "pitch_gearbox.backlash_mm",
    )?;
    let gearbox_tolerance = positive(
        raw.pitch_gearbox.chord_tolerance_mm,
        "pitch_gearbox.chord_tolerance_mm",
    )?;
    let gearbox_small = external(
        "pitch gearbox small",
        gearbox_module,
        raw.pitch_gearbox.small_teeth,
        gearbox_pressure,
        gearbox_backlash,
        gearbox_tolerance,
    )?;
    let gearbox_large = external(
        "pitch gearbox large",
        gearbox_module,
        raw.pitch_gearbox.large_teeth,
        gearbox_pressure,
        gearbox_backlash,
        gearbox_tolerance,
    )?;
    let distribution = external(
        "pitch gearbox distribution",
        gearbox_module,
        raw.pitch_gearbox.distribution_teeth,
        gearbox_pressure,
        gearbox_backlash,
        gearbox_tolerance,
    )?;

    let roll_module = positive(raw.roll_axis.module_mm, "roll_axis.module_mm")?;
    let roll_pressure = degrees(
        raw.roll_axis.pressure_angle_deg,
        "roll_axis.pressure_angle_deg",
    )?;
    let roll_backlash = non_negative(raw.roll_axis.backlash_mm * 0.5, "roll_axis.backlash_mm")?;
    let roll_tolerance = positive(
        raw.roll_axis.chord_tolerance_mm,
        "roll_axis.chord_tolerance_mm",
    )?;
    let roll_driven = external(
        "roll driven gear",
        roll_module,
        raw.roll_axis.driven_teeth,
        roll_pressure,
        roll_backlash,
        roll_tolerance,
    )?;
    let roll_pinion = external(
        "roll pinion",
        roll_module,
        raw.roll_axis.pinion_teeth,
        roll_pressure,
        roll_backlash,
        roll_tolerance,
    )?;

    Ok(LoadedConfig {
        parameters: PrototypeParameters {
            pitch_sector: PitchSectorParameters {
                target_outer_diameter: positive(
                    raw.pitch_rings.target_outside_diameter_mm,
                    "pitch_rings.target_outside_diameter_mm",
                )?,
                sector,
                carrier_spacing: positive(raw.pitch_rings.spacing_mm, "pitch_rings.spacing_mm")?,
                face_width: positive(raw.pitch_rings.face_width_mm, "pitch_rings.face_width_mm")?,
                minimum_web: positive(
                    raw.pitch_rings.minimum_web_mm,
                    "pitch_rings.minimum_web_mm",
                )?,
            },
            contact_unit: ContactUnitParameters {
                drive_pinion,
                encoder_pinion,
                branch_angle_offset: degrees(
                    raw.contact_unit.branch_angle_offset_deg,
                    "contact_unit.branch_angle_offset_deg",
                )?,
                drive_shaft_radius: positive(
                    raw.contact_unit.drive_shaft_diameter_mm * 0.5,
                    "contact_unit.drive_shaft_diameter_mm",
                )?,
                encoder_shaft_radius: positive(
                    raw.contact_unit.encoder_shaft_diameter_mm * 0.5,
                    "contact_unit.encoder_shaft_diameter_mm",
                )?,
                drive_flange_clearance: positive(
                    raw.contact_unit.drive_flange_clearance_mm,
                    "contact_unit.drive_flange_clearance_mm",
                )?,
                encoder_flange_clearance: positive(
                    raw.contact_unit.encoder_flange_clearance_mm,
                    "contact_unit.encoder_flange_clearance_mm",
                )?,
                flange_thickness: positive(
                    raw.contact_unit.flange_thickness_mm,
                    "contact_unit.flange_thickness_mm",
                )?,
                retention_flexure_length: positive(
                    raw.contact_unit.retention_flexure_length_mm,
                    "contact_unit.retention_flexure_length_mm",
                )?,
                retention_flexure_beam_width: positive(
                    raw.contact_unit.retention_flexure_beam_width_mm,
                    "contact_unit.retention_flexure_beam_width_mm",
                )?,
                retention_flexure_bridge_width: positive(
                    raw.contact_unit.retention_flexure_bridge_width_mm,
                    "contact_unit.retention_flexure_bridge_width_mm",
                )?,
                retention_bearing_island_radius: positive(
                    raw.contact_unit.retention_bearing_island_radius_mm,
                    "contact_unit.retention_bearing_island_radius_mm",
                )?,
                retention_installed_deflection: PositiveLength::mm(
                    raw.contact_unit.retention_installed_deflection_mm,
                )
                .map_err(|_| {
                    ConfigError::Length("contact_unit.retention_installed_deflection_mm")
                })?,
                retention_max_modeled_surface_strain: PositiveRatio::new(
                    raw.contact_unit.retention_max_modeled_surface_strain,
                )
                .map_err(|_| {
                    ConfigError::Ratio("contact_unit.retention_max_modeled_surface_strain")
                })?,
                outboard_support_plate_offset: positive(
                    raw.contact_unit.outboard_support_plate_offset_mm,
                    "contact_unit.outboard_support_plate_offset_mm",
                )?,
            },
            pitch_gearbox: PitchGearboxParameters {
                small_gear: gearbox_small,
                large_gear: gearbox_large,
                distribution_gear: distribution,
                gear_face_width: positive(
                    raw.pitch_gearbox.gear_face_width_mm,
                    "pitch_gearbox.gear_face_width_mm",
                )?,
                shaft_radius: positive(
                    raw.pitch_gearbox.shaft_diameter_mm * 0.5,
                    "pitch_gearbox.shaft_diameter_mm",
                )?,
                flanged_bearing_outer_radius: positive(
                    raw.pitch_gearbox.flanged_bearing_outer_diameter_mm * 0.5,
                    "pitch_gearbox.flanged_bearing_outer_diameter_mm",
                )?,
                flanged_bearing_width: positive(
                    raw.pitch_gearbox.flanged_bearing_width_mm,
                    "pitch_gearbox.flanged_bearing_width_mm",
                )?,
                flanged_bearing_flange_radius: positive(
                    raw.pitch_gearbox.flanged_bearing_flange_diameter_mm * 0.5,
                    "pitch_gearbox.flanged_bearing_flange_diameter_mm",
                )?,
                flanged_bearing_flange_width: positive(
                    raw.pitch_gearbox.flanged_bearing_flange_width_mm,
                    "pitch_gearbox.flanged_bearing_flange_width_mm",
                )?,
                side_plate_thickness: positive(
                    raw.pitch_gearbox.side_plate_thickness_mm,
                    "pitch_gearbox.side_plate_thickness_mm",
                )?,
                near_plate_inboard_offset: positive(
                    raw.pitch_gearbox.near_plate_inboard_offset_mm,
                    "pitch_gearbox.near_plate_inboard_offset_mm",
                )?,
                gear_plane_inboard_offset: positive(
                    raw.pitch_gearbox.gear_plane_inboard_offset_mm,
                    "pitch_gearbox.gear_plane_inboard_offset_mm",
                )?,
                far_plate_inboard_offset: positive(
                    raw.pitch_gearbox.far_plate_inboard_offset_mm,
                    "pitch_gearbox.far_plate_inboard_offset_mm",
                )?,
            },
            roll_axis: RollAxisParameters {
                driven_gear: roll_driven,
                pinion: roll_pinion,
                shaft_length: positive(raw.roll_axis.shaft_length_mm, "roll_axis.shaft_length_mm")?,
                shaft_radius: positive(
                    raw.roll_axis.shaft_diameter_mm * 0.5,
                    "roll_axis.shaft_diameter_mm",
                )?,
                bearing_outer_radius: positive(
                    raw.roll_axis.bearing_outer_diameter_mm * 0.5,
                    "roll_axis.bearing_outer_diameter_mm",
                )?,
                bearing_width: positive(
                    raw.roll_axis.bearing_width_mm,
                    "roll_axis.bearing_width_mm",
                )?,
                drive_station: positive(
                    raw.roll_axis.drive_station_mm,
                    "roll_axis.drive_station_mm",
                )?,
                bearing_station: positive(
                    raw.roll_axis.bearing_station_mm,
                    "roll_axis.bearing_station_mm",
                )?,
                gearbox_support_half_span: positive(
                    raw.roll_axis.gearbox_support_half_span_mm,
                    "roll_axis.gearbox_support_half_span_mm",
                )?,
            },
            cockpit: CockpitParameters {
                length: positive(raw.cockpit.length_mm, "cockpit.length_mm")?,
                width: positive(raw.cockpit.width_mm, "cockpit.width_mm")?,
                height: positive(raw.cockpit.height_mm, "cockpit.height_mm")?,
                suspension_drop: positive(
                    raw.cockpit.suspension_drop_mm,
                    "cockpit.suspension_drop_mm",
                )?,
            },
            frame: FrameParameters {
                fixed_rail_length: positive(
                    raw.frame.fixed_rail_length_mm,
                    "frame.fixed_rail_length_mm",
                )?,
                fixed_crossmember_station: positive(
                    raw.frame.fixed_crossmember_station_mm,
                    "frame.fixed_crossmember_station_mm",
                )?,
                fixed_crossmember_width: positive(
                    raw.frame.fixed_crossmember_width_mm,
                    "frame.fixed_crossmember_width_mm",
                )?,
                fixed_rail_depth: positive(
                    raw.frame.fixed_rail_depth_mm,
                    "frame.fixed_rail_depth_mm",
                )?,
                bearing_pedestal_thickness: positive(
                    raw.frame.bearing_pedestal_thickness_mm,
                    "frame.bearing_pedestal_thickness_mm",
                )?,
                sheet_thickness: positive(
                    raw.frame.sheet_thickness_mm,
                    "frame.sheet_thickness_mm",
                )?,
                upper_rail_height: positive(
                    raw.frame.upper_rail_height_mm,
                    "frame.upper_rail_height_mm",
                )?,
                lower_rail_depth: positive(
                    raw.frame.lower_rail_depth_mm,
                    "frame.lower_rail_depth_mm",
                )?,
                moving_carrier_half_span: positive(
                    raw.frame.moving_carrier_half_span_mm,
                    "frame.moving_carrier_half_span_mm",
                )?,
                moving_carrier_height: positive(
                    raw.frame.moving_carrier_height_mm,
                    "frame.moving_carrier_height_mm",
                )?,
                moving_carrier_inboard_offset: positive(
                    raw.frame.moving_carrier_inboard_offset_mm,
                    "frame.moving_carrier_inboard_offset_mm",
                )?,
                moving_carrier_member_width: positive(
                    raw.frame.moving_carrier_member_width_mm,
                    "frame.moving_carrier_member_width_mm",
                )?,
                floor_top_below_axis: positive(
                    raw.frame.floor_top_below_axis_mm,
                    "frame.floor_top_below_axis_mm",
                )?,
                floor_thickness: positive(
                    raw.frame.floor_thickness_mm,
                    "frame.floor_thickness_mm",
                )?,
            },
            motion: MotionParameters {
                pitch_limit: degrees(raw.motion.pitch_limit_deg, "motion.pitch_limit_deg")?,
                roll_limit: degrees(raw.motion.roll_limit_deg, "motion.roll_limit_deg")?,
            },
        },
        fdm_material,
        fdm_hole_compensation_mm: fabrication.fdm.hole_compensation_mm,
        laser_material: fabrication.laser.material,
        laser_kerf_mm: fabrication.laser.kerf_mm,
        laser_bed_mm: [
            fabrication.laser.bed_width_mm,
            fabrication.laser.bed_height_mm,
        ],
    })
}

fn validate_fabrication(raw: &RawFabrication) -> Result<FdmMaterial, ConfigError> {
    let fdm_material = match raw.fdm.material.to_ascii_uppercase().as_str() {
        "PLA" => FdmMaterial::Pla,
        "ABS" => FdmMaterial::Abs,
        "PETG" => FdmMaterial::Petg,
        "ASA" => FdmMaterial::Asa,
        _ => {
            return Err(ConfigError::UnsupportedFdmMaterial(
                raw.fdm.material.clone(),
            ));
        }
    };
    if !raw.fdm.hole_compensation_mm.is_finite()
        || raw.fdm.hole_compensation_mm < 0.0
        || !raw.laser.kerf_mm.is_finite()
        || raw.laser.kerf_mm < 0.0
    {
        return Err(ConfigError::InvalidProcessProfile);
    }
    if raw.laser.bed_width_mm <= 0.0 || raw.laser.bed_height_mm <= 0.0 {
        return Err(ConfigError::InvalidLaserBed);
    }
    Ok(fdm_material)
}

fn read_toml<T: for<'de> Deserialize<'de>>(path: &Path) -> Result<T, ConfigError> {
    let text = fs::read_to_string(path).map_err(|source| ConfigError::Read {
        path: path.display().to_string(),
        source,
    })?;
    toml::from_str(&text).map_err(|source| ConfigError::Toml {
        path: path.display().to_string(),
        source,
    })
}

fn positive(value: f64, field: &'static str) -> Result<Length, ConfigError> {
    Length::positive_mm(value).map_err(|_| ConfigError::Length(field))
}

fn non_negative(value: f64, field: &'static str) -> Result<Length, ConfigError> {
    Length::non_negative_mm(value).map_err(|_| ConfigError::Length(field))
}

fn degrees(value: f64, field: &'static str) -> Result<Angle, ConfigError> {
    Angle::degrees(value).map_err(|_| ConfigError::Angle(field))
}

fn external(
    label: &'static str,
    module: Length,
    teeth: u16,
    pressure: Angle,
    backlash: Length,
    tolerance: Length,
) -> Result<SpurGear, ConfigError> {
    SpurGear::new(module, teeth, pressure, backlash, tolerance)
        .map_err(|error| ConfigError::Gear(label, error))
}

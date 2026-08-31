// SPDX-License-Identifier: MIT

use std::fs;
use std::path::Path;

use gimbal_core::{
    Angle, CockpitParameters, ContactUnitParameters, FrameParameters, GearSector, InternalGear,
    Length, MotionParameters, PitchGearboxParameters, PitchSectorParameters, PrototypeParameters,
    RollAxisParameters, SpurGear,
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
    side_plate_thickness_mm: f64,
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
    crossmember_diameter_mm: f64,
    bearing_pedestal_thickness_mm: f64,
    sheet_thickness_mm: f64,
    carrier_rail_offset_mm: f64,
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
    pub fdm_material: String,
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
    #[error("invalid gear specification for {0}: {1}")]
    Gear(&'static str, gimbal_core::GearError),
    #[error("this prototype currently supports PETG as its FDM material")]
    UnsupportedFdmMaterial,
    #[error("fabrication process values must be finite and non-negative")]
    InvalidProcessProfile,
    #[error("laser bed dimensions must be positive")]
    InvalidLaserBed,
}

pub fn load(parameters_path: &Path, fabrication_path: &Path) -> Result<LoadedConfig, ConfigError> {
    let raw: RawParameters = read_toml(parameters_path)?;
    let fabrication: RawFabrication = read_toml(fabrication_path)?;
    validate_fabrication(&fabrication)?;

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
                side_plate_thickness: positive(
                    raw.pitch_gearbox.side_plate_thickness_mm,
                    "pitch_gearbox.side_plate_thickness_mm",
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
                crossmember_radius: positive(
                    raw.frame.crossmember_diameter_mm * 0.5,
                    "frame.crossmember_diameter_mm",
                )?,
                bearing_pedestal_thickness: positive(
                    raw.frame.bearing_pedestal_thickness_mm,
                    "frame.bearing_pedestal_thickness_mm",
                )?,
                sheet_thickness: positive(
                    raw.frame.sheet_thickness_mm,
                    "frame.sheet_thickness_mm",
                )?,
                carrier_rail_offset: positive(
                    raw.frame.carrier_rail_offset_mm,
                    "frame.carrier_rail_offset_mm",
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
        fdm_material: fabrication.fdm.material,
        fdm_hole_compensation_mm: fabrication.fdm.hole_compensation_mm,
        laser_material: fabrication.laser.material,
        laser_kerf_mm: fabrication.laser.kerf_mm,
        laser_bed_mm: [
            fabrication.laser.bed_width_mm,
            fabrication.laser.bed_height_mm,
        ],
    })
}

fn validate_fabrication(raw: &RawFabrication) -> Result<(), ConfigError> {
    if raw.fdm.material.to_uppercase() != "PETG" {
        return Err(ConfigError::UnsupportedFdmMaterial);
    }
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
    Ok(())
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

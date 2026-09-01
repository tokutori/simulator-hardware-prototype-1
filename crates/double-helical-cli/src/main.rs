// SPDX-License-Identifier: MIT

use std::env;
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

use double_helical_core::{
    Angle, DoubleHelicalRack, GearError, GearHand, GearPose, Length, NormalGearSystem, Prototype,
    PrototypeError, SpurGear, UnitError,
};
use double_helical_export::{ExportError, ExportPart, write_3mf, write_binary_stl, write_obj};
use double_helical_kernel_manifold::{
    KernelError, PrototypeInterference, PrototypeMeshes, build_prototype, prototype_interference,
};
use serde::Deserialize;
use thiserror::Error;

const DEFAULT_CONFIG: &str = "parameters.toml";
const DEFAULT_OUTPUT_DIR: &str = "output";
const INTERFERENCE_TOLERANCE_MM3: f64 = 1.0e-6;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    spur_stage: SpurStageConfig,
    rack_stage: RackStageConfig,
    handle: HandleConfig,
    hardware: HardwareConfig,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SpurStageConfig {
    module_mm: f64,
    pressure_angle_deg: f64,
    handle_teeth: u16,
    reduction_large_teeth: u16,
    reduction_small_teeth: u16,
    output_teeth: u16,
    face_width_mm: f64,
    tooth_backlash_mm: f64,
    chord_tolerance_mm: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RackStageConfig {
    normal_module_mm: f64,
    normal_pressure_angle_deg: f64,
    helix_angle_deg: f64,
    pinion_teeth: u16,
    rack_teeth: u16,
    face_width_mm: f64,
    center_gap_mm: f64,
    pinion_lower_extension_mm: f64,
    rack_body_thickness_mm: f64,
    tooth_backlash_mm: f64,
    chord_tolerance_mm: f64,
    slices_per_half: u16,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct HardwareConfig {
    bolt_length_mm: f64,
    bolt_clearance_diameter_mm: f64,
    printed_journal_outer_diameter_mm: f64,
    rotating_bore_diameter_mm: f64,
    thrust_spacer_outer_diameter_mm: f64,
    nut_across_flats_mm: f64,
    nut_thickness_mm: f64,
    nut_pocket_depth_mm: f64,
    plate_thickness_mm: f64,
    plate_length_mm: f64,
    plate_width_mm: f64,
    plate_center_y_mm: f64,
    corner_bolt_inset_mm: f64,
    axial_clearance_mm: f64,
    top_socket_depth_mm: f64,
    top_socket_axial_clearance_mm: f64,
    top_socket_diameter_clearance_mm: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct HandleConfig {
    crank_radius_mm: f64,
}

#[derive(Debug)]
struct Options {
    config_path: PathBuf,
    output_dir: PathBuf,
}

#[derive(Debug, Error)]
enum AppError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("configuration parse error: {0}")]
    Toml(#[from] toml::de::Error),
    #[error("invalid field {field}: {reason}")]
    Unit {
        field: &'static str,
        reason: UnitError,
    },
    #[error("invalid gear configuration: {0}")]
    Gear(#[from] GearError),
    #[error("invalid prototype configuration: {0}")]
    Prototype(#[from] PrototypeError),
    #[error(transparent)]
    Kernel(#[from] KernelError),
    #[error(transparent)]
    Export(#[from] ExportError),
    #[error("unknown argument: {0}")]
    UnknownArgument(String),
    #[error("missing value after {0}")]
    MissingArgumentValue(String),
    #[error("generated assembly has excessive interference: {0}")]
    Interference(String),
}

fn main() {
    if let Err(error) = run() {
        eprintln!("error: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), AppError> {
    let Some(options) = parse_options()? else {
        return Ok(());
    };
    let source = fs::read_to_string(&options.config_path)?;
    let config: Config = toml::from_str(&source)?;
    let prototype = config.build()?;
    let interference = prototype_interference(&prototype)?;
    require_clear_meshes(interference)?;
    let meshes = build_prototype(&prototype)?;

    fs::create_dir_all(&options.output_dir)?;
    write_outputs(&options.output_dir, &prototype, &meshes)?;
    let report = report(&prototype, &meshes, interference);
    fs::write(
        options.output_dir.join("assembly").join("report.txt"),
        report.as_bytes(),
    )?;
    print!("{report}");
    Ok(())
}

fn parse_options() -> Result<Option<Options>, AppError> {
    let mut config_path = PathBuf::from(DEFAULT_CONFIG);
    let mut output_dir = PathBuf::from(DEFAULT_OUTPUT_DIR);
    let mut arguments = env::args().skip(1);
    while let Some(argument) = arguments.next() {
        match argument.as_str() {
            "--config" => {
                config_path = PathBuf::from(
                    arguments
                        .next()
                        .ok_or_else(|| AppError::MissingArgumentValue(argument.clone()))?,
                );
            }
            "--output" => {
                output_dir = PathBuf::from(
                    arguments
                        .next()
                        .ok_or_else(|| AppError::MissingArgumentValue(argument.clone()))?,
                );
            }
            "-h" | "--help" => {
                println!(
                    "double-helical-cli\n\nUsage: cargo run -p double-helical-cli -- [--config PATH] [--output DIR]\n\nDefaults:\n  --config {DEFAULT_CONFIG}\n  --output {DEFAULT_OUTPUT_DIR}"
                );
                return Ok(None);
            }
            _ => return Err(AppError::UnknownArgument(argument)),
        }
    }
    Ok(Some(Options {
        config_path,
        output_dir,
    }))
}

impl Config {
    fn build(&self) -> Result<Prototype, AppError> {
        let spur_module = positive_length("spur_stage.module_mm", self.spur_stage.module_mm)?;
        let spur_pressure = angle(
            "spur_stage.pressure_angle_deg",
            self.spur_stage.pressure_angle_deg,
        )?;
        let spur_backlash = non_negative_length(
            "spur_stage.tooth_backlash_mm",
            self.spur_stage.tooth_backlash_mm,
        )?;
        let spur_tolerance = positive_length(
            "spur_stage.chord_tolerance_mm",
            self.spur_stage.chord_tolerance_mm,
        )?;
        let handle_spur = SpurGear::new(
            spur_module,
            self.spur_stage.handle_teeth,
            spur_pressure,
            spur_backlash,
            spur_tolerance,
        )?;
        let reduction_large_spur = SpurGear::new(
            spur_module,
            self.spur_stage.reduction_large_teeth,
            spur_pressure,
            spur_backlash,
            spur_tolerance,
        )?;
        let reduction_small_spur = SpurGear::new(
            spur_module,
            self.spur_stage.reduction_small_teeth,
            spur_pressure,
            spur_backlash,
            spur_tolerance,
        )?;
        let output_spur = SpurGear::new(
            spur_module,
            self.spur_stage.output_teeth,
            spur_pressure,
            spur_backlash,
            spur_tolerance,
        )?;

        let normal_system = NormalGearSystem::new(
            positive_length(
                "rack_stage.normal_module_mm",
                self.rack_stage.normal_module_mm,
            )?,
            angle(
                "rack_stage.normal_pressure_angle_deg",
                self.rack_stage.normal_pressure_angle_deg,
            )?,
            angle(
                "rack_stage.helix_angle_deg",
                self.rack_stage.helix_angle_deg,
            )?,
            non_negative_length(
                "rack_stage.tooth_backlash_mm",
                self.rack_stage.tooth_backlash_mm,
            )?,
            positive_length(
                "rack_stage.chord_tolerance_mm",
                self.rack_stage.chord_tolerance_mm,
            )?,
        )?;
        let face_width =
            positive_length("rack_stage.face_width_mm", self.rack_stage.face_width_mm)?;
        let center_gap =
            non_negative_length("rack_stage.center_gap_mm", self.rack_stage.center_gap_mm)?;
        let rotating_bore = positive_length(
            "hardware.rotating_bore_diameter_mm",
            self.hardware.rotating_bore_diameter_mm,
        )?;
        let driven = normal_system.pinion(
            self.rack_stage.pinion_teeth,
            face_width,
            center_gap,
            rotating_bore,
            self.rack_stage.slices_per_half,
            GearHand::RightAtLowerFace,
        )?;
        let idler = normal_system.pinion(
            self.rack_stage.pinion_teeth,
            face_width,
            center_gap,
            rotating_bore,
            self.rack_stage.slices_per_half,
            GearHand::LeftAtLowerFace,
        )?;
        let rack = DoubleHelicalRack::new(
            normal_system,
            self.rack_stage.rack_teeth,
            face_width,
            center_gap,
            positive_length(
                "rack_stage.rack_body_thickness_mm",
                self.rack_stage.rack_body_thickness_mm,
            )?,
            self.rack_stage.slices_per_half,
            GearHand::LeftAtLowerFace,
        )?;

        Ok(Prototype::new(
            handle_spur,
            reduction_large_spur,
            reduction_small_spur,
            output_spur,
            driven,
            idler,
            rack,
            positive_length("spur_stage.face_width_mm", self.spur_stage.face_width_mm)?,
            positive_length(
                "rack_stage.pinion_lower_extension_mm",
                self.rack_stage.pinion_lower_extension_mm,
            )?,
            positive_length("hardware.bolt_length_mm", self.hardware.bolt_length_mm)?,
            positive_length(
                "hardware.printed_journal_outer_diameter_mm",
                self.hardware.printed_journal_outer_diameter_mm,
            )?,
            positive_length(
                "hardware.bolt_clearance_diameter_mm",
                self.hardware.bolt_clearance_diameter_mm,
            )?,
            positive_length(
                "hardware.thrust_spacer_outer_diameter_mm",
                self.hardware.thrust_spacer_outer_diameter_mm,
            )?,
            positive_length(
                "hardware.nut_across_flats_mm",
                self.hardware.nut_across_flats_mm,
            )?,
            positive_length("hardware.nut_thickness_mm", self.hardware.nut_thickness_mm)?,
            positive_length(
                "hardware.nut_pocket_depth_mm",
                self.hardware.nut_pocket_depth_mm,
            )?,
            positive_length(
                "hardware.plate_thickness_mm",
                self.hardware.plate_thickness_mm,
            )?,
            positive_length("hardware.plate_length_mm", self.hardware.plate_length_mm)?,
            positive_length("hardware.plate_width_mm", self.hardware.plate_width_mm)?,
            self.hardware.plate_center_y_mm,
            positive_length(
                "hardware.corner_bolt_inset_mm",
                self.hardware.corner_bolt_inset_mm,
            )?,
            positive_length(
                "hardware.axial_clearance_mm",
                self.hardware.axial_clearance_mm,
            )?,
            positive_length("handle.crank_radius_mm", self.handle.crank_radius_mm)?,
            positive_length(
                "hardware.top_socket_depth_mm",
                self.hardware.top_socket_depth_mm,
            )?,
            positive_length(
                "hardware.top_socket_axial_clearance_mm",
                self.hardware.top_socket_axial_clearance_mm,
            )?,
            positive_length(
                "hardware.top_socket_diameter_clearance_mm",
                self.hardware.top_socket_diameter_clearance_mm,
            )?,
        )?)
    }
}

fn positive_length(field: &'static str, value: f64) -> Result<Length, AppError> {
    Length::positive_mm(value).map_err(|reason| AppError::Unit { field, reason })
}

fn non_negative_length(field: &'static str, value: f64) -> Result<Length, AppError> {
    Length::non_negative_mm(value).map_err(|reason| AppError::Unit { field, reason })
}

fn angle(field: &'static str, value: f64) -> Result<Angle, AppError> {
    Angle::degrees(value).map_err(|reason| AppError::Unit { field, reason })
}

fn require_clear_meshes(interference: PrototypeInterference) -> Result<(), AppError> {
    let checks = [
        (
            "handle spur/shaft fit",
            interference.handle_spur_to_shaft_mm3,
        ),
        (
            "handle shaft/bottom plate fit",
            interference.handle_shaft_to_bottom_plate_mm3,
        ),
        (
            "handle shaft/top plate fit",
            interference.handle_shaft_to_top_plate_mm3,
        ),
        (
            "handle/D-large spur mesh",
            interference.handle_to_reduction_large_mm3,
        ),
        ("D-small/B spur mesh", interference.reduction_small_to_b_mm3),
        ("D-small/C spur mesh", interference.reduction_small_to_c_mm3),
        (
            "B pinion/rack mesh",
            interference.driven_b_pinion_to_rack_mm3,
        ),
        (
            "C pinion/rack mesh",
            interference.driven_c_pinion_to_rack_mm3,
        ),
        (
            "idler pinion/rack mesh",
            interference.idler_pinion_to_rack_mm3,
        ),
    ];
    let failures = checks
        .into_iter()
        .filter(|(_, volume)| *volume > INTERFERENCE_TOLERANCE_MM3)
        .map(|(name, volume)| format!("{name}={volume:.9} mm^3"))
        .collect::<Vec<_>>();
    if failures.is_empty() {
        Ok(())
    } else {
        Err(AppError::Interference(failures.join(", ")))
    }
}

fn write_outputs(
    output_dir: &Path,
    prototype: &Prototype,
    meshes: &PrototypeMeshes,
) -> Result<(), AppError> {
    let identity = pose(0.0, 0.0, 0.0, 0.0);
    let print_dir = output_dir.join("print-parts");
    let assembly_dir = output_dir.join("assembly");
    fs::create_dir_all(&print_dir)?;
    fs::create_dir_all(&assembly_dir)?;
    for obsolete_name in [
        "handle-shaft-spur.stl",
        "handle-spur.stl",
        "handle-shaft.stl",
        "handle-crank.stl",
        "handle-knob.stl",
        "reduction-d-compound.stl",
        "driven-b-compound.stl",
        "driven-c-compound.stl",
        "idler-pinion.stl",
        "double-helical-rack.stl",
        "top-frame-plate.stl",
        "bottom-frame-plate.stl",
        "handle-upper-thrust-spacer.stl",
        "reduction-d-upper-thrust-spacer.stl",
        "driven-lower-thrust-spacer.stl",
        "idler-lower-thrust-spacer.stl",
        "prototype-assembly.stl",
        "prototype-assembly.3mf",
        "prototype-assembly.obj",
        "prototype-assembly.mtl",
        "prototype-assembly.blend",
        "prototype-preview.scad",
        "prototype-preview.png",
        "prototype-blender.png",
        "prototype-compounds.png",
        "prototype-case-fit.png",
        "prototype-handle.png",
        "report.txt",
    ] {
        let obsolete_path = output_dir.join(obsolete_name);
        if obsolete_path.exists() {
            fs::remove_file(obsolete_path)?;
        }
    }
    let obsolete_handle = print_dir.join("handle-shaft-spur.stl");
    if obsolete_handle.exists() {
        fs::remove_file(obsolete_handle)?;
    }
    let individual = [
        ("handle-spur.stl", "handle-spur", &meshes.handle_spur),
        ("handle-shaft.stl", "handle-shaft", &meshes.handle_shaft),
        ("handle-crank.stl", "handle-crank", &meshes.handle_crank),
        ("handle-knob.stl", "handle-knob", &meshes.handle_knob),
        (
            "reduction-d-compound.stl",
            "reduction-d-compound",
            &meshes.reduction_compound,
        ),
        (
            "driven-b-compound.stl",
            "driven-b-compound",
            &meshes.driven_b_compound,
        ),
        (
            "driven-c-compound.stl",
            "driven-c-compound",
            &meshes.driven_c_compound,
        ),
        ("idler-pinion.stl", "idler-20t", &meshes.idler_pinion),
        (
            "double-helical-rack.stl",
            "double-helical-rack",
            &meshes.rack,
        ),
        ("top-frame-plate.stl", "top-frame-plate", &meshes.top_plate),
        (
            "bottom-frame-plate.stl",
            "bottom-frame-plate-with-nut-pockets",
            &meshes.bottom_plate,
        ),
        (
            "handle-upper-thrust-spacer.stl",
            "handle-upper-thrust-spacer",
            &meshes.handle_upper_thrust_spacer,
        ),
        (
            "reduction-d-upper-thrust-spacer.stl",
            "reduction-d-upper-thrust-spacer",
            &meshes.reduction_upper_thrust_spacer,
        ),
        (
            "driven-lower-thrust-spacer.stl",
            "driven-lower-thrust-spacer-print-two",
            &meshes.driven_lower_thrust_spacer,
        ),
        (
            "idler-lower-thrust-spacer.stl",
            "idler-lower-thrust-spacer",
            &meshes.idler_lower_thrust_spacer,
        ),
    ];
    for (filename, name, mesh) in individual {
        write_binary_stl(
            &[ExportPart {
                name,
                mesh,
                pose: identity,
                color_rgb: [0.75, 0.75, 0.75],
            }],
            &print_dir.join(filename),
        )?;
    }

    let mut assembly = vec![
        part(
            "handle-spur",
            &meshes.handle_spur,
            prototype.handle_spur_pose(),
            [0.95, 0.55, 0.20],
        ),
        part(
            "handle-shaft",
            &meshes.handle_shaft,
            prototype.handle_spur_pose(),
            [0.20, 0.24, 0.32],
        ),
        part(
            "reduction-d-large-plus-small",
            &meshes.reduction_compound,
            prototype.reduction_pose(),
            [0.72, 0.38, 0.82],
        ),
        part(
            "driven-b-output-plus-pinion",
            &meshes.driven_b_compound,
            prototype.driven_b_pose(),
            [0.20, 0.55, 0.95],
        ),
        part(
            "driven-c-output-plus-pinion",
            &meshes.driven_c_compound,
            prototype.driven_c_pose(),
            [0.20, 0.65, 0.90],
        ),
        part(
            "idler-20t",
            &meshes.idler_pinion,
            prototype.idler_pose(),
            [0.40, 0.80, 0.45],
        ),
        part("rack", &meshes.rack, identity, [0.90, 0.80, 0.25]),
        part(
            "top-plate",
            &meshes.top_plate,
            pose(0.0, 0.0, prototype.top_plate_center_z(), 0.0),
            [0.70, 0.72, 0.78],
        ),
        part(
            "bottom-plate",
            &meshes.bottom_plate,
            pose(0.0, 0.0, prototype.bottom_plate_center_z(), 0.0),
            [0.55, 0.58, 0.65],
        ),
    ];
    let handle_pose = prototype.handle_spur_pose();
    let top_outer_z = prototype.top_plate_center_z() + prototype.plate_thickness().mm() * 0.5;
    let crank_center_z = top_outer_z
        + prototype.nut_thickness().mm()
        + prototype.axial_clearance().mm()
        + prototype.nut_thickness().mm() * 0.5;
    let knob_height = prototype.rack().face_width().mm() + 5.0;
    let knob_center_z = crank_center_z + prototype.nut_thickness().mm() * 0.5 + knob_height * 0.5;
    let crank_angle = handle_pose.rotation_z_deg.to_radians();
    let crank_end_x =
        handle_pose.translation_mm[0] + prototype.handle_crank_radius().mm() * crank_angle.cos();
    let crank_end_y =
        handle_pose.translation_mm[1] + prototype.handle_crank_radius().mm() * crank_angle.sin();
    assembly.push(part(
        "handle-crank",
        &meshes.handle_crank,
        pose(
            handle_pose.translation_mm[0],
            handle_pose.translation_mm[1],
            crank_center_z,
            handle_pose.rotation_z_deg,
        ),
        [0.95, 0.30, 0.08],
    ));
    assembly.push(part(
        "handle-knob",
        &meshes.handle_knob,
        pose(
            crank_end_x,
            crank_end_y,
            knob_center_z,
            handle_pose.rotation_z_deg,
        ),
        [0.10, 0.10, 0.12],
    ));
    let handle_thrust_z = prototype.handle_spur_extended_center_z()
        + prototype.handle_spur_extended_face_width() * 0.5
        + prototype.handle_upper_thrust_spacer_length() * 0.5;
    let reduction_thrust_z = prototype.reduction_small_extended_center_z()
        + prototype.reduction_small_extended_face_width() * 0.5
        + prototype.reduction_upper_thrust_spacer_length() * 0.5;
    let driven_thrust_z = prototype.secondary_spur_layer_center_z()
        - prototype.spur_face_width().mm() * 0.5
        - prototype.driven_lower_thrust_spacer_length() * 0.5;
    let idler_thrust_z = -prototype.idler_pinion().face_width().mm() * 0.5
        - prototype.idler_lower_thrust_spacer_length() * 0.5;
    for (name, mesh, gear_pose, z, color) in [
        (
            "handle-upper-thrust-spacer",
            &meshes.handle_upper_thrust_spacer,
            prototype.handle_spur_pose(),
            handle_thrust_z,
            [0.95, 0.42, 0.38],
        ),
        (
            "reduction-d-upper-thrust-spacer",
            &meshes.reduction_upper_thrust_spacer,
            prototype.reduction_pose(),
            reduction_thrust_z,
            [0.85, 0.32, 0.72],
        ),
        (
            "driven-b-lower-thrust-spacer",
            &meshes.driven_lower_thrust_spacer,
            prototype.driven_b_pose(),
            driven_thrust_z,
            [0.12, 0.72, 0.78],
        ),
        (
            "driven-c-lower-thrust-spacer",
            &meshes.driven_lower_thrust_spacer,
            prototype.driven_c_pose(),
            driven_thrust_z,
            [0.12, 0.72, 0.78],
        ),
        (
            "idler-lower-thrust-spacer",
            &meshes.idler_lower_thrust_spacer,
            prototype.idler_pose(),
            idler_thrust_z,
            [0.38, 0.78, 0.32],
        ),
    ] {
        assembly.push(part(
            name,
            mesh,
            pose(
                gear_pose.translation_mm[0],
                gear_pose.translation_mm[1],
                z,
                0.0,
            ),
            color,
        ));
    }
    write_binary_stl(&assembly, &assembly_dir.join("prototype-assembly.stl"))?;
    write_3mf(&assembly, &assembly_dir.join("prototype-assembly.3mf"))?;
    write_obj(
        &assembly,
        &assembly_dir.join("prototype-assembly.obj"),
        &assembly_dir.join("prototype-assembly.mtl"),
    )?;
    fs::write(
        assembly_dir.join("prototype-preview.scad"),
        openscad_preview_scene(prototype),
    )?;
    Ok(())
}

fn openscad_preview_scene(prototype: &Prototype) -> String {
    let handle = prototype.handle_spur_pose();
    let reduction = prototype.reduction_pose();
    let driven_b = prototype.driven_b_pose();
    let driven_c = prototype.driven_c_pose();
    let idler = prototype.idler_pose();
    let mut scene =
        String::from("// Generated by double-helical-cli. Units: millimetres.\n$fn = 72;\n\n");
    writeln!(
        scene,
        "color([0.95,0.55,0.20]) translate([{:.9},{:.9},{:.9}]) rotate([0,0,{:.9}]) import(\"../print-parts/handle-spur.stl\", convexity=10);",
        handle.translation_mm[0],
        handle.translation_mm[1],
        handle.translation_mm[2],
        handle.rotation_z_deg
    )
    .unwrap();
    writeln!(
        scene,
        "color([0.20,0.24,0.32]) translate([{:.9},{:.9},{:.9}]) rotate([0,0,{:.9}]) import(\"../print-parts/handle-shaft.stl\", convexity=10);",
        handle.translation_mm[0],
        handle.translation_mm[1],
        handle.translation_mm[2],
        handle.rotation_z_deg
    )
    .unwrap();
    writeln!(
        scene,
        "color([0.72,0.38,0.82]) translate([{:.9},{:.9},{:.9}]) rotate([0,0,{:.9}]) import(\"../print-parts/reduction-d-compound.stl\", convexity=10);",
        reduction.translation_mm[0],
        reduction.translation_mm[1],
        reduction.translation_mm[2],
        reduction.rotation_z_deg
    )
    .unwrap();
    writeln!(
        scene,
        "color([0.20,0.55,0.95]) translate([{:.9},{:.9},{:.9}]) rotate([0,0,{:.9}]) import(\"../print-parts/driven-b-compound.stl\", convexity=10);",
        driven_b.translation_mm[0],
        driven_b.translation_mm[1],
        driven_b.translation_mm[2],
        driven_b.rotation_z_deg
    )
    .unwrap();
    writeln!(
        scene,
        "color([0.20,0.65,0.90]) translate([{:.9},{:.9},{:.9}]) rotate([0,0,{:.9}]) import(\"../print-parts/driven-c-compound.stl\", convexity=10);",
        driven_c.translation_mm[0],
        driven_c.translation_mm[1],
        driven_c.translation_mm[2],
        driven_c.rotation_z_deg
    )
    .unwrap();
    writeln!(
        scene,
        "color([0.40,0.80,0.45]) translate([{:.9},{:.9},{:.9}]) rotate([0,0,{:.9}]) import(\"../print-parts/idler-pinion.stl\", convexity=10);",
        idler.translation_mm[0],
        idler.translation_mm[1],
        idler.translation_mm[2],
        idler.rotation_z_deg
    )
    .unwrap();
    scene.push_str("color([0.90,0.80,0.25]) import(\"../print-parts/double-helical-rack.stl\", convexity=10);\n");
    writeln!(
        scene,
        "color([0.70,0.72,0.78,0.35]) translate([0,0,{:.9}]) import(\"../print-parts/top-frame-plate.stl\", convexity=10);",
        prototype.top_plate_center_z()
    )
    .unwrap();
    writeln!(
        scene,
        "color([0.55,0.58,0.65,0.65]) translate([0,0,{:.9}]) import(\"../print-parts/bottom-frame-plate.stl\", convexity=10);",
        prototype.bottom_plate_center_z()
    )
    .unwrap();
    let top_outer_z = prototype.top_plate_center_z() + prototype.plate_thickness().mm() * 0.5;
    let crank_center_z = top_outer_z
        + prototype.nut_thickness().mm()
        + prototype.axial_clearance().mm()
        + prototype.nut_thickness().mm() * 0.5;
    let knob_height = prototype.rack().face_width().mm() + 5.0;
    let knob_center_z = crank_center_z + prototype.nut_thickness().mm() * 0.5 + knob_height * 0.5;
    let crank_angle = handle.rotation_z_deg.to_radians();
    let crank_end_x =
        handle.translation_mm[0] + prototype.handle_crank_radius().mm() * crank_angle.cos();
    let crank_end_y =
        handle.translation_mm[1] + prototype.handle_crank_radius().mm() * crank_angle.sin();
    writeln!(
        scene,
        "color([0.95,0.30,0.08]) translate([{:.9},{:.9},{crank_center_z:.9}]) rotate([0,0,{:.9}]) import(\"../print-parts/handle-crank.stl\", convexity=10);",
        handle.translation_mm[0], handle.translation_mm[1], handle.rotation_z_deg
    )
    .unwrap();
    writeln!(
        scene,
        "color([0.10,0.10,0.12]) translate([{:.9},{:.9},{knob_center_z:.9}]) import(\"../print-parts/handle-knob.stl\", convexity=10);",
        crank_end_x,
        crank_end_y
    )
    .unwrap();
    let handle_thrust_z = prototype.handle_spur_extended_center_z()
        + prototype.handle_spur_extended_face_width() * 0.5
        + prototype.handle_upper_thrust_spacer_length() * 0.5;
    let reduction_thrust_z = prototype.reduction_small_extended_center_z()
        + prototype.reduction_small_extended_face_width() * 0.5
        + prototype.reduction_upper_thrust_spacer_length() * 0.5;
    let driven_thrust_z = prototype.secondary_spur_layer_center_z()
        - prototype.spur_face_width().mm() * 0.5
        - prototype.driven_lower_thrust_spacer_length() * 0.5;
    let idler_thrust_z = -prototype.idler_pinion().face_width().mm() * 0.5
        - prototype.idler_lower_thrust_spacer_length() * 0.5;
    for (filename, gear_pose, z, color) in [
        (
            "handle-upper-thrust-spacer.stl",
            handle,
            handle_thrust_z,
            [0.95, 0.42, 0.38],
        ),
        (
            "reduction-d-upper-thrust-spacer.stl",
            reduction,
            reduction_thrust_z,
            [0.85, 0.32, 0.72],
        ),
        (
            "driven-lower-thrust-spacer.stl",
            driven_b,
            driven_thrust_z,
            [0.12, 0.72, 0.78],
        ),
        (
            "driven-lower-thrust-spacer.stl",
            driven_c,
            driven_thrust_z,
            [0.12, 0.72, 0.78],
        ),
        (
            "idler-lower-thrust-spacer.stl",
            idler,
            idler_thrust_z,
            [0.38, 0.78, 0.32],
        ),
    ] {
        writeln!(
            scene,
            "color([{:.2},{:.2},{:.2}]) translate([{:.9},{:.9},{z:.9}]) import(\"../print-parts/{filename}\", convexity=10);",
            color[0], color[1], color[2], gear_pose.translation_mm[0], gear_pose.translation_mm[1]
        )
        .unwrap();
    }
    let label_z = prototype.top_plate_center_z() + prototype.plate_thickness().mm() * 0.5 + 0.2;
    for (label, x, y) in [
        ("A", idler.translation_mm[0], idler.translation_mm[1] + 29.0),
        (
            "B",
            driven_b.translation_mm[0],
            driven_b.translation_mm[1] - 34.0,
        ),
        (
            "C",
            driven_c.translation_mm[0],
            driven_c.translation_mm[1] - 34.0,
        ),
        (
            "D",
            reduction.translation_mm[0],
            reduction.translation_mm[1] - 17.0,
        ),
        (
            "H",
            handle.translation_mm[0],
            handle.translation_mm[1] - 20.0,
        ),
    ] {
        writeln!(
            scene,
            "color([0.08,0.08,0.10]) translate([{x:.9},{y:.9},{label_z:.9}]) linear_extrude(height=0.8) text(\"{label}\", size=8, halign=\"center\", valign=\"center\");"
        )
        .unwrap();
    }
    scene
}

fn part<'a>(
    name: &'a str,
    mesh: &'a double_helical_core::TriangleMesh,
    pose: GearPose,
    color_rgb: [f64; 3],
) -> ExportPart<'a> {
    ExportPart {
        name,
        mesh,
        pose,
        color_rgb,
    }
}

const fn pose(x: f64, y: f64, z: f64, rotation_z_deg: f64) -> GearPose {
    GearPose {
        translation_mm: [x, y, z],
        rotation_z_deg,
    }
}

fn report(
    prototype: &Prototype,
    meshes: &PrototypeMeshes,
    interference: PrototypeInterference,
) -> String {
    let mut output = String::new();
    writeln!(output, "Rack-and-pinion prototype generated").unwrap();
    writeln!(
        output,
        "spur reduction: {:.6}:1",
        prototype.reduction_ratio()
    )
    .unwrap();
    writeln!(
        output,
        "primary spur stage: handle {}T -> D-large {}T, ratio {:.6}:1, center distance {:.6} mm",
        prototype.handle_spur().teeth(),
        prototype.reduction_large_spur().teeth(),
        prototype.primary_reduction_ratio(),
        prototype.primary_spur_center_distance()
    )
    .unwrap();
    writeln!(
        output,
        "secondary spur stage: D-small {}T -> B/C {}T, ratio {:.6}:1, center distance {:.6} mm",
        prototype.reduction_small_spur().teeth(),
        prototype.output_spur().teeth(),
        prototype.secondary_reduction_ratio(),
        prototype.secondary_spur_center_distance()
    )
    .unwrap();
    writeln!(
        output,
        "spur face widths: handle-small {:.6}/D-large {:.6} mm, D-small {:.6}/B/C-large {:.6} mm; B/C-large to rack axial gap {:.6} mm",
        prototype.handle_spur_extended_face_width(),
        prototype.spur_face_width().mm(),
        prototype.reduction_small_extended_face_width(),
        prototype.spur_face_width().mm(),
        prototype.output_spur_to_rack_axial_gap()
    )
    .unwrap();
    writeln!(
        output,
        "final pinion: {}T, pitch diameter {:.6} mm, OD {:.6} mm",
        prototype.driven_pinion().spur().teeth(),
        prototype.driven_pinion().spur().pitch_radius() * 2.0,
        prototype.driven_pinion().spur().outside_diameter()
    )
    .unwrap();
    writeln!(
        output,
        "rack: {} teeth, toothed length {:.6} mm, overall length {:.6} mm, 30 x 20 mm flat pusher face",
        prototype.rack().teeth(),
        prototype.rack().length(),
        prototype.rack_overall_length()
    )
    .unwrap();
    writeln!(
        output,
        "M6x{:.0}: frame inner {:.6} mm, outer {:.6} mm, nut pocket {:.6} mm, thread engagement {:.6} mm",
        prototype.bolt_length().mm(),
        prototype.frame_spacer_length(),
        prototype.frame_outer_thickness_mm(),
        prototype.nut_pocket_depth().mm(),
        prototype.bolt_thread_engagement_mm()
    )
    .unwrap();
    writeln!(
        output,
        "fixed printed axle OD {:.6} mm, rotating bore {:.6} mm, bolt clearance {:.6} mm, nut AF {:.6} mm",
        prototype.journal_outer_diameter().mm(),
        prototype.driven_pinion().bore_diameter().mm(),
        prototype.bolt_clearance_diameter().mm(),
        prototype.nut_across_flats().mm()
    )
    .unwrap();
    writeln!(
        output,
        "thrust spacers OD {:.6} mm: handle upper {:.6}, D upper {:.6}, B/C lower {:.6} each, A lower {:.6} mm",
        prototype.thrust_spacer_outer_diameter().mm(),
        prototype.handle_upper_thrust_spacer_length(),
        prototype.reduction_upper_thrust_spacer_length(),
        prototype.driven_lower_thrust_spacer_length(),
        prototype.idler_lower_thrust_spacer_length()
    )
    .unwrap();
    writeln!(
        output,
        "integrated posts: length {:.6} mm, top socket depth {:.6} mm, axial clearance {:.6} mm, diameter clearance {:.6} mm",
        prototype.fixed_post_length(),
        prototype.top_socket_depth().mm(),
        prototype.top_socket_axial_clearance().mm(),
        prototype.top_socket_diameter_clearance().mm()
    )
    .unwrap();
    writeln!(
        output,
        "handle: {:.6} mm-wide separate spur with {:.6} mm gear socket/{:.6} mm square drive, bottom round taper {:.6}->{:.6} mm, top round taper {:.6}->{:.6} mm, crank {:.6}/{:.6} mm square drive/socket, radius {:.6} mm, M6 knob bore",
        prototype.handle_spur_extended_face_width(),
        prototype.handle_gear_square_socket_size(),
        prototype.handle_gear_square_shaft_size(),
        prototype.handle_bottom_taper_lower_diameter(),
        prototype.handle_bottom_taper_upper_diameter(),
        prototype.handle_top_taper_lower_diameter(),
        prototype.handle_top_taper_upper_diameter(),
        prototype.handle_crank_square_shaft_size(),
        prototype.handle_crank_square_socket_size(),
        prototype.handle_crank_radius().mm()
    )
    .unwrap();
    writeln!(
        output,
        "triangles: handle spur {}, handle shaft {}, compound D {}, compound B {}, compound C {}, idler {}, rack {}",
        meshes.handle_spur.triangles.len(),
        meshes.handle_shaft.triangles.len(),
        meshes.reduction_compound.triangles.len(),
        meshes.driven_b_compound.triangles.len(),
        meshes.driven_c_compound.triangles.len(),
        meshes.idler_pinion.triangles.len(),
        meshes.rack.triangles.len()
    )
    .unwrap();
    writeln!(
        output,
        "interference mm^3: handle-spur/shaft {:.9}, shaft/bottom-plate {:.9}, shaft/top-plate {:.9}, handle/D-large {:.9}, D-small/B {:.9}, D-small/C {:.9}, B/rack {:.9}, C/rack {:.9}, A/rack {:.9}",
        interference.handle_spur_to_shaft_mm3,
        interference.handle_shaft_to_bottom_plate_mm3,
        interference.handle_shaft_to_top_plate_mm3,
        interference.handle_to_reduction_large_mm3,
        interference.reduction_small_to_b_mm3,
        interference.reduction_small_to_c_mm3,
        interference.driven_b_pinion_to_rack_mm3,
        interference.driven_c_pinion_to_rack_mm3,
        interference.idler_pinion_to_rack_mm3
    )
    .unwrap();
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nominal_configuration_builds_requested_ratios() {
        let config: Config = toml::from_str(include_str!("../../../parameters.toml")).unwrap();
        let prototype = config.build().unwrap();
        assert_eq!(prototype.handle_spur().module().mm(), 1.8);
        assert_eq!(prototype.handle_spur().pressure_angle().as_degrees(), 25.0);
        assert_eq!(prototype.handle_spur().teeth(), 12);
        assert_eq!(prototype.reduction_large_spur().teeth(), 31);
        assert_eq!(prototype.reduction_small_spur().teeth(), 12);
        assert_eq!(prototype.output_spur().teeth(), 28);
        assert!((prototype.primary_reduction_ratio() - 31.0 / 12.0).abs() < 1.0e-12);
        assert!((prototype.secondary_reduction_ratio() - 28.0 / 12.0).abs() < 1.0e-12);
        assert!((prototype.reduction_ratio() - 217.0 / 36.0).abs() < 1.0e-12);
        assert_eq!(prototype.reduction_pose().translation_mm[0], 0.0);
        assert_eq!(prototype.handle_spur_pose().translation_mm[0], 0.0);
        assert!((prototype.handle_spur_pose().rotation_z_deg - 13.75).abs() < 1.0e-12);
        assert_eq!(
            prototype.driven_b_pose().translation_mm[0],
            -prototype.secondary_spur_center_distance()
        );
        assert_eq!(
            prototype.driven_c_pose().translation_mm[0],
            prototype.secondary_spur_center_distance()
        );
        assert_eq!(prototype.idler_pose().translation_mm[0], 0.0);
        assert_eq!(prototype.driven_pinion().center_gap().mm(), 0.0);
        assert_eq!(prototype.driven_pinion().face_width().mm(), 20.0);
        assert!(
            (prototype.driven_pinion().spur().pitch_radius() * 2.0 - 41.41104721640332).abs()
                < 1.0e-9
        );
        assert_eq!(prototype.rack().teeth(), 24);
        assert!((155.0..157.0).contains(&prototype.rack().length()));
        assert_eq!(prototype.rack_pusher_length(), 8.0);
        assert_eq!(prototype.rack_pusher_width(), 30.0);
        assert!((166.0..167.0).contains(&prototype.rack_overall_length()));
        assert!((prototype.frame_spacer_length() - 30.0).abs() < 1.0e-12);
        assert!((prototype.frame_outer_thickness_mm() - 38.0).abs() < 1.0e-12);
        assert!((prototype.bolt_thread_engagement_mm() - 5.0).abs() < 1.0e-12);
        assert_eq!(prototype.bolt_clearance_diameter().mm(), 6.4);
        assert_eq!(prototype.plate_length().mm(), 130.0);
        assert_eq!(prototype.plate_width().mm(), 130.0);
        assert_eq!(prototype.thrust_spacer_outer_diameter().mm(), 15.0);
        assert!((prototype.handle_upper_thrust_spacer_length() - 23.5).abs() < 1.0e-12);
        assert!((prototype.reduction_upper_thrust_spacer_length() - 20.0).abs() < 1.0e-12);
        assert!((prototype.driven_lower_thrust_spacer_length() - 3.5).abs() < 1.0e-12);
        assert!((prototype.idler_lower_thrust_spacer_length() - 9.0).abs() < 1.0e-12);
        assert!((prototype.fixed_post_length() - 31.5).abs() < 1.0e-12);
        assert_eq!(prototype.top_socket_depth().mm(), 2.0);
        assert_eq!(prototype.top_socket_axial_clearance().mm(), 0.5);
        assert_eq!(prototype.top_socket_diameter_clearance().mm(), 0.5);
        assert_eq!(prototype.handle_crank_radius().mm(), 40.0);
        assert_eq!(prototype.handle_gear_square_shaft_size(), 9.0);
        assert_eq!(prototype.handle_gear_square_socket_size(), 9.3);
        assert_eq!(prototype.handle_crank_square_shaft_size(), 6.0);
        assert_eq!(prototype.handle_crank_square_socket_size(), 6.3);
        assert_eq!(prototype.handle_bottom_taper_lower_diameter(), 9.0);
        assert_eq!(prototype.handle_bottom_taper_upper_diameter(), 11.0);
        assert_eq!(prototype.handle_top_taper_lower_diameter(), 9.0);
        assert_eq!(prototype.handle_top_taper_upper_diameter(), 8.6);
        assert!(
            prototype.handle_top_taper_lower_diameter()
                < prototype.handle_gear_square_socket_size()
        );
        assert!(
            prototype.handle_bottom_taper_upper_diameter()
                > prototype.handle_gear_square_socket_size()
        );
        assert!(
            prototype.handle_crank_square_shaft_size() * 2.0_f64.sqrt()
                < prototype.handle_top_taper_upper_diameter()
                    + prototype.handle_taper_hole_diameter_clearance()
        );
        let lateral_margin = prototype.plate_length().mm() * 0.5
            - (prototype.secondary_spur_center_distance() + prototype.output_spur().tip_radius());
        assert!((lateral_margin - 2.0).abs() < 1.0e-12);
        let case_min_y = prototype.plate_center_y() - prototype.plate_width().mm() * 0.5;
        let handle_min_y =
            prototype.handle_spur_pose().translation_mm[1] - prototype.handle_spur().tip_radius();
        assert!(handle_min_y - case_min_y > 0.5);
        let case_max_y = prototype.plate_center_y() + prototype.plate_width().mm() * 0.5;
        let idler_max_y =
            prototype.idler_pose().translation_mm[1] + prototype.idler_pinion().spur().tip_radius();
        assert!(case_max_y - idler_max_y > 0.75);

        let d_large_top =
            prototype.primary_spur_layer_center_z() + prototype.spur_face_width().mm() * 0.5;
        let d_small_bottom =
            prototype.secondary_spur_layer_center_z() - prototype.spur_face_width().mm() * 0.5;
        assert!((d_large_top - d_small_bottom).abs() < 1.0e-12);

        assert_eq!(prototype.spur_face_width().mm(), 3.5);
        assert_eq!(prototype.handle_spur_extended_face_width(), 5.5);
        assert_eq!(prototype.reduction_small_extended_face_width(), 5.5);
        let handle_small_bottom = prototype.handle_spur_extended_center_z()
            - prototype.handle_spur_extended_face_width() * 0.5;
        let d_large_bottom =
            prototype.primary_spur_layer_center_z() - prototype.spur_face_width().mm() * 0.5;
        assert!((handle_small_bottom - d_large_bottom).abs() < 1.0e-12);
        let d_small_bottom = prototype.reduction_small_extended_center_z()
            - prototype.reduction_small_extended_face_width() * 0.5;
        let output_large_bottom =
            prototype.secondary_spur_layer_center_z() - prototype.spur_face_width().mm() * 0.5;
        assert!((d_small_bottom - output_large_bottom).abs() < 1.0e-12);
        let output_spur_top =
            prototype.secondary_spur_layer_center_z() + prototype.spur_face_width().mm() * 0.5;
        let reduction_small_top = prototype.reduction_small_extended_center_z()
            + prototype.reduction_small_extended_face_width() * 0.5;
        let pinion_bottom = -prototype.driven_pinion().face_width().mm() * 0.5;
        assert!((prototype.output_spur_to_rack_axial_gap() - 2.0).abs() < 1.0e-12);
        assert!((pinion_bottom - output_spur_top - 2.0).abs() < 1.0e-12);
        assert!((reduction_small_top - pinion_bottom).abs() < 1.0e-12);
        assert!(
            prototype.output_spur().root_radius() >= prototype.driven_pinion().spur().tip_radius()
        );

        let meshes = build_prototype(&prototype).unwrap();
        assert!(meshes.bottom_plate.triangles.len() > meshes.top_plate.triangles.len());
    }
}

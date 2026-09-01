// SPDX-License-Identifier: MIT

mod config;

use std::error::Error;
use std::fs;
use std::path::Path;

use config::LoadedConfig;
use gimbal_core::{
    Angle, Body, Manufacturing, PitchRollCommand, RegionNode, TriangleMesh, build_prototype,
};
use gimbal_export::{
    AnimationParameters, ExportPart, sha256_file, write_3mf, write_animated_gltf, write_binary_stl,
    write_dxf_profile, write_mesh_3mf, write_obj,
};
use gimbal_kernel_manifold::Evaluator;
use serde_json::{Value, json};

fn main() {
    if let Err(error) = run() {
        eprintln!("error: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let command = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "generate".to_string());
    let workspace = std::env::current_dir()?;
    let loaded = config::load(
        &workspace.join("parameters.toml"),
        &workspace.join("fabrication.toml"),
    )?;
    match command.as_str() {
        "generate" => generate(&workspace, &loaded),
        "validate" => validate(&loaded),
        "clean-output" => clean_output(&workspace),
        unknown => Err(format!(
            "unknown command {unknown:?}; expected generate, validate, or clean-output"
        )
        .into()),
    }
}

fn validate(loaded: &LoadedConfig) -> Result<(), Box<dyn Error>> {
    let design = build_prototype(&loaded.parameters)
        .map_err(|error| format!("prototype design rejected: {error:?}"))?;
    let mut evaluator = Evaluator::new(&design.graph);
    for definition in design.assembly.definitions() {
        let metrics = evaluator.metrics(definition.body.assembly_solid())?;
        println!(
            "validated definition {:<38} {:>8} triangles {:>12.2} mm^3",
            definition.name, metrics.triangles, metrics.volume_mm3
        );
    }
    println!(
        "validation complete: {} definitions, {} instances, pitch drive {:.3}:1, gearbox {:.3}:1/stage, roll {:.3}:1",
        design.assembly.definitions().len(),
        design.assembly.instances().len(),
        design.pitch_drive_pair.ratio(),
        design.pitch_gearbox_pair.ratio(),
        design.roll_pair.ratio(),
    );
    Ok(())
}

fn generate(workspace: &Path, loaded: &LoadedConfig) -> Result<(), Box<dyn Error>> {
    let design = build_prototype(&loaded.parameters)
        .map_err(|error| format!("prototype design rejected: {error:?}"))?;
    let output = workspace.join("output");
    let model_dir = output.join("model");
    let animation_dir = output.join("animation");
    let preview_dir = output.join("preview");
    let fdm_dir = output.join("fabrication").join("fdm");
    let laser_dir = output.join("fabrication").join("laser");
    for directory in [
        &model_dir,
        &animation_dir,
        &preview_dir,
        &fdm_dir,
        &laser_dir,
    ] {
        fs::create_dir_all(directory)?;
    }

    let mut evaluator = Evaluator::new(&design.graph);
    let mut definition_meshes =
        Vec::<TriangleMesh>::with_capacity(design.assembly.definitions().len());
    let mut definition_manifest = Vec::new();
    let mut fabrication_artifacts = Vec::new();
    for (definition_index, definition) in design.assembly.definitions().iter().enumerate() {
        let solid = definition.body.assembly_solid();
        let mesh = evaluator.mesh(solid)?;
        let metrics = evaluator.metrics(solid)?;
        let quantity = design
            .assembly
            .instances()
            .iter()
            .filter(|instance| instance.definition.index() == definition_index)
            .count();
        definition_manifest.push(json!({
            "name": definition.name,
            "manufacturing": manufacturing_name(definition.manufacturing),
            "quantity": quantity,
            "vertices": metrics.vertices,
            "triangles": metrics.triangles,
            "volume_mm3": metrics.volume_mm3,
            "surface_area_mm2": metrics.surface_area_mm2
        }));
        match definition.manufacturing {
            Manufacturing::Fdm { .. } => {
                let path = fdm_dir.join(format!("{}.3mf", definition.name));
                write_mesh_3mf(&definition.name, &mesh, &path)?;
                fabrication_artifacts.push(path);
            }
            Manufacturing::LaserCut => {
                let Body::Sheet { profile, .. } = definition.body else {
                    return Err(format!(
                        "laser definition {:?} does not retain a nominal 2D profile",
                        definition.name
                    )
                    .into());
                };
                let Some(RegionNode::Polygon(points)) = design.graph.region(profile) else {
                    return Err("laser profile references an unknown region".into());
                };
                let path = laser_dir.join(format!("{}.dxf", definition.name));
                write_dxf_profile(points, &path)?;
                fabrication_artifacts.push(path);
            }
            Manufacturing::Purchased => {}
        }
        definition_meshes.push(mesh);
    }
    let zero_pose = design
        .kinematics
        .pose(PitchRollCommand {
            pitch: Angle::degrees(0.0).expect("zero angle is valid"),
            roll: Angle::degrees(0.0).expect("zero angle is valid"),
        })
        .map_err(|error| format!("zero pose rejected: {error:?}"))?;
    let mut export_parts = Vec::with_capacity(design.assembly.instances().len());
    let mut instance_manifest = Vec::with_capacity(design.assembly.instances().len());
    for instance in design.assembly.instances() {
        let definition = design
            .assembly
            .definition(instance.definition)
            .ok_or("assembly referenced an unknown definition")?;
        let frame_pose = zero_pose
            .frame(instance.frame)
            .ok_or("assembly referenced an unknown frame")?;
        let static_pose = frame_pose.compose(instance.local_pose);
        instance_manifest.push(json!({
            "name": instance.name,
            "definition": definition.name,
            "frame": instance.frame.index(),
            "static_translation_mm": static_pose.translation,
        }));
        export_parts.push(ExportPart {
            name: instance.name.clone(),
            definition: instance.definition,
            mesh: definition_meshes[instance.definition.index()].clone(),
            manufacturing: definition.manufacturing,
            frame: instance.frame,
            local_pose: instance.local_pose,
            static_pose,
            color_rgba: definition.color_rgba,
        });
    }

    let three_mf_path = model_dir.join("assembly.3mf");
    let stl_path = model_dir.join("assembly.stl");
    let obj_path = model_dir.join("assembly.obj");
    let mtl_path = model_dir.join("assembly.mtl");
    let gltf_path = animation_dir.join("gimbal-motion.gltf");
    let bin_path = animation_dir.join("gimbal-motion.bin");
    write_3mf(&export_parts, &three_mf_path)?;
    write_binary_stl(&export_parts, &stl_path)?;
    write_obj(&export_parts, &obj_path, &mtl_path)?;
    write_animated_gltf(
        &export_parts,
        &design.kinematics,
        AnimationParameters {
            pitch_limit_degrees: loaded.parameters.motion.pitch_limit.as_degrees(),
            roll_limit_degrees: loaded.parameters.motion.roll_limit.as_degrees(),
            duration_seconds: 6.0,
            sample_count: 73,
        },
        &gltf_path,
        &bin_path,
    )?;

    let mut artifact_paths = vec![
        three_mf_path,
        stl_path,
        obj_path,
        mtl_path,
        gltf_path,
        bin_path,
    ];
    artifact_paths.extend(fabrication_artifacts);
    for optional in [
        output.join("model/gimbal-prototype.blend"),
        output.join("preview/isometric.png"),
        output.join("preview/top-z.png"),
        output.join("preview/left-side-minus-y.png"),
        output.join("preview/front-plus-x.png"),
        output.join("preview/drive-unit-detail.png"),
        output.join("preview/pitch-gearbox-detail.png"),
        output.join("preview/roll-gearbox-detail.png"),
        output.join("preview/gimbal-motion.mp4"),
        output.join("preview/pitch-gearbox-motion.mp4"),
        output.join("preview/roll-gearbox-motion.mp4"),
    ] {
        if optional.is_file() {
            artifact_paths.push(optional);
        }
    }
    let artifacts = artifact_paths
        .iter()
        .map(|path| {
            let relative = path.strip_prefix(workspace).unwrap_or(path);
            Ok(json!({
                "path": relative.to_string_lossy().replace('\\', "/"),
                "bytes": fs::metadata(path)?.len(),
                "sha256": sha256_file(path)?
            }))
        })
        .collect::<Result<Vec<Value>, Box<dyn Error>>>()?;
    let sector = &loaded.parameters.pitch_sector.sector;
    let gearbox_stage_ratio = design.pitch_gearbox_pair.ratio();
    let manifest = json!({
        "schema_version": 3,
        "project": "pitch-roll cockpit attitude simulator prototype",
        "units": "millimeter",
        "status": "unpowered concept geometry only; not load-rated",
        "geometry": {
            "reference_outside_diameter_mm": sector.external_reference().outside_diameter(),
            "external_reference_teeth": sector.external_reference().teeth(),
            "internal_reference_teeth": sector.internal_reference().teeth(),
            "sector_half_angle_deg": sector.half_angle().as_degrees(),
            "physical_sector_count": 4,
            "pitch_sectors_ground_fixed": true,
            "carrier_count": 2,
            "contact_unit_count": 4,
            "contact_units_move_with_pitch": true,
            "drive_pinions_per_unit": 2,
            "drive_pinion_count": 8,
            "retention_encoder_pinion_count": 4,
            "pitch_gearbox_count": 4,
            "roll_gearbox_count": 2,
            "roll_gearboxes_below_roll_axis": true,
            "roll_mechanism_moves_with_pitch": true,
            "pitch_unit_to_roll_frame_brace_count": 8,
            "cockpit_length_mm": loaded.parameters.cockpit.length.mm(),
            "cockpit_suspension_drop_mm": loaded.parameters.cockpit.suspension_drop.mm(),
            "continuous_roll_shaft": true,
            "upper_rail_height_mm": loaded.parameters.frame.upper_rail_height.mm(),
            "lower_rail_depth_mm": loaded.parameters.frame.lower_rail_depth.mm(),
            "moving_crossbar_station_mm": loaded.parameters.frame.moving_crossbar_station.mm(),
            "floor_top_below_axis_mm": loaded.parameters.frame.floor_top_below_axis.mm(),
            "fixed_lower_frame_bears_directly_on_floor": true,
            "motor_bodies_included": false,
            "encoder_bodies_included": false
        },
        "ratios": {
            "pitch_drive_pinion_to_sector_reference": design.pitch_drive_pair.ratio(),
            "pitch_encoder_pinion_to_sector_reference": design.pitch_encoder_pair.ratio(),
            "pitch_gearbox_per_stage": gearbox_stage_ratio,
            "pitch_input_shaft_to_carrier": gearbox_stage_ratio * gearbox_stage_ratio * design.pitch_drive_pair.ratio(),
            "roll_pinion_to_cockpit": design.roll_pair.ratio(),
            "roll_input_shaft_to_cockpit": gearbox_stage_ratio * gearbox_stage_ratio * design.roll_pair.ratio()
        },
        "motion": {
            "pitch_limit_deg": loaded.parameters.motion.pitch_limit.as_degrees(),
            "roll_limit_deg": loaded.parameters.motion.roll_limit.as_degrees(),
            "yaw_degrees_of_freedom": 0
        },
        "process_profiles": {
            "fdm_material": loaded.fdm_material,
            "fdm_hole_compensation_mm": loaded.fdm_hole_compensation_mm,
            "laser_material": loaded.laser_material,
            "laser_kerf_mm": loaded.laser_kerf_mm,
            "laser_bed_mm": loaded.laser_bed_mm
        },
        "definitions": definition_manifest,
        "instances": instance_manifest,
        "artifacts": artifacts,
        "unverified": [
            "load capacity, stiffness, fatigue life, and tooth contact stress",
            "motor, encoder, bearing, brake, and emergency-stop selection",
            "leaf-spring rate and preload",
            "real manufactured backlash, runout, and load sharing",
            "production 2 m / 50-80 kg safety design"
        ]
    });
    fs::write(
        output.join("manifest.json"),
        serde_json::to_vec_pretty(&manifest)?,
    )?;
    println!(
        "generated {} component definitions and {} instances in {}",
        design.assembly.definitions().len(),
        export_parts.len(),
        output.display()
    );
    Ok(())
}

fn clean_output(workspace: &Path) -> Result<(), Box<dyn Error>> {
    let output = workspace.join("output");
    let canonical_workspace = workspace.canonicalize()?;
    if output.exists() {
        let canonical_output = output.canonicalize()?;
        if canonical_output.parent() != Some(canonical_workspace.as_path())
            || canonical_output.file_name().and_then(|name| name.to_str()) != Some("output")
        {
            return Err("refusing to remove an unexpected output path".into());
        }
        fs::remove_dir_all(&canonical_output)?;
        println!("removed {}", canonical_output.display());
    }
    Ok(())
}

fn manufacturing_name(manufacturing: Manufacturing) -> &'static str {
    match manufacturing {
        Manufacturing::Fdm { .. } => "fdm",
        Manufacturing::LaserCut => "laser-cut",
        Manufacturing::Purchased => "purchased",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gimbal_core::{PrototypeDesign, RigidTransform};

    fn load_design() -> PrototypeDesign {
        let workspace = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let loaded = config::load(
            &workspace.join("parameters.toml"),
            &workspace.join("fabrication.toml"),
        )
        .expect("repository parameters must be valid");
        build_prototype(&loaded.parameters).expect("repository design must be valid")
    }

    fn command(pitch: f64, roll: f64) -> PitchRollCommand {
        PitchRollCommand {
            pitch: Angle::degrees(pitch).expect("finite pitch"),
            roll: Angle::degrees(roll).expect("finite roll"),
        }
    }

    fn instance_pose(
        design: &PrototypeDesign,
        name: &str,
        pitch: f64,
        roll: f64,
    ) -> RigidTransform {
        let instance = design
            .assembly
            .instances()
            .iter()
            .find(|instance| instance.name == name)
            .unwrap_or_else(|| panic!("missing instance {name}"));
        design
            .kinematics
            .pose(command(pitch, roll))
            .expect("command within limits")
            .frame(instance.frame)
            .expect("instance frame exists")
            .compose(instance.local_pose)
    }

    fn instance_solid(design: &PrototypeDesign, name: &str) -> gimbal_core::SolidId {
        let instance = design
            .assembly
            .instances()
            .iter()
            .find(|instance| instance.name == name)
            .unwrap_or_else(|| panic!("missing instance {name}"));
        design
            .assembly
            .definition(instance.definition)
            .expect("instance definition exists")
            .body
            .assembly_solid()
    }

    fn count_prefix(design: &PrototypeDesign, prefix: &str) -> usize {
        design
            .assembly
            .instances()
            .iter()
            .filter(|instance| instance.name.starts_with(prefix))
            .count()
    }

    #[test]
    fn repository_design_has_the_required_reused_components() {
        let design = load_design();
        for name in [
            "pitch_sector_left_front",
            "pitch_sector_left_rear",
            "pitch_sector_right_front",
            "pitch_sector_right_rear",
        ] {
            assert!(
                design
                    .assembly
                    .instances()
                    .iter()
                    .any(|instance| instance.name == name),
                "missing fixed pitch sector {name}"
            );
        }
        assert_eq!(count_prefix(&design, "pitch_sector_left_front_"), 6);
        assert_eq!(count_prefix(&design, "pitch_drive_") / 5, 8);
        assert_eq!(count_prefix(&design, "pitch_retention_") / 7, 4);
        assert_eq!(count_prefix(&design, "roll_driven_gear_"), 2);
        assert_eq!(count_prefix(&design, "roll_output_pinion_"), 2);
        assert_eq!(count_prefix(&design, "roll_gearbox_front_input_pinion"), 1);
        assert_eq!(count_prefix(&design, "roll_gearbox_rear_input_pinion"), 1);
    }

    #[test]
    fn fixed_rack_stays_still_while_pinion_unit_orbits() {
        let design = load_design();
        let rack_zero = instance_pose(&design, "pitch_sector_left_front", 0.0, 0.0);
        let rack_pitch = instance_pose(&design, "pitch_sector_left_front", 20.0, 0.0);
        assert_eq!(rack_zero, rack_pitch);

        let pinion_zero = instance_pose(&design, "pitch_drive_left_front_1", 0.0, 0.0);
        let pinion_pitch = instance_pose(&design, "pitch_drive_left_front_1", 20.0, 0.0);
        assert_ne!(pinion_zero.translation, pinion_pitch.translation);
        let radius_zero = pinion_zero.translation[0].hypot(pinion_zero.translation[2]);
        let radius_pitch = pinion_pitch.translation[0].hypot(pinion_pitch.translation[2]);
        assert!((radius_zero - radius_pitch).abs() < 1.0e-8);

        let floor_zero = instance_pose(&design, "installation_floor_reference", 0.0, 0.0);
        let floor_pitch = instance_pose(&design, "installation_floor_reference", 20.0, 0.0);
        assert_eq!(floor_zero, floor_pitch);
    }

    #[test]
    fn pitch_drive_and_roll_mechanism_travel_as_one_moving_body() {
        let design = load_design();
        let moving_names = [
            "pitch_gearbox_left_front_contact_carriage_plate",
            "roll_gearbox_front_side_plate_1",
            "roll_shaft",
            "cockpit_body",
        ];
        for name in moving_names {
            let zero = instance_pose(&design, name, 0.0, 0.0);
            let pitched = instance_pose(&design, name, 20.0, 0.0);
            assert_ne!(zero, pitched, "{name} must follow pitch");
        }

        let rack_zero = instance_pose(&design, "pitch_sector_left_front", 0.0, 0.0);
        let rack_pitched = instance_pose(&design, "pitch_sector_left_front", 20.0, 0.0);
        assert_eq!(rack_zero, rack_pitched, "the ground rack must remain fixed");

        let roll_shaft_zero = instance_pose(&design, "roll_shaft", 0.0, 0.0);
        let roll_shaft_pitched = instance_pose(&design, "roll_shaft", 20.0, 0.0);
        let roll_gearbox_zero = instance_pose(&design, "roll_gearbox_front_side_plate_1", 0.0, 0.0);
        let roll_gearbox_pitched =
            instance_pose(&design, "roll_gearbox_front_side_plate_1", 20.0, 0.0);
        assert!(
            (distance(roll_shaft_zero, roll_gearbox_zero)
                - distance(roll_shaft_pitched, roll_gearbox_pitched))
            .abs()
                < 1.0e-8
        );
    }

    #[test]
    fn retention_supports_are_not_fixed_to_the_outer_frame() {
        let design = load_design();
        assert_eq!(count_prefix(&design, "fixed_frame_floor_support_"), 0);
        for name in [
            "pitch_retention_left_front_bearing_block",
            "pitch_retention_right_rear_leaf_spring_1",
        ] {
            let zero = instance_pose(&design, name, 0.0, 0.0);
            let pitched = instance_pose(&design, name, 20.0, 0.0);
            assert_ne!(zero, pitched, "{name} must travel with the pitch unit");
        }
    }

    #[test]
    fn base_frame_contacts_floor_and_roll_gearboxes_are_below_axis() {
        let design = load_design();
        let floor = instance_pose(&design, "installation_floor_reference", 0.0, 0.0);
        let lower_rail = instance_pose(&design, "pitch_carrier_left_lower_rail", 0.0, 0.0);
        let floor_top = floor.translation[2] + 5.0;
        let rail_bottom = lower_rail.translation[2] - 4.0;
        assert!((floor_top - rail_bottom).abs() < 1.0e-8);

        for end in ["front", "rear"] {
            let driven = instance_pose(&design, &format!("roll_driven_gear_{end}"), 0.0, 0.0);
            let input = instance_pose(
                &design,
                &format!("roll_gearbox_{end}_input_pinion"),
                0.0,
                0.0,
            );
            let plate = instance_pose(
                &design,
                &format!("roll_gearbox_{end}_side_plate_1"),
                0.0,
                0.0,
            );
            assert!(input.translation[2] < driven.translation[2]);
            assert!(plate.translation[2] < driven.translation[2]);
        }
    }

    #[test]
    fn cockpit_is_suspended_and_gravity_has_a_restoring_direction() {
        let design = load_design();
        let shaft = instance_pose(&design, "roll_shaft", 0.0, 0.0);
        let cockpit_zero = instance_pose(&design, "cockpit_body", 0.0, 0.0);
        let cockpit_rolled = instance_pose(&design, "cockpit_body", 0.0, 35.0);
        assert!(cockpit_zero.translation[2] < shaft.translation[2]);
        assert!(cockpit_zero.translation[2] < cockpit_rolled.translation[2]);
    }

    #[test]
    fn pitch_pinion_spin_includes_orbit_about_the_fixed_rack() {
        let design = load_design();
        let pitch = 1.0_f64;
        let drive = instance_pose(&design, "pitch_drive_left_front_1", pitch, 0.0);
        let encoder = instance_pose(&design, "pitch_retention_left_front", pitch, 0.0);
        let drive_angle = quaternion_y_degrees(drive.rotation);
        let encoder_angle = quaternion_y_degrees(encoder.rotation);
        let expected_drive = pitch * (1.0 - design.pitch_drive_pair.ratio());
        let expected_encoder = pitch * (1.0 + design.pitch_encoder_pair.ratio());
        assert!((drive_angle - expected_drive).abs() < 1.0e-6);
        assert!((encoder_angle - expected_encoder).abs() < 1.0e-6);
    }

    #[test]
    fn moving_assembly_clears_the_floor_over_the_command_envelope() {
        let design = load_design();
        let floor_name = "installation_floor_reference";
        let floor_solid = instance_solid(&design, floor_name);
        let floor_pose = instance_pose(&design, floor_name, 0.0, 0.0);
        let watched = [
            "cockpit_body",
            "pitch_moving_crossbar_front",
            "pitch_moving_crossbar_rear",
            "roll_gearbox_front_side_plate_1",
            "roll_gearbox_front_side_plate_2",
            "roll_gearbox_rear_side_plate_1",
            "roll_gearbox_rear_side_plate_2",
            "roll_gearbox_front_mount_arm_1",
            "roll_gearbox_front_mount_arm_2",
            "roll_gearbox_rear_mount_arm_1",
            "roll_gearbox_rear_mount_arm_2",
            "pitch_contact_left_front_lower_cradle_brace",
            "pitch_contact_right_front_lower_cradle_brace",
            "pitch_contact_left_rear_lower_cradle_brace",
            "pitch_contact_right_rear_lower_cradle_brace",
            "pitch_contact_left_front_upper_cradle_brace",
            "pitch_contact_right_front_upper_cradle_brace",
            "pitch_contact_left_rear_upper_cradle_brace",
            "pitch_contact_right_rear_upper_cradle_brace",
            "pitch_cradle_longitudinal_rail_1",
            "pitch_cradle_longitudinal_rail_2",
            "pitch_end_upper_tie_front",
            "pitch_end_upper_tie_rear",
        ];
        let mut evaluator = Evaluator::new(&design.graph);
        for pitch in [-20.0, 0.0, 20.0] {
            for roll in [-35.0, 0.0, 35.0] {
                for name in watched {
                    let volume = evaluator
                        .intersection_volume_transformed(
                            floor_solid,
                            floor_pose,
                            instance_solid(&design, name),
                            instance_pose(&design, name, pitch, roll),
                        )
                        .expect("floor interference query succeeds");
                    assert!(
                        volume <= 1.0e-7,
                        "{name} intersects the floor by {volume} mm^3 at pitch={pitch}, roll={roll}"
                    );
                }
            }
        }
    }

    #[test]
    fn pitch_sector_backbone_and_end_joints_form_a_continuous_load_path() {
        let design = load_design();
        let mut evaluator = Evaluator::new(&design.graph);
        let representative_sector = instance_solid(&design, "pitch_sector_left_front");
        let sector_mesh = evaluator
            .mesh(representative_sector)
            .expect("reinforced sector evaluates to a manifold mesh");
        let minimum_y = sector_mesh
            .vertices
            .iter()
            .map(|vertex| vertex[1])
            .fold(f64::INFINITY, f64::min);
        let maximum_y = sector_mesh
            .vertices
            .iter()
            .map(|vertex| vertex[1])
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            maximum_y - minimum_y >= 15.9,
            "sector backbone must retain its 16 mm axial depth"
        );
        for side in ["left", "right"] {
            for end in ["front", "rear"] {
                let sector = format!("pitch_sector_{side}_{end}");
                let upper_clamp = format!("{sector}_upper_end_clamp");
                let lower_clamp = format!("{sector}_lower_end_clamp");
                let upper_rail = format!("pitch_carrier_{side}_upper_rail");
                let lower_link = format!("pitch_carrier_{side}_{end}_lower_link");
                let lower_gusset = format!("pitch_carrier_{side}_{end}_lower_gusset");
                let lower_rail = format!("pitch_carrier_{side}_lower_rail");

                for (a, b) in [
                    (&sector, &upper_clamp),
                    (&upper_clamp, &upper_rail),
                    (&sector, &lower_clamp),
                    (&lower_clamp, &lower_link),
                    (&lower_link, &lower_gusset),
                    (&lower_gusset, &lower_rail),
                ] {
                    let volume = evaluator
                        .intersection_volume_transformed(
                            instance_solid(&design, a),
                            instance_pose(&design, a, 0.0, 0.0),
                            instance_solid(&design, b),
                            instance_pose(&design, b, 0.0, 0.0),
                        )
                        .expect("structural connection query succeeds");
                    assert!(
                        volume > 0.1,
                        "structural load path is disconnected between {a} and {b}"
                    );
                }
            }
        }
        assert_eq!(
            count_prefix(&design, "pitch_carrier_left_front_upper_link"),
            0
        );
        assert_eq!(
            count_prefix(&design, "pitch_carrier_left_front_lower_joint_m3_"),
            3
        );
    }

    #[test]
    fn shortened_cockpit_clears_pitch_frame_roll_supports() {
        let design = load_design();
        let cockpit_solid = instance_solid(&design, "cockpit_body");
        let fixed_to_pitch_frame = [
            "roll_bearing_pedestal_front",
            "roll_bearing_pedestal_rear",
            "roll_gearbox_front_carrier_mount_1",
            "roll_gearbox_front_carrier_mount_2",
            "roll_gearbox_rear_carrier_mount_1",
            "roll_gearbox_rear_carrier_mount_2",
            "roll_gearbox_front_mount_arm_1",
            "roll_gearbox_front_mount_arm_2",
            "roll_gearbox_rear_mount_arm_1",
            "roll_gearbox_rear_mount_arm_2",
            "pitch_end_upper_tie_front",
            "pitch_end_upper_tie_rear",
        ];
        let mut evaluator = Evaluator::new(&design.graph);
        for pitch in [-20.0, 0.0, 20.0] {
            for roll in [-35.0, 0.0, 35.0] {
                let cockpit_pose = instance_pose(&design, "cockpit_body", pitch, roll);
                for support in fixed_to_pitch_frame {
                    let volume = evaluator
                        .intersection_volume_transformed(
                            cockpit_solid,
                            cockpit_pose,
                            instance_solid(&design, support),
                            instance_pose(&design, support, pitch, roll),
                        )
                        .expect("cockpit clearance query succeeds");
                    assert!(
                        volume <= 1.0e-7,
                        "cockpit intersects {support} by {volume} mm^3 at pitch={pitch}, roll={roll}"
                    );
                }
            }
        }
    }

    #[test]
    fn gearbox_plates_clear_their_gears_and_shafts() {
        let design = load_design();
        let mut evaluator = Evaluator::new(&design.graph);
        let groups: &[(&str, &[&str])] = &[
            (
                "pitch_gearbox_right_front_contact_carriage_plate",
                &[
                    "pitch_drive_right_front_1",
                    "pitch_drive_right_front_2",
                    "pitch_drive_right_front_1_distribution_branch",
                    "pitch_drive_right_front_2_distribution_branch",
                    "pitch_gearbox_right_front_distributor",
                    "pitch_gearbox_right_front_stage2_driven",
                    "pitch_gearbox_right_front_stage2_pinion",
                    "pitch_gearbox_right_front_stage1_driven",
                    "pitch_gearbox_right_front_input_pinion",
                ],
            ),
            (
                "pitch_contact_right_front_inboard_plate",
                &[
                    "pitch_drive_right_front_1",
                    "pitch_drive_right_front_2",
                    "pitch_retention_right_front",
                    "pitch_drive_right_front_1_shaft",
                    "pitch_drive_right_front_2_shaft",
                    "pitch_retention_right_front_interface_shaft",
                ],
            ),
            (
                "pitch_gearbox_right_front_far_plate",
                &[
                    "pitch_gearbox_right_front_stage2_driven",
                    "pitch_gearbox_right_front_stage2_pinion",
                    "pitch_gearbox_right_front_stage1_driven",
                    "pitch_gearbox_right_front_input_pinion",
                    "pitch_gearbox_right_front_distributor_shaft",
                    "pitch_gearbox_right_front_compound_shaft",
                    "pitch_gearbox_right_front_input_shaft",
                ],
            ),
            (
                "roll_gearbox_front_side_plate_1",
                &[
                    "roll_output_pinion_front",
                    "roll_gearbox_front_stage2_driven",
                    "roll_gearbox_front_stage2_pinion",
                    "roll_gearbox_front_stage1_driven",
                    "roll_gearbox_front_input_pinion",
                    "roll_gearbox_front_output_shaft",
                    "roll_gearbox_front_compound_shaft",
                    "roll_gearbox_front_input_shaft",
                ],
            ),
        ];
        for (plate, parts) in groups {
            for part in *parts {
                let volume = evaluator
                    .intersection_volume_transformed(
                        instance_solid(&design, plate),
                        instance_pose(&design, plate, 0.0, 0.0),
                        instance_solid(&design, part),
                        instance_pose(&design, part, 0.0, 0.0),
                    )
                    .expect("gearbox interference query succeeds");
                assert!(
                    volume <= 1.0e-7,
                    "{plate} intersects {part} by {volume} mm^3"
                );
            }
        }
    }

    fn quaternion_y_degrees(rotation: [f64; 4]) -> f64 {
        2.0 * rotation[1].atan2(rotation[3]) * 180.0 / core::f64::consts::PI
    }

    fn distance(a: RigidTransform, b: RigidTransform) -> f64 {
        let dx = a.translation[0] - b.translation[0];
        let dy = a.translation[1] - b.translation[1];
        let dz = a.translation[2] - b.translation[2];
        dx.hypot(dy).hypot(dz)
    }
}

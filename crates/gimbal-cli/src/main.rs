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
            "role": format!("{:?}", definition.role),
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
    for (instance_index, instance) in design.assembly.instances().iter().enumerate() {
        let definition = design
            .assembly
            .definition(instance.definition)
            .ok_or("assembly referenced an unknown definition")?;
        let frame_pose = zero_pose
            .frame(instance.frame)
            .ok_or("assembly referenced an unknown frame")?;
        let static_pose = frame_pose.compose(instance.local_pose);
        instance_manifest.push(json!({
            "id": instance_index,
            "name": instance.name,
            "definition": definition.name,
            "role": format!("{:?}", definition.role),
            "location": {
                "side": instance.location.side.map(|value| value.as_str()),
                "longitudinal_end": instance.location.longitudinal_end.map(|value| value.as_str()),
                "vertical_end": instance.location.vertical_end.map(|value| value.as_str()),
                "ordinal": instance.location.ordinal,
            },
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
    use gimbal_core::{
        ComponentIdentity, ComponentInstance, ComponentLocation, ComponentRole, LongitudinalEnd,
        PrototypeDesign, RigidTransform, Side, VerticalEnd,
    };

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

    const fn singleton(role: ComponentRole) -> ComponentIdentity {
        ComponentIdentity {
            role,
            location: ComponentLocation::new(),
        }
    }

    const fn located(role: ComponentRole, location: ComponentLocation) -> ComponentIdentity {
        ComponentIdentity { role, location }
    }

    fn selected_instance(
        design: &PrototypeDesign,
        identity: ComponentIdentity,
    ) -> &ComponentInstance {
        let mut matches = design
            .assembly
            .instances_with_role(identity.role)
            .filter(|(_, instance)| instance.location == identity.location);
        let (_, instance) = matches
            .next()
            .unwrap_or_else(|| panic!("missing component identity {identity:?}"));
        assert!(
            matches.next().is_none(),
            "component identity is not unique: {identity:?}"
        );
        instance
    }

    fn instance_pose(
        design: &PrototypeDesign,
        identity: ComponentIdentity,
        pitch: f64,
        roll: f64,
    ) -> RigidTransform {
        let instance = selected_instance(design, identity);
        design
            .kinematics
            .pose(command(pitch, roll))
            .expect("command within limits")
            .frame(instance.frame)
            .expect("instance frame exists")
            .compose(instance.local_pose)
    }

    fn instance_solid(
        design: &PrototypeDesign,
        identity: ComponentIdentity,
    ) -> gimbal_core::SolidId {
        let instance = selected_instance(design, identity);
        design
            .assembly
            .definition(instance.definition)
            .expect("instance definition exists")
            .body
            .assembly_solid()
    }

    fn count_role(design: &PrototypeDesign, role: ComponentRole) -> usize {
        design.assembly.instances_with_role(role).count()
    }

    #[test]
    fn repository_design_has_the_required_reused_components() {
        let design = load_design();
        let identity_collisions = design.assembly.component_identity_collisions();
        assert!(
            identity_collisions.is_empty(),
            "component semantic identities must be unique: {identity_collisions:#?}"
        );
        assert_eq!(count_role(&design, ComponentRole::PitchSector), 4);
        assert_eq!(count_role(&design, ComponentRole::PitchDrivePinion), 8);
        assert_eq!(count_role(&design, ComponentRole::PitchRetentionPinion), 4);
        assert_eq!(count_role(&design, ComponentRole::RollDrivenGear), 2);
        assert_eq!(count_role(&design, ComponentRole::RollInputPinion), 2);
        for side in [Side::Left, Side::Right] {
            for end in [LongitudinalEnd::Front, LongitudinalEnd::Rear] {
                assert!(
                    design
                        .assembly
                        .instances_with_role(ComponentRole::PitchSector)
                        .any(|(_, instance)| {
                            instance.location.side == Some(side)
                                && instance.location.longitudinal_end == Some(end)
                        })
                );
                for ordinal in [1, 2] {
                    assert!(
                        design
                            .assembly
                            .instances_with_role(ComponentRole::PitchDrivePinion)
                            .any(|(_, instance)| {
                                instance.location.side == Some(side)
                                    && instance.location.longitudinal_end == Some(end)
                                    && instance.location.ordinal == Some(ordinal)
                            })
                    );
                }
            }
        }
        assert_eq!(count_role(&design, ComponentRole::FixedCrossmember), 4);
        for end in [LongitudinalEnd::Front, LongitudinalEnd::Rear] {
            selected_instance(
                &design,
                located(
                    ComponentRole::RollGearboxSmallGear,
                    ComponentLocation::new()
                        .with_longitudinal_end(end)
                        .with_ordinal(2),
                ),
            );
        }
    }

    #[test]
    fn fixed_rack_stays_still_while_pinion_unit_orbits() {
        let design = load_design();
        let rack = located(
            ComponentRole::PitchSector,
            ComponentLocation::new()
                .with_side(Side::Left)
                .with_longitudinal_end(LongitudinalEnd::Front),
        );
        let pinion = located(
            ComponentRole::PitchDrivePinion,
            ComponentLocation::new()
                .with_side(Side::Left)
                .with_longitudinal_end(LongitudinalEnd::Front)
                .with_ordinal(1),
        );
        let floor = singleton(ComponentRole::InstallationFloor);
        let rack_zero = instance_pose(&design, rack, 0.0, 0.0);
        let rack_pitch = instance_pose(&design, rack, 20.0, 0.0);
        assert_eq!(rack_zero, rack_pitch);

        let pinion_zero = instance_pose(&design, pinion, 0.0, 0.0);
        let pinion_pitch = instance_pose(&design, pinion, 20.0, 0.0);
        assert_ne!(pinion_zero.translation, pinion_pitch.translation);
        let radius_zero = pinion_zero.translation[0].hypot(pinion_zero.translation[2]);
        let radius_pitch = pinion_pitch.translation[0].hypot(pinion_pitch.translation[2]);
        assert!((radius_zero - radius_pitch).abs() < 1.0e-8);

        let floor_zero = instance_pose(&design, floor, 0.0, 0.0);
        let floor_pitch = instance_pose(&design, floor, 20.0, 0.0);
        assert_eq!(floor_zero, floor_pitch);
    }

    #[test]
    fn pitch_drive_and_roll_mechanism_travel_as_one_moving_body() {
        let design = load_design();
        let moving_components = [
            located(
                ComponentRole::PitchContactCarriagePlate,
                ComponentLocation::new()
                    .with_side(Side::Left)
                    .with_longitudinal_end(LongitudinalEnd::Front),
            ),
            located(
                ComponentRole::RollGearboxPlate,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Front)
                    .with_ordinal(1),
            ),
            singleton(ComponentRole::RollShaft),
            singleton(ComponentRole::Cockpit),
        ];
        for component in moving_components {
            let zero = instance_pose(&design, component, 0.0, 0.0);
            let pitched = instance_pose(&design, component, 20.0, 0.0);
            assert_ne!(zero, pitched, "{component:?} must follow pitch");
        }

        let rack = located(
            ComponentRole::PitchSector,
            ComponentLocation::new()
                .with_side(Side::Left)
                .with_longitudinal_end(LongitudinalEnd::Front),
        );
        let rack_zero = instance_pose(&design, rack, 0.0, 0.0);
        let rack_pitched = instance_pose(&design, rack, 20.0, 0.0);
        assert_eq!(rack_zero, rack_pitched, "the ground rack must remain fixed");

        let roll_shaft = singleton(ComponentRole::RollShaft);
        let roll_gearbox_plate = located(
            ComponentRole::RollGearboxPlate,
            ComponentLocation::new()
                .with_longitudinal_end(LongitudinalEnd::Front)
                .with_ordinal(1),
        );
        let roll_shaft_zero = instance_pose(&design, roll_shaft, 0.0, 0.0);
        let roll_shaft_pitched = instance_pose(&design, roll_shaft, 20.0, 0.0);
        let roll_gearbox_zero = instance_pose(&design, roll_gearbox_plate, 0.0, 0.0);
        let roll_gearbox_pitched = instance_pose(&design, roll_gearbox_plate, 20.0, 0.0);
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
        for component in [
            located(
                ComponentRole::RetentionBearingBlock,
                ComponentLocation::new()
                    .with_side(Side::Left)
                    .with_longitudinal_end(LongitudinalEnd::Front),
            ),
            located(
                ComponentRole::RetentionLeafSpring,
                ComponentLocation::new()
                    .with_side(Side::Right)
                    .with_longitudinal_end(LongitudinalEnd::Rear)
                    .with_ordinal(1),
            ),
        ] {
            let zero = instance_pose(&design, component, 0.0, 0.0);
            let pitched = instance_pose(&design, component, 20.0, 0.0);
            assert_ne!(
                zero, pitched,
                "{component:?} must travel with the pitch unit"
            );
        }
    }

    #[test]
    fn base_frame_contacts_floor_and_roll_gearboxes_are_below_axis() {
        let design = load_design();
        let floor = instance_pose(
            &design,
            singleton(ComponentRole::InstallationFloor),
            0.0,
            0.0,
        );
        let lower_rail = instance_pose(
            &design,
            located(
                ComponentRole::FixedCarrierRail,
                ComponentLocation::new()
                    .with_side(Side::Left)
                    .with_vertical_end(VerticalEnd::Lower),
            ),
            0.0,
            0.0,
        );
        let floor_top = floor.translation[2] + 5.0;
        let rail_bottom = lower_rail.translation[2] - 4.0;
        assert!((floor_top - rail_bottom).abs() < 1.0e-8);

        for end in [LongitudinalEnd::Front, LongitudinalEnd::Rear] {
            let location = ComponentLocation::new().with_longitudinal_end(end);
            let driven = instance_pose(
                &design,
                located(ComponentRole::RollDrivenGear, location),
                0.0,
                0.0,
            );
            let input = instance_pose(
                &design,
                located(
                    ComponentRole::RollGearboxSmallGear,
                    location.with_ordinal(2),
                ),
                0.0,
                0.0,
            );
            let plate = instance_pose(
                &design,
                located(ComponentRole::RollGearboxPlate, location.with_ordinal(1)),
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
        let shaft = instance_pose(&design, singleton(ComponentRole::RollShaft), 0.0, 0.0);
        let cockpit = singleton(ComponentRole::Cockpit);
        let cockpit_zero = instance_pose(&design, cockpit, 0.0, 0.0);
        let cockpit_rolled = instance_pose(&design, cockpit, 0.0, 35.0);
        assert!(cockpit_zero.translation[2] < shaft.translation[2]);
        assert!(cockpit_zero.translation[2] < cockpit_rolled.translation[2]);
    }

    #[test]
    fn pitch_pinion_spin_includes_orbit_about_the_fixed_rack() {
        let design = load_design();
        let pitch = 1.0_f64;
        let drive = instance_pose(
            &design,
            located(
                ComponentRole::PitchDrivePinion,
                ComponentLocation::new()
                    .with_side(Side::Left)
                    .with_longitudinal_end(LongitudinalEnd::Front)
                    .with_ordinal(1),
            ),
            pitch,
            0.0,
        );
        let encoder = instance_pose(
            &design,
            located(
                ComponentRole::PitchRetentionPinion,
                ComponentLocation::new()
                    .with_side(Side::Left)
                    .with_longitudinal_end(LongitudinalEnd::Front),
            ),
            pitch,
            0.0,
        );
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
        let floor = singleton(ComponentRole::InstallationFloor);
        let floor_solid = instance_solid(&design, floor);
        let floor_pose = instance_pose(&design, floor, 0.0, 0.0);
        let watched = [
            singleton(ComponentRole::Cockpit),
            located(
                ComponentRole::MovingCrossbar,
                ComponentLocation::new().with_longitudinal_end(LongitudinalEnd::Front),
            ),
            located(
                ComponentRole::MovingCrossbar,
                ComponentLocation::new().with_longitudinal_end(LongitudinalEnd::Rear),
            ),
            located(
                ComponentRole::RollGearboxPlate,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Front)
                    .with_ordinal(1),
            ),
            located(
                ComponentRole::RollGearboxPlate,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Front)
                    .with_ordinal(2),
            ),
            located(
                ComponentRole::RollGearboxPlate,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Rear)
                    .with_ordinal(1),
            ),
            located(
                ComponentRole::RollGearboxPlate,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Rear)
                    .with_ordinal(2),
            ),
            located(
                ComponentRole::MovingDriveMountArm,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Front)
                    .with_ordinal(1),
            ),
            located(
                ComponentRole::MovingDriveMountArm,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Front)
                    .with_ordinal(2),
            ),
            located(
                ComponentRole::MovingDriveMountArm,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Rear)
                    .with_ordinal(1),
            ),
            located(
                ComponentRole::MovingDriveMountArm,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Rear)
                    .with_ordinal(2),
            ),
            located(
                ComponentRole::PitchUnitLowerFrameArm,
                ComponentLocation::new()
                    .with_side(Side::Left)
                    .with_longitudinal_end(LongitudinalEnd::Front)
                    .with_vertical_end(VerticalEnd::Lower),
            ),
            located(
                ComponentRole::PitchUnitLowerFrameArm,
                ComponentLocation::new()
                    .with_side(Side::Right)
                    .with_longitudinal_end(LongitudinalEnd::Front)
                    .with_vertical_end(VerticalEnd::Lower),
            ),
            located(
                ComponentRole::PitchUnitLowerFrameArm,
                ComponentLocation::new()
                    .with_side(Side::Left)
                    .with_longitudinal_end(LongitudinalEnd::Rear)
                    .with_vertical_end(VerticalEnd::Lower),
            ),
            located(
                ComponentRole::PitchUnitLowerFrameArm,
                ComponentLocation::new()
                    .with_side(Side::Right)
                    .with_longitudinal_end(LongitudinalEnd::Rear)
                    .with_vertical_end(VerticalEnd::Lower),
            ),
            located(
                ComponentRole::PitchUnitUpperFrameArm,
                ComponentLocation::new()
                    .with_side(Side::Left)
                    .with_longitudinal_end(LongitudinalEnd::Front)
                    .with_vertical_end(VerticalEnd::Upper),
            ),
            located(
                ComponentRole::PitchUnitUpperFrameArm,
                ComponentLocation::new()
                    .with_side(Side::Right)
                    .with_longitudinal_end(LongitudinalEnd::Front)
                    .with_vertical_end(VerticalEnd::Upper),
            ),
            located(
                ComponentRole::PitchUnitUpperFrameArm,
                ComponentLocation::new()
                    .with_side(Side::Left)
                    .with_longitudinal_end(LongitudinalEnd::Rear)
                    .with_vertical_end(VerticalEnd::Upper),
            ),
            located(
                ComponentRole::PitchUnitUpperFrameArm,
                ComponentLocation::new()
                    .with_side(Side::Right)
                    .with_longitudinal_end(LongitudinalEnd::Rear)
                    .with_vertical_end(VerticalEnd::Upper),
            ),
            located(
                ComponentRole::PitchCradleLongitudinalRail,
                ComponentLocation::new().with_ordinal(1),
            ),
            located(
                ComponentRole::PitchCradleLongitudinalRail,
                ComponentLocation::new().with_ordinal(2),
            ),
            located(
                ComponentRole::PitchEndUpperTie,
                ComponentLocation::new().with_longitudinal_end(LongitudinalEnd::Front),
            ),
            located(
                ComponentRole::PitchEndUpperTie,
                ComponentLocation::new().with_longitudinal_end(LongitudinalEnd::Rear),
            ),
        ];
        let mut evaluator = Evaluator::new(&design.graph);
        for pitch in [-20.0, 0.0, 20.0] {
            for roll in [-35.0, 0.0, 35.0] {
                for component in watched {
                    let volume = evaluator
                        .intersection_volume_transformed(
                            floor_solid,
                            floor_pose,
                            instance_solid(&design, component),
                            instance_pose(&design, component, pitch, roll),
                        )
                        .expect("floor interference query succeeds");
                    assert!(
                        volume <= 1.0e-7,
                        "{component:?} intersects the floor by {volume} mm^3 at pitch={pitch}, roll={roll}"
                    );
                }
            }
        }
    }

    #[test]
    fn obsolete_overlapping_sector_reinforcement_is_absent() {
        let design = load_design();
        let mut evaluator = Evaluator::new(&design.graph);
        let sector = located(
            ComponentRole::PitchSector,
            ComponentLocation::new()
                .with_side(Side::Left)
                .with_longitudinal_end(LongitudinalEnd::Front),
        );
        let sector_mesh = evaluator
            .mesh(instance_solid(&design, sector))
            .expect("unreinforced sector evaluates to a manifold mesh");
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
            maximum_y - minimum_y <= 8.01,
            "obsolete 16 mm sector backbone is still present"
        );
    }

    #[test]
    fn shortened_cockpit_clears_pitch_frame_roll_supports() {
        let design = load_design();
        let cockpit = singleton(ComponentRole::Cockpit);
        let cockpit_solid = instance_solid(&design, cockpit);
        let fixed_to_pitch_frame = [
            located(
                ComponentRole::RollBearingPedestal,
                ComponentLocation::new().with_longitudinal_end(LongitudinalEnd::Front),
            ),
            located(
                ComponentRole::RollBearingPedestal,
                ComponentLocation::new().with_longitudinal_end(LongitudinalEnd::Rear),
            ),
            located(
                ComponentRole::RollGearboxMount,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Front)
                    .with_ordinal(1),
            ),
            located(
                ComponentRole::RollGearboxMount,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Front)
                    .with_ordinal(2),
            ),
            located(
                ComponentRole::RollGearboxMount,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Rear)
                    .with_ordinal(1),
            ),
            located(
                ComponentRole::RollGearboxMount,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Rear)
                    .with_ordinal(2),
            ),
            located(
                ComponentRole::MovingDriveMountArm,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Front)
                    .with_ordinal(1),
            ),
            located(
                ComponentRole::MovingDriveMountArm,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Front)
                    .with_ordinal(2),
            ),
            located(
                ComponentRole::MovingDriveMountArm,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Rear)
                    .with_ordinal(1),
            ),
            located(
                ComponentRole::MovingDriveMountArm,
                ComponentLocation::new()
                    .with_longitudinal_end(LongitudinalEnd::Rear)
                    .with_ordinal(2),
            ),
            located(
                ComponentRole::PitchEndUpperTie,
                ComponentLocation::new().with_longitudinal_end(LongitudinalEnd::Front),
            ),
            located(
                ComponentRole::PitchEndUpperTie,
                ComponentLocation::new().with_longitudinal_end(LongitudinalEnd::Rear),
            ),
        ];
        let mut evaluator = Evaluator::new(&design.graph);
        for pitch in [-20.0, 0.0, 20.0] {
            for roll in [-35.0, 0.0, 35.0] {
                let cockpit_pose = instance_pose(&design, cockpit, pitch, roll);
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
                        "cockpit intersects {support:?} by {volume} mm^3 at pitch={pitch}, roll={roll}"
                    );
                }
            }
        }
    }

    #[test]
    fn gearbox_plates_clear_their_gears_and_shafts() {
        let design = load_design();
        let mut evaluator = Evaluator::new(&design.graph);
        let right_front = ComponentLocation::new()
            .with_side(Side::Right)
            .with_longitudinal_end(LongitudinalEnd::Front);
        let roll_front = ComponentLocation::new().with_longitudinal_end(LongitudinalEnd::Front);
        let groups: &[(ComponentIdentity, &[ComponentIdentity])] = &[
            (
                located(ComponentRole::PitchContactCarriagePlate, right_front),
                &[
                    located(ComponentRole::PitchDrivePinion, right_front.with_ordinal(1)),
                    located(ComponentRole::PitchDrivePinion, right_front.with_ordinal(2)),
                    located(
                        ComponentRole::PitchGearboxSmallGear,
                        right_front.with_ordinal(1),
                    ),
                    located(
                        ComponentRole::PitchGearboxSmallGear,
                        right_front.with_ordinal(2),
                    ),
                    located(
                        ComponentRole::PitchGearboxSmallGear,
                        right_front.with_ordinal(3),
                    ),
                    located(
                        ComponentRole::PitchGearboxLargeGear,
                        right_front.with_ordinal(1),
                    ),
                    located(
                        ComponentRole::PitchGearboxSmallGear,
                        right_front.with_ordinal(4),
                    ),
                    located(
                        ComponentRole::PitchGearboxLargeGear,
                        right_front.with_ordinal(2),
                    ),
                    located(
                        ComponentRole::PitchGearboxSmallGear,
                        right_front.with_ordinal(5),
                    ),
                ],
            ),
            (
                located(ComponentRole::PitchContactInboardPlate, right_front),
                &[
                    located(ComponentRole::PitchDrivePinion, right_front.with_ordinal(1)),
                    located(ComponentRole::PitchDrivePinion, right_front.with_ordinal(2)),
                    located(ComponentRole::PitchRetentionPinion, right_front),
                    located(ComponentRole::PitchDriveShaft, right_front.with_ordinal(1)),
                    located(ComponentRole::PitchDriveShaft, right_front.with_ordinal(2)),
                    located(ComponentRole::PitchRetentionShaft, right_front),
                ],
            ),
            (
                located(ComponentRole::PitchGearboxFarPlate, right_front),
                &[
                    located(
                        ComponentRole::PitchGearboxLargeGear,
                        right_front.with_ordinal(1),
                    ),
                    located(
                        ComponentRole::PitchGearboxSmallGear,
                        right_front.with_ordinal(4),
                    ),
                    located(
                        ComponentRole::PitchGearboxLargeGear,
                        right_front.with_ordinal(2),
                    ),
                    located(
                        ComponentRole::PitchGearboxSmallGear,
                        right_front.with_ordinal(5),
                    ),
                    located(
                        ComponentRole::PitchGearboxShaft,
                        right_front.with_ordinal(1),
                    ),
                    located(
                        ComponentRole::PitchGearboxShaft,
                        right_front.with_ordinal(2),
                    ),
                    located(
                        ComponentRole::PitchGearboxShaft,
                        right_front.with_ordinal(3),
                    ),
                ],
            ),
            (
                located(ComponentRole::RollGearboxPlate, roll_front.with_ordinal(1)),
                &[
                    located(ComponentRole::RollInputPinion, roll_front),
                    located(
                        ComponentRole::RollGearboxLargeGear,
                        roll_front.with_ordinal(1),
                    ),
                    located(
                        ComponentRole::RollGearboxSmallGear,
                        roll_front.with_ordinal(1),
                    ),
                    located(
                        ComponentRole::RollGearboxLargeGear,
                        roll_front.with_ordinal(2),
                    ),
                    located(
                        ComponentRole::RollGearboxSmallGear,
                        roll_front.with_ordinal(2),
                    ),
                    located(ComponentRole::RollGearboxShaft, roll_front.with_ordinal(1)),
                    located(ComponentRole::RollGearboxShaft, roll_front.with_ordinal(2)),
                    located(ComponentRole::RollGearboxShaft, roll_front.with_ordinal(3)),
                ],
            ),
        ];
        for (plate, parts) in groups {
            for part in *parts {
                let volume = evaluator
                    .intersection_volume_transformed(
                        instance_solid(&design, *plate),
                        instance_pose(&design, *plate, 0.0, 0.0),
                        instance_solid(&design, *part),
                        instance_pose(&design, *part, 0.0, 0.0),
                    )
                    .expect("gearbox interference query succeeds");
                assert!(
                    volume <= 1.0e-7,
                    "{plate:?} intersects {part:?} by {volume} mm^3"
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

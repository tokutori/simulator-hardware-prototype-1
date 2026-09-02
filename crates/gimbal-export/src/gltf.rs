// SPDX-License-Identifier: MIT

use std::collections::BTreeMap;
use std::path::Path;

use gimbal_core::{Angle, Kinematics, PitchRollCommand};
use serde_json::{Value, json};

use crate::{ExportError, ExportPart};

#[derive(Clone, Copy, Debug)]
pub struct AnimationParameters {
    pub pitch_limit_degrees: f64,
    pub roll_limit_degrees: f64,
    pub duration_seconds: f32,
    /// Dense sampling preserves multi-turn gearbox motion through quaternion interpolation.
    pub sample_count: usize,
}

/// In-memory glTF JSON and its external binary buffer.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EncodedGltf {
    pub gltf: Vec<u8>,
    pub bin: Vec<u8>,
}

/// Encodes an animated assembly as glTF JSON and an external binary buffer.
///
/// `bin_name` is written to the glTF buffer URI and should match the filename
/// used when the returned `bin` bytes are persisted.
pub fn encode_animated_gltf(
    parts: &[ExportPart],
    kinematics: &Kinematics,
    parameters: AnimationParameters,
    bin_name: &str,
) -> Result<EncodedGltf, ExportError> {
    encode_animated_gltf_impl(parts, kinematics, parameters, bin_name)
}

/// Writes the bytes produced by [`encode_animated_gltf`] to a glTF/bin pair.
pub fn write_animated_gltf(
    parts: &[ExportPart],
    kinematics: &Kinematics,
    parameters: AnimationParameters,
    gltf_path: &Path,
    bin_path: &Path,
) -> Result<(), ExportError> {
    let bin_name = bin_path
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("gimbal-motion.bin");
    let encoded = encode_animated_gltf(parts, kinematics, parameters, bin_name)?;
    std::fs::write(bin_path, encoded.bin)?;
    std::fs::write(gltf_path, encoded.gltf)?;
    Ok(())
}

fn encode_animated_gltf_impl(
    parts: &[ExportPart],
    kinematics: &Kinematics,
    parameters: AnimationParameters,
    bin_name: &str,
) -> Result<EncodedGltf, ExportError> {
    let mut binary = BinaryBuilder::default();
    let mut buffer_views = Vec::<Value>::new();
    let mut accessors = Vec::<Value>::new();
    let mut materials = Vec::<Value>::new();
    let mut meshes = Vec::<Value>::new();
    let mut definition_meshes = BTreeMap::new();

    for part in parts {
        if definition_meshes.contains_key(&part.definition) {
            continue;
        }
        let positions = part
            .mesh
            .vertices
            .iter()
            .flat_map(|vertex| core_point_mm_to_gltf_m(*vertex))
            .collect::<Vec<_>>();
        let (minimum, maximum) = position_bounds(&positions);
        let position_view = push_view(
            &mut binary,
            &mut buffer_views,
            f32_bytes(&positions),
            Some(34962),
        );
        let position_accessor = accessors.len();
        accessors.push(json!({
            "bufferView": position_view,
            "componentType": 5126,
            "count": positions.len() / 3,
            "type": "VEC3",
            "min": minimum,
            "max": maximum
        }));
        let indices = part
            .mesh
            .triangles
            .iter()
            .flat_map(|triangle| triangle.iter().copied())
            .collect::<Vec<_>>();
        let index_view = push_view(
            &mut binary,
            &mut buffer_views,
            u32_bytes(&indices),
            Some(34963),
        );
        let index_accessor = accessors.len();
        accessors.push(json!({
            "bufferView": index_view,
            "componentType": 5125,
            "count": indices.len(),
            "type": "SCALAR"
        }));
        let material_index = materials.len();
        materials.push(json!({
            "name": format!("{}_material", part.name),
            "pbrMetallicRoughness": {
                "baseColorFactor": part.color_rgba,
                "metallicFactor": 0.05,
                "roughnessFactor": 0.62
            },
            "doubleSided": true
        }));
        let mesh_index = meshes.len();
        meshes.push(json!({
            "name": format!("definition_{}", part.definition.index()),
            "primitives": [{
                "attributes": { "POSITION": position_accessor },
                "indices": index_accessor,
                "material": material_index,
                "mode": 4
            }]
        }));
        definition_meshes.insert(part.definition, mesh_index);
    }

    let zero_pose = kinematics
        .pose(PitchRollCommand {
            pitch: Angle::degrees(0.0).map_err(|_| ExportError::InvalidAnimationPose)?,
            roll: Angle::degrees(0.0).map_err(|_| ExportError::InvalidAnimationPose)?,
        })
        .map_err(|_| ExportError::InvalidAnimationPose)?;
    let mut nodes = Vec::<Value>::new();
    for part in parts {
        let frame = zero_pose
            .frame(part.frame)
            .ok_or(ExportError::InvalidAnimationPose)?;
        let transform = frame.compose(part.local_pose);
        nodes.push(json!({
            "name": part.name,
            "mesh": definition_meshes[&part.definition],
            "translation": core_translation_mm_to_gltf_m(transform.translation),
            "rotation": core_rotation_to_gltf(transform.rotation),
            "extras": semantic_extras(&part.semantics)
        }));
    }

    if parameters.sample_count < 5 {
        return Err(ExportError::InvalidAnimationPose);
    }
    let samples = motion_samples(parameters);
    let times = samples
        .iter()
        .map(|sample| sample.0 * parameters.duration_seconds)
        .collect::<Vec<_>>();
    let time_view = push_view(&mut binary, &mut buffer_views, f32_bytes(&times), None);
    let time_accessor = accessors.len();
    accessors.push(json!({
        "bufferView": time_view,
        "componentType": 5126,
        "count": times.len(),
        "type": "SCALAR",
        "min": [0.0],
        "max": [parameters.duration_seconds]
    }));
    let mut samplers = Vec::<Value>::new();
    let mut channels = Vec::<Value>::new();
    for (node_index, part) in parts.iter().enumerate() {
        let mut translations = Vec::with_capacity(times.len() * 3);
        let mut rotations = Vec::with_capacity(times.len() * 4);
        for &(_, pitch, roll) in &samples {
            let pose = kinematics
                .pose(PitchRollCommand {
                    pitch: Angle::degrees(pitch).map_err(|_| ExportError::InvalidAnimationPose)?,
                    roll: Angle::degrees(roll).map_err(|_| ExportError::InvalidAnimationPose)?,
                })
                .map_err(|_| ExportError::InvalidAnimationPose)?;
            let transform = pose
                .frame(part.frame)
                .ok_or(ExportError::InvalidAnimationPose)?
                .compose(part.local_pose);
            translations.extend(core_translation_mm_to_gltf_m(transform.translation));
            rotations.extend(core_rotation_to_gltf(transform.rotation));
        }
        add_animation_channel(
            &mut binary,
            &mut buffer_views,
            &mut accessors,
            &mut samplers,
            &mut channels,
            time_accessor,
            node_index,
            "translation",
            "VEC3",
            &translations,
        );
        add_animation_channel(
            &mut binary,
            &mut buffer_views,
            &mut accessors,
            &mut samplers,
            &mut channels,
            time_accessor,
            node_index,
            "rotation",
            "VEC4",
            &rotations,
        );
    }

    let document = json!({
        "asset": { "version": "2.0", "generator": "gimbal-export 0.1.0" },
        "scene": 0,
        "scenes": [{ "name": "Pitch-roll cockpit simulator", "nodes": (0..parts.len()).collect::<Vec<_>>() }],
        "nodes": nodes,
        "meshes": meshes,
        "materials": materials,
        "buffers": [{ "uri": bin_name, "byteLength": binary.bytes.len() }],
        "bufferViews": buffer_views,
        "accessors": accessors,
        "animations": [{
            "name": "pitch_and_roll_motion",
            "samplers": samplers,
            "channels": channels
        }]
    });
    Ok(EncodedGltf {
        gltf: serde_json::to_vec_pretty(&document)?,
        bin: binary.bytes,
    })
}

fn motion_samples(parameters: AnimationParameters) -> Vec<(f32, f64, f64)> {
    (0..parameters.sample_count)
        .map(|index| {
            let normalized = index as f64 / (parameters.sample_count - 1) as f64;
            let phase = normalized * 4.0;
            let amplitude = if phase <= 1.0 {
                phase
            } else if phase <= 2.0 {
                2.0 - phase
            } else if phase <= 3.0 {
                -(phase - 2.0)
            } else {
                -(4.0 - phase)
            };
            (
                normalized as f32,
                amplitude * parameters.pitch_limit_degrees,
                -amplitude * parameters.roll_limit_degrees,
            )
        })
        .collect()
}

#[allow(clippy::too_many_arguments)]
fn add_animation_channel(
    binary: &mut BinaryBuilder,
    views: &mut Vec<Value>,
    accessors: &mut Vec<Value>,
    samplers: &mut Vec<Value>,
    channels: &mut Vec<Value>,
    time_accessor: usize,
    node_index: usize,
    path: &str,
    accessor_type: &str,
    values: &[f32],
) {
    let view = push_view(binary, views, f32_bytes(values), None);
    let accessor = accessors.len();
    let width = if accessor_type == "VEC4" { 4 } else { 3 };
    accessors.push(json!({
        "bufferView": view,
        "componentType": 5126,
        "count": values.len() / width,
        "type": accessor_type
    }));
    let sampler = samplers.len();
    samplers.push(json!({
        "input": time_accessor,
        "output": accessor,
        "interpolation": "LINEAR"
    }));
    channels.push(json!({
        "sampler": sampler,
        "target": { "node": node_index, "path": path }
    }));
}

#[derive(Default)]
struct BinaryBuilder {
    bytes: Vec<u8>,
}

fn push_view(
    binary: &mut BinaryBuilder,
    views: &mut Vec<Value>,
    bytes: Vec<u8>,
    target: Option<u32>,
) -> usize {
    while !binary.bytes.len().is_multiple_of(4) {
        binary.bytes.push(0);
    }
    let offset = binary.bytes.len();
    let length = bytes.len();
    binary.bytes.extend(bytes);
    let mut view = json!({ "buffer": 0, "byteOffset": offset, "byteLength": length });
    if let Some(target) = target {
        view["target"] = json!(target);
    }
    let index = views.len();
    views.push(view);
    index
}

fn f32_bytes(values: &[f32]) -> Vec<u8> {
    values
        .iter()
        .flat_map(|value| value.to_le_bytes())
        .collect()
}

fn u32_bytes(values: &[u32]) -> Vec<u8> {
    values
        .iter()
        .flat_map(|value| value.to_le_bytes())
        .collect()
}

fn position_bounds(positions: &[f32]) -> ([f32; 3], [f32; 3]) {
    let mut minimum = [f32::INFINITY; 3];
    let mut maximum = [f32::NEG_INFINITY; 3];
    for vertex in positions.as_chunks::<3>().0 {
        for axis in 0..3 {
            minimum[axis] = minimum[axis].min(vertex[axis]);
            maximum[axis] = maximum[axis].max(vertex[axis]);
        }
    }
    (minimum, maximum)
}

fn semantic_extras(semantics: &crate::ExportSemantics) -> Value {
    json!({
        "component_role": semantics.role,
        "side": semantics.side,
        "longitudinal_end": semantics.longitudinal_end,
        "vertical_end": semantics.vertical_end,
        "ordinal": semantics.ordinal,
    })
}

/// Convert the core's right-handed Z-up millimetre coordinates to glTF's
/// right-handed Y-up metre coordinates.
fn core_point_mm_to_gltf_m(point: [f64; 3]) -> [f32; 3] {
    [
        (point[0] * 0.001) as f32,
        (point[2] * 0.001) as f32,
        (-point[1] * 0.001) as f32,
    ]
}

fn core_translation_mm_to_gltf_m(translation: [f64; 3]) -> [f32; 3] {
    core_point_mm_to_gltf_m(translation)
}

/// A basis change by -90 degrees around X maps quaternion vector parts in the
/// same way as points; the scalar part is unchanged.
fn core_rotation_to_gltf(rotation: [f64; 4]) -> [f32; 4] {
    [
        rotation[0] as f32,
        rotation[2] as f32,
        -rotation[1] as f32,
        rotation[3] as f32,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use gimbal_core::FrameGraph;

    #[test]
    fn converts_z_up_millimetres_to_gltf_y_up_metres() {
        assert_eq!(
            core_point_mm_to_gltf_m([1000.0, 2000.0, 3000.0]),
            [1.0, 3.0, -2.0]
        );
    }

    #[test]
    fn converts_z_axis_rotation_to_gltf_y_axis_rotation() {
        let sine = core::f64::consts::FRAC_1_SQRT_2;
        let converted = core_rotation_to_gltf([0.0, 0.0, sine, sine]);
        assert!((converted[1] - sine as f32).abs() < 1.0e-6);
        assert_eq!(converted[0], 0.0);
        assert_eq!(converted[2], 0.0);
    }

    #[test]
    fn dense_motion_samples_preserve_high_ratio_rotation_direction() {
        let samples = motion_samples(AnimationParameters {
            pitch_limit_degrees: 20.0,
            roll_limit_degrees: 35.0,
            duration_seconds: 6.0,
            sample_count: 73,
        });
        assert_eq!(samples.len(), 73);
        assert_eq!(samples[0], (0.0, 0.0, -0.0));
        assert!((samples[18].1 - 20.0).abs() < 1.0e-10);
        assert!((samples[18].2 + 35.0).abs() < 1.0e-10);
        let maximum_input_step = samples
            .windows(2)
            .map(|window| (window[1].2 - window[0].2).abs() * 18.0)
            .fold(0.0_f64, f64::max);
        assert!(maximum_input_step < 180.0);
    }

    #[test]
    fn component_semantics_are_preserved_as_gltf_extras() {
        let extras = semantic_extras(&crate::ExportSemantics {
            role: "PitchDrivePinion".to_string(),
            side: Some("right".to_string()),
            longitudinal_end: Some("front".to_string()),
            vertical_end: None,
            ordinal: Some(2),
        });
        assert_eq!(extras["component_role"], "PitchDrivePinion");
        assert_eq!(extras["side"], "right");
        assert_eq!(extras["longitudinal_end"], "front");
        assert!(extras["vertical_end"].is_null());
        assert_eq!(extras["ordinal"], 2);
    }

    #[test]
    fn animated_gltf_encoder_matches_filesystem_wrapper() {
        let kinematics = Kinematics::new(
            FrameGraph::new(),
            Angle::degrees(20.0).unwrap(),
            Angle::degrees(35.0).unwrap(),
        );
        let parameters = AnimationParameters {
            pitch_limit_degrees: 20.0,
            roll_limit_degrees: 35.0,
            duration_seconds: 1.0,
            sample_count: 5,
        };
        let encoded = encode_animated_gltf(&[], &kinematics, parameters, "fixture.bin").unwrap();
        let directory =
            std::env::temp_dir().join(format!("gimbal-export-{}-gltf-encoder", std::process::id()));
        std::fs::create_dir_all(&directory).unwrap();
        let gltf_path = directory.join("fixture.gltf");
        let bin_path = directory.join("fixture.bin");

        write_animated_gltf(&[], &kinematics, parameters, &gltf_path, &bin_path).unwrap();

        assert_eq!(std::fs::read(&gltf_path).unwrap(), encoded.gltf);
        assert_eq!(std::fs::read(&bin_path).unwrap(), encoded.bin);
        std::fs::remove_dir_all(directory).unwrap();
    }
}

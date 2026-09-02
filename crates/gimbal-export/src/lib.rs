// SPDX-License-Identifier: MIT

mod dxf;
mod gltf;
mod mesh_files;
mod three_mf;

use gimbal_core::{ComponentDefinitionId, FrameId, Manufacturing, RigidTransform, TriangleMesh};
use thiserror::Error;

pub use dxf::{write_dxf_profile, write_dxf_sheet_profile};
pub use gltf::{AnimationParameters, write_animated_gltf};
pub use mesh_files::{EncodedObj, encode_binary_stl, encode_obj, write_binary_stl, write_obj};
pub use three_mf::{encode_3mf, encode_mesh_3mf, write_3mf, write_mesh_3mf};

#[derive(Clone, Debug)]
pub struct ExportPart {
    pub name: String,
    pub definition: ComponentDefinitionId,
    pub mesh: TriangleMesh,
    pub manufacturing: Manufacturing,
    pub frame: FrameId,
    pub local_pose: RigidTransform,
    pub static_pose: RigidTransform,
    pub color_rgba: [f32; 4],
    pub semantics: ExportSemantics,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ExportSemantics {
    pub role: String,
    pub side: Option<String>,
    pub longitudinal_end: Option<String>,
    pub vertical_end: Option<String>,
    pub ordinal: Option<u16>,
}

#[derive(Debug, Error)]
pub enum ExportError {
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error("3MF writer failed: {0}")]
    ThreeMf(String),
    #[error("animation requested an invalid kinematic pose")]
    InvalidAnimationPose,
    #[error("mesh contains more vertices than the selected file format supports")]
    MeshTooLarge,
    #[error("DXF writer failed: {0}")]
    Dxf(String),
    #[error("invalid DXF output: {0}")]
    InvalidDxf(&'static str),
    #[error("profile must contain at least three points")]
    InvalidProfile,
}

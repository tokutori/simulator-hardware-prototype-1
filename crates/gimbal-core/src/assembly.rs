// SPDX-License-Identifier: MIT

use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;

use crate::{Length, Manufacturing, RegionId, SolidId};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ComponentDefinitionId(u32);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct FrameId(u32);

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Body {
    Solid(SolidId),
    Sheet {
        profile: RegionId,
        thickness: Length,
        assembly_solid: SolidId,
    },
}

impl Body {
    pub const fn assembly_solid(self) -> SolidId {
        match self {
            Self::Solid(solid)
            | Self::Sheet {
                assembly_solid: solid,
                ..
            } => solid,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ComponentDefinition {
    pub name: String,
    pub body: Body,
    pub manufacturing: Manufacturing,
    pub color_rgba: [f32; 4],
}

#[derive(Clone, Debug, PartialEq)]
pub struct ComponentInstance {
    pub name: String,
    pub definition: ComponentDefinitionId,
    pub frame: FrameId,
    pub local_pose: RigidTransform,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis3 {
    X,
    Y,
    Z,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoordinateExpr {
    pub pitch_scale: f64,
    pub roll_scale: f64,
    pub offset_radians: f64,
}

impl CoordinateExpr {
    pub const fn fixed() -> Self {
        Self {
            pitch_scale: 0.0,
            roll_scale: 0.0,
            offset_radians: 0.0,
        }
    }

    pub const fn pitch(scale: f64) -> Self {
        Self {
            pitch_scale: scale,
            roll_scale: 0.0,
            offset_radians: 0.0,
        }
    }

    pub const fn roll(scale: f64) -> Self {
        Self {
            pitch_scale: 0.0,
            roll_scale: scale,
            offset_radians: 0.0,
        }
    }

    fn evaluate(self, pitch: f64, roll: f64) -> f64 {
        self.pitch_scale * pitch + self.roll_scale * roll + self.offset_radians
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Joint {
    Fixed,
    Revolute {
        axis: Axis3,
        coordinate: CoordinateExpr,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RigidTransform {
    pub translation: [f64; 3],
    /// Quaternion in glTF order: x, y, z, w.
    pub rotation: [f64; 4],
}

impl RigidTransform {
    pub const IDENTITY: Self = Self {
        translation: [0.0; 3],
        rotation: [0.0, 0.0, 0.0, 1.0],
    };

    pub const fn translated(x: f64, y: f64, z: f64) -> Self {
        Self {
            translation: [x, y, z],
            rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }

    pub fn rotated(axis: Axis3, radians: f64) -> Self {
        Self {
            translation: [0.0; 3],
            rotation: axis_quaternion(axis, radians),
        }
    }

    pub fn compose(self, child: Self) -> Self {
        let moved = rotate_vector(self.rotation, child.translation);
        Self {
            translation: [
                self.translation[0] + moved[0],
                self.translation[1] + moved[1],
                self.translation[2] + moved[2],
            ],
            rotation: quaternion_multiply(self.rotation, child.rotation),
        }
    }

    pub fn transform_point(self, point: [f64; 3]) -> [f64; 3] {
        let rotated = rotate_vector(self.rotation, point);
        [
            rotated[0] + self.translation[0],
            rotated[1] + self.translation[1],
            rotated[2] + self.translation[2],
        ]
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame {
    parent: Option<FrameId>,
    nominal: RigidTransform,
    joint: Joint,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FrameGraph {
    frames: Vec<Frame>,
}

impl FrameGraph {
    pub fn new() -> Self {
        Self {
            frames: vec![Frame {
                parent: None,
                nominal: RigidTransform::IDENTITY,
                joint: Joint::Fixed,
            }],
        }
    }

    pub const fn world(&self) -> FrameId {
        FrameId(0)
    }

    pub fn add_frame(&mut self, parent: FrameId, nominal: RigidTransform, joint: Joint) -> FrameId {
        assert!(
            parent.index() < self.frames.len(),
            "parent frame must already exist"
        );
        let id = FrameId(self.frames.len() as u32);
        self.frames.push(Frame {
            parent: Some(parent),
            nominal,
            joint,
        });
        id
    }

    pub fn world_poses(&self, pitch_radians: f64, roll_radians: f64) -> Vec<RigidTransform> {
        let mut poses = vec![RigidTransform::IDENTITY; self.frames.len()];
        for (index, frame) in self.frames.iter().enumerate().skip(1) {
            let parent = poses[frame.parent.expect("non-world frame has parent").index()];
            let joint = match frame.joint {
                Joint::Fixed => RigidTransform::IDENTITY,
                Joint::Revolute { axis, coordinate } => {
                    RigidTransform::rotated(axis, coordinate.evaluate(pitch_radians, roll_radians))
                }
            };
            poses[index] = parent.compose(frame.nominal).compose(joint);
        }
        poses
    }

    pub fn len(&self) -> usize {
        self.frames.len()
    }

    /// A frame graph always contains its world frame.
    pub const fn is_empty(&self) -> bool {
        false
    }
}

impl Default for FrameGraph {
    fn default() -> Self {
        Self::new()
    }
}

impl ComponentDefinitionId {
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

impl FrameId {
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Assembly {
    definitions: Vec<ComponentDefinition>,
    instances: Vec<ComponentInstance>,
}

impl Assembly {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_definition(&mut self, definition: ComponentDefinition) -> ComponentDefinitionId {
        let id = ComponentDefinitionId(self.definitions.len() as u32);
        self.definitions.push(definition);
        id
    }

    pub fn add_instance(&mut self, instance: ComponentInstance) {
        assert!(
            instance.definition.index() < self.definitions.len(),
            "component definition must already exist"
        );
        self.instances.push(instance);
    }

    pub fn definitions(&self) -> &[ComponentDefinition] {
        &self.definitions
    }

    pub fn instances(&self) -> &[ComponentInstance] {
        &self.instances
    }

    pub fn definition(&self, id: ComponentDefinitionId) -> Option<&ComponentDefinition> {
        self.definitions.get(id.index())
    }
}

fn axis_quaternion(axis: Axis3, radians: f64) -> [f64; 4] {
    let half = radians * 0.5;
    let sine = libm::sin(half);
    let cosine = libm::cos(half);
    match axis {
        Axis3::X => [sine, 0.0, 0.0, cosine],
        Axis3::Y => [0.0, sine, 0.0, cosine],
        Axis3::Z => [0.0, 0.0, sine, cosine],
    }
}

fn quaternion_multiply(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]
}

fn rotate_vector(quaternion: [f64; 4], point: [f64; 3]) -> [f64; 3] {
    let q = [quaternion[0], quaternion[1], quaternion[2]];
    let uv = cross(q, point);
    let uuv = cross(q, uv);
    [
        point[0] + 2.0 * (quaternion[3] * uv[0] + uuv[0]),
        point[1] + 2.0 * (quaternion[3] * uv[1] + uuv[1]),
        point[2] + 2.0 * (quaternion[3] * uv[2] + uuv[2]),
    ]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nested_roll_axis_follows_pitch_frame() {
        let mut frames = FrameGraph::new();
        let pitch = frames.add_frame(
            frames.world(),
            RigidTransform::IDENTITY,
            Joint::Revolute {
                axis: Axis3::Y,
                coordinate: CoordinateExpr::pitch(1.0),
            },
        );
        let roll = frames.add_frame(
            pitch,
            RigidTransform::IDENTITY,
            Joint::Revolute {
                axis: Axis3::X,
                coordinate: CoordinateExpr::roll(1.0),
            },
        );
        let poses = frames.world_poses(core::f64::consts::FRAC_PI_2, 0.0);
        let point = poses[roll.index()].transform_point([1.0, 0.0, 0.0]);
        assert!(point[0].abs() < 1.0e-10);
        assert!((point[2] + 1.0).abs() < 1.0e-10);
    }
}

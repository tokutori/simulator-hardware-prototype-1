// SPDX-License-Identifier: MIT

use alloc::vec::Vec;

use crate::{Angle, FrameGraph, FrameId, RigidTransform};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PitchRollCommand {
    pub pitch: Angle,
    pub roll: Angle,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AssemblyPose {
    frame_poses: Vec<RigidTransform>,
}

impl AssemblyPose {
    pub fn frame(&self, id: FrameId) -> Option<RigidTransform> {
        self.frame_poses.get(id.index()).copied()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KinematicError {
    PitchLimitExceeded,
    RollLimitExceeded,
}

#[derive(Clone, Debug)]
pub struct Kinematics {
    frames: FrameGraph,
    pitch_limit: Angle,
    roll_limit: Angle,
}

impl Kinematics {
    pub const fn new(frames: FrameGraph, pitch_limit: Angle, roll_limit: Angle) -> Self {
        Self {
            frames,
            pitch_limit,
            roll_limit,
        }
    }

    pub fn pose(&self, command: PitchRollCommand) -> Result<AssemblyPose, KinematicError> {
        if libm::fabs(command.pitch.as_radians()) > self.pitch_limit.as_radians() {
            return Err(KinematicError::PitchLimitExceeded);
        }
        if libm::fabs(command.roll.as_radians()) > self.roll_limit.as_radians() {
            return Err(KinematicError::RollLimitExceeded);
        }
        Ok(AssemblyPose {
            frame_poses: self
                .frames
                .world_poses(command.pitch.as_radians(), command.roll.as_radians()),
        })
    }

    pub const fn frames(&self) -> &FrameGraph {
        &self.frames
    }

    pub const fn pitch_limit(&self) -> Angle {
        self.pitch_limit
    }

    pub const fn roll_limit(&self) -> Angle {
        self.roll_limit
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Axis3, CoordinateExpr, Joint};

    #[test]
    fn yaw_is_not_representable_and_limits_are_checked() {
        let mut frames = FrameGraph::new();
        frames.add_frame(
            frames.world(),
            RigidTransform::IDENTITY,
            Joint::Revolute {
                axis: Axis3::Y,
                coordinate: CoordinateExpr::pitch(1.0),
            },
        );
        let model = Kinematics::new(
            frames,
            Angle::degrees(20.0).unwrap(),
            Angle::degrees(35.0).unwrap(),
        );
        assert!(
            model
                .pose(PitchRollCommand {
                    pitch: Angle::degrees(20.1).unwrap(),
                    roll: Angle::degrees(0.0).unwrap(),
                })
                .is_err()
        );
    }
}

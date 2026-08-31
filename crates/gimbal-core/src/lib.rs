// SPDX-License-Identifier: MIT
#![no_std]

extern crate alloc;

mod assembly;
mod gear;
mod geometry;
mod kinematics;
mod manufacturing;
mod prototype;
mod units;

pub use assembly::{
    Assembly, Axis3, Body, ComponentDefinition, ComponentDefinitionId, ComponentInstance,
    CoordinateExpr, FrameGraph, FrameId, Joint, RigidTransform,
};
pub use gear::{
    ExternalGearPair, GearError, GearProfile, GearSector, InternalGear, InternalGearPair, Point2,
    SpurGear,
};
pub use geometry::{
    BooleanOperation, FeatureBuilder, FeatureError, FeatureGraph, Primitive3, RegionId, RegionNode,
    Rotation3, SolidId, SolidNode, Translation3, TriangleMesh,
};
pub use kinematics::{AssemblyPose, KinematicError, Kinematics, PitchRollCommand};
pub use manufacturing::{FdmMaterial, Manufacturing, RingSide, UnitPosition};
pub use prototype::{
    CockpitParameters, ContactUnitParameters, FrameParameters, MotionParameters,
    PitchGearboxParameters, PitchSectorParameters, PrototypeDesign, PrototypeError,
    PrototypeParameters, RollAxisParameters, build_prototype,
};
pub use units::{Angle, Length, UnitError};

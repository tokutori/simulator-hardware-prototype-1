// SPDX-License-Identifier: MIT
#![no_std]

extern crate alloc;

mod assembly;
mod constraint;
mod datum;
mod gear;
mod geometry;
mod kinematics;
mod manufacturing;
mod relation;
mod units;

pub use assembly::{
    Assembly, AssemblyError, Axis3, Body, ComponentDefinition, ComponentDefinitionId,
    ComponentIdentity, ComponentIdentityCollision, ComponentInstance, ComponentInstanceId,
    ComponentInstancePair, CoordinateExpr, FrameGraph, FrameId, Joint, RigidTransform,
};
pub use constraint::{
    AngularConstraint, AngularConstraintId, AngularCoordinate, AngularCoordinateId,
    ConstraintCycle, ConstraintDirection, ConstraintError, ConstraintGraph, ConstraintStep,
};
pub use datum::{
    AxisDatum, CylinderDatum, DatumError, DatumGeometry, DatumId, DatumKind, DatumSet, DatumType,
    NamedDatum, PlaneDatum, Point3, PointDatum, UnitVector3,
};
pub use gear::{
    ExternalGearPair, GearError, GearProfile, GearSector, InternalGear, InternalGearPair, SpurGear,
};
pub use geometry::{
    BooleanOperation, FeatureBuilder, FeatureError, FeatureGraph, Point2, Primitive3, RegionId,
    RegionNode, Rotation3, SolidId, SolidNode, Translation3, TriangleMesh,
};
pub use kinematics::{AssemblyPose, KinematicError, Kinematics, PitchRollCommand};
pub use manufacturing::{
    ComponentLocation, ComponentRole, FdmMaterial, LongitudinalEnd, Manufacturing, Side,
    VerticalEnd,
};
pub use relation::{
    AssemblyRelation, AssemblyRelationId, BoltHardware, CylindricalFit, DatumEndpoint,
    EngineeringTolerance, FastenedJoint, FastenerHardware, GearMesh, GearMeshKind, MetricThread,
    NumericalTolerance, NutHardware, PlaneClearance, SurfaceContact, WasherHardware,
};
pub use units::{
    Angle, Length, NonNegativeAngle, NonNegativeLength, PositiveAngle, PositiveArea,
    PositiveLength, PositiveVolume, UnitError,
};

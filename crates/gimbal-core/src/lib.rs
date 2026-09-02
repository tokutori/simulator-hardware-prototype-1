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
mod prototype;
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
    ExternalGearPair, GearError, GearProfile, GearSector, InternalGear, InternalGearPair, Point2,
    SpurGear,
};
pub use geometry::{
    BooleanOperation, FeatureBuilder, FeatureError, FeatureGraph, Primitive3, RegionId, RegionNode,
    Rotation3, SolidId, SolidNode, Translation3, TriangleMesh,
};
pub use kinematics::{AssemblyPose, KinematicError, Kinematics, PitchRollCommand};
pub use manufacturing::{
    ComponentLocation, ComponentRole, FdmMaterial, LongitudinalEnd, Manufacturing, Side,
    VerticalEnd,
};
pub use prototype::{
    CockpitParameters, ContactUnitParameters, FrameParameters, MotionParameters,
    PitchGearboxParameters, PitchSectorParameters, PrototypeDesign, PrototypeError,
    PrototypeParameters, RollAxisParameters, build_prototype,
};
pub use relation::{
    AssemblyRelation, AssemblyRelationId, CylindricalFit, DatumEndpoint, EngineeringTolerance,
    GearMesh, GearMeshKind, NumericalTolerance, SurfaceContact,
};
pub use units::{
    Angle, Length, NonNegativeAngle, NonNegativeLength, PositiveAngle, PositiveArea,
    PositiveLength, PositiveVolume, UnitError,
};

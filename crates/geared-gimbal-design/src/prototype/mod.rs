// SPDX-License-Identifier: MIT

use alloc::format;
use alloc::string::ToString;
use alloc::vec;
use alloc::vec::Vec;
use core::f64::consts::{FRAC_PI_2, PI};

use crate::{
    Angle, Assembly, AssemblyRelation, Axis3, AxisDatum, Body, BoltHardware, BooleanOperation,
    ComponentDefinition, ComponentDefinitionId, ComponentIdentity, ComponentInstance,
    ComponentInstanceId, ComponentLocation, ComponentRole, CoordinateExpr, CylinderDatum,
    CylindricalFit, DatumEndpoint, DatumId, DatumSet, EngineeringTolerance, ExternalGearPair,
    FastenedJoint, FastenerHardware, FeatureBuilder, FeatureError, FeatureGraph, FrameGraph,
    FrameId, InternalGearPair, Joint, Kinematics, Length, LongitudinalEnd, Manufacturing,
    MetricThread, NonNegativeAngle, NonNegativeLength, NutHardware, PlaneClearance, PlaneDatum,
    Point2, Point3, PositiveArea, PositiveLength, Primitive3, RigidTransform, Rotation3, Side,
    SolidId, SpurGear, SurfaceContact, Translation3, UnitVector3, VerticalEnd, WasherHardware,
};

mod component_geometry;
mod definitions;
mod fixed_frame;
mod parameters;
mod pitch_geometry;
mod pitch_unit;
mod roll;
mod validation;

use component_geometry::*;
use definitions::*;
use fixed_frame::*;
pub use parameters::*;
use pitch_geometry::*;
use pitch_unit::*;
use roll::*;
use validation::*;

#[derive(Clone, Debug)]
pub struct PrototypeDesign {
    pub graph: FeatureGraph,
    pub assembly: Assembly,
    pub kinematics: Kinematics,
    pub pitch_drive_pair: ExternalGearPair,
    pub pitch_encoder_pair: InternalGearPair,
    pub pitch_gearbox_pair: ExternalGearPair,
    pub roll_pair: ExternalGearPair,
}

pub fn build_prototype(
    parameters: &PrototypeParameters,
) -> Result<PrototypeDesign, PrototypeError> {
    validate(parameters)?;
    let pitch_drive_pair = ExternalGearPair::new(
        parameters.contact_unit.drive_pinion.clone(),
        parameters.pitch_sector.sector.external_reference().clone(),
    )
    .map_err(|_| PrototypeError::IncompatibleGearPair)?;
    let pitch_encoder_pair = InternalGearPair::new(
        parameters.contact_unit.encoder_pinion.clone(),
        parameters.pitch_sector.sector.internal_reference().clone(),
    )
    .map_err(|_| PrototypeError::IncompatibleGearPair)?;
    let pitch_gearbox_pair = ExternalGearPair::new(
        parameters.pitch_gearbox.small_gear.clone(),
        parameters.pitch_gearbox.large_gear.clone(),
    )
    .map_err(|_| PrototypeError::IncompatibleGearPair)?;
    let roll_pair = ExternalGearPair::new(
        parameters.roll_axis.pinion.clone(),
        parameters.roll_axis.driven_gear.clone(),
    )
    .map_err(|_| PrototypeError::IncompatibleGearPair)?;

    let mut builder = FeatureBuilder::new();
    let mut assembly = Assembly::new();
    let mut frames = FrameGraph::new();
    let world = frames.world();
    let pitch_frame = frames.add_frame(
        world,
        RigidTransform::IDENTITY,
        Joint::Revolute {
            axis: Axis3::Y,
            coordinate: CoordinateExpr::pitch(1.0),
        },
    );
    let roll_frame = frames.add_frame(
        pitch_frame,
        RigidTransform::IDENTITY,
        Joint::Revolute {
            axis: Axis3::X,
            coordinate: CoordinateExpr::roll(1.0),
        },
    );

    let definitions = build_definitions(&mut builder, &mut assembly, parameters)?;
    build_pitch_carrier(&mut assembly, &definitions, parameters, world);
    build_crossmembers(&mut assembly, &definitions, parameters, world);
    build_fixed_frame_contacts(&mut assembly, &definitions, parameters)?;
    build_contact_units(
        &mut assembly,
        &definitions,
        &mut frames,
        parameters,
        pitch_frame,
        pitch_drive_pair.ratio(),
        pitch_encoder_pair.ratio(),
        pitch_gearbox_pair.ratio(),
    )?;
    build_roll_assembly(
        &mut assembly,
        &definitions,
        &mut frames,
        parameters,
        pitch_frame,
        roll_frame,
        roll_pair.ratio(),
        pitch_gearbox_pair.ratio(),
    );
    build_moving_carrier_contacts(&mut assembly, &definitions, parameters)?;
    build_roll_bearing_fits(&mut assembly, &definitions, parameters)?;

    let kinematics = Kinematics::new(
        frames,
        parameters.motion.pitch_limit,
        parameters.motion.roll_limit,
    );
    Ok(PrototypeDesign {
        graph: builder.finish(),
        assembly,
        kinematics,
        pitch_drive_pair,
        pitch_encoder_pair,
        pitch_gearbox_pair,
        roll_pair,
    })
}

impl From<FeatureError> for PrototypeError {
    fn from(value: FeatureError) -> Self {
        Self::Feature(value)
    }
}

#[cfg(test)]
mod tests;

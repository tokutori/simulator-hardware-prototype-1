// SPDX-License-Identifier: MIT

use core::marker::PhantomData;

use crate::assembly::{ComponentDefinitionId, ComponentInstanceId};
use crate::datum::{AxisDatum, CylinderDatum, DatumId, DatumKind, DatumType, PlaneDatum};
use crate::{
    Angle, NonNegativeAngle, NonNegativeLength, PositiveAngle, PositiveArea, PositiveLength,
    PositiveVolume,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct AssemblyRelationId(u32);

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct DatumEndpoint<T> {
    pub instance: ComponentInstanceId,
    pub datum: DatumId<T>,
    marker: PhantomData<fn() -> T>,
}

impl<T> Clone for DatumEndpoint<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T> Copy for DatumEndpoint<T> {}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EngineeringTolerance {
    pub linear: NonNegativeLength,
    pub angular: NonNegativeAngle,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NumericalTolerance {
    pub linear_epsilon: PositiveLength,
    pub area_epsilon: PositiveArea,
    pub volume_epsilon: PositiveVolume,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceContact {
    pub first: DatumEndpoint<PlaneDatum>,
    pub second: DatumEndpoint<PlaneDatum>,
    pub minimum_contact_area: PositiveArea,
    pub tolerance: EngineeringTolerance,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlaneClearance {
    pub first: DatumEndpoint<PlaneDatum>,
    pub second: DatumEndpoint<PlaneDatum>,
    pub target_separation: NonNegativeLength,
    pub minimum_overlap_area: PositiveArea,
    pub tolerance: EngineeringTolerance,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CylindricalFit {
    pub shaft: DatumEndpoint<CylinderDatum>,
    pub bore: DatumEndpoint<CylinderDatum>,
    pub target_radial_clearance: NonNegativeLength,
    pub tolerance: EngineeringTolerance,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MetricThread {
    M3,
}

impl MetricThread {
    pub const fn nominal_diameter_mm(self) -> f64 {
        match self {
            Self::M3 => 3.0,
        }
    }

    pub const fn nominal_pitch_mm(self) -> f64 {
        match self {
            Self::M3 => 0.5,
        }
    }

    pub const fn minimum_full_thread_engagement_mm(self) -> f64 {
        self.nominal_diameter_mm() * 0.8
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoltHardware {
    pub instance: ComponentInstanceId,
    pub axis: DatumId<AxisDatum>,
    pub under_head_face: DatumId<PlaneDatum>,
    pub shank_tip_face: DatumId<PlaneDatum>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NutHardware {
    pub instance: ComponentInstanceId,
    pub axis: DatumId<AxisDatum>,
    pub bearing_face: DatumId<PlaneDatum>,
    pub outer_face: DatumId<PlaneDatum>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WasherHardware {
    pub instance: ComponentInstanceId,
    pub axis: DatumId<AxisDatum>,
    pub member_face: DatumId<PlaneDatum>,
    pub hardware_face: DatumId<PlaneDatum>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FastenerHardware {
    pub bolt: BoltHardware,
    pub nut: NutHardware,
    pub first_washer: Option<WasherHardware>,
    pub second_washer: Option<WasherHardware>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FastenedJoint {
    pub first_hole: DatumEndpoint<CylinderDatum>,
    pub second_hole: DatumEndpoint<CylinderDatum>,
    pub head_seat: DatumEndpoint<PlaneDatum>,
    pub nut_seat: DatumEndpoint<PlaneDatum>,
    pub hardware: FastenerHardware,
    pub thread: MetricThread,
    pub target_hole_radial_clearance: NonNegativeLength,
    pub grip_length: PositiveLength,
    pub tolerance: EngineeringTolerance,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GearMeshKind {
    External,
    Internal,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GearMesh {
    pub first_axis: DatumEndpoint<AxisDatum>,
    pub second_axis: DatumEndpoint<AxisDatum>,
    pub first_mid_plane: DatumEndpoint<PlaneDatum>,
    pub second_mid_plane: DatumEndpoint<PlaneDatum>,
    pub kind: GearMeshKind,
    pub target_backlash: NonNegativeLength,
    pub reference_phase: Angle,
    pub tooth_period: PositiveAngle,
    pub phase_backlash: NonNegativeAngle,
    pub tolerance: EngineeringTolerance,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AssemblyRelation {
    SurfaceContact(SurfaceContact),
    PlaneClearance(PlaneClearance),
    Fastened(FastenedJoint),
    CylindricalFit(CylindricalFit),
    GearMesh(GearMesh),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct RelationEndpointRef {
    pub instance: ComponentInstanceId,
    pub datum_owner: Option<ComponentDefinitionId>,
    pub datum_index: usize,
    pub kind: DatumKind,
}

impl AssemblyRelationId {
    pub(crate) fn new(index: usize) -> Self {
        Self(index as u32)
    }

    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

impl<T> DatumEndpoint<T> {
    pub const fn new(instance: ComponentInstanceId, datum: DatumId<T>) -> Self {
        Self {
            instance,
            datum,
            marker: PhantomData,
        }
    }
}

impl AssemblyRelation {
    pub(crate) fn endpoint_refs(self) -> [Option<RelationEndpointRef>; 16] {
        match self {
            Self::SurfaceContact(contact) => [
                Some(erased(contact.first, DatumKind::Plane)),
                Some(erased(contact.second, DatumKind::Plane)),
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
            ],
            Self::PlaneClearance(clearance) => [
                Some(erased(clearance.first, DatumKind::Plane)),
                Some(erased(clearance.second, DatumKind::Plane)),
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
            ],
            Self::Fastened(joint) => {
                let bolt = joint.hardware.bolt;
                let nut = joint.hardware.nut;
                let first_washer = joint.hardware.first_washer;
                let second_washer = joint.hardware.second_washer;
                [
                    Some(erased(joint.first_hole, DatumKind::Cylinder)),
                    Some(erased(joint.second_hole, DatumKind::Cylinder)),
                    Some(erased(joint.head_seat, DatumKind::Plane)),
                    Some(erased(joint.nut_seat, DatumKind::Plane)),
                    Some(erased(
                        DatumEndpoint::new(bolt.instance, bolt.axis),
                        DatumKind::Axis,
                    )),
                    Some(erased(
                        DatumEndpoint::new(bolt.instance, bolt.under_head_face),
                        DatumKind::Plane,
                    )),
                    Some(erased(
                        DatumEndpoint::new(bolt.instance, bolt.shank_tip_face),
                        DatumKind::Plane,
                    )),
                    Some(erased(
                        DatumEndpoint::new(nut.instance, nut.axis),
                        DatumKind::Axis,
                    )),
                    Some(erased(
                        DatumEndpoint::new(nut.instance, nut.bearing_face),
                        DatumKind::Plane,
                    )),
                    Some(erased(
                        DatumEndpoint::new(nut.instance, nut.outer_face),
                        DatumKind::Plane,
                    )),
                    first_washer.map(|washer| {
                        erased(
                            DatumEndpoint::new(washer.instance, washer.axis),
                            DatumKind::Axis,
                        )
                    }),
                    first_washer.map(|washer| {
                        erased(
                            DatumEndpoint::new(washer.instance, washer.member_face),
                            DatumKind::Plane,
                        )
                    }),
                    first_washer.map(|washer| {
                        erased(
                            DatumEndpoint::new(washer.instance, washer.hardware_face),
                            DatumKind::Plane,
                        )
                    }),
                    second_washer.map(|washer| {
                        erased(
                            DatumEndpoint::new(washer.instance, washer.axis),
                            DatumKind::Axis,
                        )
                    }),
                    second_washer.map(|washer| {
                        erased(
                            DatumEndpoint::new(washer.instance, washer.member_face),
                            DatumKind::Plane,
                        )
                    }),
                    second_washer.map(|washer| {
                        erased(
                            DatumEndpoint::new(washer.instance, washer.hardware_face),
                            DatumKind::Plane,
                        )
                    }),
                ]
            }
            Self::CylindricalFit(fit) => [
                Some(erased(fit.shaft, DatumKind::Cylinder)),
                Some(erased(fit.bore, DatumKind::Cylinder)),
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
            ],
            Self::GearMesh(mesh) => [
                Some(erased(mesh.first_axis, DatumKind::Axis)),
                Some(erased(mesh.second_axis, DatumKind::Axis)),
                Some(erased(mesh.first_mid_plane, DatumKind::Plane)),
                Some(erased(mesh.second_mid_plane, DatumKind::Plane)),
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
            ],
        }
    }

    pub(crate) const fn instance_pair(self) -> [ComponentInstanceId; 2] {
        match self {
            Self::SurfaceContact(contact) => [contact.first.instance, contact.second.instance],
            Self::PlaneClearance(clearance) => {
                [clearance.first.instance, clearance.second.instance]
            }
            Self::Fastened(joint) => [joint.first_hole.instance, joint.second_hole.instance],
            Self::CylindricalFit(fit) => [fit.shaft.instance, fit.bore.instance],
            Self::GearMesh(mesh) => [mesh.first_axis.instance, mesh.second_axis.instance],
        }
    }

    pub(crate) fn participant_instances(self) -> [Option<ComponentInstanceId>; 6] {
        match self {
            Self::Fastened(joint) => [
                Some(joint.first_hole.instance),
                Some(joint.second_hole.instance),
                Some(joint.hardware.bolt.instance),
                Some(joint.hardware.nut.instance),
                joint.hardware.first_washer.map(|washer| washer.instance),
                joint.hardware.second_washer.map(|washer| washer.instance),
            ],
            _ => {
                let [first, second] = self.instance_pair();
                [Some(first), Some(second), None, None, None, None]
            }
        }
    }
}

fn erased<T: DatumType>(endpoint: DatumEndpoint<T>, kind: DatumKind) -> RelationEndpointRef {
    RelationEndpointRef {
        instance: endpoint.instance,
        datum_owner: endpoint.datum.owner(),
        datum_index: endpoint.datum.index(),
        kind,
    }
}

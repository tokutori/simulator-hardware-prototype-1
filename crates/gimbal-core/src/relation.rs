// SPDX-License-Identifier: MIT

use core::marker::PhantomData;

use crate::assembly::ComponentInstanceId;
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
pub struct CylindricalFit {
    pub shaft: DatumEndpoint<CylinderDatum>,
    pub bore: DatumEndpoint<CylinderDatum>,
    pub target_radial_clearance: NonNegativeLength,
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
    CylindricalFit(CylindricalFit),
    GearMesh(GearMesh),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct RelationEndpointRef {
    pub instance: ComponentInstanceId,
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
    pub(crate) fn endpoint_refs(self) -> [Option<RelationEndpointRef>; 4] {
        match self {
            Self::SurfaceContact(contact) => [
                Some(erased(contact.first, DatumKind::Plane)),
                Some(erased(contact.second, DatumKind::Plane)),
                None,
                None,
            ],
            Self::CylindricalFit(fit) => [
                Some(erased(fit.shaft, DatumKind::Cylinder)),
                Some(erased(fit.bore, DatumKind::Cylinder)),
                None,
                None,
            ],
            Self::GearMesh(mesh) => [
                Some(erased(mesh.first_axis, DatumKind::Axis)),
                Some(erased(mesh.second_axis, DatumKind::Axis)),
                Some(erased(mesh.first_mid_plane, DatumKind::Plane)),
                Some(erased(mesh.second_mid_plane, DatumKind::Plane)),
            ],
        }
    }

    pub(crate) const fn instance_pair(self) -> [ComponentInstanceId; 2] {
        match self {
            Self::SurfaceContact(contact) => [contact.first.instance, contact.second.instance],
            Self::CylindricalFit(fit) => [fit.shaft.instance, fit.bore.instance],
            Self::GearMesh(mesh) => [mesh.first_axis.instance, mesh.second_axis.instance],
        }
    }
}

fn erased<T: DatumType>(endpoint: DatumEndpoint<T>, kind: DatumKind) -> RelationEndpointRef {
    RelationEndpointRef {
        instance: endpoint.instance,
        datum_index: endpoint.datum.index(),
        kind,
    }
}

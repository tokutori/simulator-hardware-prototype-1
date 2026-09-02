// SPDX-License-Identifier: MIT

use alloc::string::String;
use alloc::vec::Vec;
use core::marker::PhantomData;

use crate::{ComponentDefinitionId, PositiveLength};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point3 {
    coordinates_mm: [f64; 3],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitVector3 {
    components: [f64; 3],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointDatum {
    pub point: Point3,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AxisDatum {
    pub origin: Point3,
    pub direction: UnitVector3,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlaneDatum {
    pub origin: Point3,
    pub normal: UnitVector3,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CylinderDatum {
    pub axis: AxisDatum,
    pub radius: PositiveLength,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DatumError {
    NonFinite,
    ZeroDirection,
}

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct DatumId<T> {
    owner: Option<ComponentDefinitionId>,
    index: u32,
    marker: PhantomData<fn() -> T>,
}

impl<T> Clone for DatumId<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T> Copy for DatumId<T> {}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DatumGeometry {
    Point(PointDatum),
    Axis(AxisDatum),
    Plane(PlaneDatum),
    Cylinder(CylinderDatum),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DatumKind {
    Point,
    Axis,
    Plane,
    Cylinder,
}

#[derive(Clone, Debug, PartialEq)]
pub struct NamedDatum {
    pub name: String,
    pub geometry: DatumGeometry,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct DatumSet {
    owner: Option<ComponentDefinitionId>,
    datums: Vec<NamedDatum>,
}

pub trait DatumType: Copy {
    fn into_geometry(self) -> DatumGeometry;
    fn from_geometry(geometry: DatumGeometry) -> Option<Self>;
}

impl Point3 {
    pub fn from_mm(coordinates_mm: [f64; 3]) -> Result<Self, DatumError> {
        if coordinates_mm.iter().all(|value| value.is_finite()) {
            Ok(Self { coordinates_mm })
        } else {
            Err(DatumError::NonFinite)
        }
    }

    pub const fn coordinates_mm(self) -> [f64; 3] {
        self.coordinates_mm
    }
}

impl UnitVector3 {
    pub fn new(components: [f64; 3]) -> Result<Self, DatumError> {
        if !components.iter().all(|value| value.is_finite()) {
            return Err(DatumError::NonFinite);
        }
        let norm = libm::sqrt(
            components[0] * components[0]
                + components[1] * components[1]
                + components[2] * components[2],
        );
        if norm <= f64::EPSILON {
            return Err(DatumError::ZeroDirection);
        }
        Ok(Self {
            components: [
                components[0] / norm,
                components[1] / norm,
                components[2] / norm,
            ],
        })
    }

    pub const fn components(self) -> [f64; 3] {
        self.components
    }
}

impl<T> DatumId<T> {
    pub const fn index(self) -> usize {
        self.index as usize
    }

    fn new(owner: Option<ComponentDefinitionId>, index: usize) -> Self {
        Self {
            owner,
            index: index as u32,
            marker: PhantomData,
        }
    }

    pub(crate) const fn owner(self) -> Option<ComponentDefinitionId> {
        self.owner
    }
}

impl DatumSet {
    pub const fn new() -> Self {
        Self {
            owner: None,
            datums: Vec::new(),
        }
    }

    pub const fn for_definition(owner: ComponentDefinitionId) -> Self {
        Self {
            owner: Some(owner),
            datums: Vec::new(),
        }
    }

    pub fn add<T: DatumType>(&mut self, name: String, datum: T) -> DatumId<T> {
        let id = DatumId::new(self.owner, self.datums.len());
        self.datums.push(NamedDatum {
            name,
            geometry: datum.into_geometry(),
        });
        id
    }

    pub fn get<T: DatumType>(&self, id: DatumId<T>) -> Option<T> {
        if id.owner != self.owner {
            return None;
        }
        self.datums
            .get(id.index())
            .and_then(|datum| T::from_geometry(datum.geometry))
    }

    pub fn named(&self, index: usize) -> Option<&NamedDatum> {
        self.datums.get(index)
    }

    pub fn len(&self) -> usize {
        self.datums.len()
    }

    pub fn is_empty(&self) -> bool {
        self.datums.is_empty()
    }

    pub(crate) const fn owner(&self) -> Option<ComponentDefinitionId> {
        self.owner
    }
}

impl DatumGeometry {
    pub const fn kind(self) -> DatumKind {
        match self {
            Self::Point(_) => DatumKind::Point,
            Self::Axis(_) => DatumKind::Axis,
            Self::Plane(_) => DatumKind::Plane,
            Self::Cylinder(_) => DatumKind::Cylinder,
        }
    }
}

macro_rules! impl_datum_type {
    ($type:ty, $variant:ident) => {
        impl DatumType for $type {
            fn into_geometry(self) -> DatumGeometry {
                DatumGeometry::$variant(self)
            }

            fn from_geometry(geometry: DatumGeometry) -> Option<Self> {
                match geometry {
                    DatumGeometry::$variant(value) => Some(value),
                    _ => None,
                }
            }
        }
    };
}

impl_datum_type!(PointDatum, Point);
impl_datum_type!(AxisDatum, Axis);
impl_datum_type!(PlaneDatum, Plane);
impl_datum_type!(CylinderDatum, Cylinder);

#[cfg(test)]
mod tests {
    use alloc::string::ToString;

    use super::*;

    #[test]
    fn unit_vector_rejects_invalid_direction_and_normalizes_valid_input() {
        assert_eq!(
            UnitVector3::new([0.0, 0.0, 0.0]),
            Err(DatumError::ZeroDirection)
        );
        let direction = UnitVector3::new([0.0, 3.0, 4.0]).expect("valid direction");
        assert_eq!(direction.components(), [0.0, 0.6, 0.8]);
    }

    #[test]
    fn typed_datum_id_only_recovers_the_matching_geometry() {
        let mut datums = DatumSet::new();
        let axis = AxisDatum {
            origin: Point3::from_mm([1.0, 2.0, 3.0]).expect("finite point"),
            direction: UnitVector3::new([1.0, 0.0, 0.0]).expect("valid direction"),
        };
        let id = datums.add("shaft_axis".to_string(), axis);
        assert_eq!(datums.get(id), Some(axis));
        assert_eq!(
            datums.named(id.index()).expect("named datum").name,
            "shaft_axis"
        );
    }
}

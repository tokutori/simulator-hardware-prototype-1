// SPDX-License-Identifier: MIT

use alloc::vec::Vec;

use crate::{Angle, Length};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point2 {
    pub x: f64,
    pub y: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct TriangleMesh {
    pub vertices: Vec<[f64; 3]>,
    pub triangles: Vec<[u32; 3]>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct RegionId(u32);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SolidId(u32);

#[derive(Clone, Debug, PartialEq)]
pub enum RegionNode {
    Polygon(Vec<Point2>),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Primitive3 {
    Box {
        x: Length,
        y: Length,
        z: Length,
        centered: bool,
    },
    Cylinder {
        height: Length,
        radius: Length,
        segments: u16,
        centered: bool,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Translation3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rotation3 {
    pub x: Angle,
    pub y: Angle,
    pub z: Angle,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BooleanOperation {
    Union,
    Difference,
    Intersection,
}

#[derive(Clone, Debug, PartialEq)]
pub enum SolidNode {
    Primitive(Primitive3),
    Extrude {
        profile: RegionId,
        distance: Length,
    },
    Translate {
        solid: SolidId,
        by: Translation3,
    },
    Rotate {
        solid: SolidId,
        by: Rotation3,
    },
    Boolean {
        operation: BooleanOperation,
        lhs: SolidId,
        rhs: SolidId,
    },
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct FeatureGraph {
    regions: Vec<RegionNode>,
    solids: Vec<SolidNode>,
}

#[derive(Clone, Debug, Default)]
pub struct FeatureBuilder {
    graph: FeatureGraph,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FeatureError {
    PolygonTooSmall,
    NonFiniteCoordinate,
    InvalidRegionId,
    InvalidSolidId,
}

impl RegionId {
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

impl SolidId {
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

impl FeatureGraph {
    pub fn region(&self, id: RegionId) -> Option<&RegionNode> {
        self.regions.get(id.index())
    }

    pub fn solid(&self, id: SolidId) -> Option<&SolidNode> {
        self.solids.get(id.index())
    }

    pub fn region_count(&self) -> usize {
        self.regions.len()
    }

    pub fn solid_count(&self) -> usize {
        self.solids.len()
    }
}

impl FeatureBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn polygon(&mut self, points: Vec<Point2>) -> Result<RegionId, FeatureError> {
        if points.len() < 3 {
            return Err(FeatureError::PolygonTooSmall);
        }
        if points
            .iter()
            .any(|point| !point.x.is_finite() || !point.y.is_finite())
        {
            return Err(FeatureError::NonFiniteCoordinate);
        }
        let id = RegionId(self.graph.regions.len() as u32);
        self.graph.regions.push(RegionNode::Polygon(points));
        Ok(id)
    }

    pub fn primitive(&mut self, primitive: Primitive3) -> SolidId {
        self.push_solid(SolidNode::Primitive(primitive))
    }

    pub fn extrude(
        &mut self,
        profile: RegionId,
        distance: Length,
    ) -> Result<SolidId, FeatureError> {
        self.require_region(profile)?;
        Ok(self.push_solid(SolidNode::Extrude { profile, distance }))
    }

    pub fn translate(&mut self, solid: SolidId, by: Translation3) -> Result<SolidId, FeatureError> {
        self.require_solid(solid)?;
        if !by.x.is_finite() || !by.y.is_finite() || !by.z.is_finite() {
            return Err(FeatureError::NonFiniteCoordinate);
        }
        Ok(self.push_solid(SolidNode::Translate { solid, by }))
    }

    pub fn rotate(&mut self, solid: SolidId, by: Rotation3) -> Result<SolidId, FeatureError> {
        self.require_solid(solid)?;
        Ok(self.push_solid(SolidNode::Rotate { solid, by }))
    }

    pub fn boolean(
        &mut self,
        operation: BooleanOperation,
        lhs: SolidId,
        rhs: SolidId,
    ) -> Result<SolidId, FeatureError> {
        self.require_solid(lhs)?;
        self.require_solid(rhs)?;
        Ok(self.push_solid(SolidNode::Boolean {
            operation,
            lhs,
            rhs,
        }))
    }

    pub fn finish(self) -> FeatureGraph {
        self.graph
    }

    fn push_solid(&mut self, node: SolidNode) -> SolidId {
        let id = SolidId(self.graph.solids.len() as u32);
        self.graph.solids.push(node);
        id
    }

    fn require_region(&self, id: RegionId) -> Result<(), FeatureError> {
        self.graph
            .region(id)
            .map(|_| ())
            .ok_or(FeatureError::InvalidRegionId)
    }

    fn require_solid(&self, id: SolidId) -> Result<(), FeatureError> {
        self.graph
            .solid(id)
            .map(|_| ())
            .ok_or(FeatureError::InvalidSolidId)
    }
}

#[cfg(test)]
mod tests {
    use alloc::vec;

    use super::*;

    #[test]
    fn region_and_solid_ids_cannot_be_interchanged() {
        let mut builder = FeatureBuilder::new();
        let region = builder
            .polygon(vec![
                Point2 { x: 0.0, y: 0.0 },
                Point2 { x: 1.0, y: 0.0 },
                Point2 { x: 0.0, y: 1.0 },
            ])
            .unwrap();
        let solid = builder
            .extrude(region, Length::positive_mm(2.0).unwrap())
            .unwrap();
        assert_eq!(region.index(), 0);
        assert_eq!(solid.index(), 0);
        let graph = builder.finish();
        assert_eq!(graph.region_count(), 1);
        assert_eq!(graph.solid_count(), 1);
    }

    #[test]
    fn append_only_graph_cannot_reference_future_solid() {
        let mut builder = FeatureBuilder::new();
        let invalid = SolidId(0);
        let result = builder.translate(
            invalid,
            Translation3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert_eq!(result, Err(FeatureError::InvalidSolidId));
    }
}

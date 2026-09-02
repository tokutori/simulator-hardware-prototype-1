// SPDX-License-Identifier: MIT

use gimbal_core::{
    BooleanOperation, FeatureGraph, Primitive3, RegionNode, RigidTransform, Rotation3, SolidId,
    SolidNode, TriangleMesh,
};
use manifold_rust::linalg::{Mat3x4, Vec2, Vec3};
use manifold_rust::manifold::Manifold;
use manifold_rust::types::{BooleanEngine, Error as ManifoldStatus};
use thiserror::Error;

mod validation;

pub use validation::{
    AssemblyValidator, DefinitionValidation, PairValidation, ValidationError, ValidationIssue,
    ValidationIssueKind, ValidationProgress, ValidationReport, ValidationSeverity,
    ValidatorSettings,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SolidMetrics {
    pub volume_mm3: f64,
    pub surface_area_mm2: f64,
    pub vertices: usize,
    pub triangles: usize,
}

#[derive(Debug, Error)]
pub enum KernelError {
    #[error("unknown solid id {0}")]
    UnknownSolid(usize),
    #[error("unknown region id {0}")]
    UnknownRegion(usize),
    #[error("geometry kernel returned {0:?}")]
    Manifold(ManifoldStatus),
    #[error("geometry operation produced an empty solid")]
    EmptySolid,
}

pub struct Evaluator<'a> {
    graph: &'a FeatureGraph,
    cache: Vec<Option<Manifold>>,
}

impl<'a> Evaluator<'a> {
    pub fn new(graph: &'a FeatureGraph) -> Self {
        Self {
            graph,
            cache: vec![None; graph.solid_count()],
        }
    }

    pub fn evaluate(&mut self, id: SolidId) -> Result<Manifold, KernelError> {
        if let Some(cached) = self.cache.get(id.index()).and_then(Clone::clone) {
            return Ok(cached);
        }
        let node = self
            .graph
            .solid(id)
            .ok_or(KernelError::UnknownSolid(id.index()))?
            .clone();
        let solid = match node {
            SolidNode::Primitive(primitive) => self.primitive(primitive),
            SolidNode::Extrude { profile, distance } => {
                let region = self
                    .graph
                    .region(profile)
                    .ok_or(KernelError::UnknownRegion(profile.index()))?;
                let RegionNode::Polygon(points) = region;
                let polygon = points
                    .iter()
                    .map(|point| Vec2::new(point.x, point.y))
                    .collect::<Vec<_>>();
                Manifold::extrude(&vec![polygon], distance.mm(), 0, 0.0, Vec2::new(1.0, 1.0))
            }
            SolidNode::Translate { solid, by } => {
                self.evaluate(solid)?.translate(Vec3::new(by.x, by.y, by.z))
            }
            SolidNode::Rotate { solid, by } => rotate(self.evaluate(solid)?, by),
            SolidNode::Boolean {
                operation,
                lhs,
                rhs,
            } => {
                let lhs = self.evaluate(lhs)?;
                let rhs = self.evaluate(rhs)?;
                match operation {
                    BooleanOperation::Union => lhs.union_with_engine(&rhs, BooleanEngine::Robust),
                    BooleanOperation::Difference => {
                        lhs.difference_with_engine(&rhs, BooleanEngine::Robust)
                    }
                    BooleanOperation::Intersection => {
                        lhs.intersection_with_engine(&rhs, BooleanEngine::Robust)
                    }
                }
            }
        };
        validate_solid(&solid)?;
        self.cache[id.index()] = Some(solid.clone());
        Ok(solid)
    }

    pub fn mesh(&mut self, id: SolidId) -> Result<TriangleMesh, KernelError> {
        let solid = self.evaluate(id)?;
        Ok(mesh_from_manifold(&solid))
    }

    pub fn metrics(&mut self, id: SolidId) -> Result<SolidMetrics, KernelError> {
        let solid = self.evaluate(id)?;
        Ok(SolidMetrics {
            volume_mm3: solid.volume(),
            surface_area_mm2: solid.surface_area(),
            vertices: solid.num_vert(),
            triangles: solid.num_tri(),
        })
    }

    pub fn intersection_volume(&mut self, lhs: SolidId, rhs: SolidId) -> Result<f64, KernelError> {
        let lhs = self.evaluate(lhs)?;
        let rhs = self.evaluate(rhs)?;
        let intersection = lhs.intersection(&rhs);
        if intersection.status() != ManifoldStatus::NoError {
            return Err(KernelError::Manifold(intersection.status()));
        }
        Ok(intersection.volume())
    }

    pub fn intersection_volume_transformed(
        &mut self,
        lhs: SolidId,
        lhs_pose: RigidTransform,
        rhs: SolidId,
        rhs_pose: RigidTransform,
    ) -> Result<f64, KernelError> {
        let lhs = transform(self.evaluate(lhs)?, lhs_pose);
        let rhs = transform(self.evaluate(rhs)?, rhs_pose);
        let intersection = lhs.intersection(&rhs);
        if intersection.status() != ManifoldStatus::NoError {
            return Err(KernelError::Manifold(intersection.status()));
        }
        Ok(intersection.volume())
    }

    fn primitive(&self, primitive: Primitive3) -> Manifold {
        match primitive {
            Primitive3::Box { x, y, z, centered } => {
                Manifold::cube(Vec3::new(x.mm(), y.mm(), z.mm()), centered)
            }
            Primitive3::Cylinder {
                height,
                radius,
                segments,
                centered,
            } => Manifold::cylinder_centered(
                height.mm(),
                radius.mm(),
                radius.mm(),
                i32::from(segments),
                centered,
            ),
        }
    }
}

pub(crate) fn transform(solid: Manifold, pose: RigidTransform) -> Manifold {
    let [x, y, z, w] = pose.rotation;
    let xx = x * x;
    let yy = y * y;
    let zz = z * z;
    let xy = x * y;
    let xz = x * z;
    let yz = y * z;
    let wx = w * x;
    let wy = w * y;
    let wz = w * z;
    let matrix = Mat3x4::from_cols(
        Vec3::new(1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy)),
        Vec3::new(2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx)),
        Vec3::new(2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy)),
        Vec3::new(
            pose.translation[0],
            pose.translation[1],
            pose.translation[2],
        ),
    );
    solid.transform(&matrix)
}

pub fn mesh_from_manifold(solid: &Manifold) -> TriangleMesh {
    let mesh = solid.get_mesh_gl64(-1);
    let vertices = (0..mesh.num_vert())
        .map(|index| mesh.get_vert_pos(index))
        .collect();
    let triangles = (0..mesh.num_tri())
        .map(|index| {
            let triangle = mesh.get_tri_verts(index);
            [
                u32::try_from(triangle[0]).expect("mesh index fits u32"),
                u32::try_from(triangle[1]).expect("mesh index fits u32"),
                u32::try_from(triangle[2]).expect("mesh index fits u32"),
            ]
        })
        .collect();
    TriangleMesh {
        vertices,
        triangles,
    }
}

fn rotate(solid: Manifold, rotation: Rotation3) -> Manifold {
    solid.rotate(
        rotation.x.as_degrees(),
        rotation.y.as_degrees(),
        rotation.z.as_degrees(),
    )
}

fn validate_solid(solid: &Manifold) -> Result<(), KernelError> {
    if solid.status() != ManifoldStatus::NoError {
        Err(KernelError::Manifold(solid.status()))
    } else if solid.is_empty() || solid.volume() <= 0.0 {
        Err(KernelError::EmptySolid)
    } else {
        Ok(())
    }
}

// SPDX-License-Identifier: MIT

use gimbal_core::RigidTransform;
use manifold_rust::manifold::Manifold;

pub(super) struct InstanceGeometry {
    pub(super) solid: Manifold,
    pub(super) bounds: Aabb3,
    pub(super) world_pose: RigidTransform,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Aabb3 {
    minimum: [f64; 3],
    maximum: [f64; 3],
}

impl Aabb3 {
    pub(super) fn from_manifold(solid: &Manifold) -> Self {
        let bounds = solid.bounding_box();
        Self {
            minimum: [bounds.min.x, bounds.min.y, bounds.min.z],
            maximum: [bounds.max.x, bounds.max.y, bounds.max.z],
        }
    }

    pub(super) fn has_interior_overlap(self, other: Self, epsilon: f64) -> bool {
        (0..3).all(|axis| {
            self.maximum[axis].min(other.maximum[axis])
                - self.minimum[axis].max(other.minimum[axis])
                > epsilon
        })
    }

    pub(super) fn interior_overlap_volume(self, other: Self) -> f64 {
        (0..3)
            .map(|axis| {
                (self.maximum[axis].min(other.maximum[axis])
                    - self.minimum[axis].max(other.minimum[axis]))
                .max(0.0)
            })
            .product()
    }

    pub(super) fn is_within_gap(self, other: Self, gap: f64) -> bool {
        (0..3).all(|axis| {
            self.minimum[axis] <= other.maximum[axis] + gap
                && other.minimum[axis] <= self.maximum[axis] + gap
        })
    }

    pub(super) fn minimum_gap(self, other: Self) -> f64 {
        let squared = (0..3)
            .map(|axis| {
                let axis_gap = (self.minimum[axis] - other.maximum[axis])
                    .max(other.minimum[axis] - self.maximum[axis])
                    .max(0.0);
                axis_gap * axis_gap
            })
            .sum::<f64>();
        squared.sqrt()
    }
}

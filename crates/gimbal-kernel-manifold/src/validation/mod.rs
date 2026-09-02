// SPDX-License-Identifier: MIT

use gimbal_core::{
    Assembly, AssemblyPose, ComponentInstanceId, ComponentInstancePair, FeatureGraph,
};
use manifold_rust::types::Error as ManifoldStatus;
use thiserror::Error;

use crate::KernelError;

mod instance;
mod interference;
mod plan;
mod proximity;
mod relations;
mod report;

use instance::{Aabb3, InstanceGeometry};

pub use plan::*;
pub use report::*;

#[derive(Debug, Error)]
pub enum ValidationError {
    #[error(transparent)]
    Kernel(#[from] KernelError),
    #[error("instance {0:?} references a frame that is absent from the assembly pose")]
    MissingFrame(ComponentInstanceId),
    #[error("geometry kernel returned {status:?} while checking instance pair {pair:?}")]
    PairIntersection {
        pair: ComponentInstancePair,
        status: ManifoldStatus,
    },
    #[error("geometry kernel panicked while checking instance pair {0:?}")]
    PairPanicked(ComponentInstancePair),
}

pub struct AssemblyValidator<'a> {
    graph: &'a FeatureGraph,
    assembly: &'a Assembly,
    pose: &'a AssemblyPose,
    settings: ValidatorSettings,
}

#[cfg(test)]
mod tests;

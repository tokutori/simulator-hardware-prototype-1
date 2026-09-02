// SPDX-License-Identifier: MIT

use gimbal_core::{ComponentDefinitionId, NonNegativeLength, NumericalTolerance};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UnrelatedProximityPolicy {
    Ignore,
    Warning,
    Error,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeometryFidelity {
    StructuralProxy,
    Exact,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotionCoverage {
    StaticPose,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ValidationProfile {
    pub geometry: GeometryFidelity,
    pub motion: MotionCoverage,
}

impl ValidationProfile {
    pub const STRUCTURAL_STATIC: Self = Self {
        geometry: GeometryFidelity::StructuralProxy,
        motion: MotionCoverage::StaticPose,
    };

    pub const EXACT_STATIC: Self = Self {
        geometry: GeometryFidelity::Exact,
        motion: MotionCoverage::StaticPose,
    };
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ValidationPlan {
    pub profile: ValidationProfile,
    included_definitions: Option<Vec<ComponentDefinitionId>>,
}

impl ValidationPlan {
    pub const fn all(profile: ValidationProfile) -> Self {
        Self {
            profile,
            included_definitions: None,
        }
    }

    pub fn include_only(
        profile: ValidationProfile,
        definitions: impl IntoIterator<Item = ComponentDefinitionId>,
    ) -> Self {
        Self {
            profile,
            included_definitions: Some(definitions.into_iter().collect()),
        }
    }

    pub fn includes_definition(&self, definition: ComponentDefinitionId) -> bool {
        self.included_definitions
            .as_ref()
            .is_none_or(|included| included.contains(&definition))
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ValidatorSettings {
    pub plan: ValidationPlan,
    pub numerical_tolerance: NumericalTolerance,
    pub unrelated_proximity_threshold: NonNegativeLength,
    pub unrelated_proximity_policy: UnrelatedProximityPolicy,
}

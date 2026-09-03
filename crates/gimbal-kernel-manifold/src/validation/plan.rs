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
pub enum DefinitionCoverage {
    Selected,
    All,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotionCoverage {
    StaticPose,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ValidationProfile {
    pub definitions: DefinitionCoverage,
    pub geometry: GeometryFidelity,
    pub motion: MotionCoverage,
}

impl ValidationProfile {
    pub const STRUCTURAL_PROXY_STATIC: Self = Self {
        definitions: DefinitionCoverage::Selected,
        geometry: GeometryFidelity::StructuralProxy,
        motion: MotionCoverage::StaticPose,
    };

    pub const STRUCTURAL_EXACT_STATIC: Self = Self {
        definitions: DefinitionCoverage::Selected,
        geometry: GeometryFidelity::Exact,
        motion: MotionCoverage::StaticPose,
    };

    pub const EXACT_STATIC: Self = Self {
        definitions: DefinitionCoverage::All,
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

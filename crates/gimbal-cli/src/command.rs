// SPDX-License-Identifier: MIT

use crate::generate::GenerationMode;
use gimbal_kernel_manifold::ValidationProfile;
use thiserror::Error;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Command {
    Generate(GenerationMode),
    Validate(ValidationProfile),
    RefreshManifest,
    CleanOutput,
    Help,
}

impl Command {
    pub(crate) const HELP: &'static str = "Usage: gimbal [COMMAND]\n\
\n\
Commands:\n\
  generate           Validate exact static geometry, then generate artifacts\n\
  generate-preview   Generate unvalidated preview artifacts\n\
  validate-proxy     Fast AABB candidate scan without high-detail gears\n\
  validate           Exact structural validation without high-detail gears\n\
  validate-full      Exact validation including high-detail gears at the static pose\n\
  refresh-manifest   Rehash artifacts already present in output\n\
  clean-output       Remove the generated output directory\n\
  help               Show this help";

    pub(crate) fn parse(argument: Option<&str>) -> Result<Self, CommandError> {
        match argument.unwrap_or("generate") {
            "generate" => Ok(Self::Generate(GenerationMode::Validated)),
            "generate-preview" => Ok(Self::Generate(GenerationMode::PreviewOnly)),
            "validate-proxy" => Ok(Self::Validate(ValidationProfile::STRUCTURAL_PROXY_STATIC)),
            "validate" => Ok(Self::Validate(ValidationProfile::STRUCTURAL_EXACT_STATIC)),
            "validate-full" => Ok(Self::Validate(ValidationProfile::EXACT_STATIC)),
            "refresh-manifest" => Ok(Self::RefreshManifest),
            "clean-output" => Ok(Self::CleanOutput),
            "help" | "--help" | "-h" => Ok(Self::Help),
            unknown => Err(CommandError::Unknown(unknown.to_owned())),
        }
    }
}

#[derive(Debug, Error, PartialEq, Eq)]
pub(crate) enum CommandError {
    #[error("unknown command {0:?}; run 'gimbal help' for usage")]
    Unknown(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_to_validated_generation() {
        assert_eq!(
            Command::parse(None),
            Ok(Command::Generate(GenerationMode::Validated))
        );
    }

    #[test]
    fn parses_each_supported_command_and_rejects_unknown_values() {
        assert_eq!(Command::parse(Some("-h")), Ok(Command::Help));
        assert_eq!(
            Command::parse(Some("validate-proxy")),
            Ok(Command::Validate(
                ValidationProfile::STRUCTURAL_PROXY_STATIC
            ))
        );
        assert_eq!(
            Command::parse(Some("validate")),
            Ok(Command::Validate(
                ValidationProfile::STRUCTURAL_EXACT_STATIC
            ))
        );
        assert_eq!(
            Command::parse(Some("validate-full")),
            Ok(Command::Validate(ValidationProfile::EXACT_STATIC))
        );
        assert_eq!(
            Command::parse(Some("nope")),
            Err(CommandError::Unknown("nope".to_owned()))
        );
    }
}

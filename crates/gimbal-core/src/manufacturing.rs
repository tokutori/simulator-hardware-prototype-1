// SPDX-License-Identifier: MIT

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FdmMaterial {
    Petg,
    Pla,
    Abs,
    Asa,
}

/// Nominal fabrication intent. Machine calibration belongs to the I/O/process layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Manufacturing {
    Fdm,
    LaserCut,
    Purchased,
}

impl FdmMaterial {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Petg => "PETG",
            Self::Pla => "PLA",
            Self::Abs => "ABS",
            Self::Asa => "ASA",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Side {
    Left,
    Right,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum LongitudinalEnd {
    Front,
    Rear,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VerticalEnd {
    Upper,
    Lower,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ComponentRole {
    PitchSector,
    FixedCarrierRail,
    FixedCarrierPost,
    FixedCrossmember,
    PitchCradleLongitudinalRail,
    RollBearingCarrierEnd,
    InstallationFloor,
    PitchDrivePinion,
    PitchRetentionPinion,
    PitchDriveFlange,
    PitchRetentionFlange,
    PitchDriveShaft,
    PitchRetentionShaft,
    PitchGearboxSmallGear,
    PitchGearboxDistributionGear,
    PitchGearboxLargeGear,
    PitchContactOutboardPlate,
    PitchContactCarriagePlate,
    PitchGearboxFarPlate,
    PitchGearboxShaft,
    Cockpit,
    CockpitHanger,
    RollShaft,
    RollDrivenGear,
    RollInputPinion,
    RollGearboxSmallGear,
    RollGearboxLargeGear,
    RollGearboxShaft,
    RollBearing,
    RollBearingRetainer,
    RollGearboxPlate,
    MovingDriveMountArm,
    M3Bolt,
    M3Nut,
    M3Washer,
}

impl ComponentRole {
    /// Components whose exact involute tooth meshes are intentionally omitted
    /// from fast, whole-assembly structural checks. Their envelopes and
    /// mechanical surroundings must still be checked independently of the
    /// expensive tooth-to-tooth validation route.
    pub const fn has_high_detail_gear_geometry(self) -> bool {
        matches!(
            self,
            Self::PitchSector
                | Self::PitchDrivePinion
                | Self::PitchRetentionPinion
                | Self::PitchGearboxSmallGear
                | Self::PitchGearboxDistributionGear
                | Self::PitchGearboxLargeGear
                | Self::RollDrivenGear
                | Self::RollInputPinion
                | Self::RollGearboxSmallGear
                | Self::RollGearboxLargeGear
        )
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct ComponentLocation {
    pub side: Option<Side>,
    pub longitudinal_end: Option<LongitudinalEnd>,
    pub vertical_end: Option<VerticalEnd>,
    pub ordinal: Option<u16>,
}

impl Side {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Left => "left",
            Self::Right => "right",
        }
    }
}

impl LongitudinalEnd {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Front => "front",
            Self::Rear => "rear",
        }
    }
}

impl VerticalEnd {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Upper => "upper",
            Self::Lower => "lower",
        }
    }
}

impl ComponentLocation {
    pub const fn new() -> Self {
        Self {
            side: None,
            longitudinal_end: None,
            vertical_end: None,
            ordinal: None,
        }
    }

    pub const fn with_side(mut self, side: Side) -> Self {
        self.side = Some(side);
        self
    }

    pub const fn with_longitudinal_end(mut self, end: LongitudinalEnd) -> Self {
        self.longitudinal_end = Some(end);
        self
    }

    pub const fn with_vertical_end(mut self, end: VerticalEnd) -> Self {
        self.vertical_end = Some(end);
        self
    }

    pub const fn with_ordinal(mut self, ordinal: u16) -> Self {
        self.ordinal = Some(ordinal);
        self
    }
}

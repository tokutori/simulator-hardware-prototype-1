// SPDX-License-Identifier: MIT

use core::fmt;

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Length(f64);

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Angle(f64);

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct PositiveLength(Length);

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct NonNegativeLength(Length);

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct NonNegativeAngle(Angle);

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct PositiveAngle(Angle);

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct PositiveArea(f64);

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct PositiveVolume(f64);

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct PositiveRatio(f64);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UnitError {
    NonFinite,
    NotPositive,
    Negative,
}

impl Length {
    pub fn positive_mm(value: f64) -> Result<Self, UnitError> {
        if !value.is_finite() {
            Err(UnitError::NonFinite)
        } else if value <= 0.0 {
            Err(UnitError::NotPositive)
        } else {
            Ok(Self(value))
        }
    }

    pub fn non_negative_mm(value: f64) -> Result<Self, UnitError> {
        if !value.is_finite() {
            Err(UnitError::NonFinite)
        } else if value < 0.0 {
            Err(UnitError::Negative)
        } else {
            Ok(Self(value))
        }
    }

    pub const fn mm(self) -> f64 {
        self.0
    }
}

impl Angle {
    pub fn degrees(value: f64) -> Result<Self, UnitError> {
        if value.is_finite() {
            Ok(Self(value.to_radians()))
        } else {
            Err(UnitError::NonFinite)
        }
    }

    pub fn radians(value: f64) -> Result<Self, UnitError> {
        if value.is_finite() {
            Ok(Self(value))
        } else {
            Err(UnitError::NonFinite)
        }
    }

    pub const fn as_radians(self) -> f64 {
        self.0
    }

    pub fn as_degrees(self) -> f64 {
        self.0.to_degrees()
    }
}

impl PositiveLength {
    pub fn mm(value: f64) -> Result<Self, UnitError> {
        Length::positive_mm(value).map(Self)
    }

    pub const fn as_length(self) -> Length {
        self.0
    }

    pub const fn as_mm(self) -> f64 {
        self.0.mm()
    }
}

impl NonNegativeLength {
    pub fn mm(value: f64) -> Result<Self, UnitError> {
        Length::non_negative_mm(value).map(Self)
    }

    pub const fn as_length(self) -> Length {
        self.0
    }

    pub const fn as_mm(self) -> f64 {
        self.0.mm()
    }
}

impl NonNegativeAngle {
    pub fn radians(value: f64) -> Result<Self, UnitError> {
        if !value.is_finite() {
            Err(UnitError::NonFinite)
        } else if value < 0.0 {
            Err(UnitError::Negative)
        } else {
            Ok(Self(Angle(value)))
        }
    }

    pub fn degrees(value: f64) -> Result<Self, UnitError> {
        if !value.is_finite() {
            Err(UnitError::NonFinite)
        } else if value < 0.0 {
            Err(UnitError::Negative)
        } else {
            Ok(Self(Angle(value.to_radians())))
        }
    }

    pub const fn as_angle(self) -> Angle {
        self.0
    }

    pub const fn as_radians(self) -> f64 {
        self.0.as_radians()
    }
}

impl PositiveAngle {
    pub fn radians(value: f64) -> Result<Self, UnitError> {
        if !value.is_finite() {
            Err(UnitError::NonFinite)
        } else if value <= 0.0 {
            Err(UnitError::NotPositive)
        } else {
            Ok(Self(Angle(value)))
        }
    }

    pub fn degrees(value: f64) -> Result<Self, UnitError> {
        if !value.is_finite() {
            Err(UnitError::NonFinite)
        } else if value <= 0.0 {
            Err(UnitError::NotPositive)
        } else {
            Ok(Self(Angle(value.to_radians())))
        }
    }

    pub const fn as_angle(self) -> Angle {
        self.0
    }

    pub const fn as_radians(self) -> f64 {
        self.0.as_radians()
    }
}

impl PositiveArea {
    pub fn square_mm(value: f64) -> Result<Self, UnitError> {
        if !value.is_finite() {
            Err(UnitError::NonFinite)
        } else if value <= 0.0 {
            Err(UnitError::NotPositive)
        } else {
            Ok(Self(value))
        }
    }

    pub const fn as_square_mm(self) -> f64 {
        self.0
    }
}

impl PositiveVolume {
    pub fn cubic_mm(value: f64) -> Result<Self, UnitError> {
        if !value.is_finite() {
            Err(UnitError::NonFinite)
        } else if value <= 0.0 {
            Err(UnitError::NotPositive)
        } else {
            Ok(Self(value))
        }
    }

    pub const fn as_cubic_mm(self) -> f64 {
        self.0
    }
}

impl PositiveRatio {
    pub fn new(value: f64) -> Result<Self, UnitError> {
        if !value.is_finite() {
            Err(UnitError::NonFinite)
        } else if value <= 0.0 {
            Err(UnitError::NotPositive)
        } else {
            Ok(Self(value))
        }
    }

    pub const fn get(self) -> f64 {
        self.0
    }
}

impl fmt::Display for UnitError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFinite => formatter.write_str("value must be finite"),
            Self::NotPositive => formatter.write_str("value must be greater than zero"),
            Self::Negative => formatter.write_str("value must not be negative"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tolerance_quantities_preserve_positive_and_non_negative_contracts() {
        assert!(PositiveLength::mm(0.0).is_err());
        assert!(PositiveArea::square_mm(0.0).is_err());
        assert!(PositiveVolume::cubic_mm(0.0).is_err());
        assert!(PositiveAngle::degrees(0.0).is_err());
        assert!(PositiveRatio::new(0.0).is_err());
        assert_eq!(PositiveRatio::new(0.005).unwrap().get(), 0.005);
        assert!(NonNegativeLength::mm(0.0).is_ok());
        assert!(NonNegativeAngle::degrees(0.0).is_ok());
        assert!(NonNegativeAngle::radians(-f64::EPSILON).is_err());
    }
}

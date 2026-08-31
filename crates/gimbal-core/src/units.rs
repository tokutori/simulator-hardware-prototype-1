// SPDX-License-Identifier: MIT

use core::fmt;

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Length(f64);

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Angle(f64);

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

impl fmt::Display for UnitError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFinite => formatter.write_str("value must be finite"),
            Self::NotPositive => formatter.write_str("value must be greater than zero"),
            Self::Negative => formatter.write_str("value must not be negative"),
        }
    }
}

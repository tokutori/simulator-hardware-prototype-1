// SPDX-License-Identifier: MIT

use alloc::vec::Vec;
use core::f64::consts::PI;

use crate::{Angle, DoubleHelicalGear, GearError, GearHand, GearProfile, Length, Point2, SpurGear};

#[derive(Clone, Debug, PartialEq)]
pub struct NormalGearSystem {
    normal_module: Length,
    normal_pressure_angle: Angle,
    helix_angle: Angle,
    backlash: Length,
    chord_tolerance: Length,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DoubleHelicalRack {
    system: NormalGearSystem,
    teeth: u16,
    face_width: Length,
    center_gap: Length,
    body_thickness: Length,
    slices_per_half: u16,
    hand: GearHand,
}

impl NormalGearSystem {
    pub fn new(
        normal_module: Length,
        normal_pressure_angle: Angle,
        helix_angle: Angle,
        backlash: Length,
        chord_tolerance: Length,
    ) -> Result<Self, GearError> {
        if !(10.0..=35.0).contains(&normal_pressure_angle.as_degrees()) {
            return Err(GearError::PressureAngleOutOfRange);
        }
        if !(5.0..=45.0).contains(&helix_angle.as_degrees().abs()) {
            return Err(GearError::HelixAngleOutOfRange);
        }
        if backlash.mm() >= PI * normal_module.mm() * 0.45 {
            return Err(GearError::ExcessiveBacklash);
        }
        Ok(Self {
            normal_module,
            normal_pressure_angle,
            helix_angle: Angle::radians(helix_angle.as_radians().abs())
                .expect("validated finite angle"),
            backlash,
            chord_tolerance,
        })
    }

    pub const fn normal_module(&self) -> Length {
        self.normal_module
    }

    pub const fn normal_pressure_angle(&self) -> Angle {
        self.normal_pressure_angle
    }

    pub const fn helix_angle(&self) -> Angle {
        self.helix_angle
    }

    pub fn transverse_module(&self) -> f64 {
        self.normal_module.mm() / libm::cos(self.helix_angle.as_radians())
    }

    pub fn transverse_pressure_angle(&self) -> f64 {
        libm::atan(
            libm::tan(self.normal_pressure_angle.as_radians())
                / libm::cos(self.helix_angle.as_radians()),
        )
    }

    pub fn transverse_pitch(&self) -> f64 {
        PI * self.transverse_module()
    }

    #[allow(clippy::too_many_arguments)]
    pub fn pinion(
        &self,
        teeth: u16,
        face_width: Length,
        center_gap: Length,
        bore_diameter: Length,
        slices_per_half: u16,
        hand: GearHand,
    ) -> Result<DoubleHelicalGear, GearError> {
        let transverse_module =
            Length::positive_mm(self.transverse_module()).expect("positive converted module");
        let transverse_pressure = Angle::radians(self.transverse_pressure_angle())
            .expect("finite converted pressure angle");
        let section = SpurGear::with_tooth_height_module(
            transverse_module,
            self.normal_module,
            teeth,
            transverse_pressure,
            self.backlash,
            self.chord_tolerance,
        )?;
        DoubleHelicalGear::new(
            section,
            face_width,
            center_gap,
            self.helix_angle,
            bore_diameter,
            slices_per_half,
            hand,
        )
    }
}

impl DoubleHelicalRack {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        system: NormalGearSystem,
        teeth: u16,
        face_width: Length,
        center_gap: Length,
        body_thickness: Length,
        slices_per_half: u16,
        hand: GearHand,
    ) -> Result<Self, GearError> {
        if teeth < 4 {
            return Err(GearError::TooFewRackTeeth);
        }
        if center_gap.mm() >= face_width.mm() {
            return Err(GearError::CenterGapTooWide);
        }
        if body_thickness.mm() < system.normal_module.mm() * 2.0 {
            return Err(GearError::RackBodyTooThin);
        }
        if slices_per_half < 2 {
            return Err(GearError::TooFewSlices);
        }
        Ok(Self {
            system,
            teeth,
            face_width,
            center_gap,
            body_thickness,
            slices_per_half,
            hand,
        })
    }

    pub const fn system(&self) -> &NormalGearSystem {
        &self.system
    }

    pub const fn teeth(&self) -> u16 {
        self.teeth
    }

    pub const fn face_width(&self) -> Length {
        self.face_width
    }

    pub const fn center_gap(&self) -> Length {
        self.center_gap
    }

    pub const fn body_thickness(&self) -> Length {
        self.body_thickness
    }

    pub const fn slices_per_half(&self) -> u16 {
        self.slices_per_half
    }

    pub const fn hand(&self) -> GearHand {
        self.hand
    }

    pub fn length(&self) -> f64 {
        f64::from(self.teeth) * self.system.transverse_pitch()
    }

    pub fn pitch_line_offset(&self) -> f64 {
        self.body_thickness.mm() * 0.5 + 1.25 * self.system.normal_module.mm()
    }

    pub fn tooth_band_width(&self) -> f64 {
        (self.face_width.mm() - self.center_gap.mm()) * 0.5
    }

    pub fn half_shift_mm(&self) -> f64 {
        self.tooth_band_width() * libm::tan(self.system.helix_angle.as_radians())
    }

    pub const fn lower_shift_sign(&self) -> f64 {
        match self.hand {
            GearHand::LeftAtLowerFace => 1.0,
            GearHand::RightAtLowerFace => -1.0,
        }
    }

    pub fn profile(&self) -> GearProfile {
        let module = self.system.normal_module.mm();
        let pitch = self.system.transverse_pitch();
        let pressure = self.system.transverse_pressure_angle();
        let pitch_half_thickness = (pitch * 0.5 - self.system.backlash.mm()) * 0.5;
        let addendum = module;
        let dedendum = 1.25 * module;
        let root_half_width = pitch_half_thickness + dedendum * libm::tan(pressure);
        let tip_half_width = pitch_half_thickness - addendum * libm::tan(pressure);
        let root_y = self.body_thickness.mm() * 0.5;
        let tip_y = root_y + addendum + dedendum;
        let half_length = self.length() * 0.5;
        let mut top = Vec::with_capacity(usize::from(self.teeth) * 4 + 2);
        top.push(Point2 {
            x: -half_length,
            y: root_y,
        });
        for tooth in 0..self.teeth {
            let center = -half_length + (f64::from(tooth) + 0.5) * pitch;
            top.push(Point2 {
                x: center - root_half_width,
                y: root_y,
            });
            top.push(Point2 {
                x: center - tip_half_width,
                y: tip_y,
            });
            top.push(Point2 {
                x: center + tip_half_width,
                y: tip_y,
            });
            top.push(Point2 {
                x: center + root_half_width,
                y: root_y,
            });
        }
        top.push(Point2 {
            x: half_length,
            y: root_y,
        });

        let mut points = Vec::with_capacity(top.len() * 2);
        points.extend(top.iter().map(|point| Point2 {
            x: point.x,
            y: -point.y,
        }));
        points.extend(top.into_iter().rev());
        GearProfile { points }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn system() -> NormalGearSystem {
        NormalGearSystem::new(
            Length::positive_mm(2.0).unwrap(),
            Angle::degrees(20.0).unwrap(),
            Angle::degrees(15.0).unwrap(),
            Length::non_negative_mm(0.10).unwrap(),
            Length::positive_mm(0.05).unwrap(),
        )
        .unwrap()
    }

    #[test]
    fn converts_normal_to_transverse_system() {
        let system = system();
        assert!((system.transverse_module() - 2.070552360820166).abs() < 1.0e-12);
        assert!(
            (system.transverse_pressure_angle().to_degrees() - 20.64689648704647).abs() < 1.0e-12
        );
        let pinion = system
            .pinion(
                20,
                Length::positive_mm(18.0).unwrap(),
                Length::positive_mm(2.0).unwrap(),
                Length::positive_mm(12.4).unwrap(),
                12,
                GearHand::LeftAtLowerFace,
            )
            .unwrap();
        assert!((pinion.spur().pitch_radius() - 20.70552360820166).abs() < 1.0e-12);
        assert!((pinion.spur().tip_radius() - 22.70552360820166).abs() < 1.0e-12);
    }

    #[test]
    fn double_sided_rack_has_expected_length_and_extents() {
        let rack = DoubleHelicalRack::new(
            system(),
            30,
            Length::positive_mm(18.0).unwrap(),
            Length::positive_mm(2.0).unwrap(),
            Length::positive_mm(8.0).unwrap(),
            12,
            GearHand::LeftAtLowerFace,
        )
        .unwrap();
        assert!((rack.length() - 195.144_962_568_769).abs() < 1.0e-9);
        let profile = rack.profile();
        let maximum_y = profile
            .points
            .iter()
            .map(|point| point.y)
            .fold(f64::NEG_INFINITY, f64::max);
        let minimum_y = profile
            .points
            .iter()
            .map(|point| point.y)
            .fold(f64::INFINITY, f64::min);
        assert!((maximum_y - 8.5).abs() < 1.0e-12);
        assert!((minimum_y + 8.5).abs() < 1.0e-12);
    }
}

// SPDX-License-Identifier: MIT

use alloc::vec::Vec;
use core::f64::consts::{PI, TAU};
use core::fmt;

use crate::{Angle, Length};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point2 {
    pub x: f64,
    pub y: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct GearProfile {
    pub points: Vec<Point2>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct TriangleMesh {
    pub vertices: Vec<[f64; 3]>,
    pub triangles: Vec<[u32; 3]>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SpurGear {
    transverse_module: Length,
    tooth_height_module: Length,
    teeth: u16,
    pressure_angle: Angle,
    backlash: Length,
    chord_tolerance: Length,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GearHand {
    LeftAtLowerFace,
    RightAtLowerFace,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DoubleHelicalGear {
    spur: SpurGear,
    face_width: Length,
    center_gap: Length,
    helix_angle: Angle,
    bore_diameter: Length,
    slices_per_half: u16,
    hand: GearHand,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DoubleHelicalGearPair {
    driver: DoubleHelicalGear,
    driven: DoubleHelicalGear,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GearPose {
    pub translation_mm: [f64; 3],
    pub rotation_z_deg: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GearError {
    TooFewTeeth,
    PressureAngleOutOfRange,
    ExcessiveBacklash,
    StandardProfileWouldUndercut,
    HelixAngleOutOfRange,
    CenterGapTooWide,
    BoreTooLarge,
    TooFewSlices,
    IncompatibleModule,
    IncompatiblePressureAngle,
    IncompatibleFaceWidth,
    IncompatibleCenterGap,
    IncompatibleHelixAngle,
    SameHandedPair,
    TooFewRackTeeth,
    RackBodyTooThin,
}

impl SpurGear {
    pub fn new(
        module: Length,
        teeth: u16,
        pressure_angle: Angle,
        backlash: Length,
        chord_tolerance: Length,
    ) -> Result<Self, GearError> {
        Self::with_tooth_height_module(
            module,
            module,
            teeth,
            pressure_angle,
            backlash,
            chord_tolerance,
        )
    }

    pub fn with_tooth_height_module(
        transverse_module: Length,
        tooth_height_module: Length,
        teeth: u16,
        pressure_angle: Angle,
        backlash: Length,
        chord_tolerance: Length,
    ) -> Result<Self, GearError> {
        if teeth < 3 {
            return Err(GearError::TooFewTeeth);
        }
        let pressure_degrees = pressure_angle.as_degrees();
        if !(10.0..=35.0).contains(&pressure_degrees) {
            return Err(GearError::PressureAngleOutOfRange);
        }
        if backlash.mm() >= PI * transverse_module.mm() * 0.45 {
            return Err(GearError::ExcessiveBacklash);
        }
        let sin_pressure = libm::sin(pressure_angle.as_radians());
        let minimum_teeth = libm::ceil(2.0 / (sin_pressure * sin_pressure)) as u16;
        if teeth < minimum_teeth {
            return Err(GearError::StandardProfileWouldUndercut);
        }
        Ok(Self {
            transverse_module,
            tooth_height_module,
            teeth,
            pressure_angle,
            backlash,
            chord_tolerance,
        })
    }

    pub const fn teeth(&self) -> u16 {
        self.teeth
    }

    pub const fn module(&self) -> Length {
        self.transverse_module
    }

    pub const fn tooth_height_module(&self) -> Length {
        self.tooth_height_module
    }

    pub const fn pressure_angle(&self) -> Angle {
        self.pressure_angle
    }

    pub const fn backlash(&self) -> Length {
        self.backlash
    }

    pub fn pitch_radius(&self) -> f64 {
        self.transverse_module.mm() * f64::from(self.teeth) * 0.5
    }

    pub fn base_radius(&self) -> f64 {
        self.pitch_radius() * libm::cos(self.pressure_angle.as_radians())
    }

    pub fn tip_radius(&self) -> f64 {
        self.pitch_radius() + self.tooth_height_module.mm()
    }

    pub fn root_radius(&self) -> f64 {
        self.pitch_radius() - 1.25 * self.tooth_height_module.mm()
    }

    pub fn outside_diameter(&self) -> f64 {
        self.tip_radius() * 2.0
    }

    pub fn profile(&self) -> GearProfile {
        let pitch_radius = self.pitch_radius();
        let base_radius = self.base_radius();
        let tip_radius = self.tip_radius();
        let root_radius = self.root_radius();
        let tolerance = self.chord_tolerance.mm();
        let pitch_angle = TAU / f64::from(self.teeth);
        let half_tooth_angle =
            PI / (2.0 * f64::from(self.teeth)) - self.backlash.mm() / (2.0 * pitch_radius);
        let pitch_t = involute_parameter(base_radius, pitch_radius);
        let pitch_involute = involute_polar_angle(pitch_t);
        let flank_start_radius = root_radius.max(base_radius);
        let flank_start_t = involute_parameter(base_radius, flank_start_radius);
        let tip_t = involute_parameter(base_radius, tip_radius);
        let base_offset = half_tooth_angle + pitch_involute;
        let mut points = Vec::new();

        for tooth in 0..self.teeth {
            let center = f64::from(tooth) * pitch_angle;
            let right_root_angle = center - base_offset + involute_polar_angle(flank_start_t);
            push_unique(&mut points, polar(root_radius, right_root_angle));
            if flank_start_radius > root_radius + 1.0e-12 {
                push_unique(&mut points, polar(flank_start_radius, right_root_angle));
            }

            let mut right_flank = Vec::new();
            adaptive_involute(
                base_radius,
                base_offset,
                center,
                -1.0,
                flank_start_t,
                tip_t,
                tolerance,
                0,
                &mut right_flank,
            );
            for point in right_flank.into_iter().skip(1) {
                push_unique(&mut points, point);
            }

            let right_tip_angle = center - base_offset + involute_polar_angle(tip_t);
            let left_tip_angle = center + base_offset - involute_polar_angle(tip_t);
            append_arc(
                &mut points,
                tip_radius,
                right_tip_angle,
                left_tip_angle,
                tolerance,
                true,
            );

            let mut left_flank = Vec::new();
            adaptive_involute(
                base_radius,
                base_offset,
                center,
                1.0,
                flank_start_t,
                tip_t,
                tolerance,
                0,
                &mut left_flank,
            );
            for point in left_flank.into_iter().rev().skip(1) {
                push_unique(&mut points, point);
            }

            let left_root_angle = center + base_offset - involute_polar_angle(flank_start_t);
            if flank_start_radius > root_radius + 1.0e-12 {
                push_unique(&mut points, polar(root_radius, left_root_angle));
            }
            let next_right_root =
                center + pitch_angle - base_offset + involute_polar_angle(flank_start_t);
            append_arc(
                &mut points,
                root_radius,
                left_root_angle,
                next_right_root,
                tolerance,
                false,
            );
        }

        if points.len() > 1 && squared_distance(points[0], *points.last().unwrap()) < 1.0e-20 {
            points.pop();
        }
        GearProfile { points }
    }
}

impl DoubleHelicalGear {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        spur: SpurGear,
        face_width: Length,
        center_gap: Length,
        helix_angle: Angle,
        bore_diameter: Length,
        slices_per_half: u16,
        hand: GearHand,
    ) -> Result<Self, GearError> {
        let helix_degrees = helix_angle.as_degrees().abs();
        if !(5.0..=45.0).contains(&helix_degrees) {
            return Err(GearError::HelixAngleOutOfRange);
        }
        if center_gap.mm() >= face_width.mm() {
            return Err(GearError::CenterGapTooWide);
        }
        if bore_diameter.mm() * 0.5 >= spur.root_radius() {
            return Err(GearError::BoreTooLarge);
        }
        if slices_per_half < 2 {
            return Err(GearError::TooFewSlices);
        }
        Ok(Self {
            spur,
            face_width,
            center_gap,
            helix_angle: Angle::radians(helix_angle.as_radians().abs())
                .expect("validated finite angle"),
            bore_diameter,
            slices_per_half,
            hand,
        })
    }

    pub const fn spur(&self) -> &SpurGear {
        &self.spur
    }

    pub const fn face_width(&self) -> Length {
        self.face_width
    }

    pub const fn center_gap(&self) -> Length {
        self.center_gap
    }

    pub const fn helix_angle(&self) -> Angle {
        self.helix_angle
    }

    pub const fn bore_diameter(&self) -> Length {
        self.bore_diameter
    }

    pub const fn slices_per_half(&self) -> u16 {
        self.slices_per_half
    }

    pub const fn hand(&self) -> GearHand {
        self.hand
    }

    pub fn tooth_band_width(&self) -> f64 {
        (self.face_width.mm() - self.center_gap.mm()) * 0.5
    }

    pub fn half_twist_degrees(&self) -> f64 {
        let radians = self.tooth_band_width() * libm::tan(self.helix_angle.as_radians())
            / self.spur.pitch_radius();
        radians.to_degrees()
    }

    pub const fn lower_twist_sign(&self) -> f64 {
        match self.hand {
            GearHand::LeftAtLowerFace => 1.0,
            GearHand::RightAtLowerFace => -1.0,
        }
    }
}

impl DoubleHelicalGearPair {
    pub fn new(driver: DoubleHelicalGear, driven: DoubleHelicalGear) -> Result<Self, GearError> {
        if (driver.spur.module().mm() - driven.spur.module().mm()).abs() > 1.0e-12 {
            return Err(GearError::IncompatibleModule);
        }
        if (driver.spur.pressure_angle.as_radians() - driven.spur.pressure_angle.as_radians()).abs()
            > 1.0e-12
        {
            return Err(GearError::IncompatiblePressureAngle);
        }
        if (driver.face_width.mm() - driven.face_width.mm()).abs() > 1.0e-12 {
            return Err(GearError::IncompatibleFaceWidth);
        }
        if (driver.center_gap.mm() - driven.center_gap.mm()).abs() > 1.0e-12 {
            return Err(GearError::IncompatibleCenterGap);
        }
        if (driver.helix_angle.as_radians() - driven.helix_angle.as_radians()).abs() > 1.0e-12 {
            return Err(GearError::IncompatibleHelixAngle);
        }
        if driver.hand == driven.hand {
            return Err(GearError::SameHandedPair);
        }
        Ok(Self { driver, driven })
    }

    pub const fn driver(&self) -> &DoubleHelicalGear {
        &self.driver
    }

    pub const fn driven(&self) -> &DoubleHelicalGear {
        &self.driven
    }

    pub fn ratio(&self) -> f64 {
        f64::from(self.driven.spur.teeth) / f64::from(self.driver.spur.teeth)
    }

    pub fn center_distance(&self) -> f64 {
        self.driver.spur.pitch_radius() + self.driven.spur.pitch_radius()
    }

    pub const fn driver_pose(&self) -> GearPose {
        GearPose {
            translation_mm: [0.0, 0.0, 0.0],
            rotation_z_deg: -90.0,
        }
    }

    pub fn driven_pose(&self) -> GearPose {
        GearPose {
            translation_mm: [self.center_distance(), 0.0, 0.0],
            rotation_z_deg: 90.0 - 180.0 / f64::from(self.driven.spur.teeth),
        }
    }
}

fn involute_parameter(base_radius: f64, radius: f64) -> f64 {
    libm::sqrt((radius * radius) / (base_radius * base_radius) - 1.0)
}

fn involute_polar_angle(parameter: f64) -> f64 {
    parameter - libm::atan(parameter)
}

fn involute_point(
    base_radius: f64,
    base_offset: f64,
    center: f64,
    side: f64,
    parameter: f64,
) -> Point2 {
    let radius = base_radius * libm::sqrt(1.0 + parameter * parameter);
    let angle = center + side * (base_offset - involute_polar_angle(parameter));
    polar(radius, angle)
}

#[allow(clippy::too_many_arguments)]
fn adaptive_involute(
    base_radius: f64,
    base_offset: f64,
    center: f64,
    side: f64,
    start: f64,
    end: f64,
    tolerance: f64,
    depth: u8,
    output: &mut Vec<Point2>,
) {
    let p0 = involute_point(base_radius, base_offset, center, side, start);
    let p1 = involute_point(base_radius, base_offset, center, side, end);
    if output.is_empty() {
        output.push(p0);
    }
    let mid = (start + end) * 0.5;
    let pm = involute_point(base_radius, base_offset, center, side, mid);
    if depth < 14 && distance_to_segment(pm, p0, p1) > tolerance {
        adaptive_involute(
            base_radius,
            base_offset,
            center,
            side,
            start,
            mid,
            tolerance,
            depth + 1,
            output,
        );
        adaptive_involute(
            base_radius,
            base_offset,
            center,
            side,
            mid,
            end,
            tolerance,
            depth + 1,
            output,
        );
    } else {
        push_unique(output, p1);
    }
}

fn append_arc(
    points: &mut Vec<Point2>,
    radius: f64,
    start: f64,
    end: f64,
    tolerance: f64,
    include_end: bool,
) {
    let delta = (end - start).max(0.0);
    let max_angle = if tolerance >= radius {
        PI
    } else {
        2.0 * libm::acos(1.0 - tolerance / radius)
    };
    let segment_count = (libm::ceil(delta / max_angle) as usize).max(1);
    let stop = if include_end {
        segment_count
    } else {
        segment_count.saturating_sub(1)
    };
    for index in 1..=stop {
        push_unique(
            points,
            polar(radius, start + delta * index as f64 / segment_count as f64),
        );
    }
}

fn polar(radius: f64, angle: f64) -> Point2 {
    Point2 {
        x: radius * libm::cos(angle),
        y: radius * libm::sin(angle),
    }
}

fn push_unique(points: &mut Vec<Point2>, point: Point2) {
    if points
        .last()
        .is_none_or(|last| squared_distance(*last, point) > 1.0e-20)
    {
        points.push(point);
    }
}

fn squared_distance(a: Point2, b: Point2) -> f64 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    dx * dx + dy * dy
}

fn distance_to_segment(point: Point2, start: Point2, end: Point2) -> f64 {
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let length_squared = dx * dx + dy * dy;
    if length_squared <= 1.0e-24 {
        return libm::sqrt(squared_distance(point, start));
    }
    let projection =
        (((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared).clamp(0.0, 1.0);
    let nearest = Point2 {
        x: start.x + projection * dx,
        y: start.y + projection * dy,
    };
    libm::sqrt(squared_distance(point, nearest))
}

impl fmt::Display for GearError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooFewTeeth => formatter.write_str("gear must have at least three teeth"),
            Self::PressureAngleOutOfRange => {
                formatter.write_str("pressure angle must be between 10 and 35 degrees")
            }
            Self::ExcessiveBacklash => formatter.write_str("backlash removes too much tooth width"),
            Self::StandardProfileWouldUndercut => formatter.write_str(
                "unshifted full-depth involute tooth count would undercut at this pressure angle",
            ),
            Self::HelixAngleOutOfRange => {
                formatter.write_str("helix angle must be between 5 and 45 degrees")
            }
            Self::CenterGapTooWide => {
                formatter.write_str("center gap must be narrower than the face width")
            }
            Self::BoreTooLarge => formatter.write_str("bore must fit inside the gear root circle"),
            Self::TooFewSlices => {
                formatter.write_str("each helical half needs at least two slices")
            }
            Self::IncompatibleModule => {
                formatter.write_str("meshing gears must use the same transverse module")
            }
            Self::IncompatiblePressureAngle => {
                formatter.write_str("meshing gears must use the same transverse pressure angle")
            }
            Self::IncompatibleFaceWidth => {
                formatter.write_str("meshing gears must use the same face width")
            }
            Self::IncompatibleCenterGap => {
                formatter.write_str("meshing gears must use the same center gap")
            }
            Self::IncompatibleHelixAngle => {
                formatter.write_str("meshing gears must use the same helix angle magnitude")
            }
            Self::SameHandedPair => {
                formatter.write_str("meshing gears must use opposite lower-face helix hands")
            }
            Self::TooFewRackTeeth => formatter.write_str("rack must contain at least four teeth"),
            Self::RackBodyTooThin => {
                formatter.write_str("rack body must be at least two normal modules thick")
            }
        }
    }
}

impl core::error::Error for GearError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn spur(teeth: u16) -> SpurGear {
        SpurGear::new(
            Length::positive_mm(1.5).unwrap(),
            teeth,
            Angle::degrees(20.0).unwrap(),
            Length::non_negative_mm(0.08).unwrap(),
            Length::positive_mm(0.04).unwrap(),
        )
        .unwrap()
    }

    fn double_gear(teeth: u16, hand: GearHand) -> DoubleHelicalGear {
        DoubleHelicalGear::new(
            spur(teeth),
            Length::positive_mm(14.0).unwrap(),
            Length::positive_mm(2.0).unwrap(),
            Angle::degrees(25.0).unwrap(),
            Length::positive_mm(5.0).unwrap(),
            12,
            hand,
        )
        .unwrap()
    }

    #[test]
    fn profile_stays_between_root_and_tip() {
        let gear = spur(18);
        let profile = gear.profile();
        assert!(profile.points.len() > usize::from(gear.teeth()) * 6);
        for point in profile.points {
            let radius = libm::sqrt(point.x * point.x + point.y * point.y);
            assert!(radius >= gear.root_radius() - 1.0e-9);
            assert!(radius <= gear.tip_radius() + 1.0e-9);
        }
    }

    #[test]
    fn computes_twist_from_transverse_pitch_radius() {
        let gear = double_gear(18, GearHand::LeftAtLowerFace);
        let expected = (6.0 * libm::tan(25.0_f64.to_radians()) / 13.5).to_degrees();
        assert!((gear.half_twist_degrees() - expected).abs() < 1.0e-12);
    }

    #[test]
    fn pair_requires_opposite_hands_and_reports_placement() {
        let driver = double_gear(18, GearHand::LeftAtLowerFace);
        let driven = double_gear(36, GearHand::RightAtLowerFace);
        let pair = DoubleHelicalGearPair::new(driver.clone(), driven).unwrap();
        assert!((pair.ratio() - 2.0).abs() < 1.0e-12);
        assert!((pair.center_distance() - 40.5).abs() < 1.0e-12);
        assert!((pair.driven_pose().translation_mm[0] - 40.5).abs() < 1.0e-12);
        assert_eq!(
            DoubleHelicalGearPair::new(driver.clone(), driver),
            Err(GearError::SameHandedPair)
        );
    }

    #[test]
    fn rejects_gap_that_consumes_both_tooth_bands() {
        let result = DoubleHelicalGear::new(
            spur(18),
            Length::positive_mm(10.0).unwrap(),
            Length::positive_mm(10.0).unwrap(),
            Angle::degrees(25.0).unwrap(),
            Length::positive_mm(5.0).unwrap(),
            12,
            GearHand::LeftAtLowerFace,
        );
        assert_eq!(result, Err(GearError::CenterGapTooWide));
    }
}

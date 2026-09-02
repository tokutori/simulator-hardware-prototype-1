// SPDX-License-Identifier: MIT

use alloc::vec::Vec;
use core::f64::consts::{PI, TAU};
use core::fmt;

use crate::geometry::Point2;
use crate::{Angle, Length};

#[derive(Clone, Debug, PartialEq)]
pub struct GearProfile {
    pub points: Vec<Point2>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SpurGear {
    module: Length,
    teeth: u16,
    pressure_angle: Angle,
    backlash: Length,
    chord_tolerance: Length,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ExternalGearPair {
    driver: SpurGear,
    driven: SpurGear,
}

#[derive(Clone, Debug, PartialEq)]
pub struct InternalGear {
    module: Length,
    teeth: u16,
    pressure_angle: Angle,
    backlash: Length,
    chord_tolerance: Length,
}

#[derive(Clone, Debug, PartialEq)]
pub struct InternalGearPair {
    pinion: SpurGear,
    ring: InternalGear,
}

/// A manufactured arc cut from the reference geometry of a dual-tooth ring.
/// Reference tooth counts define pitch radii; they are not physical sector counts.
#[derive(Clone, Debug, PartialEq)]
pub struct GearSector {
    external_reference: SpurGear,
    internal_reference: InternalGear,
    half_angle: Angle,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GearError {
    TooFewTeeth,
    PressureAngleOutOfRange,
    ExcessiveBacklash,
    StandardProfileWouldUndercut,
    IncompatibleModule,
    IncompatiblePressureAngle,
    InternalGearTooSmall,
    InvalidSectorAngle,
}

impl GearSector {
    pub fn new(
        external_reference: SpurGear,
        internal_reference: InternalGear,
        half_angle: Angle,
    ) -> Result<Self, GearError> {
        if external_reference.module() != internal_reference.module()
            || external_reference.pressure_angle() != internal_reference.pressure_angle()
        {
            return Err(GearError::IncompatibleModule);
        }
        if half_angle.as_radians() <= 0.0 || half_angle.as_degrees() >= 90.0 {
            return Err(GearError::InvalidSectorAngle);
        }
        Ok(Self {
            external_reference,
            internal_reference,
            half_angle,
        })
    }

    pub const fn external_reference(&self) -> &SpurGear {
        &self.external_reference
    }

    pub const fn internal_reference(&self) -> &InternalGear {
        &self.internal_reference
    }

    pub const fn half_angle(&self) -> Angle {
        self.half_angle
    }

    pub fn approximate_external_tooth_count(&self) -> u16 {
        libm::ceil(f64::from(self.external_reference.teeth()) * self.half_angle.as_radians() / PI)
            as u16
    }

    pub fn supports_motion(&self, motion_limit: Angle, angular_margin: Angle) -> bool {
        motion_limit.as_radians().abs() + angular_margin.as_radians().abs()
            <= self.half_angle.as_radians()
    }
}

impl SpurGear {
    pub fn new(
        module: Length,
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
        let circular_pitch = PI * module.mm();
        if backlash.mm() >= circular_pitch * 0.45 {
            return Err(GearError::ExcessiveBacklash);
        }

        // Unshifted full-depth involute teeth below this bound are expected to
        // undercut. This prototype rejects that state rather than emitting a
        // deceptively printable but mechanically weak tooth form.
        let sin_pressure = libm::sin(pressure_angle.as_radians());
        let minimum_teeth = libm::ceil(2.0 / (sin_pressure * sin_pressure)) as u16;
        if teeth < minimum_teeth {
            return Err(GearError::StandardProfileWouldUndercut);
        }

        Ok(Self {
            module,
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
        self.module
    }

    pub const fn pressure_angle(&self) -> Angle {
        self.pressure_angle
    }

    pub fn pitch_radius(&self) -> f64 {
        self.module.mm() * f64::from(self.teeth) * 0.5
    }

    pub fn base_radius(&self) -> f64 {
        self.pitch_radius() * libm::cos(self.pressure_angle.as_radians())
    }

    pub fn tip_radius(&self) -> f64 {
        self.pitch_radius() + self.module.mm()
    }

    pub fn root_radius(&self) -> f64 {
        self.pitch_radius() - 1.25 * self.module.mm()
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

impl ExternalGearPair {
    pub fn new(driver: SpurGear, driven: SpurGear) -> Result<Self, GearError> {
        if (driver.module.mm() - driven.module.mm()).abs() > 1.0e-12 {
            return Err(GearError::IncompatibleModule);
        }
        if (driver.pressure_angle.as_radians() - driven.pressure_angle.as_radians()).abs() > 1.0e-12
        {
            return Err(GearError::IncompatiblePressureAngle);
        }
        Ok(Self { driver, driven })
    }

    pub fn ratio(&self) -> f64 {
        f64::from(self.driven.teeth) / f64::from(self.driver.teeth)
    }

    pub fn center_distance(&self) -> f64 {
        self.driver.pitch_radius() + self.driven.pitch_radius()
    }

    pub const fn driver(&self) -> &SpurGear {
        &self.driver
    }

    pub const fn driven(&self) -> &SpurGear {
        &self.driven
    }
}

impl InternalGear {
    pub fn new(
        module: Length,
        teeth: u16,
        pressure_angle: Angle,
        backlash: Length,
        chord_tolerance: Length,
    ) -> Result<Self, GearError> {
        if teeth < 12 {
            return Err(GearError::TooFewTeeth);
        }
        let pressure_degrees = pressure_angle.as_degrees();
        if !(10.0..=35.0).contains(&pressure_degrees) {
            return Err(GearError::PressureAngleOutOfRange);
        }
        let circular_pitch = PI * module.mm();
        if backlash.mm() >= circular_pitch * 0.45 {
            return Err(GearError::ExcessiveBacklash);
        }
        let gear = Self {
            module,
            teeth,
            pressure_angle,
            backlash,
            chord_tolerance,
        };
        if gear.tip_radius() <= gear.base_radius() {
            return Err(GearError::InternalGearTooSmall);
        }
        Ok(gear)
    }

    pub const fn teeth(&self) -> u16 {
        self.teeth
    }

    pub const fn module(&self) -> Length {
        self.module
    }

    pub const fn pressure_angle(&self) -> Angle {
        self.pressure_angle
    }

    pub fn pitch_radius(&self) -> f64 {
        self.module.mm() * f64::from(self.teeth) * 0.5
    }

    pub fn base_radius(&self) -> f64 {
        self.pitch_radius() * libm::cos(self.pressure_angle.as_radians())
    }

    pub fn tip_radius(&self) -> f64 {
        self.pitch_radius() - self.module.mm()
    }

    pub fn root_radius(&self) -> f64 {
        self.pitch_radius() + 1.25 * self.module.mm()
    }

    /// Boundary of the central void. Extruding and subtracting this profile
    /// from an outer blank leaves inward-pointing internal involute teeth.
    pub fn void_profile(&self) -> GearProfile {
        let pitch_radius = self.pitch_radius();
        let base_radius = self.base_radius();
        let tip_radius = self.tip_radius();
        let root_radius = self.root_radius();
        let tolerance = self.chord_tolerance.mm();
        let tooth_pitch = TAU / f64::from(self.teeth);
        let half_tooth_angle =
            PI / (2.0 * f64::from(self.teeth)) - self.backlash.mm() / (2.0 * pitch_radius);
        let pitch_t = involute_parameter(base_radius, pitch_radius);
        let tip_t = involute_parameter(base_radius, tip_radius);
        let root_t = involute_parameter(base_radius, root_radius);
        let pitch_involute = involute_polar_angle(pitch_t);
        let tip_half = half_tooth_angle + involute_polar_angle(tip_t) - pitch_involute;
        let root_half = half_tooth_angle + involute_polar_angle(root_t) - pitch_involute;
        let mut points = Vec::new();

        for tooth in 0..self.teeth {
            let center = f64::from(tooth) * tooth_pitch;
            let negative_tip = center - tip_half;
            push_unique(&mut points, polar(tip_radius, negative_tip));
            append_arc(
                &mut points,
                tip_radius,
                negative_tip,
                center + tip_half,
                tolerance,
                true,
            );

            let mut positive_flank = Vec::new();
            adaptive_internal_involute(
                base_radius,
                half_tooth_angle,
                pitch_involute,
                center,
                1.0,
                tip_t,
                root_t,
                tolerance,
                0,
                &mut positive_flank,
            );
            for point in positive_flank.into_iter().skip(1) {
                push_unique(&mut points, point);
            }

            let next_center = center + tooth_pitch;
            append_arc(
                &mut points,
                root_radius,
                center + root_half,
                next_center - root_half,
                tolerance,
                true,
            );

            let mut negative_flank = Vec::new();
            adaptive_internal_involute(
                base_radius,
                half_tooth_angle,
                pitch_involute,
                next_center,
                -1.0,
                tip_t,
                root_t,
                tolerance,
                0,
                &mut negative_flank,
            );
            for point in negative_flank.into_iter().rev().skip(1) {
                push_unique(&mut points, point);
            }
        }

        if points.len() > 1 && squared_distance(points[0], *points.last().unwrap()) < 1.0e-20 {
            points.pop();
        }
        GearProfile { points }
    }
}

impl InternalGearPair {
    pub fn new(pinion: SpurGear, ring: InternalGear) -> Result<Self, GearError> {
        if (pinion.module.mm() - ring.module.mm()).abs() > 1.0e-12 {
            return Err(GearError::IncompatibleModule);
        }
        if (pinion.pressure_angle.as_radians() - ring.pressure_angle.as_radians()).abs() > 1.0e-12 {
            return Err(GearError::IncompatiblePressureAngle);
        }
        if ring.teeth <= pinion.teeth + 2 {
            return Err(GearError::InternalGearTooSmall);
        }
        Ok(Self { pinion, ring })
    }

    pub fn ratio(&self) -> f64 {
        f64::from(self.ring.teeth) / f64::from(self.pinion.teeth)
    }

    pub fn center_distance(&self) -> f64 {
        self.ring.pitch_radius() - self.pinion.pitch_radius()
    }

    pub const fn pinion(&self) -> &SpurGear {
        &self.pinion
    }

    pub const fn ring(&self) -> &InternalGear {
        &self.ring
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

fn internal_involute_point(
    base_radius: f64,
    half_tooth_angle: f64,
    pitch_involute: f64,
    center: f64,
    side: f64,
    parameter: f64,
) -> Point2 {
    let radius = base_radius * libm::sqrt(1.0 + parameter * parameter);
    let half_angle = half_tooth_angle + involute_polar_angle(parameter) - pitch_involute;
    polar(radius, center + side * half_angle)
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

#[allow(clippy::too_many_arguments)]
fn adaptive_internal_involute(
    base_radius: f64,
    half_tooth_angle: f64,
    pitch_involute: f64,
    center: f64,
    side: f64,
    start: f64,
    end: f64,
    tolerance: f64,
    depth: u8,
    output: &mut Vec<Point2>,
) {
    let p0 = internal_involute_point(
        base_radius,
        half_tooth_angle,
        pitch_involute,
        center,
        side,
        start,
    );
    let p1 = internal_involute_point(
        base_radius,
        half_tooth_angle,
        pitch_involute,
        center,
        side,
        end,
    );
    if output.is_empty() {
        output.push(p0);
    }
    let mid = (start + end) * 0.5;
    let pm = internal_involute_point(
        base_radius,
        half_tooth_angle,
        pitch_involute,
        center,
        side,
        mid,
    );
    if depth < 14 && distance_to_segment(pm, p0, p1) > tolerance {
        adaptive_internal_involute(
            base_radius,
            half_tooth_angle,
            pitch_involute,
            center,
            side,
            start,
            mid,
            tolerance,
            depth + 1,
            output,
        );
        adaptive_internal_involute(
            base_radius,
            half_tooth_angle,
            pitch_involute,
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
        let angle = start + delta * index as f64 / segment_count as f64;
        push_unique(points, polar(radius, angle));
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
            Self::IncompatibleModule => {
                formatter.write_str("meshing gears must use the same module")
            }
            Self::IncompatiblePressureAngle => {
                formatter.write_str("meshing gears must use the same pressure angle")
            }
            Self::InternalGearTooSmall => formatter
                .write_str("internal ring gear is too small for its base circle or mating pinion"),
            Self::InvalidSectorAngle => formatter.write_str(
                "gear sector half-angle must be greater than 0 and less than 90 degrees",
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn standard_gear(teeth: u16) -> SpurGear {
        SpurGear::new(
            Length::positive_mm(1.2).unwrap(),
            teeth,
            Angle::degrees(20.0).unwrap(),
            Length::non_negative_mm(0.15).unwrap(),
            Length::positive_mm(0.04).unwrap(),
        )
        .unwrap()
    }

    #[test]
    fn standard_diameters_follow_module_definition() {
        let gear = standard_gear(72);
        assert!((gear.pitch_radius() - 43.2).abs() < 1.0e-12);
        assert!((gear.outside_diameter() - 88.8).abs() < 1.0e-12);
        assert!((gear.root_radius() - 41.7).abs() < 1.0e-12);
    }

    #[test]
    fn profile_is_closed_by_contract_without_duplicate_endpoint() {
        let gear = standard_gear(18);
        let profile = gear.profile();
        assert!(profile.points.len() > usize::from(gear.teeth()) * 6);
        assert!(squared_distance(profile.points[0], *profile.points.last().unwrap()) > 1.0e-20);
        for point in profile.points {
            let radius = libm::sqrt(point.x * point.x + point.y * point.y);
            assert!(radius >= gear.root_radius() - 1.0e-9);
            assert!(radius <= gear.tip_radius() + 1.0e-9);
        }
    }

    #[test]
    fn rejects_unshifted_pinion_that_would_undercut() {
        let result = SpurGear::new(
            Length::positive_mm(1.0).unwrap(),
            12,
            Angle::degrees(20.0).unwrap(),
            Length::non_negative_mm(0.1).unwrap(),
            Length::positive_mm(0.02).unwrap(),
        );
        assert_eq!(result, Err(GearError::StandardProfileWouldUndercut));
    }

    #[test]
    fn gear_pair_reports_ratio_and_standard_center_distance() {
        let pair = ExternalGearPair::new(standard_gear(18), standard_gear(72)).unwrap();
        assert!((pair.ratio() - 4.0).abs() < 1.0e-12);
        assert!((pair.center_distance() - 54.0).abs() < 1.0e-12);
    }

    #[test]
    fn internal_profile_has_inward_tips_and_outward_roots() {
        let ring = InternalGear::new(
            Length::positive_mm(2.5).unwrap(),
            100,
            Angle::degrees(20.0).unwrap(),
            Length::non_negative_mm(0.1).unwrap(),
            Length::positive_mm(0.05).unwrap(),
        )
        .unwrap();
        let profile = ring.void_profile();
        let mut minimum = f64::MAX;
        let mut maximum: f64 = 0.0;
        for point in profile.points {
            let radius = libm::sqrt(point.x * point.x + point.y * point.y);
            minimum = minimum.min(radius);
            maximum = maximum.max(radius);
        }
        assert!((minimum - ring.tip_radius()).abs() < 1.0e-8);
        assert!((maximum - ring.root_radius()).abs() < 1.0e-8);
    }

    #[test]
    fn internal_pair_rotates_same_direction_at_ratio() {
        let pinion = SpurGear::new(
            Length::positive_mm(2.5).unwrap(),
            20,
            Angle::degrees(20.0).unwrap(),
            Length::non_negative_mm(0.1).unwrap(),
            Length::positive_mm(0.05).unwrap(),
        )
        .unwrap();
        let ring = InternalGear::new(
            Length::positive_mm(2.5).unwrap(),
            100,
            Angle::degrees(20.0).unwrap(),
            Length::non_negative_mm(0.1).unwrap(),
            Length::positive_mm(0.05).unwrap(),
        )
        .unwrap();
        let pair = InternalGearPair::new(pinion, ring).unwrap();
        assert!((pair.ratio() - 5.0).abs() < 1.0e-12);
        assert!((pair.center_distance() - 100.0).abs() < 1.0e-12);
    }
}

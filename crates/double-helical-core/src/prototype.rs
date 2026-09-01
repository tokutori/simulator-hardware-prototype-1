// SPDX-License-Identifier: MIT

use alloc::vec::Vec;
use core::fmt;

use crate::{DoubleHelicalGear, DoubleHelicalRack, GearPose, Length, SpurGear};

#[derive(Clone, Debug, PartialEq)]
pub struct Prototype {
    handle_spur: SpurGear,
    reduction_large_spur: SpurGear,
    reduction_small_spur: SpurGear,
    output_spur: SpurGear,
    driven_pinion: DoubleHelicalGear,
    idler_pinion: DoubleHelicalGear,
    rack: DoubleHelicalRack,
    spur_face_width: Length,
    output_spur_face_width: Length,
    pinion_lower_extension: Length,
    bolt_length: Length,
    journal_outer_diameter: Length,
    bolt_clearance_diameter: Length,
    thrust_spacer_outer_diameter: Length,
    nut_across_flats: Length,
    nut_thickness: Length,
    nut_pocket_depth: Length,
    plate_thickness: Length,
    plate_length: Length,
    plate_width: Length,
    plate_center_y: f64,
    corner_inset: Length,
    axial_clearance: Length,
    handle_crank_radius: Length,
    top_socket_depth: Length,
    top_socket_axial_clearance: Length,
    top_socket_diameter_clearance: Length,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrototypeError {
    IncompatibleSpurModule,
    IncompatibleSpurPressureAngle,
    ReductionSmallSpurMustHaveEvenTeeth,
    OutputPinionBoreDoesNotFitJournal,
    IdlerPinionBoreDoesNotFitJournal,
    BoltDoesNotFitJournal,
    NutDoesNotClearBolt,
    ThrustSpacerDoesNotClearAxle,
    ThrustSpacerHasNoRoom,
    NutPocketTooDeep,
    BoltTooShort,
    OutputSpurDoesNotSupportPinionExtension,
    InvalidOutputSpurFaceWidth,
    PlateTooSmall,
    InvalidPlateCenter,
    InvalidTopSocket,
    InvalidHandleSquareDrive,
}

impl Prototype {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        handle_spur: SpurGear,
        reduction_large_spur: SpurGear,
        reduction_small_spur: SpurGear,
        output_spur: SpurGear,
        driven_pinion: DoubleHelicalGear,
        idler_pinion: DoubleHelicalGear,
        rack: DoubleHelicalRack,
        spur_face_width: Length,
        output_spur_face_width: Length,
        pinion_lower_extension: Length,
        bolt_length: Length,
        journal_outer_diameter: Length,
        bolt_clearance_diameter: Length,
        thrust_spacer_outer_diameter: Length,
        nut_across_flats: Length,
        nut_thickness: Length,
        nut_pocket_depth: Length,
        plate_thickness: Length,
        plate_length: Length,
        plate_width: Length,
        plate_center_y: f64,
        corner_inset: Length,
        axial_clearance: Length,
        handle_crank_radius: Length,
        top_socket_depth: Length,
        top_socket_axial_clearance: Length,
        top_socket_diameter_clearance: Length,
    ) -> Result<Self, PrototypeError> {
        for gear in [&reduction_large_spur, &reduction_small_spur, &output_spur] {
            if (handle_spur.module().mm() - gear.module().mm()).abs() > 1.0e-12 {
                return Err(PrototypeError::IncompatibleSpurModule);
            }
            if (handle_spur.pressure_angle().as_radians() - gear.pressure_angle().as_radians())
                .abs()
                > 1.0e-12
            {
                return Err(PrototypeError::IncompatibleSpurPressureAngle);
            }
        }
        if !reduction_small_spur.teeth().is_multiple_of(2) {
            return Err(PrototypeError::ReductionSmallSpurMustHaveEvenTeeth);
        }
        if driven_pinion.bore_diameter().mm() <= journal_outer_diameter.mm() {
            return Err(PrototypeError::OutputPinionBoreDoesNotFitJournal);
        }
        if idler_pinion.bore_diameter().mm() <= journal_outer_diameter.mm() {
            return Err(PrototypeError::IdlerPinionBoreDoesNotFitJournal);
        }
        if output_spur.root_radius() + 1.0e-12 < driven_pinion.spur().tip_radius() {
            return Err(PrototypeError::OutputSpurDoesNotSupportPinionExtension);
        }
        if output_spur_face_width.mm() + 1.0e-12 < spur_face_width.mm()
            || output_spur_face_width.mm()
                > spur_face_width.mm() + pinion_lower_extension.mm() + 1.0e-12
        {
            return Err(PrototypeError::InvalidOutputSpurFaceWidth);
        }
        if bolt_clearance_diameter.mm() >= journal_outer_diameter.mm() {
            return Err(PrototypeError::BoltDoesNotFitJournal);
        }
        if nut_across_flats.mm() <= bolt_clearance_diameter.mm() {
            return Err(PrototypeError::NutDoesNotClearBolt);
        }
        if thrust_spacer_outer_diameter.mm() <= driven_pinion.bore_diameter().mm() {
            return Err(PrototypeError::ThrustSpacerDoesNotClearAxle);
        }
        if nut_pocket_depth.mm() >= plate_thickness.mm() {
            return Err(PrototypeError::NutPocketTooDeep);
        }
        if top_socket_depth.mm() >= plate_thickness.mm()
            || top_socket_axial_clearance.mm() >= top_socket_depth.mm()
        {
            return Err(PrototypeError::InvalidTopSocket);
        }
        if !plate_center_y.is_finite() {
            return Err(PrototypeError::InvalidPlateCenter);
        }
        let minimum_plate_length = corner_inset.mm() * 2.0 + journal_outer_diameter.mm();
        let minimum_plate_width = corner_inset.mm() * 2.0 + journal_outer_diameter.mm();
        if plate_length.mm() <= minimum_plate_length || plate_width.mm() <= minimum_plate_width {
            return Err(PrototypeError::PlateTooSmall);
        }
        let prototype = Self {
            handle_spur,
            reduction_large_spur,
            reduction_small_spur,
            output_spur,
            driven_pinion,
            idler_pinion,
            rack,
            spur_face_width,
            output_spur_face_width,
            pinion_lower_extension,
            bolt_length,
            journal_outer_diameter,
            bolt_clearance_diameter,
            thrust_spacer_outer_diameter,
            nut_across_flats,
            nut_thickness,
            nut_pocket_depth,
            plate_thickness,
            plate_length,
            plate_width,
            plate_center_y,
            corner_inset,
            axial_clearance,
            handle_crank_radius,
            top_socket_depth,
            top_socket_axial_clearance,
            top_socket_diameter_clearance,
        };
        if prototype.bolt_thread_engagement_mm() + 1.0e-12 < nut_thickness.mm() {
            return Err(PrototypeError::BoltTooShort);
        }
        let half_length = prototype.plate_length.mm() * 0.5;
        let min_y = prototype.plate_center_y - prototype.plate_width.mm() * 0.5;
        let max_y = prototype.plate_center_y + prototype.plate_width.mm() * 0.5;
        let lateral_extent =
            prototype.secondary_spur_center_distance() + prototype.output_spur.tip_radius();
        let handle_min_y =
            prototype.handle_spur_pose().translation_mm[1] - prototype.handle_spur.tip_radius();
        let idler_max_y =
            prototype.idler_pose().translation_mm[1] + prototype.idler_pinion.spur().tip_radius();
        if lateral_extent > half_length + 1.0e-12
            || handle_min_y < min_y - 1.0e-12
            || idler_max_y > max_y + 1.0e-12
        {
            return Err(PrototypeError::PlateTooSmall);
        }
        if [
            prototype.handle_upper_thrust_spacer_length(),
            prototype.reduction_upper_thrust_spacer_length(),
            prototype.driven_lower_thrust_spacer_length(),
            prototype.idler_lower_thrust_spacer_length(),
        ]
        .into_iter()
        .any(|length| length <= 0.0)
        {
            return Err(PrototypeError::ThrustSpacerHasNoRoom);
        }
        let gear_socket_diagonal = prototype.handle_gear_square_socket_size() * libm::sqrt(2.0);
        let crank_square_diagonal = prototype.handle_crank_square_shaft_size() * libm::sqrt(2.0);
        if prototype.handle_gear_square_socket_size() <= prototype.handle_gear_square_shaft_size()
            || prototype.handle_top_taper_lower_diameter()
                >= prototype.handle_gear_square_socket_size()
            || prototype.handle_bottom_taper_upper_diameter()
                <= prototype.handle_gear_square_socket_size()
            || gear_socket_diagonal >= prototype.handle_spur.root_radius() * 2.0
            || prototype.handle_crank_square_socket_size()
                <= prototype.handle_crank_square_shaft_size()
            || crank_square_diagonal >= prototype.handle_top_taper_upper_diameter()
        {
            return Err(PrototypeError::InvalidHandleSquareDrive);
        }
        Ok(prototype)
    }

    pub const fn handle_spur(&self) -> &SpurGear {
        &self.handle_spur
    }

    pub const fn reduction_large_spur(&self) -> &SpurGear {
        &self.reduction_large_spur
    }

    pub const fn reduction_small_spur(&self) -> &SpurGear {
        &self.reduction_small_spur
    }

    pub const fn output_spur(&self) -> &SpurGear {
        &self.output_spur
    }

    pub const fn driven_pinion(&self) -> &DoubleHelicalGear {
        &self.driven_pinion
    }

    pub const fn idler_pinion(&self) -> &DoubleHelicalGear {
        &self.idler_pinion
    }

    pub const fn rack(&self) -> &DoubleHelicalRack {
        &self.rack
    }

    pub const fn spur_face_width(&self) -> Length {
        self.spur_face_width
    }

    pub const fn output_spur_face_width(&self) -> Length {
        self.output_spur_face_width
    }

    pub const fn pinion_lower_extension(&self) -> Length {
        self.pinion_lower_extension
    }

    pub const fn journal_outer_diameter(&self) -> Length {
        self.journal_outer_diameter
    }

    pub const fn bolt_clearance_diameter(&self) -> Length {
        self.bolt_clearance_diameter
    }

    pub const fn thrust_spacer_outer_diameter(&self) -> Length {
        self.thrust_spacer_outer_diameter
    }

    pub const fn bolt_length(&self) -> Length {
        self.bolt_length
    }

    pub const fn nut_across_flats(&self) -> Length {
        self.nut_across_flats
    }

    pub const fn nut_thickness(&self) -> Length {
        self.nut_thickness
    }

    pub const fn nut_pocket_depth(&self) -> Length {
        self.nut_pocket_depth
    }

    pub const fn plate_thickness(&self) -> Length {
        self.plate_thickness
    }

    pub const fn plate_length(&self) -> Length {
        self.plate_length
    }

    pub const fn plate_width(&self) -> Length {
        self.plate_width
    }

    pub const fn plate_center_y(&self) -> f64 {
        self.plate_center_y
    }

    pub const fn handle_crank_radius(&self) -> Length {
        self.handle_crank_radius
    }

    pub const fn top_socket_depth(&self) -> Length {
        self.top_socket_depth
    }

    pub const fn top_socket_axial_clearance(&self) -> Length {
        self.top_socket_axial_clearance
    }

    pub const fn top_socket_diameter_clearance(&self) -> Length {
        self.top_socket_diameter_clearance
    }

    pub const fn axial_clearance(&self) -> Length {
        self.axial_clearance
    }

    pub fn primary_reduction_ratio(&self) -> f64 {
        f64::from(self.reduction_large_spur.teeth()) / f64::from(self.handle_spur.teeth())
    }

    pub fn secondary_reduction_ratio(&self) -> f64 {
        f64::from(self.output_spur.teeth()) / f64::from(self.reduction_small_spur.teeth())
    }

    pub fn reduction_ratio(&self) -> f64 {
        self.primary_reduction_ratio() * self.secondary_reduction_ratio()
    }

    pub fn primary_spur_center_distance(&self) -> f64 {
        self.handle_spur.pitch_radius() + self.reduction_large_spur.pitch_radius()
    }

    pub fn secondary_spur_center_distance(&self) -> f64 {
        self.reduction_small_spur.pitch_radius() + self.output_spur.pitch_radius()
    }

    fn driven_y(&self) -> f64 {
        -(self.rack.pitch_line_offset() + self.driven_pinion.spur().pitch_radius())
    }

    pub fn driven_b_pose(&self) -> GearPose {
        GearPose {
            translation_mm: [-self.secondary_spur_center_distance(), self.driven_y(), 0.0],
            rotation_z_deg: 0.0,
        }
    }

    pub fn driven_c_pose(&self) -> GearPose {
        GearPose {
            translation_mm: [self.secondary_spur_center_distance(), self.driven_y(), 0.0],
            rotation_z_deg: 180.0,
        }
    }

    fn rack_mesh_rotation_deg(&self, x_mm: f64) -> f64 {
        90.0 + (x_mm / self.driven_pinion.spur().pitch_radius()).to_degrees()
    }

    pub fn driven_b_pinion_pose(&self) -> GearPose {
        let pose = self.driven_b_pose();
        GearPose {
            rotation_z_deg: self.rack_mesh_rotation_deg(pose.translation_mm[0]),
            ..pose
        }
    }

    pub fn driven_c_pinion_pose(&self) -> GearPose {
        let pose = self.driven_c_pose();
        GearPose {
            rotation_z_deg: self.rack_mesh_rotation_deg(pose.translation_mm[0]),
            ..pose
        }
    }

    pub fn driven_b_internal_pinion_rotation_deg(&self) -> f64 {
        self.driven_b_pinion_pose().rotation_z_deg - self.driven_b_pose().rotation_z_deg
    }

    pub fn driven_c_internal_pinion_rotation_deg(&self) -> f64 {
        self.driven_c_pinion_pose().rotation_z_deg - self.driven_c_pose().rotation_z_deg
    }

    pub fn idler_pose(&self) -> GearPose {
        GearPose {
            translation_mm: [
                0.0,
                self.rack.pitch_line_offset() + self.idler_pinion.spur().pitch_radius(),
                0.0,
            ],
            rotation_z_deg: -90.0,
        }
    }

    pub fn output_b_spur_pose(&self) -> GearPose {
        let driven = self.driven_b_pose();
        GearPose {
            translation_mm: [
                driven.translation_mm[0],
                driven.translation_mm[1],
                self.secondary_spur_layer_center_z(),
            ],
            rotation_z_deg: -90.0,
        }
    }

    pub fn output_c_spur_pose(&self) -> GearPose {
        let driven = self.driven_c_pose();
        GearPose {
            translation_mm: [
                driven.translation_mm[0],
                driven.translation_mm[1],
                self.secondary_spur_layer_center_z(),
            ],
            rotation_z_deg: driven.rotation_z_deg,
        }
    }

    pub fn reduction_pose(&self) -> GearPose {
        GearPose {
            translation_mm: [0.0, self.driven_y(), 0.0],
            rotation_z_deg: 180.0 / f64::from(self.reduction_small_spur.teeth()),
        }
    }

    pub fn reduction_small_spur_pose(&self) -> GearPose {
        let reduction = self.reduction_pose();
        GearPose {
            translation_mm: [
                reduction.translation_mm[0],
                reduction.translation_mm[1],
                self.reduction_small_extended_center_z(),
            ],
            rotation_z_deg: reduction.rotation_z_deg,
        }
    }

    pub fn reduction_large_spur_pose(&self) -> GearPose {
        let reduction = self.reduction_pose();
        GearPose {
            translation_mm: [
                reduction.translation_mm[0],
                reduction.translation_mm[1],
                self.primary_spur_layer_center_z(),
            ],
            rotation_z_deg: reduction.rotation_z_deg,
        }
    }

    pub fn handle_spur_pose(&self) -> GearPose {
        let reduction = self.reduction_pose();
        let reduction_contact_turns = (-90.0 - reduction.rotation_z_deg)
            * f64::from(self.reduction_large_spur.teeth())
            / 360.0;
        let reduction_contact_phase =
            reduction_contact_turns - libm::floor(reduction_contact_turns);
        let unwrapped_handle_gap_phase = 0.5 - reduction_contact_phase;
        let handle_gap_phase = unwrapped_handle_gap_phase - libm::floor(unwrapped_handle_gap_phase);
        let handle_pitch_angle = 360.0 / f64::from(self.handle_spur.teeth());
        let unwrapped_handle_rotation = 90.0 - handle_gap_phase * handle_pitch_angle;
        let handle_rotation = unwrapped_handle_rotation
            - libm::floor(unwrapped_handle_rotation / handle_pitch_angle) * handle_pitch_angle;
        GearPose {
            translation_mm: [
                0.0,
                self.driven_y() - self.primary_spur_center_distance(),
                self.handle_spur_extended_center_z(),
            ],
            rotation_z_deg: handle_rotation,
        }
    }

    pub fn secondary_spur_layer_center_z(&self) -> f64 {
        self.secondary_spur_bottom_z() + self.output_spur_face_width.mm() * 0.5
    }

    pub fn secondary_spur_bottom_z(&self) -> f64 {
        -self.driven_pinion.face_width().mm() * 0.5
            - self.pinion_lower_extension.mm()
            - self.spur_face_width.mm()
    }

    pub fn reduction_small_extended_face_width(&self) -> f64 {
        self.spur_face_width.mm() + self.pinion_lower_extension.mm()
    }

    pub fn reduction_small_extended_center_z(&self) -> f64 {
        self.secondary_spur_bottom_z() + self.reduction_small_extended_face_width() * 0.5
    }

    pub fn handle_spur_extended_face_width(&self) -> f64 {
        self.spur_face_width.mm() + self.pinion_lower_extension.mm()
    }

    pub fn handle_spur_extended_center_z(&self) -> f64 {
        self.primary_spur_layer_center_z() + self.pinion_lower_extension.mm() * 0.5
    }

    pub fn output_spur_to_rack_axial_gap(&self) -> f64 {
        let output_spur_top =
            self.secondary_spur_layer_center_z() + self.output_spur_face_width.mm() * 0.5;
        -self.rack.face_width().mm() * 0.5 - output_spur_top
    }

    pub fn output_spur_to_pinion_axial_gap(&self) -> f64 {
        let output_spur_top =
            self.secondary_spur_layer_center_z() + self.output_spur_face_width.mm() * 0.5;
        -self.driven_pinion.face_width().mm() * 0.5 - output_spur_top
    }

    pub fn primary_spur_layer_center_z(&self) -> f64 {
        self.secondary_spur_bottom_z() - self.spur_face_width.mm() * 0.5
    }

    pub fn frame_inner_bottom_z(&self) -> f64 {
        self.primary_spur_layer_center_z()
            - self.spur_face_width.mm() * 0.5
            - self.axial_clearance.mm()
    }

    pub fn frame_inner_top_z(&self) -> f64 {
        self.driven_pinion.face_width().mm() * 0.5 + self.axial_clearance.mm()
    }

    pub fn bottom_plate_center_z(&self) -> f64 {
        self.frame_inner_bottom_z() - self.plate_thickness.mm() * 0.5
    }

    pub fn top_plate_center_z(&self) -> f64 {
        self.frame_inner_top_z() + self.plate_thickness.mm() * 0.5
    }

    pub fn frame_spacer_length(&self) -> f64 {
        self.frame_inner_top_z() - self.frame_inner_bottom_z()
    }

    pub fn fixed_post_length(&self) -> f64 {
        self.frame_spacer_length() + self.top_socket_depth.mm()
            - self.top_socket_axial_clearance.mm()
    }

    pub fn handle_gear_square_shaft_size(&self) -> f64 {
        9.0
    }

    pub fn handle_gear_square_socket_size(&self) -> f64 {
        self.handle_gear_square_shaft_size() + 0.30
    }

    pub fn handle_crank_square_shaft_size(&self) -> f64 {
        6.0
    }

    pub fn handle_crank_square_socket_size(&self) -> f64 {
        self.handle_crank_square_shaft_size() + 0.30
    }

    pub fn handle_bottom_taper_lower_diameter(&self) -> f64 {
        9.0
    }

    pub fn handle_bottom_taper_upper_diameter(&self) -> f64 {
        11.0
    }

    pub fn handle_top_taper_lower_diameter(&self) -> f64 {
        9.0
    }

    pub fn handle_top_taper_upper_diameter(&self) -> f64 {
        8.6
    }

    pub fn handle_taper_hole_diameter_clearance(&self) -> f64 {
        0.4
    }

    pub const fn rack_pusher_length(&self) -> f64 {
        8.0
    }

    pub const fn rack_pusher_width(&self) -> f64 {
        30.0
    }

    pub fn rack_overall_length(&self) -> f64 {
        self.rack.length() + self.rack.half_shift_mm() + self.rack_pusher_length()
    }

    pub fn rack_top_plate_clearance(&self) -> f64 {
        self.frame_inner_top_z() - self.rack.face_width().mm() * 0.5
    }

    pub fn rack_bottom_plate_clearance(&self) -> f64 {
        -self.rack.face_width().mm() * 0.5 - self.frame_inner_bottom_z()
    }

    pub fn handle_upper_thrust_spacer_length(&self) -> f64 {
        let gear_top =
            self.handle_spur_extended_center_z() + self.handle_spur_extended_face_width() * 0.5;
        self.frame_inner_top_z() - gear_top - self.axial_clearance.mm()
    }

    pub fn reduction_upper_thrust_spacer_length(&self) -> f64 {
        let gear_top = self.reduction_small_extended_center_z()
            + self.reduction_small_extended_face_width() * 0.5;
        self.frame_inner_top_z() - gear_top - self.axial_clearance.mm()
    }

    pub fn driven_lower_thrust_spacer_length(&self) -> f64 {
        let gear_bottom =
            self.secondary_spur_layer_center_z() - self.output_spur_face_width.mm() * 0.5;
        gear_bottom - self.frame_inner_bottom_z() - self.axial_clearance.mm()
    }

    pub fn idler_lower_thrust_spacer_length(&self) -> f64 {
        let gear_bottom = -self.idler_pinion.face_width().mm() * 0.5;
        gear_bottom - self.frame_inner_bottom_z() - self.axial_clearance.mm()
    }

    pub fn frame_outer_thickness_mm(&self) -> f64 {
        self.frame_spacer_length() + self.plate_thickness.mm() * 2.0
    }

    pub fn bolt_thread_engagement_mm(&self) -> f64 {
        self.bolt_length.mm() - (self.frame_outer_thickness_mm() - self.nut_pocket_depth.mm())
    }

    pub fn corner_positions(&self) -> Vec<[f64; 2]> {
        let x = self.plate_length.mm() * 0.5 - self.corner_inset.mm();
        let y = self.plate_width.mm() * 0.5 - self.corner_inset.mm();
        let center_y = self.plate_center_y;
        alloc::vec![
            [-x, center_y - y],
            [x, center_y - y],
            [-x, center_y + y],
            [x, center_y + y],
        ]
    }
}

impl fmt::Display for PrototypeError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::IncompatibleSpurModule => {
                formatter.write_str("all handle, D, B, and C spur gears must use the same module")
            }
            Self::IncompatibleSpurPressureAngle => formatter
                .write_str("all handle, D, B, and C spur gears must use the same pressure angle"),
            Self::ReductionSmallSpurMustHaveEvenTeeth => formatter.write_str(
                "the small D spur must have an even tooth count to mesh with B and C symmetrically",
            ),
            Self::OutputPinionBoreDoesNotFitJournal => {
                formatter.write_str("driven pinion bore must clear the printed journal")
            }
            Self::IdlerPinionBoreDoesNotFitJournal => {
                formatter.write_str("idler pinion bore must clear the printed journal")
            }
            Self::BoltDoesNotFitJournal => {
                formatter.write_str("bolt clearance must be smaller than journal outside diameter")
            }
            Self::NutDoesNotClearBolt => {
                formatter.write_str("nut pocket must be wider than the bolt clearance hole")
            }
            Self::ThrustSpacerDoesNotClearAxle => {
                formatter.write_str("thrust spacer outside diameter must exceed its rotating bore")
            }
            Self::ThrustSpacerHasNoRoom => {
                formatter.write_str("frame has no positive-length room for a thrust spacer")
            }
            Self::NutPocketTooDeep => {
                formatter.write_str("nut pocket must leave material in the bottom plate")
            }
            Self::BoltTooShort => formatter.write_str(
                "bolt is too short to engage the full configured nut thickness through the frame",
            ),
            Self::OutputSpurDoesNotSupportPinionExtension => formatter.write_str(
                "output spur root radius must fully support the extended lower helical teeth",
            ),
            Self::InvalidOutputSpurFaceWidth => formatter.write_str(
                "B/C output spur face width must be between the base spur width and the extended D-small face width",
            ),
            Self::PlateTooSmall => formatter.write_str("frame plate is too small for corner bolts"),
            Self::InvalidPlateCenter => formatter.write_str("plate center must be finite"),
            Self::InvalidTopSocket => formatter.write_str(
                "top socket depth must fit inside the plate and exceed its axial clearance",
            ),
            Self::InvalidHandleSquareDrive => formatter.write_str(
                "handle shaft tapers and square drives must remain insertable and leave material in the spur",
            ),
        }
    }
}

impl core::error::Error for PrototypeError {}

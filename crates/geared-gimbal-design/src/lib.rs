// SPDX-License-Identifier: MIT
#![no_std]

extern crate alloc;

use gimbal_core::*;

mod prototype;

pub use prototype::{
    CockpitParameters, ContactUnitParameters, FrameParameters, MotionParameters,
    PitchGearboxParameters, PitchSectorParameters, PrototypeDesign, PrototypeError,
    PrototypeParameters, RollAxisParameters, build_prototype,
};

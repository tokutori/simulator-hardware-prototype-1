// SPDX-License-Identifier: MIT
#![no_std]

extern crate alloc;

mod gear;
mod prototype;
mod rack;
mod units;

pub use gear::{
    DoubleHelicalGear, DoubleHelicalGearPair, GearError, GearHand, GearPose, GearProfile, Point2,
    SpurGear, TriangleMesh,
};
pub use prototype::{Prototype, PrototypeError, PrototypeMotion};
pub use rack::{DoubleHelicalRack, NormalGearSystem};
pub use units::{Angle, Length, UnitError};

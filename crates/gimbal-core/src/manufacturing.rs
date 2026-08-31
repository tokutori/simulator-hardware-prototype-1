// SPDX-License-Identifier: MIT

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FdmMaterial {
    Petg,
    Pla,
    Asa,
}

/// Nominal fabrication intent. Machine calibration belongs to the I/O/process layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Manufacturing {
    Fdm { material: FdmMaterial },
    LaserCut,
    Purchased,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RingSide {
    Left,
    Right,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum UnitPosition {
    Front,
    Rear,
}

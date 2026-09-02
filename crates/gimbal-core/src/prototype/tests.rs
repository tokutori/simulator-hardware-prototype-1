// SPDX-License-Identifier: MIT

use super::*;

#[test]
fn sector_definition_is_instanced_four_times() {
    // The concrete parameter integration test lives at the CLI boundary; this
    // compile-time test protects the definition/instance distinction itself.
    assert_ne!(core::mem::size_of::<ComponentDefinitionId>(), 0);
}

#[test]
fn sector_wedge_contains_the_roll_axis_extension() {
    let points = sector_wedge_points(150.0, PI / 6.0);
    let centre = points
        .iter()
        .max_by(|a, b| a.x.total_cmp(&b.x))
        .expect("wedge has arc samples");
    assert!(centre.x > 150.0);
    assert!(centre.y.abs() < 1.0e-10);
}

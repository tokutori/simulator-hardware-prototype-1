// SPDX-License-Identifier: MIT

use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;

use crate::{Angle, NonNegativeAngle, PositiveAngle};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct AngularCoordinateId(u32);

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct AngularConstraintId(u32);

#[derive(Clone, Debug, PartialEq)]
pub struct AngularCoordinate {
    pub name: String,
    pub period: PositiveAngle,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AngularConstraint {
    pub input: AngularCoordinateId,
    pub output: AngularCoordinateId,
    pub scale: f64,
    pub offset: Angle,
    pub phase_backlash: NonNegativeAngle,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConstraintDirection {
    Forward,
    Reverse,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ConstraintStep {
    pub constraint: AngularConstraintId,
    pub direction: ConstraintDirection,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ConstraintCycle {
    pub steps: Vec<ConstraintStep>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConstraintError {
    InvalidCoordinate(AngularCoordinateId),
    InvalidScale,
    SelfConstraint(AngularCoordinateId),
    DuplicateConstraint(AngularConstraintId),
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ConstraintGraph {
    coordinates: Vec<AngularCoordinate>,
    constraints: Vec<AngularConstraint>,
}

impl AngularCoordinateId {
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

impl AngularConstraintId {
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

impl ConstraintGraph {
    pub const fn new() -> Self {
        Self {
            coordinates: Vec::new(),
            constraints: Vec::new(),
        }
    }

    pub fn add_coordinate(&mut self, coordinate: AngularCoordinate) -> AngularCoordinateId {
        let id = AngularCoordinateId(self.coordinates.len() as u32);
        self.coordinates.push(coordinate);
        id
    }

    pub fn add_constraint(
        &mut self,
        constraint: AngularConstraint,
    ) -> Result<AngularConstraintId, ConstraintError> {
        for coordinate in [constraint.input, constraint.output] {
            if coordinate.index() >= self.coordinates.len() {
                return Err(ConstraintError::InvalidCoordinate(coordinate));
            }
        }
        if constraint.input == constraint.output {
            return Err(ConstraintError::SelfConstraint(constraint.input));
        }
        if !constraint.scale.is_finite() || constraint.scale.abs() <= f64::EPSILON {
            return Err(ConstraintError::InvalidScale);
        }
        if let Some(index) = self
            .constraints
            .iter()
            .position(|existing| *existing == constraint)
        {
            return Err(ConstraintError::DuplicateConstraint(AngularConstraintId(
                index as u32,
            )));
        }
        let id = AngularConstraintId(self.constraints.len() as u32);
        self.constraints.push(constraint);
        Ok(id)
    }

    pub fn coordinates(&self) -> &[AngularCoordinate] {
        &self.coordinates
    }

    pub fn constraints(&self) -> &[AngularConstraint] {
        &self.constraints
    }

    pub fn fundamental_cycles(&self) -> Vec<ConstraintCycle> {
        let mut cycles = Vec::new();
        for closing_index in 0..self.constraints.len() {
            let closing = self.constraints[closing_index];
            if let Some(mut steps) = self.path_before(
                closing.input,
                closing.output,
                AngularConstraintId(closing_index as u32),
            ) {
                steps.push(ConstraintStep {
                    constraint: AngularConstraintId(closing_index as u32),
                    direction: ConstraintDirection::Reverse,
                });
                cycles.push(ConstraintCycle { steps });
            }
        }
        cycles
    }

    fn path_before(
        &self,
        start: AngularCoordinateId,
        goal: AngularCoordinateId,
        before: AngularConstraintId,
    ) -> Option<Vec<ConstraintStep>> {
        #[derive(Clone, Copy)]
        struct Predecessor {
            coordinate: AngularCoordinateId,
            step: ConstraintStep,
        }

        let mut predecessor = vec![None; self.coordinates.len()];
        let mut queue = vec![start];
        predecessor[start.index()] = Some(Predecessor {
            coordinate: start,
            step: ConstraintStep {
                constraint: before,
                direction: ConstraintDirection::Forward,
            },
        });
        let mut cursor = 0;
        while cursor < queue.len() {
            let current = queue[cursor];
            cursor += 1;
            if current == goal {
                break;
            }
            for (index, constraint) in self.constraints[..before.index()].iter().enumerate() {
                let (next, direction) = if constraint.input == current {
                    (constraint.output, ConstraintDirection::Forward)
                } else if constraint.output == current {
                    (constraint.input, ConstraintDirection::Reverse)
                } else {
                    continue;
                };
                if predecessor[next.index()].is_none() {
                    predecessor[next.index()] = Some(Predecessor {
                        coordinate: current,
                        step: ConstraintStep {
                            constraint: AngularConstraintId(index as u32),
                            direction,
                        },
                    });
                    queue.push(next);
                }
            }
        }
        predecessor[goal.index()]?;
        let mut reverse_steps = Vec::new();
        let mut current = goal;
        while current != start {
            let entry = predecessor[current.index()]?;
            reverse_steps.push(entry.step);
            current = entry.coordinate;
        }
        reverse_steps.reverse();
        Some(reverse_steps)
    }
}

#[cfg(test)]
mod tests {
    use alloc::string::ToString;

    use super::*;

    fn coordinate(name: &str) -> AngularCoordinate {
        AngularCoordinate {
            name: name.to_string(),
            period: PositiveAngle::degrees(360.0).expect("positive period"),
        }
    }

    fn constraint(input: AngularCoordinateId, output: AngularCoordinateId) -> AngularConstraint {
        AngularConstraint {
            input,
            output,
            scale: -2.0,
            offset: Angle::degrees(0.0).expect("finite offset"),
            phase_backlash: NonNegativeAngle::degrees(0.0).expect("zero backlash"),
        }
    }

    #[test]
    fn fundamental_cycle_preserves_traversal_direction() {
        let mut graph = ConstraintGraph::new();
        let a = graph.add_coordinate(coordinate("a"));
        let b = graph.add_coordinate(coordinate("b"));
        let c = graph.add_coordinate(coordinate("c"));
        let ab = graph.add_constraint(constraint(a, b)).expect("a-b edge");
        let cb = graph.add_constraint(constraint(c, b)).expect("c-b edge");
        let ac = graph.add_constraint(constraint(a, c)).expect("a-c edge");

        assert_eq!(
            graph.fundamental_cycles(),
            vec![ConstraintCycle {
                steps: vec![
                    ConstraintStep {
                        constraint: ab,
                        direction: ConstraintDirection::Forward,
                    },
                    ConstraintStep {
                        constraint: cb,
                        direction: ConstraintDirection::Reverse,
                    },
                    ConstraintStep {
                        constraint: ac,
                        direction: ConstraintDirection::Reverse,
                    },
                ],
            }]
        );
    }

    #[test]
    fn rejects_self_zero_scale_and_duplicate_constraints() {
        let mut graph = ConstraintGraph::new();
        let a = graph.add_coordinate(coordinate("a"));
        let b = graph.add_coordinate(coordinate("b"));
        assert_eq!(
            graph.add_constraint(constraint(a, a)),
            Err(ConstraintError::SelfConstraint(a))
        );
        let mut zero = constraint(a, b);
        zero.scale = 0.0;
        assert_eq!(
            graph.add_constraint(zero),
            Err(ConstraintError::InvalidScale)
        );
        let edge = graph.add_constraint(constraint(a, b)).expect("first edge");
        assert_eq!(
            graph.add_constraint(constraint(a, b)),
            Err(ConstraintError::DuplicateConstraint(edge))
        );
    }
}

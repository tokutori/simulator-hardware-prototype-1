// SPDX-License-Identifier: MIT

use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;

use crate::datum::{DatumKind, DatumSet};
use crate::relation::{AssemblyRelation, AssemblyRelationId, RelationEndpointRef};
use crate::{ComponentLocation, ComponentRole, Length, Manufacturing, RegionId, SolidId};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ComponentDefinitionId(u32);

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ComponentInstanceId(u32);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct FrameId(u32);

#[derive(Clone, Debug, PartialEq)]
pub enum Body {
    Solid(SolidId),
    Sheet {
        outer: RegionId,
        cutouts: Vec<RegionId>,
        thickness: Length,
        assembly_solid: SolidId,
    },
}

impl Body {
    pub const fn assembly_solid(&self) -> SolidId {
        match self {
            Self::Solid(solid)
            | Self::Sheet {
                assembly_solid: solid,
                ..
            } => *solid,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ComponentDefinition {
    pub name: String,
    pub role: ComponentRole,
    pub body: Body,
    pub manufacturing: Manufacturing,
    pub color_rgba: [f32; 4],
    pub datums: DatumSet,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ComponentInstance {
    pub name: String,
    pub definition: ComponentDefinitionId,
    pub frame: FrameId,
    pub local_pose: RigidTransform,
    pub location: ComponentLocation,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ComponentIdentity {
    pub role: ComponentRole,
    pub location: ComponentLocation,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ComponentIdentityCollision {
    pub first: ComponentInstanceId,
    pub second: ComponentInstanceId,
    pub identity: ComponentIdentity,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ComponentInstancePair {
    pub first: ComponentInstanceId,
    pub second: ComponentInstanceId,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AssemblyError {
    InvalidInstance(ComponentInstanceId),
    InvalidDatum {
        instance: ComponentInstanceId,
        datum_index: usize,
    },
    DatumKindMismatch {
        instance: ComponentInstanceId,
        datum_index: usize,
        expected: DatumKind,
        actual: DatumKind,
    },
    SelfRelation(ComponentInstanceId),
    RelationEndpointInstanceMismatch {
        first: ComponentInstanceId,
        second: ComponentInstanceId,
    },
    RelationParticipantRoleMismatch {
        instance: ComponentInstanceId,
        expected: ComponentRole,
        actual: ComponentRole,
    },
    DuplicateRelationParticipant(ComponentInstanceId),
    DuplicateRelation(AssemblyRelationId),
    DuplicateComponentIdentity {
        first: ComponentInstanceId,
        second: ComponentInstanceId,
        identity: ComponentIdentity,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis3 {
    X,
    Y,
    Z,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoordinateExpr {
    pub pitch_scale: f64,
    pub roll_scale: f64,
    pub offset_radians: f64,
}

impl CoordinateExpr {
    pub const fn fixed() -> Self {
        Self {
            pitch_scale: 0.0,
            roll_scale: 0.0,
            offset_radians: 0.0,
        }
    }

    pub const fn pitch(scale: f64) -> Self {
        Self {
            pitch_scale: scale,
            roll_scale: 0.0,
            offset_radians: 0.0,
        }
    }

    pub const fn roll(scale: f64) -> Self {
        Self {
            pitch_scale: 0.0,
            roll_scale: scale,
            offset_radians: 0.0,
        }
    }

    fn evaluate(self, pitch: f64, roll: f64) -> f64 {
        self.pitch_scale * pitch + self.roll_scale * roll + self.offset_radians
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Joint {
    Fixed,
    Revolute {
        axis: Axis3,
        coordinate: CoordinateExpr,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RigidTransform {
    pub translation: [f64; 3],
    /// Quaternion in glTF order: x, y, z, w.
    pub rotation: [f64; 4],
}

impl RigidTransform {
    pub const IDENTITY: Self = Self {
        translation: [0.0; 3],
        rotation: [0.0, 0.0, 0.0, 1.0],
    };

    pub const fn translated(x: f64, y: f64, z: f64) -> Self {
        Self {
            translation: [x, y, z],
            rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }

    pub fn rotated(axis: Axis3, radians: f64) -> Self {
        Self {
            translation: [0.0; 3],
            rotation: axis_quaternion(axis, radians),
        }
    }

    pub fn compose(self, child: Self) -> Self {
        let moved = rotate_vector(self.rotation, child.translation);
        Self {
            translation: [
                self.translation[0] + moved[0],
                self.translation[1] + moved[1],
                self.translation[2] + moved[2],
            ],
            rotation: quaternion_multiply(self.rotation, child.rotation),
        }
    }

    pub fn transform_point(self, point: [f64; 3]) -> [f64; 3] {
        let rotated = rotate_vector(self.rotation, point);
        [
            rotated[0] + self.translation[0],
            rotated[1] + self.translation[1],
            rotated[2] + self.translation[2],
        ]
    }

    pub fn transform_vector(self, vector: [f64; 3]) -> [f64; 3] {
        rotate_vector(self.rotation, vector)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame {
    parent: Option<FrameId>,
    nominal: RigidTransform,
    joint: Joint,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FrameGraph {
    frames: Vec<Frame>,
}

impl FrameGraph {
    pub fn new() -> Self {
        Self {
            frames: vec![Frame {
                parent: None,
                nominal: RigidTransform::IDENTITY,
                joint: Joint::Fixed,
            }],
        }
    }

    pub const fn world(&self) -> FrameId {
        FrameId(0)
    }

    pub fn add_frame(&mut self, parent: FrameId, nominal: RigidTransform, joint: Joint) -> FrameId {
        assert!(
            parent.index() < self.frames.len(),
            "parent frame must already exist"
        );
        let id = FrameId(self.frames.len() as u32);
        self.frames.push(Frame {
            parent: Some(parent),
            nominal,
            joint,
        });
        id
    }

    pub fn world_poses(&self, pitch_radians: f64, roll_radians: f64) -> Vec<RigidTransform> {
        let mut poses = vec![RigidTransform::IDENTITY; self.frames.len()];
        for (index, frame) in self.frames.iter().enumerate().skip(1) {
            let parent = poses[frame.parent.expect("non-world frame has parent").index()];
            let joint = match frame.joint {
                Joint::Fixed => RigidTransform::IDENTITY,
                Joint::Revolute { axis, coordinate } => {
                    RigidTransform::rotated(axis, coordinate.evaluate(pitch_radians, roll_radians))
                }
            };
            poses[index] = parent.compose(frame.nominal).compose(joint);
        }
        poses
    }

    pub fn len(&self) -> usize {
        self.frames.len()
    }

    /// A frame graph always contains its world frame.
    pub const fn is_empty(&self) -> bool {
        false
    }
}

impl Default for FrameGraph {
    fn default() -> Self {
        Self::new()
    }
}

impl ComponentDefinitionId {
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

impl ComponentInstanceId {
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

impl FrameId {
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Assembly {
    definitions: Vec<ComponentDefinition>,
    instances: Vec<ComponentInstance>,
    relations: Vec<AssemblyRelation>,
}

impl Assembly {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_definition(&mut self, definition: ComponentDefinition) -> ComponentDefinitionId {
        let id = ComponentDefinitionId(self.definitions.len() as u32);
        self.definitions.push(definition);
        id
    }

    pub fn add_instance(&mut self, instance: ComponentInstance) -> ComponentInstanceId {
        assert!(
            instance.definition.index() < self.definitions.len(),
            "component definition must already exist"
        );
        let id = ComponentInstanceId(self.instances.len() as u32);
        self.instances.push(instance);
        id
    }

    pub fn add_relation(
        &mut self,
        relation: AssemblyRelation,
    ) -> Result<AssemblyRelationId, AssemblyError> {
        if let AssemblyRelation::Fastened(joint) = relation {
            for (hole, seat) in [
                (joint.first_hole.instance, joint.first_seat.instance),
                (joint.second_hole.instance, joint.second_seat.instance),
            ] {
                if hole != seat {
                    return Err(AssemblyError::RelationEndpointInstanceMismatch {
                        first: hole,
                        second: seat,
                    });
                }
            }
        }
        let [first, second] = relation.instance_pair();
        if first == second {
            return Err(AssemblyError::SelfRelation(first));
        }
        let participants = relation.participant_instances();
        for participant in participants.into_iter().flatten() {
            if participant.index() >= self.instances.len() {
                return Err(AssemblyError::InvalidInstance(participant));
            }
        }
        for (index, participant) in participants.iter().copied().flatten().enumerate() {
            if participants
                .iter()
                .copied()
                .skip(index + 1)
                .flatten()
                .any(|other| other == participant)
            {
                return Err(AssemblyError::DuplicateRelationParticipant(participant));
            }
        }
        if let AssemblyRelation::Fastened(joint) = relation {
            for (participant, expected) in [
                (Some(joint.hardware.bolt), ComponentRole::M3Bolt),
                (Some(joint.hardware.nut), ComponentRole::M3Nut),
                (joint.hardware.first_washer, ComponentRole::M3Washer),
                (joint.hardware.second_washer, ComponentRole::M3Washer),
            ] {
                let Some(participant) = participant else {
                    continue;
                };
                let actual =
                    self.definitions[self.instances[participant.index()].definition.index()].role;
                if actual != expected {
                    return Err(AssemblyError::RelationParticipantRoleMismatch {
                        instance: participant,
                        expected,
                        actual,
                    });
                }
            }
        }
        let endpoints = relation.endpoint_refs();
        for endpoint in endpoints.into_iter().flatten() {
            self.validate_endpoint(endpoint)?;
        }
        if let Some(index) = self
            .relations
            .iter()
            .position(|existing| *existing == relation)
        {
            return Err(AssemblyError::DuplicateRelation(AssemblyRelationId::new(
                index,
            )));
        }
        let id = AssemblyRelationId::new(self.relations.len());
        self.relations.push(relation);
        Ok(id)
    }

    pub fn definitions(&self) -> &[ComponentDefinition] {
        &self.definitions
    }

    pub fn definitions_with_ids(
        &self,
    ) -> impl Iterator<Item = (ComponentDefinitionId, &ComponentDefinition)> {
        self.definitions
            .iter()
            .enumerate()
            .map(|(index, definition)| (ComponentDefinitionId(index as u32), definition))
    }

    pub fn instances(&self) -> &[ComponentInstance] {
        &self.instances
    }

    pub fn instances_with_ids(
        &self,
    ) -> impl Iterator<Item = (ComponentInstanceId, &ComponentInstance)> {
        self.instances
            .iter()
            .enumerate()
            .map(|(index, instance)| (ComponentInstanceId(index as u32), instance))
    }

    pub fn relations(&self) -> &[AssemblyRelation] {
        &self.relations
    }

    pub fn relations_with_ids(
        &self,
    ) -> impl Iterator<Item = (AssemblyRelationId, &AssemblyRelation)> {
        self.relations
            .iter()
            .enumerate()
            .map(|(index, relation)| (AssemblyRelationId::new(index), relation))
    }

    pub fn instance_pairs(&self) -> impl Iterator<Item = ComponentInstancePair> + '_ {
        (0..self.instances.len()).flat_map(move |first_index| {
            (first_index + 1..self.instances.len()).map(move |second_index| ComponentInstancePair {
                first: ComponentInstanceId(first_index as u32),
                second: ComponentInstanceId(second_index as u32),
            })
        })
    }

    pub fn relations_between(
        &self,
        pair: ComponentInstancePair,
    ) -> impl Iterator<Item = (AssemblyRelationId, &AssemblyRelation)> {
        self.relations
            .iter()
            .enumerate()
            .filter_map(move |(index, relation)| {
                let [first, second] = relation.instance_pair();
                ((first == pair.first && second == pair.second)
                    || (first == pair.second && second == pair.first))
                    .then_some((AssemblyRelationId::new(index), relation))
            })
    }

    pub fn unrelated_instance_pairs(&self) -> impl Iterator<Item = ComponentInstancePair> + '_ {
        self.instance_pairs()
            .filter(|pair| self.relations_between(*pair).next().is_none())
    }

    pub fn definition(&self, id: ComponentDefinitionId) -> Option<&ComponentDefinition> {
        self.definitions.get(id.index())
    }

    pub fn instance(&self, id: ComponentInstanceId) -> Option<&ComponentInstance> {
        self.instances.get(id.index())
    }

    pub fn instance_by_identity(&self, identity: ComponentIdentity) -> Option<ComponentInstanceId> {
        self.instances_with_ids().find_map(|(id, instance)| {
            (ComponentIdentity {
                role: self.definitions[instance.definition.index()].role,
                location: instance.location,
            } == identity)
                .then_some(id)
        })
    }

    pub fn instances_with_role(
        &self,
        role: ComponentRole,
    ) -> impl Iterator<Item = (ComponentInstanceId, &ComponentInstance)> {
        self.instances
            .iter()
            .enumerate()
            .filter(move |(_, instance)| self.definitions[instance.definition.index()].role == role)
            .map(|(index, instance)| (ComponentInstanceId(index as u32), instance))
    }

    pub fn component_identity(&self, id: ComponentInstanceId) -> Option<ComponentIdentity> {
        let instance = self.instance(id)?;
        Some(ComponentIdentity {
            role: self.definition(instance.definition)?.role,
            location: instance.location,
        })
    }

    pub fn validate_unique_component_identities(&self) -> Result<(), AssemblyError> {
        if let Some(collision) = self.component_identity_collisions().first().copied() {
            return Err(AssemblyError::DuplicateComponentIdentity {
                first: collision.first,
                second: collision.second,
                identity: collision.identity,
            });
        }
        Ok(())
    }

    pub fn component_identity_collisions(&self) -> Vec<ComponentIdentityCollision> {
        let mut collisions = Vec::new();
        for first_index in 0..self.instances.len() {
            let first_id = ComponentInstanceId(first_index as u32);
            let first_identity = self
                .component_identity(first_id)
                .expect("inserted instances always reference a definition");
            for second_index in first_index + 1..self.instances.len() {
                let second_id = ComponentInstanceId(second_index as u32);
                if self.component_identity(second_id) == Some(first_identity) {
                    collisions.push(ComponentIdentityCollision {
                        first: first_id,
                        second: second_id,
                        identity: first_identity,
                    });
                }
            }
        }
        collisions
    }

    fn validate_endpoint(&self, endpoint: RelationEndpointRef) -> Result<(), AssemblyError> {
        let instance = self
            .instance(endpoint.instance)
            .ok_or(AssemblyError::InvalidInstance(endpoint.instance))?;
        let definition = self
            .definition(instance.definition)
            .expect("instance definition was validated when inserted");
        let datum =
            definition
                .datums
                .named(endpoint.datum_index)
                .ok_or(AssemblyError::InvalidDatum {
                    instance: endpoint.instance,
                    datum_index: endpoint.datum_index,
                })?;
        let actual = datum.geometry.kind();
        if actual != endpoint.kind {
            return Err(AssemblyError::DatumKindMismatch {
                instance: endpoint.instance,
                datum_index: endpoint.datum_index,
                expected: endpoint.kind,
                actual,
            });
        }
        Ok(())
    }
}

fn axis_quaternion(axis: Axis3, radians: f64) -> [f64; 4] {
    let half = radians * 0.5;
    let sine = libm::sin(half);
    let cosine = libm::cos(half);
    match axis {
        Axis3::X => [sine, 0.0, 0.0, cosine],
        Axis3::Y => [0.0, sine, 0.0, cosine],
        Axis3::Z => [0.0, 0.0, sine, cosine],
    }
}

fn quaternion_multiply(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]
}

fn rotate_vector(quaternion: [f64; 4], point: [f64; 3]) -> [f64; 3] {
    let q = [quaternion[0], quaternion[1], quaternion[2]];
    let uv = cross(q, point);
    let uuv = cross(q, uv);
    [
        point[0] + 2.0 * (quaternion[3] * uv[0] + uuv[0]),
        point[1] + 2.0 * (quaternion[3] * uv[1] + uuv[1]),
        point[2] + 2.0 * (quaternion[3] * uv[2] + uuv[2]),
    ]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[cfg(test)]
mod tests {
    use alloc::string::ToString;

    use super::*;
    use crate::{
        AxisDatum, CylinderDatum, DatumEndpoint, EngineeringTolerance, FastenedJoint,
        FastenerHardware, FastenerHeadSide, FeatureBuilder, Manufacturing, MetricThread,
        NonNegativeAngle, NonNegativeLength, PlaneDatum, Point3, PositiveArea, PositiveLength,
        Primitive3, SurfaceContact, UnitVector3,
    };

    #[test]
    fn nested_roll_axis_follows_pitch_frame() {
        let mut frames = FrameGraph::new();
        let pitch = frames.add_frame(
            frames.world(),
            RigidTransform::IDENTITY,
            Joint::Revolute {
                axis: Axis3::Y,
                coordinate: CoordinateExpr::pitch(1.0),
            },
        );
        let roll = frames.add_frame(
            pitch,
            RigidTransform::IDENTITY,
            Joint::Revolute {
                axis: Axis3::X,
                coordinate: CoordinateExpr::roll(1.0),
            },
        );
        let poses = frames.world_poses(core::f64::consts::FRAC_PI_2, 0.0);
        let point = poses[roll.index()].transform_point([1.0, 0.0, 0.0]);
        assert!(point[0].abs() < 1.0e-10);
        assert!((point[2] + 1.0).abs() < 1.0e-10);
    }

    #[test]
    fn relation_references_are_checked_against_each_instance_definition() {
        let mut builder = FeatureBuilder::new();
        let solid = builder.primitive(Primitive3::Box {
            x: Length::positive_mm(1.0).expect("positive length"),
            y: Length::positive_mm(1.0).expect("positive length"),
            z: Length::positive_mm(1.0).expect("positive length"),
            centered: true,
        });
        let plane = PlaneDatum {
            origin: Point3::from_mm([0.0, 0.0, 0.0]).expect("finite point"),
            normal: UnitVector3::new([0.0, 0.0, 1.0]).expect("valid normal"),
        };
        let mut first_datums = DatumSet::new();
        let first_plane = first_datums.add("mounting_plane".to_string(), plane);
        let mut second_datums = DatumSet::new();
        let second_plane = second_datums.add("mounting_plane".to_string(), plane);

        let mut assembly = Assembly::new();
        let first_definition = assembly.add_definition(ComponentDefinition {
            name: "first".to_string(),
            role: ComponentRole::FixedCrossmember,
            body: Body::Solid(solid),
            manufacturing: Manufacturing::Purchased,
            color_rgba: [1.0; 4],
            datums: first_datums,
        });
        let second_definition = assembly.add_definition(ComponentDefinition {
            name: "second".to_string(),
            role: ComponentRole::FixedCrossmember,
            body: Body::Solid(solid),
            manufacturing: Manufacturing::Purchased,
            color_rgba: [1.0; 4],
            datums: second_datums,
        });
        let first = assembly.add_instance(ComponentInstance {
            name: "first".to_string(),
            definition: first_definition,
            frame: FrameGraph::new().world(),
            local_pose: RigidTransform::IDENTITY,
            location: ComponentLocation::default(),
        });
        let second = assembly.add_instance(ComponentInstance {
            name: "second".to_string(),
            definition: second_definition,
            frame: FrameGraph::new().world(),
            local_pose: RigidTransform::IDENTITY,
            location: ComponentLocation::default(),
        });
        let tolerance = EngineeringTolerance {
            linear: NonNegativeLength::mm(0.01).expect("non-negative tolerance"),
            angular: NonNegativeAngle::degrees(0.1).expect("non-negative tolerance"),
        };
        let relation = AssemblyRelation::SurfaceContact(SurfaceContact {
            first: DatumEndpoint::new(first, first_plane),
            second: DatumEndpoint::new(second, second_plane),
            minimum_contact_area: PositiveArea::square_mm(1.0).expect("positive area"),
            tolerance,
        });
        let relation_id = assembly
            .add_relation(relation)
            .expect("valid relation references");
        assert_eq!(relation_id.index(), 0);
        assert_eq!(assembly.relations(), &[relation]);
        let connected_pair = ComponentInstancePair { first, second };
        assert_eq!(
            assembly.instance_pairs().collect::<Vec<_>>(),
            [connected_pair]
        );
        assert_eq!(assembly.relations_between(connected_pair).count(), 1);
        assert_eq!(assembly.unrelated_instance_pairs().count(), 0);
        assert_eq!(
            assembly.add_relation(relation),
            Err(AssemblyError::DuplicateRelation(relation_id))
        );
    }

    #[test]
    fn relation_rejects_a_datum_id_borrowed_from_another_definition() {
        let mut builder = FeatureBuilder::new();
        let solid = builder.primitive(Primitive3::Box {
            x: Length::positive_mm(1.0).expect("positive length"),
            y: Length::positive_mm(1.0).expect("positive length"),
            z: Length::positive_mm(1.0).expect("positive length"),
            centered: true,
        });
        let point = Point3::from_mm([0.0, 0.0, 0.0]).expect("finite point");
        let mut plane_datums = DatumSet::new();
        let plane_id = plane_datums.add(
            "mounting_plane".to_string(),
            PlaneDatum {
                origin: point,
                normal: UnitVector3::new([0.0, 0.0, 1.0]).expect("valid normal"),
            },
        );
        let mut wrong_datums = DatumSet::new();
        wrong_datums.add(
            "shaft_axis".to_string(),
            crate::AxisDatum {
                origin: point,
                direction: UnitVector3::new([1.0, 0.0, 0.0]).expect("valid direction"),
            },
        );
        let mut assembly = Assembly::new();
        let plane_definition = assembly.add_definition(ComponentDefinition {
            name: "plane".to_string(),
            role: ComponentRole::FixedCrossmember,
            body: Body::Solid(solid),
            manufacturing: Manufacturing::Purchased,
            color_rgba: [1.0; 4],
            datums: plane_datums,
        });
        let wrong_definition = assembly.add_definition(ComponentDefinition {
            name: "axis".to_string(),
            role: ComponentRole::RollShaft,
            body: Body::Solid(solid),
            manufacturing: Manufacturing::Purchased,
            color_rgba: [1.0; 4],
            datums: wrong_datums,
        });
        let first = assembly.add_instance(ComponentInstance {
            name: "plane".to_string(),
            definition: plane_definition,
            frame: FrameGraph::new().world(),
            local_pose: RigidTransform::IDENTITY,
            location: ComponentLocation::default(),
        });
        let second = assembly.add_instance(ComponentInstance {
            name: "axis".to_string(),
            definition: wrong_definition,
            frame: FrameGraph::new().world(),
            local_pose: RigidTransform::IDENTITY,
            location: ComponentLocation::default(),
        });
        let relation = AssemblyRelation::SurfaceContact(SurfaceContact {
            first: DatumEndpoint::new(first, plane_id),
            second: DatumEndpoint::new(second, plane_id),
            minimum_contact_area: PositiveArea::square_mm(1.0).expect("positive area"),
            tolerance: EngineeringTolerance {
                linear: NonNegativeLength::mm(0.0).expect("zero tolerance"),
                angular: NonNegativeAngle::radians(0.0).expect("zero tolerance"),
            },
        });
        assert!(matches!(
            assembly.add_relation(relation),
            Err(AssemblyError::DatumKindMismatch {
                instance,
                datum_index: 0,
                expected: DatumKind::Plane,
                actual: DatumKind::Axis,
            }) if instance == second
        ));
    }

    #[test]
    fn fastened_relation_requires_distinct_valid_hardware_and_member_datums() {
        let mut builder = FeatureBuilder::new();
        let solid = builder.primitive(Primitive3::Box {
            x: Length::positive_mm(10.0).expect("positive length"),
            y: Length::positive_mm(10.0).expect("positive length"),
            z: Length::positive_mm(3.0).expect("positive length"),
            centered: true,
        });
        let origin = Point3::from_mm([0.0, 0.0, 0.0]).expect("finite point");
        let axis = AxisDatum {
            origin,
            direction: UnitVector3::new([0.0, 0.0, 1.0]).expect("valid direction"),
        };
        let mut member_datums = DatumSet::new();
        let hole = member_datums.add(
            "m3_clearance_hole".to_string(),
            CylinderDatum {
                axis,
                radius: PositiveLength::mm(1.7).expect("positive radius"),
            },
        );
        let seat = member_datums.add(
            "washer_seat".to_string(),
            PlaneDatum {
                origin,
                normal: UnitVector3::new([0.0, 0.0, 1.0]).expect("valid normal"),
            },
        );
        let mut assembly = Assembly::new();
        let member_definition = assembly.add_definition(ComponentDefinition {
            name: "member".to_string(),
            role: ComponentRole::FixedCrossmember,
            body: Body::Solid(solid),
            manufacturing: Manufacturing::Fdm,
            color_rgba: [1.0; 4],
            datums: member_datums,
        });
        let first = assembly.add_instance(ComponentInstance {
            name: "first_member".to_string(),
            definition: member_definition,
            frame: FrameGraph::new().world(),
            local_pose: RigidTransform::IDENTITY,
            location: ComponentLocation::default(),
        });
        let second = assembly.add_instance(ComponentInstance {
            name: "second_member".to_string(),
            definition: member_definition,
            frame: FrameGraph::new().world(),
            local_pose: RigidTransform::translated(0.0, 0.0, 3.0),
            location: ComponentLocation::default(),
        });
        let mut add_hardware = |name: &str, role| {
            let definition = assembly.add_definition(ComponentDefinition {
                name: name.to_string(),
                role,
                body: Body::Solid(solid),
                manufacturing: Manufacturing::Purchased,
                color_rgba: [1.0; 4],
                datums: DatumSet::new(),
            });
            assembly.add_instance(ComponentInstance {
                name: name.to_string(),
                definition,
                frame: FrameGraph::new().world(),
                local_pose: RigidTransform::IDENTITY,
                location: ComponentLocation::default(),
            })
        };
        let bolt = add_hardware("m3_bolt", ComponentRole::M3Bolt);
        let nut = add_hardware("m3_nut", ComponentRole::M3Nut);
        let wrong_nut = add_hardware("wrong_nut", ComponentRole::M3Bolt);
        let tolerance = EngineeringTolerance {
            linear: NonNegativeLength::mm(0.05).expect("non-negative tolerance"),
            angular: NonNegativeAngle::degrees(0.2).expect("non-negative tolerance"),
        };
        let relation = AssemblyRelation::Fastened(FastenedJoint {
            first_hole: DatumEndpoint::new(first, hole),
            second_hole: DatumEndpoint::new(second, hole),
            first_seat: DatumEndpoint::new(first, seat),
            second_seat: DatumEndpoint::new(second, seat),
            hardware: FastenerHardware {
                bolt,
                nut,
                first_washer: None,
                second_washer: None,
            },
            thread: MetricThread::M3,
            target_hole_radial_clearance: NonNegativeLength::mm(0.2)
                .expect("non-negative clearance"),
            grip_length: PositiveLength::mm(6.0).expect("positive grip"),
            head_side: FastenerHeadSide::First,
            tolerance,
        });
        assembly
            .add_relation(relation)
            .expect("valid fastened relation");

        let AssemblyRelation::Fastened(joint) = relation else {
            unreachable!();
        };
        let mut mismatched = joint;
        mismatched.first_seat = DatumEndpoint::new(second, seat);
        assert!(matches!(
            assembly.add_relation(AssemblyRelation::Fastened(mismatched)),
            Err(AssemblyError::RelationEndpointInstanceMismatch {
                first: actual_first,
                second: actual_second,
            }) if actual_first == first && actual_second == second
        ));
        let mut aliased = joint;
        aliased.hardware.nut = bolt;
        assert_eq!(
            assembly.add_relation(AssemblyRelation::Fastened(aliased)),
            Err(AssemblyError::DuplicateRelationParticipant(bolt))
        );
        let mut wrong_role = joint;
        wrong_role.hardware.nut = wrong_nut;
        assert_eq!(
            assembly.add_relation(AssemblyRelation::Fastened(wrong_role)),
            Err(AssemblyError::RelationParticipantRoleMismatch {
                instance: wrong_nut,
                expected: ComponentRole::M3Nut,
                actual: ComponentRole::M3Bolt,
            })
        );
    }
}

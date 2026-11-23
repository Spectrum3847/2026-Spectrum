//! Java's `Field2d`: the robot pose and named pose lists for a field widget.
//!
//! Each object is a double array topic of `x, y, degrees` triples. Java also reads poses back
//! from the dashboard, so a drag in the widget changes `get_pose`. Here the getters return what
//! the robot last set.

use super::io::SharedIo;
use super::sendable::{Sendable, SendableBuilder};
use crate::nt::Value;
use std::cell::RefCell;
use std::rc::Rc;
use wpimath::geometry::{Pose2d, Rotation2d};

struct Object {
    name: String,
    poses: Vec<Pose2d>,
}

struct Published {
    table: String,
    io: SharedIo,
}

struct State {
    objects: Vec<Object>,
    published: Option<Published>,
}

/// `x, y, degrees` for each pose.
fn encode(poses: &[Pose2d]) -> Vec<f64> {
    poses
        .iter()
        .flat_map(|p| [p.translation.x, p.translation.y, p.rotation.degrees()])
        .collect()
}

impl State {
    fn path(published: &Published, name: &str) -> String {
        format!("{}/{name}", published.table)
    }

    fn write(&self, index: usize) {
        if let Some(published) = &self.published {
            let object = &self.objects[index];
            published.io.borrow_mut().set(
                &Self::path(published, &object.name),
                &Value::DoubleArray(encode(&object.poses)),
            );
        }
    }
}

/// A field widget. Clones share state.
#[derive(Clone)]
pub struct Field2d {
    state: Rc<RefCell<State>>,
}

impl Default for Field2d {
    fn default() -> Self {
        Self::new()
    }
}

impl Field2d {
    /// Starts with a `Robot` object at the origin.
    pub fn new() -> Self {
        Self {
            state: Rc::new(RefCell::new(State {
                objects: vec![Object {
                    name: "Robot".to_owned(),
                    poses: vec![Pose2d::ZERO],
                }],
                published: None,
            })),
        }
    }

    pub fn set_robot_pose(&self, pose: Pose2d) {
        self.get_robot_object().set_pose(pose);
    }

    pub fn set_robot_pose_xy(&self, x: f64, y: f64, rotation: Rotation2d) {
        self.set_robot_pose(Pose2d::from_xy(x, y, rotation));
    }

    pub fn get_robot_pose(&self) -> Pose2d {
        self.get_robot_object().get_pose()
    }

    pub fn get_robot_object(&self) -> FieldObject2d {
        FieldObject2d {
            state: self.state.clone(),
            index: 0,
        }
    }

    /// The object called `name`, created empty if it is new. A new object appears on the
    /// dashboard when its first pose is set.
    pub fn get_object(&self, name: &str) -> FieldObject2d {
        let mut state = self.state.borrow_mut();
        let index = match state.objects.iter().position(|o| o.name == name) {
            Some(i) => i,
            None => {
                state.objects.push(Object {
                    name: name.to_owned(),
                    poses: Vec::new(),
                });
                state.objects.len() - 1
            }
        };
        FieldObject2d {
            state: self.state.clone(),
            index,
        }
    }
}

impl Sendable for Field2d {
    fn init_sendable(&self, builder: &mut SendableBuilder) {
        builder.set_smartdashboard_type("Field2d");
        let published = Published {
            table: builder.table().to_owned(),
            io: builder.io(),
        };
        let mut state = self.state.borrow_mut();
        for object in &state.objects {
            published.io.borrow_mut().set_default(
                &State::path(&published, &object.name),
                &Value::DoubleArray(encode(&object.poses)),
            );
        }
        state.published = Some(published);
    }

    fn identity(&self) -> usize {
        Rc::as_ptr(&self.state) as usize
    }
}

/// A named list of poses in a [`Field2d`].
#[derive(Clone)]
pub struct FieldObject2d {
    state: Rc<RefCell<State>>,
    index: usize,
}

impl FieldObject2d {
    pub fn set_pose(&self, pose: Pose2d) {
        self.set_poses(&[pose]);
    }

    pub fn set_pose_xy(&self, x: f64, y: f64, rotation: Rotation2d) {
        self.set_pose(Pose2d::from_xy(x, y, rotation));
    }

    pub fn set_poses(&self, poses: &[Pose2d]) {
        let mut state = self.state.borrow_mut();
        let object = &mut state.objects[self.index];
        object.poses.clear();
        object.poses.extend_from_slice(poses);
        state.write(self.index);
    }

    /// The first pose, or the origin when the list is empty.
    pub fn get_pose(&self) -> Pose2d {
        let state = self.state.borrow();
        state.objects[self.index]
            .poses
            .first()
            .copied()
            .unwrap_or(Pose2d::ZERO)
    }

    pub fn get_poses(&self) -> Vec<Pose2d> {
        self.state.borrow().objects[self.index].poses.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dashboard::io::FakeIo;

    const TABLE: &str = "/SmartDashboard/Field2d";

    fn publish(field: &Field2d) -> Rc<RefCell<FakeIo>> {
        let fake = FakeIo::shared();
        let mut builder = SendableBuilder::new(TABLE, fake.clone());
        field.init_sendable(&mut builder);
        fake
    }

    #[test]
    fn layout_matches_java() {
        let field = Field2d::new();
        field.set_robot_pose(Pose2d::from_xy(1.0, 2.0, Rotation2d::from_degrees(90.0)));
        let fake = publish(&field);
        let io = fake.borrow();
        assert_eq!(io.text(&format!("{TABLE}/.type")), "Field2d");
        let robot = io.doubles(&format!("{TABLE}/Robot"));
        assert_eq!(&robot[..2], [1.0, 2.0]);
        assert!((robot[2] - 90.0).abs() < 1e-9);
        assert_eq!(robot.len(), 3);
    }

    #[test]
    fn robot_starts_at_the_origin() {
        let fake = publish(&Field2d::new());
        assert_eq!(
            fake.borrow().doubles(&format!("{TABLE}/Robot")),
            [0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn named_objects_hold_pose_lists() {
        let field = Field2d::new();
        let fake = publish(&field);
        let tags = field.get_object("tags");
        tags.set_poses(&[
            Pose2d::from_xy(1.0, 1.0, Rotation2d::ZERO),
            Pose2d::from_xy(2.0, 3.0, Rotation2d::from_degrees(180.0)),
        ]);
        let got = fake.borrow().doubles(&format!("{TABLE}/tags"));
        assert_eq!(got.len(), 6);
        assert_eq!(&got[3..5], [2.0, 3.0]);
        assert!((got[5] - 180.0).abs() < 1e-9);
        assert_eq!(tags.get_poses().len(), 2);
    }

    #[test]
    fn same_name_gives_the_same_object() {
        let field = Field2d::new();
        field
            .get_object("cam")
            .set_pose(Pose2d::from_xy(4.0, 5.0, Rotation2d::ZERO));
        assert_eq!(field.get_object("cam").get_pose().x(), 4.0);
    }

    #[test]
    fn objects_added_after_publish_appear_on_first_set() {
        let field = Field2d::new();
        let fake = publish(&field);
        let cam = field.get_object("cam");
        assert!(!fake.borrow().values.contains_key(&format!("{TABLE}/cam")));
        cam.set_pose(Pose2d::ZERO);
        assert!(fake.borrow().values.contains_key(&format!("{TABLE}/cam")));
    }

    #[test]
    fn objects_set_before_publish_are_announced_with_a_default() {
        let field = Field2d::new();
        field
            .get_object("cam")
            .set_pose(Pose2d::from_xy(7.0, 0.0, Rotation2d::ZERO));
        let fake = publish(&field);
        assert_eq!(fake.borrow().doubles(&format!("{TABLE}/cam"))[0], 7.0);
    }

    #[test]
    fn empty_list_publishes_an_empty_array() {
        let field = Field2d::new();
        let cam = field.get_object("cam");
        let fake = publish(&field);
        assert!(fake.borrow().doubles(&format!("{TABLE}/cam")).is_empty());
        assert_eq!(cam.get_pose(), Pose2d::ZERO);
    }

    #[test]
    fn setting_before_publish_writes_nothing() {
        let field = Field2d::new();
        field.set_robot_pose(Pose2d::from_xy(1.0, 1.0, Rotation2d::ZERO));
        assert_eq!(field.get_robot_pose().x(), 1.0);
    }
}

//! Java's `Mechanism2d`: a canvas of roots with lines (ligaments) attached to them.
//!
//! Java also reads ligament values back from the dashboard. Here the getters return what the
//! robot last set.

use super::io::SharedIo;
use super::sendable::{Sendable, SendableBuilder};
use crate::nt::Value;
use std::cell::RefCell;
use std::rc::Rc;
use wpimath::geometry::Rotation2d;

/// An 8 bit RGB color, Java's `Color8Bit`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Color8Bit {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
}

impl Color8Bit {
    pub const BLACK: Self = Self::new(0, 0, 0);
    pub const BLUE: Self = Self::new(0, 0, 255);
    pub const GREEN: Self = Self::new(0, 128, 0);
    pub const GRAY: Self = Self::new(128, 128, 128);
    pub const LIGHT_GRAY: Self = Self::new(211, 211, 211);
    pub const ORANGE: Self = Self::new(255, 165, 0);
    pub const PURPLE: Self = Self::new(128, 0, 128);
    pub const RED: Self = Self::new(255, 0, 0);
    pub const WHITE: Self = Self::new(255, 255, 255);
    pub const YELLOW: Self = Self::new(255, 255, 0);

    pub const fn new(red: u8, green: u8, blue: u8) -> Self {
        Self { red, green, blue }
    }

    /// `#RRGGBB` in upper case.
    pub fn hex(self) -> String {
        format!("#{:02X}{:02X}{:02X}", self.red, self.green, self.blue)
    }
}

/// A line to attach to a root or another line. Java's `MechanismLigament2d` constructor.
#[derive(Clone, Debug)]
pub struct Ligament {
    name: String,
    length: f64,
    angle: f64,
    weight: f64,
    color: Color8Bit,
}

impl Ligament {
    /// Length in canvas units, angle in degrees. Defaults to weight 10 and color
    /// `(235, 137, 52)`, as in Java.
    pub fn new(name: &str, length: f64, angle_degrees: f64) -> Self {
        Self {
            name: name.to_owned(),
            length,
            angle: angle_degrees,
            weight: 10.0,
            color: Color8Bit::new(235, 137, 52),
        }
    }

    pub fn with_weight(mut self, weight: f64) -> Self {
        self.weight = weight;
        self
    }

    pub fn with_color(mut self, color: Color8Bit) -> Self {
        self.color = color;
        self
    }
}

enum Kind {
    Root {
        x: f64,
        y: f64,
    },
    Line {
        length: f64,
        angle: f64,
        weight: f64,
        color: String,
    },
}

struct Node {
    name: String,
    kind: Kind,
    /// Topic path of this node's table once the canvas is published.
    path: Option<String>,
    children: Vec<usize>,
}

struct State {
    dims: [f64; 2],
    background: String,
    /// Index 0.. are roots and lines in creation order.
    nodes: Vec<Node>,
    roots: Vec<usize>,
    table: Option<String>,
    io: Option<SharedIo>,
}

impl State {
    fn child_table(&self, parent: usize, name: &str) -> Option<String> {
        self.nodes[parent]
            .path
            .as_ref()
            .map(|p| format!("{p}/{name}"))
    }

    fn write(&self, node: usize, key: &str, value: Value) {
        if let (Some(io), Some(path)) = (&self.io, &self.nodes[node].path) {
            io.borrow_mut().set(&format!("{path}/{key}"), &value);
        }
    }

    /// Publishes a node and everything under it.
    fn publish_node(&mut self, node: usize, path: String) {
        self.nodes[node].path = Some(path.clone());
        let io = self.io.clone().expect("canvas is published");
        {
            let mut io = io.borrow_mut();
            match &self.nodes[node].kind {
                Kind::Root { x, y } => {
                    io.set(&format!("{path}/x"), &Value::Double(*x));
                    io.set(&format!("{path}/y"), &Value::Double(*y));
                }
                Kind::Line {
                    length,
                    angle,
                    weight,
                    color,
                } => {
                    let kind = Value::String("line".to_owned());
                    io.publish_with_properties(
                        &format!("{path}/.type"),
                        "{\"SmartDashboard\":\"line\"}",
                        &kind,
                    );
                    io.set(&format!("{path}/.type"), &kind);
                    io.set(&format!("{path}/angle"), &Value::Double(*angle));
                    io.set(&format!("{path}/length"), &Value::Double(*length));
                    io.set(&format!("{path}/color"), &Value::String(color.clone()));
                    io.set(&format!("{path}/weight"), &Value::Double(*weight));
                }
            }
        }
        for child in self.nodes[node].children.clone() {
            let child_path = format!("{path}/{}", self.nodes[child].name);
            self.publish_node(child, child_path);
        }
    }

    fn append(&mut self, parent: usize, ligament: Ligament) -> usize {
        let taken = self.nodes[parent]
            .children
            .iter()
            .any(|c| self.nodes[*c].name == ligament.name);
        assert!(!taken, "Mechanism object names must be unique!");
        let index = self.nodes.len();
        let path = self.child_table(parent, &ligament.name);
        self.nodes.push(Node {
            name: ligament.name,
            kind: Kind::Line {
                length: ligament.length,
                angle: ligament.angle,
                weight: ligament.weight,
                color: ligament.color.hex(),
            },
            path: None,
            children: Vec::new(),
        });
        self.nodes[parent].children.push(index);
        if let Some(path) = path {
            self.publish_node(index, path);
        }
        index
    }
}

/// A canvas. Clones share state.
#[derive(Clone)]
pub struct Mechanism2d {
    state: Rc<RefCell<State>>,
}

impl Mechanism2d {
    /// A canvas with the default background `#000020`.
    pub fn new(width: f64, height: f64) -> Self {
        Self::with_background(width, height, Color8Bit::new(0, 0, 32))
    }

    pub fn with_background(width: f64, height: f64, background: Color8Bit) -> Self {
        Self {
            state: Rc::new(RefCell::new(State {
                dims: [width, height],
                background: background.hex(),
                nodes: Vec::new(),
                roots: Vec::new(),
                table: None,
                io: None,
            })),
        }
    }

    /// The root called `name`. An existing root keeps its position and ignores `x` and `y`.
    pub fn get_root(&self, name: &str, x: f64, y: f64) -> MechanismRoot2d {
        let mut state = self.state.borrow_mut();
        let existing = state
            .roots
            .iter()
            .copied()
            .find(|r| state.nodes[*r].name == name);
        let index = match existing {
            Some(i) => i,
            None => {
                let index = state.nodes.len();
                state.nodes.push(Node {
                    name: name.to_owned(),
                    kind: Kind::Root { x, y },
                    path: None,
                    children: Vec::new(),
                });
                state.roots.push(index);
                if let Some(table) = state.table.clone() {
                    state.publish_node(index, format!("{table}/{name}"));
                }
                index
            }
        };
        MechanismRoot2d {
            state: self.state.clone(),
            index,
        }
    }

    pub fn set_background_color(&self, color: Color8Bit) {
        let mut state = self.state.borrow_mut();
        state.background = color.hex();
        if let (Some(io), Some(table)) = (&state.io, &state.table) {
            io.borrow_mut().set(
                &format!("{table}/backgroundColor"),
                &Value::String(state.background.clone()),
            );
        }
    }
}

impl Sendable for Mechanism2d {
    fn init_sendable(&self, builder: &mut SendableBuilder) {
        builder.set_smartdashboard_type("Mechanism2d");
        let (table, io) = (builder.table().to_owned(), builder.io());
        let mut state = self.state.borrow_mut();
        {
            let mut io = io.borrow_mut();
            io.set(
                &format!("{table}/dims"),
                &Value::DoubleArray(state.dims.to_vec()),
            );
            io.set(
                &format!("{table}/backgroundColor"),
                &Value::String(state.background.clone()),
            );
        }
        state.table = Some(table.clone());
        state.io = Some(io);
        for root in state.roots.clone() {
            let name = state.nodes[root].name.clone();
            state.publish_node(root, format!("{table}/{name}"));
        }
    }

    fn identity(&self) -> usize {
        Rc::as_ptr(&self.state) as usize
    }
}

/// An anchor on the canvas that lines attach to.
#[derive(Clone)]
pub struct MechanismRoot2d {
    state: Rc<RefCell<State>>,
    index: usize,
}

impl MechanismRoot2d {
    pub fn set_position(&self, x: f64, y: f64) {
        let mut state = self.state.borrow_mut();
        state.nodes[self.index].kind = Kind::Root { x, y };
        state.write(self.index, "x", Value::Double(x));
        state.write(self.index, "y", Value::Double(y));
    }

    /// Attaches `ligament`, which must have a name no other child of this root has.
    ///
    /// # Panics
    /// When the name is taken, as Java throws.
    pub fn append(&self, ligament: Ligament) -> MechanismLigament2d {
        let index = self.state.borrow_mut().append(self.index, ligament);
        MechanismLigament2d {
            state: self.state.clone(),
            index,
        }
    }
}

/// A line on the canvas. Changes publish at once when the canvas is on the dashboard.
#[derive(Clone)]
pub struct MechanismLigament2d {
    state: Rc<RefCell<State>>,
    index: usize,
}

impl MechanismLigament2d {
    fn set(&self, key: &str, value: Value, apply: impl FnOnce(&mut Kind)) {
        let mut state = self.state.borrow_mut();
        apply(&mut state.nodes[self.index].kind);
        state.write(self.index, key, value);
    }

    fn read<T>(&self, f: impl FnOnce(&Kind) -> T) -> T {
        f(&self.state.borrow().nodes[self.index].kind)
    }

    /// Attaches `ligament` to this one.
    ///
    /// # Panics
    /// When the name is taken, as Java throws.
    pub fn append(&self, ligament: Ligament) -> MechanismLigament2d {
        let index = self.state.borrow_mut().append(self.index, ligament);
        MechanismLigament2d {
            state: self.state.clone(),
            index,
        }
    }

    pub fn set_angle(&self, degrees: f64) {
        self.set("angle", Value::Double(degrees), |k| {
            if let Kind::Line { angle, .. } = k {
                *angle = degrees;
            }
        });
    }

    pub fn set_angle_rotation(&self, angle: Rotation2d) {
        self.set_angle(angle.degrees());
    }

    pub fn set_length(&self, length: f64) {
        self.set("length", Value::Double(length), |k| {
            if let Kind::Line { length: l, .. } = k {
                *l = length;
            }
        });
    }

    pub fn set_line_weight(&self, weight: f64) {
        self.set("weight", Value::Double(weight), |k| {
            if let Kind::Line { weight: w, .. } = k {
                *w = weight;
            }
        });
    }

    pub fn set_color(&self, color: Color8Bit) {
        let hex = color.hex();
        self.set("color", Value::String(hex.clone()), |k| {
            if let Kind::Line { color: c, .. } = k {
                *c = hex;
            }
        });
    }

    pub fn angle(&self) -> f64 {
        self.read(|k| {
            if let Kind::Line { angle, .. } = k {
                *angle
            } else {
                0.0
            }
        })
    }

    pub fn length(&self) -> f64 {
        self.read(|k| {
            if let Kind::Line { length, .. } = k {
                *length
            } else {
                0.0
            }
        })
    }

    pub fn line_weight(&self) -> f64 {
        self.read(|k| {
            if let Kind::Line { weight, .. } = k {
                *weight
            } else {
                0.0
            }
        })
    }

    /// The color as `#RRGGBB`.
    pub fn color(&self) -> String {
        self.read(|k| match k {
            Kind::Line { color, .. } => color.clone(),
            Kind::Root { .. } => String::new(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dashboard::io::FakeIo;

    const TABLE: &str = "/SmartDashboard/Sim/LeftView";

    fn publish(mech: &Mechanism2d) -> Rc<RefCell<FakeIo>> {
        let fake = FakeIo::shared();
        let mut builder = SendableBuilder::new(TABLE, fake.clone());
        mech.init_sendable(&mut builder);
        fake
    }

    #[test]
    fn color_hex_is_upper_case_rrggbb() {
        assert_eq!(Color8Bit::new(235, 137, 52).hex(), "#EB8934");
        assert_eq!(Color8Bit::new(0, 0, 32).hex(), "#000020");
        assert_eq!(Color8Bit::ORANGE.hex(), "#FFA500");
        assert_eq!(Color8Bit::LIGHT_GRAY.hex(), "#D3D3D3");
    }

    #[test]
    fn canvas_publishes_type_dims_and_background() {
        let fake = publish(&Mechanism2d::new(3.0, 2.0));
        let io = fake.borrow();
        assert_eq!(io.text(&format!("{TABLE}/.type")), "Mechanism2d");
        assert_eq!(io.doubles(&format!("{TABLE}/dims")), [3.0, 2.0]);
        assert_eq!(io.text(&format!("{TABLE}/backgroundColor")), "#000020");
    }

    #[test]
    fn roots_and_ligaments_nest_under_their_parents() {
        let mech = Mechanism2d::with_background(3.0, 2.0, Color8Bit::LIGHT_GRAY);
        let root = mech.get_root("Root", 1.0, 0.5);
        let arm = root.append(
            Ligament::new("Arm", 0.4, 90.0)
                .with_weight(6.0)
                .with_color(Color8Bit::BLUE),
        );
        arm.append(Ligament::new("Wrist", 0.1, 45.0));
        let fake = publish(&mech);
        let io = fake.borrow();
        assert_eq!(io.text(&format!("{TABLE}/backgroundColor")), "#D3D3D3");
        assert_eq!(io.double(&format!("{TABLE}/Root/x")), 1.0);
        assert_eq!(io.double(&format!("{TABLE}/Root/y")), 0.5);
        let arm_table = format!("{TABLE}/Root/Arm");
        assert_eq!(io.text(&format!("{arm_table}/.type")), "line");
        assert_eq!(
            io.properties[&format!("{arm_table}/.type")],
            "{\"SmartDashboard\":\"line\"}"
        );
        assert_eq!(io.double(&format!("{arm_table}/angle")), 90.0);
        assert_eq!(io.double(&format!("{arm_table}/length")), 0.4);
        assert_eq!(io.double(&format!("{arm_table}/weight")), 6.0);
        assert_eq!(io.text(&format!("{arm_table}/color")), "#0000FF");
        assert_eq!(io.double(&format!("{arm_table}/Wrist/angle")), 45.0);
        assert_eq!(io.double(&format!("{arm_table}/Wrist/weight")), 10.0);
        assert_eq!(io.text(&format!("{arm_table}/Wrist/color")), "#EB8934");
    }

    #[test]
    fn setters_publish_once_published() {
        let mech = Mechanism2d::new(1.0, 1.0);
        let arm = mech
            .get_root("R", 0.0, 0.0)
            .append(Ligament::new("A", 1.0, 0.0));
        arm.set_angle(30.0);
        let fake = publish(&mech);
        let writes = fake.borrow().writes.len();
        arm.set_angle(60.0);
        arm.set_length(2.0);
        arm.set_line_weight(3.0);
        arm.set_color(Color8Bit::RED);
        let io = fake.borrow();
        assert_eq!(io.writes.len(), writes + 4);
        assert_eq!(io.double(&format!("{TABLE}/R/A/angle")), 60.0);
        assert_eq!(io.double(&format!("{TABLE}/R/A/length")), 2.0);
        assert_eq!(io.double(&format!("{TABLE}/R/A/weight")), 3.0);
        assert_eq!(io.text(&format!("{TABLE}/R/A/color")), "#FF0000");
        assert_eq!(
            (arm.angle(), arm.length(), arm.line_weight()),
            (60.0, 2.0, 3.0)
        );
        assert_eq!(arm.color(), "#FF0000");
    }

    #[test]
    fn setters_before_publish_are_kept_for_the_first_publish() {
        let mech = Mechanism2d::new(1.0, 1.0);
        let arm = mech
            .get_root("R", 0.0, 0.0)
            .append(Ligament::new("A", 1.0, 0.0));
        arm.set_angle_rotation(Rotation2d::from_degrees(30.0));
        let fake = publish(&mech);
        assert!((fake.borrow().double(&format!("{TABLE}/R/A/angle")) - 30.0).abs() < 1e-9);
    }

    #[test]
    fn nodes_added_after_publish_are_published_at_once() {
        let mech = Mechanism2d::new(1.0, 1.0);
        let fake = publish(&mech);
        let root = mech.get_root("Late", 2.0, 3.0);
        assert_eq!(fake.borrow().double(&format!("{TABLE}/Late/x")), 2.0);
        root.append(Ligament::new("L", 1.0, 5.0));
        assert_eq!(fake.borrow().double(&format!("{TABLE}/Late/L/angle")), 5.0);
        root.set_position(4.0, 5.0);
        assert_eq!(fake.borrow().double(&format!("{TABLE}/Late/y")), 5.0);
    }

    #[test]
    fn existing_root_is_returned_unchanged() {
        let mech = Mechanism2d::new(1.0, 1.0);
        mech.get_root("R", 1.0, 2.0);
        mech.get_root("R", 9.0, 9.0);
        let fake = publish(&mech);
        assert_eq!(fake.borrow().double(&format!("{TABLE}/R/x")), 1.0);
    }

    #[test]
    fn background_change_publishes() {
        let mech = Mechanism2d::new(1.0, 1.0);
        let fake = publish(&mech);
        mech.set_background_color(Color8Bit::WHITE);
        assert_eq!(
            fake.borrow().text(&format!("{TABLE}/backgroundColor")),
            "#FFFFFF"
        );
    }

    #[test]
    #[should_panic(expected = "Mechanism object names must be unique!")]
    fn duplicate_names_under_one_parent_panic() {
        let root = Mechanism2d::new(1.0, 1.0).get_root("R", 0.0, 0.0);
        root.append(Ligament::new("A", 1.0, 0.0));
        root.append(Ligament::new("A", 1.0, 0.0));
    }

    #[test]
    fn same_name_under_different_parents_is_fine() {
        let mech = Mechanism2d::new(1.0, 1.0);
        let a = mech
            .get_root("A", 0.0, 0.0)
            .append(Ligament::new("L", 1.0, 0.0));
        mech.get_root("B", 0.0, 0.0)
            .append(Ligament::new("L", 1.0, 0.0));
        a.append(Ligament::new("L", 1.0, 0.0));
    }
}

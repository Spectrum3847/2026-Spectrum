//! The Sendable protocol: an object describes its dashboard properties to a builder, and the
//! builder publishes getters and applies dashboard changes through setters on every update.

use super::io::SharedIo;
use crate::nt::Value;

/// An object a dashboard can show or control.
pub trait Sendable {
    /// Describes the object's properties. Called once when it is put on the dashboard.
    fn init_sendable(&self, builder: &mut SendableBuilder);

    /// Identifies the object, so putting the same one again under a key does nothing.
    /// Handles that share state return the same value.
    fn identity(&self) -> usize;
}

type Getter = Box<dyn FnMut() -> Value>;
type Setter = Box<dyn FnMut(Value)>;

struct Property {
    path: String,
    getter: Option<Getter>,
    setter: Option<Setter>,
}

/// Java's `SendableBuilderImpl`: properties live under one table path.
pub struct SendableBuilder {
    table: String,
    io: SharedIo,
    properties: Vec<Property>,
    controllable: bool,
    type_published: bool,
}

impl SendableBuilder {
    /// Publishes `<table>/.controllable` as false until [`start_listeners`](Self::start_listeners).
    pub fn new(table: impl Into<String>, io: SharedIo) -> Self {
        let table = table.into();
        {
            let mut io = io.borrow_mut();
            io.set_default(&format!("{table}/.controllable"), &Value::Boolean(false));
        }
        Self {
            table,
            io,
            properties: Vec::new(),
            controllable: false,
            type_published: false,
        }
    }

    /// Full path of this object's table.
    pub fn table(&self) -> &str {
        &self.table
    }

    pub fn io(&self) -> SharedIo {
        self.io.clone()
    }

    fn path(&self, key: &str) -> String {
        format!("{}/{key}", self.table)
    }

    /// Publishes `.type` as `{"SmartDashboard":"<kind>"}` topic properties plus the value.
    pub fn set_smartdashboard_type(&mut self, kind: &str) {
        let path = self.path(".type");
        let value = Value::String(kind.to_owned());
        let mut io = self.io.borrow_mut();
        if !self.type_published {
            io.publish_with_properties(
                &path,
                &format!("{{\"SmartDashboard\":\"{kind}\"}}"),
                &value,
            );
            self.type_published = true;
        }
        io.set(&path, &value);
    }

    /// Applies dashboard changes to setters from now on and tells dashboards the object is
    /// controllable.
    pub fn start_listeners(&mut self) {
        self.controllable = true;
        let path = self.path(".controllable");
        self.io.borrow_mut().set(&path, &Value::Boolean(true));
    }

    pub fn stop_listeners(&mut self) {
        self.controllable = false;
        let path = self.path(".controllable");
        self.io.borrow_mut().set(&path, &Value::Boolean(false));
    }

    /// Runs getters and setters. The robot loop calls this through `SmartDashboard`.
    pub fn update(&mut self) {
        for property in &mut self.properties {
            if self.controllable
                && let Some(setter) = &mut property.setter
            {
                let queued = self.io.borrow_mut().read_queue(&property.path);
                for value in queued {
                    setter(value);
                }
            }
            if let Some(getter) = &mut property.getter {
                let value = getter();
                self.io.borrow_mut().set(&property.path, &value);
            }
        }
    }

    /// Adds a property. A getter publishes its value each update. A setter receives values
    /// the dashboard writes; `proto` fixes the topic's type.
    pub fn add_property(
        &mut self,
        key: &str,
        proto: Value,
        getter: Option<Getter>,
        setter: Option<Setter>,
    ) {
        let path = self.path(key);
        {
            let mut io = self.io.borrow_mut();
            if getter.is_some() {
                io.publish(&path, &proto);
            }
            if setter.is_some() {
                io.subscribe(&path, &proto);
            }
        }
        self.properties.push(Property {
            path,
            getter,
            setter,
        });
    }

    /// Publishes a value once.
    pub fn publish_const(&mut self, key: &str, value: Value) {
        let path = self.path(key);
        self.io.borrow_mut().set(&path, &value);
    }

    pub fn publish_const_integer(&mut self, key: &str, value: i64) {
        self.publish_const(key, Value::Integer(value));
    }

    pub fn publish_const_string(&mut self, key: &str, value: &str) {
        self.publish_const(key, Value::String(value.to_owned()));
    }
}

macro_rules! typed_properties {
    ($($ty:ty, $variant:ident, $proto:expr, $both:ident, $getter:ident, $setter:ident;)*) => {
        impl SendableBuilder {
            $(
                pub fn $both(
                    &mut self,
                    key: &str,
                    mut getter: impl FnMut() -> $ty + 'static,
                    mut setter: impl FnMut($ty) + 'static,
                ) {
                    self.add_property(
                        key,
                        $proto,
                        Some(Box::new(move || Value::$variant(getter()))),
                        Some(Box::new(move |v| {
                            if let Value::$variant(v) = v {
                                setter(v);
                            }
                        })),
                    );
                }

                pub fn $getter(&mut self, key: &str, mut getter: impl FnMut() -> $ty + 'static) {
                    self.add_property(
                        key,
                        $proto,
                        Some(Box::new(move || Value::$variant(getter()))),
                        None,
                    );
                }

                pub fn $setter(&mut self, key: &str, mut setter: impl FnMut($ty) + 'static) {
                    self.add_property(
                        key,
                        $proto,
                        None,
                        Some(Box::new(move |v| {
                            if let Value::$variant(v) = v {
                                setter(v);
                            }
                        })),
                    );
                }
            )*
        }
    };
}

typed_properties! {
    bool, Boolean, Value::Boolean(false),
        add_boolean_property, add_boolean_getter, add_boolean_setter;
    f64, Double, Value::Double(0.0),
        add_double_property, add_double_getter, add_double_setter;
    String, String, Value::String(String::new()),
        add_string_property, add_string_getter, add_string_setter;
    Vec<String>, StringArray, Value::StringArray(Vec::new()),
        add_string_array_property, add_string_array_getter, add_string_array_setter;
    Vec<i64>, IntegerArray, Value::IntegerArray(Vec::new()),
        add_integer_array_property, add_integer_array_getter, add_integer_array_setter;
    Vec<f64>, DoubleArray, Value::DoubleArray(Vec::new()),
        add_double_array_property, add_double_array_getter, add_double_array_setter;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dashboard::io::FakeIo;
    use std::cell::{Cell, RefCell};
    use std::rc::Rc;

    #[test]
    fn controllable_starts_false_then_true() {
        let fake = FakeIo::shared();
        let mut b = SendableBuilder::new("/SmartDashboard/X", fake.clone());
        assert_eq!(
            fake.borrow().values["/SmartDashboard/X/.controllable"],
            Value::Boolean(false)
        );
        b.start_listeners();
        assert_eq!(
            fake.borrow().values["/SmartDashboard/X/.controllable"],
            Value::Boolean(true)
        );
    }

    #[test]
    fn type_is_published_with_properties() {
        let fake = FakeIo::shared();
        let mut b = SendableBuilder::new("/T", fake.clone());
        b.set_smartdashboard_type("Field2d");
        let io = fake.borrow();
        assert_eq!(io.text("/T/.type"), "Field2d");
        assert_eq!(
            io.properties["/T/.type"],
            "{\"SmartDashboard\":\"Field2d\"}"
        );
    }

    #[test]
    fn getter_publishes_each_update() {
        let fake = FakeIo::shared();
        let mut b = SendableBuilder::new("/T", fake.clone());
        let n = Rc::new(Cell::new(1.0));
        let n2 = n.clone();
        b.add_double_getter("n", move || n2.get());
        b.update();
        assert_eq!(fake.borrow().double("/T/n"), 1.0);
        n.set(2.0);
        b.update();
        assert_eq!(fake.borrow().double("/T/n"), 2.0);
    }

    #[test]
    fn setter_runs_only_when_controllable() {
        let fake = FakeIo::shared();
        let mut b = SendableBuilder::new("/T", fake.clone());
        let got = Rc::new(RefCell::new(Vec::new()));
        let got2 = got.clone();
        b.add_string_setter("sel", move |s| got2.borrow_mut().push(s));
        assert_eq!(fake.borrow().subscribed, ["/T/sel"]);
        fake.borrow_mut().push("/T/sel", Value::String("a".into()));
        b.update();
        assert!(got.borrow().is_empty());
        fake.borrow_mut().push("/T/sel", Value::String("b".into()));
        b.start_listeners();
        b.update();
        // Queued while not controllable, delivered once it is, as in Java.
        assert_eq!(*got.borrow(), ["a", "b"]);
    }

    #[test]
    fn setter_ignores_wrong_types() {
        let fake = FakeIo::shared();
        let mut b = SendableBuilder::new("/T", fake.clone());
        let got = Rc::new(Cell::new(0));
        let got2 = got.clone();
        b.add_double_setter("v", move |_| got2.set(got2.get() + 1));
        b.start_listeners();
        fake.borrow_mut().push("/T/v", Value::String("x".into()));
        fake.borrow_mut().push("/T/v", Value::Double(1.0));
        b.update();
        assert_eq!(got.get(), 1);
    }

    #[test]
    fn setters_run_before_getters() {
        let fake = FakeIo::shared();
        let mut b = SendableBuilder::new("/T", fake.clone());
        let state = Rc::new(RefCell::new(String::from("old")));
        let (s1, s2) = (state.clone(), state.clone());
        b.add_string_property(
            "p",
            move || s1.borrow().clone(),
            move |v| *s2.borrow_mut() = v,
        );
        b.start_listeners();
        fake.borrow_mut().push("/T/p", Value::String("new".into()));
        b.update();
        assert_eq!(fake.borrow().text("/T/p"), "new");
    }
}

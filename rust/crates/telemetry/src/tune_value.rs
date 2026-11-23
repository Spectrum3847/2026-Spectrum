//! A double published to SmartDashboard that a driver can edit at runtime. Nothing reads it
//! back on its own, so call [`TuneValue::update`] or [`TuneValue::supplier`] to get the
//! current value.

use std::cell::Cell;
use std::rc::Rc;
use wpilib::dashboard::smart_dashboard;

pub struct TuneValue {
    value: Cell<f64>,
    /// SmartDashboard key this value is published under.
    name: String,
}

impl TuneValue {
    /// Publishes `default_value` to SmartDashboard under `name`.
    pub fn new(name: &str, default_value: f64) -> Self {
        smart_dashboard::put_number(name, default_value);
        Self {
            value: Cell::new(default_value),
            name: name.to_owned(),
        }
    }

    pub fn value(&self) -> f64 {
        self.value.get()
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn update(&self) -> f64 {
        let value = smart_dashboard::get_number(&self.name, self.value.get());
        self.value.set(value);
        value
    }

    /// A supplier that reads the dashboard on every call.
    pub fn supplier(self: &Rc<Self>) -> impl Fn() -> f64 + 'static {
        let tune = Rc::clone(self);
        move || tune.update()
    }
}

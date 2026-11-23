use crate::command::{BooleanSupplier, Command};
use crate::env;
use crate::event_loop::EventLoop;
use crate::scheduler::CommandScheduler;
use std::rc::Rc;

/// A condition with command bindings, polled by an [`EventLoop`].
///
/// Each binding reads the condition once when it is created and treats that as
/// the previous value, so a condition already true at bind time raises no edge.
#[derive(Clone)]
pub struct Trigger {
    event_loop: Rc<EventLoop>,
    condition: Rc<dyn Fn() -> bool>,
}

impl BooleanSupplier for Trigger {
    fn into_supplier(self) -> Rc<dyn Fn() -> bool> {
        self.condition
    }
}

impl BooleanSupplier for &Trigger {
    fn into_supplier(self) -> Rc<dyn Fn() -> bool> {
        self.condition.clone()
    }
}

impl Trigger {
    /// Polled by the scheduler's default button loop.
    pub fn new(condition: impl BooleanSupplier) -> Self {
        let event_loop = CommandScheduler::get_instance().get_default_button_loop();
        Self::with_loop(event_loop, condition)
    }

    pub fn with_loop(event_loop: Rc<EventLoop>, condition: impl BooleanSupplier) -> Self {
        Self {
            event_loop,
            condition: condition.into_supplier(),
        }
    }

    pub fn get_as_boolean(&self) -> bool {
        (self.condition)()
    }

    fn add_binding(&self, mut body: impl FnMut(bool, bool) + 'static) {
        let condition = self.condition.clone();
        let mut previous = condition();
        self.event_loop.bind(move || {
            let current = condition();
            body(previous, current);
            previous = current;
        });
    }

    pub fn on_true(&self, command: Command) -> Trigger {
        self.add_binding(move |previous, current| {
            if !previous && current {
                CommandScheduler::get_instance().schedule(&command);
            }
        });
        self.clone()
    }

    pub fn on_false(&self, command: Command) -> Trigger {
        self.add_binding(move |previous, current| {
            if previous && !current {
                CommandScheduler::get_instance().schedule(&command);
            }
        });
        self.clone()
    }

    pub fn while_true(&self, command: Command) -> Trigger {
        self.add_binding(move |previous, current| {
            if !previous && current {
                CommandScheduler::get_instance().schedule(&command);
            } else if previous && !current {
                command.cancel();
            }
        });
        self.clone()
    }

    pub fn and(&self, other: impl BooleanSupplier) -> Trigger {
        let (a, b) = (self.condition.clone(), other.into_supplier());
        Self::with_loop(self.event_loop.clone(), move || a() && b())
    }

    pub fn or(&self, other: impl BooleanSupplier) -> Trigger {
        let (a, b) = (self.condition.clone(), other.into_supplier());
        Self::with_loop(self.event_loop.clone(), move || a() || b())
    }

    pub fn negate(&self) -> Trigger {
        let a = self.condition.clone();
        Self::with_loop(self.event_loop.clone(), move || !a())
    }
}

/// Triggers on the robot mode from the installed [`RobotState`](crate::RobotState).
pub struct RobotModeTriggers;

impl RobotModeTriggers {
    pub fn autonomous() -> Trigger {
        Trigger::new(|| env::robot_state().is_autonomous_enabled())
    }

    pub fn teleop() -> Trigger {
        Trigger::new(|| env::robot_state().is_teleop_enabled())
    }

    pub fn disabled() -> Trigger {
        Trigger::new(|| env::robot_state().is_disabled())
    }

    pub fn test() -> Trigger {
        Trigger::new(|| env::robot_state().is_test_enabled())
    }
}

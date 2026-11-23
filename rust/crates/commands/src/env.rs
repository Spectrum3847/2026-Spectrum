//! Clock and robot-mode sources. The `wpilib` crate installs the real ones at startup.

use std::cell::RefCell;
use std::rc::Rc;
use std::time::Instant;

/// Robot mode as the driver station reports it.
pub trait RobotState {
    fn is_disabled(&self) -> bool;
    fn is_autonomous_enabled(&self) -> bool;
    fn is_teleop_enabled(&self) -> bool;
    fn is_test_enabled(&self) -> bool;
    fn is_simulation(&self) -> bool {
        false
    }
}

/// Default until a real source is installed: enabled in teleop.
struct EnabledTeleop;

impl RobotState for EnabledTeleop {
    fn is_disabled(&self) -> bool {
        false
    }
    fn is_autonomous_enabled(&self) -> bool {
        false
    }
    fn is_teleop_enabled(&self) -> bool {
        true
    }
    fn is_test_enabled(&self) -> bool {
        false
    }
}

type Clock = Rc<dyn Fn() -> f64>;

thread_local! {
    static CLOCK: RefCell<Clock> = RefCell::new(monotonic_clock());
    static ROBOT_STATE: RefCell<Rc<dyn RobotState>> = RefCell::new(Rc::new(EnabledTeleop));
}

fn monotonic_clock() -> Clock {
    let start = Instant::now();
    Rc::new(move || start.elapsed().as_secs_f64())
}

/// Replaces the time source (seconds, monotonic) for the calling thread.
pub fn set_clock(clock: impl Fn() -> f64 + 'static) {
    CLOCK.with(|c| *c.borrow_mut() = Rc::new(clock));
}

pub fn now_seconds() -> f64 {
    let clock = CLOCK.with(|c| c.borrow().clone());
    clock()
}

/// Replaces the robot-mode source for the calling thread.
pub fn set_robot_state(state: impl RobotState + 'static) {
    ROBOT_STATE.with(|s| *s.borrow_mut() = Rc::new(state));
}

pub fn robot_state() -> Rc<dyn RobotState> {
    ROBOT_STATE.with(|s| s.borrow().clone())
}

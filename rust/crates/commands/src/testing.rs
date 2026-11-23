//! Fake clock and robot mode for tests, here so other crates can use them too.

use crate::env::{self, RobotState};
use std::cell::Cell;
use std::rc::Rc;

/// Time source set by hand. Installing replaces the thread's clock.
#[derive(Clone, Default)]
pub struct FakeClock(Rc<Cell<f64>>);

impl FakeClock {
    pub fn install() -> Self {
        let clock = Self::default();
        let time = clock.0.clone();
        env::set_clock(move || time.get());
        clock
    }

    pub fn advance(&self, seconds: f64) {
        self.0.set(self.0.get() + seconds);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Disabled,
    Autonomous,
    Teleop,
    Test,
}

/// Robot mode set by hand. Installing replaces the thread's source and starts in teleop.
#[derive(Clone)]
pub struct FakeRobotState(Rc<Cell<(Mode, bool)>>);

impl FakeRobotState {
    pub fn install() -> Self {
        let state = Self(Rc::new(Cell::new((Mode::Teleop, false))));
        env::set_robot_state(state.clone());
        state
    }

    pub fn set_mode(&self, mode: Mode) {
        self.0.set((mode, self.0.get().1));
    }

    pub fn set_simulation(&self, simulation: bool) {
        self.0.set((self.0.get().0, simulation));
    }
}

impl RobotState for FakeRobotState {
    fn is_disabled(&self) -> bool {
        self.0.get().0 == Mode::Disabled
    }
    fn is_autonomous_enabled(&self) -> bool {
        self.0.get().0 == Mode::Autonomous
    }
    fn is_teleop_enabled(&self) -> bool {
        self.0.get().0 == Mode::Teleop
    }
    fn is_test_enabled(&self) -> bool {
        self.0.get().0 == Mode::Test
    }
    fn is_simulation(&self) -> bool {
        self.0.get().1
    }
}

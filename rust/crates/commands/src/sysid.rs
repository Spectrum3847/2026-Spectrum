//! Port of WPILib `wpilibj2.command.sysid.SysIdRoutine`. State recording goes through a
//! callback, so this crate needs no telemetry dependency.

use crate::timer::Timer;
use crate::{Command, Commands, Subsystem, SubsystemId};
use std::cell::RefCell;
use std::rc::Rc;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Reverse,
}

/// What `SysIdRoutine` reports to its state recorder. Names are what WPILib writes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    QuasistaticForward,
    QuasistaticReverse,
    DynamicForward,
    DynamicReverse,
    None,
}

impl State {
    pub fn name(self) -> &'static str {
        match self {
            Self::QuasistaticForward => "quasistatic-forward",
            Self::QuasistaticReverse => "quasistatic-reverse",
            Self::DynamicForward => "dynamic-forward",
            Self::DynamicReverse => "dynamic-reverse",
            Self::None => "none",
        }
    }
}

/// `SysIdRoutine.Config`. Default is WPILib's: ramp 1 V/s, step 7 V, timeout 10 s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RoutineConfig {
    pub ramp_rate_volts_per_second: f64,
    pub step_voltage: f64,
    pub timeout_seconds: f64,
}

impl Default for RoutineConfig {
    fn default() -> Self {
        Self {
            ramp_rate_volts_per_second: 1.0,
            step_voltage: 7.0,
            timeout_seconds: 10.0,
        }
    }
}

/// WPILib's `SysIdRoutine` without the `SysIdRoutineLog` data path: the robot passes no `log`
/// callback, so only the state recorder and `drive` exist.
pub struct SysIdRoutine {
    config: RoutineConfig,
    drive: Rc<dyn Fn(f64)>,
    record_state: Rc<dyn Fn(State)>,
    subsystem: SubsystemId,
    name: String,
}

impl SysIdRoutine {
    pub fn new(
        config: RoutineConfig,
        record_state: impl Fn(State) + 'static,
        drive: impl Fn(f64) + 'static,
        subsystem: &impl Subsystem,
    ) -> Self {
        Self {
            config,
            drive: Rc::new(drive),
            record_state: Rc::new(record_state),
            subsystem: subsystem.id(),
            name: subsystem.name(),
        }
    }

    /// Slow voltage ramp.
    pub fn quasistatic(&self, direction: Direction) -> Command {
        let (state, output_sign) = match direction {
            Direction::Forward => (State::QuasistaticForward, 1.0),
            Direction::Reverse => (State::QuasistaticReverse, -1.0),
        };
        let ramp_rate = self.config.ramp_rate_volts_per_second;
        let timer = Rc::new(RefCell::new(Timer::new()));
        let run_timer = timer.clone();
        self.build(state, timer, move || {
            output_sign * run_timer.borrow().get() * ramp_rate
        })
    }

    /// Constant voltage step.
    pub fn dynamic(&self, direction: Direction) -> Command {
        let (state, output_sign) = match direction {
            Direction::Forward => (State::DynamicForward, 1.0),
            Direction::Reverse => (State::DynamicReverse, -1.0),
        };
        let step_voltage = self.config.step_voltage;
        let timer = Rc::new(RefCell::new(Timer::new()));
        self.build(state, timer, move || output_sign * step_voltage)
    }

    fn build(
        &self,
        state: State,
        timer: Rc<RefCell<Timer>>,
        output_volts: impl Fn() -> f64 + 'static,
    ) -> Command {
        let requirements = [self.subsystem];
        let start_timer = timer.clone();
        let (drive, record_state) = (self.drive.clone(), self.record_state.clone());
        let (end_drive, end_record_state) = (self.drive.clone(), self.record_state.clone());
        Commands::run_once(move || start_timer.borrow_mut().restart(), &requirements)
            .and_then(Commands::run(
                move || {
                    drive(output_volts());
                    record_state(state);
                },
                &requirements,
            ))
            .finally_do(move || {
                end_drive(0.0);
                end_record_state(State::None);
                timer.borrow_mut().stop();
            })
            .with_name(format!("sysid-{}-{}", state.name(), self.name))
            .with_timeout(self.config.timeout_seconds)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::FakeClock;
    use crate::{CommandScheduler, SubsystemBase};

    struct Fake(SubsystemBase);

    impl Subsystem for Fake {
        fn id(&self) -> SubsystemId {
            self.0.id()
        }

        fn name(&self) -> String {
            "Swerve".to_owned()
        }
    }

    type Log<T> = Rc<RefCell<Vec<T>>>;

    fn routine(driven: &Log<f64>, states: &Log<State>) -> SysIdRoutine {
        let fake = Fake(SubsystemBase::new("Swerve"));
        let (driven, states) = (driven.clone(), states.clone());
        SysIdRoutine::new(
            RoutineConfig {
                ramp_rate_volts_per_second: 2.0,
                step_voltage: 4.0,
                timeout_seconds: 10.0,
            },
            move |state| states.borrow_mut().push(state),
            move |volts| driven.borrow_mut().push(volts),
            &fake,
        )
    }

    #[test]
    fn dynamic_drives_the_signed_step_then_zero_when_cancelled() {
        for (direction, sign) in [(Direction::Forward, 1.0), (Direction::Reverse, -1.0)] {
            let driven: Log<f64> = Rc::default();
            let states: Log<State> = Rc::default();
            let command = routine(&driven, &states).dynamic(direction);
            let scheduler = CommandScheduler::get_instance();
            scheduler.schedule(&command);
            scheduler.run(); // the restart step finishes
            scheduler.run(); // the drive step starts
            scheduler.run();
            assert_eq!(*driven.borrow(), vec![4.0 * sign, 4.0 * sign]);
            scheduler.cancel(&command);
            assert_eq!(driven.borrow().last().copied(), Some(0.0));
            assert_eq!(states.borrow().last().copied(), Some(State::None));
        }
    }

    #[test]
    fn quasistatic_ramps_with_elapsed_time() {
        let driven: Log<f64> = Rc::default();
        let states: Log<State> = Rc::default();
        let clock = FakeClock::install();
        let command = routine(&driven, &states).quasistatic(Direction::Reverse);
        let scheduler = CommandScheduler::get_instance();
        scheduler.schedule(&command);
        scheduler.run(); // timer restarts
        clock.advance(0.5);
        scheduler.run();
        clock.advance(1.0);
        scheduler.run();
        let driven_values = driven.borrow().clone();
        assert_eq!(driven_values.len(), 2);
        assert!((driven_values[0] + 1.0).abs() < 1e-9, "{driven_values:?}");
        assert!((driven_values[1] + 3.0).abs() < 1e-9, "{driven_values:?}");
        assert_eq!(
            states.borrow().first().copied(),
            Some(State::QuasistaticReverse)
        );
        scheduler.cancel(&command);
    }

    #[test]
    fn state_names_match_wpilib() {
        assert_eq!(State::QuasistaticForward.name(), "quasistatic-forward");
        assert_eq!(State::QuasistaticReverse.name(), "quasistatic-reverse");
        assert_eq!(State::DynamicForward.name(), "dynamic-forward");
        assert_eq!(State::DynamicReverse.name(), "dynamic-reverse");
        assert_eq!(State::None.name(), "none");
    }
}

//! Java `TimedRobot` and `IterativeRobotBase`: the robot trait and the periodic loop.

use crate::driver_station::{self, ControlWord};
use crate::hal::{self, HalError};
use crate::hooks::RobotMode;
use crate::notifier::{Notifier, panic_message};
use crate::robot_controller;
use crate::shutdown;
use crate::watchdog::Watchdog;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use std::sync::{Arc, Mutex, PoisonError, mpsc};
use std::thread;
use std::time::Duration;

/// Default loop period, seconds.
pub const DEFAULT_PERIOD: f64 = 0.02;

/// What a robot program implements. Every method defaults to doing nothing.
pub trait Robot {
    fn robot_init(&mut self) {}
    fn driver_station_connected(&mut self) {}
    fn simulation_init(&mut self) {}

    fn disabled_init(&mut self) {}
    fn autonomous_init(&mut self) {}
    fn teleop_init(&mut self) {}
    fn test_init(&mut self) {}

    fn robot_periodic(&mut self) {}
    fn simulation_periodic(&mut self) {}
    fn disabled_periodic(&mut self) {}
    fn autonomous_periodic(&mut self) {}
    fn teleop_periodic(&mut self) {}
    fn test_periodic(&mut self) {}

    fn disabled_exit(&mut self) {}
    fn autonomous_exit(&mut self) {}
    fn teleop_exit(&mut self) {}
    fn test_exit(&mut self) {}
}

/// Ends [`TimedRobot::run`] from any thread, including from inside the loop.
#[derive(Clone, Default)]
pub struct Stopper(Arc<StopState>);

#[derive(Default)]
struct StopState {
    stopped: AtomicBool,
    /// Notifier handle of the running loop, 0 before the loop starts.
    notifier: AtomicI32,
}

impl Stopper {
    pub fn stop(&self) {
        // SeqCst pairs with the loop's check so a stop is never missed for a period.
        self.0.stopped.store(true, Ordering::SeqCst);
        let handle = self.0.notifier.load(Ordering::SeqCst);
        if handle != 0 {
            hal::stop_notifier(handle);
        }
    }

    fn is_stopped(&self) -> bool {
        self.0.stopped.load(Ordering::SeqCst)
    }
}

/// Everything the loop needs from outside. The real one is the HAL; tests use a fake.
trait Platform {
    fn refresh_data(&mut self);
    fn control_word(&self) -> ControlWord;
    fn observe_starting(&mut self);
    fn observe_mode(&mut self, mode: RobotMode);
    fn is_simulation(&self) -> bool;
    fn sim_periodic_before(&mut self);
    fn sim_periodic_after(&mut self);
    fn now_micros(&self) -> u64;
    fn warn(&mut self, message: &str);
    fn update_alarm(&mut self, micros: u64);
    /// Blocks until the alarm. Returns 0 when the loop must end.
    fn wait_alarm(&mut self) -> u64;
}

enum Task {
    /// One pass of the iterative robot logic.
    Loop,
    User(Box<dyn FnMut()>),
}

type PostPeriodic = (&'static str, Box<dyn FnMut()>);

struct Callback {
    task: Task,
    period_us: i64,
    expiration_us: i64,
}

impl Callback {
    fn new(task: Task, start_us: i64, period_us: i64, offset_us: i64, now_us: i64) -> Self {
        assert!(
            period_us > 0,
            "periodic callback period must be at least 1 microsecond"
        );
        let expiration_us =
            start_us + offset_us + period_us + (now_us - start_us) / period_us * period_us;
        Self {
            task,
            period_us,
            expiration_us,
        }
    }

    /// Moves the deadline past `current`, skipping periods that were missed.
    fn reschedule(&mut self, current_us: i64) {
        self.expiration_us +=
            self.period_us + (current_us - self.expiration_us) / self.period_us * self.period_us;
    }
}

/// A robot on a fixed-period loop. Runs `Robot` methods in the order Java's `TimedRobot` does.
pub struct TimedRobot<R: Robot> {
    robot: R,
    period_s: f64,
    start_us: i64,
    callbacks: Vec<Callback>,
    post_periodic: Vec<PostPeriodic>,
    watchdog: Watchdog,
    last_mode: Option<RobotMode>,
    called_ds_connected: bool,
    loop_start_us: u64,
    stopper: Stopper,
}

impl<R: Robot> TimedRobot<R> {
    /// A robot on the default 20 ms period.
    pub fn new(robot: R) -> Self {
        Self::with_period(robot, DEFAULT_PERIOD)
    }

    pub fn with_period(robot: R, period_s: f64) -> Self {
        Self::build(robot, period_s, robot_controller::fpga_time())
    }

    fn build(robot: R, period_s: f64, start_us: u64) -> Self {
        let start_us = start_us as i64;
        let period_us = (period_s * 1e6) as i64;
        let first = Callback::new(Task::Loop, start_us, period_us, 0, start_us);
        Self {
            robot,
            period_s,
            start_us,
            callbacks: vec![first],
            post_periodic: Vec::new(),
            watchdog: Watchdog::new(period_s),
            last_mode: None,
            called_ds_connected: false,
            loop_start_us: 0,
            stopper: Stopper::default(),
        }
    }

    /// Seconds a loop pass may take before the overrun report. Defaults to the period.
    pub fn set_watchdog_timeout(&mut self, seconds: f64) {
        self.watchdog
            .set_timeout(seconds, robot_controller::fpga_time());
    }

    /// Runs `callback` every `period_s` seconds on the loop thread, `offset_s` after the
    /// loop's own schedule. Callbacks due at the same time run in the order they were added.
    pub fn add_periodic(&mut self, callback: impl FnMut() + 'static, period_s: f64, offset_s: f64) {
        self.add_periodic_at(callback, period_s, offset_s, robot_controller::fpga_time());
    }

    fn add_periodic_at(
        &mut self,
        callback: impl FnMut() + 'static,
        period_s: f64,
        offset_s: f64,
        now_us: u64,
    ) {
        let (period_us, offset_us) = ((period_s * 1e6) as i64, (offset_s * 1e6) as i64);
        let task = Task::User(Box::new(callback));
        self.callbacks.push(Callback::new(
            task,
            self.start_us,
            period_us,
            offset_us,
            now_us as i64,
        ));
    }

    /// Runs `hook` after `robot_periodic` on every pass and records it as `name` in the
    /// overrun report. Dashboards update here.
    pub fn add_post_periodic(&mut self, name: &'static str, hook: impl FnMut() + 'static) {
        self.post_periodic.push((name, Box::new(hook)));
    }

    pub fn stopper(&self) -> Stopper {
        self.stopper.clone()
    }

    /// FPGA time at which the current pass started, microseconds.
    pub fn loop_start_time(&self) -> u64 {
        self.loop_start_us
    }

    pub fn period(&self) -> f64 {
        self.period_s
    }

    pub fn robot(&self) -> &R {
        &self.robot
    }

    /// Runs `robot_init`, then the loop until the [`Stopper`] fires or the HAL stops the
    /// notifier.
    pub fn run(&mut self) -> Result<(), HalError> {
        let mut platform = HalPlatform::new(&self.stopper)?;
        hal::report_usage(hal::RESOURCE_FRAMEWORK, hal::INSTANCE_FRAMEWORK_TIMED);
        self.run_with(&mut platform);
        Ok(())
    }

    fn run_with(&mut self, platform: &mut impl Platform) {
        self.robot.robot_init();
        if platform.is_simulation() {
            self.robot.simulation_init();
        }
        println!("********** Robot program startup complete **********");
        platform.observe_starting();
        loop {
            if self.stopper.is_stopped() {
                break;
            }
            let next = self.next_callback();
            let mut callback = self.callbacks.remove(next);
            platform.update_alarm(callback.expiration_us as u64);
            let current = platform.wait_alarm() as i64;
            if current == 0 {
                self.callbacks.push(callback);
                break;
            }
            self.loop_start_us = platform.now_micros();
            self.run_task(&mut callback.task, platform);
            callback.reschedule(current);
            self.callbacks.push(callback);
            while let Some(i) = self.due_callback(current) {
                let mut due = self.callbacks.remove(i);
                self.run_task(&mut due.task, platform);
                due.reschedule(current);
                self.callbacks.push(due);
            }
        }
    }

    /// Earliest deadline; the oldest entry wins a tie.
    fn next_callback(&self) -> usize {
        let mut best = 0;
        for (i, cb) in self.callbacks.iter().enumerate() {
            if cb.expiration_us < self.callbacks[best].expiration_us {
                best = i;
            }
        }
        best
    }

    fn due_callback(&self, current_us: i64) -> Option<usize> {
        let i = self.next_callback();
        (self.callbacks[i].expiration_us <= current_us).then_some(i)
    }

    fn run_task(&mut self, task: &mut Task, platform: &mut impl Platform) {
        match task {
            Task::Loop => self.loop_func(platform),
            Task::User(f) => f(),
        }
    }

    fn epoch(&mut self, name: &str, platform: &impl Platform) {
        self.watchdog.add_epoch(name, platform.now_micros());
    }

    /// One pass of Java's `IterativeRobotBase.loopFunc`.
    fn loop_func(&mut self, platform: &mut impl Platform) {
        platform.refresh_data();
        self.watchdog.enable(platform.now_micros());
        let word = platform.control_word();
        let mode = RobotMode::from_control_word(word);

        if !self.called_ds_connected && word.ds_attached() {
            self.called_ds_connected = true;
            self.robot.driver_station_connected();
        }

        if self.last_mode != Some(mode) {
            match self.last_mode {
                Some(RobotMode::Disabled) => self.robot.disabled_exit(),
                Some(RobotMode::Autonomous) => self.robot.autonomous_exit(),
                Some(RobotMode::Teleop) => self.robot.teleop_exit(),
                Some(RobotMode::Test) => self.robot.test_exit(),
                None => {}
            }
            match mode {
                RobotMode::Disabled => {
                    self.robot.disabled_init();
                    self.epoch("disabledInit()", platform);
                }
                RobotMode::Autonomous => {
                    self.robot.autonomous_init();
                    self.epoch("autonomousInit()", platform);
                }
                RobotMode::Teleop => {
                    self.robot.teleop_init();
                    self.epoch("teleopInit()", platform);
                }
                RobotMode::Test => {
                    self.robot.test_init();
                    self.epoch("testInit()", platform);
                }
            }
            self.last_mode = Some(mode);
        }

        platform.observe_mode(mode);
        match mode {
            RobotMode::Disabled => {
                self.robot.disabled_periodic();
                self.epoch("disabledPeriodic()", platform);
            }
            RobotMode::Autonomous => {
                self.robot.autonomous_periodic();
                self.epoch("autonomousPeriodic()", platform);
            }
            RobotMode::Teleop => {
                self.robot.teleop_periodic();
                self.epoch("teleopPeriodic()", platform);
            }
            RobotMode::Test => {
                self.robot.test_periodic();
                self.epoch("testPeriodic()", platform);
            }
        }

        self.robot.robot_periodic();
        self.epoch("robotPeriodic()", platform);

        let mut hooks = std::mem::take(&mut self.post_periodic);
        for (name, hook) in &mut hooks {
            hook();
            self.epoch(name, platform);
        }
        self.post_periodic = hooks;

        if platform.is_simulation() {
            platform.sim_periodic_before();
            self.robot.simulation_periodic();
            platform.sim_periodic_after();
            self.epoch("simulationPeriodic()", platform);
        }

        let now = platform.now_micros();
        let expired = self.watchdog.disable(now, &mut |m| platform.warn(m));
        if expired {
            platform.warn(&format!("Loop time of {}s overrun\n", self.period_s));
            let now = platform.now_micros();
            self.watchdog.print_epochs(now, &mut |m| platform.warn(m));
        }
    }
}

struct HalPlatform {
    handle: i32,
    stopper: Stopper,
}

impl HalPlatform {
    fn new(stopper: &Stopper) -> Result<Self, HalError> {
        let handle = hal::initialize_notifier()?;
        hal::set_notifier_name(handle, "TimedRobot");
        stopper.0.notifier.store(handle, Ordering::SeqCst);
        Ok(Self {
            handle,
            stopper: stopper.clone(),
        })
    }
}

impl Drop for HalPlatform {
    fn drop(&mut self) {
        // Clear the shared handle first so a later stop() does not use a freed one.
        self.stopper.0.notifier.store(0, Ordering::SeqCst);
        hal::stop_notifier(self.handle);
        hal::clean_notifier(self.handle);
    }
}

impl Platform for HalPlatform {
    fn refresh_data(&mut self) {
        driver_station::refresh_data();
    }

    fn control_word(&self) -> ControlWord {
        driver_station::control_word()
    }

    fn observe_starting(&mut self) {
        hal::observe_user_program_starting();
    }

    fn observe_mode(&mut self, mode: RobotMode) {
        match mode {
            RobotMode::Disabled => hal::observe_user_program_disabled(),
            RobotMode::Autonomous => hal::observe_user_program_autonomous(),
            RobotMode::Teleop => hal::observe_user_program_teleop(),
            RobotMode::Test => hal::observe_user_program_test(),
        }
    }

    fn is_simulation(&self) -> bool {
        hal::is_simulation()
    }

    fn sim_periodic_before(&mut self) {
        hal::sim_periodic_before();
    }

    fn sim_periodic_after(&mut self) {
        hal::sim_periodic_after();
    }

    fn now_micros(&self) -> u64 {
        robot_controller::fpga_time()
    }

    fn warn(&mut self, message: &str) {
        driver_station::report_warning(message);
    }

    fn update_alarm(&mut self, micros: u64) {
        hal::update_notifier_alarm(self.handle, micros);
    }

    fn wait_alarm(&mut self) -> u64 {
        hal::wait_for_notifier_alarm(self.handle)
    }
}

/// Starts the robot program, like Java's `RobotBase.startRobot`: initializes the HAL, builds
/// the robot, runs it, and shuts the HAL down when the loop ends. Build panics and loop panics
/// are reported to the driver station and end the program.
///
/// When a simulation extension needs the main thread (the Sim GUI), the robot is built and
/// run on a new thread, and the main thread runs the extension until the user closes it.
pub fn start_robot<R: Robot>(build: impl FnOnce() -> TimedRobot<R> + Send + 'static) {
    println!("********** Robot program starting **********");
    hal::initialize();
    driver_station::refresh_data();
    hal::report_usage(hal::RESOURCE_LANGUAGE, hal::INSTANCE_LANGUAGE_RUST);
    if !Notifier::set_hal_thread_priority(true, 40) {
        driver_station::report_warning("Setting HAL Notifier RT priority to 40 failed");
    }

    if hal::has_main() {
        run_with_main_thread(build);
    } else {
        run_robot(build, &StopSlot::default());
    }
    // Threads and notifiers must stop before the HAL's static destructors run.
    shutdown::run();
    hal::shutdown();
}

/// Lets the main thread stop a robot that another thread is still building or running.
#[derive(Default)]
struct StopSlot(Mutex<SlotState>);

#[derive(Default)]
struct SlotState {
    stopper: Option<Stopper>,
    quit: bool,
}

impl StopSlot {
    /// Called by the robot thread once the robot exists. Stops it at once if quit came first.
    fn set(&self, stopper: Stopper) {
        let mut state = self.0.lock().unwrap_or_else(PoisonError::into_inner);
        if state.quit {
            stopper.stop();
        }
        state.stopper = Some(stopper);
    }

    fn quit(&self) {
        let mut state = self.0.lock().unwrap_or_else(PoisonError::into_inner);
        state.quit = true;
        if let Some(stopper) = &state.stopper {
            stopper.stop();
        }
    }
}

/// Java gives the robot thread one second to finish after the extension closes.
const ROBOT_EXIT_WAIT: Duration = Duration::from_secs(1);

fn run_with_main_thread<R: Robot>(build: impl FnOnce() -> TimedRobot<R> + Send + 'static) {
    let slot = Arc::new(StopSlot::default());
    let (done_tx, done_rx) = mpsc::channel();
    let robot_slot = slot.clone();
    let spawned = thread::Builder::new()
        .name("robot main".into())
        .spawn(move || {
            run_robot(build, &robot_slot);
            hal::exit_main();
            let _ = done_tx.send(());
        });
    if let Err(e) = spawned {
        driver_station::report_error(&format!("Could not start the robot thread: {e}"));
        return;
    }
    hal::run_main();
    slot.quit();
    // Detached on timeout: the process exits right after, as in Java.
    let _ = done_rx.recv_timeout(ROBOT_EXIT_WAIT);
}

/// Builds and runs the robot on the calling thread, which becomes the loop thread.
fn run_robot<R: Robot>(build: impl FnOnce() -> TimedRobot<R>, slot: &StopSlot) {
    match catch_unwind(AssertUnwindSafe(build)) {
        Err(panic) => {
            driver_station::report_error(&format!(
                "Unhandled exception instantiating robot: {}",
                panic_message(panic.as_ref())
            ));
            driver_station::report_error(
                "The robot program quit unexpectedly. This is usually due to a code error.",
            );
        }
        Ok(mut robot) => {
            let stopper = robot.stopper();
            slot.set(stopper.clone());
            let outcome = catch_unwind(AssertUnwindSafe(|| robot.run()));
            if !stopper.is_stopped() {
                report_unexpected_exit(outcome);
            }
        }
    }
}

fn report_unexpected_exit(outcome: std::thread::Result<Result<(), HalError>>) {
    driver_station::report_warning(
        "The robot program quit unexpectedly. This is usually due to a code error.",
    );
    match outcome {
        Err(panic) => {
            driver_station::report_error(&format!(
                "Unhandled exception: {}",
                panic_message(panic.as_ref())
            ));
            driver_station::report_error(
                "The robot loop (or functions called by it) should have handled the exception above.",
            );
        }
        Ok(Err(e)) => driver_station::report_error(&format!("Could not start the robot loop: {e}")),
        Ok(Ok(())) => driver_station::report_error("Unexpected return from the robot loop."),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::{Cell, RefCell};
    use std::rc::Rc;

    type Log = Rc<RefCell<Vec<String>>>;

    struct Recorder {
        log: Log,
        clock: Rc<Cell<u64>>,
        slow_periodic_us: u64,
    }

    macro_rules! record {
        ($($name:ident),*) => {
            $(fn $name(&mut self) {
                self.log.borrow_mut().push(stringify!($name).to_owned());
            })*
        };
    }

    impl Robot for Recorder {
        record!(
            robot_init,
            driver_station_connected,
            simulation_init,
            disabled_init,
            autonomous_init,
            teleop_init,
            test_init,
            simulation_periodic,
            disabled_periodic,
            autonomous_periodic,
            teleop_periodic,
            test_periodic,
            disabled_exit,
            autonomous_exit,
            teleop_exit,
            test_exit
        );

        fn robot_periodic(&mut self) {
            self.log.borrow_mut().push("robot_periodic".to_owned());
            self.clock.set(self.clock.get() + self.slow_periodic_us);
        }
    }

    struct Fake {
        log: Log,
        clock: Rc<Cell<u64>>,
        words: Vec<ControlWord>,
        pass: usize,
        simulation: bool,
        max_waits: usize,
        waits: usize,
        alarm: u64,
        warnings: Vec<String>,
    }

    impl Fake {
        fn new(log: &Log, clock: &Rc<Cell<u64>>, words: Vec<ControlWord>) -> Self {
            Self {
                log: Rc::clone(log),
                clock: Rc::clone(clock),
                words,
                pass: 0,
                simulation: false,
                max_waits: usize::MAX,
                waits: 0,
                alarm: 0,
                warnings: Vec::new(),
            }
        }
    }

    impl Platform for Fake {
        fn refresh_data(&mut self) {
            self.pass += 1;
        }
        fn control_word(&self) -> ControlWord {
            let i = (self.pass - 1).min(self.words.len() - 1);
            self.words[i]
        }
        fn observe_starting(&mut self) {
            self.log.borrow_mut().push("observe_starting".to_owned());
        }
        fn observe_mode(&mut self, mode: RobotMode) {
            self.log.borrow_mut().push(format!("observe_{mode:?}"));
        }
        fn is_simulation(&self) -> bool {
            self.simulation
        }
        fn sim_periodic_before(&mut self) {
            self.log.borrow_mut().push("sim_before".to_owned());
        }
        fn sim_periodic_after(&mut self) {
            self.log.borrow_mut().push("sim_after".to_owned());
        }
        fn now_micros(&self) -> u64 {
            self.clock.get()
        }
        fn warn(&mut self, message: &str) {
            self.warnings.push(message.to_owned());
        }
        fn update_alarm(&mut self, micros: u64) {
            self.alarm = micros;
        }
        fn wait_alarm(&mut self) -> u64 {
            self.waits += 1;
            if self.waits > self.max_waits {
                return 0;
            }
            self.clock.set(self.clock.get().max(self.alarm));
            self.alarm
        }
    }

    const START: u64 = 1_000_000;

    fn setup(slow_periodic_us: u64) -> (Log, Rc<Cell<u64>>, TimedRobot<Recorder>) {
        let log = Log::default();
        let clock = Rc::new(Cell::new(START));
        let robot = Recorder {
            log: Rc::clone(&log),
            clock: Rc::clone(&clock),
            slow_periodic_us,
        };
        (log, clock, TimedRobot::build(robot, 0.02, START))
    }

    fn word(enabled: bool, auto: bool, test: bool) -> ControlWord {
        ControlWord::new(enabled, auto, test, true)
    }

    fn entries(log: &Log) -> Vec<String> {
        log.borrow().clone()
    }

    #[test]
    fn modes_run_exit_init_periodic_in_order() {
        let (log, clock, mut robot) = setup(0);
        let words = vec![
            word(false, false, false),
            word(true, true, false),
            word(true, false, false),
        ];
        let mut fake = Fake::new(&log, &clock, words);
        fake.max_waits = 3;
        robot.run_with(&mut fake);
        let log = entries(&log);
        let from_first_pass = log
            .iter()
            .position(|e| e == "driver_station_connected")
            .unwrap();
        assert_eq!(
            &log[from_first_pass..],
            [
                "driver_station_connected",
                "disabled_init",
                "observe_Disabled",
                "disabled_periodic",
                "robot_periodic",
                "disabled_exit",
                "autonomous_init",
                "observe_Autonomous",
                "autonomous_periodic",
                "robot_periodic",
                "autonomous_exit",
                "teleop_init",
                "observe_Teleop",
                "teleop_periodic",
                "robot_periodic",
            ]
        );
    }

    #[test]
    fn startup_order_and_test_mode_exit() {
        let (log, clock, mut robot) = setup(0);
        let mut fake = Fake::new(
            &log,
            &clock,
            vec![word(true, false, true), word(false, false, false)],
        );
        fake.max_waits = 2;
        robot.run_with(&mut fake);
        let log = entries(&log);
        assert_eq!(&log[..2], ["robot_init", "observe_starting"]);
        assert!(log.contains(&"test_init".to_owned()));
        let exit = log.iter().position(|e| e == "test_exit").unwrap();
        assert_eq!(log[exit + 1], "disabled_init");
    }

    #[test]
    fn same_mode_does_not_reinit() {
        let (log, clock, mut robot) = setup(0);
        let mut fake = Fake::new(&log, &clock, vec![word(true, false, false)]);
        fake.max_waits = 4;
        robot.run_with(&mut fake);
        let log = entries(&log);
        assert_eq!(log.iter().filter(|e| *e == "teleop_init").count(), 1);
        assert_eq!(log.iter().filter(|e| *e == "teleop_periodic").count(), 4);
        assert!(!log.contains(&"teleop_exit".to_owned()));
    }

    #[test]
    fn driver_station_connected_fires_once() {
        let (log, clock, mut robot) = setup(0);
        let words = vec![
            ControlWord::default(),
            word(false, false, false),
            word(false, false, false),
        ];
        let mut fake = Fake::new(&log, &clock, words);
        fake.max_waits = 3;
        robot.run_with(&mut fake);
        let log = entries(&log);
        assert_eq!(
            log.iter()
                .filter(|e| *e == "driver_station_connected")
                .count(),
            1
        );
        let connected = log
            .iter()
            .position(|e| e == "driver_station_connected")
            .unwrap();
        let first_periodic = log.iter().position(|e| e == "robot_periodic").unwrap();
        assert!(connected > first_periodic, "no DS on the first pass");
    }

    #[test]
    fn simulation_hooks_wrap_simulation_periodic() {
        let (log, clock, mut robot) = setup(0);
        let mut fake = Fake::new(&log, &clock, vec![word(false, false, false)]);
        fake.simulation = true;
        fake.max_waits = 1;
        robot.run_with(&mut fake);
        let log = entries(&log);
        assert_eq!(log[0], "robot_init");
        assert_eq!(log[1], "simulation_init");
        let tail = &log[log.len() - 4..];
        assert_eq!(
            tail,
            [
                "robot_periodic",
                "sim_before",
                "simulation_periodic",
                "sim_after"
            ]
        );
    }

    #[test]
    fn callback_expiration_skips_past_periods() {
        let cb = Callback::new(Task::Loop, 1_000_000, 20_000, 0, 1_000_000);
        assert_eq!(cb.expiration_us, 1_020_000);
        let late = Callback::new(Task::Loop, 1_000_000, 20_000, 5_000, 1_055_000);
        assert_eq!(late.expiration_us, 1_000_000 + 5_000 + 20_000 + 2 * 20_000);
    }

    #[test]
    fn reschedule_lands_after_the_current_time() {
        let mut cb = Callback::new(Task::Loop, 0, 20_000, 0, 0);
        cb.reschedule(20_000);
        assert_eq!(cb.expiration_us, 40_000);
        cb.reschedule(95_000);
        assert_eq!(cb.expiration_us, 100_000);
    }

    #[test]
    fn periodic_callbacks_run_at_their_own_rate() {
        let (log, clock, mut robot) = setup(0);
        let fast = Rc::new(Cell::new(0));
        let counter = Rc::clone(&fast);
        robot.add_periodic_at(move || counter.set(counter.get() + 1), 0.005, 0.0, START);
        let mut fake = Fake::new(&log, &clock, vec![word(false, false, false)]);
        fake.max_waits = 20;
        robot.run_with(&mut fake);
        let passes = entries(&log)
            .iter()
            .filter(|e| *e == "robot_periodic")
            .count();
        // 100 ms of schedule: five 20 ms passes and twenty 5 ms ticks.
        assert_eq!(passes, 5);
        assert_eq!(fast.get(), 20);
    }

    #[test]
    fn post_periodic_hooks_run_after_robot_periodic() {
        let (log, clock, mut robot) = setup(0);
        let hook_log = Rc::clone(&log);
        robot.add_post_periodic("Dash.update()", move || {
            hook_log.borrow_mut().push("hook".to_owned())
        });
        let mut fake = Fake::new(&log, &clock, vec![word(false, false, false)]);
        fake.max_waits = 1;
        robot.run_with(&mut fake);
        let log = entries(&log);
        let rp = log.iter().position(|e| e == "robot_periodic").unwrap();
        assert_eq!(log[rp + 1], "hook");
    }

    #[test]
    fn slow_pass_prints_overrun_and_epochs() {
        let (log, clock, mut robot) = setup(30_000);
        let mut fake = Fake::new(&log, &clock, vec![word(false, false, false)]);
        fake.max_waits = 1;
        robot.run_with(&mut fake);
        assert_eq!(fake.warnings.len(), 2);
        assert_eq!(fake.warnings[0], "Loop time of 0.02s overrun\n");
        assert!(fake.warnings[1].contains("\tdisabledPeriodic(): 0.000000s\n"));
        assert!(fake.warnings[1].contains("\trobotPeriodic(): 0.030000s\n"));
    }

    #[test]
    fn fast_pass_prints_nothing() {
        let (log, clock, mut robot) = setup(5_000);
        let mut fake = Fake::new(&log, &clock, vec![word(false, false, false)]);
        fake.max_waits = 3;
        robot.run_with(&mut fake);
        assert!(fake.warnings.is_empty());
    }

    #[test]
    fn stopper_ends_the_loop_before_the_next_wait() {
        let (log, clock, mut robot) = setup(0);
        let stopper = robot.stopper();
        let hook_log = Rc::clone(&log);
        robot.add_post_periodic("stop", move || {
            hook_log.borrow_mut().push("stop".to_owned());
            // No notifier handle is set, so this only raises the flag.
            stopper.stop();
        });
        let mut fake = Fake::new(&log, &clock, vec![word(false, false, false)]);
        robot.run_with(&mut fake);
        assert_eq!(fake.waits, 1);
    }

    #[test]
    fn quit_before_the_robot_exists_stops_it_when_set() {
        let slot = StopSlot::default();
        slot.quit();
        let stopper = Stopper::default();
        slot.set(stopper.clone());
        assert!(stopper.is_stopped());
    }

    #[test]
    fn quit_after_the_robot_exists_stops_it() {
        let slot = StopSlot::default();
        let stopper = Stopper::default();
        slot.set(stopper.clone());
        assert!(!stopper.is_stopped());
        slot.quit();
        assert!(stopper.is_stopped());
    }

    #[test]
    #[should_panic(expected = "at least 1 microsecond")]
    fn zero_period_is_rejected() {
        let _ = Callback::new(Task::Loop, 0, 0, 0, 0);
    }
}

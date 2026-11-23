//! Shared high-frequency simulation loop. Mechanism sims register a step callback and are
//! driven from a single `Notifier` thread far faster than the 50 Hz main loop, so the
//! simulated TalonFX closed loops advance close to their real 1 kHz cadence and PID gains
//! behave like hardware. This mirrors the swerve drivetrain's sim thread.
//!
//! Sim-only: registration is a no-op on real hardware, so no thread starts on the roboRIO.
//!
//! Canvas objects are single-threaded, so the steps only update physics and the numbers to
//! draw. Each sim registers a draw callback with [`register_draw`], and the robot loop calls
//! [`draw_all`] once per period. Java draws from the sim thread.

use phoenix6::utils;
use std::cell::RefCell;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use wpilib::driver_station;
use wpilib::notifier::Notifier;
use wpilib::shutdown;

const PERIOD_SECS: f64 = 1.0 / 200.0; // 200 Hz

type Step = Arc<dyn Fn(f64) + Send + Sync>;

static STEPS: Mutex<Vec<Step>> = Mutex::new(Vec::new());
static STATE: Mutex<State> = Mutex::new(State {
    notifier: None,
    last_time: 0.0,
});

struct State {
    notifier: Option<Notifier>,
    last_time: f64,
}

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Registers a step callback, invoked with the real elapsed time in seconds since the
/// previous tick. The first registration starts the shared thread. No-op outside simulation.
pub fn register(step: impl Fn(f64) + Send + Sync + 'static) {
    if !utils::is_simulation() {
        return;
    }
    let mut state = lock(&STATE);
    lock(&STEPS).push(Arc::new(step));
    if state.notifier.is_none() {
        state.last_time = utils::current_time_seconds();
        let started = Notifier::new(tick).expect("notifier");
        started.start_periodic(PERIOD_SECS);
        state.notifier = Some(started);
        shutdown::register(stop);
    }
}

/// Stops the shared thread and releases its HAL notifier. A later `register` starts a new one.
pub fn stop() {
    let notifier = lock(&STATE).notifier.take();
    // Dropped outside the lock: it joins the thread, and a tick takes the lock.
    drop(notifier);
}

fn tick() {
    let now = utils::current_time_seconds();
    let dt = {
        let mut state = lock(&STATE);
        let dt = now - state.last_time;
        state.last_time = now;
        dt
    };
    run_steps(dt);
}

/// Runs every step. A panicking step is reported and the rest still run.
fn run_steps(dt: f64) {
    // Copy the list so a step may register another one.
    let steps: Vec<Step> = lock(&STEPS).clone();
    for step in steps {
        if let Err(payload) = catch_unwind(AssertUnwindSafe(|| step(dt))) {
            let reason = payload
                .downcast_ref::<&str>()
                .map(|s| (*s).to_owned())
                .or_else(|| payload.downcast_ref::<String>().cloned())
                .unwrap_or_default();
            driver_station::report_error(&format!("SimLoop step failed: {reason}"));
        }
    }
}

thread_local! {
    static DRAWS: RefCell<Vec<Box<dyn Fn()>>> = const { RefCell::new(Vec::new()) };
}

/// Registers a callback that [`draw_all`] runs. Call on the robot loop thread.
pub fn register_draw(draw: impl Fn() + 'static) {
    DRAWS.with(|draws| draws.borrow_mut().push(Box::new(draw)));
}

/// Redraws every registered sim from its latest physics state. Call once per robot period,
/// on the thread that built the sims.
pub fn draw_all() {
    DRAWS.with(|draws| {
        for draw in draws.borrow().iter() {
            draw();
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU32, Ordering};

    #[test]
    fn stop_without_a_notifier_is_idempotent() {
        stop();
        stop();
        assert!(lock(&STATE).notifier.is_none());
    }

    // The step list is global, so tests add unique steps and never remove them.
    #[test]
    fn steps_run_in_order_with_dt_and_a_panic_does_not_stop_the_rest() {
        static BEFORE: AtomicU32 = AtomicU32::new(0);
        static AFTER: AtomicU32 = AtomicU32::new(0);
        static LAST_DT: Mutex<f64> = Mutex::new(0.0);
        lock(&STEPS).push(Arc::new(|_| {
            BEFORE.fetch_add(1, Ordering::SeqCst);
        }));
        lock(&STEPS).push(Arc::new(|_| panic!("boom")));
        lock(&STEPS).push(Arc::new(|dt| {
            AFTER.fetch_add(1, Ordering::SeqCst);
            *lock(&LAST_DT) = dt;
        }));
        run_steps(0.005);
        run_steps(0.006);
        assert_eq!(BEFORE.load(Ordering::SeqCst), 2);
        assert_eq!(AFTER.load(Ordering::SeqCst), 2);
        assert_eq!(*lock(&LAST_DT), 0.006);
    }

    #[test]
    fn draw_all_runs_every_registered_draw_on_this_thread() {
        let count = std::rc::Rc::new(std::cell::Cell::new(0));
        let c1 = count.clone();
        let c2 = count.clone();
        register_draw(move || c1.set(c1.get() + 1));
        register_draw(move || c2.set(c2.get() + 10));
        draw_all();
        draw_all();
        assert_eq!(count.get(), 22);
    }
}

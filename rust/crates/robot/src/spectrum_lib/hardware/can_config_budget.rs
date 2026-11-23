//! A cap on how long boot may spend on CAN device configuration that is not working.
//!
//! Every Phoenix configuration call blocks until the device answers or the timeout
//! expires. That is invisible when the bus is healthy and quadratic in frustration when it
//! is not: each of the roughly two dozen CANivore devices is configured with several such
//! calls, plus retry loops on top of them, and every one of those waits out its full
//! timeout against a device that is not there.
//!
//! In the 2026-09-19 Chezy practice match the CANivore bus went down mid-match on a cut
//! wire. The roboRIO was restarted from the Driver Station and the code looked like it
//! never came back: robot init took 64.0 s against 16.9 s in the match before it, and the
//! rio then sat at 93-99% CPU with 58-77% of loops over 25 ms. It had in fact come back,
//! it was just unusable, and the match was over before anyone could tell the difference.
//!
//! This module bounds that. Callers run their configuration through [`run`] and consult
//! [`exhausted`] before retrying or before making calls that are merely nice to have. Once
//! the cumulative time lost to failed configuration passes [`BUDGET_SECONDS`], the budget
//! is spent: retries stop, optional calls are skipped, and an alert names it. Boot then
//! completes with a dead bus in roughly the time it takes to fail once per device instead
//! of ten times.
//!
//! Open question, 2026-09-19. This bounds the boot, but it is not the whole of what
//! happened after that restart. With the bus already dead, CPU sat at a normal 40 to 50
//! percent for 20 s after init, then went to 95 to 100 percent at exactly 30.0 s after init
//! completed and stayed there, with 58 to 77 percent of loops over 25 ms. Nothing in the
//! Java loop changed at that moment (vision and the scheduler were each under 20 ms) and
//! the DataLog writer thread was starved too, so the load is native or a background
//! service, not the robot loop. Bench test to find it: unplug the CANivore, boot, watch
//! `System/CpuPercent` at plus 30 s, then try disabling the Phoenix diagnostics server
//! (`Unmanaged.setPhoenixDiagnosticsStartTime(-1)`) and the SignalLogger to see which one
//! it is. Log: `logs/matches/FRC_20260919_034444.wpilog`.
//!
//! The cap is deliberately cause-agnostic. A cut wire, a powered-down bus, a wrong bus name
//! and a missing CANivore all present identically here, and the correct response to all of
//! them is the same: stop waiting and come up.
//!
//! State is per thread, and all configuration runs on the loop thread.

use phoenix6::StatusCode;
use std::cell::{OnceCell, RefCell};
use wpilib::dashboard::{Alert, AlertType};
use wpilib::timer;

/// Cumulative seconds of failed configuration allowed before retries are abandoned.
///
/// Generous enough that a single slow device, or a handful of first-call timeouts on a
/// healthy bus, never trips it; small enough that a fully dead bus costs a few seconds
/// rather than the better part of a minute.
pub const BUDGET_SECONDS: f64 = 3.0;

/// Timeout for a single boot configuration call, in seconds.
///
/// Phoenix's own default is 0.050 s. This is deliberately the same: the fix here is not a
/// shorter individual wait, which would risk failing on a healthy-but-busy bus, but
/// refusing to repeat the wait once it is clear nothing is answering.
pub const CALL_TIMEOUT_SECONDS: f64 = 0.050;

/// Retry count used while the budget holds.
pub const MAX_ATTEMPTS: i32 = 10;

#[derive(Debug, Default)]
struct Budget {
    spent_seconds: f64,
    failed_calls: i32,
}

impl Budget {
    fn exhausted(&self) -> bool {
        self.spent_seconds >= BUDGET_SECONDS
    }

    /// Charges one failed call. Returns the alert text on the call that spends the budget.
    fn charge(&mut self, elapsed_seconds: f64, name: &str) -> Option<String> {
        let was_exhausted = self.exhausted();
        self.spent_seconds += elapsed_seconds;
        self.failed_calls += 1;
        (!was_exhausted && self.exhausted()).then(|| {
            format!(
                "CAN config budget spent: {:.1} s lost over {} failed device configs (first \
                 noticed at {}). Retries are now off so boot can finish -- expect mechanisms \
                 to be unconfigured. Check the CAN bus wiring and power.",
                self.spent_seconds, self.failed_calls, name
            )
        })
    }
}

thread_local! {
    static BUDGET: RefCell<Budget> = RefCell::new(Budget::default());
    // Created on first use: an Alert needs the dashboard natives, and an inactive alert
    // shows nothing.
    static EXHAUSTED_ALERT: OnceCell<Alert> = const { OnceCell::new() };
}

/// Whether the budget is spent and callers should stop retrying: true once failed
/// configuration has cost more than [`BUDGET_SECONDS`].
pub fn exhausted() -> bool {
    BUDGET.with(|b| b.borrow().exhausted())
}

/// Attempts to retry, given how many attempts a caller should make for one configuration:
/// [`MAX_ATTEMPTS`] while the budget holds, otherwise 1.
pub fn max_attempts() -> i32 {
    if exhausted() { 1 } else { MAX_ATTEMPTS }
}

/// Runs one configuration call with the bounded timeout and charges its cost to the budget
/// when it fails.
///
/// A successful call costs nothing, however long it took: time spent talking to a device
/// that is really there is time well spent. Only failures are charged, because only
/// failures repeat.
///
/// `name` is the device or mechanism being configured, for the alert text. `call` takes
/// the timeout in seconds. Returns the status the call returned.
pub fn run(name: &str, call: impl FnOnce(f64) -> StatusCode) -> StatusCode {
    run_with(name, call, timer::fpga_timestamp, raise_alert)
}

fn run_with(
    name: &str,
    call: impl FnOnce(f64) -> StatusCode,
    clock: impl Fn() -> f64,
    raise_alert: impl FnOnce(&str),
) -> StatusCode {
    let start = clock();
    let result = call(CALL_TIMEOUT_SECONDS);
    if !result.is_ok() {
        let text = BUDGET.with(|b| b.borrow_mut().charge(clock() - start, name));
        if let Some(text) = text {
            raise_alert(&text);
        }
    }
    result
}

fn raise_alert(text: &str) {
    EXHAUSTED_ALERT.with(|cell| {
        let alert = cell.get_or_init(|| Alert::new("", AlertType::Error));
        alert.set_text(text);
        alert.set(true);
    });
}

/// Seconds charged to the budget so far. Logged so a boot that nearly tripped the cap is
/// visible before the one that trips it.
pub fn get_spent_seconds() -> f64 {
    BUDGET.with(|b| b.borrow().spent_seconds)
}

/// Number of failed configuration calls so far.
pub fn get_failed_calls() -> i32 {
    BUDGET.with(|b| b.borrow().failed_calls)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;

    /// Runs one call that takes `duration` fake seconds and returns `status`.
    fn timed_call(
        name: &str,
        now: &Cell<f64>,
        duration: f64,
        status: StatusCode,
        alerts: &RefCell<Vec<String>>,
    ) -> StatusCode {
        run_with(
            name,
            |_| {
                now.set(now.get() + duration);
                status
            },
            || now.get(),
            |text| alerts.borrow_mut().push(text.to_owned()),
        )
    }

    #[test]
    fn a_fresh_budget_allows_full_retries() {
        assert!(!exhausted());
        assert_eq!(max_attempts(), MAX_ATTEMPTS);
        assert_eq!(get_spent_seconds(), 0.0);
        assert_eq!(get_failed_calls(), 0);
    }

    #[test]
    fn the_call_gets_the_bounded_timeout() {
        let seen = Cell::new(0.0);
        run_with(
            "x",
            |timeout| {
                seen.set(timeout);
                StatusCode::OK
            },
            || 0.0,
            |_| {},
        );
        assert_eq!(seen.get(), CALL_TIMEOUT_SECONDS);
    }

    #[test]
    fn a_successful_call_costs_nothing_however_long_it_took() {
        let now = Cell::new(0.0);
        let alerts = RefCell::new(Vec::new());
        timed_call("Talon 1", &now, 10.0, StatusCode::OK, &alerts);
        assert_eq!(get_spent_seconds(), 0.0);
        assert_eq!(get_failed_calls(), 0);
        assert!(!exhausted());
    }

    #[test]
    fn a_failed_call_charges_its_elapsed_time() {
        let now = Cell::new(100.0);
        let alerts = RefCell::new(Vec::new());
        let status = timed_call("Talon 1", &now, 0.5, StatusCode::InvalidNetwork, &alerts);
        assert_eq!(status, StatusCode::InvalidNetwork);
        assert!((get_spent_seconds() - 0.5).abs() < 1e-9);
        assert_eq!(get_failed_calls(), 1);
        assert!(alerts.borrow().is_empty());
    }

    #[test]
    fn the_budget_is_spent_at_the_limit_and_retries_drop_to_one() {
        let now = Cell::new(0.0);
        let alerts = RefCell::new(Vec::new());
        timed_call("a", &now, 2.9, StatusCode::InvalidNetwork, &alerts);
        assert!(!exhausted());
        assert_eq!(max_attempts(), MAX_ATTEMPTS);
        timed_call("b", &now, 0.1, StatusCode::InvalidNetwork, &alerts);
        assert!(exhausted());
        assert_eq!(max_attempts(), 1);
    }

    #[test]
    fn the_alert_is_raised_once_and_names_the_call_that_spent_the_budget() {
        let now = Cell::new(0.0);
        let alerts = RefCell::new(Vec::new());
        timed_call("Talon 7", &now, 3.0, StatusCode::InvalidNetwork, &alerts);
        timed_call("Talon 8", &now, 1.0, StatusCode::InvalidNetwork, &alerts);
        let alerts = alerts.borrow();
        assert_eq!(alerts.len(), 1);
        assert_eq!(
            alerts[0],
            "CAN config budget spent: 3.0 s lost over 1 failed device configs (first noticed \
             at Talon 7). Retries are now off so boot can finish -- expect mechanisms to be \
             unconfigured. Check the CAN bus wiring and power."
        );
    }

    #[test]
    fn calls_after_exhaustion_keep_counting() {
        let now = Cell::new(0.0);
        let alerts = RefCell::new(Vec::new());
        timed_call("a", &now, 3.5, StatusCode::InvalidNetwork, &alerts);
        timed_call("b", &now, 0.05, StatusCode::InvalidNetwork, &alerts);
        assert_eq!(get_failed_calls(), 2);
        assert!((get_spent_seconds() - 3.55).abs() < 1e-9);
    }

    #[test]
    fn warnings_from_the_device_count_as_failures() {
        // Java: isOK() is true only for status 0, so a warning status is charged.
        let now = Cell::new(0.0);
        let alerts = RefCell::new(Vec::new());
        timed_call("a", &now, 1.0, StatusCode::TaskIsBusy, &alerts);
        assert_eq!(get_failed_calls(), 1);
    }
}

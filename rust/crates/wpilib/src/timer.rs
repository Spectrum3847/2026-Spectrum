//! Java `Timer`: a stopwatch on the robot time source.

use crate::robot_controller;
use std::time::Duration;

/// A stopwatch. Arithmetic is in milliseconds as `f64`, the same as Java, so results match.
#[derive(Debug, Clone)]
pub struct Timer {
    clock: fn() -> u64,
    start_ms: f64,
    accumulated: f64,
    running: bool,
}

impl Default for Timer {
    fn default() -> Self {
        Self::new()
    }
}

impl Timer {
    pub fn new() -> Self {
        Self::with_clock(robot_controller::time)
    }

    /// A timer on a custom microsecond clock.
    pub fn with_clock(clock: fn() -> u64) -> Self {
        let mut timer = Self {
            clock,
            start_ms: 0.0,
            accumulated: 0.0,
            running: false,
        };
        timer.reset();
        timer
    }

    fn clock_ms(&self) -> f64 {
        (self.clock)() as f64 / 1000.0
    }

    /// Seconds accumulated while running.
    pub fn get(&self) -> f64 {
        if self.running {
            self.accumulated + (self.clock_ms() - self.start_ms) / 1000.0
        } else {
            self.accumulated
        }
    }

    pub fn reset(&mut self) {
        self.accumulated = 0.0;
        self.start_ms = self.clock_ms();
    }

    pub fn start(&mut self) {
        if !self.running {
            self.start_ms = self.clock_ms();
            self.running = true;
        }
    }

    pub fn restart(&mut self) {
        if self.running {
            self.stop();
        }
        self.reset();
        self.start();
    }

    pub fn stop(&mut self) {
        self.accumulated = self.get();
        self.running = false;
    }

    pub fn has_elapsed(&self, seconds: f64) -> bool {
        self.get() >= seconds
    }

    /// True once per elapsed period, moving the start forward by exactly one period.
    pub fn advance_if_elapsed(&mut self, seconds: f64) -> bool {
        if self.get() >= seconds {
            self.start_ms += seconds * 1000.0;
            true
        } else {
            false
        }
    }

    pub fn is_running(&self) -> bool {
        self.running
    }
}

/// Seconds from the robot time source. Java `Timer.getTimestamp`.
pub fn timestamp() -> f64 {
    robot_controller::time_seconds()
}

/// FPGA seconds. Java `Timer.getFPGATimestamp`.
pub fn fpga_timestamp() -> f64 {
    robot_controller::fpga_time() as f64 / 1e6
}

/// Seconds left in the current match period, or -1. Java `Timer.getMatchTime`.
pub fn match_time() -> f64 {
    crate::driver_station::match_time()
}

/// Sleeps the calling thread. Truncates to whole milliseconds, as Java does.
pub fn delay(seconds: f64) {
    std::thread::sleep(Duration::from_millis((seconds * 1e3) as u64));
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;

    thread_local!(static NOW_US: Cell<u64> = const { Cell::new(0) });

    fn now() -> u64 {
        NOW_US.with(Cell::get)
    }

    fn advance_ms(ms: u64) {
        NOW_US.with(|n| n.set(n.get() + ms * 1000));
    }

    fn timer() -> Timer {
        NOW_US.with(|n| n.set(5_000_000));
        Timer::with_clock(now)
    }

    #[test]
    fn stopped_timer_reads_zero() {
        let t = timer();
        advance_ms(500);
        assert_eq!(t.get(), 0.0);
        assert!(!t.is_running());
    }

    #[test]
    fn runs_and_stops() {
        let mut t = timer();
        t.start();
        advance_ms(1500);
        assert_eq!(t.get(), 1.5);
        t.stop();
        advance_ms(1000);
        assert_eq!(t.get(), 1.5);
        t.start();
        advance_ms(500);
        assert_eq!(t.get(), 2.0);
    }

    #[test]
    fn start_twice_keeps_original_start() {
        let mut t = timer();
        t.start();
        advance_ms(100);
        t.start();
        advance_ms(100);
        assert_eq!(t.get(), 0.2);
    }

    #[test]
    fn reset_clears_without_stopping() {
        let mut t = timer();
        t.start();
        advance_ms(1000);
        t.reset();
        assert!(t.is_running());
        assert_eq!(t.get(), 0.0);
        advance_ms(250);
        assert_eq!(t.get(), 0.25);
    }

    #[test]
    fn restart_begins_from_zero() {
        let mut t = timer();
        t.start();
        advance_ms(1000);
        t.restart();
        assert!(t.is_running());
        assert_eq!(t.get(), 0.0);
    }

    #[test]
    fn has_elapsed_is_inclusive() {
        let mut t = timer();
        t.start();
        advance_ms(999);
        assert!(!t.has_elapsed(1.0));
        advance_ms(1);
        assert!(t.has_elapsed(1.0));
    }

    #[test]
    fn advance_if_elapsed_keeps_the_remainder() {
        let mut t = timer();
        t.start();
        advance_ms(2500);
        assert!(t.advance_if_elapsed(1.0));
        assert_eq!(t.get(), 1.5);
        assert!(t.advance_if_elapsed(1.0));
        assert!(!t.advance_if_elapsed(1.0));
        assert_eq!(t.get(), 0.5);
    }
}

//! Counts robot loops so work can be done "once per loop" or "every Nth loop" without a
//! timer.
//!
//! `Robot::robot_periodic` calls [`next`] first thing. Anything that wants to do something
//! once per loop remembers the count it last ran at and compares; anything that wants a
//! slower cadence uses [`every`]. Both are an integer compare, so they are safe to call
//! from tight per-loop paths such as status-signal reads.
//!
//! Loop thread only: the count is per thread. Unit tests never call [`next`], so
//! once-per-loop caches there refresh exactly once.

use std::cell::Cell;
use telemetry::telemetry;

thread_local! {
    static COUNT: Cell<u64> = const { Cell::new(0) };
}

/// Advances to the next loop. Called once, at the top of `robot_periodic`. Also tells the
/// `telemetry` crate, which cannot read this module and needs the count for its slow tier.
pub fn next() {
    let count = COUNT.get() + 1;
    COUNT.set(count);
    telemetry::set_loop_count(count);
}

/// The current loop number. Starts at 0 and increases by one per [`next`].
pub fn count() -> u64 {
    COUNT.get()
}

/// True on every `n`th loop, so a 20 ms loop with `n = 5` is 10 Hz. All callers in the
/// same loop agree, which keeps related values on the same records. Values below 1 mean
/// every loop.
pub fn every(n: i32) -> bool {
    n <= 1 || count().is_multiple_of(n as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn count_starts_at_zero_and_advances_by_one() {
        assert_eq!(count(), 0);
        next();
        assert_eq!(count(), 1);
        next();
        next();
        assert_eq!(count(), 3);
    }

    #[test]
    fn every_fires_on_multiples_of_n() {
        // Loop 0 is a multiple of everything.
        assert!(every(5));
        let mut fired = Vec::new();
        for _ in 0..10 {
            next();
            if every(5) {
                fired.push(count());
            }
        }
        assert_eq!(fired, [5, 10]);
    }

    #[test]
    fn small_n_means_every_loop() {
        next();
        assert!(every(1));
        assert!(every(0));
        assert!(every(-3));
    }
}

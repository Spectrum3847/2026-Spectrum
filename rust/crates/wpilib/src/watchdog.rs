//! Java `Tracer` and `Watchdog`, for the loop overrun report.
//!
//! Both take the current FPGA time in microseconds as an argument, so they run against a fake
//! clock in tests.

/// Records how long each named section took and prints them at most once a second.
#[derive(Debug, Default)]
pub struct Tracer {
    last_print_us: u64,
    start_us: u64,
    epochs: Vec<(String, u64)>,
}

const MIN_PRINT_PERIOD_US: u64 = 1_000_000;

impl Tracer {
    pub fn new(now_us: u64) -> Self {
        Self {
            start_us: now_us,
            ..Self::default()
        }
    }

    pub fn clear_epochs(&mut self, now_us: u64) {
        self.epochs.clear();
        self.start_us = now_us;
    }

    /// Records the time since the previous epoch. A repeated name replaces its entry.
    pub fn add_epoch(&mut self, name: &str, now_us: u64) {
        let elapsed = now_us.saturating_sub(self.start_us);
        self.start_us = now_us;
        match self.epochs.iter_mut().find(|(n, _)| n == name) {
            Some(entry) => entry.1 = elapsed,
            None => self.epochs.push((name.to_owned(), elapsed)),
        }
    }

    /// Sends the epochs to `out` unless they printed within the last second. Java iterates a
    /// hash map, so its order is arbitrary; this keeps insertion order.
    pub fn print_epochs(&mut self, now_us: u64, out: &mut dyn FnMut(&str)) {
        if now_us.saturating_sub(self.last_print_us) <= MIN_PRINT_PERIOD_US {
            return;
        }
        self.last_print_us = now_us;
        let text: String = self
            .epochs
            .iter()
            .map(|(name, us)| format!("\t{name}: {:.6}s\n", *us as f64 / 1.0e6))
            .collect();
        if !text.is_empty() {
            out(&text);
        }
    }
}

/// A deadline for one pass of the robot loop.
///
/// Java expires the watchdog on its own thread the instant the deadline passes. This one
/// checks the deadline in [`Watchdog::disable`] at the end of the pass, so the overrun
/// messages come after the slow pass instead of during it, and never if the pass hangs.
#[derive(Debug)]
pub struct Watchdog {
    timeout_s: f64,
    start_s: f64,
    deadline_us: u64,
    enabled: bool,
    expired: bool,
    last_timeout_print_s: f64,
    tracer: Tracer,
}

impl Watchdog {
    pub fn new(timeout_s: f64) -> Self {
        Self {
            timeout_s,
            start_s: 0.0,
            deadline_us: 0,
            enabled: false,
            expired: false,
            last_timeout_print_s: 0.0,
            tracer: Tracer::default(),
        }
    }

    pub fn timeout(&self) -> f64 {
        self.timeout_s
    }

    pub fn set_timeout(&mut self, timeout_s: f64, now_us: u64) {
        self.timeout_s = timeout_s;
        self.enable(now_us);
    }

    /// Starts a pass: new start time, no epochs, not expired.
    pub fn enable(&mut self, now_us: u64) {
        self.start_s = now_us as f64 / 1e6;
        self.deadline_us = ((self.start_s + self.timeout_s) * 1e6) as u64;
        self.tracer.clear_epochs(now_us);
        self.expired = false;
        self.enabled = true;
    }

    pub fn add_epoch(&mut self, name: &str, now_us: u64) {
        self.tracer.add_epoch(name, now_us);
    }

    pub fn is_expired(&self) -> bool {
        self.expired
    }

    /// Ends a pass. Returns true when it ran past the timeout.
    ///
    /// The "not fed" message keeps Java's throttle: it compares seconds against 1e6, so it
    /// first prints after about 11 days of uptime. Overruns still surface through the caller's
    /// own message and the epoch print.
    pub fn disable(&mut self, now_us: u64, out: &mut dyn FnMut(&str)) -> bool {
        if !self.enabled {
            return self.expired;
        }
        self.enabled = false;
        let now_s = now_us as f64 / 1e6;
        if now_us >= self.deadline_us {
            if now_s - self.last_timeout_print_s > MIN_PRINT_PERIOD_US as f64 {
                self.last_timeout_print_s = now_s;
                out(&format!("Watchdog not fed within {:.6}s\n", self.timeout_s));
            }
            self.expired = true;
        }
        self.expired
    }

    pub fn print_epochs(&mut self, now_us: u64, out: &mut dyn FnMut(&str)) {
        self.tracer.print_epochs(now_us, out);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn collect(f: impl FnOnce(&mut dyn FnMut(&str))) -> Vec<String> {
        let mut lines = Vec::new();
        f(&mut |s: &str| lines.push(s.to_owned()));
        lines
    }

    #[test]
    fn tracer_records_durations_in_insertion_order() {
        let mut t = Tracer::new(0);
        t.add_epoch("a", 1_500);
        t.add_epoch("b", 4_000);
        let lines = collect(|out| t.print_epochs(2_000_000, out));
        assert_eq!(lines, ["\ta: 0.001500s\n\tb: 0.002500s\n"]);
    }

    #[test]
    fn tracer_replaces_a_repeated_epoch() {
        let mut t = Tracer::new(0);
        t.add_epoch("a", 1_000);
        t.add_epoch("a", 4_000);
        let lines = collect(|out| t.print_epochs(2_000_000, out));
        assert_eq!(lines, ["\ta: 0.003000s\n"]);
    }

    #[test]
    fn tracer_prints_at_most_once_a_second() {
        let mut t = Tracer::new(0);
        t.add_epoch("a", 1_000);
        assert_eq!(collect(|out| t.print_epochs(2_000_000, out)).len(), 1);
        assert!(collect(|out| t.print_epochs(2_500_000, out)).is_empty());
        assert!(collect(|out| t.print_epochs(3_000_000, out)).is_empty());
        assert_eq!(collect(|out| t.print_epochs(3_000_001, out)).len(), 1);
    }

    #[test]
    fn tracer_with_no_epochs_prints_nothing() {
        let mut t = Tracer::new(0);
        assert!(collect(|out| t.print_epochs(5_000_000, out)).is_empty());
    }

    #[test]
    fn watchdog_expires_past_the_timeout() {
        let mut w = Watchdog::new(0.2);
        w.enable(10_000_000);
        assert!(!w.disable(10_199_000, &mut |_| {}));
        w.enable(10_000_000);
        assert!(w.disable(10_201_000, &mut |_| {}));
        assert!(w.is_expired());
    }

    #[test]
    fn enable_clears_expiry_and_epochs() {
        let mut w = Watchdog::new(0.1);
        w.enable(0);
        w.add_epoch("slow", 500_000);
        w.disable(500_000, &mut |_| {});
        assert!(w.is_expired());
        w.enable(600_000);
        assert!(!w.is_expired());
        assert!(collect(|out| w.print_epochs(9_000_000, out)).is_empty());
    }

    #[test]
    fn not_fed_message_keeps_the_java_unit_mixup() {
        let mut w = Watchdog::new(0.1);
        w.enable(0);
        let early = collect(|out| {
            w.disable(500_000, out);
        });
        assert!(early.is_empty());
        // 2e6 s of uptime is past the 1e6 threshold.
        w.enable(2_000_000_000_000);
        let late = collect(|out| {
            w.disable(2_000_000_500_000, out);
        });
        assert_eq!(late, ["Watchdog not fed within 0.100000s\n"]);
    }

    #[test]
    fn disable_without_enable_is_a_no_op() {
        let mut w = Watchdog::new(0.1);
        assert!(!w.disable(5_000_000, &mut |_| {}));
    }
}

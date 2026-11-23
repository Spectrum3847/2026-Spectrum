//! Java `Notifier`: a callback on its own thread, timed by a HAL notifier.

use crate::driver_station;
use crate::hal::{self, HalError};
use crate::robot_controller;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::JoinHandle;

/// Alarm value the HAL treats as "never".
const NO_ALARM: u64 = u64::MAX;

/// When the next alarm fires. Pure, so the timing arithmetic is testable.
#[derive(Debug, Default, PartialEq)]
struct Schedule {
    expiration_s: f64,
    period_s: f64,
    periodic: bool,
}

impl Schedule {
    fn alarm_us(&self) -> u64 {
        (self.expiration_s * 1e6) as u64
    }

    /// Returns the alarm time in microseconds.
    fn start_periodic(&mut self, now_us: u64, period_s: f64) -> u64 {
        self.periodic = true;
        self.period_s = period_s;
        self.expiration_s = now_us as f64 * 1e-6 + period_s;
        self.alarm_us()
    }

    fn start_single(&mut self, now_us: u64, delay_s: f64) -> u64 {
        self.periodic = false;
        self.period_s = delay_s;
        self.expiration_s = now_us as f64 * 1e-6 + delay_s;
        self.alarm_us()
    }

    /// Next alarm after one fired.
    fn on_alarm(&mut self) -> u64 {
        if self.periodic {
            self.expiration_s += self.period_s;
            self.alarm_us()
        } else {
            NO_ALARM
        }
    }
}

struct Shared {
    schedule: Mutex<Schedule>,
    /// 0 once closed.
    handle: AtomicI32,
}

impl Shared {
    fn lock(&self) -> std::sync::MutexGuard<'_, Schedule> {
        self.schedule.lock().unwrap_or_else(PoisonError::into_inner)
    }
}

/// Runs a callback once or periodically on a dedicated thread. The callback stays on the
/// notifier thread; a panic in it is reported to the driver station and stops the thread.
pub struct Notifier {
    shared: Arc<Shared>,
    thread: Option<JoinHandle<()>>,
}

impl Notifier {
    pub fn new(mut callback: impl FnMut() + Send + 'static) -> Result<Self, HalError> {
        let handle = hal::initialize_notifier()?;
        let shared = Arc::new(Shared {
            schedule: Mutex::default(),
            handle: AtomicI32::new(handle),
        });
        let worker = Arc::clone(&shared);
        let thread = std::thread::Builder::new()
            .name("Notifier".into())
            .spawn(move || run(&worker, &mut callback))
            .map_err(|e| {
                hal::clean_notifier(handle);
                HalError::invalid(format!("cannot start notifier thread: {e}"))
            })?;
        Ok(Self {
            shared,
            thread: Some(thread),
        })
    }

    pub fn set_name(&self, name: &str) {
        let handle = self.shared.handle.load(Ordering::Acquire);
        if handle != 0 {
            hal::set_notifier_name(handle, name);
        }
    }

    pub fn start_periodic(&self, period_s: f64) {
        let mut schedule = self.shared.lock();
        let alarm = schedule.start_periodic(robot_controller::fpga_time(), period_s);
        self.update_alarm(alarm);
    }

    pub fn start_single(&self, delay_s: f64) {
        let mut schedule = self.shared.lock();
        let alarm = schedule.start_single(robot_controller::fpga_time(), delay_s);
        self.update_alarm(alarm);
    }

    pub fn stop(&self) {
        let mut schedule = self.shared.lock();
        schedule.periodic = false;
        let handle = self.shared.handle.load(Ordering::Acquire);
        if handle != 0 {
            hal::cancel_notifier_alarm(handle);
        }
    }

    fn update_alarm(&self, alarm_us: u64) {
        let handle = self.shared.handle.load(Ordering::Acquire);
        if handle != 0 {
            hal::update_notifier_alarm(handle, alarm_us);
        }
    }

    /// Sets the HAL notifier thread priority. False when the OS refuses.
    pub fn set_hal_thread_priority(real_time: bool, priority: i32) -> bool {
        hal::set_notifier_thread_priority(real_time, priority)
    }
}

impl Drop for Notifier {
    fn drop(&mut self) {
        let handle = self.shared.handle.swap(0, Ordering::AcqRel);
        if handle == 0 {
            return;
        }
        hal::stop_notifier(handle);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
        hal::clean_notifier(handle);
    }
}

fn run(shared: &Shared, callback: &mut dyn FnMut()) {
    loop {
        let handle = shared.handle.load(Ordering::Acquire);
        if handle == 0 || hal::wait_for_notifier_alarm(handle) == 0 {
            break;
        }
        {
            let mut schedule = shared.lock();
            let alarm = schedule.on_alarm();
            hal::update_notifier_alarm(handle, alarm);
        }
        if let Err(panic) = catch_unwind(AssertUnwindSafe(&mut *callback)) {
            driver_station::report_error(&format!(
                "Unhandled exception in Notifier thread: {}",
                panic_message(panic.as_ref())
            ));
            driver_station::report_error(
                "The callback for this Notifier (or functions called by it) should have \
                 handled the exception above.",
            );
            break;
        }
    }
}

/// Text of a panic payload.
pub(crate) fn panic_message(payload: &(dyn std::any::Any + Send)) -> String {
    payload
        .downcast_ref::<&str>()
        .map(|s| (*s).to_owned())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "unknown panic".to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn periodic_alarm_is_now_plus_period() {
        let mut s = Schedule::default();
        assert_eq!(s.start_periodic(5_000_000, 0.5), 5_500_000);
    }

    #[test]
    fn periodic_alarms_advance_by_whole_periods() {
        let mut s = Schedule::default();
        s.start_periodic(1_000_000, 0.25);
        assert_eq!(s.on_alarm(), 1_500_000);
        assert_eq!(s.on_alarm(), 1_750_000);
    }

    #[test]
    fn single_shot_disarms_after_firing() {
        let mut s = Schedule::default();
        assert_eq!(s.start_single(2_000_000, 0.5), 2_500_000);
        assert_eq!(s.on_alarm(), NO_ALARM);
        assert_eq!(s.on_alarm(), NO_ALARM);
    }

    #[test]
    fn restarting_replaces_the_schedule() {
        let mut s = Schedule::default();
        s.start_periodic(0, 1.0);
        s.start_single(10_000_000, 2.0);
        assert!(!s.periodic);
        assert_eq!(s.alarm_us(), 12_000_000);
    }

    #[test]
    fn panic_message_reads_str_and_string() {
        assert_eq!(panic_message(&"boom"), "boom");
        assert_eq!(panic_message(&String::from("bang")), "bang");
        assert_eq!(panic_message(&5_u8), "unknown panic");
    }
}

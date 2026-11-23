//! Shutdown hooks. A crate that owns a background thread or notifier registers its stop
//! function here at startup, so `wpilib` needs no dependency on it. [`start_robot`] runs them
//! before the HAL shuts down: native threads still running when the C++ static destructors
//! execute corrupt the heap.
//!
//! [`start_robot`]: crate::start_robot

use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::{Mutex, PoisonError};

type Callback = Box<dyn FnOnce() + Send>;

#[derive(Default)]
struct Hooks(Mutex<Vec<Callback>>);

impl Hooks {
    fn register(&self, callback: Callback) {
        self.0
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .push(callback);
    }

    fn run(&self) {
        // Take the list first so a callback may register another one without deadlocking.
        let callbacks = std::mem::take(&mut *self.0.lock().unwrap_or_else(PoisonError::into_inner));
        for callback in callbacks.into_iter().rev() {
            if catch_unwind(AssertUnwindSafe(callback)).is_err() {
                eprintln!("A shutdown callback panicked");
            }
        }
    }
}

static HOOKS: Hooks = Hooks(Mutex::new(Vec::new()));

/// Registers a function that stops one thread or notifier. It runs once, in [`run`].
pub fn register(callback: impl FnOnce() + Send + 'static) {
    HOOKS.register(Box::new(callback));
}

/// Runs the registered callbacks, last registered first, then forgets them. Safe to call more
/// than once.
pub fn run() {
    HOOKS.run();
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    fn recorder() -> (Arc<Mutex<Vec<u32>>>, impl Fn(u32) -> Callback) {
        let log = Arc::new(Mutex::new(Vec::new()));
        let sink = log.clone();
        let make = move |n: u32| -> Callback {
            let log = sink.clone();
            Box::new(move || log.lock().unwrap().push(n))
        };
        (log, make)
    }

    #[test]
    fn callbacks_run_in_reverse_registration_order() {
        let hooks = Hooks::default();
        let (log, make) = recorder();
        for n in 1..=3 {
            hooks.register(make(n));
        }
        hooks.run();
        assert_eq!(*log.lock().unwrap(), [3, 2, 1]);
    }

    #[test]
    fn running_twice_does_not_repeat_and_later_registrations_still_run() {
        let hooks = Hooks::default();
        let (log, make) = recorder();
        hooks.register(make(1));
        hooks.run();
        hooks.run();
        assert_eq!(*log.lock().unwrap(), [1]);
        hooks.register(make(2));
        hooks.run();
        assert_eq!(*log.lock().unwrap(), [1, 2]);
    }

    #[test]
    fn a_panicking_callback_does_not_stop_the_rest() {
        let hooks = Hooks::default();
        let (log, make) = recorder();
        hooks.register(make(1));
        hooks.register(Box::new(|| panic!("boom")));
        hooks.register(make(3));
        hooks.run();
        assert_eq!(*log.lock().unwrap(), [3, 1]);
    }

    #[test]
    fn a_callback_may_register_during_run() {
        let hooks = Arc::new(Hooks::default());
        let (log, make) = recorder();
        let inner = hooks.clone();
        let later = make(9);
        hooks.register(Box::new(move || inner.register(later)));
        hooks.run();
        assert!(log.lock().unwrap().is_empty());
        hooks.run();
        assert_eq!(*log.lock().unwrap(), [9]);
    }

    #[test]
    fn the_public_functions_use_the_shared_list() {
        let (log, make) = recorder();
        register(make(5));
        run();
        run();
        assert_eq!(*log.lock().unwrap(), [5]);
    }
}

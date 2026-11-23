use std::cell::{Cell, RefCell};

/// Ordered list of actions polled together, like WPILib's `EventLoop`.
#[derive(Default)]
pub struct EventLoop {
    bindings: RefCell<Vec<Box<dyn FnMut()>>>,
    running: Cell<bool>,
}

struct RunningGuard<'a>(&'a Cell<bool>);

impl Drop for RunningGuard<'_> {
    fn drop(&mut self) {
        self.0.set(false);
    }
}

impl EventLoop {
    pub fn new() -> Self {
        Self::default()
    }

    /// Panics while the loop is polling, as WPILib throws.
    pub fn bind(&self, action: impl FnMut() + 'static) {
        assert!(
            !self.running.get(),
            "Cannot bind EventLoop while it is running"
        );
        self.bindings.borrow_mut().push(Box::new(action));
    }

    pub fn poll(&self) {
        assert!(!self.running.get(), "EventLoop is already running");
        self.running.set(true);
        let _guard = RunningGuard(&self.running);
        for action in self.bindings.borrow_mut().iter_mut() {
            action();
        }
    }

    pub fn clear(&self) {
        assert!(
            !self.running.get(),
            "Cannot clear EventLoop while it is running"
        );
        self.bindings.borrow_mut().clear();
    }
}

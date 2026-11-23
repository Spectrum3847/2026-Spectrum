use crate::env::now_seconds;

/// WPILib `Timer` semantics on the injected clock.
pub(crate) struct Timer {
    start: f64,
    accumulated: f64,
    running: bool,
}

impl Timer {
    pub fn new() -> Self {
        Self {
            start: now_seconds(),
            accumulated: 0.0,
            running: false,
        }
    }

    pub fn get(&self) -> f64 {
        if self.running {
            self.accumulated + (now_seconds() - self.start)
        } else {
            self.accumulated
        }
    }

    pub fn restart(&mut self) {
        self.accumulated = 0.0;
        self.start = now_seconds();
        self.running = true;
    }

    pub fn stop(&mut self) {
        self.accumulated = self.get();
        self.running = false;
    }

    pub fn has_elapsed(&self, seconds: f64) -> bool {
        self.get() >= seconds
    }
}

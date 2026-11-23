use commands::{CommandScheduler, Subsystem, SubsystemBase, SubsystemId};
use std::cell::Cell;
use std::rc::Rc;

/// Caches a supplier so its value is computed at most once per scheduler iteration.
/// It registers with the scheduler on construction, and the scheduler's `periodic` call
/// clears the cache.
pub struct CachedDouble {
    base: SubsystemBase,
    cached: Cell<bool>,
    value: Cell<f64>,
    source: Box<dyn Fn() -> f64>,
}

impl CachedDouble {
    pub fn new(source: impl Fn() -> f64 + 'static) -> Rc<Self> {
        let cached_double = Rc::new(Self {
            base: SubsystemBase::new("CachedDouble"),
            cached: Cell::new(false),
            value: Cell::new(0.0),
            source: Box::new(source),
        });
        CommandScheduler::get_instance().register_subsystem(cached_double.clone());
        cached_double
    }

    pub fn get_as_double(&self) -> f64 {
        if !self.cached.get() {
            self.value.set((self.source)());
            self.cached.set(true);
        }
        self.value.get()
    }
}

impl Subsystem for CachedDouble {
    fn id(&self) -> SubsystemId {
        self.base.id()
    }

    fn name(&self) -> String {
        self.base.name().to_owned()
    }

    /// Drops the cached value so the next [`Self::get_as_double`] reads the source again.
    fn periodic(&self) {
        self.cached.set(false);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn caches_within_an_iteration_and_invalidates_on_periodic() {
        let calls = Rc::new(Cell::new(0));
        let counter = calls.clone();
        let cached = CachedDouble::new(move || {
            counter.set(counter.get() + 1);
            42.0
        });

        assert_eq!(calls.get(), 0);

        assert!((cached.get_as_double() - 42.0).abs() < 1e-6);
        assert_eq!(calls.get(), 1);

        assert!((cached.get_as_double() - 42.0).abs() < 1e-6);
        assert!((cached.get_as_double() - 42.0).abs() < 1e-6);
        assert_eq!(calls.get(), 1);

        cached.periodic();

        assert!((cached.get_as_double() - 42.0).abs() < 1e-6);
        assert_eq!(calls.get(), 2);
    }

    #[test]
    fn the_scheduler_run_clears_the_cache() {
        let calls = Rc::new(Cell::new(0));
        let counter = calls.clone();
        let cached = CachedDouble::new(move || {
            counter.set(counter.get() + 1);
            1.0
        });
        cached.get_as_double();
        CommandScheduler::get_instance().run();
        cached.get_as_double();
        assert_eq!(calls.get(), 2);
    }
}

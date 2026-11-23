use commands::env::now_seconds;
use std::cell::RefCell;
use std::cmp::Ordering;
use std::path::PathBuf;

thread_local! {
    static DEPLOY_OVERRIDE: RefCell<Option<PathBuf>> = const { RefCell::new(None) };
}

/// Replaces the deploy folder for the calling thread, for tests. `None` restores the default.
pub fn set_deploy_directory(dir: Option<PathBuf>) {
    DEPLOY_OVERRIDE.with(|d| *d.borrow_mut() = dir);
}

/// The deploy folder holding the `pathplanner` directory.
pub fn deploy_directory() -> PathBuf {
    DEPLOY_OVERRIDE
        .with(|d| d.borrow().clone())
        .unwrap_or_else(wpilib::filesystem::deploy_directory)
}

/// WPILib `Timer` semantics on the command framework clock, so tests can drive time.
#[derive(Debug)]
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

    pub fn reset(&mut self) {
        self.accumulated = 0.0;
        self.start = now_seconds();
    }

    pub fn start(&mut self) {
        if !self.running {
            self.start = now_seconds();
            self.running = true;
        }
    }

    pub fn stop(&mut self) {
        self.accumulated = self.get();
        self.running = false;
    }

    pub fn has_elapsed(&self, seconds: f64) -> bool {
        self.get() >= seconds
    }
}

/// `java.util.PriorityQueue` ordered by an `f64` key. It uses the same binary heap sift
/// steps, so items with equal keys come out in the order Java produces.
pub(crate) struct JavaPriorityQueue<T> {
    heap: Vec<T>,
    key: fn(&T) -> f64,
}

impl<T> JavaPriorityQueue<T> {
    pub fn new(key: fn(&T) -> f64) -> Self {
        Self {
            heap: Vec::new(),
            key,
        }
    }

    fn cmp(&self, a: usize, b: usize) -> Ordering {
        (self.key)(&self.heap[a]).total_cmp(&(self.key)(&self.heap[b]))
    }

    #[cfg(test)]
    pub fn is_empty(&self) -> bool {
        self.heap.is_empty()
    }

    pub fn clear(&mut self) {
        self.heap.clear();
    }

    pub fn peek(&self) -> Option<&T> {
        self.heap.first()
    }

    /// Items in heap array order, which is what iterating a Java `PriorityQueue` gives.
    pub fn iter(&self) -> impl Iterator<Item = &T> {
        self.heap.iter()
    }

    pub fn push(&mut self, item: T) {
        self.heap.push(item);
        let mut k = self.heap.len() - 1;
        while k > 0 {
            let parent = (k - 1) >> 1;
            if self.cmp(k, parent) != Ordering::Less {
                break;
            }
            self.heap.swap(k, parent);
            k = parent;
        }
    }

    pub fn pop(&mut self) -> Option<T> {
        if self.heap.is_empty() {
            return None;
        }
        let last = self.heap.len() - 1;
        self.heap.swap(0, last);
        let result = self.heap.pop();
        let n = self.heap.len();
        let half = n >> 1;
        let mut k = 0;
        while k < half {
            let mut child = 2 * k + 1;
            let right = child + 1;
            if right < n && self.cmp(child, right) == Ordering::Greater {
                child = right;
            }
            if self.cmp(k, child) != Ordering::Greater {
                break;
            }
            self.heap.swap(k, child);
            k = child;
        }
        result
    }
}

/// A scratch deploy folder for tests. It becomes the thread's deploy folder and is deleted on drop.
#[cfg(test)]
pub(crate) struct TestDeploy {
    pub dir: PathBuf,
}

#[cfg(test)]
impl TestDeploy {
    pub fn new(name: &str) -> Self {
        let dir = std::env::temp_dir().join(format!("pathplanner-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(dir.join("pathplanner/paths")).unwrap();
        std::fs::create_dir_all(dir.join("pathplanner/autos")).unwrap();
        set_deploy_directory(Some(dir.clone()));
        Self { dir }
    }

    pub fn write(&self, relative: &str, text: &str) {
        std::fs::write(self.dir.join("pathplanner").join(relative), text).unwrap();
    }
}

#[cfg(test)]
impl Drop for TestDeploy {
    fn drop(&mut self) {
        set_deploy_directory(None);
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

/// The robot's own deploy folder, for tests that load its paths and autos.
#[cfg(test)]
pub(crate) fn robot_deploy_folder() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../src/main/deploy")
}

#[cfg(test)]
pub(crate) fn use_robot_deploy_folder() {
    set_deploy_directory(Some(robot_deploy_folder()));
}

/// The robot's config, readable while a scratch deploy folder is in use.
#[cfg(test)]
pub(crate) fn robot_config() -> crate::config::RobotConfig {
    let text =
        std::fs::read_to_string(robot_deploy_folder().join("pathplanner/settings.json")).unwrap();
    crate::config::RobotConfig::from_gui_settings_json(&text).unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn drain(queue: &mut JavaPriorityQueue<(f64, u32)>) -> Vec<u32> {
        std::iter::from_fn(|| queue.pop().map(|(_, id)| id)).collect()
    }

    #[test]
    fn priority_queue_orders_by_key() {
        let mut q = JavaPriorityQueue::new(|item: &(f64, u32)| item.0);
        for (i, key) in [5.0, 1.0, 4.0, 2.0, 3.0].into_iter().enumerate() {
            q.push((key, i as u32));
        }
        assert_eq!(q.peek().map(|i| i.1), Some(1));
        assert_eq!(drain(&mut q), vec![1, 3, 4, 2, 0]);
        assert!(q.is_empty());
    }

    // Order of equal keys from running java.util.PriorityQueue.
    #[test]
    fn priority_queue_ties_follow_the_java_heap() {
        let mut q = JavaPriorityQueue::new(|item: &(f64, u32)| item.0);
        for id in 0..8 {
            q.push((1.0, id));
        }
        assert_eq!(drain(&mut q), vec![0, 7, 6, 5, 4, 3, 2, 1]);
    }

    #[test]
    fn timer_follows_the_fake_clock() {
        let clock = commands::testing::FakeClock::install();
        let mut t = Timer::new();
        t.reset();
        t.start();
        clock.advance(1.5);
        assert!(t.has_elapsed(1.5));
        t.stop();
        clock.advance(10.0);
        assert_eq!(t.get(), 1.5);
    }
}

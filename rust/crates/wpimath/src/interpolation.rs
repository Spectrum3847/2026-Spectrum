//! Interpolating containers from WPILib.

use crate::math_util;
use std::cmp::Ordering;
use std::collections::BTreeMap;

pub trait Interpolatable: Sized {
    /// Value between `self` (`t = 0`) and `end` (`t = 1`).
    fn interpolate(&self, end: &Self, t: f64) -> Self;
}

impl Interpolatable for f64 {
    fn interpolate(&self, end: &Self, t: f64) -> Self {
        math_util::interpolate(*self, *end, t)
    }
}

/// `f64` map key ordered like Java `Double.compareTo`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Key(pub f64);

impl Eq for Key {}

impl PartialOrd for Key {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Key {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.total_cmp(&other.0)
    }
}

/// `InterpolatingTreeMap<Double, Double>`.
#[derive(Clone, Debug, Default)]
pub struct InterpolatingDoubleTreeMap {
    map: BTreeMap<Key, f64>,
}

impl InterpolatingDoubleTreeMap {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn put(&mut self, key: f64, value: f64) {
        self.map.insert(Key(key), value);
    }

    /// Interpolates between neighbors, clamps outside the keys, `None` when empty.
    pub fn get(&self, key: f64) -> Option<f64> {
        if let Some(v) = self.map.get(&Key(key)) {
            return Some(*v);
        }
        let floor = self.map.range(..=Key(key)).next_back();
        let ceiling = self.map.range(Key(key)..).next();
        match (floor, ceiling) {
            (None, None) => None,
            (Some((_, v)), None) | (None, Some((_, v))) => Some(*v),
            (Some((fk, fv)), Some((ck, cv))) => Some(math_util::interpolate(
                *fv,
                *cv,
                math_util::inverse_interpolate(fk.0, ck.0, key),
            )),
        }
    }

    pub fn clear(&mut self) {
        self.map.clear();
    }
}

/// Time-keyed samples that drop entries older than the history window.
#[derive(Clone, Debug)]
pub struct TimeInterpolatableBuffer<T> {
    history_size: f64,
    snapshots: BTreeMap<Key, T>,
}

impl<T: Interpolatable + Clone> TimeInterpolatableBuffer<T> {
    pub fn new(history_size_seconds: f64) -> Self {
        Self {
            history_size: history_size_seconds,
            snapshots: BTreeMap::new(),
        }
    }

    pub fn add_sample(&mut self, time_seconds: f64, sample: T) {
        self.clean_up(time_seconds);
        self.snapshots.insert(Key(time_seconds), sample);
    }

    fn clean_up(&mut self, time: f64) {
        while let Some((&first, _)) = self.snapshots.first_key_value() {
            if time - first.0 >= self.history_size {
                self.snapshots.remove(&first);
            } else {
                return;
            }
        }
    }

    pub fn clear(&mut self) {
        self.snapshots.clear();
    }

    /// Sample at `time_seconds`, interpolated, or the nearest end when out of range.
    pub fn get_sample(&self, time_seconds: f64) -> Option<T> {
        if self.snapshots.is_empty() {
            return None;
        }
        if let Some(now) = self.snapshots.get(&Key(time_seconds)) {
            return Some(now.clone());
        }
        let bottom = self.snapshots.range(..=Key(time_seconds)).next_back();
        let top = self.snapshots.range(Key(time_seconds)..).next();
        match (bottom, top) {
            (None, None) => None,
            (Some((_, v)), None) | (None, Some((_, v))) => Some(v.clone()),
            (Some((bk, bv)), Some((tk, tv))) => {
                Some(bv.interpolate(tv, (time_seconds - bk.0) / (tk.0 - bk.0)))
            }
        }
    }

    pub fn is_empty(&self) -> bool {
        self.snapshots.is_empty()
    }

    pub fn oldest_time(&self) -> Option<f64> {
        self.snapshots.first_key_value().map(|(k, _)| k.0)
    }

    pub fn newest_time(&self) -> Option<f64> {
        self.snapshots.last_key_value().map(|(k, _)| k.0)
    }
}

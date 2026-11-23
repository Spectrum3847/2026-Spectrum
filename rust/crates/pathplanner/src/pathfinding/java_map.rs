use super::ad_star::GridPosition;

const MIN_TREEIFY_CAPACITY: usize = 64;

/// Iteration order of a `java.util.HashMap<GridPosition, V>`.
///
/// AD* picks the first of several equal keys while it scans its open set, so the result depends
/// on that order. This reproduces the buckets, the resize steps and the order inside a bucket.
/// Java turns a bucket of 9 or more entries into a tree on tables of 64 or more, which changes
/// its order. That is not reproduced.
pub(crate) struct JavaHashMap<V> {
    table: Vec<Vec<(GridPosition, V)>>,
    size: usize,
    threshold: usize,
}

impl<V: Copy> JavaHashMap<V> {
    pub fn new() -> Self {
        Self {
            table: Vec::new(),
            size: 0,
            threshold: 0,
        }
    }

    // The record hash is 31 * x + y, then HashMap mixes the high bits in.
    fn bucket(&self, pos: GridPosition) -> usize {
        let h = pos.x.wrapping_mul(31).wrapping_add(pos.y) as u32;
        let spread = h ^ (h >> 16);
        spread as usize & (self.table.len() - 1)
    }

    fn resize(&mut self) {
        let old_cap = self.table.len();
        let new_cap = if old_cap == 0 { 16 } else { old_cap << 1 };
        self.threshold = new_cap / 4 * 3;
        let old = std::mem::replace(&mut self.table, vec![Vec::new(); new_cap]);
        // Walking the old buckets in order keeps the relative order inside each new bucket.
        for (pos, value) in old.into_iter().flatten() {
            let b = self.bucket(pos);
            self.table[b].push((pos, value));
        }
    }

    pub fn put(&mut self, pos: GridPosition, value: V) {
        if self.table.is_empty() {
            self.resize();
        }
        let b = self.bucket(pos);
        if let Some(entry) = self.table[b].iter_mut().find(|(p, _)| *p == pos) {
            entry.1 = value;
            return;
        }
        self.table[b].push((pos, value));
        if self.table[b].len() > 8 && self.table.len() < MIN_TREEIFY_CAPACITY {
            self.resize();
        }
        self.size += 1;
        if self.size > self.threshold {
            self.resize();
        }
    }

    pub fn remove(&mut self, pos: GridPosition) {
        if self.table.is_empty() {
            return;
        }
        let b = self.bucket(pos);
        if let Some(i) = self.table[b].iter().position(|(p, _)| *p == pos) {
            self.table[b].remove(i);
            self.size -= 1;
        }
    }

    /// Empties the map and keeps the table size.
    pub fn clear(&mut self) {
        for bucket in &mut self.table {
            bucket.clear();
        }
        self.size = 0;
    }

    #[cfg(test)]
    pub fn len(&self) -> usize {
        self.size
    }

    pub fn iter(&self) -> impl Iterator<Item = &(GridPosition, V)> {
        self.table.iter().flatten()
    }

    /// `putAll`: grows the table up front when the other map is bigger than the threshold.
    pub fn put_all(&mut self, other: &JavaHashMap<V>) {
        if other.size == 0 {
            return;
        }
        if self.table.is_empty() {
            // Java sizes a map that has no table from the incoming count. The planner never
            // gets here, because the open set always has an entry by then.
            self.resize();
        }
        while other.size > self.threshold {
            self.resize();
        }
        for (pos, value) in other.iter() {
            self.put(*pos, *value);
        }
    }

    pub fn replace_all(&mut self, mut f: impl FnMut(GridPosition, V) -> V) {
        for bucket in &mut self.table {
            for (pos, value) in bucket {
                *value = f(*pos, *value);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn keys(map: &JavaHashMap<u8>) -> Vec<(i32, i32)> {
        map.iter().map(|(p, _)| (p.x, p.y)).collect()
    }

    fn pos(x: i32, y: i32) -> GridPosition {
        GridPosition { x, y }
    }

    #[test]
    fn iterates_by_bucket_then_insertion() {
        let mut m = JavaHashMap::new();
        // Hashes 31*0+17 = 17 (bucket 1), 31*1+1 = 32 (bucket 0), 0 (bucket 0), 33 (bucket 1).
        for (x, y) in [(0, 17), (1, 1), (0, 0), (1, 2)] {
            m.put(pos(x, y), 0u8);
        }
        assert_eq!(keys(&m), [(1, 1), (0, 0), (0, 17), (1, 2)]);
    }

    #[test]
    fn put_on_an_existing_key_keeps_its_place() {
        let mut m = JavaHashMap::new();
        m.put(pos(0, 0), 1u8);
        m.put(pos(0, 16), 2);
        m.put(pos(0, 0), 3);
        assert_eq!(m.iter().map(|(_, v)| *v).collect::<Vec<_>>(), [3, 2]);
        m.remove(pos(0, 0));
        m.put(pos(0, 0), 4);
        assert_eq!(m.iter().map(|(_, v)| *v).collect::<Vec<_>>(), [2, 4]);
    }

    #[test]
    fn grows_past_the_threshold_and_keeps_bucket_order() {
        let mut m = JavaHashMap::new();
        for y in 0..13 {
            m.put(pos(0, y), 0u8);
        }
        assert_eq!(m.table.len(), 32);
        assert_eq!(m.len(), 13);
        let order: Vec<i32> = m.iter().map(|(p, _)| p.y).collect();
        assert_eq!(order, (0..13).collect::<Vec<_>>());
    }

    #[test]
    fn clear_keeps_the_table_size() {
        let mut m = JavaHashMap::new();
        for y in 0..13 {
            m.put(pos(0, y), 0u8);
        }
        m.clear();
        assert_eq!(m.len(), 0);
        assert_eq!(m.table.len(), 32);
    }
}

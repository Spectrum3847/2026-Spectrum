use super::Pathfinder;
use super::java_map::JavaHashMap;
use crate::error::{Error, Result};
use crate::path::{GoalEndState, PathConstraints, PathPlannerPath, Waypoint};
use crate::util::deploy_directory;
use serde::Deserialize;
use std::cmp::Ordering;
use std::collections::{HashSet, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering as AtomicOrdering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::{self, JoinHandle, Thread};
use std::time::Duration;
use wpimath::geometry::{Pose2d, Translation2d};

const SMOOTHING_ANCHOR_PCT: f64 = 0.8;
const EPS: f64 = 2.5;
const INF: f64 = f64::INFINITY;

/// A node of the pathfinding grid.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct GridPosition {
    pub x: i32,
    pub y: i32,
}

impl GridPosition {
    pub const fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }
}

/// A grid position outside the field. Java throws a null pointer exception on one, and the
/// planning thread starts over. This is the same condition.
#[derive(Debug, PartialEq, Eq)]
pub struct OutOfGrid;

#[derive(Deserialize)]
struct FieldSizeJson {
    x: f64,
    y: f64,
}

#[derive(Deserialize)]
struct NavGridJson {
    #[serde(rename = "nodeSizeMeters")]
    node_size_meters: f64,
    grid: Vec<Vec<bool>>,
    field_size: FieldSizeJson,
}

/// The navigation grid: the field cut into square nodes, some of them obstacles.
#[derive(Clone, Debug)]
pub struct Grid {
    node_size: f64,
    nodes_x: i32,
    nodes_y: i32,
    obstacles: Vec<bool>,
}

impl Grid {
    /// The grid Java uses when there is no navgrid file: an empty 16.54 m by 8.02 m field of
    /// 0.2 m nodes.
    pub fn empty() -> Self {
        let node_size = 0.2;
        let nodes_x = (16.54_f64 / node_size).ceil() as i32;
        let nodes_y = (8.02_f64 / node_size).ceil() as i32;
        Self {
            node_size,
            nodes_x,
            nodes_y,
            obstacles: vec![false; (nodes_x * nodes_y) as usize],
        }
    }

    /// Loads `pathplanner/navgrid.json` from the deploy folder. A missing or unreadable file
    /// gives the empty grid.
    pub fn load() -> Self {
        let file = deploy_directory().join("pathplanner/navgrid.json");
        std::fs::read_to_string(file)
            .map_err(Error::from)
            .and_then(|text| Self::from_json(&text))
            .unwrap_or_else(|_| Self::empty())
    }

    pub fn from_json(text: &str) -> Result<Self> {
        let json: NavGridJson = serde_json::from_str(text)?;
        let nodes_y = json.grid.len() as i32;
        let nodes_x = json.grid.first().map_or(0, Vec::len) as i32;
        let mut obstacles = vec![false; (nodes_x * nodes_y) as usize];
        for (row, cells) in json.grid.iter().enumerate() {
            for (col, obstacle) in cells.iter().enumerate().take(nodes_x as usize) {
                obstacles[row * nodes_x as usize + col] = *obstacle;
            }
        }
        // The field size is read but unused, as in Java.
        let _ = (json.field_size.x, json.field_size.y);
        Ok(Self {
            node_size: json.node_size_meters,
            nodes_x,
            nodes_y,
            obstacles,
        })
    }

    pub fn nodes_x(&self) -> i32 {
        self.nodes_x
    }

    pub fn nodes_y(&self) -> i32 {
        self.nodes_y
    }

    fn index(&self, p: GridPosition) -> Option<usize> {
        (p.x >= 0 && p.x < self.nodes_x && p.y >= 0 && p.y < self.nodes_y)
            .then(|| (p.y * self.nodes_x + p.x) as usize)
    }

    fn is_obstacle(&self, p: GridPosition) -> bool {
        self.index(p).is_some_and(|i| self.obstacles[i])
    }

    pub fn grid_pos(&self, pos: Translation2d) -> GridPosition {
        GridPosition::new(
            (pos.x / self.node_size).floor() as i32,
            (pos.y / self.node_size).floor() as i32,
        )
    }

    fn to_translation(&self, p: GridPosition) -> Translation2d {
        Translation2d::new(
            (f64::from(p.x) * self.node_size) + (self.node_size / 2.0),
            (f64::from(p.y) * self.node_size) + (self.node_size / 2.0),
        )
    }

    /// The nodes around `s` inside the grid, `s` included, in Java's order.
    fn neighbors(&self, s: GridPosition, skip_obstacles: bool) -> Neighbors {
        let mut out = Neighbors {
            items: [GridPosition::new(0, 0); 9],
            len: 0,
        };
        for x_move in -1..=1 {
            for y_move in -1..=1 {
                let next = GridPosition::new(s.x + x_move, s.y + y_move);
                if self.index(next).is_some() && !(skip_obstacles && self.is_obstacle(next)) {
                    out.items[out.len] = next;
                    out.len += 1;
                }
            }
        }
        out
    }

    /// `pos` if it is free, else the nearest free node found breadth first.
    pub fn find_closest_non_obstacle(&self, pos: GridPosition) -> Option<GridPosition> {
        if !self.is_obstacle(pos) {
            return Some(pos);
        }

        let mut seen: HashSet<GridPosition> = HashSet::new();
        let mut queue: VecDeque<GridPosition> = VecDeque::new();
        for n in self.neighbors(pos, false).iter() {
            seen.insert(*n);
            queue.push_back(*n);
        }

        while let Some(check) = queue.pop_front() {
            if !self.is_obstacle(check) {
                return Some(check);
            }
            for n in self.neighbors(check, false).iter() {
                if seen.insert(*n) {
                    queue.push_back(*n);
                }
            }
        }
        None
    }

    /// Whether a straight line between two nodes crosses no obstacle.
    fn walkable(&self, s1: GridPosition, s2: GridPosition) -> bool {
        let (x0, y0, x1, y1) = (s1.x, s1.y, s2.x, s2.y);

        let mut dx = (x1 - x0).abs();
        let mut dy = (y1 - y0).abs();
        let mut x = x0;
        let mut y = y0;
        let mut n = 1 + dx + dy;
        let x_inc = if x1 > x0 { 1 } else { -1 };
        let y_inc = if y1 > y0 { 1 } else { -1 };
        let mut error = dx - dy;
        dx *= 2;
        dy *= 2;

        while n > 0 {
            if self.is_obstacle(GridPosition::new(x, y)) {
                return false;
            }

            if error > 0 {
                x += x_inc;
                error -= dy;
            } else if error < 0 {
                y += y_inc;
                error += dx;
            } else {
                x += x_inc;
                y += y_inc;
                error -= dy;
                error += dx;
                n -= 1;
            }
            n -= 1;
        }
        true
    }

    fn is_collision(&self, s_start: GridPosition, s_end: GridPosition) -> bool {
        if self.is_obstacle(s_start) || self.is_obstacle(s_end) {
            return true;
        }

        if s_start.x != s_end.x && s_start.y != s_end.y {
            let (s1, s2) = if s_end.x - s_start.x == s_start.y - s_end.y {
                (
                    GridPosition::new(s_start.x.min(s_end.x), s_start.y.min(s_end.y)),
                    GridPosition::new(s_start.x.max(s_end.x), s_start.y.max(s_end.y)),
                )
            } else {
                (
                    GridPosition::new(s_start.x.min(s_end.x), s_start.y.max(s_end.y)),
                    GridPosition::new(s_start.x.max(s_end.x), s_start.y.min(s_end.y)),
                )
            };
            return self.is_obstacle(s1) || self.is_obstacle(s2);
        }
        false
    }
}

struct Neighbors {
    items: [GridPosition; 9],
    len: usize,
}

impl Neighbors {
    fn iter(&self) -> std::slice::Iter<'_, GridPosition> {
        self.items[..self.len].iter()
    }
}

fn heuristic(s_start: GridPosition, s_goal: GridPosition) -> f64 {
    f64::from(s_goal.x - s_start.x).hypot(f64::from(s_goal.y - s_start.y))
}

fn compare_pair(a: (f64, f64), b: (f64, f64)) -> Ordering {
    a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1))
}

/// `Double.equals`: same bits, with all NaNs equal.
fn double_equals(a: f64, b: f64) -> bool {
    a.to_bits() == b.to_bits() || (a.is_nan() && b.is_nan())
}

/// What one pass of the planner does and where.
#[derive(Clone, Copy, Debug)]
pub struct PlanRequest {
    pub reset: bool,
    pub minor: bool,
    pub major: bool,
    pub start: GridPosition,
    pub goal: GridPosition,
    pub real_start: Translation2d,
    pub real_goal: Translation2d,
}

/// A path found by one pass of the planner.
#[derive(Clone, Debug)]
pub struct Plan {
    pub grid_path: Vec<GridPosition>,
    pub waypoints: Vec<Waypoint>,
}

/// The AD* search without its thread. [`LocalAdStar`] runs one on a background thread.
pub struct AdStarPlanner {
    grid: Arc<Grid>,
    g: Vec<f64>,
    rhs: Vec<f64>,
    open: JavaHashMap<(f64, f64)>,
    incons: JavaHashMap<(f64, f64)>,
    closed: Vec<bool>,
    eps: f64,
}

impl AdStarPlanner {
    pub fn new(grid: Arc<Grid>) -> Self {
        let nodes = (grid.nodes_x * grid.nodes_y) as usize;
        Self {
            grid,
            g: vec![INF; nodes],
            rhs: vec![INF; nodes],
            open: JavaHashMap::new(),
            incons: JavaHashMap::new(),
            closed: vec![false; nodes],
            eps: 0.0,
        }
    }

    /// The inflation of the current search. It reaches 1.0 after three major passes.
    pub fn eps(&self) -> f64 {
        self.eps
    }

    /// Plans from `real_start` to `real_goal` through the same passes the thread runs when it is
    /// left alone, until the search is exact. Returns the last plan.
    pub fn plan_to_convergence(
        &mut self,
        real_start: Translation2d,
        real_goal: Translation2d,
    ) -> Option<Plan> {
        let start = self
            .grid
            .find_closest_non_obstacle(self.grid.grid_pos(real_start))?;
        let goal = self
            .grid
            .find_closest_non_obstacle(self.grid.grid_pos(real_goal))?;

        let mut requests = Requests {
            start,
            real_start,
            goal,
            real_goal,
            minor: true,
            major: true,
            reset: true,
        };
        let mut last = None;
        while let Some(request) = requests.take_pass(self.eps) {
            match self.do_work(&request) {
                Ok(Some(plan)) => last = Some(plan),
                Ok(None) => {}
                Err(OutOfGrid) => return None,
            }
        }
        last
    }

    /// One planning pass. `None` when there was nothing new to publish.
    pub fn do_work(
        &mut self,
        request: &PlanRequest,
    ) -> std::result::Result<Option<Plan>, OutOfGrid> {
        let (start, goal) = (request.start, request.goal);
        if request.reset {
            self.reset(start, goal)?;
        }

        if request.minor {
            self.compute_or_improve_path(start, goal)?;
            return Ok(Some(self.plan(request)?));
        }

        if request.major && self.eps > 1.0 {
            self.eps -= 0.5;
            self.open.put_all(&self.incons);
            let positions: Vec<GridPosition> = self.open.iter().map(|(s, _)| *s).collect();
            let mut keys = Vec::with_capacity(positions.len());
            for s in &positions {
                keys.push(self.key(*s, start)?);
            }
            let mut keys = keys.into_iter();
            self.open
                .replace_all(|_, _| keys.next().expect("one key per open entry"));
            self.closed.fill(false);
            self.compute_or_improve_path(start, goal)?;
            return Ok(Some(self.plan(request)?));
        }
        Ok(None)
    }

    fn plan(&self, request: &PlanRequest) -> std::result::Result<Plan, OutOfGrid> {
        let grid_path = self.extract_path(request.start, request.goal)?;
        let waypoints = self.create_waypoints(&grid_path, request.real_start, request.real_goal);
        Ok(Plan {
            grid_path,
            waypoints,
        })
    }

    fn index(&self, p: GridPosition) -> std::result::Result<usize, OutOfGrid> {
        self.grid.index(p).ok_or(OutOfGrid)
    }

    fn reset(
        &mut self,
        s_start: GridPosition,
        s_goal: GridPosition,
    ) -> std::result::Result<(), OutOfGrid> {
        self.g.fill(INF);
        self.rhs.fill(INF);
        self.open.clear();
        self.incons.clear();
        self.closed.fill(false);

        let goal = self.index(s_goal)?;
        self.rhs[goal] = 0.0;
        self.eps = EPS;

        let key = self.key(s_goal, s_start)?;
        self.open.put(s_goal, key);
        Ok(())
    }

    fn key(
        &self,
        s: GridPosition,
        s_start: GridPosition,
    ) -> std::result::Result<(f64, f64), OutOfGrid> {
        let i = self.index(s)?;
        let (g, rhs) = (self.g[i], self.rhs[i]);
        Ok(if g > rhs {
            (rhs + self.eps * heuristic(s_start, s), rhs)
        } else {
            (g + heuristic(s_start, s), g)
        })
    }

    /// The open entry with the smallest key. The first one wins a tie.
    fn top_key(&self) -> Option<(GridPosition, (f64, f64))> {
        let mut min: Option<(GridPosition, (f64, f64))> = None;
        for (s, v) in self.open.iter() {
            if min.is_none_or(|(_, mv)| compare_pair(*v, mv) == Ordering::Less) {
                min = Some((*s, *v));
            }
        }
        min
    }

    fn compute_or_improve_path(
        &mut self,
        s_start: GridPosition,
        s_goal: GridPosition,
    ) -> std::result::Result<(), OutOfGrid> {
        while let Some((s, v)) = self.top_key() {
            let start = self.index(s_start)?;
            if compare_pair(v, self.key(s_start, s_start)?) != Ordering::Less
                && double_equals(self.rhs[start], self.g[start])
            {
                break;
            }

            self.open.remove(s);

            let i = self.index(s)?;
            if self.g[i] > self.rhs[i] {
                self.g[i] = self.rhs[i];
                self.closed[i] = true;

                for sn in self.grid.neighbors(s, true).iter() {
                    self.update_state(*sn, s_start, s_goal)?;
                }
            } else {
                self.g[i] = INF;
                for sn in self.grid.neighbors(s, true).iter() {
                    self.update_state(*sn, s_start, s_goal)?;
                }
                self.update_state(s, s_start, s_goal)?;
            }
        }
        Ok(())
    }

    fn update_state(
        &mut self,
        s: GridPosition,
        s_start: GridPosition,
        s_goal: GridPosition,
    ) -> std::result::Result<(), OutOfGrid> {
        let i = self.index(s)?;
        if s != s_goal {
            self.rhs[i] = INF;

            for x in self.grid.neighbors(s, true).iter() {
                let g = self.g[self.index(*x)?];
                self.rhs[i] = self.rhs[i].min(g + self.cost(s, *x));
            }
        }

        self.open.remove(s);

        if !double_equals(self.g[i], self.rhs[i]) {
            if !self.closed[i] {
                let key = self.key(s, s_start)?;
                self.open.put(s, key);
            } else {
                self.incons.put(s, (0.0, 0.0));
            }
        }
        Ok(())
    }

    fn cost(&self, s_start: GridPosition, s_goal: GridPosition) -> f64 {
        if self.grid.is_collision(s_start, s_goal) {
            return INF;
        }
        heuristic(s_start, s_goal)
    }

    fn extract_path(
        &self,
        s_start: GridPosition,
        s_goal: GridPosition,
    ) -> std::result::Result<Vec<GridPosition>, OutOfGrid> {
        if s_goal == s_start {
            return Ok(Vec::new());
        }

        let mut path = vec![s_start];
        let mut s = s_start;

        for _ in 0..200 {
            // Java scans a HashMap here, so equal costs resolve in its iteration order.
            let mut g_list: JavaHashMap<f64> = JavaHashMap::new();
            for x in self.grid.neighbors(s, true).iter() {
                g_list.put(*x, self.g[self.index(*x)?]);
            }

            let mut min = (s_goal, INF);
            for (pos, g) in g_list.iter() {
                if *g < min.1 {
                    min = (*pos, *g);
                }
            }
            s = min.0;

            path.push(s);
            if s == s_goal {
                break;
            }
        }
        Ok(path)
    }

    fn create_waypoints(
        &self,
        path: &[GridPosition],
        real_start_pos: Translation2d,
        real_goal_pos: Translation2d,
    ) -> Vec<Waypoint> {
        if path.is_empty() {
            return Vec::new();
        }

        let mut simplified_path = vec![path[0]];
        for i in 1..path.len() - 1 {
            if !self
                .grid
                .walkable(simplified_path[simplified_path.len() - 1], path[i + 1])
            {
                simplified_path.push(path[i]);
            }
        }
        simplified_path.push(path[path.len() - 1]);

        let mut field_pos_path: Vec<Translation2d> = simplified_path
            .iter()
            .map(|p| self.grid.to_translation(*p))
            .collect();

        if field_pos_path.len() < 2 {
            return Vec::new();
        }

        // Replace the start and end with their real positions.
        let last = field_pos_path.len() - 1;
        field_pos_path[0] = real_start_pos;
        field_pos_path[last] = real_goal_pos;

        let mut path_poses = vec![Pose2d::new(
            field_pos_path[0],
            (field_pos_path[1] - field_pos_path[0]).angle(),
        )];
        for i in 1..field_pos_path.len() - 1 {
            let (prev, current, next) = (
                field_pos_path[i - 1],
                field_pos_path[i],
                field_pos_path[i + 1],
            );

            let anchor1 = (current - prev) * SMOOTHING_ANCHOR_PCT + prev;
            let heading1 = (current - prev).angle();
            let anchor2 = (current - next) * SMOOTHING_ANCHOR_PCT + next;
            let heading2 = (next - anchor2).angle();

            path_poses.push(Pose2d::new(anchor1, heading1));
            path_poses.push(Pose2d::new(anchor2, heading2));
        }
        path_poses.push(Pose2d::new(
            field_pos_path[last],
            (field_pos_path[last] - field_pos_path[last - 1]).angle(),
        ));

        PathPlannerPath::waypoints_from_poses(&path_poses)
    }
}

/// What the command thread asked for, and what the planning thread has not done yet.
struct Requests {
    start: GridPosition,
    real_start: Translation2d,
    goal: GridPosition,
    real_goal: Translation2d,
    minor: bool,
    major: bool,
    reset: bool,
}

impl Requests {
    /// The next pass to run, or `None` when there is nothing to do. It clears the flags for what
    /// the pass does. `eps` is the planner's current inflation.
    fn take_pass(&mut self, eps: f64) -> Option<PlanRequest> {
        let request = PlanRequest {
            reset: self.reset,
            minor: self.minor,
            major: self.major,
            start: self.start,
            goal: self.goal,
            real_start: self.real_start,
            real_goal: self.real_goal,
        };

        if self.reset {
            self.reset = false;
        }
        if self.minor {
            self.minor = false;
        } else if self.major && (eps - 0.5) <= 1.0 {
            self.major = false;
        }

        (request.reset || request.minor || request.major).then_some(request)
    }
}

struct Shared {
    grid: Arc<Grid>,
    requests: Mutex<Requests>,
    waypoints: Mutex<Vec<Waypoint>>,
    new_path_available: AtomicBool,
    stop: AtomicBool,
}

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(|e| e.into_inner())
}

/// AD* pathfinding on a background thread, over the grid in `navgrid.json`.
pub struct LocalAdStar {
    shared: Arc<Shared>,
    thread: Thread,
    handle: Option<JoinHandle<()>>,
}

impl LocalAdStar {
    /// Loads the grid and starts the planning thread.
    pub fn new() -> Self {
        Self::with_grid(Grid::load())
    }

    pub fn with_grid(grid: Grid) -> Self {
        let shared = Arc::new(Shared {
            grid: Arc::new(grid),
            requests: Mutex::new(Requests {
                start: GridPosition::new(0, 0),
                real_start: Translation2d::ZERO,
                goal: GridPosition::new(0, 0),
                real_goal: Translation2d::ZERO,
                minor: true,
                major: true,
                reset: true,
            }),
            waypoints: Mutex::new(Vec::new()),
            new_path_available: AtomicBool::new(false),
            stop: AtomicBool::new(false),
        });

        let worker = shared.clone();
        let handle = thread::Builder::new()
            .name("ADStar Planning Thread".to_string())
            .spawn(move || run_thread(&worker))
            .expect("cannot start the AD* planning thread");

        Self {
            shared,
            thread: handle.thread().clone(),
            handle: Some(handle),
        }
    }
}

impl Default for LocalAdStar {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for LocalAdStar {
    fn drop(&mut self) {
        self.shared.stop.store(true, AtomicOrdering::Relaxed);
        self.thread.unpark();
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

impl Pathfinder for LocalAdStar {
    fn is_new_path_available(&self) -> bool {
        self.shared.new_path_available.load(AtomicOrdering::Relaxed)
    }

    fn get_current_path(
        &self,
        constraints: PathConstraints,
        goal_end_state: GoalEndState,
    ) -> Option<PathPlannerPath> {
        let waypoints = lock(&self.shared.waypoints).clone();

        self.shared
            .new_path_available
            .store(false, AtomicOrdering::Relaxed);

        if waypoints.len() < 2 {
            return None;
        }

        Some(PathPlannerPath::simple(
            waypoints,
            constraints,
            None,
            goal_end_state,
        ))
    }

    fn set_start_position(&self, start_position: Translation2d) {
        let grid = &self.shared.grid;
        let start = grid.find_closest_non_obstacle(grid.grid_pos(start_position));

        if let Some(start) = start {
            let mut requests = lock(&self.shared.requests);
            if start != requests.start {
                requests.start = start;
                requests.real_start = start_position;

                requests.minor = true;
                self.shared
                    .new_path_available
                    .store(false, AtomicOrdering::Relaxed);
                drop(requests);
                self.thread.unpark();
            }
        }
    }

    fn set_goal_position(&self, goal_position: Translation2d) {
        let grid = &self.shared.grid;
        let goal = grid.find_closest_non_obstacle(grid.grid_pos(goal_position));

        if let Some(goal) = goal {
            let mut requests = lock(&self.shared.requests);
            requests.goal = goal;
            requests.real_goal = goal_position;

            requests.minor = true;
            requests.major = true;
            requests.reset = true;
            self.shared
                .new_path_available
                .store(false, AtomicOrdering::Relaxed);
            drop(requests);
            self.thread.unpark();
        }
    }
}

fn run_thread(shared: &Shared) {
    let mut planner = AdStarPlanner::new(shared.grid.clone());

    while !shared.stop.load(AtomicOrdering::Relaxed) {
        let pass = lock(&shared.requests).take_pass(planner.eps());
        let Some(request) = pass else {
            thread::park_timeout(Duration::from_millis(10));
            continue;
        };

        match planner.do_work(&request) {
            Ok(Some(plan)) => {
                *lock(&shared.waypoints) = plan.waypoints;
                shared
                    .new_path_available
                    .store(true, AtomicOrdering::Relaxed);
            }
            Ok(None) => {}
            Err(OutOfGrid) => {
                // Start over, as Java does after any exception.
                lock(&shared.requests).reset = true;
                thread::park_timeout(Duration::from_millis(10));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::util::{TestDeploy, use_robot_deploy_folder};
    use std::time::Instant;
    use wpimath::geometry::Rotation2d;

    /// A 12 by 8 grid of 1 m nodes with a wall at x = 6 and a gap at y = 6 and 7.
    fn walled_grid() -> Grid {
        let rows: Vec<Vec<bool>> = (0..8)
            .map(|y| (0..12).map(|x| x == 6 && y < 6).collect())
            .collect();
        Grid::from_json(
            &serde_json::json!({
                "field_size": {"x": 12.0, "y": 8.0},
                "nodeSizeMeters": 1.0,
                "grid": rows,
            })
            .to_string(),
        )
        .unwrap()
    }

    fn pos(x: f64, y: f64) -> Translation2d {
        Translation2d::new(x, y)
    }

    #[test]
    fn the_default_grid_matches_java() {
        let grid = Grid::empty();
        assert_eq!((grid.nodes_x(), grid.nodes_y()), (83, 41));
        assert!(!grid.is_obstacle(GridPosition::new(0, 0)));
    }

    #[test]
    fn the_robot_navgrid_loads() {
        use_robot_deploy_folder();
        let grid = Grid::load();
        assert_eq!(grid.node_size, 0.3);
        assert_eq!((grid.nodes_x(), grid.nodes_y()), (56, 27));
        assert!(grid.is_obstacle(GridPosition::new(0, 0)));
    }

    #[test]
    fn a_missing_or_broken_navgrid_gives_the_default_grid() {
        let deploy = TestDeploy::new("navgrid");
        assert_eq!(Grid::load().nodes_x(), 83);
        deploy.write("navgrid.json", "{ nope");
        assert_eq!(Grid::load().nodes_x(), 83);
        assert!(Grid::from_json("{ nope").is_err());
    }

    #[test]
    fn grid_positions_and_node_centers_round_trip() {
        let grid = walled_grid();
        assert_eq!(grid.grid_pos(pos(3.7, 2.2)), GridPosition::new(3, 2));
        assert_eq!(grid.grid_pos(pos(-0.1, 0.0)), GridPosition::new(-1, 0));
        assert_eq!(grid.to_translation(GridPosition::new(3, 2)), pos(3.5, 2.5));
    }

    #[test]
    fn a_blocked_node_moves_to_the_nearest_free_one() {
        let grid = walled_grid();
        assert_eq!(
            grid.find_closest_non_obstacle(GridPosition::new(2, 2)),
            Some(GridPosition::new(2, 2))
        );
        let moved = grid
            .find_closest_non_obstacle(GridPosition::new(6, 3))
            .unwrap();
        assert!(!grid.is_obstacle(moved));
        assert!((moved.x - 6).abs() <= 1 && (moved.y - 3).abs() <= 1);

        let full = Grid::from_json(
            r#"{"field_size": {"x": 2, "y": 2}, "nodeSizeMeters": 1.0, "grid": [[true, true], [true, true]]}"#,
        )
        .unwrap();
        assert_eq!(
            full.find_closest_non_obstacle(GridPosition::new(0, 0)),
            None
        );
    }

    #[test]
    fn a_line_is_walkable_only_when_it_misses_the_obstacles() {
        let grid = walled_grid();
        assert!(grid.walkable(GridPosition::new(1, 1), GridPosition::new(5, 4)));
        assert!(!grid.walkable(GridPosition::new(1, 1), GridPosition::new(10, 1)));
        assert!(grid.walkable(GridPosition::new(1, 7), GridPosition::new(10, 7)));
    }

    #[test]
    fn a_diagonal_between_two_obstacles_is_a_collision() {
        let grid = Grid::from_json(
            r#"{"field_size": {"x": 2, "y": 2}, "nodeSizeMeters": 1.0, "grid": [[false, true], [true, false]]}"#,
        )
        .unwrap();
        assert!(grid.is_collision(GridPosition::new(0, 0), GridPosition::new(1, 1)));
        assert!(!walled_grid().is_collision(GridPosition::new(1, 1), GridPosition::new(2, 2)));
    }

    #[test]
    fn the_planner_goes_around_a_wall_through_the_gap() {
        let grid = Arc::new(walled_grid());
        let mut planner = AdStarPlanner::new(grid.clone());
        let plan = planner
            .plan_to_convergence(pos(1.5, 1.5), pos(10.5, 1.5))
            .unwrap();

        assert_eq!(planner.eps(), 1.0);
        assert!(plan.grid_path.iter().all(|p| !grid.is_obstacle(*p)));
        assert!(plan.grid_path.iter().any(|p| p.x == 6 && p.y >= 6));
        assert_eq!(plan.grid_path.first(), Some(&GridPosition::new(1, 1)));
        assert_eq!(plan.grid_path.last(), Some(&GridPosition::new(10, 1)));

        // The path starts and ends at the real positions, not the node centers.
        let first = plan.waypoints.first().unwrap().anchor;
        let last = plan.waypoints.last().unwrap().anchor;
        assert_eq!((first, last), (pos(1.5, 1.5), pos(10.5, 1.5)));
    }

    #[test]
    fn an_open_field_gives_a_straight_path() {
        let mut planner = AdStarPlanner::new(Arc::new(Grid::empty()));
        let plan = planner
            .plan_to_convergence(pos(1.0, 1.0), pos(10.0, 6.0))
            .unwrap();
        assert_eq!(plan.waypoints.len(), 2);
    }

    #[test]
    fn the_same_start_and_goal_give_no_path() {
        let mut planner = AdStarPlanner::new(Arc::new(walled_grid()));
        let plan = planner
            .plan_to_convergence(pos(2.5, 2.5), pos(2.6, 2.4))
            .unwrap();
        assert!(plan.grid_path.is_empty());
        assert!(plan.waypoints.is_empty());
    }

    #[test]
    fn a_start_outside_the_grid_is_an_error_like_java() {
        let mut planner = AdStarPlanner::new(Arc::new(walled_grid()));
        let request = PlanRequest {
            reset: true,
            minor: true,
            major: true,
            start: GridPosition::new(-5, 3),
            goal: GridPosition::new(10, 1),
            real_start: pos(-4.5, 3.5),
            real_goal: pos(10.5, 1.5),
        };
        assert_eq!(planner.do_work(&request).err(), Some(OutOfGrid));
        assert!(
            planner
                .plan_to_convergence(pos(-4.5, 3.5), pos(10.5, 1.5))
                .is_none()
        );
    }

    #[test]
    fn the_pass_flags_follow_the_thread_loop() {
        let mut requests = Requests {
            start: GridPosition::new(0, 0),
            real_start: Translation2d::ZERO,
            goal: GridPosition::new(0, 0),
            real_goal: Translation2d::ZERO,
            minor: true,
            major: true,
            reset: true,
        };
        let mut passes = Vec::new();
        let mut eps = 2.5;
        while let Some(r) = requests.take_pass(eps) {
            passes.push((r.reset, r.minor, r.major));
            if !r.minor {
                eps -= 0.5;
            }
        }
        assert_eq!(
            passes,
            [
                (true, true, true),
                (false, false, true),
                (false, false, true),
                (false, false, true)
            ]
        );
    }

    #[test]
    fn the_thread_finds_a_path_and_stops_when_dropped() {
        let pathfinder = LocalAdStar::with_grid(walled_grid());
        pathfinder.set_start_position(pos(1.5, 1.5));
        pathfinder.set_goal_position(pos(10.5, 1.5));

        let started = Instant::now();
        let path = loop {
            if pathfinder.is_new_path_available()
                && let Some(path) = pathfinder.get_current_path(
                    PathConstraints::new(3.0, 3.0, 4.0, 4.0),
                    GoalEndState::new(0.0, Rotation2d::ZERO),
                )
            {
                break path;
            }
            assert!(started.elapsed().as_secs() < 30, "no path from the thread");
            thread::sleep(Duration::from_millis(5));
        };
        assert!(!pathfinder.is_new_path_available());
        assert_eq!(path.waypoints().first().unwrap().anchor, pos(1.5, 1.5));
        assert_eq!(path.waypoints().last().unwrap().anchor, pos(10.5, 1.5));
        assert!(path.ideal_starting_state().is_none());
        drop(pathfinder);
    }

    #[test]
    fn a_goal_in_an_obstacle_moves_and_a_new_goal_makes_a_new_path() {
        let pathfinder = LocalAdStar::with_grid(walled_grid());
        pathfinder.set_start_position(pos(1.5, 1.5));
        pathfinder.set_goal_position(pos(6.5, 3.5));
        let wait_for_path = |expected_end: Translation2d| {
            let started = Instant::now();
            loop {
                if pathfinder.is_new_path_available()
                    && let Some(path) = pathfinder.get_current_path(
                        PathConstraints::new(3.0, 3.0, 4.0, 4.0),
                        GoalEndState::new(0.0, Rotation2d::ZERO),
                    )
                    && path.waypoints().last().unwrap().anchor == expected_end
                {
                    return;
                }
                assert!(started.elapsed().as_secs() < 30, "no path from the thread");
                thread::sleep(Duration::from_millis(5));
            }
        };
        wait_for_path(pos(6.5, 3.5));
        pathfinder.set_goal_position(pos(3.5, 5.5));
        wait_for_path(pos(3.5, 5.5));
    }
}

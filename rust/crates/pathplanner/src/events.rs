use crate::path::PathPlannerPath;
use crate::trajectory::PathPlannerTrajectory;
use crate::util::JavaPriorityQueue;
use commands::{Command, CommandScheduler, Commands, EventLoop, SubsystemId, Trigger};
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

thread_local! {
    static EVENT_LOOP: Rc<EventLoop> = Rc::new(EventLoop::new());
    static EVENT_CONDITIONS: RefCell<HashMap<String, bool>> = RefCell::new(HashMap::new());
    static ZONE_CONDITIONS: RefCell<HashMap<String, bool>> = RefCell::new(HashMap::new());
}

fn condition(
    map: &'static std::thread::LocalKey<RefCell<HashMap<String, bool>>>,
    name: &str,
) -> impl Fn() -> bool + 'static {
    map.with(|m| {
        m.borrow_mut().entry(name.to_string()).or_insert(false);
    });
    let name = name.to_string();
    move || map.with(|m| m.borrow().get(&name).copied().unwrap_or(false))
}

fn set_condition(
    map: &'static std::thread::LocalKey<RefCell<HashMap<String, bool>>>,
    name: &str,
    value: bool,
) {
    map.with(|m| m.borrow_mut().insert(name.to_string(), value));
}

/// The loop that polls the global event and zone triggers. Path following commands poll it.
fn event_loop() -> Rc<EventLoop> {
    EVENT_LOOP.with(Rc::clone)
}

/// A trigger controlled by the event markers and zones of the path being followed.
pub struct EventTrigger;

impl EventTrigger {
    /// Polled by the event scheduler's loop, so it only updates while a path is followed.
    /// `name` is the event marker name from the GUI.
    #[allow(clippy::new_ret_no_self)]
    pub fn new(name: &str) -> Trigger {
        Self::with_loop(event_loop(), name)
    }

    pub fn with_loop(event_loop: Rc<EventLoop>, name: &str) -> Trigger {
        Trigger::with_loop(event_loop, condition(&EVENT_CONDITIONS, name))
    }
}

/// A trigger that is high while the robot is inside a point towards zone.
pub struct PointTowardsZoneTrigger;

impl PointTowardsZoneTrigger {
    #[allow(clippy::new_ret_no_self)]
    pub fn new(name: &str) -> Trigger {
        Self::with_loop(event_loop(), name)
    }

    pub fn with_loop(event_loop: Rc<EventLoop>, name: &str) -> Trigger {
        Trigger::with_loop(event_loop, condition(&ZONE_CONDITIONS, name))
    }
}

/// Something that happens at a trajectory time while a path is followed.
#[derive(Clone, Debug)]
pub enum Event {
    ScheduleCommand {
        timestamp: f64,
        command: Command,
    },
    CancelCommand {
        timestamp: f64,
        command: Command,
    },
    /// Raises a trigger, then lowers it on the next loop.
    OneShotTrigger {
        timestamp: f64,
        name: String,
    },
    Trigger {
        timestamp: f64,
        name: String,
        active: bool,
    },
    PointTowardsZone {
        timestamp: f64,
        name: String,
        active: bool,
    },
}

impl Event {
    pub fn timestamp_seconds(&self) -> f64 {
        match self {
            Event::ScheduleCommand { timestamp, .. }
            | Event::CancelCommand { timestamp, .. }
            | Event::OneShotTrigger { timestamp, .. }
            | Event::Trigger { timestamp, .. }
            | Event::PointTowardsZone { timestamp, .. } => *timestamp,
        }
    }

    pub fn set_timestamp(&mut self, seconds: f64) {
        match self {
            Event::ScheduleCommand { timestamp, .. }
            | Event::CancelCommand { timestamp, .. }
            | Event::OneShotTrigger { timestamp, .. }
            | Event::Trigger { timestamp, .. }
            | Event::PointTowardsZone { timestamp, .. } => *timestamp = seconds,
        }
    }

    pub fn copy_with_timestamp(&self, seconds: f64) -> Event {
        let mut copy = self.clone();
        copy.set_timestamp(seconds);
        copy
    }

    fn handle_event(&self, scheduler: &mut EventScheduler) {
        match self {
            Event::ScheduleCommand { command, .. } => scheduler.schedule_command(command),
            Event::CancelCommand { command, .. } => scheduler.cancel_command(command),
            Event::OneShotTrigger { name, .. } => {
                set_condition(&EVENT_CONDITIONS, name, true);
                // The main scheduler runs the reset, so the event scheduler cannot cancel it
                // before the trigger drops.
                let name = name.clone();
                let reset = Commands::sequence([
                    Commands::wait_seconds(0.0),
                    Commands::run_once(move || set_condition(&EVENT_CONDITIONS, &name, false), &[]),
                ])
                .ignoring_disable(true);
                CommandScheduler::get_instance().schedule(&reset);
            }
            Event::Trigger { name, active, .. } => set_condition(&EVENT_CONDITIONS, name, *active),
            Event::PointTowardsZone { name, active, .. } => {
                set_condition(&ZONE_CONDITIONS, name, *active);
            }
        }
    }

    /// Called for events a path following command ended before reaching.
    fn cancel_event(&self) {
        match self {
            Event::Trigger { name, active, .. } if !active => {
                set_condition(&EVENT_CONDITIONS, name, false);
            }
            Event::PointTowardsZone { name, active, .. } if !active => {
                set_condition(&ZONE_CONDITIONS, name, false);
            }
            _ => {}
        }
    }
}

/// Runs the events of a trajectory while a path following command executes.
///
/// The command running it must require everything the event commands require.
pub struct EventScheduler {
    // Insertion ordered, with false for a command that ended or was cancelled.
    event_commands: Vec<(Command, bool)>,
    upcoming_events: JavaPriorityQueue<Event>,
}

impl Default for EventScheduler {
    fn default() -> Self {
        Self::new()
    }
}

impl EventScheduler {
    pub fn new() -> Self {
        Self {
            event_commands: Vec::new(),
            upcoming_events: JavaPriorityQueue::new(Event::timestamp_seconds),
        }
    }

    /// Call from the owning command's `initialize`.
    pub fn initialize(&mut self, trajectory: &PathPlannerTrajectory) {
        self.event_commands.clear();
        self.upcoming_events.clear();
        for event in trajectory.events() {
            self.upcoming_events.push(event.clone());
        }
    }

    /// Call from the owning command's `execute` with the time along the trajectory.
    pub fn execute(&mut self, time: f64) {
        while self
            .upcoming_events
            .peek()
            .is_some_and(|e| time >= e.timestamp_seconds())
        {
            if let Some(event) = self.upcoming_events.pop() {
                event.handle_event(self);
            }
        }

        for i in 0..self.event_commands.len() {
            if !self.event_commands[i].1 {
                continue;
            }
            let command = self.event_commands[i].0.clone();
            command.execute();
            if command.is_finished() {
                command.end(false);
                self.event_commands[i].1 = false;
            }
        }

        event_loop().poll();
    }

    /// Call from the owning command's `end`.
    pub fn end(&mut self) {
        for (command, active) in &self.event_commands {
            if *active {
                command.end(true);
            }
        }
        for event in self.upcoming_events.iter() {
            event.cancel_event();
        }
        self.event_commands.clear();
        self.upcoming_events.clear();
    }

    /// Subsystems the marker commands of a path require.
    pub fn get_scheduler_requirements(path: &PathPlannerPath) -> Vec<SubsystemId> {
        let mut all = Vec::new();
        for marker in path.event_markers() {
            if let Some(command) = &marker.command {
                for id in command.get_requirements() {
                    if !all.contains(id) {
                        all.push(*id);
                    }
                }
            }
        }
        all
    }

    /// Cancels running commands that share requirements with `command`, then starts it.
    fn schedule_command(&mut self, command: &Command) {
        let requirements = command.get_requirements();
        for i in 0..self.event_commands.len() {
            if !self.event_commands[i].1 {
                continue;
            }
            let running = self.event_commands[i].0.clone();
            if running
                .get_requirements()
                .iter()
                .any(|r| requirements.contains(r))
            {
                running.end(true);
                self.event_commands[i].1 = false;
            }
        }

        command.initialize();
        match self.event_commands.iter_mut().find(|(c, _)| c == command) {
            Some(entry) => entry.1 = true,
            None => self.event_commands.push((command.clone(), true)),
        }
    }

    fn cancel_command(&mut self, command: &Command) {
        if let Some(entry) = self
            .event_commands
            .iter_mut()
            .find(|(c, active)| c == command && *active)
        {
            command.end(true);
            entry.1 = false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::RobotConfig;
    use crate::path::{EventMarker, GoalEndState, IdealStartingState, PathConstraints};
    use crate::util::robot_config;
    use commands::testing::FakeClock;
    use std::cell::Cell;
    use wpimath::geometry::{Pose2d, Rotation2d};
    use wpimath::kinematics::ChassisSpeeds;

    type Log = Rc<RefCell<Vec<String>>>;

    /// Logs init and end, requires `id`, and never finishes.
    fn logging_command(log: &Log, name: &str, id: SubsystemId) -> Command {
        let (init_log, end_log) = (log.clone(), log.clone());
        let (init_name, end_name) = (name.to_string(), name.to_string());
        Commands::start_end(
            move || init_log.borrow_mut().push(format!("{init_name} init")),
            move || end_log.borrow_mut().push(format!("{end_name} end")),
            &[id],
        )
    }

    fn path_with(markers: Vec<EventMarker>) -> PathPlannerPath {
        PathPlannerPath::new(
            PathPlannerPath::waypoints_from_poses(&[
                Pose2d::from_xy(0.0, 0.0, Rotation2d::ZERO),
                Pose2d::from_xy(5.0, 0.0, Rotation2d::ZERO),
            ]),
            Vec::new(),
            Vec::new(),
            Vec::new(),
            markers,
            PathConstraints::new(3.0, 3.0, 4.0, 4.0),
            Some(IdealStartingState::new(0.0, Rotation2d::ZERO)),
            GoalEndState::new(0.0, Rotation2d::ZERO),
            false,
        )
    }

    fn marker(name: &str, position: f64, end: f64, command: Option<Command>) -> EventMarker {
        EventMarker {
            trigger_name: name.into(),
            position,
            end_position: end,
            command,
        }
    }

    fn trajectory(path: &PathPlannerPath, config: &RobotConfig) -> PathPlannerTrajectory {
        path.generate_trajectory(ChassisSpeeds::default(), Rotation2d::ZERO, config)
    }

    /// Runs the scheduler over the whole trajectory in 20 ms steps.
    fn run_through(scheduler: &mut EventScheduler, trajectory: &PathPlannerTrajectory) {
        let mut t = 0.0;
        while t <= trajectory.total_time_seconds() {
            scheduler.execute(t);
            t += 0.02;
        }
    }

    #[test]
    fn marker_commands_start_at_their_position_and_stop_at_their_end() {
        let log: Log = Rc::default();
        let id = SubsystemId::new();
        let path = path_with(vec![marker(
            "m",
            0.2,
            0.6,
            Some(logging_command(&log, "a", id)),
        )]);
        let trajectory = trajectory(&path, &robot_config());

        let kinds: Vec<_> = trajectory
            .events()
            .iter()
            .map(|e| match e {
                Event::ScheduleCommand { .. } => "schedule",
                Event::CancelCommand { .. } => "cancel",
                Event::Trigger { active: true, .. } => "on",
                Event::Trigger { active: false, .. } => "off",
                _ => "other",
            })
            .collect();
        assert_eq!(kinds.len(), 4);
        assert!(kinds.contains(&"schedule") && kinds.contains(&"cancel"));
        let times: Vec<f64> = trajectory
            .events()
            .iter()
            .map(Event::timestamp_seconds)
            .collect();
        assert!(times[0] > 0.0 && times.iter().all(|t| *t < trajectory.total_time_seconds()));

        let mut scheduler = EventScheduler::new();
        scheduler.initialize(&trajectory);
        scheduler.execute(0.0);
        assert!(log.borrow().is_empty());
        run_through(&mut scheduler, &trajectory);
        assert_eq!(*log.borrow(), ["a init", "a end"]);
    }

    #[test]
    fn a_new_command_cancels_a_running_one_it_shares_a_requirement_with() {
        let log: Log = Rc::default();
        let id = SubsystemId::new();
        let path = path_with(vec![
            marker("a", 0.2, -1.0, Some(logging_command(&log, "a", id))),
            marker("b", 0.5, -1.0, Some(logging_command(&log, "b", id))),
        ]);
        let trajectory = trajectory(&path, &robot_config());

        let mut scheduler = EventScheduler::new();
        scheduler.initialize(&trajectory);
        run_through(&mut scheduler, &trajectory);
        assert_eq!(*log.borrow(), ["a init", "a end", "b init"]);

        // Ending the path interrupts what is still running.
        scheduler.end();
        assert_eq!(log.borrow().last().map(String::as_str), Some("b end"));
    }

    #[test]
    fn requirements_of_all_marker_commands_are_collected() {
        let (one, two) = (SubsystemId::new(), SubsystemId::new());
        let log: Log = Rc::default();
        let path = path_with(vec![
            marker("a", 0.2, -1.0, Some(logging_command(&log, "a", one))),
            marker("b", 0.5, -1.0, Some(logging_command(&log, "b", two))),
            marker("c", 0.7, -1.0, Some(logging_command(&log, "c", one))),
            marker("d", 0.9, -1.0, None),
        ]);
        assert_eq!(
            EventScheduler::get_scheduler_requirements(&path),
            [one, two]
        );
    }

    #[test]
    fn a_zoned_trigger_is_high_inside_its_zone() {
        let clock = FakeClock::install();
        let _ = clock;
        let path = path_with(vec![marker("zoned-test", 0.3, 0.7, None)]);
        let trajectory = trajectory(&path, &robot_config());
        let zone = EventTrigger::new("zoned-test");
        assert!(!zone.get_as_boolean());

        let on = trajectory
            .events()
            .iter()
            .find_map(|e| match e {
                Event::Trigger {
                    timestamp,
                    active: true,
                    ..
                } => Some(*timestamp),
                _ => None,
            })
            .unwrap();
        let off = trajectory
            .events()
            .iter()
            .find_map(|e| match e {
                Event::Trigger {
                    timestamp,
                    active: false,
                    ..
                } => Some(*timestamp),
                _ => None,
            })
            .unwrap();
        assert!(on < off);

        let mut scheduler = EventScheduler::new();
        scheduler.initialize(&trajectory);
        scheduler.execute(on - 0.01);
        assert!(!zone.get_as_boolean());
        scheduler.execute(on);
        assert!(zone.get_as_boolean());
        scheduler.execute((on + off) / 2.0);
        assert!(zone.get_as_boolean());
        scheduler.execute(off);
        assert!(!zone.get_as_boolean());
    }

    #[test]
    fn ending_a_path_inside_a_zone_lowers_the_trigger() {
        let path = path_with(vec![marker("cut-short", 0.3, 0.7, None)]);
        let trajectory = trajectory(&path, &robot_config());
        let zone = EventTrigger::new("cut-short");

        let mut scheduler = EventScheduler::new();
        scheduler.initialize(&trajectory);
        scheduler.execute(trajectory.total_time_seconds() * 0.4);
        assert!(zone.get_as_boolean());
        scheduler.end();
        assert!(!zone.get_as_boolean());
    }

    #[test]
    fn a_one_shot_trigger_is_high_until_the_next_scheduler_loop() {
        let clock = FakeClock::install();
        let path = path_with(vec![marker("one-shot-test", 0.5, -1.0, None)]);
        let trajectory = trajectory(&path, &robot_config());
        let fired = Rc::new(Cell::new(0));
        let count = fired.clone();
        EventTrigger::new("one-shot-test")
            .on_true(Commands::run_once(move || count.set(count.get() + 1), &[]));

        let mut scheduler = EventScheduler::new();
        scheduler.initialize(&trajectory);
        let at = trajectory.events()[0].timestamp_seconds();
        scheduler.execute(at);
        assert_eq!(fired.get(), 1);

        // The main scheduler runs the reset, so a second execute sees it low again.
        for _ in 0..2 {
            clock.advance(0.02);
            CommandScheduler::get_instance().run();
        }
        scheduler.execute(at + 0.02);
        assert_eq!(fired.get(), 1);
    }

    #[test]
    fn point_towards_zone_triggers_follow_their_events() {
        let mut scheduler = EventScheduler::new();
        let trigger = PointTowardsZoneTrigger::new("zone-test");
        assert!(!trigger.get_as_boolean());
        let on = Event::PointTowardsZone {
            timestamp: 0.0,
            name: "zone-test".into(),
            active: true,
        };
        on.handle_event(&mut scheduler);
        assert!(trigger.get_as_boolean());
        let off = on.copy_with_timestamp(1.0);
        assert_eq!(off.timestamp_seconds(), 1.0);
        let Event::PointTowardsZone { active, .. } = &off else {
            unreachable!()
        };
        assert!(*active);
        Event::PointTowardsZone {
            timestamp: 1.0,
            name: "zone-test".into(),
            active: false,
        }
        .handle_event(&mut scheduler);
        assert!(!trigger.get_as_boolean());
    }

    #[test]
    fn events_at_the_same_time_come_out_in_java_order() {
        // Two markers at one position: the heap order decides which event runs first.
        let path = path_with(vec![
            marker("first", 0.5, -1.0, None),
            marker("second", 0.5, -1.0, None),
            marker("third", 0.5, -1.0, None),
        ]);
        let trajectory = trajectory(&path, &robot_config());
        let names: Vec<String> = trajectory
            .events()
            .iter()
            .filter_map(|e| match e {
                Event::OneShotTrigger { name, .. } => Some(name.clone()),
                _ => None,
            })
            .collect();
        assert_eq!(names, ["first", "third", "second"]);
    }
}

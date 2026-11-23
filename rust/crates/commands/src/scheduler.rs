use crate::command::{Command, InterruptionBehavior};
use crate::env;
use crate::event_loop::EventLoop;
use crate::subsystem::{Subsystem, SubsystemId};
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

struct Entry {
    id: SubsystemId,
    subsystem: Option<Rc<dyn Subsystem>>,
    default: Option<Command>,
}

struct State {
    scheduled: Vec<Command>,
    requirements: HashMap<SubsystemId, Command>,
    entries: Vec<Entry>,
    active_loop: Rc<EventLoop>,
    disabled: bool,
    in_run_loop: bool,
    to_schedule: Vec<Command>,
    to_cancel: Vec<Command>,
    to_end: Vec<Command>,
    ending: Vec<Command>,
    period: f64,
}

/// Port of WPILib's `CommandScheduler`. No borrow of the state is held while a
/// command, subsystem or trigger callback runs, so callbacks may schedule or
/// cancel freely.
pub struct CommandScheduler {
    state: RefCell<State>,
    default_loop: Rc<EventLoop>,
}

thread_local! {
    static INSTANCE: Rc<CommandScheduler> = Rc::new(CommandScheduler::new());
}

const DEFAULT_PERIOD: f64 = 0.02;

impl CommandScheduler {
    fn new() -> Self {
        let default_loop = Rc::new(EventLoop::new());
        Self {
            state: RefCell::new(State {
                scheduled: Vec::new(),
                requirements: HashMap::new(),
                entries: Vec::new(),
                active_loop: default_loop.clone(),
                disabled: false,
                in_run_loop: false,
                to_schedule: Vec::new(),
                to_cancel: Vec::new(),
                to_end: Vec::new(),
                ending: Vec::new(),
                period: DEFAULT_PERIOD,
            }),
            default_loop,
        }
    }

    pub fn get_instance() -> Rc<CommandScheduler> {
        INSTANCE.with(Rc::clone)
    }

    /// Loop time above which `run` prints an overrun message.
    pub fn set_period(&self, seconds: f64) {
        self.state.borrow_mut().period = seconds;
    }

    pub fn get_default_button_loop(&self) -> Rc<EventLoop> {
        self.default_loop.clone()
    }

    pub fn get_active_button_loop(&self) -> Rc<EventLoop> {
        self.state.borrow().active_loop.clone()
    }

    pub fn set_active_button_loop(&self, event_loop: Rc<EventLoop>) {
        self.state.borrow_mut().active_loop = event_loop;
    }

    pub fn register_subsystem(&self, subsystem: Rc<dyn Subsystem>) {
        let id = subsystem.id();
        let mut st = self.state.borrow_mut();
        match st.entries.iter_mut().find(|e| e.id == id) {
            Some(e) if e.subsystem.is_some() => {
                eprintln!("Warning: Tried to register an already-registered subsystem");
            }
            Some(e) => e.subsystem = Some(subsystem),
            None => st.entries.push(Entry {
                id,
                subsystem: Some(subsystem),
                default: None,
            }),
        }
    }

    pub fn unregister_subsystem(&self, id: SubsystemId) {
        self.state.borrow_mut().entries.retain(|e| e.id != id);
    }

    pub fn unregister_all_subsystems(&self) {
        self.state.borrow_mut().entries.clear();
    }

    /// Panics if the command does not require the subsystem or is composed.
    /// Unlike Java, this does not make the scheduler call the subsystem's
    /// `periodic`; only [`Self::register_subsystem`] does.
    pub fn set_default_command(&self, id: SubsystemId, command: Command) {
        command.require_not_composed();
        assert!(
            command.has_requirement(id),
            "Default commands must require their subsystem!"
        );
        if command.get_interruption_behavior() == InterruptionBehavior::CancelIncoming {
            eprintln!(
                "Warning: Registering a non-interruptible default command!\nThis will likely \
                 prevent any other commands from requiring this subsystem."
            );
        }
        self.set_default(id, Some(command));
    }

    pub fn remove_default_command(&self, id: SubsystemId) {
        self.set_default(id, None);
    }

    fn set_default(&self, id: SubsystemId, command: Option<Command>) {
        let mut st = self.state.borrow_mut();
        match st.entries.iter_mut().find(|e| e.id == id) {
            Some(e) => e.default = command,
            None => st.entries.push(Entry {
                id,
                subsystem: None,
                default: command,
            }),
        }
    }

    pub fn get_default_command(&self, id: SubsystemId) -> Option<Command> {
        let st = self.state.borrow();
        st.entries.iter().find(|e| e.id == id)?.default.clone()
    }

    pub fn requiring(&self, id: SubsystemId) -> Option<Command> {
        self.state.borrow().requirements.get(&id).cloned()
    }

    pub fn is_scheduled(&self, command: &Command) -> bool {
        self.state.borrow().scheduled.contains(command)
    }

    pub fn disable(&self) {
        self.state.borrow_mut().disabled = true;
    }

    pub fn enable(&self) {
        self.state.borrow_mut().disabled = false;
    }

    /// Panics if the command is composed. While the run loop executes, the
    /// request is queued and applied after the loop.
    pub fn schedule(&self, command: &Command) {
        {
            let mut st = self.state.borrow_mut();
            if st.in_run_loop {
                if !st.to_schedule.contains(command) {
                    st.to_schedule.push(command.clone());
                }
                return;
            }
        }
        command.require_not_composed();
        let skip = {
            let st = self.state.borrow();
            st.disabled || st.scheduled.contains(command)
        } || (env::robot_state().is_disabled() && !command.runs_when_disabled());
        if skip {
            return;
        }

        let mut requiring: Vec<Command> = Vec::new();
        {
            let st = self.state.borrow();
            for r in command.get_requirements() {
                if let Some(c) = st.requirements.get(r)
                    && !requiring.contains(c)
                {
                    requiring.push(c.clone());
                }
            }
        }
        if requiring
            .iter()
            .any(|c| c.get_interruption_behavior() == InterruptionBehavior::CancelIncoming)
        {
            return;
        }
        for c in &requiring {
            self.cancel_now(c);
        }
        self.init_command(command);
    }

    fn init_command(&self, command: &Command) {
        {
            let mut st = self.state.borrow_mut();
            if !st.scheduled.contains(command) {
                st.scheduled.push(command.clone());
            }
            for r in command.get_requirements() {
                st.requirements.insert(*r, command.clone());
            }
        }
        command.initialize();
        self.flush_deferred_cancels();
    }

    /// `end(true)` calls postponed because the command was cancelled from inside
    /// its own `initialize`. Commands still inside a callback stay queued.
    fn flush_deferred_cancels(&self) {
        let ready: Vec<Command> = {
            let mut st = self.state.borrow_mut();
            let (busy, ready) = std::mem::take(&mut st.to_end)
                .into_iter()
                .partition(|c| c.is_busy());
            st.to_end = busy;
            ready
        };
        for c in &ready {
            c.end(true);
        }
    }

    pub fn cancel(&self, command: &Command) {
        self.cancel_now(command);
    }

    fn cancel_now(&self, command: &Command) {
        {
            let mut st = self.state.borrow_mut();
            if st.ending.contains(command) {
                return;
            }
            if st.in_run_loop {
                st.to_cancel.push(command.clone());
                return;
            }
            if !st.scheduled.contains(command) {
                return;
            }
            if command.is_busy() {
                // Cancelled from inside its own initialize: a callback cannot
                // re-enter itself, so leave the books now and call end later.
                st.scheduled.retain(|c| c != command);
                let reqs = command.get_requirements();
                st.requirements.retain(|k, _| !reqs.contains(k));
                st.to_end.push(command.clone());
                return;
            }
            st.ending.push(command.clone());
        }
        command.end(true);
        self.remove_finished(command);
    }

    fn remove_finished(&self, command: &Command) {
        let mut st = self.state.borrow_mut();
        if let Some(i) = st.ending.iter().position(|c| c == command) {
            st.ending.remove(i);
        }
        st.scheduled.retain(|c| c != command);
        let reqs = command.get_requirements();
        st.requirements.retain(|k, _| !reqs.contains(k));
    }

    pub fn cancel_all(&self) {
        let all = self.state.borrow().scheduled.clone();
        for c in &all {
            self.cancel_now(c);
        }
    }

    /// One scheduler iteration: subsystem periodics, button polling, commands,
    /// queued schedule and cancel requests, then default commands.
    pub fn run(&self) {
        if self.state.borrow().disabled {
            return;
        }
        let start = env::now_seconds();

        let simulation = env::robot_state().is_simulation();
        let mut i = 0;
        while let Some(entry) = self.subsystem_at(i) {
            if let Some(subsystem) = entry {
                subsystem.periodic();
                if simulation {
                    subsystem.simulation_periodic();
                }
            }
            i += 1;
        }

        let button_loop = self.state.borrow().active_loop.clone();
        button_loop.poll();

        self.state.borrow_mut().in_run_loop = true;
        let is_disabled = env::robot_state().is_disabled();
        let mut i = 0;
        while let Some(command) = self.scheduled_at(i) {
            if is_disabled && !command.runs_when_disabled() {
                self.cancel_now(&command);
                i += 1;
                continue;
            }
            command.execute();
            if command.is_finished() {
                self.state.borrow_mut().ending.push(command.clone());
                command.end(false);
                self.remove_finished(&command);
            } else {
                i += 1;
            }
        }

        let (to_schedule, to_cancel) = {
            let mut st = self.state.borrow_mut();
            st.in_run_loop = false;
            (
                std::mem::take(&mut st.to_schedule),
                std::mem::take(&mut st.to_cancel),
            )
        };
        for c in &to_schedule {
            self.schedule(c);
        }
        for c in &to_cancel {
            self.cancel_now(c);
        }

        let mut i = 0;
        while let Some(default) = self.default_to_schedule(i) {
            if let Some(c) = default {
                self.schedule(&c);
            }
            i += 1;
        }

        if env::now_seconds() - start > self.state.borrow().period {
            println!("CommandScheduler loop overrun");
        }
    }

    /// `None` ends the walk; `Some(None)` is a default-command-only entry.
    fn subsystem_at(&self, i: usize) -> Option<Option<Rc<dyn Subsystem>>> {
        let st = self.state.borrow();
        Some(st.entries.get(i)?.subsystem.clone())
    }

    fn scheduled_at(&self, i: usize) -> Option<Command> {
        self.state.borrow().scheduled.get(i).cloned()
    }

    /// `None` ends the walk; `Some(None)` means this subsystem needs nothing.
    fn default_to_schedule(&self, i: usize) -> Option<Option<Command>> {
        let st = self.state.borrow();
        let entry = st.entries.get(i)?;
        if st.requirements.contains_key(&entry.id) {
            return Some(None);
        }
        Some(entry.default.clone())
    }
}

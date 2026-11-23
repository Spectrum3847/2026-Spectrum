use crate::command::Command;
use crate::factories::Commands;
use crate::scheduler::CommandScheduler;
use std::sync::atomic::{AtomicU32, Ordering};

/// Identity of a subsystem for requirements. Any struct can hold one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SubsystemId(u32);

impl SubsystemId {
    pub fn new() -> Self {
        static NEXT: AtomicU32 = AtomicU32::new(0);
        Self(NEXT.fetch_add(1, Ordering::Relaxed))
    }
}

impl Default for SubsystemId {
    fn default() -> Self {
        Self::new()
    }
}

/// Register an `Rc` of it with [`CommandScheduler::register_subsystem`] to get
/// `periodic` calls and default-command scheduling.
pub trait Subsystem {
    fn id(&self) -> SubsystemId;

    fn name(&self) -> String {
        "Subsystem".to_string()
    }

    fn periodic(&self) {}

    fn simulation_periodic(&self) {}

    fn set_default_command(&self, command: Command) {
        CommandScheduler::get_instance().set_default_command(self.id(), command);
    }

    fn remove_default_command(&self) {
        CommandScheduler::get_instance().remove_default_command(self.id());
    }

    fn get_default_command(&self) -> Option<Command> {
        CommandScheduler::get_instance().get_default_command(self.id())
    }

    fn get_current_command(&self) -> Option<Command> {
        CommandScheduler::get_instance().requiring(self.id())
    }

    fn idle(&self) -> Command {
        Commands::idle(&[self.id()])
    }

    fn run_once(&self, action: impl FnMut() + 'static) -> Command
    where
        Self: Sized,
    {
        Commands::run_once(action, &[self.id()])
    }

    fn run(&self, action: impl FnMut() + 'static) -> Command
    where
        Self: Sized,
    {
        Commands::run(action, &[self.id()])
    }

    fn start_end(&self, start: impl FnMut() + 'static, end: impl FnMut() + 'static) -> Command
    where
        Self: Sized,
    {
        Commands::start_end(start, end, &[self.id()])
    }

    fn run_end(&self, run: impl FnMut() + 'static, end: impl FnMut() + 'static) -> Command
    where
        Self: Sized,
    {
        Commands::run_end(run, end, &[self.id()])
    }

    fn start_run(&self, start: impl FnMut() + 'static, run: impl FnMut() + 'static) -> Command
    where
        Self: Sized,
    {
        Commands::start_run(start, run, &[self.id()])
    }

    fn defer(&self, supplier: impl FnMut() -> Command + 'static) -> Command
    where
        Self: Sized,
    {
        Commands::defer(supplier, &[self.id()])
    }
}

/// Id and name holder for subsystem structs to embed.
#[derive(Clone, Debug)]
pub struct SubsystemBase {
    id: SubsystemId,
    name: String,
}

impl SubsystemBase {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            id: SubsystemId::new(),
            name: name.into(),
        }
    }

    pub fn id(&self) -> SubsystemId {
        self.id
    }

    pub fn name(&self) -> &str {
        &self.name
    }
}

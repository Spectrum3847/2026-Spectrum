//! Command bodies behind the factories and decorators: functional commands,
//! waits, wrappers, conditional, deferred and the four groups. Each follows the
//! WPILib v2026.2.1 class of the same name.

use crate::command::{Command, CommandBody, InterruptionBehavior};
use crate::subsystem::SubsystemId;
use crate::timer::Timer;
use std::rc::Rc;

fn dedup(requirements: &[SubsystemId]) -> Vec<SubsystemId> {
    let mut out = Vec::with_capacity(requirements.len());
    for r in requirements {
        if !out.contains(r) {
            out.push(*r);
        }
    }
    out
}

// Functional family: FunctionalCommand, InstantCommand, RunCommand, StartEndCommand, PrintCommand.

pub(crate) enum Finish {
    Never,
    Immediately,
    When(Box<dyn FnMut() -> bool>),
}

pub(crate) struct Functional {
    pub init: Option<Box<dyn FnMut()>>,
    pub execute: Option<Box<dyn FnMut()>>,
    pub end: Option<Box<dyn FnMut(bool)>>,
    pub finish: Finish,
}

impl CommandBody for Functional {
    fn initialize(&mut self) {
        if let Some(f) = &mut self.init {
            f();
        }
    }

    fn execute(&mut self) {
        if let Some(f) = &mut self.execute {
            f();
        }
    }

    fn end(&mut self, interrupted: bool) {
        if let Some(f) = &mut self.end {
            f(interrupted);
        }
    }

    fn is_finished(&mut self) -> bool {
        match &mut self.finish {
            Finish::Never => false,
            Finish::Immediately => true,
            Finish::When(f) => f(),
        }
    }
}

pub(crate) fn functional(
    name: &str,
    body: Functional,
    requirements: &[SubsystemId],
    runs_when_disabled: bool,
) -> Command {
    Command::from_parts(
        name.to_string(),
        Box::new(body),
        dedup(requirements),
        runs_when_disabled,
        InterruptionBehavior::CancelSelf,
    )
}

// WaitCommand and WaitUntilCommand run when disabled.

struct Wait {
    timer: Timer,
    duration: f64,
}

impl CommandBody for Wait {
    fn initialize(&mut self) {
        self.timer.restart();
    }

    fn end(&mut self, _interrupted: bool) {
        self.timer.stop();
    }

    fn is_finished(&mut self) -> bool {
        self.timer.has_elapsed(self.duration)
    }
}

pub(crate) fn wait(seconds: f64) -> Command {
    Command::from_parts(
        format!("WaitCommand: {seconds:?} seconds"),
        Box::new(Wait {
            timer: Timer::new(),
            duration: seconds,
        }),
        Vec::new(),
        true,
        InterruptionBehavior::CancelSelf,
    )
}

struct WaitUntil(Rc<dyn Fn() -> bool>);

impl CommandBody for WaitUntil {
    fn is_finished(&mut self) -> bool {
        (self.0)()
    }
}

pub(crate) fn wait_until(condition: Rc<dyn Fn() -> bool>) -> Command {
    Command::from_parts(
        "WaitUntilCommand".to_string(),
        Box::new(WaitUntil(condition)),
        Vec::new(),
        true,
        InterruptionBehavior::CancelSelf,
    )
}

// WrapperCommand

struct Wrapper {
    inner: Command,
    end_hook: Option<Box<dyn FnMut(bool)>>,
}

impl CommandBody for Wrapper {
    fn initialize(&mut self) {
        self.inner.initialize();
    }

    fn execute(&mut self) {
        self.inner.execute();
    }

    fn end(&mut self, interrupted: bool) {
        self.inner.end(interrupted);
        if let Some(hook) = &mut self.end_hook {
            hook(interrupted);
        }
    }

    fn is_finished(&mut self) -> bool {
        self.inner.is_finished()
    }
}

/// Wraps `inner`, overriding whichever of name, flags and end hook are given.
pub(crate) fn wrapper(
    inner: Command,
    name: Option<String>,
    runs_when_disabled: Option<bool>,
    interruption: Option<InterruptionBehavior>,
    end_hook: Option<Box<dyn FnMut(bool)>>,
) -> Command {
    Command::register_composed(std::slice::from_ref(&inner));
    let name = name.unwrap_or_else(|| inner.get_name());
    let requirements = inner.get_requirements().to_vec();
    let runs_when_disabled = runs_when_disabled.unwrap_or_else(|| inner.runs_when_disabled());
    let interruption = interruption.unwrap_or_else(|| inner.get_interruption_behavior());
    Command::from_parts(
        name,
        Box::new(Wrapper { inner, end_hook }),
        requirements,
        runs_when_disabled,
        interruption,
    )
}

// ConditionalCommand

struct Conditional {
    on_true: Command,
    on_false: Command,
    condition: Rc<dyn Fn() -> bool>,
    selected: Option<Command>,
}

impl CommandBody for Conditional {
    fn initialize(&mut self) {
        let selected = if (self.condition)() {
            self.on_true.clone()
        } else {
            self.on_false.clone()
        };
        selected.initialize();
        self.selected = Some(selected);
    }

    fn execute(&mut self) {
        if let Some(c) = &self.selected {
            c.execute();
        }
    }

    fn end(&mut self, interrupted: bool) {
        if let Some(c) = &self.selected {
            c.end(interrupted);
        }
    }

    fn is_finished(&mut self) -> bool {
        self.selected.as_ref().is_some_and(|c| c.is_finished())
    }
}

pub(crate) fn conditional(
    on_true: Command,
    on_false: Command,
    condition: Rc<dyn Fn() -> bool>,
) -> Command {
    Command::register_composed(&[on_true.clone(), on_false.clone()]);
    Meta::of(&[on_true.clone(), on_false.clone()], None).build(
        "ConditionalCommand",
        Conditional {
            on_true,
            on_false,
            condition,
            selected: None,
        },
    )
}

// DeferredCommand

struct Deferred {
    supplier: Box<dyn FnMut() -> Command>,
    command: Option<Command>,
}

impl CommandBody for Deferred {
    fn initialize(&mut self) {
        let command = (self.supplier)();
        Command::register_composed(std::slice::from_ref(&command));
        command.initialize();
        self.command = Some(command);
    }

    fn execute(&mut self) {
        if let Some(c) = &self.command {
            c.execute();
        }
    }

    fn end(&mut self, interrupted: bool) {
        if let Some(c) = self.command.take() {
            c.end(interrupted);
        }
    }

    fn is_finished(&mut self) -> bool {
        self.command.as_ref().is_none_or(|c| c.is_finished())
    }
}

pub(crate) fn deferred(
    supplier: Box<dyn FnMut() -> Command>,
    requirements: &[SubsystemId],
) -> Command {
    Command::from_parts(
        "DeferredCommand".to_string(),
        Box::new(Deferred {
            supplier,
            command: None,
        }),
        dedup(requirements),
        false,
        InterruptionBehavior::CancelSelf,
    )
}

// Groups

/// Requirements and flags a group derives from its members.
struct Meta {
    requirements: Vec<SubsystemId>,
    runs_when_disabled: bool,
    interruption: InterruptionBehavior,
}

impl Meta {
    /// With `overlap_message`, members may not share requirements (parallel groups).
    fn of(commands: &[Command], overlap_message: Option<&str>) -> Meta {
        let mut requirements: Vec<SubsystemId> = Vec::new();
        let mut runs_when_disabled = true;
        let mut interruption = InterruptionBehavior::CancelIncoming;
        for c in commands {
            if let Some(message) = overlap_message {
                assert!(
                    !c.get_requirements()
                        .iter()
                        .any(|r| requirements.contains(r)),
                    "{message}"
                );
            }
            for r in c.get_requirements() {
                if !requirements.contains(r) {
                    requirements.push(*r);
                }
            }
            runs_when_disabled &= c.runs_when_disabled();
            if c.get_interruption_behavior() == InterruptionBehavior::CancelSelf {
                interruption = InterruptionBehavior::CancelSelf;
            }
        }
        Meta {
            requirements,
            runs_when_disabled,
            interruption,
        }
    }
}

impl Meta {
    fn build(self, name: &str, body: impl CommandBody + 'static) -> Command {
        Command::from_parts(
            name.to_string(),
            Box::new(body),
            self.requirements,
            self.runs_when_disabled,
            self.interruption,
        )
    }
}

const PARALLEL_OVERLAP: &str =
    "Multiple commands in a parallel composition cannot require the same subsystems";
const DEADLINE_OVERLAP: &str =
    "Multiple commands in a parallel group cannot require the same subsystems";

struct Sequential {
    commands: Vec<Command>,
    current: Option<usize>,
}

impl CommandBody for Sequential {
    fn initialize(&mut self) {
        self.current = Some(0);
        if let Some(first) = self.commands.first() {
            first.initialize();
        }
    }

    fn execute(&mut self) {
        let Some(i) = self.current else { return };
        let Some(command) = self.commands.get(i) else {
            return;
        };
        command.execute();
        if command.is_finished() {
            command.end(false);
            self.current = Some(i + 1);
            if let Some(next) = self.commands.get(i + 1) {
                next.initialize();
            }
        }
    }

    fn end(&mut self, interrupted: bool) {
        if interrupted
            && let Some(i) = self.current
            && let Some(command) = self.commands.get(i)
        {
            command.end(true);
        }
        self.current = None;
    }

    fn is_finished(&mut self) -> bool {
        self.current == Some(self.commands.len())
    }
}

pub(crate) fn sequence(commands: Vec<Command>) -> Command {
    Command::register_composed(&commands);
    Meta::of(&commands, None).build(
        "SequentialCommandGroup",
        Sequential {
            commands,
            current: None,
        },
    )
}

struct Parallel {
    commands: Vec<(Command, bool)>,
}

impl CommandBody for Parallel {
    fn initialize(&mut self) {
        for (command, running) in &mut self.commands {
            command.initialize();
            *running = true;
        }
    }

    fn execute(&mut self) {
        for (command, running) in &mut self.commands {
            if !*running {
                continue;
            }
            command.execute();
            if command.is_finished() {
                command.end(false);
                *running = false;
            }
        }
    }

    fn end(&mut self, interrupted: bool) {
        if interrupted {
            for (command, running) in &self.commands {
                if *running {
                    command.end(true);
                }
            }
        }
    }

    fn is_finished(&mut self) -> bool {
        !self.commands.iter().any(|(_, running)| *running)
    }
}

pub(crate) fn parallel(commands: Vec<Command>) -> Command {
    Command::register_composed(&commands);
    Meta::of(&commands, Some(PARALLEL_OVERLAP)).build(
        "ParallelCommandGroup",
        Parallel {
            commands: commands.into_iter().map(|c| (c, false)).collect(),
        },
    )
}

struct Race {
    commands: Vec<Command>,
    finished: bool,
}

impl CommandBody for Race {
    fn initialize(&mut self) {
        self.finished = false;
        for command in &self.commands {
            command.initialize();
        }
    }

    fn execute(&mut self) {
        for command in &self.commands {
            command.execute();
            if command.is_finished() {
                self.finished = true;
            }
        }
    }

    fn end(&mut self, _interrupted: bool) {
        for command in &self.commands {
            command.end(!command.is_finished());
        }
    }

    fn is_finished(&mut self) -> bool {
        self.finished
    }
}

pub(crate) fn race(commands: Vec<Command>) -> Command {
    Command::register_composed(&commands);
    Meta::of(&commands, Some(PARALLEL_OVERLAP)).build(
        "ParallelRaceGroup",
        Race {
            commands,
            finished: true,
        },
    )
}

/// The deadline is `commands[0]`.
struct Deadline {
    commands: Vec<(Command, bool)>,
    finished: bool,
}

impl CommandBody for Deadline {
    fn initialize(&mut self) {
        for (command, running) in &mut self.commands {
            command.initialize();
            *running = true;
        }
        self.finished = false;
    }

    fn execute(&mut self) {
        for (i, (command, running)) in self.commands.iter_mut().enumerate() {
            if !*running {
                continue;
            }
            command.execute();
            if command.is_finished() {
                command.end(false);
                *running = false;
                if i == 0 {
                    self.finished = true;
                }
            }
        }
    }

    fn end(&mut self, _interrupted: bool) {
        for (command, running) in &self.commands {
            if *running {
                command.end(true);
            }
        }
    }

    fn is_finished(&mut self) -> bool {
        self.finished
    }
}

pub(crate) fn deadline(deadline: Command, others: Vec<Command>) -> Command {
    let mut commands = vec![deadline];
    commands.extend(others);
    Command::register_composed(&commands);
    Meta::of(&commands, Some(DEADLINE_OVERLAP)).build(
        "ParallelDeadlineGroup",
        Deadline {
            commands: commands.into_iter().map(|c| (c, false)).collect(),
            finished: true,
        },
    )
}

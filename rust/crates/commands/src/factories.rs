use crate::command::{BooleanSupplier, Command, IntoCommands};
use crate::groups::{self, Finish, Functional};
use crate::subsystem::SubsystemId;

/// Port of WPILib's `Commands`. Also stands in for `new InstantCommand`,
/// `RunCommand`, `StartEndCommand`, `FunctionalCommand`, `WaitCommand`,
/// `WaitUntilCommand`, `PrintCommand`, `ConditionalCommand`, `DeferredCommand`
/// and the group classes, which Java builds with `new`.
pub struct Commands;

impl Commands {
    pub fn none() -> Command {
        Self::run_once(|| {}, &[])
    }

    pub fn idle(requirements: &[SubsystemId]) -> Command {
        Self::run(|| {}, requirements)
    }

    pub fn run_once(action: impl FnMut() + 'static, requirements: &[SubsystemId]) -> Command {
        groups::functional(
            "InstantCommand",
            Functional {
                init: Some(Box::new(action)),
                execute: None,
                end: None,
                finish: Finish::Immediately,
            },
            requirements,
            false,
        )
    }

    pub fn run(action: impl FnMut() + 'static, requirements: &[SubsystemId]) -> Command {
        groups::functional(
            "RunCommand",
            Functional {
                init: None,
                execute: Some(Box::new(action)),
                end: None,
                finish: Finish::Never,
            },
            requirements,
            false,
        )
    }

    pub fn start_end(
        start: impl FnMut() + 'static,
        mut end: impl FnMut() + 'static,
        requirements: &[SubsystemId],
    ) -> Command {
        groups::functional(
            "StartEndCommand",
            Functional {
                init: Some(Box::new(start)),
                execute: None,
                end: Some(Box::new(move |_| end())),
                finish: Finish::Never,
            },
            requirements,
            false,
        )
    }

    pub fn run_end(
        run: impl FnMut() + 'static,
        mut end: impl FnMut() + 'static,
        requirements: &[SubsystemId],
    ) -> Command {
        groups::functional(
            "FunctionalCommand",
            Functional {
                init: None,
                execute: Some(Box::new(run)),
                end: Some(Box::new(move |_| end())),
                finish: Finish::Never,
            },
            requirements,
            false,
        )
    }

    pub fn start_run(
        start: impl FnMut() + 'static,
        run: impl FnMut() + 'static,
        requirements: &[SubsystemId],
    ) -> Command {
        groups::functional(
            "FunctionalCommand",
            Functional {
                init: Some(Box::new(start)),
                execute: Some(Box::new(run)),
                end: None,
                finish: Finish::Never,
            },
            requirements,
            false,
        )
    }

    pub fn functional(
        init: impl FnMut() + 'static,
        execute: impl FnMut() + 'static,
        end: impl FnMut(bool) + 'static,
        is_finished: impl FnMut() -> bool + 'static,
        requirements: &[SubsystemId],
    ) -> Command {
        groups::functional(
            "FunctionalCommand",
            Functional {
                init: Some(Box::new(init)),
                execute: Some(Box::new(execute)),
                end: Some(Box::new(end)),
                finish: Finish::When(Box::new(is_finished)),
            },
            requirements,
            false,
        )
    }

    /// Prints the message to stdout once. Runs when disabled.
    pub fn print(message: impl Into<String>) -> Command {
        let message = message.into();
        groups::functional(
            "PrintCommand",
            Functional {
                init: Some(Box::new(move || println!("{message}"))),
                execute: None,
                end: None,
                finish: Finish::Immediately,
            },
            &[],
            true,
        )
    }

    pub fn wait_seconds(seconds: f64) -> Command {
        groups::wait(seconds)
    }

    pub fn wait_until(condition: impl BooleanSupplier) -> Command {
        groups::wait_until(condition.into_supplier())
    }

    pub fn either(on_true: Command, on_false: Command, selector: impl BooleanSupplier) -> Command {
        groups::conditional(on_true, on_false, selector.into_supplier())
    }

    /// Runs the command the supplier returns at initialize time. A supplier
    /// cannot return null in Rust, so WPILib's null-command message is gone.
    pub fn defer(
        supplier: impl FnMut() -> Command + 'static,
        requirements: &[SubsystemId],
    ) -> Command {
        groups::deferred(Box::new(supplier), requirements)
    }

    pub fn sequence(commands: impl IntoCommands) -> Command {
        groups::sequence(commands.into_commands())
    }

    pub fn parallel(commands: impl IntoCommands) -> Command {
        groups::parallel(commands.into_commands())
    }

    pub fn race(commands: impl IntoCommands) -> Command {
        groups::race(commands.into_commands())
    }

    pub fn deadline(deadline: Command, others: impl IntoCommands) -> Command {
        groups::deadline(deadline, others.into_commands())
    }
}

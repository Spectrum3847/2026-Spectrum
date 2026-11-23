use crate::factories::Commands;
use crate::groups;
use crate::scheduler::CommandScheduler;
use crate::subsystem::SubsystemId;
use std::cell::{Cell, RefCell};
use std::fmt;
use std::hash::{Hash, Hasher};
use std::rc::Rc;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InterruptionBehavior {
    CancelSelf,
    CancelIncoming,
}

/// Lifecycle callbacks of a command. Requirements, name and flags live on the
/// [`Command`] handle, so the scheduler can read them while a callback runs.
pub trait CommandBody {
    fn initialize(&mut self) {}
    fn execute(&mut self) {}
    fn end(&mut self, _interrupted: bool) {}
    fn is_finished(&mut self) -> bool {
        false
    }
}

/// A condition read each time it is needed: closures and [`Trigger`](crate::Trigger)s.
pub trait BooleanSupplier {
    fn into_supplier(self) -> Rc<dyn Fn() -> bool>;
}

impl<F: Fn() -> bool + 'static> BooleanSupplier for F {
    fn into_supplier(self) -> Rc<dyn Fn() -> bool> {
        Rc::new(self)
    }
}

impl BooleanSupplier for Rc<dyn Fn() -> bool> {
    fn into_supplier(self) -> Rc<dyn Fn() -> bool> {
        self
    }
}

/// Accepted wherever Java takes `Command...`: one command, an array or a `Vec`.
pub trait IntoCommands {
    fn into_commands(self) -> Vec<Command>;
}

impl IntoCommands for Command {
    fn into_commands(self) -> Vec<Command> {
        vec![self]
    }
}

impl IntoCommands for Vec<Command> {
    fn into_commands(self) -> Vec<Command> {
        self
    }
}

impl<const N: usize> IntoCommands for [Command; N] {
    fn into_commands(self) -> Vec<Command> {
        self.into()
    }
}

struct Inner {
    body: RefCell<Box<dyn CommandBody>>,
    name: RefCell<String>,
    requirements: Vec<SubsystemId>,
    runs_when_disabled: bool,
    interruption: InterruptionBehavior,
    composed: Cell<bool>,
    // True while a body callback runs, so the scheduler defers re-entrant calls.
    busy: Cell<bool>,
}

/// Cheap shared handle to a command. Clones refer to the same command, so one
/// can be bound to a trigger and scheduled repeatedly.
///
/// A command put into a group, or wrapped by a decorator, is marked composed.
/// Scheduling it or composing it again panics, as in WPILib. The decorators take
/// `self`, so the check only matters for clones kept elsewhere.
#[derive(Clone)]
pub struct Command(Rc<Inner>);

impl PartialEq for Command {
    fn eq(&self, other: &Self) -> bool {
        Rc::ptr_eq(&self.0, &other.0)
    }
}

impl Eq for Command {}

impl Hash for Command {
    fn hash<H: Hasher>(&self, state: &mut H) {
        Rc::as_ptr(&self.0).hash(state);
    }
}

impl fmt::Debug for Command {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Command({})", self.get_name())
    }
}

/// Builds a command from a custom [`CommandBody`].
pub struct CommandBuilder {
    body: Box<dyn CommandBody>,
    name: String,
    requirements: Vec<SubsystemId>,
    runs_when_disabled: bool,
    interruption: InterruptionBehavior,
}

impl CommandBuilder {
    pub fn requires(mut self, requirements: &[SubsystemId]) -> Self {
        for r in requirements {
            if !self.requirements.contains(r) {
                self.requirements.push(*r);
            }
        }
        self
    }

    pub fn runs_when_disabled(mut self, value: bool) -> Self {
        self.runs_when_disabled = value;
        self
    }

    pub fn interruption_behavior(mut self, value: InterruptionBehavior) -> Self {
        self.interruption = value;
        self
    }

    pub fn build(self) -> Command {
        Command::from_parts(
            self.name,
            self.body,
            self.requirements,
            self.runs_when_disabled,
            self.interruption,
        )
    }
}

impl Command {
    pub fn builder(name: impl Into<String>, body: impl CommandBody + 'static) -> CommandBuilder {
        CommandBuilder {
            body: Box::new(body),
            name: name.into(),
            requirements: Vec::new(),
            runs_when_disabled: false,
            interruption: InterruptionBehavior::CancelSelf,
        }
    }

    pub(crate) fn from_parts(
        name: String,
        body: Box<dyn CommandBody>,
        requirements: Vec<SubsystemId>,
        runs_when_disabled: bool,
        interruption: InterruptionBehavior,
    ) -> Command {
        Command(Rc::new(Inner {
            body: RefCell::new(body),
            name: RefCell::new(name),
            requirements,
            runs_when_disabled,
            interruption,
            composed: Cell::new(false),
            busy: Cell::new(false),
        }))
    }

    fn with_body<R>(&self, f: impl FnOnce(&mut dyn CommandBody) -> R) -> R {
        let inner = &*self.0;
        let was_busy = inner.busy.replace(true);
        let result = {
            let mut body = inner
                .body
                .try_borrow_mut()
                .expect("command callback re-entered");
            f(&mut **body)
        };
        inner.busy.set(was_busy);
        result
    }

    pub fn initialize(&self) {
        self.with_body(|b| b.initialize());
    }

    pub fn execute(&self) {
        self.with_body(|b| b.execute());
    }

    pub fn end(&self, interrupted: bool) {
        self.with_body(|b| b.end(interrupted));
    }

    pub fn is_finished(&self) -> bool {
        self.with_body(|b| b.is_finished())
    }

    pub(crate) fn is_busy(&self) -> bool {
        self.0.busy.get()
    }

    pub fn get_name(&self) -> String {
        self.0.name.borrow().clone()
    }

    pub fn set_name(&self, name: impl Into<String>) {
        *self.0.name.borrow_mut() = name.into();
    }

    pub fn get_requirements(&self) -> &[SubsystemId] {
        &self.0.requirements
    }

    pub fn has_requirement(&self, subsystem: SubsystemId) -> bool {
        self.0.requirements.contains(&subsystem)
    }

    pub fn runs_when_disabled(&self) -> bool {
        self.0.runs_when_disabled
    }

    pub fn get_interruption_behavior(&self) -> InterruptionBehavior {
        self.0.interruption
    }

    pub fn is_composed(&self) -> bool {
        self.0.composed.get()
    }

    pub fn cancel(&self) {
        CommandScheduler::get_instance().cancel(self);
    }

    pub fn is_scheduled(&self) -> bool {
        CommandScheduler::get_instance().is_scheduled(self)
    }

    pub(crate) fn require_not_composed(&self) {
        assert!(
            !self.0.composed.get(),
            "Commands that have been composed may not be added to another composition or \
             scheduled individually!"
        );
    }

    /// Marks the commands as owned by a composition. Panics if any is already
    /// composed, scheduled, or listed twice.
    pub(crate) fn register_composed(commands: &[Command]) {
        for (i, c) in commands.iter().enumerate() {
            assert!(
                !commands[..i].contains(c),
                "Cannot compose a command twice in the same composition!"
            );
        }
        for c in commands {
            assert!(
                !c.is_scheduled(),
                "Commands that have been scheduled individually may not be added to a composition!"
            );
            c.require_not_composed();
        }
        for c in commands {
            c.0.composed.set(true);
        }
    }

    // Decorators

    pub fn with_name(self, name: impl Into<String>) -> Command {
        groups::wrapper(self, Some(name.into()), None, None, None)
    }

    pub fn ignoring_disable(self, runs_when_disabled: bool) -> Command {
        groups::wrapper(self, None, Some(runs_when_disabled), None, None)
    }

    pub fn with_interrupt_behavior(self, behavior: InterruptionBehavior) -> Command {
        groups::wrapper(self, None, None, Some(behavior), None)
    }

    pub fn finally_do(self, mut end: impl FnMut() + 'static) -> Command {
        groups::wrapper(self, None, None, None, Some(Box::new(move |_| end())))
    }

    pub fn finally_do_with(self, end: impl FnMut(bool) + 'static) -> Command {
        groups::wrapper(self, None, None, None, Some(Box::new(end)))
    }

    pub fn with_timeout(self, seconds: f64) -> Command {
        self.race_with(Commands::wait_seconds(seconds))
    }

    pub fn until(self, condition: impl BooleanSupplier) -> Command {
        self.race_with(Commands::wait_until(condition))
    }

    pub fn only_while(self, condition: impl BooleanSupplier) -> Command {
        let condition = condition.into_supplier();
        self.until(move || !condition())
    }

    pub fn before_starting(self, before: Command) -> Command {
        Commands::sequence([before, self])
    }

    pub fn and_then(self, next: impl IntoCommands) -> Command {
        let mut all = vec![self];
        all.extend(next.into_commands());
        Commands::sequence(all)
    }

    pub fn and_then_run(self, action: impl FnMut() + 'static) -> Command {
        self.and_then(Commands::run_once(action, &[]))
    }

    pub fn with_deadline(self, deadline: Command) -> Command {
        Commands::deadline(deadline, self)
    }

    pub fn deadline_for(self, others: impl IntoCommands) -> Command {
        Commands::deadline(self, others)
    }

    pub fn along_with(self, others: impl IntoCommands) -> Command {
        let mut all = vec![self];
        all.extend(others.into_commands());
        Commands::parallel(all)
    }

    pub fn race_with(self, others: impl IntoCommands) -> Command {
        let mut all = vec![self];
        all.extend(others.into_commands());
        Commands::race(all)
    }

    pub fn unless(self, condition: impl BooleanSupplier) -> Command {
        Commands::either(Commands::none(), self, condition)
    }

    pub fn only_if(self, condition: impl BooleanSupplier) -> Command {
        let condition = condition.into_supplier();
        self.unless(move || !condition())
    }
}

use crate::spectrum_lib::util::java_double;
use commands::{BooleanSupplier, Command, Commands, EventLoop, Trigger};
use std::cell::{Cell, RefCell};
use std::collections::HashMap;
use std::ops::Deref;
use std::rc::Rc;
use wpilib::dashboard::{Alert, AlertType};

thread_local! {
    /// Keyed by state name, so states that share a name share a value.
    static STATE_CONDITIONS: RefCell<HashMap<String, bool>> = RefCell::new(HashMap::new());
}

struct Inner {
    name: String,
    value: Cell<bool>,
    alert: Option<Alert>,
}

/// A named boolean state usable as a [`Trigger`]. Every instance built with the same name
/// reads and writes one shared value, and each raises an [`Alert`] in the States group.
///
/// Java's `SpectrumState extends Trigger`. Here it dereferences to its [`Trigger`] and
/// can be passed wherever a condition is taken. Clones are the same state.
#[derive(Clone)]
pub struct SpectrumState {
    trigger: Trigger,
    inner: Rc<Inner>,
}

impl Deref for SpectrumState {
    type Target = Trigger;

    fn deref(&self) -> &Trigger {
        &self.trigger
    }
}

impl BooleanSupplier for SpectrumState {
    fn into_supplier(self) -> Rc<dyn Fn() -> bool> {
        self.trigger.into_supplier()
    }
}

impl BooleanSupplier for &SpectrumState {
    fn into_supplier(self) -> Rc<dyn Fn() -> bool> {
        (&self.trigger).into_supplier()
    }
}

impl SpectrumState {
    /// Polled by the command scheduler's default event loop.
    pub fn new(name: &str) -> Self {
        let trigger = Trigger::new(poll_condition(name));
        Self::build(trigger, name, Some(state_alert(name)))
    }

    /// Polled by the given event loop instead of the scheduler's default one.
    pub fn with_event_loop(event_loop: Rc<EventLoop>, name: &str) -> Self {
        let trigger = Trigger::with_loop(event_loop, poll_condition(name));
        Self::build(trigger, name, Some(state_alert(name)))
    }

    /// Without the alert, which needs the dashboard natives.
    #[cfg(test)]
    pub(crate) fn without_alert(name: &str) -> Self {
        Self::build(Trigger::new(poll_condition(name)), name, None)
    }

    fn build(trigger: Trigger, name: &str, alert: Option<Alert>) -> Self {
        Self {
            trigger,
            inner: Rc::new(Inner {
                name: name.to_owned(),
                value: Cell::new(false),
                alert,
            }),
        }
    }

    pub fn trigger(&self) -> &Trigger {
        &self.trigger
    }

    pub fn set_state(&self, value: bool) {
        self.inner.value.set(value);
        if let Some(alert) = &self.inner.alert {
            alert.set(value);
        }
        set_condition(&self.inner.name, value);
    }

    pub fn set_true_while_running(&self) -> Command {
        let (on, off) = (self.clone(), self.clone());
        Commands::start_end(
            move || on.set_state(true),
            move || off.set_state(false),
            &[],
        )
        .ignoring_disable(true)
        .with_name(format!("{} state: TrueWhileRunning", self.inner.name))
    }

    /// Holds the state true for `time` seconds. The supplier is read when the command is
    /// built, not when it runs.
    pub fn set_true_for_time(&self, time: impl Fn() -> f64) -> Command {
        let (on, off) = (self.clone(), self.clone());
        Commands::run_once(move || on.set_state(true), &[])
            .along_with(Commands::wait_seconds(time()))
            .and_then_run(move || off.set_state(false))
            .ignoring_disable(true)
            .with_name(format!(
                "{} state: SetTrueForTime->{}",
                self.inner.name,
                java_double::to_string(time())
            ))
    }

    /// Holds the state false for `time` seconds. The supplier is read when the command is
    /// built, not when it runs.
    pub fn set_false_for_time(&self, time: impl Fn() -> f64) -> Command {
        let (off, on) = (self.clone(), self.clone());
        Commands::run_once(move || off.set_state(false), &[])
            .along_with(Commands::wait_seconds(time()))
            .and_then_run(move || on.set_state(true))
            // Java: this one is not ignoringDisable(true) like its siblings.
            .ignoring_disable(false)
            .with_name(format!(
                "{} state: SetFalseForTime->{}",
                self.inner.name,
                java_double::to_string(time())
            ))
    }

    /// Holds the state true for at most `time` seconds. `cancel_condition` ends the wait
    /// early and drops the state to false.
    pub fn set_true_for_time_with_cancel(
        &self,
        time: impl Fn() -> f64,
        cancel_condition: &Trigger,
    ) -> Command {
        let (on, off) = (self.clone(), self.clone());
        Commands::run_once(move || on.set_state(true), &[])
            .along_with(Commands::wait_seconds(time()).only_while(cancel_condition.negate()))
            .and_then_run(move || off.set_state(false))
            .ignoring_disable(true)
            .with_name(format!(
                "{} state: SetTrueForTimeWithCancel->{}",
                self.inner.name,
                java_double::to_string(time())
            ))
    }

    /// Sets the state false, then true after a 5 ms wait, so a binding polled between them
    /// still sees the false state.
    pub fn toggle_to_true(&self) -> Command {
        let end = self.clone();
        self.set_false()
            .and_then([Commands::wait_seconds(0.005), self.set_true()])
            .finally_do(move || end.set_state(true))
            .ignoring_disable(true)
            .with_name(format!("{} state: ToggleToTrue", self.inner.name))
    }

    /// Sets the state true, then false after a 5 ms wait, so a binding polled between them
    /// still sees the true state.
    pub fn toggle_to_false(&self) -> Command {
        let end = self.clone();
        self.set_true()
            .and_then([Commands::wait_seconds(0.005), self.set_false()])
            .finally_do(move || end.set_state(false))
            .ignoring_disable(true)
            .with_name(format!("{} state: ToggleToFalse", self.inner.name))
    }

    pub fn set(&self, value: bool) -> Command {
        let state = self.clone();
        Commands::run_once(move || state.set_state(value), &[]).ignoring_disable(true)
    }

    pub fn set_true(&self) -> Command {
        self.set(true)
            .with_name(format!("{} state: SetTrue", self.inner.name))
    }

    pub fn set_false(&self) -> Command {
        self.set(false)
            .with_name(format!("{} state: SetFalse", self.inner.name))
    }

    pub fn toggle(&self) -> Command {
        let state = self.clone();
        // Java: flips this instance's own last value, not the shared one, so two
        // instances with one name can disagree.
        Commands::run_once(move || state.set_state(!state.inner.value.get()), &[])
            .ignoring_disable(true)
            .with_name(format!("{} state: Toggle", self.inner.name))
    }
}

fn state_alert(name: &str) -> Alert {
    Alert::in_group("States", name, AlertType::Info)
}

/// A supplier reading the shared value for `name`, registering the name as false if it is new.
fn poll_condition(name: &str) -> impl Fn() -> bool + 'static {
    STATE_CONDITIONS.with(|m| {
        m.borrow_mut().entry(name.to_owned()).or_insert(false);
    });
    let name = name.to_owned();
    move || get_condition(&name)
}

fn get_condition(name: &str) -> bool {
    STATE_CONDITIONS.with(|m| m.borrow().get(name).copied().unwrap_or(false))
}

pub fn set_condition(name: &str, value: bool) {
    STATE_CONDITIONS.with(|m| {
        m.borrow_mut().insert(name.to_owned(), value);
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use commands::CommandScheduler;
    use commands::testing::FakeClock;

    #[test]
    fn a_new_state_starts_false() {
        let state = SpectrumState::without_alert("starts-false");
        assert!(!state.get_as_boolean());
    }

    #[test]
    fn states_with_one_name_share_a_value() {
        let a = SpectrumState::without_alert("shared");
        let b = SpectrumState::without_alert("shared");
        a.set_state(true);
        assert!(b.get_as_boolean());
        b.set_state(false);
        assert!(!a.get_as_boolean());
    }

    #[test]
    fn states_with_different_names_are_independent() {
        let a = SpectrumState::without_alert("one");
        let b = SpectrumState::without_alert("two");
        a.set_state(true);
        assert!(!b.get_as_boolean());
    }

    #[test]
    fn creating_a_state_does_not_reset_a_shared_value() {
        let a = SpectrumState::without_alert("kept");
        a.set_state(true);
        let b = SpectrumState::without_alert("kept");
        assert!(b.get_as_boolean());
    }

    #[test]
    fn set_condition_writes_the_shared_value() {
        let state = SpectrumState::without_alert("by-name");
        set_condition("by-name", true);
        assert!(state.get_as_boolean());
    }

    #[test]
    fn toggle_flips_the_instance_value_and_the_shared_one_follows() {
        let state = SpectrumState::without_alert("toggler");
        let toggle = state.toggle();
        CommandScheduler::get_instance().schedule(&toggle);
        CommandScheduler::get_instance().run();
        assert!(state.get_as_boolean());
    }

    #[test]
    fn toggle_uses_the_instances_own_last_value() {
        let a = SpectrumState::without_alert("stale");
        let b = SpectrumState::without_alert("stale");
        a.set_state(true);
        // b never set the state itself, so its own last value is still false.
        let toggle = b.toggle();
        CommandScheduler::get_instance().schedule(&toggle);
        CommandScheduler::get_instance().run();
        assert!(b.get_as_boolean());
    }

    #[test]
    fn command_names_follow_java() {
        let state = SpectrumState::without_alert("named");
        assert_eq!(state.set_true().get_name(), "named state: SetTrue");
        assert_eq!(state.set_false().get_name(), "named state: SetFalse");
        assert_eq!(state.toggle().get_name(), "named state: Toggle");
        assert_eq!(
            state.set_true_while_running().get_name(),
            "named state: TrueWhileRunning"
        );
        assert_eq!(
            state.set_true_for_time(|| 2.0).get_name(),
            "named state: SetTrueForTime->2.0"
        );
        assert_eq!(
            state.set_false_for_time(|| 0.25).get_name(),
            "named state: SetFalseForTime->0.25"
        );
        assert_eq!(
            state.toggle_to_true().get_name(),
            "named state: ToggleToTrue"
        );
        assert_eq!(
            state.toggle_to_false().get_name(),
            "named state: ToggleToFalse"
        );
    }

    #[test]
    fn set_true_for_time_holds_the_state_then_drops_it() {
        let clock = FakeClock::install();
        let state = SpectrumState::without_alert("timed");
        let command = state.set_true_for_time(|| 1.0);
        CommandScheduler::get_instance().schedule(&command);
        CommandScheduler::get_instance().run();
        assert!(state.get_as_boolean());
        clock.advance(0.5);
        CommandScheduler::get_instance().run();
        assert!(state.get_as_boolean());
        clock.advance(0.6);
        CommandScheduler::get_instance().run();
        assert!(!state.get_as_boolean());
    }

    #[test]
    fn set_false_for_time_holds_false_then_restores_true() {
        let clock = FakeClock::install();
        let state = SpectrumState::without_alert("timed-false");
        state.set_state(true);
        let command = state.set_false_for_time(|| 1.0);
        CommandScheduler::get_instance().schedule(&command);
        CommandScheduler::get_instance().run();
        assert!(!state.get_as_boolean());
        clock.advance(1.1);
        CommandScheduler::get_instance().run();
        assert!(state.get_as_boolean());
    }

    #[test]
    fn set_true_while_running_drops_the_state_when_cancelled() {
        let state = SpectrumState::without_alert("while-running");
        let command = state.set_true_while_running();
        CommandScheduler::get_instance().schedule(&command);
        CommandScheduler::get_instance().run();
        assert!(state.get_as_boolean());
        command.cancel();
        assert!(!state.get_as_boolean());
    }

    #[test]
    fn cancel_trigger_ends_the_timed_hold_early() {
        let clock = FakeClock::install();
        let state = SpectrumState::without_alert("cancelable");
        let cancel = Rc::new(Cell::new(false));
        let cancel_flag = cancel.clone();
        let cancel_trigger = Trigger::new(move || cancel_flag.get());
        let command = state.set_true_for_time_with_cancel(|| 10.0, &cancel_trigger);
        CommandScheduler::get_instance().schedule(&command);
        CommandScheduler::get_instance().run();
        assert!(state.get_as_boolean());
        clock.advance(1.0);
        cancel.set(true);
        CommandScheduler::get_instance().run();
        assert!(!state.get_as_boolean());
    }

    #[test]
    fn the_state_works_as_a_condition_for_other_triggers() {
        let state = SpectrumState::without_alert("as-condition");
        let both = Trigger::new(|| true).and(&state);
        assert!(!both.get_as_boolean());
        state.set_state(true);
        assert!(both.get_as_boolean());
    }
}

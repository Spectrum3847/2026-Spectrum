use crate::common::*;
use commands::testing::{FakeRobotState, Mode};
use commands::{Command, Commands, EventLoop, RobotModeTriggers, Trigger};
use std::cell::Cell;
use std::rc::Rc;

fn switch() -> (Rc<Cell<bool>>, Trigger) {
    let value = Rc::new(Cell::new(false));
    let v = value.clone();
    (value, Trigger::new(move || v.get()))
}

fn wait_for(flag: &Rc<Cell<bool>>) -> Command {
    let f = flag.clone();
    Commands::wait_until(move || f.get())
}

#[test]
fn on_true_schedules_on_rising_edge() {
    let (pressed, trigger) = switch();
    let finished = Rc::new(Cell::new(false));
    let cmd = wait_for(&finished);
    trigger.on_true(cmd.clone());
    run_n(1);
    assert!(!cmd.is_scheduled());
    pressed.set(true);
    run_n(1);
    assert!(cmd.is_scheduled());
    finished.set(true);
    run_n(1);
    assert!(!cmd.is_scheduled());
}

#[test]
fn on_true_ignores_level_and_falling_edge() {
    let (pressed, trigger) = switch();
    let (cmd, c) = mock(true, &[]);
    trigger.on_true(cmd);
    pressed.set(true);
    run_n(1);
    run_n(3);
    pressed.set(false);
    run_n(1);
    assert_eq!(c.init.get(), 1);
}

#[test]
fn on_false_schedules_on_falling_edge() {
    let (pressed, trigger) = switch();
    pressed.set(true);
    let (cmd, c) = mock(true, &[]);
    trigger.on_false(cmd);
    run_n(1);
    assert_eq!(c.init.get(), 0);
    pressed.set(false);
    run_n(1);
    assert_eq!(c.init.get(), 1);
}

#[test]
fn binding_captures_state_at_bind_time() {
    let (pressed, trigger) = switch();
    pressed.set(true);
    let (cmd, c) = mock(true, &[]);
    trigger.on_true(cmd);
    // Already true when bound: no edge on the first poll.
    run_n(1);
    assert_eq!(c.init.get(), 0);
    pressed.set(false);
    run_n(1);
    pressed.set(true);
    run_n(1);
    assert_eq!(c.init.get(), 1);
}

#[test]
fn each_binding_has_its_own_previous_value() {
    let (pressed, trigger) = switch();
    let (first, c1) = mock(true, &[]);
    trigger.on_true(first);
    pressed.set(true);
    run_n(1);
    // Bound while already true: sees no edge.
    let (second, c2) = mock(true, &[]);
    trigger.on_true(second);
    run_n(1);
    assert_eq!((c1.init.get(), c2.init.get()), (1, 0));
}

#[test]
fn state_is_sampled_only_when_polled() {
    let (pressed, trigger) = switch();
    let (cmd, c) = mock(true, &[]);
    trigger.on_true(cmd);
    // A pulse between polls is missed.
    pressed.set(true);
    pressed.set(false);
    run_n(1);
    assert_eq!(c.init.get(), 0);
}

#[test]
fn while_true_runs_until_released() {
    let (pressed, trigger) = switch();
    let (cmd, c) = mock(true, &[]);
    trigger.while_true(cmd);
    run_n(1);
    assert_eq!(c.init.get(), 0);
    pressed.set(true);
    run_n(1);
    run_n(1);
    assert_eq!((c.init.get(), c.exec.get()), (1, 2));
    pressed.set(false);
    run_n(1);
    assert_eq!((c.end_true.get(), c.exec.get()), (1, 2));
}

#[test]
fn while_true_start_end_command() {
    let (pressed, trigger) = switch();
    let (starts, ends) = (counter(), counter());
    trigger.while_true(Commands::start_end(bump(&starts), bump(&ends), &[]));
    pressed.set(true);
    run_n(2);
    assert_eq!((starts.get(), ends.get()), (1, 0));
    pressed.set(false);
    run_n(1);
    assert_eq!((starts.get(), ends.get()), (1, 1));
}

#[test]
fn and_or_negate() {
    let (a, ta) = switch();
    let (b, tb) = switch();
    let and = ta.and(&tb);
    let or = ta.or(&tb);
    let neg = ta.negate();
    assert!(!and.get_as_boolean() && !or.get_as_boolean() && neg.get_as_boolean());
    a.set(true);
    assert!(!and.get_as_boolean() && or.get_as_boolean() && !neg.get_as_boolean());
    b.set(true);
    assert!(and.get_as_boolean() && or.get_as_boolean());
    a.set(false);
    assert!(!and.get_as_boolean() && or.get_as_boolean());
}

#[test]
fn and_accepts_closures_and_composed_triggers_bind() {
    let (a, ta) = switch();
    let b = Rc::new(Cell::new(false));
    let b2 = b.clone();
    let both = ta.and(move || b2.get());
    let (cmd, c) = mock(true, &[]);
    both.on_true(cmd);
    run_n(1);
    a.set(true);
    run_n(1);
    assert_eq!(c.init.get(), 0);
    b.set(true);
    run_n(1);
    assert_eq!(c.init.get(), 1);
}

#[test]
fn negate_edges_match_the_inverse() {
    let (pressed, trigger) = switch();
    pressed.set(true);
    let (cmd, c) = mock(true, &[]);
    trigger.negate().on_true(cmd);
    run_n(1);
    assert_eq!(c.init.get(), 0);
    pressed.set(false);
    run_n(1);
    assert_eq!(c.init.get(), 1);
}

#[test]
fn chained_bindings_on_one_trigger() {
    let (pressed, trigger) = switch();
    let (down, cd) = mock(true, &[]);
    let (up, cu) = mock(true, &[]);
    trigger.on_true(down).on_false(up);
    pressed.set(true);
    run_n(1);
    pressed.set(false);
    run_n(1);
    assert_eq!((cd.init.get(), cu.init.get()), (1, 1));
}

#[test]
fn one_command_can_be_bound_and_triggered_repeatedly() {
    let (pressed, trigger) = switch();
    let (cmd, c) = mock(true, &[]);
    c.finished.set(true);
    trigger.on_true(cmd);
    for _ in 0..3 {
        pressed.set(true);
        run_n(2);
        pressed.set(false);
        run_n(1);
    }
    assert_eq!((c.init.get(), c.end_false.get()), (3, 3));
}

#[test]
fn trigger_with_custom_loop_is_polled_only_by_that_loop() {
    let event_loop = Rc::new(EventLoop::new());
    let value = Rc::new(Cell::new(false));
    let v = value.clone();
    let trigger = Trigger::with_loop(event_loop.clone(), move || v.get());
    let (cmd, c) = mock(true, &[]);
    trigger.on_true(cmd);
    value.set(true);
    run_n(1);
    assert_eq!(c.init.get(), 0);
    event_loop.poll();
    assert_eq!(c.init.get(), 1);
}

#[test]
fn set_active_button_loop_switches_polling() {
    let other = Rc::new(EventLoop::new());
    let (pressed, trigger) = switch();
    let (cmd, c) = mock(true, &[]);
    trigger.on_true(cmd);
    scheduler().set_active_button_loop(other);
    pressed.set(true);
    run_n(1);
    assert_eq!(c.init.get(), 0);
    scheduler().set_active_button_loop(scheduler().get_default_button_loop());
    run_n(1);
    assert_eq!(c.init.get(), 1);
}

#[test]
fn robot_mode_triggers_follow_the_mode() {
    let mode = FakeRobotState::install();
    let (auto, teleop, disabled, test) = (
        RobotModeTriggers::autonomous(),
        RobotModeTriggers::teleop(),
        RobotModeTriggers::disabled(),
        RobotModeTriggers::test(),
    );
    let read = || {
        (
            auto.get_as_boolean(),
            teleop.get_as_boolean(),
            disabled.get_as_boolean(),
            test.get_as_boolean(),
        )
    };
    assert_eq!(read(), (false, true, false, false));
    mode.set_mode(Mode::Autonomous);
    assert_eq!(read(), (true, false, false, false));
    mode.set_mode(Mode::Disabled);
    assert_eq!(read(), (false, false, true, false));
    mode.set_mode(Mode::Test);
    assert_eq!(read(), (false, false, false, true));
}

#[test]
fn robot_mode_trigger_binding_fires_on_mode_change() {
    let mode = FakeRobotState::install();
    mode.set_mode(Mode::Disabled);
    let (cmd, c) = mock(true, &[]);
    RobotModeTriggers::autonomous().on_true(cmd);
    run_n(1);
    mode.set_mode(Mode::Autonomous);
    run_n(1);
    assert_eq!(c.init.get(), 1);
}

#[test]
fn disabled_trigger_and_button_runs_ignoring_disable_command() {
    let mode = FakeRobotState::install();
    mode.set_mode(Mode::Disabled);
    let (pressed, button) = switch();
    let hits = counter();
    let cmd = Commands::run_once(bump(&hits), &[]).ignoring_disable(true);
    RobotModeTriggers::disabled().and(&button).on_true(cmd);
    run_n(1);
    pressed.set(true);
    run_n(1);
    assert_eq!(hits.get(), 1);
}

#[test]
fn trigger_clone_shares_condition() {
    let (value, trigger) = switch();
    let copy = trigger.clone();
    value.set(true);
    assert!(copy.get_as_boolean());
}

use crate::common::*;
use commands::testing::{FakeClock, FakeRobotState, Mode};
use commands::{Commands, InterruptionBehavior, SubsystemId};
use std::cell::{Cell, RefCell};
use std::rc::Rc;

#[test]
fn with_timeout_ends_after_the_duration() {
    let clock = FakeClock::install();
    let (inner, c) = mock(true, &[]);
    let timeout = inner.with_timeout(0.1);
    scheduler().schedule(&timeout);
    run_n(1);
    assert!(timeout.is_scheduled());
    clock.advance(0.15);
    run_n(1);
    assert!(!timeout.is_scheduled());
    assert_eq!(c.end_true.get(), 1);
}

#[test]
fn until_ends_when_condition_becomes_true() {
    let done = Rc::new(Cell::new(false));
    let d = done.clone();
    let cmd = Commands::idle(&[]).until(move || d.get());
    scheduler().schedule(&cmd);
    run_n(1);
    assert!(cmd.is_scheduled());
    done.set(true);
    run_n(1);
    assert!(!cmd.is_scheduled());
}

#[test]
fn until_polls_after_the_command_ran() {
    let ran = Rc::new(Cell::new(false));
    let polled = Rc::new(Cell::new(false));
    let (r, p) = (ran.clone(), polled.clone());
    let first = Commands::functional(
        || {},
        move || r.set(true),
        |_| {},
        move || {
            p.set(true);
            true
        },
        &[],
    );
    let (r, p) = (ran.clone(), polled.clone());
    let cmd = first.until(move || {
        assert!(r.get() && p.get());
        true
    });
    scheduler().schedule(&cmd);
    run_n(1);
    assert!(ran.get() && polled.get());
}

#[test]
fn only_while_ends_when_condition_becomes_false() {
    let on = Rc::new(Cell::new(true));
    let o = on.clone();
    let cmd = Commands::idle(&[]).only_while(move || o.get());
    scheduler().schedule(&cmd);
    run_n(1);
    assert!(cmd.is_scheduled());
    on.set(false);
    run_n(1);
    assert!(!cmd.is_scheduled());
}

#[test]
fn ignoring_disable_overrides_the_flag() {
    let mode = FakeRobotState::install();
    let runs = Commands::idle(&[]).ignoring_disable(true);
    let stops = Commands::idle(&[])
        .ignoring_disable(true)
        .ignoring_disable(false);
    assert!(runs.runs_when_disabled() && !stops.runs_when_disabled());
    scheduler().schedule(&runs);
    scheduler().schedule(&stops);
    mode.set_mode(Mode::Disabled);
    run_n(1);
    assert!(runs.is_scheduled());
    assert!(!stops.is_scheduled());
}

#[test]
fn before_starting_and_and_then_order() {
    let log = Rc::new(RefCell::new(String::new()));
    let step = |tag: &'static str| {
        let l = log.clone();
        Commands::run_once(move || l.borrow_mut().push_str(tag), &[])
    };
    let cmd = step("b")
        .before_starting(step("a"))
        .and_then([step("c"), step("d")]);
    let l = log.clone();
    let cmd = cmd.and_then_run(move || l.borrow_mut().push('e'));
    scheduler().schedule(&cmd);
    run_n(6);
    assert_eq!(*log.borrow(), "abcde");
    assert!(!cmd.is_scheduled());
}

#[test]
fn along_with_runs_in_parallel_and_finishes_with_last() {
    let (a, ca) = mock(true, &[]);
    let (b, cb) = mock(true, &[]);
    let cmd = a.along_with(b);
    scheduler().schedule(&cmd);
    ca.finished.set(true);
    run_n(1);
    assert!(cmd.is_scheduled());
    cb.finished.set(true);
    run_n(1);
    assert!(!cmd.is_scheduled());
}

#[test]
fn race_with_ends_with_first() {
    let (a, ca) = mock(true, &[]);
    let (b, cb) = mock(true, &[]);
    let cmd = a.race_with(b);
    scheduler().schedule(&cmd);
    cb.finished.set(true);
    run_n(1);
    assert!(!cmd.is_scheduled());
    assert_eq!((ca.end_true.get(), cb.end_false.get()), (1, 1));
}

#[test]
fn deadline_for_and_with_deadline() {
    let (main, cm) = mock(true, &[]);
    let (side, cs) = mock(true, &[]);
    let cmd = main.deadline_for(side);
    scheduler().schedule(&cmd);
    cm.finished.set(true);
    run_n(1);
    assert!(!cmd.is_scheduled());
    assert_eq!(cs.end_true.get(), 1);

    let (main, cm) = mock(true, &[]);
    let (limit, cl) = mock(true, &[]);
    let cmd = main.with_deadline(limit);
    scheduler().schedule(&cmd);
    cl.finished.set(true);
    run_n(1);
    assert!(!cmd.is_scheduled());
    assert_eq!(cm.end_true.get(), 1);
}

#[test]
fn unless_and_only_if() {
    let flag = Rc::new(Cell::new(true));
    let f = flag.clone();
    let (a, ca) = mock(true, &[]);
    let unless = a.unless(move || f.get());
    scheduler().schedule(&unless);
    run_n(1);
    assert_eq!(ca.init.get(), 0);
    assert!(!unless.is_scheduled());

    let f = flag.clone();
    let (b, cb) = mock(true, &[]);
    let only_if = b.only_if(move || f.get());
    scheduler().schedule(&only_if);
    assert_eq!(cb.init.get(), 1);
}

#[test]
fn unless_keeps_requirements() {
    let sub = SubsystemId::new();
    let cmd = Commands::idle(&[sub]).only_if(|| true);
    assert!(cmd.has_requirement(sub));
}

#[test]
fn finally_do_runs_after_inner_end() {
    let flags = Rc::new(RefCell::new(Vec::new()));
    let f = flags.clone();
    let cmd = Commands::run_once(|| {}, &[])
        .finally_do_with(move |interrupted| f.borrow_mut().push(interrupted));
    scheduler().schedule(&cmd);
    run_n(1);
    let f = flags.clone();
    let cmd =
        Commands::idle(&[]).finally_do_with(move |interrupted| f.borrow_mut().push(interrupted));
    scheduler().schedule(&cmd);
    scheduler().cancel(&cmd);
    assert_eq!(*flags.borrow(), [false, true]);

    let ends = counter();
    let cmd = Commands::idle(&[]).finally_do(bump(&ends));
    scheduler().schedule(&cmd);
    scheduler().cancel(&cmd);
    assert_eq!(ends.get(), 1);
}

#[test]
fn with_name_renames_and_keeps_requirements() {
    let sub = SubsystemId::new();
    let inner = Commands::idle(&[sub]);
    assert_eq!(inner.get_name(), "RunCommand");
    let named = inner.with_name("Custom");
    assert_eq!(named.get_name(), "Custom");
    assert!(named.has_requirement(sub));
    assert_eq!(named.ignoring_disable(true).get_name(), "Custom");
}

#[test]
fn with_interrupt_behavior_overrides() {
    let sub = SubsystemId::new();
    let cmd = Commands::idle(&[sub]).with_interrupt_behavior(InterruptionBehavior::CancelIncoming);
    assert_eq!(
        cmd.get_interruption_behavior(),
        InterruptionBehavior::CancelIncoming
    );
}

#[test]
#[should_panic(expected = "composed may not")]
fn decorated_command_cannot_be_scheduled_alone() {
    let inner = Commands::idle(&[]);
    let _named = inner.clone().with_name("x");
    scheduler().schedule(&inner);
}

#[test]
fn reusing_a_decorated_command_via_factory_is_fine() {
    let make = || Commands::idle(&[]).with_timeout(1.0).with_name("a");
    scheduler().schedule(&make());
    scheduler().schedule(&make());
}

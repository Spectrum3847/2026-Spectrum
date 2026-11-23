use crate::common::*;
use commands::testing::FakeClock;
use commands::{Commands, InterruptionBehavior, SubsystemId};
use std::cell::{Cell, RefCell};
use std::rc::Rc;

#[test]
fn sequential_runs_members_in_order() {
    let (a, ca) = mock(true, &[]);
    let (b, cb) = mock(true, &[]);
    let (c, cc) = mock(true, &[]);
    let group = Commands::sequence([a, b, c]);
    scheduler().schedule(&group);
    assert_eq!((ca.init.get(), cb.init.get(), cc.init.get()), (1, 0, 0));

    ca.finished.set(true);
    run_n(1);
    assert_eq!(
        (ca.end_false.get(), cb.init.get(), cb.exec.get()),
        (1, 1, 0)
    );

    cb.finished.set(true);
    run_n(1);
    assert_eq!(
        (cb.exec.get(), cb.end_false.get(), cc.init.get()),
        (1, 1, 1)
    );

    cc.finished.set(true);
    run_n(1);
    assert!(!group.is_scheduled());
    assert_eq!(cc.end_false.get(), 1);
    assert_eq!(ca.exec.get(), 1);
}

#[test]
fn sequential_interrupt_ends_only_current() {
    let (a, ca) = mock(true, &[]);
    let (b, cb) = mock(true, &[]);
    let group = Commands::sequence([a, b]);
    scheduler().schedule(&group);
    ca.finished.set(true);
    run_n(2);
    scheduler().cancel(&group);
    assert_eq!((ca.end_true.get(), ca.end_false.get()), (0, 1));
    assert_eq!((cb.end_true.get(), cb.init.get()), (1, 1));
}

#[test]
fn empty_sequence_finishes_immediately() {
    let group = Commands::sequence(Vec::new());
    scheduler().schedule(&group);
    run_n(1);
    assert!(!group.is_scheduled());
    assert!(group.runs_when_disabled());
    assert_eq!(
        group.get_interruption_behavior(),
        InterruptionBehavior::CancelIncoming
    );
}

#[test]
fn sequential_unions_requirements_and_scheduling_twice_restarts() {
    let (s1, s2) = (SubsystemId::new(), SubsystemId::new());
    let (a, ca) = mock(true, &[s1]);
    let (b, _) = mock(true, &[s2]);
    let group = Commands::sequence([a, b]);
    assert!(group.has_requirement(s1) && group.has_requirement(s2));
    scheduler().schedule(&group);
    scheduler().cancel(&group);
    scheduler().schedule(&group);
    assert_eq!(ca.init.get(), 2);
}

#[test]
fn parallel_runs_all_and_finishes_when_all_finish() {
    let (a, ca) = mock(true, &[]);
    let (b, cb) = mock(true, &[]);
    let group = Commands::parallel([a, b]);
    scheduler().schedule(&group);
    assert_eq!((ca.init.get(), cb.init.get()), (1, 1));
    ca.finished.set(true);
    run_n(1);
    assert!(group.is_scheduled());
    run_n(1);
    assert_eq!((ca.exec.get(), cb.exec.get()), (1, 2));
    cb.finished.set(true);
    run_n(1);
    assert!(!group.is_scheduled());
    assert_eq!((ca.end_false.get(), cb.end_false.get()), (1, 1));
}

#[test]
fn parallel_interrupt_ends_only_running_members() {
    let (a, ca) = mock(true, &[]);
    let (b, cb) = mock(true, &[]);
    let group = Commands::parallel([a, b]);
    scheduler().schedule(&group);
    ca.finished.set(true);
    run_n(1);
    scheduler().cancel(&group);
    assert_eq!((ca.end_false.get(), ca.end_true.get()), (1, 0));
    assert_eq!(cb.end_true.get(), 1);
}

#[test]
#[should_panic(expected = "cannot require the same subsystems")]
fn parallel_rejects_shared_requirements() {
    let sub = SubsystemId::new();
    let _ = Commands::parallel([mock(true, &[sub]).0, mock(true, &[sub]).0]);
}

#[test]
#[should_panic(expected = "cannot require the same subsystems")]
fn race_rejects_shared_requirements() {
    let sub = SubsystemId::new();
    let _ = Commands::race([mock(true, &[sub]).0, mock(true, &[sub]).0]);
}

#[test]
#[should_panic(expected = "cannot require the same subsystems")]
fn deadline_rejects_shared_requirements() {
    let sub = SubsystemId::new();
    let _ = Commands::deadline(mock(true, &[sub]).0, mock(true, &[sub]).0);
}

#[test]
fn group_interruption_behavior_is_cancel_self_if_any_member_is() {
    let incoming =
        Commands::idle(&[]).with_interrupt_behavior(InterruptionBehavior::CancelIncoming);
    let incoming2 =
        Commands::idle(&[]).with_interrupt_behavior(InterruptionBehavior::CancelIncoming);
    let all_incoming = Commands::sequence([incoming, incoming2]);
    assert_eq!(
        all_incoming.get_interruption_behavior(),
        InterruptionBehavior::CancelIncoming
    );
    let mixed = Commands::parallel([all_incoming, Commands::idle(&[])]);
    assert_eq!(
        mixed.get_interruption_behavior(),
        InterruptionBehavior::CancelSelf
    );
}

#[test]
fn race_ends_when_first_finishes_and_interrupts_the_rest() {
    let (a, ca) = mock(true, &[]);
    let (b, cb) = mock(true, &[]);
    let group = Commands::race([a, b]);
    scheduler().schedule(&group);
    run_n(1);
    assert!(group.is_scheduled());
    ca.finished.set(true);
    run_n(1);
    assert!(!group.is_scheduled());
    assert_eq!((ca.end_false.get(), ca.end_true.get()), (1, 0));
    assert_eq!((cb.end_false.get(), cb.end_true.get()), (0, 1));
}

#[test]
fn race_interrupted_from_outside_interrupts_all() {
    let (a, ca) = mock(true, &[]);
    let (b, cb) = mock(true, &[]);
    let group = Commands::race([a, b]);
    scheduler().schedule(&group);
    run_n(1);
    scheduler().cancel(&group);
    assert_eq!((ca.end_true.get(), cb.end_true.get()), (1, 1));
}

#[test]
fn race_nested_sequence_ends_once() {
    let (s1, s2) = (SubsystemId::new(), SubsystemId::new());
    let (a, ca) = mock(true, &[s1]);
    let (b, cb) = mock(true, &[s2]);
    let (c, _) = mock(true, &[]);
    let race = Commands::race([Commands::sequence([a, b]), c]);
    scheduler().schedule(&race);
    run_n(1);
    ca.finished.set(true);
    run_n(1);
    cb.finished.set(true);
    run_n(1);
    assert!(!race.is_scheduled());
    assert_eq!(cb.end_false.get(), 1);
    assert_eq!(cb.end_true.get(), 0);
}

#[test]
fn race_can_be_scheduled_twice() {
    let (a, ca) = mock(true, &[]);
    let (b, cb) = mock(true, &[]);
    let group = Commands::race([a, b]);
    scheduler().schedule(&group);
    ca.finished.set(true);
    run_n(1);
    assert!(!group.is_scheduled());
    ca.finished.set(false);
    scheduler().schedule(&group);
    assert_eq!((ca.init.get(), cb.init.get()), (2, 2));
    run_n(2);
    assert!(group.is_scheduled());
    cb.finished.set(true);
    run_n(1);
    assert!(!group.is_scheduled());
}

#[test]
fn deadline_ends_when_deadline_finishes() {
    let (deadline, cd) = mock(true, &[]);
    let (other, co) = mock(true, &[]);
    let (done, cx) = mock(true, &[]);
    cx.finished.set(true);
    let group = Commands::deadline(deadline, [other, done]);
    scheduler().schedule(&group);
    run_n(1);
    assert!(group.is_scheduled());
    assert_eq!(cx.end_false.get(), 1);
    cd.finished.set(true);
    run_n(1);
    assert!(!group.is_scheduled());
    assert_eq!(cd.end_false.get(), 1);
    assert_eq!(co.end_true.get(), 1);
    assert_eq!(cx.end_true.get(), 0);
}

#[test]
fn deadline_interrupt_ends_running_members() {
    let (deadline, cd) = mock(true, &[]);
    let (other, co) = mock(true, &[]);
    let group = Commands::deadline(deadline, other);
    scheduler().schedule(&group);
    run_n(1);
    scheduler().cancel(&group);
    assert_eq!((cd.end_true.get(), co.end_true.get()), (1, 1));
}

#[test]
#[should_panic(expected = "twice in the same composition")]
fn deadline_cannot_also_be_an_other_command() {
    let (d, _) = mock(true, &[]);
    let _ = Commands::deadline(d.clone(), d);
}

#[test]
fn either_picks_branch_at_initialize() {
    let flag = Rc::new(Cell::new(true));
    let (on_true, ct) = mock(true, &[]);
    let (on_false, cf) = mock(true, &[]);
    let f = flag.clone();
    let cond = Commands::either(on_true, on_false, move || f.get());
    scheduler().schedule(&cond);
    flag.set(false);
    ct.finished.set(true);
    run_n(1);
    assert!(!cond.is_scheduled());
    assert_eq!((ct.init.get(), cf.init.get()), (1, 0));
    assert_eq!(ct.end_false.get(), 1);

    scheduler().schedule(&cond);
    run_n(1);
    assert_eq!((cf.init.get(), cf.exec.get()), (1, 1));
    scheduler().cancel(&cond);
    assert_eq!(cf.end_true.get(), 1);
}

#[test]
fn either_requirements_flags_and_interruption() {
    let (s1, s2) = (SubsystemId::new(), SubsystemId::new());
    let incoming =
        Commands::idle(&[s1]).with_interrupt_behavior(InterruptionBehavior::CancelIncoming);
    let cond = Commands::either(incoming, mock(false, &[s2]).0, || true);
    assert!(cond.has_requirement(s1) && cond.has_requirement(s2));
    assert!(!cond.runs_when_disabled());
    assert_eq!(
        cond.get_interruption_behavior(),
        InterruptionBehavior::CancelSelf
    );

    let both = Commands::either(
        Commands::none().ignoring_disable(true),
        Commands::print("x"),
        || true,
    );
    assert!(both.runs_when_disabled());
}

#[test]
fn deferred_calls_supplier_only_at_initialize() {
    let calls = counter();
    let c = calls.clone();
    let deferred = Commands::defer(
        move || {
            c.set(c.get() + 1);
            Commands::none()
        },
        &[],
    );
    assert_eq!(calls.get(), 0);
    scheduler().schedule(&deferred);
    assert_eq!(calls.get(), 1);
    run_n(1);
    assert!(!deferred.is_scheduled());
    scheduler().schedule(&deferred);
    assert_eq!(calls.get(), 2);
}

#[test]
fn deferred_forwards_callbacks_and_requirements() {
    let sub = SubsystemId::new();
    let slot = Rc::new(RefCell::new(None));
    let s = slot.clone();
    let deferred = Commands::defer(
        move || {
            let (cmd, counts) = mock(true, &[]);
            *s.borrow_mut() = Some(counts);
            cmd
        },
        &[sub],
    );
    assert!(deferred.has_requirement(sub));
    scheduler().schedule(&deferred);
    let counts = slot.borrow().clone().unwrap();
    run_n(1);
    assert_eq!((counts.init.get(), counts.exec.get()), (1, 1));
    counts.finished.set(true);
    run_n(1);
    assert_eq!(counts.end_false.get(), 1);
    assert!(!deferred.is_scheduled());
}

#[test]
fn deferred_interrupt_ends_inner_interrupted() {
    let slot = Rc::new(RefCell::new(None));
    let s = slot.clone();
    let deferred = Commands::defer(
        move || {
            let (cmd, counts) = mock(true, &[]);
            *s.borrow_mut() = Some(counts);
            cmd
        },
        &[],
    );
    scheduler().schedule(&deferred);
    scheduler().cancel(&deferred);
    assert_eq!(slot.borrow().clone().unwrap().end_true.get(), 1);
}

#[test]
fn wait_seconds_uses_the_clock_and_runs_when_disabled() {
    let clock = FakeClock::install();
    let wait = Commands::wait_seconds(1.0);
    assert!(wait.runs_when_disabled());
    assert_eq!(wait.get_name(), "WaitCommand: 1.0 seconds");
    scheduler().schedule(&wait);
    clock.advance(0.5);
    run_n(1);
    assert!(wait.is_scheduled());
    clock.advance(0.5);
    run_n(1);
    assert!(!wait.is_scheduled());
}

#[test]
fn wait_restarts_on_reschedule() {
    let clock = FakeClock::install();
    let wait = Commands::wait_seconds(1.0);
    scheduler().schedule(&wait);
    clock.advance(0.9);
    scheduler().cancel(&wait);
    clock.advance(5.0);
    scheduler().schedule(&wait);
    clock.advance(0.9);
    run_n(1);
    assert!(wait.is_scheduled());
    clock.advance(0.2);
    run_n(1);
    assert!(!wait.is_scheduled());
}

#[test]
fn wait_until_runs_when_disabled_and_finishes_on_condition() {
    let flag = Rc::new(Cell::new(false));
    let f = flag.clone();
    let wait = Commands::wait_until(move || f.get());
    assert!(wait.runs_when_disabled());
    scheduler().schedule(&wait);
    run_n(1);
    assert!(wait.is_scheduled());
    flag.set(true);
    run_n(1);
    assert!(!wait.is_scheduled());
}

#[test]
fn factory_names_match_wpilib() {
    assert_eq!(Commands::none().get_name(), "InstantCommand");
    assert_eq!(Commands::idle(&[]).get_name(), "RunCommand");
    assert_eq!(
        Commands::start_end(|| {}, || {}, &[]).get_name(),
        "StartEndCommand"
    );
    assert_eq!(Commands::print("hi").get_name(), "PrintCommand");
    assert!(Commands::print("hi").runs_when_disabled());
    assert_eq!(Commands::sequence([]).get_name(), "SequentialCommandGroup");
    assert_eq!(Commands::parallel([]).get_name(), "ParallelCommandGroup");
    assert_eq!(Commands::race([]).get_name(), "ParallelRaceGroup");
    assert_eq!(
        Commands::defer(Commands::none, &[]).get_name(),
        "DeferredCommand"
    );
}

#[test]
fn start_end_and_run_end_callbacks() {
    let (starts, ends, runs) = (counter(), counter(), counter());
    let cmd = Commands::start_end(bump(&starts), bump(&ends), &[]);
    scheduler().schedule(&cmd);
    run_n(2);
    assert_eq!((starts.get(), ends.get()), (1, 0));
    scheduler().cancel(&cmd);
    assert_eq!(ends.get(), 1);

    let cmd = Commands::run_end(bump(&runs), bump(&ends), &[]);
    scheduler().schedule(&cmd);
    run_n(3);
    scheduler().cancel(&cmd);
    assert_eq!((runs.get(), ends.get()), (3, 2));
}

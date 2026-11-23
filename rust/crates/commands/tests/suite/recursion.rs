//! Commands that schedule or cancel from inside callbacks must not panic or
//! corrupt scheduler state (WPILib SchedulingRecursionTest and the run-loop queue).

use crate::common::*;
use commands::testing::FakeRobotState;
use commands::{
    Command, CommandBody, Commands, EventLoop, InterruptionBehavior, RobotModeTriggers, Subsystem,
    SubsystemId, Trigger,
};
use std::cell::{Cell, RefCell};
use std::rc::Rc;

const BEHAVIORS: [InterruptionBehavior; 2] = [
    InterruptionBehavior::CancelSelf,
    InterruptionBehavior::CancelIncoming,
];

/// Slot a command can read its own handle from.
type SelfSlot = Rc<RefCell<Option<Command>>>;

struct SelfCanceller {
    slot: SelfSlot,
    ends: Rc<Cell<u32>>,
}

impl CommandBody for SelfCanceller {
    fn initialize(&mut self) {
        let me = self.slot.borrow().clone().unwrap();
        scheduler().cancel(&me);
    }
    fn end(&mut self, _interrupted: bool) {
        self.ends.set(self.ends.get() + 1);
    }
}

fn self_canceller(sub: SubsystemId, behavior: InterruptionBehavior) -> (Command, Rc<Cell<u32>>) {
    let slot: SelfSlot = Rc::default();
    let ends = counter();
    let cmd = Command::builder(
        "SelfCancels",
        SelfCanceller {
            slot: slot.clone(),
            ends: ends.clone(),
        },
    )
    .requires(&[sub])
    .interruption_behavior(behavior)
    .build();
    *slot.borrow_mut() = Some(cmd.clone());
    (cmd, ends)
}

#[test]
fn cancel_from_initialize() {
    for behavior in BEHAVIORS {
        let sub = SubsystemId::new();
        let ran = counter();
        let (self_cancels, ends) = self_canceller(sub, behavior);
        let other = Commands::run(bump(&ran), &[sub]);
        scheduler().schedule(&self_cancels);
        run_n(1);
        scheduler().schedule(&other);
        assert!(!self_cancels.is_scheduled());
        assert!(other.is_scheduled());
        assert_eq!(ends.get(), 1);
        run_n(1);
        assert_eq!(ran.get(), 1);
    }
}

#[test]
fn default_command_rescheduled_after_self_cancel() {
    for behavior in BEHAVIORS {
        let sub = SubsystemId::new();
        let ran = counter();
        let (self_cancels, ends) = self_canceller(sub, behavior);
        let other = Commands::run(bump(&ran), &[sub]);
        scheduler().set_default_command(sub, other.clone());
        scheduler().schedule(&self_cancels);
        run_n(2);
        assert!(!self_cancels.is_scheduled());
        assert!(other.is_scheduled());
        assert_eq!(ends.get(), 1);
        run_n(1);
        assert!(ran.get() > 0);
    }
}

#[test]
fn cancel_from_end() {
    let slot: SelfSlot = Rc::default();
    let ends = counter();
    let (s, e) = (slot.clone(), ends.clone());
    let cmd = Commands::functional(
        || {},
        || {},
        move |_| {
            e.set(e.get() + 1);
            let me = s.borrow().clone().unwrap();
            scheduler().cancel(&me);
        },
        || false,
        &[],
    );
    *slot.borrow_mut() = Some(cmd.clone());
    scheduler().schedule(&cmd);
    scheduler().cancel(&cmd);
    assert_eq!(ends.get(), 1);
    assert!(!cmd.is_scheduled());
}

/// A -> B -> C -> D, each ending cancels the next; D cancels everything.
fn cancel_chain(count: &Rc<Cell<u32>>) -> Vec<Command> {
    let slots: Vec<SelfSlot> = (0..4).map(|_| Rc::default()).collect();
    let cmds: Vec<Command> = (0..4)
        .map(|i| {
            let (count, slots) = (count.clone(), slots.clone());
            Commands::functional(
                || {},
                || {},
                move |_| {
                    count.set(count.get() + 1);
                    if i == 3 {
                        scheduler().cancel_all();
                    } else {
                        let next = slots[i + 1].borrow().clone().unwrap();
                        scheduler().cancel(&next);
                    }
                },
                || true,
                &[],
            )
        })
        .collect();
    for (slot, cmd) in slots.iter().zip(&cmds) {
        *slot.borrow_mut() = Some(cmd.clone());
    }
    cmds
}

#[test]
fn cancel_from_end_loop() {
    let count = counter();
    let cmds = cancel_chain(&count);
    for c in &cmds {
        scheduler().schedule(c);
    }
    scheduler().cancel(&cmds[0]);
    assert_eq!(count.get(), 4);
    assert!(cmds.iter().all(|c| !c.is_scheduled()));
}

#[test]
fn cancel_from_end_loop_while_in_run_loop() {
    let count = counter();
    let cmds = cancel_chain(&count);
    for c in &cmds {
        scheduler().schedule(c);
    }
    run_n(1);
    assert_eq!(count.get(), 4);
    assert!(cmds.iter().all(|c| !c.is_scheduled()));
}

#[test]
fn schedule_from_end_of_interrupted_command() {
    let sub = SubsystemId::new();
    let count = counter();
    let other = Commands::run_once(|| {}, &[sub]);
    let (c, o) = (count.clone(), other.clone());
    let victim = Commands::functional(
        || {},
        || {},
        move |_| {
            c.set(c.get() + 1);
            scheduler().schedule(&o);
        },
        || false,
        &[sub],
    );
    scheduler().schedule(&victim);
    scheduler().schedule(&other);
    assert_eq!(count.get(), 1);
    assert!(!victim.is_scheduled());
    assert!(other.is_scheduled());
}

#[test]
fn schedule_initialize_from_default_command() {
    for behavior in BEHAVIORS {
        let sub = SubsystemId::new();
        let count = counter();
        let other = Commands::run_once(|| {}, &[sub]).with_interrupt_behavior(behavior);
        let (c, o) = (count.clone(), other.clone());
        let default = Commands::functional(
            move || {
                c.set(c.get() + 1);
                scheduler().schedule(&o);
            },
            || {},
            |_| {},
            || false,
            &[sub],
        );
        scheduler().set_default_command(sub, default.clone());
        run_n(3);
        assert_eq!(count.get(), 3);
        assert!(!default.is_scheduled());
        assert!(other.is_scheduled());
    }
}

#[test]
fn cancel_default_command_from_end() {
    let sub = SubsystemId::new();
    let count = counter();
    let c = count.clone();
    let default = Commands::functional(|| {}, || {}, move |_| c.set(c.get() + 1), || false, &[sub]);
    let other = Commands::run_once(|| {}, &[sub]);
    let (c, o) = (count.clone(), other.clone());
    let canceller = Commands::functional(
        || {},
        || {},
        move |_| {
            c.set(c.get() + 1);
            scheduler().schedule(&o);
        },
        || false,
        &[],
    );
    scheduler().schedule(&canceller);
    scheduler().set_default_command(sub, default.clone());
    run_n(1);
    scheduler().cancel(&canceller);
    assert_eq!(count.get(), 2);
    assert!(!default.is_scheduled());
    assert!(other.is_scheduled());
}

#[test]
fn schedule_from_execute_is_queued_until_after_the_loop() {
    let (target, tc) = mock(true, &[]);
    let t = target.clone();
    let log = Rc::new(RefCell::new(Vec::new()));
    let l = log.clone();
    let scheduler_cmd = Commands::run(
        move || {
            scheduler().schedule(&t);
            l.borrow_mut().push(t.is_scheduled());
        },
        &[],
    );
    scheduler().schedule(&scheduler_cmd);
    run_n(1);
    assert_eq!(*log.borrow(), [false]);
    assert!(target.is_scheduled());
    assert_eq!(tc.init.get(), 1);
    assert_eq!(tc.exec.get(), 0);
}

#[test]
fn cancel_other_from_execute_is_queued_until_after_the_loop() {
    let (target, tc) = mock(true, &[]);
    let t = target.clone();
    let canceller = Commands::run(move || scheduler().cancel(&t), &[]);
    scheduler().schedule(&target);
    scheduler().schedule(&canceller);
    run_n(1);
    assert!(!target.is_scheduled());
    assert_eq!((tc.exec.get(), tc.end_true.get()), (1, 1));
}

#[test]
fn schedule_self_from_execute_is_a_noop() {
    let slot: SelfSlot = Rc::default();
    let s = slot.clone();
    let inits = counter();
    let i = inits.clone();
    let cmd = Commands::functional(
        move || i.set(i.get() + 1),
        move || {
            let me = s.borrow().clone().unwrap();
            scheduler().schedule(&me);
        },
        |_| {},
        || false,
        &[],
    );
    *slot.borrow_mut() = Some(cmd.clone());
    scheduler().schedule(&cmd);
    run_n(3);
    assert_eq!(inits.get(), 1);
    assert!(cmd.is_scheduled());
}

#[test]
fn cancel_self_from_execute_ends_after_the_loop() {
    let slot: SelfSlot = Rc::default();
    let s = slot.clone();
    let ends = counter();
    let e = ends.clone();
    let cmd = Commands::functional(
        || {},
        move || {
            let me = s.borrow().clone().unwrap();
            scheduler().cancel(&me);
        },
        move |interrupted| {
            assert!(interrupted);
            e.set(e.get() + 1);
        },
        || false,
        &[],
    );
    *slot.borrow_mut() = Some(cmd.clone());
    scheduler().schedule(&cmd);
    run_n(1);
    assert_eq!(ends.get(), 1);
    assert!(!cmd.is_scheduled());
}

#[test]
fn schedule_from_finishing_command_end() {
    let (target, tc) = mock(true, &[]);
    let t = target.clone();
    let cmd = Commands::functional(
        || {},
        || {},
        move |_| scheduler().schedule(&t),
        || true,
        &[],
    );
    scheduler().schedule(&cmd);
    run_n(2);
    assert_eq!(tc.init.get(), 1);
    assert!(target.is_scheduled());
}

#[test]
fn trigger_binding_scheduling_during_poll() {
    let pressed = Rc::new(Cell::new(false));
    let p = pressed.clone();
    let (cmd, c) = mock(true, &[]);
    Trigger::new(move || p.get()).on_true(cmd.clone());
    run_n(1);
    pressed.set(true);
    run_n(1);
    assert_eq!((c.init.get(), c.exec.get()), (1, 1));
}

#[test]
fn command_initialize_may_cancel_other_scheduled_command() {
    let (victim, vc) = mock(true, &[]);
    let v = victim.clone();
    let killer = Commands::run_once(move || scheduler().cancel(&v), &[]);
    scheduler().schedule(&victim);
    scheduler().schedule(&killer);
    assert_eq!(vc.end_true.get(), 1);
    assert!(!victim.is_scheduled());
}

#[test]
fn group_member_cancelling_its_group_from_initialize() {
    let slot: SelfSlot = Rc::default();
    let s = slot.clone();
    let canceller = Commands::run_once(
        move || {
            let group = s.borrow().clone().unwrap();
            scheduler().cancel(&group);
        },
        &[],
    );
    let (after, ac) = mock(true, &[]);
    let group = Commands::sequence([canceller, after]);
    *slot.borrow_mut() = Some(group.clone());
    scheduler().schedule(&group);
    run_n(2);
    assert!(!group.is_scheduled());
    assert_eq!(ac.exec.get(), 0);
}

#[test]
fn deferred_supplier_may_schedule() {
    let (other, oc) = mock(true, &[]);
    let o = other.clone();
    let deferred = Commands::defer(
        move || {
            scheduler().schedule(&o);
            Commands::none()
        },
        &[],
    );
    scheduler().schedule(&deferred);
    run_n(1);
    assert_eq!(oc.init.get(), 1);
}

#[test]
fn event_loop_bind_while_polling_panics_with_message() {
    let event_loop = Rc::new(EventLoop::new());
    let l = event_loop.clone();
    event_loop.bind(move || l.bind(|| {}));
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| event_loop.poll()));
    assert!(result.is_err());
    // The loop is usable again afterwards.
    event_loop.clear();
    event_loop.bind(|| {});
    event_loop.poll();
}

#[test]
fn robot_mode_trigger_schedules_via_poll_without_borrow_conflicts() {
    let mode = FakeRobotState::install();
    let (cmd, c) = mock(true, &[]);
    RobotModeTriggers::teleop().negate().on_true(cmd);
    mode.set_mode(commands::testing::Mode::Disabled);
    run_n(1);
    assert_eq!(c.init.get(), 1);
    let sub: Rc<dyn Subsystem> = TestSubsystem::new();
    scheduler().register_subsystem(sub);
    run_n(1);
}

use crate::common::*;
use commands::testing::{FakeRobotState, Mode};
use commands::{Command, Commands, InterruptionBehavior, Subsystem, SubsystemId};
use std::cell::RefCell;
use std::rc::Rc;

#[test]
fn instant_schedule() {
    let (cmd, c) = mock(true, &[]);
    c.finished.set(true);
    scheduler().schedule(&cmd);
    assert!(cmd.is_scheduled());
    assert_eq!(c.init.get(), 1);
    run_n(1);
    assert_eq!((c.exec.get(), c.end_false.get()), (1, 1));
    assert!(!cmd.is_scheduled());
}

#[test]
fn single_iteration_schedule() {
    let (cmd, c) = mock(true, &[]);
    scheduler().schedule(&cmd);
    run_n(1);
    c.finished.set(true);
    run_n(1);
    assert_eq!((c.init.get(), c.exec.get(), c.end_false.get()), (1, 2, 1));
    assert!(!cmd.is_scheduled());
}

#[test]
fn multi_schedule() {
    let mocks: Vec<_> = (0..3).map(|_| mock(true, &[])).collect();
    for (cmd, _) in &mocks {
        scheduler().schedule(cmd);
    }
    run_n(1);
    assert!(mocks.iter().all(|(c, _)| c.is_scheduled()));
    for (i, (_, counts)) in mocks.iter().enumerate() {
        counts.finished.set(true);
        run_n(1);
        let scheduled: Vec<bool> = mocks.iter().map(|(c, _)| c.is_scheduled()).collect();
        assert_eq!(scheduled.iter().filter(|s| **s).count(), 2 - i);
    }
}

#[test]
fn cancel_calls_end_interrupted() {
    let (cmd, c) = mock(true, &[]);
    scheduler().schedule(&cmd);
    run_n(1);
    scheduler().cancel(&cmd);
    run_n(1);
    assert_eq!(
        (c.exec.get(), c.end_true.get(), c.end_false.get()),
        (1, 1, 0)
    );
    assert!(!cmd.is_scheduled());
}

#[test]
fn cancel_unscheduled_is_noop() {
    let (cmd, c) = mock(true, &[]);
    scheduler().cancel(&cmd);
    assert_eq!(c.end_true.get(), 0);
}

#[test]
fn schedule_scheduled_is_noop() {
    let (cmd, c) = mock(true, &[]);
    scheduler().schedule(&cmd);
    scheduler().schedule(&cmd);
    assert_eq!(c.init.get(), 1);
}

#[test]
fn cancel_all_ends_everything() {
    let (a, ca) = mock(true, &[]);
    let (b, cb) = mock(true, &[]);
    scheduler().schedule(&a);
    scheduler().schedule(&b);
    scheduler().cancel_all();
    assert_eq!((ca.end_true.get(), cb.end_true.get()), (1, 1));
    assert!(!a.is_scheduled() && !b.is_scheduled());
}

#[test]
fn requirement_interrupts_running_command() {
    let sub = SubsystemId::new();
    let (interrupted, ci) = mock(true, &[sub]);
    let (interrupter, cr) = mock(true, &[sub]);
    scheduler().schedule(&interrupted);
    run_n(1);
    scheduler().schedule(&interrupter);
    run_n(1);
    assert_eq!((ci.init.get(), ci.exec.get(), ci.end_true.get()), (1, 1, 1));
    assert_eq!((cr.init.get(), cr.exec.get()), (1, 1));
    assert!(!interrupted.is_scheduled());
    assert!(interrupter.is_scheduled());
}

#[test]
fn cancel_incoming_blocks_new_command() {
    let sub = SubsystemId::new();
    let kept = Commands::idle(&[sub]).with_interrupt_behavior(InterruptionBehavior::CancelIncoming);
    let (incoming, _) = mock(true, &[sub]);
    scheduler().schedule(&kept);
    scheduler().schedule(&incoming);
    assert!(kept.is_scheduled());
    assert!(!incoming.is_scheduled());
}

#[test]
fn requiring_tracks_owner_and_clears_on_end() {
    let sub = SubsystemId::new();
    let (cmd, _) = mock(true, &[sub]);
    scheduler().schedule(&cmd);
    assert_eq!(scheduler().requiring(sub), Some(cmd.clone()));
    scheduler().cancel(&cmd);
    assert_eq!(scheduler().requiring(sub), None);
}

#[test]
#[should_panic(expected = "Default commands must require their subsystem")]
fn default_command_must_require_subsystem() {
    scheduler().set_default_command(SubsystemId::new(), Commands::idle(&[]));
}

#[test]
fn default_command_scheduled_on_run() {
    let sub = TestSubsystem::new();
    let (default, _) = mock(true, &[sub.id]);
    sub.set_default_command(default.clone());
    run_n(1);
    assert!(default.is_scheduled());
    assert_eq!(sub.get_default_command(), Some(default.clone()));
    assert_eq!(sub.get_current_command(), Some(default));
}

#[test]
fn default_command_interrupt_and_resume() {
    let sub = SubsystemId::new();
    let (default, _) = mock(true, &[sub]);
    let (interrupter, _) = mock(true, &[sub]);
    scheduler().set_default_command(sub, default.clone());
    run_n(1);
    scheduler().schedule(&interrupter);
    assert!(!default.is_scheduled() && interrupter.is_scheduled());
    scheduler().cancel(&interrupter);
    run_n(1);
    assert!(default.is_scheduled() && !interrupter.is_scheduled());
}

#[test]
fn remove_default_command() {
    let sub = SubsystemId::new();
    let (default, _) = mock(true, &[sub]);
    scheduler().set_default_command(sub, default.clone());
    scheduler().remove_default_command(sub);
    run_n(1);
    assert!(!default.is_scheduled());
}

#[test]
fn default_command_resumes_after_disable() {
    let mode = FakeRobotState::install();
    let sub = SubsystemId::new();
    let (default, c) = mock(false, &[sub]);
    scheduler().set_default_command(sub, default.clone());
    run_n(1);
    assert!(default.is_scheduled());
    mode.set_mode(Mode::Disabled);
    run_n(1);
    assert!(!default.is_scheduled());
    mode.set_mode(Mode::Teleop);
    run_n(1);
    assert!(default.is_scheduled());
    assert_eq!(c.end_true.get(), 1);
}

#[test]
fn disabled_robot_cancels_and_refuses_commands() {
    let mode = FakeRobotState::install();
    let (cmd, c) = mock(false, &[]);
    scheduler().schedule(&cmd);
    mode.set_mode(Mode::Disabled);
    run_n(1);
    assert!(!cmd.is_scheduled());
    assert_eq!(c.end_true.get(), 1);
    scheduler().schedule(&cmd);
    assert!(!cmd.is_scheduled());
}

#[test]
fn runs_when_disabled_survives_disable() {
    let mode = FakeRobotState::install();
    let (cmd, _) = mock(true, &[]);
    scheduler().schedule(&cmd);
    mode.set_mode(Mode::Disabled);
    run_n(1);
    assert!(cmd.is_scheduled());
}

#[test]
fn group_runs_when_disabled_only_if_all_members_do() {
    let mode = FakeRobotState::install();
    let yes = Commands::sequence([mock(true, &[]).0, mock(true, &[]).0]);
    let no = Commands::sequence([mock(true, &[]).0, mock(false, &[]).0]);
    let par_yes = Commands::parallel([mock(true, &[]).0, mock(true, &[]).0]);
    let par_no = Commands::parallel([mock(true, &[]).0, mock(false, &[]).0]);
    for c in [&yes, &no, &par_yes, &par_no] {
        scheduler().schedule(c);
    }
    mode.set_mode(Mode::Disabled);
    run_n(1);
    assert!(yes.is_scheduled() && par_yes.is_scheduled());
    assert!(!no.is_scheduled() && !par_no.is_scheduled());
}

#[test]
fn disabled_scheduler_does_nothing() {
    let (cmd, _) = mock(true, &[]);
    scheduler().disable();
    scheduler().schedule(&cmd);
    assert!(!cmd.is_scheduled());
    scheduler().enable();
    scheduler().schedule(&cmd);
    assert!(cmd.is_scheduled());
}

#[test]
fn registered_subsystem_periodic_runs_each_iteration() {
    let sub = TestSubsystem::new();
    scheduler().register_subsystem(sub.clone());
    run_n(3);
    assert_eq!(sub.periodic.get(), 3);
    assert_eq!(sub.sim_periodic.get(), 0);
}

#[test]
fn simulation_periodic_only_in_simulation() {
    let state = FakeRobotState::install();
    state.set_simulation(true);
    let sub = TestSubsystem::new();
    scheduler().register_subsystem(sub.clone());
    run_n(2);
    assert_eq!((sub.periodic.get(), sub.sim_periodic.get()), (2, 2));
}

#[test]
fn unregistered_subsystem_stops() {
    let sub = TestSubsystem::new();
    scheduler().register_subsystem(sub.clone());
    scheduler().unregister_subsystem(sub.id);
    run_n(1);
    assert_eq!(sub.periodic.get(), 0);
}

#[test]
fn double_register_runs_periodic_once() {
    let sub = TestSubsystem::new();
    scheduler().register_subsystem(sub.clone());
    scheduler().register_subsystem(sub.clone());
    run_n(1);
    assert_eq!(sub.periodic.get(), 1);
}

#[test]
fn subsystem_helpers_require_the_subsystem() {
    let sub = TestSubsystem::new();
    let cmd = sub.run_once(|| {});
    assert_eq!(cmd.get_requirements(), &[sub.id]);
    assert!(sub.idle().has_requirement(sub.id));
    assert!(sub.defer(Commands::none).has_requirement(sub.id));
}

#[test]
fn run_order_is_periodic_then_buttons_then_commands_then_defaults() {
    let log = Rc::new(RefCell::new(Vec::<&'static str>::new()));
    struct Logger(SubsystemId, Rc<RefCell<Vec<&'static str>>>);
    impl Subsystem for Logger {
        fn id(&self) -> SubsystemId {
            self.0
        }
        fn periodic(&self) {
            self.1.borrow_mut().push("periodic");
        }
    }
    let sub = Rc::new(Logger(SubsystemId::new(), log.clone()));
    scheduler().register_subsystem(sub.clone());
    let l = log.clone();
    scheduler()
        .get_default_button_loop()
        .bind(move || l.borrow_mut().push("buttons"));
    let l = log.clone();
    let running = Commands::run(move || l.borrow_mut().push("execute"), &[]);
    scheduler().schedule(&running);
    let l = log.clone();
    let default = Commands::run_once(move || l.borrow_mut().push("default"), &[sub.0]);
    scheduler().set_default_command(sub.0, default);
    run_n(1);
    assert_eq!(*log.borrow(), ["periodic", "buttons", "execute", "default"]);
}

#[test]
fn schedule_from_periodic_runs_in_same_iteration() {
    struct Scheduling(SubsystemId, Command);
    impl Subsystem for Scheduling {
        fn id(&self) -> SubsystemId {
            self.0
        }
        fn periodic(&self) {
            scheduler().schedule(&self.1);
        }
    }
    let (cmd, c) = mock(true, &[]);
    scheduler().register_subsystem(Rc::new(Scheduling(SubsystemId::new(), cmd)));
    run_n(1);
    assert_eq!((c.init.get(), c.exec.get()), (1, 1));
}

#[test]
#[should_panic(expected = "composed may not")]
fn composed_command_cannot_be_scheduled() {
    let (inner, _) = mock(true, &[]);
    let _group = Commands::sequence(inner.clone());
    scheduler().schedule(&inner);
}

#[test]
#[should_panic(expected = "composed may not")]
fn composed_command_cannot_be_composed_again() {
    let (inner, _) = mock(true, &[]);
    let _a = Commands::sequence(inner.clone());
    let _b = Commands::parallel(inner);
}

#[test]
#[should_panic(expected = "scheduled individually")]
fn scheduled_command_cannot_be_composed() {
    let (inner, _) = mock(true, &[]);
    scheduler().schedule(&inner);
    let _group = Commands::sequence(inner);
}

#[test]
#[should_panic(expected = "twice in the same composition")]
fn command_cannot_be_composed_twice_in_one_group() {
    let (inner, _) = mock(true, &[]);
    let _group = Commands::sequence([inner.clone(), inner]);
}

#[test]
#[should_panic(expected = "composed may not")]
fn composed_command_cannot_be_default() {
    let sub = SubsystemId::new();
    let (inner, _) = mock(true, &[sub]);
    let _wrapped = inner.clone().with_name("x");
    scheduler().set_default_command(sub, inner);
}

#[test]
fn custom_builder_sets_metadata() {
    let sub = SubsystemId::new();
    let cmd = Command::builder("Custom", Noop)
        .requires(&[sub, sub])
        .build();
    assert_eq!(cmd.get_name(), "Custom");
    assert_eq!(cmd.get_requirements(), &[sub]);
}

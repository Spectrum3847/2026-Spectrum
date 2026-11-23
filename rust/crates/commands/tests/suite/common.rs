#![allow(dead_code)]

use commands::{Command, CommandBody, CommandScheduler, Subsystem, SubsystemId};
use std::cell::Cell;
use std::rc::Rc;

#[derive(Default)]
pub struct Counts {
    pub init: Cell<u32>,
    pub exec: Cell<u32>,
    pub end_false: Cell<u32>,
    pub end_true: Cell<u32>,
    pub finished: Cell<bool>,
}

struct MockBody(Rc<Counts>);

impl CommandBody for MockBody {
    fn initialize(&mut self) {
        self.0.init.set(self.0.init.get() + 1);
    }
    fn execute(&mut self) {
        self.0.exec.set(self.0.exec.get() + 1);
    }
    fn end(&mut self, interrupted: bool) {
        let c = if interrupted {
            &self.0.end_true
        } else {
            &self.0.end_false
        };
        c.set(c.get() + 1);
    }
    fn is_finished(&mut self) -> bool {
        self.0.finished.get()
    }
}

/// A command that counts its callbacks and finishes when `finished` is set.
pub fn mock(runs_when_disabled: bool, requirements: &[SubsystemId]) -> (Command, Rc<Counts>) {
    let counts = Rc::new(Counts::default());
    let command = Command::builder("Mock", MockBody(counts.clone()))
        .requires(requirements)
        .runs_when_disabled(runs_when_disabled)
        .build();
    (command, counts)
}

pub fn scheduler() -> Rc<CommandScheduler> {
    CommandScheduler::get_instance()
}

pub fn run_n(n: usize) {
    for _ in 0..n {
        scheduler().run();
    }
}

pub fn counter() -> Rc<Cell<u32>> {
    Rc::new(Cell::new(0))
}

pub fn bump(c: &Rc<Cell<u32>>) -> impl FnMut() + 'static + use<> {
    let c = c.clone();
    move || c.set(c.get() + 1)
}

pub struct TestSubsystem {
    pub id: SubsystemId,
    pub periodic: Rc<Cell<u32>>,
    pub sim_periodic: Rc<Cell<u32>>,
}

impl TestSubsystem {
    pub fn new() -> Rc<Self> {
        Rc::new(Self {
            id: SubsystemId::new(),
            periodic: counter(),
            sim_periodic: counter(),
        })
    }
}

impl Subsystem for TestSubsystem {
    fn id(&self) -> SubsystemId {
        self.id
    }
    fn periodic(&self) {
        self.periodic.set(self.periodic.get() + 1);
    }
    fn simulation_periodic(&self) {
        self.sim_periodic.set(self.sim_periodic.get() + 1);
    }
}

pub struct Noop;

impl CommandBody for Noop {}

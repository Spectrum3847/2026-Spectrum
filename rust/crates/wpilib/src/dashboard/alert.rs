//! Java's `Alert`: messages dashboards show while a condition holds.
//!
//! Alerts register on the loop thread, as [`put_data`](super::smart_dashboard::put_data) does.
//! Each group is a Sendable under `SmartDashboard/<group>` with string arrays `errors`,
//! `warnings` and `infos`, newest first.

use super::sendable::{Sendable, SendableBuilder};
use super::smart_dashboard;
use crate::robot_controller;
use std::cell::{Cell, RefCell};
use std::collections::{BTreeSet, HashMap};
use std::rc::Rc;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AlertType {
    Error,
    Warning,
    Info,
}

/// An active alert. Newest first, then by text.
#[derive(Clone, Debug, PartialEq, Eq)]
struct PublishedAlert {
    timestamp: i64,
    text: String,
}

impl Ord for PublishedAlert {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other
            .timestamp
            .cmp(&self.timestamp)
            .then_with(|| self.text.cmp(&other.text))
    }
}

impl PartialOrd for PublishedAlert {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

/// The active alerts of one group.
#[derive(Default)]
struct Group {
    errors: BTreeSet<PublishedAlert>,
    warnings: BTreeSet<PublishedAlert>,
    infos: BTreeSet<PublishedAlert>,
}

impl Group {
    fn set(&mut self, kind: AlertType) -> &mut BTreeSet<PublishedAlert> {
        match kind {
            AlertType::Error => &mut self.errors,
            AlertType::Warning => &mut self.warnings,
            AlertType::Info => &mut self.infos,
        }
    }

    fn texts(set: &BTreeSet<PublishedAlert>) -> Vec<String> {
        set.iter().map(|a| a.text.clone()).collect()
    }
}

type SharedGroup = Rc<RefCell<Group>>;

struct SendableAlerts(SharedGroup);

impl Sendable for SendableAlerts {
    fn init_sendable(&self, builder: &mut SendableBuilder) {
        builder.set_smartdashboard_type("Alerts");
        let group = self.0.clone();
        builder.add_string_array_getter("errors", move || Group::texts(&group.borrow().errors));
        let group = self.0.clone();
        builder.add_string_array_getter("warnings", move || Group::texts(&group.borrow().warnings));
        let group = self.0.clone();
        builder.add_string_array_getter("infos", move || Group::texts(&group.borrow().infos));
    }

    fn identity(&self) -> usize {
        Rc::as_ptr(&self.0) as usize
    }
}

thread_local! {
    static GROUPS: RefCell<HashMap<String, SharedGroup>> = RefCell::new(HashMap::new());
}

/// The group's storage, putting the group on the dashboard the first time.
fn for_group(name: &str) -> SharedGroup {
    smart_dashboard::check_loop_thread();
    if let Some(group) = GROUPS.with(|g| g.borrow().get(name).cloned()) {
        return group;
    }
    let group = SharedGroup::default();
    GROUPS.with(|g| g.borrow_mut().insert(name.to_owned(), group.clone()));
    smart_dashboard::put_data(name, &SendableAlerts(group.clone()));
    group
}

pub struct Alert {
    kind: AlertType,
    active: Cell<bool>,
    start_time: Cell<i64>,
    text: RefCell<String>,
    group: SharedGroup,
}

impl Alert {
    /// An alert in the `Alerts` group.
    pub fn new(text: &str, kind: AlertType) -> Self {
        Self::in_group("Alerts", text, kind)
    }

    pub fn in_group(group: &str, text: &str, kind: AlertType) -> Self {
        Self::with_group(for_group(group), text, kind)
    }

    fn with_group(group: SharedGroup, text: &str, kind: AlertType) -> Self {
        Self {
            kind,
            active: Cell::new(false),
            start_time: Cell::new(0),
            text: RefCell::new(text.to_owned()),
            group,
        }
    }

    /// Shows or hides the alert. Setting the state it already has does nothing.
    pub fn set(&self, active: bool) {
        if active == self.active.get() {
            return;
        }
        let time = if active {
            robot_controller::time() as i64
        } else {
            0
        };
        self.set_at(active, time);
    }

    fn set_at(&self, active: bool, time: i64) {
        if active == self.active.get() {
            return;
        }
        let mut group = self.group.borrow_mut();
        if active {
            self.start_time.set(time);
            group.set(self.kind).insert(self.published());
        } else {
            group.set(self.kind).remove(&self.published());
        }
        self.active.set(active);
    }

    pub fn get(&self) -> bool {
        self.active.get()
    }

    /// Changes the text, keeping the activation time if the alert is active.
    pub fn set_text(&self, text: &str) {
        if *self.text.borrow() == text {
            return;
        }
        let old = self.published();
        *self.text.borrow_mut() = text.to_owned();
        if self.active.get() {
            let mut group = self.group.borrow_mut();
            let set = group.set(self.kind);
            set.remove(&old);
            set.insert(self.published());
        }
    }

    pub fn text(&self) -> String {
        self.text.borrow().clone()
    }

    pub fn kind(&self) -> AlertType {
        self.kind
    }

    fn published(&self) -> PublishedAlert {
        PublishedAlert {
            timestamp: self.start_time.get(),
            text: self.text.borrow().clone(),
        }
    }
}

impl Drop for Alert {
    fn drop(&mut self) {
        self.set(false);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn alert(group: &SharedGroup, text: &str, kind: AlertType) -> Alert {
        Alert::with_group(group.clone(), text, kind)
    }

    fn errors(group: &SharedGroup) -> Vec<String> {
        Group::texts(&group.borrow().errors)
    }

    #[test]
    fn newest_comes_first() {
        let group = SharedGroup::default();
        let (a, b, c) = (
            alert(&group, "a", AlertType::Error),
            alert(&group, "b", AlertType::Error),
            alert(&group, "c", AlertType::Error),
        );
        a.set_at(true, 100);
        b.set_at(true, 300);
        c.set_at(true, 200);
        assert_eq!(errors(&group), ["b", "c", "a"]);
    }

    #[test]
    fn same_time_sorts_by_text() {
        let group = SharedGroup::default();
        let (z, a) = (
            alert(&group, "z", AlertType::Error),
            alert(&group, "a", AlertType::Error),
        );
        z.set_at(true, 5);
        a.set_at(true, 5);
        assert_eq!(errors(&group), ["a", "z"]);
    }

    #[test]
    fn types_are_separate_lists() {
        let group = SharedGroup::default();
        let (e, w, i) = (
            alert(&group, "e", AlertType::Error),
            alert(&group, "w", AlertType::Warning),
            alert(&group, "i", AlertType::Info),
        );
        e.set_at(true, 1);
        w.set_at(true, 1);
        i.set_at(true, 1);
        let g = group.borrow();
        assert_eq!(Group::texts(&g.errors), ["e"]);
        assert_eq!(Group::texts(&g.warnings), ["w"]);
        assert_eq!(Group::texts(&g.infos), ["i"]);
    }

    #[test]
    fn clearing_removes_and_repeats_do_nothing() {
        let group = SharedGroup::default();
        let a = alert(&group, "a", AlertType::Error);
        a.set_at(true, 1);
        a.set_at(true, 99);
        assert_eq!(a.start_time.get(), 1);
        assert_eq!(errors(&group), ["a"]);
        a.set_at(false, 0);
        assert!(errors(&group).is_empty());
        assert!(!a.get());
        a.set_at(false, 0);
    }

    #[test]
    fn two_alerts_with_one_text_and_time_collapse_like_java() {
        let group = SharedGroup::default();
        let (a, b) = (
            alert(&group, "same", AlertType::Error),
            alert(&group, "same", AlertType::Error),
        );
        a.set_at(true, 7);
        b.set_at(true, 7);
        assert_eq!(errors(&group), ["same"]);
    }

    #[test]
    fn set_text_while_active_keeps_the_time() {
        let group = SharedGroup::default();
        let (a, b) = (
            alert(&group, "old", AlertType::Warning),
            alert(&group, "other", AlertType::Warning),
        );
        a.set_at(true, 10);
        b.set_at(true, 20);
        a.set_text("new");
        assert_eq!(Group::texts(&group.borrow().warnings), ["other", "new"]);
        assert_eq!(a.text(), "new");
        a.set_at(false, 0);
        assert_eq!(Group::texts(&group.borrow().warnings), ["other"]);
    }

    #[test]
    fn set_text_while_inactive_only_changes_the_text() {
        let group = SharedGroup::default();
        let a = alert(&group, "old", AlertType::Error);
        a.set_text("new");
        assert!(errors(&group).is_empty());
        a.set_at(true, 1);
        assert_eq!(errors(&group), ["new"]);
    }

    #[test]
    fn drop_clears_an_active_alert() {
        let group = SharedGroup::default();
        {
            let a = alert(&group, "a", AlertType::Error);
            a.set_at(true, 1);
        }
        assert!(errors(&group).is_empty());
    }

    #[test]
    fn group_layout_matches_java() {
        use crate::dashboard::io::FakeIo;
        let group = SharedGroup::default();
        let a = alert(&group, "boom", AlertType::Error);
        a.set_at(true, 1);
        let fake = FakeIo::shared();
        let mut builder = SendableBuilder::new("/SmartDashboard/Alerts", fake.clone());
        SendableAlerts(group).init_sendable(&mut builder);
        builder.update();
        let io = fake.borrow();
        assert_eq!(io.text("/SmartDashboard/Alerts/.type"), "Alerts");
        assert_eq!(io.strings("/SmartDashboard/Alerts/errors"), ["boom"]);
        assert!(io.strings("/SmartDashboard/Alerts/warnings").is_empty());
        assert!(io.strings("/SmartDashboard/Alerts/infos").is_empty());
    }
}

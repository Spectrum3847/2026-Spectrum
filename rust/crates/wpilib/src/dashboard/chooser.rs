//! Java's `SendableChooser`: a drop-down the driver picks from.

use super::sendable::{Sendable, SendableBuilder};
use crate::nt::Value;
use std::cell::RefCell;
use std::rc::Rc;
use std::sync::atomic::{AtomicI64, Ordering};

static INSTANCES: AtomicI64 = AtomicI64::new(0);

type Listener<V> = Rc<dyn Fn(Option<V>)>;

struct State<V> {
    /// Insertion order, like Java's `LinkedHashMap`. Re-adding a name keeps its place.
    options: Vec<(String, V)>,
    default_choice: String,
    selected: Option<String>,
    previous: Option<String>,
    listener: Option<Listener<V>>,
}

impl<V: Clone> State<V> {
    fn lookup(&self, name: &str) -> Option<V> {
        self.options
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, v)| v.clone())
    }

    /// The name the dashboard shows as active.
    fn active(&self) -> &str {
        self.selected.as_deref().unwrap_or(&self.default_choice)
    }
}

/// A set of named options. Clones share state, so one can sit on the dashboard while another
/// is read by the robot code.
pub struct SendableChooser<V> {
    state: Rc<RefCell<State<V>>>,
    instance: i64,
}

impl<V> Clone for SendableChooser<V> {
    fn clone(&self) -> Self {
        Self {
            state: self.state.clone(),
            instance: self.instance,
        }
    }
}

impl<V: Clone + 'static> Default for SendableChooser<V> {
    fn default() -> Self {
        Self::new()
    }
}

impl<V: Clone + 'static> SendableChooser<V> {
    pub fn new() -> Self {
        Self {
            state: Rc::new(RefCell::new(State {
                options: Vec::new(),
                default_choice: String::new(),
                selected: None,
                previous: None,
                listener: None,
            })),
            instance: INSTANCES.fetch_add(1, Ordering::Relaxed),
        }
    }

    pub fn add_option(&self, name: &str, value: V) {
        let mut state = self.state.borrow_mut();
        match state.options.iter_mut().find(|(n, _)| n == name) {
            Some(slot) => slot.1 = value,
            None => state.options.push((name.to_owned(), value)),
        }
    }

    pub fn set_default_option(&self, name: &str, value: V) {
        self.state.borrow_mut().default_choice = name.to_owned();
        self.add_option(name, value);
    }

    /// The selected option, or the default when the dashboard has not chosen. `None` when the
    /// name has no option.
    pub fn get_selected(&self) -> Option<V> {
        let state = self.state.borrow();
        state.lookup(state.active())
    }

    /// Calls `listener` when the dashboard picks a different name than last time. It receives
    /// `None` for a name with no option.
    pub fn on_change(&self, listener: impl Fn(Option<V>) + 'static) {
        self.state.borrow_mut().listener = Some(Rc::new(listener));
    }
}

impl<V: Clone + 'static> Sendable for SendableChooser<V> {
    fn init_sendable(&self, builder: &mut SendableBuilder) {
        builder.set_smartdashboard_type("String Chooser");
        builder.publish_const(".instance", Value::Integer(self.instance));
        let state = self.state.clone();
        builder.add_string_getter("default", move || state.borrow().default_choice.clone());
        let state = self.state.clone();
        builder.add_string_array_getter("options", move || {
            state
                .borrow()
                .options
                .iter()
                .map(|(n, _)| n.clone())
                .collect()
        });
        let state = self.state.clone();
        builder.add_string_getter("active", move || state.borrow().active().to_owned());
        let state = self.state.clone();
        builder.add_string_setter("selected", move |value| {
            // The listener runs after the borrow ends, so it may use the chooser.
            let call = {
                let mut state = state.borrow_mut();
                state.selected = Some(value.clone());
                let call = if state.previous.as_deref() != Some(&value) {
                    state
                        .listener
                        .clone()
                        .map(|listener| (listener, state.lookup(&value)))
                } else {
                    None
                };
                state.previous = Some(value);
                call
            };
            if let Some((listener, choice)) = call {
                listener(choice);
            }
        });
    }

    fn identity(&self) -> usize {
        Rc::as_ptr(&self.state) as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dashboard::io::FakeIo;
    use std::cell::Cell;

    const TABLE: &str = "/SmartDashboard/Auto Chooser";

    fn published(chooser: &SendableChooser<i32>) -> (Rc<RefCell<FakeIo>>, SendableBuilder) {
        let fake = FakeIo::shared();
        let mut builder = SendableBuilder::new(TABLE, fake.clone());
        chooser.init_sendable(&mut builder);
        builder.start_listeners();
        (fake, builder)
    }

    fn two_options() -> SendableChooser<i32> {
        let chooser = SendableChooser::new();
        chooser.set_default_option("Leave", 1);
        chooser.add_option("Score", 2);
        chooser
    }

    #[test]
    fn layout_matches_java() {
        let chooser = two_options();
        let (fake, mut builder) = published(&chooser);
        builder.update();
        let io = fake.borrow();
        assert_eq!(io.text(&format!("{TABLE}/.type")), "String Chooser");
        assert_eq!(
            io.properties[&format!("{TABLE}/.type")],
            "{\"SmartDashboard\":\"String Chooser\"}"
        );
        assert!(matches!(
            io.values[&format!("{TABLE}/.instance")],
            Value::Integer(_)
        ));
        assert_eq!(io.text(&format!("{TABLE}/default")), "Leave");
        assert_eq!(io.strings(&format!("{TABLE}/options")), ["Leave", "Score"]);
        assert_eq!(io.text(&format!("{TABLE}/active")), "Leave");
        assert_eq!(
            io.values[&format!("{TABLE}/.controllable")],
            Value::Boolean(true)
        );
        assert_eq!(io.subscribed, [format!("{TABLE}/selected")]);
        assert!(!io.values.contains_key(&format!("{TABLE}/selected")));
    }

    #[test]
    fn instances_are_numbered() {
        let (a, b) = (SendableChooser::<i32>::new(), SendableChooser::<i32>::new());
        assert_eq!(b.instance, a.instance + 1);
    }

    #[test]
    fn default_is_selected_until_the_dashboard_picks() {
        let chooser = two_options();
        assert_eq!(chooser.get_selected(), Some(1));
        let (fake, mut builder) = published(&chooser);
        fake.borrow_mut()
            .push(&format!("{TABLE}/selected"), Value::String("Score".into()));
        builder.update();
        assert_eq!(chooser.get_selected(), Some(2));
        builder.update();
        assert_eq!(fake.borrow().text(&format!("{TABLE}/active")), "Score");
    }

    #[test]
    fn unknown_selection_gives_none() {
        let chooser = two_options();
        let (fake, mut builder) = published(&chooser);
        fake.borrow_mut()
            .push(&format!("{TABLE}/selected"), Value::String("Nope".into()));
        builder.update();
        assert_eq!(chooser.get_selected(), None);
    }

    #[test]
    fn empty_chooser_has_no_selection() {
        assert_eq!(SendableChooser::<i32>::new().get_selected(), None);
    }

    #[test]
    fn re_adding_a_name_replaces_the_value_in_place() {
        let chooser = two_options();
        chooser.add_option("Leave", 9);
        let (fake, mut builder) = published(&chooser);
        builder.update();
        assert_eq!(
            fake.borrow().strings(&format!("{TABLE}/options")),
            ["Leave", "Score"]
        );
        assert_eq!(chooser.get_selected(), Some(9));
    }

    #[test]
    fn on_change_fires_only_when_the_name_changes() {
        let chooser = two_options();
        let calls = Rc::new(RefCell::new(Vec::new()));
        let seen = calls.clone();
        chooser.on_change(move |v| seen.borrow_mut().push(v));
        let (fake, mut builder) = published(&chooser);
        let path = format!("{TABLE}/selected");
        for name in ["Score", "Score", "Leave", "Nope"] {
            fake.borrow_mut().push(&path, Value::String(name.into()));
            builder.update();
        }
        assert_eq!(*calls.borrow(), [Some(2), Some(1), None]);
    }

    #[test]
    fn listener_may_read_the_chooser() {
        let chooser = two_options();
        let seen = Rc::new(Cell::new(0));
        let (c, s) = (chooser.clone(), seen.clone());
        chooser.on_change(move |_| s.set(c.get_selected().unwrap_or(0)));
        let (fake, mut builder) = published(&chooser);
        fake.borrow_mut()
            .push(&format!("{TABLE}/selected"), Value::String("Score".into()));
        builder.update();
        assert_eq!(seen.get(), 2);
    }

    #[test]
    fn clones_share_state_and_identity() {
        let chooser = two_options();
        let clone = chooser.clone();
        assert_eq!(chooser.identity(), clone.identity());
        clone.add_option("Third", 3);
        assert_eq!(chooser.state.borrow().options.len(), 3);
    }
}

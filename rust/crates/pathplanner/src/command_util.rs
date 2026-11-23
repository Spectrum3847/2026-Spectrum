use crate::auto_builder::AutoBuilder;
use crate::error::{Error, Result};
use crate::path::PathPlannerPath;
use commands::{Command, Commands};
use serde_json::Value;
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

thread_local! {
    static NAMED_COMMANDS: RefCell<HashMap<String, Command>> = RefCell::new(HashMap::new());
}

/// Commands that autos and event markers refer to by name.
pub struct NamedCommands;

impl NamedCommands {
    pub fn register_command(name: impl Into<String>, command: Command) {
        NAMED_COMMANDS.with(|m| m.borrow_mut().insert(name.into(), command));
    }

    pub fn has_command(name: &str) -> bool {
        NAMED_COMMANDS.with(|m| m.borrow().contains_key(name))
    }

    /// The registered command wrapped so it can run in several groups. An unregistered name
    /// gives a warning and a command that does nothing.
    pub fn get_command(name: &str) -> Command {
        match NAMED_COMMANDS.with(|m| m.borrow().get(name).cloned()) {
            Some(command) => wrapped_event_command(command),
            None => {
                wpilib::driver_station::report_warning(&format!(
                    "PathPlanner attempted to create a command '{name}' that has not been \
                     registered with NamedCommands.registerCommand"
                ));
                Commands::none()
            }
        }
    }

    pub fn clear_all() {
        NAMED_COMMANDS.with(|m| m.borrow_mut().clear());
    }
}

/// Wraps a command in one that calls its initialize, execute, end and is_finished, so a
/// registered command can be reused in different command groups.
pub fn wrapped_event_command(event_command: Command) -> Command {
    let requirements = event_command.get_requirements().to_vec();
    let (init, run, end, finished) = (
        event_command.clone(),
        event_command.clone(),
        event_command.clone(),
        event_command,
    );
    Commands::functional(
        move || init.initialize(),
        move || run.execute(),
        move |interrupted| end.end(interrupted),
        move || finished.is_finished(),
        &requirements,
    )
}

/// An auto command tree as data, before the commands are built.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum CommandNode {
    None,
    Wait(f64),
    Named(String),
    Path(String),
    Sequential(Vec<CommandNode>),
    Parallel(Vec<CommandNode>),
    Race(Vec<CommandNode>),
    /// The first command is the deadline.
    Deadline(Vec<CommandNode>),
}

fn invalid(what: &str) -> Error {
    Error::Invalid(format!("auto command: {what}"))
}

impl CommandNode {
    pub fn from_json(json: &Value) -> Result<Self> {
        let kind = json
            .get("type")
            .and_then(Value::as_str)
            .ok_or_else(|| invalid("missing type"))?;
        let data = json.get("data").ok_or_else(|| invalid("missing data"))?;

        let children = |data: &Value| -> Result<Vec<CommandNode>> {
            data.get("commands")
                .and_then(Value::as_array)
                .ok_or_else(|| invalid("missing commands"))?
                .iter()
                .map(Self::from_json)
                .collect()
        };

        Ok(match kind {
            "wait" => {
                let wait_time = data
                    .get("waitTime")
                    .ok_or_else(|| invalid("missing waitTime"))?;
                // A Choreo expression keeps its number in `val`.
                let seconds = wait_time
                    .as_f64()
                    .or_else(|| wait_time.get("val").and_then(Value::as_f64))
                    .ok_or_else(|| invalid("bad waitTime"))?;
                CommandNode::Wait(seconds)
            }
            "named" => CommandNode::Named(
                data.get("name")
                    .and_then(Value::as_str)
                    .ok_or_else(|| invalid("missing name"))?
                    .to_string(),
            ),
            "path" => CommandNode::Path(
                data.get("pathName")
                    .and_then(Value::as_str)
                    .ok_or_else(|| invalid("missing pathName"))?
                    .to_string(),
            ),
            "sequential" => CommandNode::Sequential(children(data)?),
            "parallel" => CommandNode::Parallel(children(data)?),
            "race" => CommandNode::Race(children(data)?),
            "deadline" => CommandNode::Deadline(children(data)?),
            _ => CommandNode::None,
        })
    }

    /// Builds the commands. `mirror` mirrors every path.
    pub fn build(&self, mirror: bool) -> Result<Command> {
        let build_all = |nodes: &[CommandNode]| -> Result<Vec<Command>> {
            nodes.iter().map(|n| n.build(mirror)).collect()
        };
        Ok(match self {
            CommandNode::None => Commands::none(),
            CommandNode::Wait(seconds) => Commands::wait_seconds(*seconds),
            CommandNode::Named(name) => NamedCommands::get_command(name),
            CommandNode::Path(name) => {
                let mut path = PathPlannerPath::from_path_file(name)?;
                if mirror {
                    path = Rc::new(path.mirror_path());
                }
                AutoBuilder::follow_path(path)
            }
            CommandNode::Sequential(nodes) => Commands::sequence(build_all(nodes)?),
            CommandNode::Parallel(nodes) => Commands::parallel(build_all(nodes)?),
            CommandNode::Race(nodes) => Commands::race(build_all(nodes)?),
            CommandNode::Deadline(nodes) => match nodes.split_first() {
                Some((deadline, others)) => {
                    Commands::deadline(deadline.build(mirror)?, build_all(others)?)
                }
                None => Commands::none(),
            },
        })
    }

    /// Path names in the tree, depth first.
    pub fn path_names(&self, out: &mut Vec<String>) {
        match self {
            CommandNode::Path(name) => out.push(name.clone()),
            CommandNode::Sequential(nodes)
            | CommandNode::Parallel(nodes)
            | CommandNode::Race(nodes)
            | CommandNode::Deadline(nodes) => {
                for node in nodes {
                    node.path_names(out);
                }
            }
            _ => {}
        }
    }
}

/// Builds a command from the JSON of an auto or an event marker.
pub fn command_from_json(json: &Value, mirror: bool) -> Result<Command> {
    CommandNode::from_json(json)?.build(mirror)
}

#[cfg(test)]
mod tests {
    use super::*;
    use commands::testing::FakeClock;
    use commands::{CommandScheduler, SubsystemId};
    use serde_json::json;

    type Log = Rc<RefCell<Vec<String>>>;

    /// Logs init and end, and finishes after `ticks` executes. `ticks` 0 never finishes.
    fn logging_command(log: &Log, name: &str, ticks: u32) -> Command {
        let count = Rc::new(std::cell::Cell::new(0));
        let (init_log, end_log) = (log.clone(), log.clone());
        let (init_name, end_name) = (name.to_string(), name.to_string());
        let (run_count, done_count) = (count.clone(), count);
        Commands::functional(
            move || init_log.borrow_mut().push(format!("{init_name} init")),
            move || run_count.set(run_count.get() + 1),
            move |interrupted| {
                end_log
                    .borrow_mut()
                    .push(format!("{end_name} end {interrupted}"));
            },
            move || ticks != 0 && done_count.get() >= ticks,
            &[],
        )
    }

    fn run_to_end(command: &Command) {
        let clock = FakeClock::install();
        let scheduler = CommandScheduler::get_instance();
        scheduler.schedule(command);
        let mut loops = 0;
        while command.is_scheduled() {
            clock.advance(0.02);
            scheduler.run();
            loops += 1;
            assert!(loops < 1000, "the command did not end");
        }
    }

    #[test]
    fn named_commands_run_inside_the_groups_of_an_auto() {
        NamedCommands::clear_all();
        let log: Log = Rc::default();
        NamedCommands::register_command("a", logging_command(&log, "a", 1));
        NamedCommands::register_command("b", logging_command(&log, "b", 1));
        NamedCommands::register_command("c", logging_command(&log, "c", 2));
        NamedCommands::register_command("d", logging_command(&log, "d", 0));
        assert!(NamedCommands::has_command("a"));
        assert!(!NamedCommands::has_command("zzz"));

        let named = |name: &str| json!({"type": "named", "data": {"name": name}});
        let wait = |seconds: f64| json!({"type": "wait", "data": {"waitTime": seconds}});
        let tree = json!({"type": "sequential", "data": {"commands": [
            named("a"),
            {"type": "parallel", "data": {"commands": [named("b"), wait(0.1)]}},
            {"type": "race", "data": {"commands": [wait(0.5), named("c")]}},
            {"type": "deadline", "data": {"commands": [wait(0.1), named("d")]}},
        ]}});
        run_to_end(&command_from_json(&tree, false).unwrap());

        assert_eq!(
            *log.borrow(),
            [
                "a init",
                "a end false",
                "b init",
                "b end false",
                "c init",
                "c end false",
                "d init",
                "d end true"
            ]
        );
        NamedCommands::clear_all();
    }

    #[test]
    fn a_named_command_can_run_twice() {
        NamedCommands::clear_all();
        let log: Log = Rc::default();
        NamedCommands::register_command("x", logging_command(&log, "x", 1));
        let tree = json!({"type": "sequential", "data": {"commands": [
            {"type": "named", "data": {"name": "x"}},
            {"type": "named", "data": {"name": "x"}},
        ]}});
        run_to_end(&command_from_json(&tree, false).unwrap());
        assert_eq!(log.borrow().len(), 4);
        NamedCommands::clear_all();
    }

    #[test]
    fn a_name_that_is_not_registered_does_nothing() {
        NamedCommands::clear_all();
        let command = NamedCommands::get_command("zzz");
        command.initialize();
        assert!(command.is_finished());
    }

    #[test]
    fn the_wrapper_keeps_the_requirements() {
        let id = SubsystemId::new();
        let wrapped = wrapped_event_command(Commands::idle(&[id]));
        assert!(wrapped.has_requirement(id));
    }

    #[test]
    fn waits_read_a_plain_number_or_an_expression() {
        let plain = json!({"type": "wait", "data": {"waitTime": 2.5}});
        let expression = json!({"type": "wait", "data": {"waitTime": {"exp": "x", "val": 1.25}}});
        assert_eq!(
            CommandNode::from_json(&plain).unwrap(),
            CommandNode::Wait(2.5)
        );
        assert_eq!(
            CommandNode::from_json(&expression).unwrap(),
            CommandNode::Wait(1.25)
        );
    }

    #[test]
    fn an_unknown_type_is_a_command_that_does_nothing() {
        let node = CommandNode::from_json(&json!({"type": "mystery", "data": {}})).unwrap();
        assert_eq!(node, CommandNode::None);
        assert!(node.build(false).unwrap().is_finished());
    }

    #[test]
    fn an_empty_deadline_group_does_nothing() {
        let node =
            CommandNode::from_json(&json!({"type": "deadline", "data": {"commands": []}})).unwrap();
        assert!(node.build(false).unwrap().is_finished());
    }

    #[test]
    fn missing_fields_are_errors() {
        for bad in [
            json!({"data": {}}),
            json!({"type": "path"}),
            json!({"type": "path", "data": {}}),
            json!({"type": "named", "data": {}}),
            json!({"type": "wait", "data": {}}),
            json!({"type": "wait", "data": {"waitTime": "soon"}}),
            json!({"type": "sequential", "data": {}}),
        ] {
            assert!(CommandNode::from_json(&bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn a_missing_path_file_is_an_error() {
        let _deploy = crate::util::TestDeploy::new("command-util-missing-path");
        let node = CommandNode::Path("no such path".into());
        assert!(matches!(node.build(false), Err(Error::Io(_))));
    }
}

use crate::auto_builder::AutoBuilder;
use crate::command_util::CommandNode;
use crate::error::{Error, Result};
use crate::path::PathPlannerPath;
use crate::util::deploy_directory;
use commands::{Command, CommandBody, Commands};
use serde_json::Value;
use std::cell::RefCell;
use std::rc::Rc;
use wpimath::geometry::Pose2d;

thread_local! {
    static CURRENT_PATH_NAME: RefCell<String> = const { RefCell::new(String::new()) };
}

/// The name of the path being followed, or empty when none is.
pub fn current_path_name() -> String {
    CURRENT_PATH_NAME.with(|n| n.borrow().clone())
}

pub(crate) fn set_current_path_name(name: &str) {
    CURRENT_PATH_NAME.with(|n| {
        let mut n = n.borrow_mut();
        n.clear();
        n.push_str(name);
    });
}

/// An autonomous routine loaded from a PathPlanner auto file.
///
/// The auto's own triggers (`isRunning`, `timeElapsed`, `event` and the rest) are not ported,
/// because the robot does not use them. Use [`command`](Self::command) to run it.
pub struct PathPlannerAuto {
    command: Command,
    starting_pose: Option<Pose2d>,
}

struct AutoBody {
    command: Command,
}

impl CommandBody for AutoBody {
    fn initialize(&mut self) {
        self.command.initialize();
    }

    fn execute(&mut self) {
        self.command.execute();
    }

    fn is_finished(&mut self) -> bool {
        self.command.is_finished()
    }

    fn end(&mut self, interrupted: bool) {
        self.command.end(interrupted);
    }
}

impl PathPlannerAuto {
    /// Loads `pathplanner/autos/<auto_name>.auto`. `mirror` mirrors every path to the other side
    /// of the current alliance. A file that cannot be loaded is reported to the driver station
    /// and gives an auto that does nothing.
    ///
    /// # Panics
    /// When [`AutoBuilder`] is not configured.
    pub fn new(auto_name: &str, mirror: bool) -> Self {
        assert!(
            AutoBuilder::is_configured(),
            "AutoBuilder was not configured before attempting to load a PathPlannerAuto from file"
        );

        let (auto_command, starting_pose) = match load_auto(auto_name, mirror) {
            Ok(loaded) => loaded,
            Err(error) => {
                wpilib::driver_station::report_error(&format!(
                    "Failed to load auto: {auto_name}. {error}"
                ));
                (Commands::none(), None)
            }
        };

        let requirements = auto_command.get_requirements().to_vec();
        let command = Command::builder(
            auto_name,
            AutoBody {
                command: auto_command,
            },
        )
        .requires(&requirements)
        .build();
        Self {
            command,
            starting_pose,
        }
    }

    /// The command that runs the auto.
    pub fn command(&self) -> &Command {
        &self.command
    }

    pub fn into_command(self) -> Command {
        self.command
    }

    /// The start pose relative to a blue origin. `None` for an auto with no paths.
    pub fn starting_pose(&self) -> Option<Pose2d> {
        self.starting_pose
    }

    /// Every path in the auto, depth first.
    pub fn get_path_group_from_auto_file(auto_name: &str) -> Result<Vec<Rc<PathPlannerPath>>> {
        let json = read_auto_file(auto_name)?;
        let node = command_node(&json)?;
        let mut names = Vec::new();
        node.path_names(&mut names);
        names
            .iter()
            .map(|name| PathPlannerPath::from_path_file(name))
            .collect()
    }
}

fn read_auto_file(auto_name: &str) -> Result<Value> {
    let file = deploy_directory()
        .join("pathplanner/autos")
        .join(format!("{auto_name}.auto"));
    let text = std::fs::read_to_string(file)?;
    Ok(serde_json::from_str(&text)?)
}

fn command_node(json: &Value) -> Result<CommandNode> {
    if json.get("choreoAuto").and_then(Value::as_bool) == Some(true) {
        return Err(Error::Unsupported("Choreo autos"));
    }
    let command = json
        .get("command")
        .ok_or_else(|| Error::Invalid("auto file has no command".to_string()))?;
    CommandNode::from_json(command)
}

fn load_auto(auto_name: &str, mirror: bool) -> Result<(Command, Option<Pose2d>)> {
    let json = read_auto_file(auto_name)?;

    let version = match json.get("version") {
        Some(Value::String(s)) => s.clone(),
        Some(other) => other.to_string(),
        None => String::new(),
    };
    if version.split('.').next() != Some("2025") {
        return Err(Error::Version {
            found: version,
            file: format!("{auto_name}.auto"),
        });
    }

    let node = command_node(&json)?;
    let command = node.build(mirror)?;
    let reset_odom = json.get("resetOdom").and_then(Value::as_bool) == Some(true);

    let mut names = Vec::new();
    node.path_names(&mut names);
    let starting_pose = match names.first() {
        Some(first) => {
            let mut path0 = PathPlannerPath::from_path_file(first)?;
            if mirror {
                path0 = Rc::new(path0.mirror_path());
            }
            Some(if AutoBuilder::is_holonomic() {
                let ideal = path0.ideal_starting_state().ok_or_else(|| {
                    Error::Invalid(format!("path {first} has no ideal starting state"))
                })?;
                Pose2d::new(path0.point(0).position, ideal.rotation)
            } else {
                path0.get_starting_differential_pose()
            })
        }
        None => None,
    };

    let command = match (reset_odom, starting_pose) {
        (true, Some(pose)) => Commands::sequence([AutoBuilder::reset_odom(pose), command]),
        _ => command,
    };
    Ok((command, starting_pose))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::PidConstants;
    use crate::controller::PpHolonomicDriveController;
    use crate::util::{TestDeploy, robot_config, use_robot_deploy_folder};
    use std::cell::RefCell;
    use wpimath::kinematics::ChassisSpeeds;

    fn configure() {
        AutoBuilder::reset_for_testing();
        AutoBuilder::configure(
            Rc::new(|| Pose2d::ZERO),
            Rc::new(|_| {}),
            Rc::new(ChassisSpeeds::default),
            Rc::new(|_, _| {}),
            Rc::new(RefCell::new(PpHolonomicDriveController::new(
                PidConstants::new(1.0, 0.0, 0.0),
                PidConstants::new(1.0, 0.0, 0.0),
            ))),
            robot_config(),
            Rc::new(|| false),
            &[],
        );
    }

    /// The shape of an auto as numbers, as the Java fixture writes it. A command with children
    /// is its code, its child count, then the children. A wait adds its seconds. The reset
    /// odometry step is an instant command in front of the auto.
    fn describe(node: &CommandNode, reset_odom: bool, out: &mut Vec<f64>) {
        fn node_codes(node: &CommandNode, out: &mut Vec<f64>) {
            let group = |code: f64, children: &[CommandNode], out: &mut Vec<f64>| {
                out.extend([code, children.len() as f64]);
                for child in children {
                    node_codes(child, out);
                }
            };
            match node {
                CommandNode::None => out.push(0.0),
                CommandNode::Path(_) => out.push(1.0),
                CommandNode::Sequential(children) => group(2.0, children, out),
                CommandNode::Parallel(children) => group(3.0, children, out),
                CommandNode::Race(children) => group(4.0, children, out),
                CommandNode::Deadline(children) => group(5.0, children, out),
                CommandNode::Wait(seconds) => out.extend([6.0, *seconds]),
                CommandNode::Named(_) => out.push(7.0),
            }
        }
        if reset_odom {
            out.extend([2.0, 2.0, 0.0]);
        }
        node_codes(node, out);
    }

    #[test]
    fn autos_match_java() {
        use_robot_deploy_folder();
        configure();
        golden::check("pathplanner_autos", 1e-9, |op, _| {
            let (head, suffix) = op.split_once('|')?;
            if head == "names" {
                assert_eq!(AutoBuilder::get_all_auto_names().join(","), suffix);
                return Some(Vec::new());
            }

            let mut parts = head.splitn(3, ':');
            let (_, name, mirror) = (parts.next()?, parts.next()?, parts.next()? == "1");
            let paths: Vec<String> = PathPlannerAuto::get_path_group_from_auto_file(name)
                .unwrap()
                .iter()
                .map(|p| p.name.clone())
                .collect();
            assert_eq!(paths.join(","), suffix, "paths of {name}");

            let auto = PathPlannerAuto::new(name, mirror);
            let mut out = match auto.starting_pose() {
                Some(p) => vec![1.0, p.x(), p.y(), p.rotation.radians()],
                None => vec![0.0; 4],
            };
            let json = read_auto_file(name).unwrap();
            let reset_odom = json.get("resetOdom").and_then(Value::as_bool) == Some(true);
            describe(&command_node(&json).unwrap(), reset_odom, &mut out);
            Some(out)
        });
    }

    #[test]
    fn a_file_with_the_wrong_version_does_nothing() {
        let deploy = TestDeploy::new("auto-version");
        deploy.write(
            "autos/old.auto",
            r#"{"version": "2024.0", "command": {"type": "sequential", "data": {"commands": []}}}"#,
        );
        configure();
        let auto = PathPlannerAuto::new("old", false);
        assert!(auto.starting_pose().is_none());
        assert_eq!(auto.command().get_name(), "old");
    }

    #[test]
    fn load_errors_name_the_problem() {
        let deploy = TestDeploy::new("auto-errors");
        deploy.write("autos/broken.auto", "{ not json");
        deploy.write(
            "autos/choreo.auto",
            r#"{"version": "2025.0", "choreoAuto": true, "command": {"type": "sequential", "data": {"commands": []}}}"#,
        );
        deploy.write("autos/nocommand.auto", r#"{"version": "2025.0"}"#);

        assert!(matches!(
            PathPlannerAuto::get_path_group_from_auto_file("missing"),
            Err(Error::Io(_))
        ));
        assert!(matches!(
            PathPlannerAuto::get_path_group_from_auto_file("broken"),
            Err(Error::Parse(_))
        ));
        assert!(matches!(
            PathPlannerAuto::get_path_group_from_auto_file("choreo"),
            Err(Error::Unsupported(_))
        ));
        assert!(matches!(
            PathPlannerAuto::get_path_group_from_auto_file("nocommand"),
            Err(Error::Invalid(_))
        ));
    }

    #[test]
    fn an_auto_without_paths_has_no_starting_pose() {
        let deploy = TestDeploy::new("auto-no-paths");
        deploy.write(
            "autos/waits.auto",
            r#"{"version": "2025.0", "resetOdom": true, "command": {"type": "wait", "data": {"waitTime": 1.5}}}"#,
        );
        configure();
        let auto = PathPlannerAuto::new("waits", false);
        assert!(auto.starting_pose().is_none());
        assert!(
            PathPlannerAuto::get_path_group_from_auto_file("waits")
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn path_group_follows_the_tree_depth_first() {
        let deploy = TestDeploy::new("auto-order");
        deploy.write(
            "autos/mixed.auto",
            r#"{"version": "2025.0", "command": {"type": "sequential", "data": {"commands": [
                {"type": "path", "data": {"pathName": "b"}},
                {"type": "parallel", "data": {"commands": [
                    {"type": "path", "data": {"pathName": "c"}},
                    {"type": "deadline", "data": {"commands": [
                        {"type": "path", "data": {"pathName": "a"}}]}}]}},
                {"type": "path", "data": {"pathName": "d"}}]}}}"#,
        );
        let json = read_auto_file("mixed").unwrap();
        let mut names = Vec::new();
        command_node(&json).unwrap().path_names(&mut names);
        assert_eq!(names, ["b", "c", "a", "d"]);
    }
}

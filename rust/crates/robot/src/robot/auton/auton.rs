use crate::robot::subsystems::super_structure::{SuperStructure, WantedSuperState};
use crate::spectrum_lib::framework::spectrum_state::SpectrumState;
use commands::{Command, CommandScheduler, Commands, Trigger};
use pathplanner::{EventTrigger, PathPlannerAuto};
use std::cell::Cell;
use std::rc::Rc;
use telemetry::telemetry;
use wpilib::dashboard::{SendableChooser, smart_dashboard};
use wpilib::{driver_station, timer};

// Java's static fields. Built on first use and handed out as clones.
thread_local! {
    static AUTON_LAUNCHING: SpectrumState = SpectrumState::new("AutonLaunching");
    static AUTON_INTAKE: Trigger = EventTrigger::new("intake");
    static AUTON_SHOT_PREP: Trigger = EventTrigger::new("shotPrep");
    static AUTON_CLEAR_STATE: Trigger = EventTrigger::new("clearState");
    static AUTON_UNJAM: Trigger = EventTrigger::new("unjam");
    static AUTON_POSE_UPDATE: Trigger = EventTrigger::new("poseUpdate");
}

pub fn auton_launching() -> SpectrumState {
    AUTON_LAUNCHING.with(SpectrumState::clone)
}

pub fn auton_intake() -> Trigger {
    AUTON_INTAKE.with(Trigger::clone)
}

pub fn auton_shot_prep() -> Trigger {
    AUTON_SHOT_PREP.with(Trigger::clone)
}

pub fn auton_clear_state() -> Trigger {
    AUTON_CLEAR_STATE.with(Trigger::clone)
}

pub fn auton_unjam() -> Trigger {
    AUTON_UNJAM.with(Trigger::clone)
}

pub fn auton_pose_update() -> Trigger {
    AUTON_POSE_UPDATE.with(Trigger::clone)
}

const SECOND_MAN_DELAY: f64 = 1.0;
const OPTIONAL_DELAY: f64 = 1.0;

type StateCommand = Rc<dyn Fn(WantedSuperState) -> Command>;

pub struct Auton {
    launching: SpectrumState,
    state_command: StateCommand,
    path_chooser: SendableChooser<Command>,
    auto_message_printed: Cell<bool>,
    auton_start: Cell<f64>,
}

impl Auton {
    pub fn new(robot_super_structure: Rc<SuperStructure>) -> Auton {
        let auton = Self::with_parts(
            Rc::new(move |state| robot_super_structure.set_state_command(state)),
            auton_launching(),
        );
        auton.setup_selectors();
        telemetry::print("Auton Subsystem Initialized");
        auton
    }

    /// The only things Java's `Auton` takes from `SuperStructure`, so tests can run without
    /// hardware.
    fn with_parts(state_command: StateCommand, launching: SpectrumState) -> Auton {
        Auton {
            launching,
            state_command,
            path_chooser: SendableChooser::new(),
            auto_message_printed: Cell::new(true),
            auton_start: Cell::new(0.0),
        }
    }

    pub fn setup_selectors(&self) {
        self.add_options();
        smart_dashboard::put_data("Auto Chooser", &self.path_chooser);
    }

    fn add_options(&self) {
        self.path_chooser
            .set_default_option("Do Nothing", self.do_nothing());
        for (name, command) in self.options() {
            self.path_chooser.add_option(name, command);
        }
    }

    /// Every option after the default, in dashboard order.
    fn options(&self) -> Vec<(&'static str, Command)> {
        vec![
            ("TBTB Left", self.tbtb(false)),
            ("TBTB Right", self.tbtb(true)),
            ("TBTT Left", self.tbtt(false)),
            ("TBTT Right", self.tbtt(true)),
            ("TTTT Left", self.tttt(false)),
            ("TTTT Right", self.tttt(true)),
            ("BBBB Left", self.bbbb(false)),
            ("BBBB Right", self.bbbb(true)),
            ("Optional - Left TBT", self.optional_tbt(false)),
            ("Optional - Right TBT", self.optional_tbt(true)),
            ("Optional - Left BBB", self.optional_bbb(false)),
            ("Optional - Right BBB", self.optional_bbb(true)),
            ("2nd Man - TBTB Left", self.second_man_tbtb(false)),
            ("2nd Man - TBTB Right", self.second_man_tbtb(true)),
            ("2nd Man - BBD Left", self.second_man_bbd(false)),
            ("2nd Man - BBD Right", self.second_man_bbd(true)),
        ]
    }

    pub fn init(&self) {
        let auton_command = self.get_autonomous_command();
        CommandScheduler::get_instance().schedule(&auton_command);
        self.start_auton_timer();
    }

    pub fn exit(&self) {
        self.print_auto_duration();
    }

    pub fn do_nothing(&self) -> Command {
        Commands::print("Do Nothing Auto ran").with_name("Do Nothing")
    }

    pub fn launch(&self) -> Command {
        Commands::sequence([
            self.launching.set_true(),
            (self.state_command)(WantedSuperState::LaunchWithSqueeze),
            Commands::wait_seconds(2.5),
            (self.state_command)(WantedSuperState::Idle),
            self.launching.set_false(),
        ])
        .with_name("Auton.launch")
    }

    pub fn second_man_tbtb(&self, mirrored: bool) -> Command {
        Commands::sequence([
            Commands::wait_seconds(SECOND_MAN_DELAY),
            spectrum_auton("2nd-TBTB 1", mirrored),
            self.launch(),
            spectrum_auton("2nd-TBTB 2", mirrored),
        ])
        .with_name(full_name("2nd-TBTB Full", mirrored))
    }

    pub fn second_man_bbd(&self, mirrored: bool) -> Command {
        Commands::sequence([
            Commands::wait_seconds(SECOND_MAN_DELAY),
            spectrum_auton("2nd-BBD 1", mirrored),
            self.launch(),
            spectrum_auton("2nd-BBD 2", mirrored),
        ])
        .with_name(full_name("2nd-BBD Full", mirrored))
    }

    pub fn optional_tbt(&self, mirrored: bool) -> Command {
        Commands::sequence([
            spectrum_auton("Option TBT 1", mirrored),
            Commands::wait_seconds(OPTIONAL_DELAY),
            spectrum_auton("Option TBT 2", mirrored),
            self.launch(),
            spectrum_auton("Option TBT 3", mirrored),
        ])
        .with_name(full_name("Option TBT Full", mirrored))
    }

    pub fn optional_bbb(&self, mirrored: bool) -> Command {
        Commands::sequence([
            spectrum_auton("Option BBB 1", mirrored),
            Commands::wait_seconds(OPTIONAL_DELAY),
            spectrum_auton("Option BBB 2", mirrored),
            self.launch(),
            spectrum_auton("Option BBB 3", mirrored),
        ])
        .with_name(full_name("Option BBB Full", mirrored))
    }

    pub fn tbtb(&self, mirrored: bool) -> Command {
        Commands::sequence([
            spectrum_auton("TBTB 1", mirrored),
            self.launch(),
            spectrum_auton("TBTB 2", mirrored),
            self.launch(),
            spectrum_auton("TBTB 3", mirrored),
        ])
        .with_name(full_name("TBTB Full", mirrored))
    }

    pub fn tbtt(&self, mirrored: bool) -> Command {
        Commands::sequence([
            spectrum_auton("TBTT 1", mirrored),
            self.launch(),
            spectrum_auton("TBTT 2", mirrored),
            self.launch(),
            spectrum_auton("TBTT 3", mirrored),
        ])
        .with_name(full_name("TBTT Full", mirrored))
    }

    pub fn tttt(&self, mirrored: bool) -> Command {
        Commands::sequence([
            spectrum_auton("TTTT 1", mirrored),
            self.launch(),
            spectrum_auton("TTTT 2", mirrored),
            self.launch(),
            spectrum_auton("TTTT 3", mirrored),
        ])
        .with_name(full_name("TTTT Full", mirrored))
    }

    pub fn bbbb(&self, mirrored: bool) -> Command {
        Commands::sequence([
            spectrum_auton("BBBB 1", mirrored),
            self.launch(),
            spectrum_auton("BBBB 2", mirrored),
            self.launch(),
        ])
        .with_name(full_name("BBBB Full", mirrored))
    }

    /// Never returns null. Falls back to a command that prints a warning.
    pub fn get_autonomous_command(&self) -> Command {
        self.path_chooser
            .get_selected()
            .unwrap_or_else(|| Commands::print("*** AUTON COMMAND IS NULL ***"))
    }

    pub fn start_auton_timer(&self) {
        self.auton_start.set(timer::fpga_timestamp());
        self.auto_message_printed.set(false);
    }

    /// Prints the elapsed auton time at auton exit, calling the run cancelled if it ended early.
    /// Based on 6328 code.
    pub fn print_auto_duration(&self) {
        let auto_command = self.get_autonomous_command();
        if !auto_command.is_scheduled() && !self.auto_message_printed.get() {
            let elapsed = timer::fpga_timestamp() - self.auton_start.get();
            if driver_station::is_autonomous_enabled() {
                telemetry::print(&format!("*** Auton finished in {elapsed:.2} secs ***"));
            } else {
                telemetry::print(&format!("*** Auton CANCELLED in {elapsed:.2} secs ***"));
            }
            self.auto_message_printed.set(true);
        }
    }
}

/// The " - Left" / " - Right" suffix is what the field visualizer reads to mirror.
fn full_name(base: &str, mirrored: bool) -> String {
    format!("{base} - {}", if mirrored { "Right" } else { "Left" })
}

/// Java's `SpectrumAuton(String, boolean)`.
fn spectrum_auton(auto_name: &str, mirrored: bool) -> Command {
    let auto_command = PathPlannerAuto::new(auto_name, mirrored).into_command();
    Commands::wait_seconds(0.01)
        .and_then(auto_command)
        .with_name(auto_name)
}

#[cfg(test)]
mod tests {
    use super::*;
    use pathplanner::{
        AutoBuilder, PidConstants, PpHolonomicDriveController, RobotConfig, set_deploy_directory,
    };
    use std::cell::RefCell;
    use std::path::PathBuf;
    use wpimath::geometry::Pose2d;
    use wpimath::kinematics::ChassisSpeeds;

    fn use_robot_deploy_folder() -> PathBuf {
        let deploy = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../src/main/deploy");
        set_deploy_directory(Some(deploy.clone()));
        deploy
    }

    fn auton() -> Auton {
        let deploy = use_robot_deploy_folder();
        let settings = std::fs::read_to_string(deploy.join("pathplanner/settings.json")).unwrap();
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
            RobotConfig::from_gui_settings_json(&settings).unwrap(),
            Rc::new(|| false),
            &[],
        );
        Auton::with_parts(
            Rc::new(|state| Commands::print(format!("{state:?}"))),
            SpectrumState::without_alert("AutonLaunching"),
        )
    }

    #[test]
    fn chooser_options_and_command_names_match_java() {
        let auton = auton();
        let listed: Vec<(&str, String)> = auton
            .options()
            .into_iter()
            .map(|(name, command)| (name, command.get_name()))
            .collect();
        let expected = [
            ("TBTB Left", "TBTB Full - Left"),
            ("TBTB Right", "TBTB Full - Right"),
            ("TBTT Left", "TBTT Full - Left"),
            ("TBTT Right", "TBTT Full - Right"),
            ("TTTT Left", "TTTT Full - Left"),
            ("TTTT Right", "TTTT Full - Right"),
            ("BBBB Left", "BBBB Full - Left"),
            ("BBBB Right", "BBBB Full - Right"),
            ("Optional - Left TBT", "Option TBT Full - Left"),
            ("Optional - Right TBT", "Option TBT Full - Right"),
            ("Optional - Left BBB", "Option BBB Full - Left"),
            ("Optional - Right BBB", "Option BBB Full - Right"),
            ("2nd Man - TBTB Left", "2nd-TBTB Full - Left"),
            ("2nd Man - TBTB Right", "2nd-TBTB Full - Right"),
            ("2nd Man - BBD Left", "2nd-BBD Full - Left"),
            ("2nd Man - BBD Right", "2nd-BBD Full - Right"),
        ];
        let expected: Vec<(&str, String)> = expected
            .into_iter()
            .map(|(option, command)| (option, command.to_owned()))
            .collect();
        assert_eq!(listed, expected);
    }

    #[test]
    fn do_nothing_is_the_default_selection() {
        let auton = auton();
        auton.add_options();
        assert_eq!(auton.get_autonomous_command().get_name(), "Do Nothing");
    }

    #[test]
    fn an_empty_chooser_falls_back_to_a_warning_command() {
        let auton = auton();
        assert_eq!(
            auton.get_autonomous_command().get_name(),
            Commands::print("*** AUTON COMMAND IS NULL ***").get_name()
        );
    }

    #[test]
    fn launch_is_named_for_the_dashboard() {
        assert_eq!(auton().launch().get_name(), "Auton.launch");
    }

    #[test]
    fn side_suffix_is_what_the_field_visualizer_reads() {
        assert_eq!(full_name("TBTB Full", false), "TBTB Full - Left");
        assert_eq!(full_name("TBTB Full", true), "TBTB Full - Right");
    }
}

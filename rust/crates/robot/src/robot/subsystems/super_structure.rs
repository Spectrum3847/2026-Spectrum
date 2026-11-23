//! Turns one wanted state for the whole robot into a wanted state for each subsystem.

use super::fuel_intake::{self, FuelIntake};
use super::hood::{self, Hood};
use super::indexer_bed::{self, IndexerBed};
use super::indexer_tower::{self, IndexerTower};
use super::intake_extension::{self, IntakeExtension};
use super::launcher::{self, Launcher};
use super::swerve::swerve::{self as swerve_state, Swerve};
use crate::spectrum_lib::util::util;
use commands::{
    Command, CommandScheduler, Commands, Subsystem, SubsystemBase, SubsystemId, Trigger,
};
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};
use telemetry::{EnumName, telemetry};
use wpilib::timer::Timer;

const REGULAR_TELEOP_TRANSLATION_COEFFICIENT: f64 = 1.0;
const SHOOTING_TELEOP_TRANSLATION_COEFFICIENT: f64 = 0.1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WantedSuperState {
    Idle,
    IntakeFuel,
    TrackTarget,
    LaunchWithSqueeze,
    LaunchWithSqueezeWithNoDelay,
    LaunchWithoutSqueeze,
    LaunchWithBrake,
    AutonTrackTarget,
    AutonIntakeFuel,
    Unjam,
    Eject,
    ForceHome,
}

impl EnumName for WantedSuperState {
    fn name(&self) -> &'static str {
        match self {
            Self::Idle => "IDLE",
            Self::IntakeFuel => "INTAKE_FUEL",
            Self::TrackTarget => "TRACK_TARGET",
            Self::LaunchWithSqueeze => "LAUNCH_WITH_SQUEEZE",
            Self::LaunchWithSqueezeWithNoDelay => "LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY",
            Self::LaunchWithoutSqueeze => "LAUNCH_WITHOUT_SQUEEZE",
            Self::LaunchWithBrake => "LAUNCH_WITH_BRAKE",
            Self::AutonTrackTarget => "AUTON_TRACK_TARGET",
            Self::AutonIntakeFuel => "AUTON_INTAKE_FUEL",
            Self::Unjam => "UNJAM",
            Self::Eject => "EJECT",
            Self::ForceHome => "FORCE_HOME",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurrentSuperState {
    Idle,
    IntakeFuel,
    TrackTarget,
    LaunchWithSqueeze,
    LaunchWithSqueezeWithNoDelay,
    LaunchWithoutSqueeze,
    LaunchWithBrake,
    AutonIdle,
    AutonTrackTarget,
    AutonIntakeFuel,
    Unjam,
    Eject,
    ForceHome,
}

impl EnumName for CurrentSuperState {
    fn name(&self) -> &'static str {
        match self {
            Self::Idle => "IDLE",
            Self::IntakeFuel => "INTAKE_FUEL",
            Self::TrackTarget => "TRACK_TARGET",
            Self::LaunchWithSqueeze => "LAUNCH_WITH_SQUEEZE",
            Self::LaunchWithSqueezeWithNoDelay => "LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY",
            Self::LaunchWithoutSqueeze => "LAUNCH_WITHOUT_SQUEEZE",
            Self::LaunchWithBrake => "LAUNCH_WITH_BRAKE",
            Self::AutonIdle => "AUTON_IDLE",
            Self::AutonTrackTarget => "AUTON_TRACK_TARGET",
            Self::AutonIntakeFuel => "AUTON_INTAKE_FUEL",
            Self::Unjam => "UNJAM",
            Self::Eject => "EJECT",
            Self::ForceHome => "FORCE_HOME",
        }
    }
}

const SECONDS_TO_SQUEEZE: f64 = 1.0;

fn is_squeeze_state(state: CurrentSuperState) -> bool {
    state == CurrentSuperState::LaunchWithSqueeze
}

/// `idle_is_auton` is only asked for `Idle`, where auton and disabled idle differently.
pub fn handle_state_transitions(
    wanted_super_state: WantedSuperState,
    idle_is_auton: impl FnOnce() -> bool,
) -> CurrentSuperState {
    match wanted_super_state {
        WantedSuperState::Idle => {
            if idle_is_auton() {
                CurrentSuperState::AutonIdle
            } else {
                CurrentSuperState::Idle
            }
        }
        WantedSuperState::IntakeFuel => CurrentSuperState::IntakeFuel,
        WantedSuperState::TrackTarget => CurrentSuperState::TrackTarget,
        WantedSuperState::LaunchWithSqueeze => CurrentSuperState::LaunchWithSqueeze,
        WantedSuperState::LaunchWithSqueezeWithNoDelay => {
            CurrentSuperState::LaunchWithSqueezeWithNoDelay
        }
        WantedSuperState::LaunchWithoutSqueeze => CurrentSuperState::LaunchWithoutSqueeze,
        WantedSuperState::LaunchWithBrake => CurrentSuperState::LaunchWithBrake,
        WantedSuperState::AutonTrackTarget => CurrentSuperState::AutonTrackTarget,
        WantedSuperState::AutonIntakeFuel => CurrentSuperState::AutonIntakeFuel,
        WantedSuperState::Unjam => CurrentSuperState::Unjam,
        WantedSuperState::Eject => CurrentSuperState::Eject,
        WantedSuperState::ForceHome => CurrentSuperState::ForceHome,
    }
}

/// The wanted state of every subsystem for one super state. `swerve` and
/// `teleop_velocity_coefficient` are `None` where Java leaves swerve alone.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Plan {
    pub swerve: Option<swerve_state::WantedState>,
    pub teleop_velocity_coefficient: Option<f64>,
    pub fuel_intake: fuel_intake::WantedState,
    pub indexer_tower: indexer_tower::WantedState,
    pub indexer_bed: indexer_bed::WantedState,
    pub intake_extension: intake_extension::WantedState,
    pub launcher: launcher::WantedState,
    pub hood: hood::WantedState,
}

/// `squeeze_elapsed` is whether the squeeze timer has run out. Only `LaunchWithSqueeze` reads
/// it.
pub fn plan_for(state: CurrentSuperState, squeeze_elapsed: bool) -> Plan {
    use swerve_state::WantedState as Swerve;
    let regular = REGULAR_TELEOP_TRANSLATION_COEFFICIENT;
    let shooting = SHOOTING_TELEOP_TRANSLATION_COEFFICIENT;
    let base = Plan {
        swerve: Some(Swerve::TeleopDrive),
        teleop_velocity_coefficient: Some(regular),
        fuel_intake: fuel_intake::WantedState::Neutral,
        indexer_tower: indexer_tower::WantedState::Off,
        indexer_bed: indexer_bed::WantedState::Off,
        intake_extension: intake_extension::WantedState::Stopped,
        launcher: launcher::WantedState::IdlePrep,
        hood: hood::WantedState::Home,
    };
    let shooting_base = Plan {
        swerve: Some(Swerve::PilotAimAtTarget),
        teleop_velocity_coefficient: Some(shooting),
        fuel_intake: fuel_intake::WantedState::Intake,
        indexer_tower: indexer_tower::WantedState::IndexMax,
        indexer_bed: indexer_bed::WantedState::IndexMax,
        launcher: launcher::WantedState::AimAtTarget,
        hood: hood::WantedState::AimAtTarget,
        ..base
    };
    match state {
        CurrentSuperState::Idle => base,
        CurrentSuperState::IntakeFuel => Plan {
            fuel_intake: fuel_intake::WantedState::Intake,
            indexer_bed: indexer_bed::WantedState::SlowIndex,
            intake_extension: intake_extension::WantedState::FullExtend,
            ..base
        },
        CurrentSuperState::TrackTarget => Plan {
            swerve: Some(Swerve::PilotAimAtTarget),
            intake_extension: intake_extension::WantedState::ConditionalExtend,
            launcher: launcher::WantedState::AimAtTarget,
            hood: hood::WantedState::AimAtTarget,
            ..base
        },
        CurrentSuperState::LaunchWithSqueeze => Plan {
            intake_extension: if squeeze_elapsed {
                intake_extension::WantedState::SlowClose
            } else {
                intake_extension::WantedState::FullExtend
            },
            ..shooting_base
        },
        CurrentSuperState::LaunchWithSqueezeWithNoDelay => Plan {
            intake_extension: intake_extension::WantedState::SlowClose,
            ..shooting_base
        },
        CurrentSuperState::LaunchWithoutSqueeze => Plan {
            intake_extension: intake_extension::WantedState::FullExtend,
            ..shooting_base
        },
        CurrentSuperState::LaunchWithBrake => Plan {
            swerve: Some(Swerve::XBrake),
            teleop_velocity_coefficient: None,
            fuel_intake: fuel_intake::WantedState::SlowIntake,
            indexer_tower: indexer_tower::WantedState::IndexMax,
            indexer_bed: indexer_bed::WantedState::IndexMax,
            intake_extension: intake_extension::WantedState::FullExtend,
            launcher: launcher::WantedState::AimAtTarget,
            hood: hood::WantedState::AimAtTarget,
        },
        CurrentSuperState::AutonIdle => Plan {
            swerve: Some(Swerve::Idle),
            teleop_velocity_coefficient: None,
            ..base
        },
        CurrentSuperState::AutonIntakeFuel => Plan {
            swerve: None,
            teleop_velocity_coefficient: None,
            fuel_intake: fuel_intake::WantedState::Intake,
            indexer_bed: indexer_bed::WantedState::SlowIndex,
            intake_extension: intake_extension::WantedState::FullExtend,
            ..base
        },
        CurrentSuperState::AutonTrackTarget => Plan {
            swerve: None,
            teleop_velocity_coefficient: None,
            intake_extension: intake_extension::WantedState::ConditionalExtend,
            launcher: launcher::WantedState::AimAtTarget,
            hood: hood::WantedState::AimAtTarget,
            ..base
        },
        CurrentSuperState::Unjam => Plan {
            indexer_tower: indexer_tower::WantedState::Unjam,
            indexer_bed: indexer_bed::WantedState::Unjam,
            intake_extension: intake_extension::WantedState::ConditionalExtend,
            launcher: launcher::WantedState::Off,
            ..base
        },
        CurrentSuperState::Eject => Plan {
            fuel_intake: fuel_intake::WantedState::Outtake,
            indexer_tower: indexer_tower::WantedState::Unjam,
            indexer_bed: indexer_bed::WantedState::Unjam,
            intake_extension: intake_extension::WantedState::ConditionalExtend,
            launcher: launcher::WantedState::Off,
            ..base
        },
        CurrentSuperState::ForceHome => Plan {
            intake_extension: intake_extension::WantedState::FullRetract,
            launcher: launcher::WantedState::Off,
            ..base
        },
    }
}

/// Restarts the squeeze timer on the first loop of a squeeze, not on every loop.
fn update_squeeze_timer(
    timer: &mut Timer,
    current_super_state: CurrentSuperState,
    previous_super_state: CurrentSuperState,
) {
    if is_squeeze_state(current_super_state) && !is_squeeze_state(previous_super_state) {
        timer.restart();
    }
}

pub struct SuperStructure {
    me: Weak<SuperStructure>,
    base: SubsystemBase,

    pub swerve: Rc<Swerve>,
    pub fuel_intake: Rc<FuelIntake>,
    pub intake_extension: Rc<IntakeExtension>,
    pub indexer_tower: Rc<IndexerTower>,
    pub indexer_bed: Rc<IndexerBed>,
    pub launcher: Rc<Launcher>,
    pub hood: Rc<Hood>,

    wanted_super_state: Cell<WantedSuperState>,
    current_super_state: Cell<CurrentSuperState>,
    previous_super_state: Cell<CurrentSuperState>,

    intake_squeeze_timer: RefCell<Timer>,
}

impl SuperStructure {
    pub fn new(
        swerve: Rc<Swerve>,
        fuel_intake: Rc<FuelIntake>,
        intake_extension: Rc<IntakeExtension>,
        indexer_tower: Rc<IndexerTower>,
        indexer_bed: Rc<IndexerBed>,
        launcher: Rc<Launcher>,
        hood: Rc<Hood>,
    ) -> Rc<SuperStructure> {
        let super_structure = Rc::new_cyclic(|me| SuperStructure {
            me: me.clone(),
            base: SubsystemBase::new("SuperStructure"),
            swerve,
            fuel_intake,
            intake_extension,
            indexer_tower,
            indexer_bed,
            launcher,
            hood,
            wanted_super_state: Cell::new(WantedSuperState::Idle),
            current_super_state: Cell::new(CurrentSuperState::Idle),
            previous_super_state: Cell::new(CurrentSuperState::Idle),
            intake_squeeze_timer: RefCell::new(Timer::new()),
        });
        CommandScheduler::get_instance().register_subsystem(super_structure.clone());
        super_structure
    }

    fn rc(&self) -> Rc<SuperStructure> {
        self.me
            .upgrade()
            .expect("SuperStructure is only built through SuperStructure::new, which returns an Rc")
    }

    pub fn get_wanted_super_state(&self) -> WantedSuperState {
        self.wanted_super_state.get()
    }

    pub fn get_current_super_state(&self) -> CurrentSuperState {
        self.current_super_state.get()
    }

    fn apply_states(&self) {
        let state = self.current_super_state.get();
        let squeeze_elapsed = state == CurrentSuperState::LaunchWithSqueeze
            && self
                .intake_squeeze_timer
                .borrow()
                .has_elapsed(SECONDS_TO_SQUEEZE);
        if squeeze_elapsed {
            self.intake_squeeze_timer.borrow_mut().stop();
        }
        let plan = plan_for(state, squeeze_elapsed);

        if let Some(wanted) = plan.swerve {
            self.swerve.set_wanted_state(wanted);
        }
        if let Some(coefficient) = plan.teleop_velocity_coefficient {
            self.swerve.set_teleop_velocity_coefficient(coefficient);
        }
        self.fuel_intake.set_wanted_state(plan.fuel_intake);
        self.indexer_tower.set_wanted_state(plan.indexer_tower);
        self.indexer_bed.set_wanted_state(plan.indexer_bed);
        self.intake_extension
            .set_wanted_state(plan.intake_extension);
        self.launcher.set_wanted_state(plan.launcher);
        self.hood.set_wanted_state(plan.hood);
    }

    pub fn coast_mechanisms(&self) -> Command {
        let (intake_extension, hood) = (self.intake_extension.clone(), self.hood.clone());
        Commands::run_once(
            move || {
                intake_extension.set_brake_mode(false);
                hood.set_brake_mode(false);
            },
            &[],
        )
        .ignoring_disable(true)
    }

    pub fn brake_mechanisms(&self) -> Command {
        let (intake_extension, hood) = (self.intake_extension.clone(), self.hood.clone());
        Commands::run_once(
            move || {
                intake_extension.set_brake_mode(true);
                hood.set_brake_mode(true);
            },
            &[],
        )
        .ignoring_disable(true)
    }

    // Allocation-free, so these are safe to call in per-loop code.
    pub fn is_robot_in_neutral_zone(&self) -> bool {
        self.swerve.is_in_neutral_zone()
    }

    pub fn is_robot_in_enemy_zone(&self) -> bool {
        self.swerve.is_in_enemy_alliance_zone()
    }

    pub fn is_robot_in_feed_zone(&self) -> bool {
        self.is_robot_in_enemy_zone() || self.is_robot_in_neutral_zone()
    }

    pub fn is_robot_in_score_zone(&self) -> bool {
        !self.is_robot_in_feed_zone()
    }

    // Each allocates a Trigger, so call these at binding time rather than per loop.
    pub fn robot_in_neutral_zone(&self) -> Trigger {
        let me = self.rc();
        Trigger::new(move || me.is_robot_in_neutral_zone())
    }

    pub fn robot_in_enemy_zone(&self) -> Trigger {
        let me = self.rc();
        Trigger::new(move || me.is_robot_in_enemy_zone())
    }

    pub fn robot_in_feed_zone(&self) -> Trigger {
        let me = self.rc();
        Trigger::new(move || me.is_robot_in_feed_zone())
    }

    pub fn robot_in_score_zone(&self) -> Trigger {
        let me = self.rc();
        Trigger::new(move || me.is_robot_in_score_zone())
    }

    pub fn set_wanted_super_state(&self, state: WantedSuperState) {
        self.wanted_super_state.set(state);
    }

    pub fn set_state_command(&self, state: WantedSuperState) -> Command {
        let me = self.rc();
        Commands::run_once(move || me.set_wanted_super_state(state), &[])
    }
}

impl Subsystem for SuperStructure {
    fn id(&self) -> SubsystemId {
        self.base.id()
    }

    fn name(&self) -> String {
        self.base.name().to_owned()
    }

    fn periodic(&self) {
        let current = handle_state_transitions(self.wanted_super_state.get(), || {
            util::auto_mode().get_as_boolean() || util::disabled().get_as_boolean()
        });
        self.current_super_state.set(current);

        update_squeeze_timer(
            &mut self.intake_squeeze_timer.borrow_mut(),
            current,
            self.previous_super_state.get(),
        );

        self.apply_states();

        self.previous_super_state.set(current);

        telemetry::log_dash(
            "SuperStructure/WantedSuperState",
            self.wanted_super_state.get().name(),
        );
        telemetry::log_dash("SuperStructure/CurrentSuperState", current.name());
        telemetry::log_dash_with_unit(
            "SuperStructure/IntakeSqueezeTimerElapsed",
            self.intake_squeeze_timer.borrow().get(),
            "seconds",
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};
    use swerve_state::WantedState as Swerve;

    static MICROS: AtomicU64 = AtomicU64::new(0);

    fn fake_clock() -> u64 {
        MICROS.load(Ordering::SeqCst)
    }

    const ALL_WANTED: [WantedSuperState; 12] = [
        WantedSuperState::Idle,
        WantedSuperState::IntakeFuel,
        WantedSuperState::TrackTarget,
        WantedSuperState::LaunchWithSqueeze,
        WantedSuperState::LaunchWithSqueezeWithNoDelay,
        WantedSuperState::LaunchWithoutSqueeze,
        WantedSuperState::LaunchWithBrake,
        WantedSuperState::AutonTrackTarget,
        WantedSuperState::AutonIntakeFuel,
        WantedSuperState::Unjam,
        WantedSuperState::Eject,
        WantedSuperState::ForceHome,
    ];

    #[test]
    fn every_wanted_state_but_idle_maps_to_the_current_state_of_the_same_name() {
        for wanted in ALL_WANTED {
            if wanted == WantedSuperState::Idle {
                continue;
            }
            let current = handle_state_transitions(wanted, || panic!("only IDLE asks"));
            assert_eq!(current.name(), wanted.name());
        }
    }

    #[test]
    fn idle_becomes_auton_idle_in_auton_or_while_disabled() {
        assert_eq!(
            handle_state_transitions(WantedSuperState::Idle, || true),
            CurrentSuperState::AutonIdle
        );
        assert_eq!(
            handle_state_transitions(WantedSuperState::Idle, || false),
            CurrentSuperState::Idle
        );
    }

    #[test]
    fn idle_spins_the_launcher_up_and_keeps_everything_else_still() {
        let plan = plan_for(CurrentSuperState::Idle, false);
        assert_eq!(plan.swerve, Some(Swerve::TeleopDrive));
        assert_eq!(plan.teleop_velocity_coefficient, Some(1.0));
        assert_eq!(plan.fuel_intake, fuel_intake::WantedState::Neutral);
        assert_eq!(plan.indexer_tower, indexer_tower::WantedState::Off);
        assert_eq!(plan.indexer_bed, indexer_bed::WantedState::Off);
        assert_eq!(
            plan.intake_extension,
            intake_extension::WantedState::Stopped
        );
        assert_eq!(plan.launcher, launcher::WantedState::IdlePrep);
        assert_eq!(plan.hood, hood::WantedState::Home);
    }

    #[test]
    fn intake_fuel_runs_the_intake_and_slow_bed_with_the_extension_out() {
        let plan = plan_for(CurrentSuperState::IntakeFuel, false);
        assert_eq!(plan.fuel_intake, fuel_intake::WantedState::Intake);
        assert_eq!(plan.indexer_bed, indexer_bed::WantedState::SlowIndex);
        assert_eq!(plan.indexer_tower, indexer_tower::WantedState::Off);
        assert_eq!(
            plan.intake_extension,
            intake_extension::WantedState::FullExtend
        );
        assert_eq!(plan.swerve, Some(Swerve::TeleopDrive));
    }

    #[test]
    fn track_target_aims_without_feeding() {
        let plan = plan_for(CurrentSuperState::TrackTarget, false);
        assert_eq!(plan.swerve, Some(Swerve::PilotAimAtTarget));
        assert_eq!(plan.teleop_velocity_coefficient, Some(1.0));
        assert_eq!(plan.launcher, launcher::WantedState::AimAtTarget);
        assert_eq!(plan.hood, hood::WantedState::AimAtTarget);
        assert_eq!(plan.indexer_tower, indexer_tower::WantedState::Off);
        assert_eq!(
            plan.intake_extension,
            intake_extension::WantedState::ConditionalExtend
        );
    }

    #[test]
    fn launching_slows_the_drive_and_feeds_at_full_rate() {
        for state in [
            CurrentSuperState::LaunchWithSqueeze,
            CurrentSuperState::LaunchWithSqueezeWithNoDelay,
            CurrentSuperState::LaunchWithoutSqueeze,
        ] {
            let plan = plan_for(state, false);
            assert_eq!(plan.swerve, Some(Swerve::PilotAimAtTarget), "{state:?}");
            assert_eq!(plan.teleop_velocity_coefficient, Some(0.1), "{state:?}");
            assert_eq!(
                plan.fuel_intake,
                fuel_intake::WantedState::Intake,
                "{state:?}"
            );
            assert_eq!(
                plan.indexer_tower,
                indexer_tower::WantedState::IndexMax,
                "{state:?}"
            );
            assert_eq!(
                plan.indexer_bed,
                indexer_bed::WantedState::IndexMax,
                "{state:?}"
            );
            assert_eq!(
                plan.launcher,
                launcher::WantedState::AimAtTarget,
                "{state:?}"
            );
            assert_eq!(plan.hood, hood::WantedState::AimAtTarget, "{state:?}");
        }
    }

    #[test]
    fn the_squeeze_closes_the_intake_after_the_delay_only_in_launch_with_squeeze() {
        assert_eq!(
            plan_for(CurrentSuperState::LaunchWithSqueeze, false).intake_extension,
            intake_extension::WantedState::FullExtend
        );
        assert_eq!(
            plan_for(CurrentSuperState::LaunchWithSqueeze, true).intake_extension,
            intake_extension::WantedState::SlowClose
        );
        assert_eq!(
            plan_for(CurrentSuperState::LaunchWithSqueezeWithNoDelay, false).intake_extension,
            intake_extension::WantedState::SlowClose
        );
        assert_eq!(
            plan_for(CurrentSuperState::LaunchWithoutSqueeze, true).intake_extension,
            intake_extension::WantedState::FullExtend
        );
    }

    #[test]
    fn launch_with_brake_locks_the_wheels_and_leaves_the_drive_coefficient() {
        let plan = plan_for(CurrentSuperState::LaunchWithBrake, false);
        assert_eq!(plan.swerve, Some(Swerve::XBrake));
        assert_eq!(plan.teleop_velocity_coefficient, None);
        assert_eq!(plan.fuel_intake, fuel_intake::WantedState::SlowIntake);
        assert_eq!(
            plan.intake_extension,
            intake_extension::WantedState::FullExtend
        );
    }

    #[test]
    fn auton_states_leave_the_drive_to_the_path_follower() {
        let idle = plan_for(CurrentSuperState::AutonIdle, false);
        assert_eq!(idle.swerve, Some(Swerve::Idle));
        assert_eq!(idle.teleop_velocity_coefficient, None);
        assert_eq!(idle.launcher, launcher::WantedState::IdlePrep);

        for state in [
            CurrentSuperState::AutonIntakeFuel,
            CurrentSuperState::AutonTrackTarget,
        ] {
            let plan = plan_for(state, false);
            assert_eq!(plan.swerve, None, "{state:?}");
            assert_eq!(plan.teleop_velocity_coefficient, None, "{state:?}");
        }
        let intake = plan_for(CurrentSuperState::AutonIntakeFuel, false);
        assert_eq!(intake.fuel_intake, fuel_intake::WantedState::Intake);
        assert_eq!(intake.indexer_bed, indexer_bed::WantedState::SlowIndex);
        let track = plan_for(CurrentSuperState::AutonTrackTarget, false);
        assert_eq!(track.launcher, launcher::WantedState::AimAtTarget);
        assert_eq!(track.fuel_intake, fuel_intake::WantedState::Neutral);
    }

    #[test]
    fn unjam_and_eject_run_the_indexers_backward_with_the_launcher_off() {
        let unjam = plan_for(CurrentSuperState::Unjam, false);
        assert_eq!(unjam.indexer_tower, indexer_tower::WantedState::Unjam);
        assert_eq!(unjam.indexer_bed, indexer_bed::WantedState::Unjam);
        assert_eq!(unjam.fuel_intake, fuel_intake::WantedState::Neutral);
        assert_eq!(unjam.launcher, launcher::WantedState::Off);
        assert_eq!(unjam.hood, hood::WantedState::Home);

        let eject = plan_for(CurrentSuperState::Eject, false);
        assert_eq!(eject.fuel_intake, fuel_intake::WantedState::Outtake);
        assert_eq!(eject.indexer_tower, indexer_tower::WantedState::Unjam);
        assert_eq!(
            eject.intake_extension,
            intake_extension::WantedState::ConditionalExtend
        );
    }

    #[test]
    fn force_home_retracts_the_intake_and_turns_the_launcher_off() {
        let plan = plan_for(CurrentSuperState::ForceHome, false);
        assert_eq!(
            plan.intake_extension,
            intake_extension::WantedState::FullRetract
        );
        assert_eq!(plan.launcher, launcher::WantedState::Off);
        assert_eq!(plan.hood, hood::WantedState::Home);
        assert_eq!(plan.swerve, Some(Swerve::TeleopDrive));
    }

    #[test]
    fn every_current_state_has_a_plan() {
        for wanted in ALL_WANTED {
            let current = handle_state_transitions(wanted, || false);
            let _ = plan_for(current, false);
        }
        let _ = plan_for(CurrentSuperState::AutonIdle, false);
    }

    #[test]
    fn the_squeeze_timer_restarts_only_when_the_squeeze_begins() {
        MICROS.store(0, Ordering::SeqCst);
        let mut timer = Timer::with_clock(fake_clock);
        let idle = CurrentSuperState::Idle;
        let squeeze = CurrentSuperState::LaunchWithSqueeze;

        update_squeeze_timer(&mut timer, squeeze, idle);
        assert!(timer.is_running());
        MICROS.store(600_000, Ordering::SeqCst);
        assert!(!timer.has_elapsed(SECONDS_TO_SQUEEZE));

        // Still squeezing: the timer keeps counting.
        update_squeeze_timer(&mut timer, squeeze, squeeze);
        MICROS.store(1_100_000, Ordering::SeqCst);
        assert!(timer.has_elapsed(SECONDS_TO_SQUEEZE));

        // Leaving and re-entering restarts it.
        update_squeeze_timer(&mut timer, idle, squeeze);
        update_squeeze_timer(&mut timer, squeeze, idle);
        assert!(!timer.has_elapsed(SECONDS_TO_SQUEEZE));
    }

    #[test]
    fn the_no_delay_squeeze_does_not_use_the_timer() {
        MICROS.store(0, Ordering::SeqCst);
        let mut timer = Timer::with_clock(fake_clock);
        update_squeeze_timer(
            &mut timer,
            CurrentSuperState::LaunchWithSqueezeWithNoDelay,
            CurrentSuperState::Idle,
        );
        assert!(!timer.is_running());
    }

    #[test]
    fn super_state_names_match_the_java_enum_constants() {
        assert_eq!(
            WantedSuperState::LaunchWithSqueezeWithNoDelay.name(),
            "LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY"
        );
        assert_eq!(CurrentSuperState::AutonIdle.name(), "AUTON_IDLE");
    }
}

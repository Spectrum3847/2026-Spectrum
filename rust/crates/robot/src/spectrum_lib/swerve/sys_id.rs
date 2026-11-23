//! SysId routines for a CTRE swerve drivetrain: translation, rotation, and steer gains.
//! Construct one per robot and bind [`SysId::sys_id_quasistatic`] and
//! [`SysId::sys_id_dynamic`] in test mode.

use crate::robot::subsystems::swerve::swerve::Swerve;
use commands::Command;
use commands::sysid::{Direction, RoutineConfig, State, SysIdRoutine};
use phoenix6::signal_logger;
use std::rc::Rc;
use std::sync::{Arc, Mutex, PoisonError};
use swerve::{SysIdSwerveRotation, SysIdSwerveSteerGains, SysIdSwerveTranslation};

fn lock<T>(mutex: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

pub struct SysId {
    pub sys_id_routine_translation: Rc<SysIdRoutine>,
    pub sys_id_routine_rotation: Rc<SysIdRoutine>,
    pub sys_id_routine_steer: Rc<SysIdRoutine>,

    /// Routine that `sys_id_quasistatic` and `sys_id_dynamic` run.
    routine_to_apply: Rc<SysIdRoutine>,
}

impl SysId {
    /// Builds all three routines. Edit the last line to pick which one runs. Steer runs at
    /// 7 V, translation and rotation at 4 V.
    pub fn new(swerve: &Rc<Swerve>) -> Self {
        let state_txt = "state";
        let record_state = move |state: State| {
            signal_logger::write_string(state_txt, state.name());
        };
        let config = |step_voltage| RoutineConfig {
            ramp_rate_volts_per_second: 1.0,
            step_voltage,
            timeout_seconds: 10.0,
        };

        let translation_characterization = Arc::new(Mutex::new(SysIdSwerveTranslation::default()));
        let translation_swerve = swerve.clone();
        let sys_id_routine_translation = Rc::new(SysIdRoutine::new(
            config(4.0),
            record_state,
            move |volts| {
                lock(&translation_characterization).volts_to_apply = volts;
                translation_swerve.set_control(&translation_characterization);
            },
            &**swerve,
        ));

        let rotation_characterization = Arc::new(Mutex::new(SysIdSwerveRotation::default()));
        let rotation_swerve = swerve.clone();
        let sys_id_routine_rotation = Rc::new(SysIdRoutine::new(
            config(4.0),
            record_state,
            move |roational_rate| {
                // Java: the voltage goes in as the rotational rate, rad/s.
                lock(&rotation_characterization).rotational_rate = roational_rate;
                rotation_swerve.set_control(&rotation_characterization);
            },
            &**swerve,
        ));

        let steer_characterization = Arc::new(Mutex::new(SysIdSwerveSteerGains::default()));
        let steer_swerve = swerve.clone();
        let sys_id_routine_steer = Rc::new(SysIdRoutine::new(
            config(7.0),
            record_state,
            move |volts| {
                lock(&steer_characterization).volts_to_apply = volts;
                steer_swerve.set_control(&steer_characterization);
            },
            &**swerve,
        ));

        let routine_to_apply = sys_id_routine_translation.clone();
        Self {
            sys_id_routine_translation,
            sys_id_routine_rotation,
            sys_id_routine_steer,
            routine_to_apply,
        }
    }

    /// Slow ramp command for the active routine.
    pub fn sys_id_quasistatic(&self, direction: Direction) -> Command {
        self.routine_to_apply.quasistatic(direction)
    }

    /// Step voltage command for the active routine.
    pub fn sys_id_dynamic(&self, direction: Direction) -> Command {
        self.routine_to_apply.dynamic(direction)
    }
}

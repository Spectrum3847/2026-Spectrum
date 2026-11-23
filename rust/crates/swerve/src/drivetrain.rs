//! The swerve drivetrain (`SwerveDrivetrain`): four modules, a Pigeon 2, and a thread that waits
//! on all their signals, runs odometry, applies the current request and calls telemetry.
//!
//! CTRE compiles the thread. The loop here follows the pre-2025 Java source of the same
//! thread: `waitForAll` on CAN FD, sleep plus `refreshAll` on CAN 2.0, request applied inline.

use crate::constants::{DrivetrainConstants, ModuleConstants};
use crate::module::{ModuleSink, SwerveModule, lock};
use crate::odometry::{OdometryCore, OdometryInput, SwerveDriveState};
use crate::request::{ControlParameters, NUM_MODULES, SwerveRequest};
use phoenix6::status_signal::{
    StatusSignal, latency_compensated_value, refresh_all_with, set_update_frequency_for_all,
    wait_for_all_with,
};
use phoenix6::{CanBus, Pigeon2, StatusCode, utils};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;
use wpimath::geometry::{Pose2d, Rotation2d, Translation2d};
use wpimath::kinematics::{SwerveDriveKinematics, SwerveModulePosition, SwerveModuleState};

const CONFIG_ATTEMPTS: usize = 2;
const CAN_FD_DEFAULT_HZ: f64 = 250.0;
const CAN_2_DEFAULT_HZ: f64 = 100.0;

/// A request the odometry thread applies each cycle. Keep a clone to edit it between cycles.
pub type SharedRequest = Arc<Mutex<dyn SwerveRequest>>;

type TelemetryFn = Box<dyn FnMut(&SwerveDriveState) + Send>;

#[derive(Clone, Debug)]
pub struct DrivetrainOptions {
    /// Zero picks 250 Hz on CAN FD and 100 Hz on CAN 2.0.
    pub odometry_update_frequency_hz: f64,
    /// Whether the bus is CAN FD. CTRE asks the C layer. The phoenix6 crate does not expose
    /// that query yet, so the caller says.
    pub can_fd: bool,
    pub odometry_std_devs: [f64; 3],
    pub vision_std_devs: [f64; 3],
}

impl Default for DrivetrainOptions {
    fn default() -> Self {
        Self {
            odometry_update_frequency_hz: 0.0,
            can_fd: true,
            odometry_std_devs: [0.1; 3],
            vision_std_devs: [0.9; 3],
        }
    }
}

struct PigeonSignals {
    yaw: StatusSignal,
    angular_velocity: StatusSignal,
}

struct Inner {
    modules: [SwerveModule; NUM_MODULES],
    module_locations: [Translation2d; NUM_MODULES],
    max_speed: f64,
    pigeon: Pigeon2,
    pigeon_signals: Mutex<PigeonSignals>,
    core: Mutex<OdometryCore>,
    request: Mutex<SharedRequest>,
    telemetry: Mutex<Option<TelemetryFn>>,
    running: AtomicBool,
    thread: Mutex<Option<JoinHandle<()>>>,
    frequency_hz: f64,
    can_fd: bool,
}

impl Inner {
    /// Stops and joins the odometry thread. Safe to call twice.
    fn stop(&self) {
        stop_thread(&self.running, &self.thread);
    }
}

fn stop_thread(running: &AtomicBool, thread: &Mutex<Option<JoinHandle<()>>>) {
    running.store(false, Ordering::Relaxed);
    let handle = lock(thread).take();
    if let Some(handle) = handle {
        let _ = handle.join();
    }
}

pub struct SwerveDrivetrain {
    inner: Arc<Inner>,
}

impl SwerveDrivetrain {
    /// Creates and configures every device, then starts the odometry thread. A device that
    /// does not accept its configuration is reported on stderr, as CTRE does. Fails when a
    /// device ID is invalid.
    pub fn new(
        drivetrain: &DrivetrainConstants,
        options: &DrivetrainOptions,
        modules: &[ModuleConstants; NUM_MODULES],
    ) -> Result<Self, StatusCode> {
        let bus = CanBus::new(drivetrain.can_bus_name.clone());
        let built = modules
            .iter()
            .map(|c| SwerveModule::new(c, &bus))
            .collect::<Result<Vec<_>, _>>()?;
        let Ok(built) = <[SwerveModule; NUM_MODULES]>::try_from(built) else {
            unreachable!("one module per constants entry");
        };
        let module_locations = std::array::from_fn(|i| {
            Translation2d::new(modules[i].location_x_m, modules[i].location_y_m)
        });
        let max_speed = modules
            .iter()
            .map(|c| c.speed_at_12_volts_mps)
            .fold(0.0, f64::max);

        let pigeon = Pigeon2::new(drivetrain.pigeon2_id, bus.clone())?;
        if let Some(configs) = &drivetrain.pigeon2_configs {
            let mut status = StatusCode::OK;
            for _ in 0..CONFIG_ATTEMPTS {
                status = pigeon.configurator().apply(configs);
                if status.is_ok() {
                    break;
                }
            }
            if !status.is_ok() {
                eprintln!(
                    "Pigeon2 ID {} failed config with error: {status}",
                    pigeon.device_id()
                );
            }
        }
        let mut pigeon_signals = PigeonSignals {
            yaw: pigeon.yaw(false),
            angular_velocity: pigeon.angular_velocity_z_world(false),
        };

        let frequency_hz = if options.odometry_update_frequency_hz > 0.0 {
            options.odometry_update_frequency_hz
        } else if options.can_fd {
            CAN_FD_DEFAULT_HZ
        } else {
            CAN_2_DEFAULT_HZ
        };

        let mut all: Vec<&StatusSignal> = Vec::new();
        let module_signals: Vec<_> = built.iter().map(|m| lock(&m.signals)).collect();
        for signals in &module_signals {
            all.extend(signals.iter());
        }
        all.push(&pigeon_signals.yaw);
        all.push(&pigeon_signals.angular_velocity);
        set_update_frequency_for_all(frequency_hz, &all);
        drop(all);
        drop(module_signals);

        // First blocking read seeds odometry. A device that is not there yet reads zero.
        let positions: [SwerveModulePosition; NUM_MODULES] =
            std::array::from_fn(|i| built[i].get_position(true));
        refresh_all_with(
            false,
            &mut [
                &mut pigeon_signals.yaw,
                &mut pigeon_signals.angular_velocity,
            ],
        );
        let initial_yaw = Rotation2d::from_degrees(latency_compensated_value(
            &pigeon_signals.yaw,
            &pigeon_signals.angular_velocity,
        ));

        let core = OdometryCore::new(
            module_locations,
            initial_yaw,
            positions,
            options.odometry_std_devs,
            options.vision_std_devs,
            utils::current_time_seconds(),
            1.0 / frequency_hz,
        );
        let idle: SharedRequest = Arc::new(Mutex::new(crate::request::Idle));
        let inner = Arc::new(Inner {
            modules: built,
            module_locations,
            max_speed,
            pigeon,
            pigeon_signals: Mutex::new(pigeon_signals),
            core: Mutex::new(core),
            request: Mutex::new(idle),
            telemetry: Mutex::new(None),
            running: AtomicBool::new(true),
            thread: Mutex::new(None),
            frequency_hz,
            can_fd: options.can_fd,
        });

        let thread_inner = Arc::clone(&inner);
        let handle = std::thread::Builder::new()
            .name("swerve-odometry".into())
            .spawn(move || run(&thread_inner))
            .map_err(|_| StatusCode::TaskIsBusy)?;
        *lock(&inner.thread) = Some(handle);
        let weak = Arc::downgrade(&inner);
        wpilib::shutdown::register(move || {
            if let Some(inner) = weak.upgrade() {
                inner.stop();
            }
        });
        Ok(Self { inner })
    }

    /// Stops the odometry thread. Also runs on drop and at program shutdown.
    pub fn shutdown(&self) {
        self.inner.stop();
    }

    pub fn is_on_can_fd(&self) -> bool {
        self.inner.can_fd
    }

    pub fn get_odometry_frequency(&self) -> f64 {
        self.inner.frequency_hz
    }

    /// True after more than two successful signal reads.
    pub fn is_odometry_valid(&self) -> bool {
        lock(&self.inner.core).is_odometry_valid()
    }

    /// A copy of the kinematics, for `to_chassis_speeds` and friends.
    pub fn get_kinematics(&self) -> SwerveDriveKinematics<NUM_MODULES> {
        lock(&self.inner.core).kinematics().clone()
    }

    /// Applied on the odometry thread every cycle until replaced. Edit the request through
    /// its mutex to change what it asks for.
    pub fn set_control<R: SwerveRequest + 'static>(&self, request: &Arc<Mutex<R>>) {
        let shared: SharedRequest = request.clone();
        *lock(&self.inner.request) = shared;
    }

    /// `set_control` for a request you will not edit again.
    pub fn set_control_once(&self, request: impl SwerveRequest + 'static) {
        let shared: SharedRequest = Arc::new(Mutex::new(request));
        *lock(&self.inner.request) = shared;
    }

    /// A copy of the cached state. Refreshed every odometry cycle.
    pub fn get_state(&self) -> SwerveDriveState {
        lock(&self.inner.core).state().clone()
    }

    /// Runs `callback` on the odometry thread after each cycle, with the newest state.
    /// Replaces any earlier callback.
    pub fn register_telemetry(&self, callback: impl FnMut(&SwerveDriveState) + Send + 'static) {
        *lock(&self.inner.telemetry) = Some(Box::new(callback));
    }

    /// Zeroes the drive motor positions and the pose.
    pub fn tare_everything(&self) {
        let mut core = lock(&self.inner.core);
        for module in &self.inner.modules {
            module.reset_position();
        }
        let positions = std::array::from_fn(|i| self.inner.modules[i].get_position(true));
        core.tare(positions);
    }

    /// Sets the current heading to `rotation` in the operator perspective.
    pub fn seed_field_centric(&self, rotation: Rotation2d) {
        lock(&self.inner.core).seed_field_centric(rotation);
    }

    pub fn reset_pose(&self, pose: Pose2d) {
        lock(&self.inner.core).reset_pose(pose);
    }

    pub fn reset_translation(&self, translation: Translation2d) {
        lock(&self.inner.core).reset_translation(translation);
    }

    pub fn reset_rotation(&self, rotation: Rotation2d) {
        lock(&self.inner.core).reset_rotation(rotation);
    }

    /// The field direction the operator calls forward. Blue alliance is zero, red is 180
    /// degrees.
    pub fn set_operator_perspective_forward(&self, field_direction: Rotation2d) {
        lock(&self.inner.core).set_operator_perspective_forward(field_direction);
    }

    pub fn get_operator_forward_direction(&self) -> Rotation2d {
        lock(&self.inner.core).operator_forward_direction()
    }

    /// `timestamp` is on the Phoenix clock. Convert FPGA time with `fpga_to_current_time`.
    pub fn add_vision_measurement(&self, pose: Pose2d, timestamp: f64) {
        lock(&self.inner.core).add_vision_measurement(pose, timestamp);
    }

    /// As `add_vision_measurement`, with standard deviations (x m, y m, heading rad) that
    /// stay in effect for later measurements too.
    pub fn add_vision_measurement_with_std_devs(
        &self,
        pose: Pose2d,
        timestamp: f64,
        std_devs: [f64; 3],
    ) {
        lock(&self.inner.core).add_vision_measurement_with_std_devs(pose, timestamp, std_devs);
    }

    pub fn set_vision_measurement_std_devs(&self, std_devs: [f64; 3]) {
        lock(&self.inner.core).set_vision_measurement_std_devs(std_devs);
    }

    /// The estimated pose at a Phoenix clock timestamp, or `None` before odometry has run.
    pub fn sample_pose_at(&self, timestamp: f64) -> Option<Pose2d> {
        lock(&self.inner.core).sample_pose_at(timestamp)
    }

    pub fn get_modules(&self) -> &[SwerveModule; NUM_MODULES] {
        &self.inner.modules
    }

    pub fn get_module(&self, index: usize) -> Option<&SwerveModule> {
        self.inner.modules.get(index)
    }

    pub fn get_module_locations(&self) -> &[Translation2d; NUM_MODULES] {
        &self.inner.module_locations
    }

    pub fn get_pigeon2(&self) -> &Pigeon2 {
        &self.inner.pigeon
    }

    /// Optimizes every device on the drivetrain. Set the signals you need first. Returns the
    /// first failure.
    pub fn optimize_bus_utilization(&self) -> StatusCode {
        let mut result = self.inner.pigeon.optimize_bus_utilization();
        for module in &self.inner.modules {
            for status in [
                module.drive_motor().optimize_bus_utilization(),
                module.steer_motor().optimize_bus_utilization(),
                module.encoder().optimize_bus_utilization(),
            ] {
                if result.is_ok() {
                    result = status;
                }
            }
        }
        result
    }
}

impl Drop for SwerveDrivetrain {
    fn drop(&mut self) {
        self.inner.stop();
    }
}

fn run(inner: &Inner) {
    let period = 1.0 / inner.frequency_hz;
    let mut kinematics = lock(&inner.core).kinematics().clone();
    while inner.running.load(Ordering::Relaxed) {
        let loop_start = utils::current_time_seconds();
        let (input, ok) = read_cycle(inner);
        let snapshot = lock(&inner.core).update(&OdometryInput {
            daq_ok: ok,
            ..input
        });

        let request = Arc::clone(&lock(&inner.request));
        {
            let mut parameters = ControlParameters {
                kinematics: &mut kinematics,
                module_locations: inner.module_locations,
                max_speed: inner.max_speed,
                operator_forward_direction: snapshot.operator_forward_direction,
                current_chassis_speed: snapshot.current_chassis_speed,
                current_pose: snapshot.current_pose,
                timestamp: snapshot.timestamp,
                update_period: snapshot.update_period,
            };
            let sinks: [&dyn ModuleSink; NUM_MODULES] =
                std::array::from_fn(|i| &inner.modules[i] as &dyn ModuleSink);
            lock(&request).apply(&mut parameters, &sinks);
        }

        if let Some(callback) = lock(&inner.telemetry).as_mut() {
            let state = lock(&inner.core).state().clone();
            callback(&state);
        }

        // A failed wait can return at once. Do not spin.
        if !ok {
            let spent = utils::current_time_seconds() - loop_start;
            if spent < period {
                std::thread::sleep(Duration::from_secs_f64(period - spent));
            }
        }
    }
}

/// Waits for (or refreshes) every signal and turns them into one odometry input. `daq_ok` of
/// the returned input is not set: the bool says whether the wait succeeded.
fn read_cycle(inner: &Inner) -> (OdometryInput, bool) {
    let mut module_guards: Vec<_> = inner.modules.iter().map(|m| lock(&m.signals)).collect();
    let mut pigeon = lock(&inner.pigeon_signals);

    let status = {
        let mut refs: Vec<&mut StatusSignal> = Vec::with_capacity(NUM_MODULES * 4 + 2);
        for guard in &mut module_guards {
            refs.extend(guard.iter_mut());
        }
        let PigeonSignals {
            yaw,
            angular_velocity,
        } = &mut *pigeon;
        refs.push(yaw);
        refs.push(angular_velocity);
        if inner.can_fd {
            wait_for_all_with(2.0 / inner.frequency_hz, false, &mut refs)
        } else {
            std::thread::sleep(Duration::from_secs_f64(1.0 / inner.frequency_hz));
            refresh_all_with(false, &mut refs)
        }
    };
    let time = utils::current_time_seconds();

    let yaw = Rotation2d::from_degrees(latency_compensated_value(
        &pigeon.yaw,
        &pigeon.angular_velocity,
    ));
    let positions = std::array::from_fn(|i| inner.modules[i].store_readings(&module_guards[i]));
    let states: [SwerveModuleState; NUM_MODULES] =
        std::array::from_fn(|i| inner.modules[i].get_current_state());
    let targets = std::array::from_fn(|i| inner.modules[i].get_target_state());
    drop(pigeon);
    drop(module_guards);

    (
        OdometryInput {
            time,
            yaw,
            positions,
            states,
            targets,
            daq_ok: status.is_ok(),
        },
        status.is_ok(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stopping_the_thread_twice_joins_it_once() {
        let running = Arc::new(AtomicBool::new(true));
        let spinner = Arc::clone(&running);
        let handle = std::thread::spawn(move || {
            while spinner.load(Ordering::Relaxed) {
                std::thread::sleep(Duration::from_millis(1));
            }
        });
        let thread = Mutex::new(Some(handle));
        stop_thread(&running, &thread);
        stop_thread(&running, &thread);
        assert!(!running.load(Ordering::Relaxed));
        assert!(lock(&thread).is_none());
    }
}

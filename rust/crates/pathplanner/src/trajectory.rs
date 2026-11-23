use crate::config::RobotConfig;
use crate::events::Event;
use crate::feedforwards::{DriveFeedforwards, MAX_MODULES, PerModule};
use crate::flipping::FlippingUtil;
use crate::geometry_util::calculate_radius;
use crate::path::{PathConstraints, PathPlannerPath};
use crate::util::JavaPriorityQueue;
use wpimath::geometry::{Pose2d, Rotation2d, Translation2d};
use wpimath::interpolation::Interpolatable;
use wpimath::kinematics::{ChassisSpeeds, SwerveModuleState};
use wpimath::math_util;

/// A state along a [`PathPlannerTrajectory`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PathPlannerTrajectoryState {
    pub time_seconds: f64,
    /// Field-relative chassis speeds.
    pub field_speeds: ChassisSpeeds,
    /// Field-relative robot pose.
    pub pose: Pose2d,
    pub linear_velocity: f64,
    /// Field-relative direction of travel.
    pub heading: Rotation2d,
    pub feedforwards: DriveFeedforwards,
}

impl PathPlannerTrajectoryState {
    pub fn interpolate(&self, end: &Self, t: f64) -> Self {
        let time_seconds = math_util::interpolate(self.time_seconds, end.time_seconds, t);
        if time_seconds - self.time_seconds < 0.0 {
            return end.interpolate(self, 1.0 - t);
        }

        let field_speeds = ChassisSpeeds::new(
            math_util::interpolate(self.field_speeds.vx, end.field_speeds.vx, t),
            math_util::interpolate(self.field_speeds.vy, end.field_speeds.vy, t),
            math_util::interpolate(self.field_speeds.omega, end.field_speeds.omega, t),
        );
        let heading = self.heading;
        let linear_velocity = math_util::interpolate(self.linear_velocity, end.linear_velocity, t);

        // Integrate the speeds for the position. Interpolating the pose is inaccurate when the
        // speeds change between the two states.
        let mut x = self.pose.x();
        let mut y = self.pose.y();
        let mut int_time = self.time_seconds + 0.01;
        loop {
            let int_t = (int_time - self.time_seconds) / (time_seconds - self.time_seconds);
            let int_linear_vel =
                math_util::interpolate(self.linear_velocity, linear_velocity, int_t);
            let int_vx = int_linear_vel * heading.cos();
            let int_vy = int_linear_vel * heading.sin();

            if int_time >= time_seconds - 0.01 {
                let dt = time_seconds - int_time;
                x += int_vx * dt;
                y += int_vy * dt;
                break;
            }

            x += int_vx * 0.01;
            y += int_vy * 0.01;
            int_time += 0.01;
        }

        Self {
            time_seconds,
            field_speeds,
            pose: Pose2d::from_xy(x, y, self.pose.rotation.interpolate(&end.pose.rotation, t)),
            linear_velocity,
            heading,
            feedforwards: self.feedforwards.interpolate(&end.feedforwards, t),
        }
    }

    /// For the other side of the field.
    pub fn flip(&self) -> Self {
        Self {
            time_seconds: self.time_seconds,
            field_speeds: FlippingUtil::flip_field_speeds(self.field_speeds),
            pose: FlippingUtil::flip_field_pose(self.pose),
            linear_velocity: self.linear_velocity,
            heading: FlippingUtil::flip_field_rotation(self.heading),
            feedforwards: self.feedforwards.flip(),
        }
    }
}

/// A trajectory generated for a path: timed states and the events to run along them.
#[derive(Clone, Debug)]
pub struct PathPlannerTrajectory {
    states: Vec<PathPlannerTrajectoryState>,
    events: Vec<Event>,
}

/// Per module data that only generation needs.
#[derive(Clone, Copy, Default)]
struct Module {
    speed: f64,
    angle: Rotation2d,
    field_angle: Rotation2d,
    field_pos: Translation2d,
    delta_pos: f64,
}

struct Generating {
    state: PathPlannerTrajectoryState,
    delta_pos: f64,
    modules: [Module; MAX_MODULES],
    constraints: PathConstraints,
    waypoint_relative_pos: f64,
}

impl PathPlannerTrajectory {
    pub fn new(states: Vec<PathPlannerTrajectoryState>, events: Vec<Event>) -> Self {
        Self { states, events }
    }

    /// Generates a trajectory for a path.
    ///
    /// `starting_speeds` are robot relative and `starting_rotation` is the field-relative
    /// rotation of the robot.
    pub fn generate(
        path: &PathPlannerPath,
        starting_speeds: ChassisSpeeds,
        starting_rotation: Rotation2d,
        config: &RobotConfig,
    ) -> Self {
        let nm = config.num_modules;
        let mut gens = generate_states(path, starting_rotation, config);

        // Initial module velocities. Java hands the field-relative speeds to the module
        // kinematics here, and so does this.
        let field_starting_speeds =
            ChassisSpeeds::from_robot_relative(starting_speeds, gens[0].state.pose.rotation);
        let initial = config.module_speeds(field_starting_speeds);
        for (m, speed) in initial.iter().enumerate().take(nm) {
            gens[0].modules[m].speed = *speed;
        }
        gens[0].state.time_seconds = 0.0;
        gens[0].state.field_speeds = field_starting_speeds;
        gens[0].state.linear_velocity = field_starting_speeds.vx.hypot(field_starting_speeds.vy);

        forward_accel_pass(&mut gens, config);

        let last = gens.len() - 1;
        let end_speed =
            Translation2d::from_polar(path.goal_end_state().velocity_mps, gens[last].state.heading);
        let end_field_speeds = ChassisSpeeds::new(end_speed.x, end_speed.y, 0.0);
        let end_speeds = config.module_speeds(ChassisSpeeds::from_field_relative(
            end_field_speeds,
            gens[last].state.pose.rotation,
        ));
        for (m, speed) in end_speeds.iter().enumerate().take(nm) {
            gens[last].modules[m].speed = *speed;
        }
        gens[last].state.field_speeds = end_field_speeds;
        gens[last].state.linear_velocity = path.goal_end_state().velocity_mps;

        reverse_accel_pass(&mut gens, config);

        let mut unadded = JavaPriorityQueue::new(Event::timestamp_seconds);
        for marker in path.event_markers() {
            if let Some(command) = &marker.command {
                unadded.push(Event::ScheduleCommand {
                    timestamp: marker.position,
                    command: command.clone(),
                });
            }
            if marker.end_position >= 0.0 {
                if let Some(command) = &marker.command {
                    unadded.push(Event::CancelCommand {
                        timestamp: marker.end_position,
                        command: command.clone(),
                    });
                }
                unadded.push(Event::Trigger {
                    timestamp: marker.position,
                    name: marker.trigger_name.clone(),
                    active: true,
                });
                unadded.push(Event::Trigger {
                    timestamp: marker.end_position,
                    name: marker.trigger_name.clone(),
                    active: false,
                });
            } else {
                unadded.push(Event::OneShotTrigger {
                    timestamp: marker.position,
                    name: marker.trigger_name.clone(),
                });
            }
        }
        for zone in path.point_towards_zones() {
            unadded.push(Event::PointTowardsZone {
                timestamp: zone.min_position,
                name: zone.name.clone(),
                active: true,
            });
            unadded.push(Event::PointTowardsZone {
                timestamp: zone.max_position,
                name: zone.name.clone(),
                active: false,
            });
        }

        let mut events: Vec<Event> = Vec::with_capacity(path.event_markers().len() * 2);

        // Loop back over the states for the time and the module feedforwards.
        for i in 1..gens.len() {
            let v0 = gens[i - 1].state.linear_velocity;
            let v = gens[i].state.linear_velocity;
            let sum_v = v + v0;
            if sum_v.abs() < 1e-6 || gens[i].delta_pos.abs() < 1e-6 {
                gens[i].state.time_seconds = gens[i - 1].state.time_seconds;
                gens[i - 1].state.feedforwards = if i != 1 {
                    gens[i - 2].state.feedforwards
                } else {
                    DriveFeedforwards::zeros(nm)
                };
            } else {
                let dt = (2.0 * gens[i].delta_pos) / sum_v;
                gens[i].state.time_seconds = gens[i - 1].state.time_seconds + dt;

                let prev_robot_speeds = ChassisSpeeds::from_field_relative(
                    gens[i - 1].state.field_speeds,
                    gens[i - 1].state.pose.rotation,
                );
                let robot_speeds = ChassisSpeeds::from_field_relative(
                    gens[i].state.field_speeds,
                    gens[i].state.pose.rotation,
                );
                let chassis_accel_x = (robot_speeds.vx - prev_robot_speeds.vx) / dt;
                let chassis_accel_y = (robot_speeds.vy - prev_robot_speeds.vy) / dt;
                let chassis_force_x = chassis_accel_x * config.mass_kg;
                let chassis_force_y = chassis_accel_y * config.mass_kg;

                let angular_accel = (robot_speeds.omega - prev_robot_speeds.omega) / dt;
                let ang_torque = angular_accel * config.moi;
                let chassis_forces =
                    ChassisSpeeds::new(chassis_force_x, chassis_force_y, ang_torque);

                let wheel_forces = config.chassis_forces_to_wheel_force_vectors(chassis_forces);
                let mut accel_ff = PerModule::zeros(nm);
                let mut linear_force_ff = PerModule::zeros(nm);
                let mut torque_current_ff = PerModule::zeros(nm);
                let mut force_x_ff = PerModule::zeros(nm);
                let mut force_y_ff = PerModule::zeros(nm);
                for (m, wheel_force) in wheel_forces.iter().enumerate().take(nm) {
                    let wheel_force_dist = wheel_force.norm();
                    let applied_force = if wheel_force_dist > 1e-6 {
                        wheel_force_dist * (wheel_force.angle() - gens[i].modules[m].angle).cos()
                    } else {
                        0.0
                    };
                    let wheel_torque = applied_force * config.module_config.wheel_radius_meters;
                    let torque_current = config
                        .module_config
                        .drive_motor
                        .current_from_torque(wheel_torque);

                    accel_ff.set(
                        m,
                        (gens[i].modules[m].speed - gens[i - 1].modules[m].speed) / dt,
                    );
                    linear_force_ff.set(m, applied_force);
                    torque_current_ff.set(m, torque_current);
                    force_x_ff.set(m, wheel_force.x);
                    force_y_ff.set(m, wheel_force.y);
                }
                gens[i - 1].state.feedforwards = DriveFeedforwards {
                    accelerations_mps_sq: accel_ff,
                    linear_forces_newtons: linear_force_ff,
                    torque_currents_amps: torque_current_ff,
                    robot_relative_forces_x_newtons: force_x_ff,
                    robot_relative_forces_y_newtons: force_y_ff,
                };
            }

            // Unadded events hold a waypoint relative position as their timestamp until a state
            // claims them.
            while unadded.peek().is_some_and(|e| {
                (e.timestamp_seconds() - gens[i - 1].waypoint_relative_pos).abs()
                    <= (e.timestamp_seconds() - gens[i].waypoint_relative_pos).abs()
            }) {
                if let Some(mut event) = unadded.pop() {
                    event.set_timestamp(gens[i - 1].state.time_seconds);
                    events.push(event);
                }
            }
        }

        let end_time = gens[last].state.time_seconds;
        while let Some(mut event) = unadded.pop() {
            event.set_timestamp(end_time);
            events.push(event);
        }

        gens[last].state.feedforwards = DriveFeedforwards::zeros(nm);

        Self {
            states: gens.into_iter().map(|g| g.state).collect(),
            events,
        }
    }

    /// Events to run while following this trajectory.
    pub fn events(&self) -> &[Event] {
        &self.events
    }

    pub fn states(&self) -> &[PathPlannerTrajectoryState] {
        &self.states
    }

    pub fn state(&self, index: usize) -> &PathPlannerTrajectoryState {
        &self.states[index]
    }

    pub fn initial_state(&self) -> &PathPlannerTrajectoryState {
        &self.states[0]
    }

    pub fn end_state(&self) -> &PathPlannerTrajectoryState {
        &self.states[self.states.len() - 1]
    }

    pub fn total_time_seconds(&self) -> f64 {
        self.end_state().time_seconds
    }

    pub fn initial_pose(&self) -> Pose2d {
        self.initial_state().pose
    }

    /// The target state at a time along the trajectory.
    pub fn sample(&self, time: f64) -> PathPlannerTrajectoryState {
        if time <= self.initial_state().time_seconds {
            return *self.initial_state();
        }
        if time >= self.total_time_seconds() {
            return *self.end_state();
        }

        let mut low = 1;
        let mut high = self.states.len() - 1;
        while low != high {
            let mid = (low + high) / 2;
            if self.states[mid].time_seconds < time {
                low = mid + 1;
            } else {
                high = mid;
            }
        }

        let sample = &self.states[low];
        let prev_sample = &self.states[low - 1];

        if (sample.time_seconds - prev_sample.time_seconds).abs() < 1e-3 {
            return *sample;
        }

        prev_sample.interpolate(
            sample,
            (time - prev_sample.time_seconds) / (sample.time_seconds - prev_sample.time_seconds),
        )
    }

    /// For the other side of the field.
    pub fn flip(&self) -> Self {
        Self {
            states: self.states.iter().map(|s| s.flip()).collect(),
            events: self.events.clone(),
        }
    }
}

fn cosine_interpolate(start: Rotation2d, end: Rotation2d, t: f64) -> Rotation2d {
    let t2 = (1.0 - (t * std::f64::consts::PI).cos()) / 2.0;
    start.interpolate(&end, t2)
}

fn next_rotation_target_idx(path: &PathPlannerPath, starting_index: usize) -> usize {
    (starting_index..path.num_points() - 1)
        .find(|&i| path.point(i).rotation_target.is_some())
        .unwrap_or(path.num_points() - 1)
}

fn generate_states(
    path: &PathPlannerPath,
    starting_rotation: Rotation2d,
    config: &RobotConfig,
) -> Vec<Generating> {
    let nm = config.num_modules;
    let n = path.num_points();
    let mut states: Vec<Generating> = Vec::with_capacity(n);

    let target_rotation = |i: usize| {
        path.point(i)
            .rotation_target
            .map(|t| t.rotation)
            .expect("the last path point always has a rotation target")
    };

    let mut prev_target_idx = 0;
    let mut prev_target_rot = starting_rotation;
    let mut next_target_idx = next_rotation_target_idx(path, 0);
    let mut next_target_rot = target_rotation(next_target_idx);

    for i in 0..n {
        let p = path.point(i);

        if i > next_target_idx {
            prev_target_idx = next_target_idx;
            prev_target_rot = next_target_rot;
            next_target_idx = next_rotation_target_idx(path, i);
            next_target_rot = target_rotation(next_target_idx);
        }

        // Rotation is interpolated by distance along the path, since the points are not evenly
        // spread over a segment.
        let t = (p.distance_along_path - path.point(prev_target_idx).distance_along_path)
            / (path.point(next_target_idx).distance_along_path
                - path.point(prev_target_idx).distance_along_path);
        let holonomic_rot = cosine_interpolate(prev_target_rot, next_target_rot, t);

        let mut pose = Pose2d::new(p.position, holonomic_rot);

        let heading = if i != n - 1 {
            let heading_translation = path.point(i + 1).position - pose.translation;
            if heading_translation.norm() <= 1e-6 {
                Rotation2d::ZERO
            } else {
                heading_translation.angle()
            }
        } else {
            states[i - 1].state.heading
        };

        if !config.is_holonomic {
            pose = Pose2d::new(pose.translation, heading);
        }

        let delta_pos = if i != 0 {
            pose.translation
                .distance(states[i - 1].state.pose.translation)
        } else {
            0.0
        };

        let mut modules = [Module::default(); MAX_MODULES];
        for (m, module) in modules.iter_mut().enumerate().take(nm) {
            module.field_pos =
                pose.translation + config.module_locations[m].rotate_by(pose.rotation);
            if i != 0 {
                module.delta_pos = module
                    .field_pos
                    .distance(states[i - 1].modules[m].field_pos);
            }
        }

        states.push(Generating {
            state: PathPlannerTrajectoryState {
                pose,
                heading,
                ..Default::default()
            },
            delta_pos,
            modules,
            constraints: p
                .constraints
                .expect("path points get constraints when the path is built"),
            waypoint_relative_pos: p.waypoint_relative_pos,
        });
    }

    // Module headings.
    for i in 0..n {
        for m in 0..nm {
            let field_angle = if i != n - 1 {
                let field_translation =
                    states[i + 1].modules[m].field_pos - states[i].modules[m].field_pos;
                if field_translation.norm() <= 1e-6 {
                    Rotation2d::ZERO
                } else {
                    field_translation.angle()
                }
            } else {
                states[i - 1].modules[m].field_angle
            };
            states[i].modules[m].field_angle = field_angle;
            states[i].modules[m].angle = field_angle - states[i].state.pose.rotation;
        }
    }

    states
}

/// Module speeds seen as states, for the chassis kinematics.
fn module_states(modules: &[Module; MAX_MODULES], nm: usize) -> [SwerveModuleState; MAX_MODULES] {
    let mut out = [SwerveModuleState::default(); MAX_MODULES];
    for (state, module) in out.iter_mut().zip(modules).take(nm) {
        *state = SwerveModuleState::new(module.speed, module.angle);
    }
    out
}

/// Force and torque on the whole robot from every module pushing with `force_at_carpet(m)`
/// along `direction(m)`.
fn total_force_and_torque(
    state: &Generating,
    config: &RobotConfig,
    force_at_carpet: impl Fn(usize) -> f64,
    direction: impl Fn(&Module) -> Rotation2d,
) -> (Translation2d, f64) {
    let mut linear_force = Translation2d::ZERO;
    let mut total_torque = 0.0;
    for m in 0..config.num_modules {
        let force = force_at_carpet(m);
        let force_vec = Translation2d::from_polar(force, direction(&state.modules[m]));
        linear_force = linear_force + force_vec;

        let angle_to_module = (state.modules[m].field_pos - state.state.pose.translation).angle();
        let theta = if force_vec.norm() <= 1e-6 {
            Rotation2d::ZERO - angle_to_module
        } else {
            force_vec.angle() - angle_to_module
        };
        total_torque += force * config.module_pivot_distance[m] * theta.sin();
    }
    (linear_force, total_torque)
}

/// Limits the robot acceleration and turns it into the acceleration of each module.
fn module_accelerations(
    state: &Generating,
    config: &RobotConfig,
    linear_force: Translation2d,
    total_torque: f64,
    make_chassis_accel: impl Fn(Translation2d, f64, Rotation2d) -> ChassisSpeeds,
) -> [f64; MAX_MODULES] {
    let max_ang_accel = state.constraints.max_angular_acceleration_rad_per_sec_sq;
    let angular_accel = math_util::clamp(total_torque / config.moi, -max_ang_accel, max_ang_accel);

    let mut accel_vec = linear_force / config.mass_kg;
    let max_accel = state.constraints.max_acceleration_mps_sq;
    let accel = accel_vec.norm();
    if accel > max_accel {
        accel_vec = accel_vec * (max_accel / accel);
    }

    let chassis_accel = make_chassis_accel(accel_vec, angular_accel, state.state.pose.rotation);
    config.module_speeds(chassis_accel)
}

/// Makes every module take the same time to reach the next state, then limits the speeds.
fn equalize_and_desaturate(
    state: &mut Generating,
    next_delta_pos: &[Module; MAX_MODULES],
    prev_angles: &[Module; MAX_MODULES],
    config: &RobotConfig,
    max_chassis_vel: f64,
    max_chassis_ang_vel: f64,
) {
    let nm = config.num_modules;
    let mut max_dt = 0.0_f64;
    let mut real_max_dt = 0.0_f64;
    for m in 0..nm {
        let prev_rot_delta = state.modules[m].angle - prev_angles[m].angle;
        let mod_vel = state.modules[m].speed;
        let dt = next_delta_pos[m].delta_pos / mod_vel;

        if dt.is_finite() {
            real_max_dt = dt.max(real_max_dt);

            if prev_rot_delta.degrees().abs() < 60.0 {
                max_dt = dt.max(max_dt);
            }
        }
    }

    if max_dt == 0.0 {
        max_dt = real_max_dt;
    }

    if max_dt > 0.0 {
        for m in 0..nm {
            let prev_rot_delta = state.modules[m].angle - prev_angles[m].angle;
            if prev_rot_delta.degrees().abs() >= 60.0 {
                continue;
            }
            state.modules[m].speed = next_delta_pos[m].delta_pos / max_dt;
        }
    }

    let desired = config.to_chassis_speeds(&module_states(&state.modules, nm)[..nm]);
    desaturate_wheel_speeds(
        &mut state.modules[..nm],
        desired,
        config.module_config.max_drive_velocity_mps,
        max_chassis_vel,
        max_chassis_ang_vel,
    );

    state.state.field_speeds = ChassisSpeeds::from_robot_relative(
        config.to_chassis_speeds(&module_states(&state.modules, nm)[..nm]),
        state.state.pose.rotation,
    );
    state.state.linear_velocity = state
        .state
        .field_speeds
        .vx
        .hypot(state.state.field_speeds.vy);
}

fn forward_accel_pass(states: &mut [Generating], config: &RobotConfig) {
    let nm = config.num_modules;
    for i in 1..states.len() - 1 {
        let prev_modules = states[i - 1].modules;
        let next_modules = states[i + 1].modules;
        let state = &mut states[i];

        // This pass handles acceleration, so friction and other losses fight the motor.
        let (linear_force, total_torque) = total_force_and_torque(
            state,
            config,
            |m| {
                let last_vel_rad_per_sec =
                    prev_modules[m].speed / config.module_config.wheel_radius_meters;
                let current_draw = config
                    .module_config
                    .drive_motor
                    .current(
                        last_vel_rad_per_sec,
                        state.constraints.nominal_voltage_volts,
                    )
                    .min(config.module_config.drive_current_limit);
                let available_torque = config.module_config.drive_motor.torque(current_draw)
                    - config.module_config.torque_loss;
                let available_torque = available_torque.min(config.max_torque_friction);
                available_torque / config.module_config.wheel_radius_meters
            },
            |module| module.field_angle,
        );

        // Kinematics is meant for velocities, but it also turns chassis accelerations into
        // module accelerations.
        let accel_speeds = module_accelerations(
            state,
            config,
            linear_force,
            total_torque,
            |accel_vec, angular_accel, rotation| {
                ChassisSpeeds::from_field_relative(
                    ChassisSpeeds::new(accel_vec.x, accel_vec.y, angular_accel),
                    rotation,
                )
            },
        );
        for m in 0..nm {
            let module_acceleration = accel_speeds[m].abs();

            // vf^2 = v0^2 + 2ad
            state.modules[m].speed = (prev_modules[m].speed * prev_modules[m].speed
                + (2.0 * module_acceleration * state.modules[m].delta_pos))
                .abs()
                .sqrt();

            let curve_radius = calculate_radius(
                prev_modules[m].field_pos,
                state.modules[m].field_pos,
                next_modules[m].field_pos,
            );
            // Keep the centripetal force Fc = M * v^2 / R under the friction force.
            if curve_radius.is_finite() {
                let max_safe_vel = ((config.wheel_friction_force * curve_radius.abs())
                    / (config.mass_kg / nm as f64))
                    .sqrt();
                state.modules[m].speed = state.modules[m].speed.min(max_safe_vel);
            }
        }

        let max_chassis_vel = state.constraints.max_velocity_mps;
        let max_chassis_ang_vel = state.constraints.max_angular_velocity_rad_per_sec;
        equalize_and_desaturate(
            state,
            &next_modules,
            &prev_modules,
            config,
            max_chassis_vel,
            max_chassis_ang_vel,
        );
    }
}

fn reverse_accel_pass(states: &mut [Generating], config: &RobotConfig) {
    let nm = config.num_modules;
    for i in (1..states.len() - 1).rev() {
        let prev_modules = states[i - 1].modules;
        let next_modules = states[i + 1].modules;
        let state = &mut states[i];

        // This pass handles deceleration, so the losses do not fight the motor.
        let (linear_force, total_torque) = total_force_and_torque(
            state,
            config,
            |m| {
                let last_vel_rad_per_sec =
                    next_modules[m].speed / config.module_config.wheel_radius_meters;
                let current_draw = config
                    .module_config
                    .drive_motor
                    .current(
                        last_vel_rad_per_sec,
                        state.constraints.nominal_voltage_volts,
                    )
                    .min(config.module_config.drive_current_limit);
                let available_torque = config.module_config.drive_motor.torque(current_draw);
                let available_torque = available_torque.min(config.max_torque_friction);
                available_torque / config.module_config.wheel_radius_meters
            },
            |module| module.field_angle + Rotation2d::PI,
        );

        let accel_speeds = module_accelerations(
            state,
            config,
            linear_force,
            total_torque,
            |accel_vec, angular_accel, rotation| {
                ChassisSpeeds::from_field_relative(
                    ChassisSpeeds::new(accel_vec.x, accel_vec.y, angular_accel),
                    rotation,
                )
            },
        );
        for m in 0..nm {
            let module_acceleration = accel_speeds[m].abs();

            // vf^2 = v0^2 + 2ad
            let max_vel = (next_modules[m].speed * next_modules[m].speed
                + (2.0 * module_acceleration * next_modules[m].delta_pos))
                .abs()
                .sqrt();
            state.modules[m].speed = max_vel.min(state.modules[m].speed);
        }

        let max_chassis_vel = state
            .constraints
            .max_velocity_mps
            .min(state.state.linear_velocity);
        let max_chassis_ang_vel = state
            .constraints
            .max_angular_velocity_rad_per_sec
            .min(state.state.field_speeds.omega.abs());
        equalize_and_desaturate(
            state,
            &next_modules,
            &prev_modules,
            config,
            max_chassis_vel,
            max_chassis_ang_vel,
        );
    }
}

fn desaturate_wheel_speeds(
    modules: &mut [Module],
    desired: ChassisSpeeds,
    max_module_speed_mps: f64,
    max_translation_speed: f64,
    max_rotation_speed: f64,
) {
    let real_max_speed = modules.iter().fold(0.0_f64, |m, s| m.max(s.speed.abs()));
    if real_max_speed == 0.0 {
        return;
    }

    let mut translation_pct = 0.0;
    if max_translation_speed.abs() > 1e-8 {
        translation_pct =
            (desired.vx * desired.vx + desired.vy * desired.vy).sqrt() / max_translation_speed;
    }

    let mut rotation_pct = 0.0;
    if max_rotation_speed.abs() > 1e-8 {
        rotation_pct = desired.omega.abs() / max_rotation_speed.abs();
    }

    let max_pct = f64::max(translation_pct, rotation_pct);

    let mut scale = f64::min(1.0, max_module_speed_mps / real_max_speed);
    if max_pct > 0.0 {
        scale = scale.min(1.0 / max_pct);
    }

    for module in modules {
        module.speed *= scale;
    }
}

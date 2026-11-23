//! Shot parameters (drive angle, hood angle, flywheel speed) for the current pose and velocity.
//!
//! Java's singleton is [`context::shot_calculator`]. [`ShotCalculator::calculate`] holds the
//! math and the filter state and logs nothing, so it runs without hardware.

use super::target_factories::{feed_target_factory, hub_target_factory};
use crate::context;
use commands::{Command, Commands};
use std::cell::{Cell, RefCell};
use wpilib::dashboard::{SendableChooser, smart_dashboard};
use wpimath::filter::LinearFilter;
use wpimath::geometry::{Pose2d, Rotation2d, Transform2d, Translation2d, Twist2d};
use wpimath::kinematics::ChassisSpeeds;
use wpimath::math_util;

/// Launcher offset in the robot frame. Zero puts the launcher at the robot centre.
const ROBOT_TO_LAUNCHER: Transform2d = Transform2d::ZERO;

/// Snapshot of all quantities needed to command the drive, hood, and flywheel subsystems for a
/// single shot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShootingParameters {
    /// True when distance is within the polynomial's fitted range.
    pub is_valid: bool,
    /// Field-relative heading the robot must face to aim at the goal.
    pub drive_angle: Rotation2d,
    /// Rate of change of `drive_angle` (rad/s) for heading feedforward.
    pub drive_angular_velocity: f64,
    /// Commanded hood/pivot angle (degrees), including the hood angle offset.
    pub hood_angle: f64,
    /// Rate of change of `hood_angle` (deg/s) for pivot feedforward.
    pub hood_velocity: f64,
    /// Commanded flywheel speed (RPM).
    pub flywheel_speed: f64,
    /// Ball exit speed from the polynomial (m/s), before RPM conversion.
    pub exit_speed_ms: f64,
    /// Shoot-on-move compensated distance to goal (metres).
    pub distance: f64,
    /// Raw uncompensated distance to goal (metres).
    pub distance_no_lookahead: f64,
    /// Estimated ball time-of-flight (seconds).
    pub time_of_flight: f64,
}

pub const STARTING_HOOD_ANGLE_OFFSET: f64 = -2.0; // degrees
pub const STARTING_DRIVE_ANGLE_OFFSET: f64 = 0.0; // degrees

thread_local! {
    static HOOD_ANGLE_OFFSET: Cell<f64> = const { Cell::new(STARTING_HOOD_ANGLE_OFFSET) };
    static DRIVE_ANGLE_OFFSET: Cell<f64> = const { Cell::new(STARTING_DRIVE_ANGLE_OFFSET) };
}

pub fn hood_angle_offset() -> f64 {
    HOOD_ANGLE_OFFSET.with(Cell::get)
}

pub fn drive_angle_offset() -> f64 {
    DRIVE_ANGLE_OFFSET.with(Cell::get)
}

pub fn increase_hood_angle_offset() -> Command {
    Commands::run_once(|| HOOD_ANGLE_OFFSET.with(|o| o.set(o.get() + 0.1)), &[])
        .ignoring_disable(true)
}

pub fn decrease_hood_angle_offset() -> Command {
    Commands::run_once(|| HOOD_ANGLE_OFFSET.with(|o| o.set(o.get() - 0.1)), &[])
        .ignoring_disable(true)
}

pub fn increase_drive_angle_offset() -> Command {
    Commands::run_once(|| DRIVE_ANGLE_OFFSET.with(|o| o.set(o.get() + 1.0)), &[])
        .ignoring_disable(true)
}

pub fn decrease_drive_angle_offset() -> Command {
    Commands::run_once(|| DRIVE_ANGLE_OFFSET.with(|o| o.set(o.get() - 1.0)), &[])
        .ignoring_disable(true)
}

/// Global exit-speed scale factor for both the hub and feed models. 1.0 leaves the fitted
/// polynomial unscaled; raise it to correct for ball compression, wear, or temperature.
const MPS_FACTOR: f64 = 0.8;

/// Scale factor converting polynomial exit speed (m/s) to flywheel RPM.
const RPM_PER_MPS: f64 = 255.0;

/// A fitted degree-3 polynomial surface plus its input domain and normalisation. Inputs are
/// mapped to zero-mean unit-variance before evaluation, so the coefficients are only valid for
/// normalised inputs.
///
/// Coefficient order is the monomial basis 1, d, v, d^2, d*v, v^2, d^3, d^2*v, d*v^2, v^3,
/// which has to match the expansion in [`eval_poly_raw`].
#[derive(Debug)]
pub struct PolyModel {
    /// Label shown on the dashboard.
    pub name: &'static str,
    /// Fitted distance lower bound (metres); `eval_poly_raw` clamps to it.
    pub dist_min: f64,
    pub dist_max: f64,
    /// Fitted radial-velocity bounds (m/s).
    pub rv_min: f64,
    pub rv_max: f64,
    pub d_mean: f64,
    pub d_std: f64,
    pub v_mean: f64,
    pub v_std: f64,
    pub speed_coeffs: [f64; 10],
    pub angle_coeffs: [f64; 10],
}

static NO_CEILING_HUB_MODEL: PolyModel = PolyModel {
    name: "No Ceiling Hub Model",
    dist_min: 1.5,
    dist_max: 8.0,
    rv_min: -3.0,
    rv_max: 3.0,
    d_mean: 4.7946224256,
    d_std: 1.9514199579,
    v_mean: -0.0434782609,
    v_std: 1.9813242725,
    speed_coeffs: [
        1.1143628795e+1,
        1.0138658152e+0,
        -3.2159777567e-1,
        -5.1612304349e-2,
        3.6484374359e-1,
        -1.7402563290e-1,
        7.9863642916e-2,
        -1.2476148148e-1,
        3.8502387398e-1,
        -3.2039252056e-1,
    ],
    angle_coeffs: [
        6.6464926591e+1,
        -7.7256852841e+0,
        1.1734641473e+1,
        8.1692458382e-3,
        5.2897492237e-1,
        -2.6845198119e-1,
        1.7588060294e-1,
        -2.0564699991e-3,
        1.2509312471e+0,
        -1.4532157984e+0,
    ],
};

static CEILING_3M_HUB_MODEL: PolyModel = PolyModel {
    name: "3 Meter Ceiling Hub Model",
    dist_min: 1.5,
    dist_max: 8.0,
    rv_min: -3.0,
    rv_max: 3.0,
    d_mean: 4.7330253114,
    d_std: 1.8890844725,
    v_mean: 0.0229007634,
    v_std: 1.9319161427,
    speed_coeffs: [
        9.1291597222e+0,
        1.4704411927e+0,
        -1.1245274618e+0,
        5.9711528113e-2,
        -1.0395358193e-1,
        6.1638746813e-2,
        -3.2358373131e-2,
        1.7465238201e-2,
        3.5205649680e-2,
        -1.6138070441e-2,
    ],
    angle_coeffs: [
        5.4780057238e+1,
        -8.0553943910e+0,
        9.8969071974e+0,
        9.3479980881e-1,
        -2.3620060557e+0,
        6.9825040437e-1,
        -4.9506580821e-1,
        -6.4209468217e-1,
        9.0324521099e-1,
        -5.4437632941e-1,
    ],
};

static FEED_MODEL: PolyModel = PolyModel {
    name: "Feed Model",
    dist_min: 5.0,
    dist_max: 10.0,
    rv_min: -3.0,
    rv_max: 3.0,
    d_mean: 7.5,
    d_std: 1.5430334996,
    v_mean: 0.0,
    v_std: 2.0,
    speed_coeffs: [
        1.2074547373e+1,
        1.1124598419e+0,
        -9.2720217607e-1,
        -5.8674357317e-2,
        -7.2912571960e-2,
        -7.0818070818e-2,
        5.6161425197e-2,
        -6.0497571810e-3,
        2.1986814335e-1,
        -1.5954415954e-1,
    ],
    angle_coeffs: [
        5.7369141664e+1,
        -4.3735130912e+0,
        1.0080892292e+1,
        7.8858336234e-2,
        -1.5120778737e+0,
        3.9384615385e-1,
        2.6249905834e-1,
        2.9750087017e-1,
        1.0186869775e+0,
        -1.2099829060e+0,
    ],
};

/// What `get_parameters` logs besides the parameters themselves.
#[derive(Debug, Clone, Copy)]
pub struct ShotDebug {
    pub lookahead_pose: Pose2d,
    pub yaw_offset_deg: f64,
    pub radial_velocity: f64,
    pub tangential_velocity: f64,
    pub model_name: &'static str,
    pub target: Translation2d,
    pub feed: bool,
}

const LOOP_PERIOD_SECS: f64 = 0.02;

/// Phase delay applied to the estimated robot pose before computing shot parameters, to cover
/// sensor and network latency (seconds).
const PHASE_DELAY_SECS: f64 = 0.03;

pub struct ShotCalculator {
    latest_parameters: Cell<Option<ShootingParameters>>,
    /// Dashboard selector for the hub model, so the ceiling-limited surface can be picked when
    /// testing indoors without a redeploy. Feed shots always use `FEED_MODEL`.
    hub_model_chooser: SendableChooser<&'static PolyModel>,
    hood_angle_filter: RefCell<LinearFilter>,
    drive_angle_filter: RefCell<LinearFilter>,
    last_hood_angle: Cell<f64>,
    last_drive_angle: Cell<Option<Rotation2d>>,
}

impl ShotCalculator {
    /// Builds the calculator and puts the hub model chooser on the dashboard.
    pub fn new() -> Self {
        let calculator = Self::without_dashboard();
        smart_dashboard::put_data("Hub Model Chooser", &calculator.hub_model_chooser);
        calculator
    }

    /// Same state as [`new`](Self::new), with nothing on the dashboard.
    pub fn without_dashboard() -> Self {
        let hub_model_chooser = SendableChooser::new();
        hub_model_chooser.set_default_option(NO_CEILING_HUB_MODEL.name, &NO_CEILING_HUB_MODEL);
        hub_model_chooser.add_option(CEILING_3M_HUB_MODEL.name, &CEILING_3M_HUB_MODEL);
        Self {
            latest_parameters: Cell::new(None),
            hub_model_chooser,
            hood_angle_filter: RefCell::new(LinearFilter::moving_average(
                (0.1 / LOOP_PERIOD_SECS) as usize, // ~100 ms window
            )),
            drive_angle_filter: RefCell::new(LinearFilter::moving_average(
                (0.1 / LOOP_PERIOD_SECS) as usize, // ~100 ms window
            )),
            last_hood_angle: Cell::new(f64::NAN),
            last_drive_angle: Cell::new(None),
        }
    }

    /// Falls back to `NO_CEILING_HUB_MODEL` when nothing has been selected yet.
    pub fn selected_hub_model(&self) -> &'static PolyModel {
        self.hub_model_chooser
            .get_selected()
            .unwrap_or(&NO_CEILING_HUB_MODEL)
    }

    /// Shot parameters for the current pose and velocity, cached until
    /// [`clear_shooting_parameters`](Self::clear_shooting_parameters) is called.
    ///
    /// 1. Delay the odometry pose by `PHASE_DELAY_SECS` to cover sensor latency.
    /// 2. Split the launcher's field-relative velocity into radial and tangential parts.
    /// 3. Run the 1690 Orbit virtual-target solver, which corrects the aim for robot motion
    ///    during flight.
    /// 4. Turn that into a drive angle, a hood angle, and a flywheel speed.
    pub fn get_parameters(&self) -> ShootingParameters {
        if let Some(latest) = self.latest_parameters.get() {
            return latest;
        }

        let feed = context::super_structure().is_robot_in_feed_zone();
        let target = if feed {
            feed_target_factory::generate()
        } else {
            let hub = hub_target_factory::generate();
            Translation2d::new(hub.x, hub.y)
        };
        // Feed and hub shots use separately-fitted polynomial surfaces.
        let model = if feed {
            &FEED_MODEL
        } else {
            self.selected_hub_model()
        };

        let swerve = context::swerve();
        let estimated_pose = swerve.get_robot_pose();
        let robot_relative_velocity = swerve.get_current_robot_chassis_speeds();

        let (parameters, debug) =
            self.calculate(model, feed, target, estimated_pose, robot_relative_velocity);
        self.log(&parameters, &debug);
        parameters
    }

    /// The math of [`get_parameters`](Self::get_parameters) for the given inputs. Advances the
    /// angle filters and caches the result.
    pub fn calculate(
        &self,
        model: &PolyModel,
        feed: bool,
        target: Translation2d,
        estimated_pose: Pose2d,
        robot_relative_velocity: ChassisSpeeds,
    ) -> (ShootingParameters, ShotDebug) {
        let estimated_pose = estimated_pose.exp(Twist2d::new(
            robot_relative_velocity.vx * PHASE_DELAY_SECS,
            robot_relative_velocity.vy * PHASE_DELAY_SECS,
            robot_relative_velocity.omega * PHASE_DELAY_SECS,
        ));

        let launcher_pose = estimated_pose.transform_by(ROBOT_TO_LAUNCHER);
        let launcher_to_target = target - launcher_pose.translation;
        let distance_no_lookahead = launcher_to_target.norm();

        // Launcher velocity picks up an omega x r term from rotating about the robot centre
        let field_velocity =
            ChassisSpeeds::from_robot_relative(robot_relative_velocity, estimated_pose.rotation);
        let robot_angle = estimated_pose.rotation.radians();
        let robot_to_launcher = ROBOT_TO_LAUNCHER.translation;
        let launcher_velocity_x = field_velocity.vx
            - field_velocity.omega
                * (robot_to_launcher.x * robot_angle.sin()
                    + robot_to_launcher.y * robot_angle.cos());
        let launcher_velocity_y = field_velocity.vy
            + field_velocity.omega
                * (robot_to_launcher.x * robot_angle.cos()
                    - robot_to_launcher.y * robot_angle.sin());

        let ux = launcher_to_target.x / distance_no_lookahead;
        let uy = launcher_to_target.y / distance_no_lookahead;
        // Positive radial_velocity = closing on target
        let radial_velocity = launcher_velocity_x * ux + launcher_velocity_y * uy;
        // Tangential: the radial axis rotated a quarter turn
        let tangential_velocity = -launcher_velocity_x * uy + launcher_velocity_y * ux;

        let poly = solve_virtual_target(
            model,
            distance_no_lookahead,
            radial_velocity,
            tangential_velocity,
        );
        let exit_speed_ms = poly[0];
        let raw_hood_angle = 90.0 - poly[1]; // degrees, before the hood angle offset
        let yaw_offset_deg = poly[2];
        let lookahead_dist = poly[3];
        let tof_final = poly[4];

        let drive_angle = launcher_to_target.angle()
            + Rotation2d::from_degrees(yaw_offset_deg)
            + Rotation2d::from_degrees(drive_angle_offset())
            + Rotation2d::PI;

        // Useful for Field2d visualization and validating shoot-on-move compensation.
        let lookahead_pose = Pose2d::new(
            launcher_pose.translation
                + Translation2d::new(
                    launcher_velocity_x * tof_final,
                    launcher_velocity_y * tof_final,
                ),
            drive_angle,
        );

        // Drive angular velocity (rad/s) for heading feedforward
        let last_drive_angle = self.last_drive_angle.get().unwrap_or(drive_angle);
        let delta_rot =
            math_util::input_modulus((drive_angle - last_drive_angle).rotations(), -0.5, 0.5);
        let drive_angular_velocity = self
            .drive_angle_filter
            .borrow_mut()
            .calculate(delta_rot / LOOP_PERIOD_SECS);
        self.last_drive_angle.set(Some(drive_angle));

        // Differentiate the raw angle so the near-constant hood angle offset does not
        // bleed into the derivative
        if self.last_hood_angle.get().is_nan() {
            self.last_hood_angle.set(raw_hood_angle);
        }
        let hood_velocity = self
            .hood_angle_filter
            .borrow_mut()
            .calculate((raw_hood_angle - self.last_hood_angle.get()) / LOOP_PERIOD_SECS);
        self.last_hood_angle.set(raw_hood_angle);
        let hood_angle = (raw_hood_angle + hood_angle_offset()).max(9.0);

        let flywheel_speed = exit_speed_ms * RPM_PER_MPS;

        let is_valid =
            distance_no_lookahead >= model.dist_min && distance_no_lookahead <= model.dist_max;

        let parameters = ShootingParameters {
            is_valid,
            drive_angle,
            drive_angular_velocity,
            hood_angle,
            hood_velocity,
            flywheel_speed,
            exit_speed_ms,
            distance: lookahead_dist,
            distance_no_lookahead,
            time_of_flight: tof_final,
        };
        self.latest_parameters.set(Some(parameters));

        (
            parameters,
            ShotDebug {
                lookahead_pose,
                yaw_offset_deg,
                radial_velocity,
                tangential_velocity,
                model_name: model.name,
                target,
                feed,
            },
        )
    }

    fn log(&self, p: &ShootingParameters, d: &ShotDebug) {
        use telemetry::telemetry as t;
        t::log_struct("ShotCalc/LookaheadPose", &d.lookahead_pose);
        t::log_dash_with_unit("ShotCalc/DistanceMeters", p.distance, "meters");
        t::log_with_unit(
            "ShotCalc/DistanceNoLookahead",
            p.distance_no_lookahead,
            "meters",
        );
        t::log_dash_with_unit("ShotCalc/DriveAngleDeg", p.drive_angle.degrees(), "degrees");
        t::log_with_unit("ShotCalc/YawOffsetDeg", d.yaw_offset_deg, "degrees");
        t::log_dash_with_unit("ShotCalc/HoodAngleDeg", p.hood_angle, "degrees");
        t::log_dash_with_unit("ShotCalc/FlywheelSpeedRPM", p.flywheel_speed, "RPM");
        t::log_dash_with_unit("ShotCalc/ExitSpeedMs", p.exit_speed_ms, "m/s");
        t::log_with_unit("ShotCalc/RadialVelocityMs", d.radial_velocity, "m/s");
        t::log_with_unit(
            "ShotCalc/TangentialVelocityMs",
            d.tangential_velocity,
            "m/s",
        );
        t::log_dash_with_unit("ShotCalc/TimeOfFlight", p.time_of_flight, "seconds");
        t::log_dash("ShotCalc/FeedShot", d.feed);
        t::log("ShotCalc/HubPolyModel", d.model_name);
        t::log_dash_with_unit(
            "ShotCalc/DriveAngleOffsetDegrees",
            drive_angle_offset(),
            "degrees",
        );
        t::log_dash_with_unit(
            "ShotCalc/HoodAngleOffsetDegrees",
            hood_angle_offset(),
            "degrees",
        );
        t::log_struct("ShotCalc/Target", &d.target);
    }

    pub fn clear_shooting_parameters(&self) {
        self.latest_parameters.set(None);
    }
}

impl Default for ShotCalculator {
    fn default() -> Self {
        Self::new()
    }
}

/// 1690 Orbit virtual-target solver. Each pass evaluates the polynomial at the aim point,
/// estimates time of flight, then shifts the aim point by how far the launcher moves during that
/// flight. Converges in two or three of the five allowed iterations.
///
/// `distance` is the horizontal distance to the goal centre (m), `radial_velocity` the launcher
/// velocity toward the goal (m/s, positive is closing), `tangential_velocity` the launcher
/// velocity across the goal line (m/s).
///
/// Returns exit speed (m/s), launch angle (degrees), yaw offset (degrees), converged lookahead
/// distance (m), time of flight (s), in that order.
fn solve_virtual_target(
    model: &PolyModel,
    distance: f64,
    radial_velocity: f64,
    tangential_velocity: f64,
) -> [f64; 5] {
    let mut vdx = distance; // virtual aim point, radial (m)
    let mut vdz = 0.0; // virtual aim point, lateral (m)
    let mut tof = 0.0;

    for iter in 0..5 {
        let v_dist = (vdx * vdx + vdz * vdz).sqrt();
        if v_dist < 0.1 {
            break;
        }

        // Evaluate at rv = 0; the shifted aim point already carries the robot's motion
        let raw = eval_poly_raw(model, v_dist, 0.0);
        let speed = raw[0] * MPS_FACTOR;
        let cos_a = (raw[1] * std::f64::consts::PI / 180.0).cos();
        let prev_tof = tof;
        tof = v_dist / (speed * cos_a).max(0.5); // guard against div-by-zero

        // Aim at where the target will be when the ball arrives
        vdx = distance - radial_velocity * tof;
        vdz = -tangential_velocity * tof;

        if iter > 0 && (tof - prev_tof).abs() < 0.002 {
            break;
        }
    }

    let virtual_dist = (vdx * vdx + vdz * vdz).sqrt();
    let yaw_offset_deg = (-tangential_velocity * tof).atan2(distance - radial_velocity * tof)
        * (180.0 / std::f64::consts::PI);

    let result = eval_poly_raw(model, virtual_dist, 0.0);
    [
        result[0] * MPS_FACTOR, // m/s
        result[1],              // degrees
        yaw_offset_deg,         // degrees
        virtual_dist,           // m
        tof,                    // s
    ]
}

/// Evaluates the surface at (distance, radial_vel), clamping both to the model's fitted range.
/// Output is raw, so callers apply `MPS_FACTOR` to the exit speed. Returns exit speed (m/s)
/// then launch angle (degrees).
fn eval_poly_raw(model: &PolyModel, distance: f64, radial_vel: f64) -> [f64; 2] {
    let d_raw = model.dist_min.max(model.dist_max.min(distance));
    let v_raw = model.rv_min.max(model.rv_max.min(radial_vel));
    let d = (d_raw - model.d_mean) / model.d_std;
    let v = (v_raw - model.v_mean) / model.v_std;

    let d2 = d * d;
    let v2 = v * v;
    let d3 = d2 * d;
    let v3 = v2 * v;

    let terms = [
        1.0,    // 1
        d,      // d
        v,      // v
        d2,     // d^2
        d * v,  // d*v
        v2,     // v^2
        d3,     // d^3
        d2 * v, // d^2*v
        d * v2, // d*v^2
        v3,     // v^3
    ];

    let mut exit_speed = 0.0;
    let mut launch_angle = 0.0;
    for (i, term) in terms.iter().enumerate() {
        exit_speed += model.speed_coeffs[i] * term;
        launch_angle += model.angle_coeffs[i] * term;
    }
    [exit_speed, launch_angle]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::subsystems::swerve::swerve::{
        enemy_alliance_zone_contains, is_in_field_left, neutral_zone_contains,
    };
    use golden::check;

    fn model(code: f64) -> &'static PolyModel {
        match code as i32 {
            0 => &NO_CEILING_HUB_MODEL,
            1 => &CEILING_3M_HUB_MODEL,
            _ => &FEED_MODEL,
        }
    }

    #[test]
    fn matches_java_poly_and_solver() {
        check("shot_calculator_poly", 1e-9, |op, i| {
            Some(match op {
                "eval_poly_raw" => eval_poly_raw(model(i[0]), i[1], i[2]).to_vec(),
                "solve_virtual_target" => {
                    solve_virtual_target(model(i[0]), i[1], i[2], i[3]).to_vec()
                }
                _ => return None,
            })
        });
    }

    /// Scenarios run through one calculator, so the filters carry state across steps like the
    /// robot loop does. `scenario_start` makes a fresh one.
    #[test]
    fn matches_java_get_parameters() {
        let mut calculator = ShotCalculator::without_dashboard();
        check("shot_calculator_params", 1e-9, |op, i| {
            Some(match op {
                // hub model, hood offset, drive offset
                "scenario_start" => {
                    calculator = ShotCalculator::without_dashboard();
                    calculator
                        .hub_model_chooser
                        .set_default_option(model(i[0]).name, model(i[0]));
                    HOOD_ANGLE_OFFSET.with(|o| o.set(i[1]));
                    DRIVE_ANGLE_OFFSET.with(|o| o.set(i[2]));
                    vec![]
                }
                // red, pose x y theta, robot relative vx vy omega
                "step" => {
                    let red = i[0] != 0.0;
                    let pose = Pose2d::new(Translation2d::new(i[1], i[2]), Rotation2d::new(i[3]));
                    let in_enemy = enemy_alliance_zone_contains(red, pose);
                    let feed = in_enemy || neutral_zone_contains(pose);
                    let target = if feed {
                        feed_target_factory::generate_for(
                            red,
                            is_in_field_left(pose),
                            in_enemy,
                            pose.translation,
                        )
                    } else {
                        let hub = hub_target_factory::generate_for(red, pose.translation);
                        Translation2d::new(hub.x, hub.y)
                    };
                    let chosen = if feed {
                        &FEED_MODEL
                    } else {
                        calculator.selected_hub_model()
                    };
                    calculator.clear_shooting_parameters();
                    let (p, _) = calculator.calculate(
                        chosen,
                        feed,
                        target,
                        pose,
                        ChassisSpeeds {
                            vx: i[4],
                            vy: i[5],
                            omega: i[6],
                        },
                    );
                    vec![
                        f64::from(u8::from(p.is_valid)),
                        p.drive_angle.radians(),
                        p.drive_angular_velocity,
                        p.hood_angle,
                        p.hood_velocity,
                        p.flywheel_speed,
                        p.exit_speed_ms,
                        p.distance,
                        p.distance_no_lookahead,
                        p.time_of_flight,
                        target.x,
                        target.y,
                    ]
                }
                _ => return None,
            })
        });
    }

    #[test]
    fn the_hub_model_defaults_to_no_ceiling() {
        let calculator = ShotCalculator::without_dashboard();
        assert_eq!(calculator.selected_hub_model().name, "No Ceiling Hub Model");
        calculator
            .hub_model_chooser
            .set_default_option(CEILING_3M_HUB_MODEL.name, &CEILING_3M_HUB_MODEL);
        assert_eq!(
            calculator.selected_hub_model().name,
            "3 Meter Ceiling Hub Model"
        );
    }

    #[test]
    fn a_still_robot_aims_straight_at_the_target_and_reports_zero_rates() {
        let calculator = ShotCalculator::without_dashboard();
        let (p, d) = calculator.calculate(
            &NO_CEILING_HUB_MODEL,
            false,
            Translation2d::new(4.0, 0.0),
            Pose2d::new(Translation2d::ZERO, Rotation2d::ZERO),
            ChassisSpeeds::default(),
        );
        assert!(p.is_valid);
        assert_eq!(p.distance_no_lookahead, 4.0);
        assert_eq!(d.yaw_offset_deg, 0.0);
        assert_eq!(p.drive_angular_velocity, 0.0);
        assert_eq!(p.hood_velocity, 0.0);
        assert!((p.drive_angle.radians().abs() - std::f64::consts::PI).abs() < 1e-12);
    }

    #[test]
    fn out_of_range_distance_is_not_valid() {
        let calculator = ShotCalculator::without_dashboard();
        let (near, _) = calculator.calculate(
            &NO_CEILING_HUB_MODEL,
            false,
            Translation2d::new(1.0, 0.0),
            Pose2d::new(Translation2d::ZERO, Rotation2d::ZERO),
            ChassisSpeeds::default(),
        );
        assert!(!near.is_valid);
    }

    #[test]
    fn the_hood_never_goes_below_nine_degrees() {
        let calculator = ShotCalculator::without_dashboard();
        let (p, _) = calculator.calculate(
            &FEED_MODEL,
            true,
            Translation2d::new(40.0, 0.0),
            Pose2d::new(Translation2d::ZERO, Rotation2d::ZERO),
            ChassisSpeeds::default(),
        );
        assert!(p.hood_angle >= 9.0);
    }

    #[test]
    fn offset_commands_step_by_a_tenth_and_a_degree() {
        HOOD_ANGLE_OFFSET.with(|o| o.set(STARTING_HOOD_ANGLE_OFFSET));
        DRIVE_ANGLE_OFFSET.with(|o| o.set(STARTING_DRIVE_ANGLE_OFFSET));
        let commands = [
            increase_hood_angle_offset(),
            increase_drive_angle_offset(),
            increase_drive_angle_offset(),
            decrease_drive_angle_offset(),
        ];
        for command in &commands {
            command.initialize();
            command.execute();
            command.end(false);
        }
        assert!((hood_angle_offset() - (STARTING_HOOD_ANGLE_OFFSET + 0.1)).abs() < 1e-12);
        assert_eq!(drive_angle_offset(), 1.0);
        let down = decrease_hood_angle_offset();
        down.initialize();
        down.execute();
        down.end(false);
        assert!((hood_angle_offset() - STARTING_HOOD_ANGLE_OFFSET).abs() < 1e-12);
    }
}

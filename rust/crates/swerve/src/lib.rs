//! Reimplementation of CTRE's Phoenix 6 swerve API behavior from CTRE's shipped headers, Java
//! sources and documentation. CTRE ships swerve only as C++ templates and a JNI library, which
//! Rust cannot call. Parts CTRE compiles (module apply, odometry thread, kinematics) are
//! reconstructed from header declarations, docs and observable behavior; each module says so.
//!
//! Units are SI at this API (meters, m/s, radians, seconds). Rotations appear only where the
//! phoenix6 crate takes them. Timestamps are on the Phoenix clock, like CTRE's; convert FPGA
//! time with [`fpga_to_current_time`].
//!
//! Ported from CTRE sources under the CTRE license. See `THIRD_PARTY.md`.

pub mod constants;
pub mod drivetrain;
pub mod module;
pub mod module_math;
pub mod odometry;
pub mod pid;
pub mod request;

pub use constants::{
    ClosedLoopOutputType, DriveMotorModel, DrivetrainConstants, ModuleConstants,
    ModuleConstantsFactory, SteerFeedbackType,
};
pub use drivetrain::{DrivetrainOptions, SharedRequest, SwerveDrivetrain};
pub use module::{ModuleSink, SwerveModule};
pub use module_math::{DriveRequestType, ModuleRequest, SteerRequestType};
pub use odometry::SwerveDriveState;
pub use phoenix6::utils::{current_time_to_fpga_time, fpga_to_current_time};
pub use pid::PhoenixPidController;
pub use request::{
    ApplyFieldSpeeds, ApplyRobotSpeeds, ControlParameters, FieldCentric, FieldCentricFacingAngle,
    ForwardPerspective, Idle, NUM_MODULES, SwerveDriveBrake, SwerveRequest, SysIdSwerveRotation,
    SysIdSwerveSteerGains, SysIdSwerveTranslation, TargetDirectionPerspective,
};

# Third-party code

Code ported into this workspace, and the license each part keeps.

| Source | Version | License | Where |
|---|---|---|---|
| WPILib (wpimath, wpilibj, command framework) | v2026.2.1 | BSD-3 (`../WPILib-License.md`) | `crates/wpimath`, `crates/wpilib`, `crates/commands`, `crates/wpilib/src/color.rs` |
| CTRE Phoenix 6 swerve (request `Apply` bodies and `PhoenixPIDController` from the 26.3.0 headers and Java; module math checked against the 24.3.0 Java `SwerveModule`) | 26.3.0, 24.3.0 | CTRE license | `crates/swerve` |
| WPILib wpiutil and wpilibj logging (DataLog format and background writer, `DataLogManager`, the `DriverStation` data log, `FileLogger`) | v2026.2.1 | BSD-3 (`../WPILib-License.md`) | `crates/telemetry` |
| DogLog (queue, log thread, wpilog and NetworkTables writers, extras, faults, tunables) | 2026.5.0 | MIT, `LICENSE` on the `main` branch of github.com/jonahsnider/doglog (Copyright (c) 2026 Jonah Snider; the 2026.5.0 tag carries no license file) | `crates/telemetry/src/dog_log.rs`, `writers.rs`, `extras.rs` |
| `BatteryLogger` from the team's Java, derived from Littleton Robotics code | team repo | MIT-style, Littleton Robotics header kept in the file | `crates/telemetry/src/battery_logger.rs` |
| `ShiftHelpers` from the team's Java, derived from Littleton Robotics code | team repo | MIT-style, Littleton Robotics header kept in the file | `crates/robot/src/rebuilt/shift_helpers.rs` |
| LimelightHelpers (functions the robot uses) | v1.14 | MIT, Copyright (c) 2026 Limelight Vision (<https://github.com/LimelightVision/limelightlib-wpijava>) | `crates/robot/src/spectrum_lib/vision/limelight_helpers.rs` |
| Team 254 FRC-2023-Public `CanDeviceId`, and 254's `Util` by way of Team 1678 C2024-Public (credited in the team's Java) | 2023, 2024 | MIT | `crates/robot/src/spectrum_lib/util/{can_device_id,util}.rs` |
| Team 100 all24 `Identity` (credited in the team's Java) | commit 2a109b2 | Unlicense, with 254-derived parts under MIT | `crates/robot/src/spectrum_lib/hardware/rio.rs` |
| FRC team 2363 curve code (credited in the team's Java; upstream source and license not identified) | unknown | unknown | `crates/robot/src/spectrum_lib/util/{curve,exp_curve}.rs` |
| `FuelPhysicsSim` from the team's Java | 2026 | MIT, Copyright (c) 2026 FRC Team 5962 perSEVERE | `crates/robot/src/rebuilt/fuel_physics_sim.rs` |
| `RobotBumpSim` from the team's Java (<https://github.com/haar09/FRC-Rebuilt-BumpSim>) | 2025-2026 | MIT-style, Copyright (c) 2025-2026 KAISER 6989 | `crates/robot/src/rebuilt/robot_bump_sim.rs` |
| maple-sim (swerve drivetrain, module, motor, battery, gyro and arena simulation; `MapleSimSwerveDrivetrain` from the team's Java) and the part of dyn4j it runs on (rigid body, SAT contacts, clipping manifold, sequential impulses, conservative advancement) | maple-sim v0.4.0-beta, dyn4j 5.0.2 | maple-sim MIT, Copyright (c) 2024-2025 Team5516 "Iron Maple" (`LICENSE-IronMaple.md`); dyn4j BSD-3, Copyright (c) 2010-2024 William Bittle (`LICENSE.md`) | `crates/robot/src/spectrum_lib/swerve/{maple_sim,maple_sim_swerve_drivetrain,physics}.rs` |
| Eigen `MatrixExponential` (Padé coefficients and degree thresholds, as bundled in WPILib) | WPILib v2026.2.1 copy | MPL-2.0 (<https://mozilla.org/MPL/2.0/>) | `crates/wpimath/src/system/discretization.rs` |
| PathPlannerLib (paths, trajectories, controller, commands, AD* pathfinding) | v2026.1.2 | MIT (`crates/pathplanner/LICENSE`) | `crates/pathplanner` |

`crates/phoenix6` calls CTRE's shipped Phoenix 6 libraries through the C
function signatures in CTRE's shipped headers. Its tables (SPN numbers, enum
values, status codes, config field defaults, control request fields) are
generated from the Phoenix 6 26.3.0 Java API and headers, which are under
CTRE's license (`CTRE_LICENSE.txt` in the CTRE Maven artifacts). The
libraries are downloaded by `cargo xtask fetch-natives` and stay under that
license.

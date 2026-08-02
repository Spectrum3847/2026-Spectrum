---
name: maple-sim
description: "Use for Team 8044 MapleSim integration work: maintaining the swerve-first MapleSim simulation, verifying WPILib headless sim logs, calibrating physics constants, and extending simulation to intake, projectiles, opponents, or vision."
metadata:
  short-description: Maintain MapleSim simulation
---

# MapleSim

## Core Rules

- Keep this skill updated whenever a future agent discovers a repeatable MapleSim workflow, command, API mismatch, or Team 8044-specific simulation convention.
- Preserve the existing AdvantageKit IO abstraction. Do not rewrite `Swerve`, `Module`, Choreo autos, or `SimAgentBridge` unless the task explicitly requires it.
- For drivetrain simulation, prefer MapleSim as the physics source while preserving the existing IO log surface. Team 8044 SIM swerve uses `ModuleIOMapleSim` and `GyroIOMapleSim` for MapleSim-specific physics/control, and keeps `ModuleIOSim` and `GyroIOSim` as the normal lightweight non-MapleSim sim path; do not reintroduce the Phoenix remote-CANcoder sim bridge unless the steering instability that caused azimuth wind/crawl has been solved.
- Check `MapleSimConstants.useMapleSim` before debugging SIM behavior. When true, SIM uses MapleSim arena/physics/collisions/fuel/projectiles. When false, SIM uses lightweight `DCMotorSim` module IO without creating/registering `SwerveDriveSimulation`, `SimulatedArena`, fuel simulation, projectiles, or shot-map verifier hooks.
- Keep real and replay modes isolated from MapleSim. MapleSim belongs in `Constants.RobotMode.SIM`.
- Record durable progress and verification notes in `.agents/maple-sim-implementation.md` when changing MapleSim behavior.

## Current Integration Shape

- Vendor dependency: `vendordeps/maple-sim.json`.
- SIM orchestration owner when `MapleSimConstants.useMapleSim` is true: `RobotContainer` creates `MapleSimRobotSimulation`, then wires `Swerve` IO from its exposed `SwerveDriveSimulation` and module simulations. Keep arena registration, MapleSim periodic updates, field pose/velocity logging, fuel simulation attachment, and SIM-only verifier hooks in the coordinator package, not in `RobotContainer`.
- SIM drivetrain owner when `MapleSimConstants.useMapleSim` is false: `RobotContainer` creates `Swerve` with `ModuleIOSim` and `GyroIOSim` so existing kinematic yaw fallback works. No MapleSim arena is created, registered, or advanced.
- MapleSim module IO shape: `ModuleIOMapleSim extends ModuleIOTalonFX`, feeds MapleSim state into Phoenix TalonFX/CANcoder sim states, calls `super.updateInputs(...)`, then patches fresh MapleSim physics/current/connection values into `ModuleIOInputs`.
- MapleSim reset coordination lives in `MapleSimRobotSimulation.resetPose(...)` plus registered pose-reset handlers. Do not put per-device reset calls inline in `RobotContainer`.
- MapleSim module IO should report drive position directly from `SwerveModuleSimulation`; MapleSim body pose teleports do not require drive-position offsets unless a future log proves otherwise.
- MapleSim gyro IO shape: `GyroIOMapleSim` receives `SwerveDriveSimulation` and pulls its matching MapleSim `GyroSimulation` internally. With `MapleSimConstants.enableGyroDrift = false` it reports near-driftless Pigeon2-style yaw from the simulated drivetrain pose plus a SIM reset offset; register `GyroIOMapleSim.syncToSimulationPoseReset(...)` with the MapleSim reset coordinator so known-good odometry reset code can stay unchanged. With the toggle true it reports MapleSim's drifted gyro reading. Normal `GyroIOSim` has no MapleSim imports and reports disconnected with a timestamp.
- MapleSim vision sim must register `VisionIOSim.syncToSimulationPoseReset(...)` with the MapleSim reset coordinator. This calls PhotonVision sim's `VisionSystemSim.resetRobotPose(...)` so camera latency history does not publish pre-reset pose observations after an artificial MapleSim teleport.
- Periodic update: `Robot.simulationPeriodic()` calls `RobotContainer.updateSimulation()`, which logs `/RealOutputs/Simulation/MapleSimEnabled` and delegates to `MapleSimRobotSimulation.update()` when present. The coordinator advances `SimulatedArena` and logs `/RealOutputs/FieldSimulation/RobotPosition` plus `/RealOutputs/FieldSimulation/RobotVelocity`.
- Fuel simulation owner when `MapleSimConstants.addGamePieces` is true: `MapleSimRobotSimulation.attachFuelSimulation(...)` constructs `MapleSimFuelSimulation` after `Swerve`, `Intake`, `Shooter`, `Hopper`, and `Turret` exist. `MapleSimFuelSimulation` places `Arena2026Rebuilt` fuel with `resetFieldForAuto()`, runs `IntakeSimulation.OverTheBumperIntake("Fuel", ...)`, launches `RebuiltFuelOnFly`, and logs `/RealOutputs/FieldSimulation/Fuel` as `Pose3d[]`.
- Shot-map verifier hook: `MapleSimFuelShotMapTestControl` owns `/SimAgent/ShotMapTest`, request IDs, pose resets, fuel injection, hood override, shooter/turret/hopper commands, and `ShotMapTest*` logs. `RobotContainer` should not import NetworkTables, timers, or shooter-map constants solely for this verifier.
- Pose reset: `Swerve.setPose(...)` invokes a simulation reset callback in SIM and a no-op callback elsewhere.
- MapleSim-specific configuration lives in `MapleSimConstants`, including the enable flags, initial pose, drivetrain config, physical TODO constants, and fuel tuning constants.
- 2026 arena setup: `MapleSimRobotSimulation` overrides the default MapleSim arena with `new Arena2026Rebuilt(MapleSimConstants.addRampColliders && !MapleSimConstants.enableBumpSimulation)`. With `enableBumpSimulation = true`, hard ramp colliders stay off and `MapleSimBumpSimulation` computes robot Z, pitch, and roll from a 2026 bump heightmap under the four swerve module contact points. MapleSim remains authoritative for planar X/Y/yaw motion; the bump helper must not correct X position, own velocity, or add slide-back behavior.
- 2026 fuel setup: `MapleSimConstants.addGamePieces = true` enables official MapleSim fuel placement and physics. This flag only matters when `MapleSimConstants.useMapleSim` is true. Team 8044 defaults to full-detail fuel staging with `MapleSimConstants.fuelUseEfficiencyMode = false`, so the neutral-zone grid uses MapleSim's `12 x 30 = 360` fuel layout for max-preload matches. `MapleSimConstants.fuelPreloadCount = 8` seeds the robot intake after the first field reset. If debugging pure drivetrain contacts, check these constants and remember fuel poses should be visible at `/RealOutputs/FieldSimulation/Fuel`.
- SIM start pose: use `MapleSimConstants.initialPose` for both `SwerveDriveSimulation` and `Swerve.setPose(...)`. If `/RealOutputs/FieldSimulation/RobotPosition` and `/RealOutputs/Swerve/Odometry/Robot` do not start aligned, AdvantageScope can make MapleSim walls/obstacles look like an offset invisible field.
- Current MapleSim scope includes swerve/arena plus 2026 fuel simulation. Fuel remains MapleSim-only and must be verified with explicit field fuel pose logging and AdvantageScope visualization.

## Known Version Note

- On 2026-06-06, the official docs vendordep URL returned `0.4.0-beta-obstacles-fix`, but the official Maven metadata only published up to `0.4.0-beta`.
- This repo pins `maplesim-java` to `0.4.0-beta`.
- If dependency resolution fails, check:
  ```sh
  curl -s https://shenzhen-robotics-alliance.github.io/maple-sim/vendordep/repos/releases/org/ironmaple/maplesim-java/maven-metadata.xml
  ```

## Verification Commands

- Compile and test:
  ```sh
  JAVA_HOME=/Users/rohit/wpilib/2026/jdk ./gradlew test
  ```
- Run a headless auto through the permanent sim bridge:
  ```sh
  python3 .agents/skills/wpilib-sim/scripts/run_auto_sim.py \
    --repo . \
    --auto depotOutpost \
    --alliance Blue1 \
    --duration 5 \
    --buffer 1
  ```
- Run full-forward teleop through joystick 0 as an Xbox controller:
  ```sh
  python3 .agents/skills/wpilib-sim/scripts/run_teleop_sim.py \
    --repo . \
    --alliance Blue1 \
    --duration 4 \
    --port 0 \
    --axes "0,-1,0,0,0,0"
  ```
- Run a timed joystick rectangle in the open center-field gap. In current default MapleSim mode, heightmap bump simulation is enabled and hard ramp colliders are disabled:
  ```sh
  python3 .agents/skills/wpilib-sim/scripts/run_teleop_sim.py \
    --repo . \
    --alliance Blue1 \
    --port 0 \
    --sequence "1.2:0,-1,0,0,0,0;1.0:-1,0,0,0,0,0;1.2:0,1,0,0,0,0;1.0:1,0,0,0,0,0;0.5:0,0,0,0,0,0"
  ```
- Run fixed red own-alliance-side fuel shots against the tuned shooter map:
  ```sh
  python3 .agents/skills/wpilib-sim/scripts/run_shot_map_sim.py \
    --repo . \
    --distances "1.75,3.25,5.0"
  ```
- Check MapleSim and swerve topics:
  ```sh
  python3 .agents/skills/wpilog-decode/scripts/read_wpilog_values.py \
    --repo . \
    --log <log.wpilog> \
    --topic /RealOutputs/Swerve/TeleopController/vxMPS \
    --topic /RealOutputs/Swerve/Odometry/Robot \
    --topic /RealOutputs/FieldSimulation/RobotPosition \
    --topic /RealOutputs/FieldSimulation/RobotVelocity \
    --topic /RealOutputs/FieldSimulation/Fuel \
    --topic /RealOutputs/Simulation/Fuel/InRobotCount \
    --topic /RealOutputs/Simulation/Fuel/IntakeRunning \
    --topic /RealOutputs/Simulation/Fuel/ShotsLaunched \
    --topic /RealOutputs/Simulation/Fuel/ShooterForwardTiltFromVerticalDeg \
    --topic /RealOutputs/Simulation/Fuel/ShooterLaunchPitchDeg \
    --topic /RealOutputs/Simulation/Fuel/ShotLaunchSpeedMetersPerSecond \
    --topic /RealOutputs/Simulation/Fuel/ShotExpectedTimeOfFlightSeconds \
    --topic /RealOutputs/Simulation/Fuel/ShotTargetDistanceMeters \
    --topic /RealOutputs/Simulation/Fuel/ReturnedFuelCount \
    --topic /RealOutputs/Simulation/Fuel/LastReturnPose \
    --topic /RealOutputs/Simulation/Fuel/LastReturnVelocityMPS \
    --topic /Swerve/Module0/DriveVelocityRadPerSec \
    --topic /Swerve/Module0/DriveAppliedVolts \
    --topic /Swerve/Module0/DriveSupplyCurrentAmps \
    --topic /DriverStation/Joystick0/AxisValues \
    --topic /DriverStation/Joystick0/Xbox \
    --topic /Swerve/Gyro/Connected \
    --topic /Swerve/Gyro/YawVelocityRadPerSec
  ```

## Expected Healthy Signals

- Auto chooser selection succeeds through `SimAgentBridge`.
- `/RealOutputs/Simulation/MapleSimEnabled` should match `MapleSimConstants.useMapleSim`.
- `/RealOutputs/Swerve/Odometry/Robot` and `/RealOutputs/FieldSimulation/RobotPosition` both move during auto.
- `/RealOutputs/Swerve/Odometry/Robot` and `/RealOutputs/FieldSimulation/RobotPosition` must start at the same SIM initial pose. Use `/RealOutputs/FieldSimulation/RobotPosition` as the source of truth for MapleSim collision debugging.
- For bump visualization, use `/RealOutputs/FieldSimulation/RobotPose3d` as the AdvantageScope 3D Field robot source with log type `Pose3d`. Check `/RealOutputs/Simulation/Bump/OnBump`, `/RealOutputs/Simulation/Bump/ModuleHeights`, `/RealOutputs/Simulation/Bump/RobotRollRad`, `/RealOutputs/Simulation/Bump/RobotPitchRad`, and `/RealOutputs/Simulation/Bump/MaxModuleHeight` when tuning the contact heightmap.
- Module drive velocity, applied volts, and current are nonzero during commanded movement.
- `/Swerve/Gyro/Connected` is true in SIM.
- For full-forward teleop or each side of the rectangle, `/RealOutputs/Swerve/TeleopController/vxMPS` or `/RealOutputs/Swerve/TeleopController/vyMPS` should equal `SwerveConstants.speedAt12Volts`; `/RealOutputs/FieldSimulation/RobotVelocity` should move toward the physically achievable speed for the current mass, gearing, wheel radius, friction, and battery model.
- Odometry and MapleSim field pose should be aligned in open-field driving. After hard contact with fixed MapleSim walls/obstacles, wheel odometry may continue integrating slip while the MapleSim body is stopped; this is expected, and collision debugging should use `FieldSimulation/RobotPosition`.
- With `MapleSimConstants.useMapleSim = false`, `FieldSimulation/*` and fuel/projectile/verifier behavior are intentionally absent; use `/RealOutputs/Swerve/Odometry/Robot` and `/RealOutputs/Swerve/Actual_Velocity` for fast path/auto checks.
- With `MapleSimConstants.addGamePieces = true`, `/RealOutputs/FieldSimulation/Fuel` should be a `Pose3d[]` that AdvantageScope can render as 3D Field game pieces with variant `Fuel`.
- During fuel intake tests, `/RealOutputs/Simulation/Fuel/InRobotCount` should increase and `/RealOutputs/Simulation/Fuel/IntakeRunning` should become true when the intake is extended and rollers run forward.
- During fuel shooting tests, `/RealOutputs/Simulation/Fuel/ShotsLaunched` should increase and fuel projectile poses should appear in `/RealOutputs/FieldSimulation/Fuel`.
- Scored fuel keeps MapleSim's normal behavior: `RebuiltFuelOnFly` disappears when `hasHitTarget()` becomes true. Team 8044 only adds a delayed rear return, incrementing `/RealOutputs/Simulation/Fuel/ReturnedFuelCount` when physical `RebuiltFuelOnField` fuel respawns behind the hub with velocity toward field center.
- Fuel projectile pitch uses a frame conversion: MapleSim expects pitch up from horizontal, while the Team 8044 hood encoder reports `0 deg` at the rear hardstop, which is `12 deg` forward from vertical in the real robot. Expected formula: `launchPitch = 90 deg - (12 deg + hoodEncoderAngle)`, so encoder `0 deg` logs about `78 deg` launch pitch.
- Fuel launch speed is calibrated against the real shooter map using one global `MapleSimConstants.fuelShooterLaunchSpeedScale`; the current tuned value is `0.35`. Use `run_shot_map_sim.py` and the `/RealOutputs/Simulation/Fuel/Shot*` topics when changing this scale.
- The shot-map verifier intentionally uses `/SimAgent/ShotMapTest` to set a commanded hood-angle override inside `MapleSimFuelSimulation`; normal MapleSim fuel shots use measured hood position unless that verifier hook is enabled. The verifier's default enabled window is long enough for scored-fuel return to complete.

## Follow-Up Areas

- Calibrate TODO MapleSim physics constants in `MapleSimConstants`.
- Tune MapleSim fuel TODO constants in `MapleSimConstants` from CAD/testing: intake width/extension, shooter exit pose, shooter wheel radius, launch speed scale, encoder-zero forward tilt, `fuelShotsPerSecond`, and post-score wall/floor/restitution values.
- Decide whether to suppress SIM-only Phoenix stale fault warnings for fault status signals.
- Add high-frequency odometry arrays only if needed.
- Extend to opponent robots and vision simulation in separate deliberate passes.

---
name: maple-sim
description: "Use for our Spectrum 3847 MapleSim integration work: maintaining the swerve-first MapleSim simulation, verifying WPILib headless sim logs, calibrating physics constants, and extending simulation to intake, projectiles, opponents, or vision."
metadata:
  short-description: Maintain MapleSim simulation
---

# MapleSim

## Core Rules

- Keep this skill updated whenever a future agent discovers a repeatable MapleSim workflow, command, API mismatch, or Spectrum-2026-specific simulation convention.
- Preserve the existing spectrumLib abstraction. Do not rewrite `Swerve`, `SwerveModule`, PathPlanner autos, or `MapleSimSwerveDrivetrain` unless the task explicitly requires it.
- For drivetrain simulation, prefer MapleSim as the physics source while preserving the existing telemetry surface. Spectrum 3847 SIM swerve uses `frc.spectrumLib.swerve.MapleSimSwerveDrivetrain` on `CTRE` Phoenix TalonFX motors/controllers; keep the lightweight non-MapleSim `SimpleMotorFeedforward` path available and do not reintroduce the Phoenix remote-CANcoder sim bridge unless the steering instability that caused azimuth wind/crawl has been solved.
- Check the SIM gate before debugging: MapleSim is constructed and updated only in SIM mode (see `Swerve.startSimThread()` and `RobotSim`), gated by Phoenix `Utils.isSimulation()`. Real and replay modes never run MapleSim. Our robot has no `MapleSimConstants.useMapleSim`-style kill switch — MapleSim is always used in SIM.
- Keep real and replay modes isolated from MapleSim. MapleSim belongs in `Constants.RobotMode.SIM`.
- Record durable progress and verification notes in `.agents/maple-sim-implementation.md` when changing MapleSim behavior.

## Current Integration Shape

> Tailored for Spectrum 3847 (2026). Our swerve sim lives in `frc.spectrumLib.swerve.MapleSimSwerveDrivetrain`, driven from `Swerve` and `RobotSim` — we do not use the IO/`MapleSimConstants` split assumed by some other team code. Verify names in `src/main/java` before trusting external examples.

- Vendor dependency: `vendordeps/maple-sim.json`.
- SIM drivetrain owner: `Swerve` constructs `MapleSimSwerveDrivetrain` (wrapping `SwerveDriveSimulation`) and calls `startSimThread()`, which runs `mapleSimSwerveDrivetrain.update()` (advances `SimulatedArena.simulationPeriodic()`, then injects the simulated pose/yaw into the CTRE TalonFX/CANcoder/`CANvirtual` Pigeon2 sim).
- Arena: `MapleSimSwerveDrivetrain` builds `new Arena2026Rebuilt(false)`, calls `arena.setEfficiencyMode(true)`, overrides `SimulatedArena` timings/instance with it, and registers the drivetrain simulation. It uses `COTS.ofPigeon2()` for the gyro and module constants clamped via `MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(...)`.
- Bump / pitch-roll: `Swerve.periodic()` (SIM only) computes `Sim/RobotPose3d` through `frc.rebuilt.RobotBumpSim`; on a ramp it overrides MapleSim with `setSimulationWorldPose`. MapleSim stays authoritative for planar X/Y/yaw.
- Fuel: simulated by `frc.rebuilt.FuelPhysicsSim` under namespace `Sim/Fuel` (not MapleSim-native game pieces). Drivetrain contacts still use the MapleSim arena.
- Telemetry/log topics come from `frc.spectrumLib.telemetry.Telemetry` (DogLog), e.g. `Sim/SimPose`, `Sim/RobotPose3d`, `Swerve/State/Pose`, `Swerve/SystemState`. Real delay logs are saved as `.wpilog`.
- SIM-only code stays gated behind `Utils.isSimulation()`. MapleSim never runs in real or replay modes.

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
  ./gradlew test
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
- Check MapleSim, swerve, and fuel topics (DogLog keys as published by `frc.spectrumLib.telemetry.Telemetry`, which DogLog prefixes under `/Robot/`):
  ```sh
  python3 .agents/skills/wpilog-decode/scripts/read_wpilog_values.py \
    --repo . \
    --log <log.wpilog> \
    --topic /Robot/Sim/SimPose \
    --topic /Robot/Sim/RobotPose3d \
    --topic /Robot/Swerve/State/Pose \
    --topic /Robot/Swerve/SystemState \
    --topic /Robot/Sim/Fuel/Positions \
    --topic /Robot/Sim/Fuel/BlueScore \
    --topic /Robot/Sim/Fuel/RedScore \
    --topic /Robot/Sim/Fuel/Stats/BallCount
  ```

## Expected Healthy Signals

- Auto chooser selection succeeds (Elastic / NT auto chooser to a registered PathPlanner auto), and the sim logs the WPILOG.
- Auto chooser selection succeeds (Elastic → PathPlanner named auto).
- `Sim/SimPose` (Pose2d) and `Sim/RobotPose3d` (Pose3d) both move during auto/teleop; `Sim/RobotPose3d` is the AdvantageScope 3D Field robot source (log type `Pose3d`).
- `Swerve/State/Pose` (odometry) tracks `Sim/SimPose` in open-field driving; after hard contact with fixed MapleSim walls/obstacles, odometry may keep integrating slip while the MapleSim body stops — collision debugging should use `Sim/RobotPose3d`.
- `Swerve/SystemState` toggles correctly and `Swerve/CurrentCommand` shows the active command.
- `Swerve/Currents/DriveStatorCurrent` and `Swerve/Currents/SteerStatorCurrent` are nonzero during commanded movement.
- Fuel (`frc.rebuilt.FuelPhysicsSim`, root `Sim/Fuel`): `Sim/Fuel/Stats/BallCount` and `ActiveBalls` reflect active pieces; scored fuel increments `Sim/Fuel/BlueScore`/`RedScore`; in-flight projectiles appear in `Sim/Fuel/InFlight`/`Positions`.
- Note: `FuelPhysicsSim` is authoritative for fuel (drag/gravity/Magnus, intake, launches via `ShotCalculator`), independent of the MapleSim drivetrain; do not expect MapleSim-native fuel topics.

## Follow-Up Areas

- Calibrate MapleSim physical constants from CAD/testing as needed.
- Verify SIM Phoenix stale-fault warnings are suppressed the same way real-mode faults are.
- Add high-frequency odometry or opponent/vision simulation only if needed.

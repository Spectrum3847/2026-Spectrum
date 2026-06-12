# MapleSim (IronMaple)

*Audience: Reference. Assumes you've read [Dependencies Overview](overview.md).*

MapleSim is a physics-based simulation library — `org.ironmaple.simulation.*` — that models the swerve drivetrain, game pieces, and per-season arenas. It plugs into WPILib's simulation loop, and the result is a GUI sim that's a lot closer to reality than a stock WPILib one.

Vendor JSON: [`vendordeps/maple-sim.json`](../../vendordeps/maple-sim.json).

## What We Sim

The swerve drivetrain runs through [`MapleSimSwerveDrivetrain`](../../src/main/java/frc/spectrumLib/swerve/MapleSimSwerveDrivetrain.java), which wraps `SwerveDriveSimulation` + `SwerveModuleSimulation`. That's where wheel slip and weight transfer come from.

Game pieces (fuel, for 2026) use `GamePieceProjectile` for flight physics. The launching code in `RobotSim.mapleSimCreateFuelProjectile()` (around line 114 of [`RobotSim.java`](../../src/main/java/frc/robot/RobotSim.java)) builds a projectile from the current pose and launcher tuning and hands it to the arena. `mapleSimLaunchFuel()` wraps that into a command that fires one projectile per held piece.

The intake uses `IntakeSimulation.OverTheBumperIntake(...)` — also in `RobotSim.java`, around line 42. It models the front-bumper intake and tracks how many fuel pieces are in the robot at any moment.

The 2026 field comes from MapleSim's season-specific helpers, `Arena2026Rebuilt` and `RebuiltFuelOnFly`. These get replaced every year when the new game ships, so plan to revisit them in the offseason.

## Guarding Sim Code

Every sim entry point in `RobotSim` opens with a guard:

```java
if (!Utils.isSimulation() || RobotSim.getIntakeSimulation() == null) return;
```

The `intakeSimulation` field is intentionally `null` on a real robot. Keep the null check — that's what stops sim construction from running on the RIO.

`RobotBase.isSimulation()` and `Utils.isSimulation()` both work; pick whichever matches the surrounding file's imports.

## Resetting the Arena

`SimulatedArena.getInstance().resetFieldForAuto()` runs in `Robot.autonomousInit` and `disabledInit`. That's what gives each match a fresh set of game pieces. If you add another mode that ought to wipe the field, drop the same call into its init — without it, pieces from old runs accumulate until the sim is unusable.

## Counting Pieces

```java
int fuelCount = RobotSim.getIntakeSimulation().getGamePiecesAmount();
SmartDashboard.putNumber("Sim/FuelCount", fuelCount);
```

The Elastic Diagnostic tab subscribes to `Sim/FuelCount`. If you rename the topic, update `src/main/deploy/elastic-layout.json` to match.

## Spawning a Projectile

When the robot shoots in sim:

```java
GamePieceProjectile fuelProjectile = /* compute from pose + launcher tuning */;
SimulatedArena.getInstance().addGamePieceProjectile(fuelProjectile);
RobotSim.getIntakeSimulation().obtainGamePieceFromIntake();
```

That last line removes one piece from the intake count when the shot leaves the robot. Pair every projectile spawn with the intake decrement — otherwise the count drifts and the sim claims you're carrying fuel you've already fired.

For the math that goes into the projectile, use `frc.rebuilt.ShotCalculator`. Don't rederive RPM-to-velocity at the call site.

## Drivetrain Hookup

`MapleSimSwerveDrivetrain` is the glue. A few specifics worth knowing:

`MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules)` adjusts module constants to physically plausible sim values. Skip this and the sim robot skitters because its wheels are too stiff.

`mapleSimDrive.getSimulatedDriveTrainPose()` is what `Swerve.getRobotPose()` returns in sim. `mapleSimDrive.setSimulationWorldPose(pose)` is the teleport call `Swerve.resetPose(...)` uses when an auto seeds the pose.

The sim thread is a WPILib `Notifier` (`simNotifier`) ticking at `config.getSimLoopPeriod()`. Don't slow it down without checking physics behavior — MapleSim integrates dynamics on every tick.

## Constants That Need to Track Reality

`Swerve.startSimThread()` hard-codes a few values that have to stay in sync with the real robot:

```java
Pounds.of(115),       // robot weight
Inches.of(30),        // bumper length
Inches.of(30),        // bumper width
DCMotor.getKrakenX60Foc(1),
```

If the real robot changes weight or motor count, change these too or sim diverges from reality.

## Further Reading

[MapleSim JavaDoc](https://shenzhen-robotics-alliance.github.io/maple-sim/javadocs/) is linked into our generated docs. The [README](https://github.com/Shenzhen-Robotics-Alliance/maple-sim) has examples and the physics knobs we haven't touched. For the broader sim workflow, see [Simulation](../tools/simulation.md).

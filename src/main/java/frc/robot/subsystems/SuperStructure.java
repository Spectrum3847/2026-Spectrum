package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.rebuilt.ShotCalculator;
import frc.robot.Robot;
import frc.robot.subsystems.dyeRotor.DyeRotor;
import frc.robot.subsystems.fuelIntake.FuelIntake;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakeExtension.IntakeExtension;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherTower;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.util.Util;
import java.util.function.BooleanSupplier;
import lombok.Getter;

/**
 * Maps one wanted robot state onto every mechanism's wanted state each loop.
 *
 * <p>Deliberately not a {@code Subsystem}: {@link frc.robot.Robot#robotPeriodic()} calls {@link
 * #periodic()} exactly once per loop, before {@code CommandScheduler.run()}, so a state decision
 * reaches the mechanism periodics in the same loop instead of one loop later. It must never be
 * registered with the scheduler, because the edge detection on {@code previousSuperState} and the
 * squeeze timer assume {@code periodic()} runs exactly once per loop.
 */
public class SuperStructure {

    @Getter private final Swerve swerve;
    @Getter private final FuelIntake fuelIntake;
    @Getter private final IntakeExtension intakeExtension;
    @Getter private final DyeRotor dyeRotor;
    @Getter private final Launcher launcher;
    @Getter private final LauncherTower launcherTower;
    @Getter private final Turret turret;
    @Getter private final Hood hood;

    private static final double REGULAR_TELEOP_TRANSLATION_COEFFICIENT = 1.0;
    private static final double SHOOTING_TELEOP_TRANSLATION_COEFFICIENT = 0.2;

    private static final double REGULAR_TELEOP_ROTATION_COEFFICIENT = 1.0;
    private static final double SHOOTING_TELEOP_ROTATION_COEFFICIENT = 0.2;

    public enum WantedSuperState {
        IDLE,
        INTAKE_FUEL,
        TRACK_TARGET,
        LAUNCH_WITH_SQUEEZE,
        LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY,
        LAUNCH_WITHOUT_SQUEEZE,
        LAUNCH_WITH_BRAKE,
        AUTON_TRACK_TARGET,
        AUTON_LAUNCH_WITH_SQUEEZE,
        AUTON_LAUNCH_WITHOUT_SQUEEZE,
        AUTON_INTAKE_FUEL,
        UNJAM,
        FORCE_HOME,
    }

    public enum CurrentSuperState {
        IDLE,
        INTAKE_FUEL,
        TRACK_TARGET,
        LAUNCH_WITH_SQUEEZE,
        LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY,
        LAUNCH_WITHOUT_SQUEEZE,
        LAUNCH_WITH_BRAKE,
        AUTON_IDLE,
        AUTON_TRACK_TARGET,
        AUTON_LAUNCH_WITH_SQUEEZE,
        AUTON_LAUNCH_WITHOUT_SQUEEZE,
        AUTON_INTAKE_FUEL,
        UNJAM,
        FORCE_HOME,
    }

    @Getter private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
    @Getter private CurrentSuperState currentSuperState = CurrentSuperState.IDLE;
    private CurrentSuperState previousSuperState = CurrentSuperState.IDLE;
    /**
     * Creates a new SuperStructure instance.
     *
     * @param swerve the swerve
     * @param fuelIntake the fuelIntake
     * @param intakeExtension the intakeExtension
     * @param dyeRotor the dyeRotor
     * @param launcher the launcher
     * @param launcherTower the launcherTower
     * @param turret the turret
     * @param hood the hood
     */
    public SuperStructure(
            Swerve swerve,
            FuelIntake fuelIntake,
            IntakeExtension intakeExtension,
            DyeRotor dyeRotor,
            Launcher launcher,
            LauncherTower launcherTower,
            Turret turret,
            Hood hood) {
        this.swerve = swerve;
        this.fuelIntake = fuelIntake;
        this.intakeExtension = intakeExtension;
        this.dyeRotor = dyeRotor;
        this.launcher = launcher;
        this.launcherTower = launcherTower;
        this.turret = turret;
        this.hood = hood;
    }

    private final Timer intakeSqueezeTimer = new Timer();
    private final double secondsToSqueeze = 1.0;
    /**
     * Returns {@code true} if the current super state is one of the launch states.
     *
     * @return {@code true} when the current super state is a launch-with-squeeze,
     *     launch-without-squeeze, or launch-with-brake state
     */
    public boolean currentStateIsLaunching() {
        return currentSuperState == CurrentSuperState.LAUNCH_WITH_SQUEEZE
                || currentSuperState == CurrentSuperState.LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY
                || currentSuperState == CurrentSuperState.LAUNCH_WITHOUT_SQUEEZE
                || currentSuperState == CurrentSuperState.LAUNCH_WITH_BRAKE
                || currentSuperState == CurrentSuperState.AUTON_LAUNCH_WITHOUT_SQUEEZE
                || currentSuperState == CurrentSuperState.AUTON_LAUNCH_WITH_SQUEEZE;
    }
    /**
     * Returns {@code true} if the current super state is an intake state or a launch-without-
     * squeeze state (the launch-without-squeeze states are included intentionally).
     *
     * @return {@code true} when the current super state is an intake or launch-without-squeeze
     *     state
     */
    public boolean currentStateIsIntaking() {
        return currentSuperState == CurrentSuperState.INTAKE_FUEL
                || currentSuperState == CurrentSuperState.AUTON_INTAKE_FUEL
                || currentSuperState == CurrentSuperState.LAUNCH_WITHOUT_SQUEEZE
                || currentSuperState == CurrentSuperState.AUTON_LAUNCH_WITHOUT_SQUEEZE;
    }
    /**
     * Returns {@code true} if the squeeze state condition is met.
     *
     * @return {@code true} if the squeeze state condition is met
     */
    private static boolean isSqueezeState(CurrentSuperState state) {
        return state == CurrentSuperState.LAUNCH_WITH_SQUEEZE;
    }
    /** Runs the periodic update. Called once per loop from {@code Robot.robotPeriodic()}. */
    public void periodic() {
        currentSuperState = handleStateTransitions();

        // Restart the squeeze timer exactly once when first entering a squeeze state
        if (isSqueezeState(currentSuperState) && !isSqueezeState(previousSuperState)) {
            intakeSqueezeTimer.restart();
        }

        // Must run before applyStates(): the launch states read the gate to pick feeder states.
        updateFeedGate();

        applyStates();

        previousSuperState = currentSuperState;

        Telemetry.log("SuperStructure/WantedSuperState", wantedSuperState.toString());
        Telemetry.log("SuperStructure/CurrentSuperState", currentSuperState.toString());
        Telemetry.log(
                "SuperStructure/IntakeSqueezeTimerElapsed", intakeSqueezeTimer.get(), "seconds");
    }

    // ── Feeder gating ──────────────────────────────────────────────────────────
    //
    // Fuel only reaches the flywheel while the gate is open, so a shot is never fed while the
    // turret is mid-unwrap, the hood has not reached its angle, or the flywheel has not spun up.
    // In the 2026-09-04 shooting log the turret slewed a full 360 deg mid-burst at 101 s while
    // fuel kept feeding; those balls went anywhere.
    //
    // The gate is hysteretic. Starting a feed uses each mechanism's own strict tolerance, after a
    // short debounce; continuing a feed uses wider tolerances, because every ball loads the
    // flywheel and a gate that had to re-satisfy the strict window between balls would chop the
    // feed on and off several times a second. Only the unwrap clause is never relaxed.
    //
    // Range is deliberately not part of the keep-feeding condition. ShotCalculator's validity flag
    // is derived from the pose estimate, which is noisy enough that a single bad frame mid-burst
    // would chop the feed -- exactly what the wider tolerances exist to prevent.
    //
    // Range is also skipped entirely while the pose cannot be trusted. Without an accepted vision
    // estimate the distance ShotCalculator reports is whatever odometry was seeded with, so the
    // range check is not measuring anything. In the 2026-09-05 16:52 system check that number sat
    // at 13.006 m for the whole session with SecondsSinceVision at infinity; it fell outside the
    // feed model's fitted range, so the gate held the feed for all 275 launching loops and the dye
    // rotor never indexed a single ball. The mechanism tolerances still gate the shot in that
    // case; only the meaningless term drops out.

    /** Consecutive loops all strict predicates must hold before feeding starts. */
    private static final int SHOT_READY_DEBOUNCE_LOOPS = 3;

    /**
     * Turret tracking error tolerated while already feeding. Wider than the turret's own 2 deg
     * trigger tolerance; set from a log of {@code Turret/TrackingErrorDegrees} during a burst.
     */
    private static final double KEEP_FEED_TURRET_TOLERANCE_DEG = 6.0;

    /** Hood angle error tolerated while already feeding, versus its 0.5 deg aim tolerance. */
    private static final double KEEP_FEED_HOOD_TOLERANCE_DEG = 3.0;

    /**
     * Flywheel droop tolerated while already feeding, as a fraction of commanded RPM. Spin-up from
     * 650 to 2600 RPM took about 0.25 s on the bench and held inside the 200 RPM window during
     * bursts, so this only has to cover the per-ball dip. Set from {@code Launcher/RPM}.
     */
    private static final double KEEP_FEED_MIN_SPEED_FRACTION = 0.75;

    /**
     * Age past which an accepted vision estimate no longer makes the pose worth range-checking.
     *
     * <p>Generous on purpose. Shortening it makes the robot fall back to "range is unknown, feed on
     * the mechanism tolerances alone" after brief dropouts, which is the permissive direction; a
     * turret aiming at the hub re-acquires a tag well inside this window.
     */
    private static final double POSE_TRUST_TIMEOUT_SECONDS = 3.0;

    /**
     * Feeder states used while the gate is closed.
     *
     * <p>The dye rotor holds at {@code IDLE_SLOW_INDEX}, whose feeder RPM is 0 — it agitates the
     * bed without indexing, so it is a true hold.
     *
     * <p>The launcher tower holds at {@code OFF}, not {@code SLOW_INDEX}. {@code SLOW_INDEX} is
     * 1000 RPM <em>forward</em>, a quarter of {@code INDEX_MAX}; with the tower already full of
     * fuel mid-burst that keeps pushing fuel into the flywheel, which is exactly what the gate
     * exists to prevent. The tower is in brake neutral mode, so {@code OFF} holds fuel in place. If
     * a bench check shows staged fuel does not reach the flywheel at 1000 RPM, switching this to
     * {@code SLOW_INDEX} would shorten the delay when the gate opens.
     */
    private static final LauncherTower.WantedState TOWER_HOLD_STATE = LauncherTower.WantedState.OFF;

    private static final DyeRotor.WantedState ROTOR_HOLD_STATE =
            DyeRotor.WantedState.IDLE_SLOW_INDEX;

    private int shotReadyStreak = 0;
    private int launchingLoops = 0;
    private int heldFeedLoops = 0;

    /** True while the gate is open and fuel is allowed into the flywheel. */
    private boolean feedGateOpen = false;

    /** Operator hold to feed regardless of the gate, for a bad sensor or a deliberate dump. */
    private BooleanSupplier feedOverride = () -> false;

    /**
     * Sets the operator control that bypasses feeder gating while held. Bound once at startup; the
     * supplier is polled every loop.
     *
     * @param override true while the operator wants the gates ignored
     */
    public void setFeedOverride(BooleanSupplier override) {
        this.feedOverride = override;
    }

    /**
     * Returns {@code true} when fuel is allowed into the flywheel this loop.
     *
     * @return true when the gate is open or the operator is overriding it
     */
    public boolean isFeedAllowed() {
        return feedGateOpen || feedOverride.getAsBoolean();
    }

    private void updateFeedGate() {
        Vision vision = Robot.getVision();
        double secondsSinceVision =
                vision == null
                        ? Double.POSITIVE_INFINITY
                        : vision.secondsSinceLastAcceptedEstimate();
        // Infinity until vision accepts its first estimate, so this is false on a shop bench.
        boolean poseTrusted = secondsSinceVision <= POSE_TRUST_TIMEOUT_SECONDS;

        boolean launcherAtSpeed = launcher.isAtSpeed();
        boolean hoodAtAngle = hood.isAtAngle();
        boolean turretOnTarget = turret.isReadyToShoot();
        boolean shotInRange = ShotCalculator.getInstance().getParameters().isValid();
        // A range check against an untrusted pose is not measuring anything, so it does not vote.
        boolean rangeOk = !poseTrusted || shotInRange;
        boolean shotReady = launcherAtSpeed && hoodAtAngle && turretOnTarget && rangeOk;

        shotReadyStreak = shotReady ? shotReadyStreak + 1 : 0;
        boolean startReady = shotReadyStreak >= SHOT_READY_DEBOUNCE_LOOPS;

        boolean keepReady =
                launcher.isAboveSpeedFraction(KEEP_FEED_MIN_SPEED_FRACTION)
                        && hood.isAtAngle(KEEP_FEED_HOOD_TOLERANCE_DEG)
                        && turret.isReadyToShoot(KEEP_FEED_TURRET_TOLERANCE_DEG);

        boolean launching = currentStateIsLaunching();
        // Closing the gate on leaving a launch state means the next burst re-earns the strict
        // window rather than inheriting the last one's open gate.
        feedGateOpen = launching && (feedGateOpen ? keepReady : startReady);

        if (launching) {
            launchingLoops++;
            if (!isFeedAllowed()) {
                heldFeedLoops++;
            }
        }

        Telemetry.log("SuperStructure/ShotReady/LauncherAtSpeed", launcherAtSpeed);
        Telemetry.log("SuperStructure/ShotReady/HoodAtAngle", hoodAtAngle);
        Telemetry.log("SuperStructure/ShotReady/TurretOnTarget", turretOnTarget);
        Telemetry.log("SuperStructure/ShotReady/ShotInRange", shotInRange);
        Telemetry.log("SuperStructure/ShotReady/PoseTrusted", poseTrusted);
        Telemetry.log("SuperStructure/ShotReady/RangeOk", rangeOk);
        Telemetry.log("SuperStructure/ShotReady/Composite", shotReady);
        Telemetry.log("SuperStructure/ShotReady/StartReady", startReady);
        Telemetry.log("SuperStructure/ShotReady/KeepReady", keepReady);
        Telemetry.log("SuperStructure/ShotReady/GateOpen", feedGateOpen);
        Telemetry.log("SuperStructure/ShotReady/Override", feedOverride.getAsBoolean());
        Telemetry.log("SuperStructure/ShotReady/FeedAllowed", isFeedAllowed());
        Telemetry.log("SuperStructure/ShotReady/HoldingFeed", launching && !isFeedAllowed());
        Telemetry.log("SuperStructure/ShotReady/LaunchingLoops", launchingLoops);
        Telemetry.log("SuperStructure/ShotReady/HeldFeedLoops", heldFeedLoops);
        Telemetry.log("SuperStructure/ShotReady/SecondsSinceVision", secondsSinceVision, "seconds");
        Telemetry.log("Turret/TrackingErrorDegrees", turret.getTrackingErrorDegrees(), "deg");
    }

    /**
     * Applies the dye rotor and launcher tower states for a launch, feeding at full index only
     * while the gate allows it and holding fuel short of the flywheel otherwise.
     */
    private void applyGatedFeed() {
        if (isFeedAllowed()) {
            dyeRotor.setWantedState(DyeRotor.WantedState.INDEX_MAX);
            launcherTower.setWantedState(LauncherTower.WantedState.INDEX_MAX);
        } else {
            dyeRotor.setWantedState(ROTOR_HOLD_STATE);
            launcherTower.setWantedState(TOWER_HOLD_STATE);
        }
    }
    /** Handles the state transitions. */
    private CurrentSuperState handleStateTransitions() {
        return switch (wantedSuperState) {
            case IDLE -> Util.autoMode.getAsBoolean() || Util.disabled.getAsBoolean()
                    ? CurrentSuperState.AUTON_IDLE
                    : CurrentSuperState.IDLE;
            case INTAKE_FUEL -> CurrentSuperState.INTAKE_FUEL;
            case TRACK_TARGET -> CurrentSuperState.TRACK_TARGET;
            case LAUNCH_WITH_SQUEEZE -> CurrentSuperState.LAUNCH_WITH_SQUEEZE;
            case LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY -> CurrentSuperState
                    .LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY;
            case LAUNCH_WITHOUT_SQUEEZE -> CurrentSuperState.LAUNCH_WITHOUT_SQUEEZE;
            case LAUNCH_WITH_BRAKE -> CurrentSuperState.LAUNCH_WITH_BRAKE;
            case AUTON_TRACK_TARGET -> CurrentSuperState.AUTON_TRACK_TARGET;
            case AUTON_LAUNCH_WITH_SQUEEZE -> CurrentSuperState.AUTON_LAUNCH_WITH_SQUEEZE;
            case AUTON_LAUNCH_WITHOUT_SQUEEZE -> CurrentSuperState.AUTON_LAUNCH_WITHOUT_SQUEEZE;
            case AUTON_INTAKE_FUEL -> CurrentSuperState.AUTON_INTAKE_FUEL;
            case UNJAM -> CurrentSuperState.UNJAM;
            case FORCE_HOME -> CurrentSuperState.FORCE_HOME;
        };
    }
    /** Applies the states. */
    private void applyStates() {
        switch (currentSuperState) {
            case IDLE:
                applyIdle();
                break;
            case INTAKE_FUEL:
                intakeFuel();
                break;
            case TRACK_TARGET:
                trackTarget();
                break;
            case LAUNCH_WITH_SQUEEZE:
                launchWithSqueeze();
                break;
            case LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY:
                launchWithSqueezeWithNoDelay();
                break;
            case LAUNCH_WITHOUT_SQUEEZE:
                launchWithoutSqueeze();
                break;
            case LAUNCH_WITH_BRAKE:
                launchWithBrake();
                break;
            case AUTON_IDLE:
                applyAutonIdle();
                break;
            case AUTON_INTAKE_FUEL:
                autonIntakeFuel();
                break;
            case AUTON_LAUNCH_WITHOUT_SQUEEZE:
                autonLaunchWithoutSqueeze();
                break;
            case AUTON_LAUNCH_WITH_SQUEEZE:
                autonLaunchWithSqueeze();
                break;
            case AUTON_TRACK_TARGET:
                autonTrackTarget();
                break;
            case UNJAM:
                unjam();
                break;
            case FORCE_HOME:
                forceHome();
                break;
        }
    }

    // ── State methods ──────────────────────────────────────────────────────────
    /** Applies the idle. */
    private void applyIdle() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        swerve.setTeleopRotationVelocityCoefficient(REGULAR_TELEOP_ROTATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        dyeRotor.setWantedState(DyeRotor.WantedState.IDLE_SLOW_INDEX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.STOPPED);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        launcherTower.setWantedState(LauncherTower.WantedState.OFF);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.HOME);
    }
    /** Intake fuel. */
    private void intakeFuel() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        swerve.setTeleopRotationVelocityCoefficient(REGULAR_TELEOP_ROTATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        dyeRotor.setWantedState(DyeRotor.WantedState.IDLE_SLOW_INDEX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        launcherTower.setWantedState(LauncherTower.WantedState.SLOW_INDEX);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.HOME);
    }
    /** Track target. */
    private void trackTarget() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        swerve.setTeleopRotationVelocityCoefficient(REGULAR_TELEOP_ROTATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        dyeRotor.setWantedState(DyeRotor.WantedState.IDLE_SLOW_INDEX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        launcherTower.setWantedState(LauncherTower.WantedState.SLOW_INDEX);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }
    /** Launches with squeeze. */
    private void launchWithSqueeze() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(SHOOTING_TELEOP_TRANSLATION_COEFFICIENT);
        swerve.setTeleopRotationVelocityCoefficient(SHOOTING_TELEOP_ROTATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.SLOW_INTAKE);
        launcher.setWantedState(Launcher.WantedState.LAUNCH);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
        applyGatedFeed();

        if (intakeSqueezeTimer.hasElapsed(secondsToSqueeze)) {
            intakeExtension.setWantedState(IntakeExtension.WantedState.AGITATE);
            intakeSqueezeTimer.stop();
        } else {
            intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_EXTEND);
        }
    }
    /** Launches with squeeze with no delay. */
    private void launchWithSqueezeWithNoDelay() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(SHOOTING_TELEOP_TRANSLATION_COEFFICIENT);
        swerve.setTeleopRotationVelocityCoefficient(SHOOTING_TELEOP_ROTATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.SLOW_INTAKE);
        intakeExtension.setWantedState(IntakeExtension.WantedState.AGITATE);
        launcher.setWantedState(Launcher.WantedState.LAUNCH);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
        applyGatedFeed();
    }
    /** Launches without squeeze. */
    private void launchWithoutSqueeze() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(SHOOTING_TELEOP_TRANSLATION_COEFFICIENT);
        swerve.setTeleopRotationVelocityCoefficient(SHOOTING_TELEOP_ROTATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.LAUNCH);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
        applyGatedFeed();
    }
    /** Launches with brake. */
    private void launchWithBrake() {
        swerve.setWantedState(Swerve.WantedState.X_BRAKE);
        fuelIntake.setWantedState(FuelIntake.WantedState.SLOW_INTAKE);
        intakeExtension.setWantedState(IntakeExtension.WantedState.AGITATE);
        launcher.setWantedState(Launcher.WantedState.LAUNCH);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
        applyGatedFeed();
    }
    /** Applies the auton idle. */
    private void applyAutonIdle() {
        swerve.setWantedState(Swerve.WantedState.IDLE);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        dyeRotor.setWantedState(DyeRotor.WantedState.IDLE_SLOW_INDEX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.STOPPED);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        launcherTower.setWantedState(LauncherTower.WantedState.OFF);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.HOME);
    }
    /** Auton intake fuel. */
    private void autonIntakeFuel() {
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        dyeRotor.setWantedState(DyeRotor.WantedState.IDLE_SLOW_INDEX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        launcherTower.setWantedState(LauncherTower.WantedState.SLOW_INDEX);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.HOME);
    }
    /** Auton track target. */
    private void autonTrackTarget() {
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        dyeRotor.setWantedState(DyeRotor.WantedState.IDLE_SLOW_INDEX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        launcherTower.setWantedState(LauncherTower.WantedState.SLOW_INDEX);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }
    /** Auton launch without squeeze. */
    private void autonLaunchWithoutSqueeze() {
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.LAUNCH);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
        applyGatedFeed();
    }
    /** Auton launch with squeeze. */
    private void autonLaunchWithSqueeze() {
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        intakeExtension.setWantedState(IntakeExtension.WantedState.AGITATE);
        launcher.setWantedState(Launcher.WantedState.LAUNCH);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
        applyGatedFeed();
    }
    /** Unjam. */
    private void unjam() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        dyeRotor.setWantedState(DyeRotor.WantedState.UNJAM);
        intakeExtension.setWantedState(IntakeExtension.WantedState.AGITATE);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        launcherTower.setWantedState(LauncherTower.WantedState.UNJAM);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.HOME);
    }
    /** Force home. */
    private void forceHome() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        swerve.setTeleopRotationVelocityCoefficient(REGULAR_TELEOP_ROTATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        dyeRotor.setWantedState(DyeRotor.WantedState.OFF);
        intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_RETRACT);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        launcherTower.setWantedState(LauncherTower.WantedState.OFF);
        turret.setWantedState(Turret.WantedState.IDLE);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    // ── Public API ─────────────────────────────────────────────────────────────

    // Allocation-free boolean checks — use these in per-loop code (e.g. ShotCalculator).
    /**
     * Returns {@code true} if the robot in neutral zone condition is met.
     *
     * @return {@code true} if the robot in neutral zone condition is met
     */
    public boolean isRobotInNeutralZone() {
        return swerve.isInNeutralZone();
    }
    /**
     * Returns {@code true} if the robot in enemy zone condition is met.
     *
     * @return {@code true} if the robot in enemy zone condition is met
     */
    public boolean isRobotInEnemyZone() {
        return swerve.isInEnemyAllianceZone();
    }
    /**
     * Returns {@code true} if the robot is in the feed zone.
     *
     * @return {@code true} when the robot is in the enemy or neutral zone (the feed zone)
     */
    public boolean isRobotInFeedZone() {
        return isRobotInEnemyZone() || isRobotInNeutralZone();
    }
    /**
     * Returns {@code true} if the robot is in the score zone.
     *
     * @return {@code true} when the robot is not in the feed zone
     */
    public boolean isRobotInScoreZone() {
        return !isRobotInFeedZone();
    }

    // Trigger factories — use these for binding-time composition only.
    /** Robot in neutral zone. */
    public Trigger robotInNeutralZone() {
        return new Trigger(this::isRobotInNeutralZone);
    }
    /** Robot in enemy zone. */
    public Trigger robotInEnemyZone() {
        return new Trigger(this::isRobotInEnemyZone);
    }
    /** Robot in feed zone. */
    public Trigger robotInFeedZone() {
        return new Trigger(this::isRobotInFeedZone);
    }
    /** Robot in score zone. */
    public Trigger robotInScoreZone() {
        return new Trigger(this::isRobotInScoreZone);
    }
    /**
     * Sets the wanted super state.
     *
     * @param state the wanted super state
     */
    public void setWantedSuperState(WantedSuperState state) {
        this.wantedSuperState = state;
    }
    /**
     * Sets the state command.
     *
     * @param state the state command
     */
    public Command setStateCommand(WantedSuperState state) {
        return new InstantCommand(() -> setWantedSuperState(state));
    }
}

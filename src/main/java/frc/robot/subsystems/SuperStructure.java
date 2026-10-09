package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.util.Util;
import java.util.function.BooleanSupplier;
import lombok.Getter;

/**
 * Maps one wanted robot state onto every mechanism's wanted state each loop.
 *
 * <p>Deliberately not a {@code Subsystem}. {@link frc.robot.Robot#robotPeriodic()} calls {@link
 * #periodic()} exactly once per loop, before {@code CommandScheduler.run()}, so a state decision
 * reaches the mechanism periodics in the same loop instead of one loop later. It must never be
 * registered with the scheduler: the edge detection on {@code previousSuperState} and the squeeze
 * timer both assume {@code periodic()} runs exactly once per loop.
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
    private static final double SHOOTING_TELEOP_TRANSLATION_COEFFICIENT = 0.15;

    private static final double REGULAR_TELEOP_ROTATION_COEFFICIENT = 1.0;
    private static final double SHOOTING_TELEOP_ROTATION_COEFFICIENT = 0.15;

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
        KICKER_UNJAM,
        FORCE_HOME,
        /**
         * Pose-independent fixed shot from a parking spot picked with {@link
         * ShotCalculator#selectSetShot}. See {@link #setShot()}.
         */
        SET_SHOT,
        /** Test-mode pit check: turret follows any tag the turret camera sees. */
        TEST_TURRET_FOLLOW_TAG,
        /** Test-mode pit check: turret runs soft limit to soft limit and back. */
        TEST_TURRET_SWEEP,
        /** Test-mode pit check: turret returns to its zero. */
        TEST_TURRET_ZERO,
        /** Test mode at rest: every mechanism off, turret held where it is. */
        TEST_TURRET_STOP,
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
        KICKER_UNJAM,
        FORCE_HOME,
        SET_SHOT,
        TEST_TURRET_FOLLOW_TAG,
        TEST_TURRET_SWEEP,
        TEST_TURRET_ZERO,
        TEST_TURRET_STOP,
    }

    @Getter private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
    @Getter private CurrentSuperState currentSuperState = CurrentSuperState.IDLE;
    private CurrentSuperState previousSuperState = CurrentSuperState.IDLE;

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

    /**
     * How long a launch runs fully extended before the extensions start agitating.
     *
     * <p>Defaults to zero, so the agitate starts with the launch: it pulls a short stroke and
     * pushes back out as soon as it meets fuel instead of squeezing the bed at the 80 A stator
     * limit, so it does not need a delay to let a full hopper draw down first. Tunable from
     * NetworkTables so a delay can be put back during a session without a redeploy.
     */
    private static final DoubleSubscriber secondsToSqueeze =
            Telemetry.tunable("SuperStructure/SecondsToSqueeze", 0.0);

    /**
     * Picks the extension state for the not-intaking states. In our own alliance zone, where we can
     * launch, agitate the extension if intaking sent it out, so the fuel is loose and ready to
     * feed, and pull it in once it comes free. Anywhere else, fall back to the given state.
     */
    private IntakeExtension.WantedState agitateInScoreZoneElse(
            IntakeExtension.WantedState otherwise) {
        return isRobotInFeedZone() ? otherwise : IntakeExtension.WantedState.CONDITIONAL_AGITATE;
    }

    /** True when the current super state is one that feeds the flywheel. */
    public boolean currentStateIsLaunching() {
        return currentSuperState == CurrentSuperState.LAUNCH_WITH_SQUEEZE
                || currentSuperState == CurrentSuperState.LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY
                || currentSuperState == CurrentSuperState.LAUNCH_WITHOUT_SQUEEZE
                || currentSuperState == CurrentSuperState.LAUNCH_WITH_BRAKE
                || currentSuperState == CurrentSuperState.AUTON_LAUNCH_WITHOUT_SQUEEZE
                || currentSuperState == CurrentSuperState.AUTON_LAUNCH_WITH_SQUEEZE
                || currentSuperState == CurrentSuperState.SET_SHOT;
    }

    /**
     * True when the current super state is an intake state, or a launch-without-squeeze state. The
     * launch-without-squeeze states are included on purpose: those keep the roller and the
     * extension out.
     */
    public boolean currentStateIsIntaking() {
        return isIntakingState(currentSuperState);
    }

    private static boolean isIntakingState(CurrentSuperState state) {
        return state == CurrentSuperState.INTAKE_FUEL
                || state == CurrentSuperState.AUTON_INTAKE_FUEL
                || state == CurrentSuperState.LAUNCH_WITHOUT_SQUEEZE
                || state == CurrentSuperState.AUTON_LAUNCH_WITHOUT_SQUEEZE;
    }

    public void periodic() {
        currentSuperState = handleStateTransitions();

        // Restart the squeeze timer exactly once when first entering a squeeze state
        if (currentSuperState == CurrentSuperState.LAUNCH_WITH_SQUEEZE
                && previousSuperState != CurrentSuperState.LAUNCH_WITH_SQUEEZE) {
            intakeSqueezeTimer.restart();
        }

        // Pressing intake again drives a coasting extension back out.
        if (isIntakingState(currentSuperState) && !isIntakingState(previousSuperState)) {
            intakeExtension.requestExtend();
        }

        // Must run before applyStates(): the launch states read the gate to pick feeder states.
        updateFeedGate();

        applyStates();

        previousSuperState = currentSuperState;

        Telemetry.logState("SuperStructure/WantedSuperState", wantedSuperState);
        Telemetry.logStateDash("SuperStructure/CurrentSuperState", currentSuperState);
        Telemetry.log(
                "SuperStructure/IntakeSqueezeTimerElapsed", intakeSqueezeTimer.get(), "seconds");
    }

    /*
     * Feeder gating. Fuel only reaches the flywheel while the gate is open, so a shot is never fed
     * while the turret is mid-unwrap, the hood has not reached its angle, or the flywheel has not
     * spun up. A turret that slewed a full 360 deg mid-burst while fuel kept feeding sent those
     * balls anywhere.
     *
     * The gate is hysteretic. Starting a feed uses each mechanism's own strict tolerance, after a
     * short debounce; continuing a feed uses wider tolerances, because every ball loads the
     * flywheel and a gate that had to re-satisfy the strict window between balls would chop the
     * feed on and off several times a second. Only the unwrap clause is never relaxed.
     *
     * Range is deliberately not part of the keep-feeding condition. ShotCalculator's validity flag
     * comes from the pose estimate, which is noisy enough that a single bad frame mid-burst would
     * chop the feed, which is exactly what the wider tolerances exist to prevent.
     *
     * Range is also skipped entirely while the pose cannot be trusted. Without an accepted vision
     * estimate the distance ShotCalculator reports is whatever odometry was seeded with, so the
     * range check is not measuring anything. The mechanism tolerances still gate the shot in that
     * case; only the meaningless term drops out.
     */
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
     * <p>The dye rotor holds at {@code IDLE_SLOW_INDEX}, whose feeder RPM is 0, so it agitates the
     * bed without indexing and is a true hold.
     *
     * <p>The launcher tower holds at {@code OFF}, not {@code SLOW_INDEX}. {@code SLOW_INDEX} is
     * 1000 RPM <em>forward</em>, a quarter of {@code INDEX_MAX}; with the tower already full of
     * fuel mid-burst that keeps pushing fuel into the flywheel, which is exactly what the gate
     * exists to prevent. The tower is in brake neutral mode, so {@code OFF} holds fuel in place. If
     * a bench check shows staged fuel does reach the flywheel at 1000 RPM, switching this to {@code
     * SLOW_INDEX} would shorten the delay when the gate opens.
     */
    private static final LauncherTower.WantedState TOWER_HOLD_STATE = LauncherTower.WantedState.OFF;

    private static final DyeRotor.WantedState ROTOR_HOLD_STATE =
            DyeRotor.WantedState.IDLE_SLOW_INDEX;

    private int shotReadyStreak = 0;
    private int launchingLoops = 0;
    private int heldFeedLoops = 0;

    /** True while the gate is open and fuel is allowed into the flywheel. */
    private boolean feedGateOpen = false;

    /** Previous loop's gate, so the rising edge can be caught for the shot record. */
    private boolean feedGateOpenLastLoop = false;

    /** Operator hold to feed regardless of the gate, for a bad sensor or a deliberate dump. */
    private BooleanSupplier feedOverride = () -> false;

    /**
     * Sets the operator control that bypasses feeder gating while held. Bound once at startup; the
     * supplier is polled every loop.
     */
    public void setFeedOverride(BooleanSupplier override) {
        this.feedOverride = override;
    }

    public boolean isFeedAllowed() {
        return feedGateOpen || feedOverride.getAsBoolean();
    }

    private void updateFeedGate() {
        double secondsSinceVision = Robot.getVision().secondsSinceLastAcceptedEstimate();
        // Infinity until vision accepts its first estimate, so this is false on a shop bench.
        boolean poseTrusted = secondsSinceVision <= POSE_TRUST_TIMEOUT_SECONDS;

        boolean launcherAtSpeed = launcher.isAtSpeed();
        boolean hoodAtAngle = hood.isAtAngle();
        boolean turretOnTarget = turret.isReadyToShoot();
        boolean shotInRange = ShotCalculator.getInstance().getParameters().isValid();
        // A range check against an untrusted pose is not measuring anything, so it does not vote.
        boolean rangeOk = !poseTrusted || shotInRange;
        // The set shot is the deliberate exception to the range check. Range is computed from the
        // pose, and the set shot exists precisely for when there is no pose to compute it from, so
        // it gets no vote; the driver has taken responsibility for parking the robot. Speed, hood
        // angle and the turret still do: the turret has a fixed angle to reach (a half turn for the
        // over-the-intake shots), and fuel fed mid-slew goes anywhere.
        boolean setShot = currentSuperState == CurrentSuperState.SET_SHOT;
        boolean shotReady =
                launcherAtSpeed && hoodAtAngle && turretOnTarget && (setShot || rangeOk);

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

        // One row per burst, on the first loop fuel is allowed through: the last loop on which the
        // aim was still a prediction. The operator's D-pad supplies the outcome later, so this is
        // half a dataset row and ShotCalc/Trim/* is the other half.
        if (feedGateOpen && !feedGateOpenLastLoop) {
            ShotCalculator.recordShot(poseTrusted);
        }
        feedGateOpenLastLoop = feedGateOpen;

        if (launching) {
            launchingLoops++;
            if (!isFeedAllowed()) {
                heldFeedLoops++;
            }
        }

        Telemetry.logDash("SuperStructure/ShotReady/LauncherAtSpeed", launcherAtSpeed);
        Telemetry.log("SuperStructure/ShotReady/HoodAtAngle", hoodAtAngle);
        Telemetry.logDash("SuperStructure/ShotReady/TurretOnTarget", turretOnTarget);
        Telemetry.log("SuperStructure/ShotReady/ShotInRange", shotInRange);
        Telemetry.log("SuperStructure/ShotReady/PoseTrusted", poseTrusted);
        Telemetry.log("SuperStructure/ShotReady/RangeOk", rangeOk);
        Telemetry.logDash("SuperStructure/ShotReady/Composite", shotReady);
        Telemetry.log("SuperStructure/ShotReady/StartReady", startReady);
        Telemetry.log("SuperStructure/ShotReady/KeepReady", keepReady);
        Telemetry.log("SuperStructure/ShotReady/GateOpen", feedGateOpen);
        Telemetry.log("SuperStructure/ShotReady/Override", feedOverride.getAsBoolean());
        Telemetry.log("SuperStructure/ShotReady/FeedAllowed", isFeedAllowed());
        Telemetry.log("SuperStructure/ShotReady/HoldingFeed", launching && !isFeedAllowed());
        Telemetry.log("SuperStructure/ShotReady/LaunchingLoops", launchingLoops);
        Telemetry.log("SuperStructure/ShotReady/HeldFeedLoops", heldFeedLoops);
        if (Telemetry.slowLogThisLoop()) {
            Telemetry.logDash(
                    "SuperStructure/ShotReady/SecondsSinceVision", secondsSinceVision, "seconds");
        }
        Telemetry.logDash("Turret/TrackingErrorDegrees", turret.getTrackingErrorDegrees(), "deg");
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
            case KICKER_UNJAM -> CurrentSuperState.KICKER_UNJAM;
            case FORCE_HOME -> CurrentSuperState.FORCE_HOME;
            case SET_SHOT -> CurrentSuperState.SET_SHOT;
            case TEST_TURRET_FOLLOW_TAG -> CurrentSuperState.TEST_TURRET_FOLLOW_TAG;
            case TEST_TURRET_SWEEP -> CurrentSuperState.TEST_TURRET_SWEEP;
            case TEST_TURRET_ZERO -> CurrentSuperState.TEST_TURRET_ZERO;
            case TEST_TURRET_STOP -> CurrentSuperState.TEST_TURRET_STOP;
        };
    }

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
                launchAgitating();
                break;
            case AUTON_TRACK_TARGET:
                autonTrackTarget();
                break;
            case UNJAM:
                unjam(FuelIntake.WantedState.REVERSE);
                break;
            case KICKER_UNJAM:
                unjam(FuelIntake.WantedState.REVERSE_KEEP_KICKER);
                break;
            case FORCE_HOME:
                forceHome();
                break;
            case SET_SHOT:
                setShot();
                break;
            case TEST_TURRET_FOLLOW_TAG:
                testTurret(Turret.WantedState.TEST_FOLLOW_TAG);
                break;
            case TEST_TURRET_SWEEP:
                testTurret(Turret.WantedState.TEST_SWEEP_LIMITS);
                break;
            case TEST_TURRET_ZERO:
                testTurret(Turret.WantedState.TEST_ZERO);
                break;
            case TEST_TURRET_STOP:
                testTurret(Turret.WantedState.OFF);
                break;
        }
    }

    /**
     * Fixed shot from a known parking spot, for when the pose is gone.
     *
     * <p>Every other launch state asks {@link ShotCalculator} where the hub is, which means asking
     * where the robot is. When vision has not seeded the pose that answer is wrong in a way nothing
     * on the robot can detect, and the turret aims off by the robot's power-on heading error. This
     * state asks nothing: the turret goes to the spot's fixed angle, and the hood and flywheel go
     * to the pair of numbers the spot's range works out to. Which spot is {@link
     * ShotCalculator#getSelectedSetShot()}, picked by the pilot binding that requested the state.
     *
     * <p>The driver does the aiming, by parking against the field element and pointing the intake
     * where the spot says. For the tower shot that means the intake pointed at the tower's
     * field-facing wall, then the heading cheated about 5 deg toward the hub, since squared up dead
     * flat is 5.3 deg off: the tower's centreline follows tag 31 and the hub sits on the field
     * centreline. That is about 29 cm of lateral miss at this range against a goal 41.7 in wide on
     * the inside, so it still scores. The over-the-intake shots are the intake pointed at the hub.
     *
     * <p>Nothing here checks any of that, since it cannot, so the shot is only as good as the
     * parking. Deliberately not a drive-to-pose: a pose good enough to drive to is a pose good
     * enough to aim with, and the case this exists for is not having one.
     */
    private void setShot() {
        teleopDrive(true);
        fuelIntake.setWantedState(FuelIntake.WantedState.SLOW_INTAKE);
        intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.SET_SHOT);
        hood.setWantedState(Hood.WantedState.SET_SHOT);
        // Not AIM_AT_TARGET: that reads the pose, which is the thing this state exists to do
        // without.
        turret.setFixedAngleDegrees(ShotCalculator.getSetShotTurretDegrees());
        turret.setWantedState(Turret.WantedState.FIXED_ANGLE);
        applyGatedFeed();
    }

    private void applyIdle() {
        teleopDrive(false);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        dyeRotor.setWantedState(DyeRotor.WantedState.IDLE_SLOW_INDEX);
        intakeExtension.setWantedState(agitateInScoreZoneElse(IntakeExtension.WantedState.STOPPED));
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        launcherTower.setWantedState(LauncherTower.WantedState.OFF);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    private void intakeFuel() {
        teleopDrive(false);
        autonIntakeFuel();
    }

    private void trackTarget() {
        teleopDrive(false);
        autonTrackTarget();
    }

    private void launchWithSqueeze() {
        teleopDrive(true);
        launchAgitating();
        // Hold the extension out until the squeeze delay has passed, then let it agitate.
        if (intakeSqueezeTimer.hasElapsed(secondsToSqueeze.get())) {
            intakeSqueezeTimer.stop();
        } else {
            intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_EXTEND);
        }
    }

    private void launchWithSqueezeWithNoDelay() {
        teleopDrive(true);
        launchAgitating();
    }

    private void launchWithoutSqueeze() {
        teleopDrive(true);
        autonLaunchWithoutSqueeze();
    }

    private void launchWithBrake() {
        swerve.setWantedState(Swerve.WantedState.X_BRAKE);
        launchAgitating();
    }

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

    private void autonIntakeFuel() {
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        dyeRotor.setWantedState(DyeRotor.WantedState.IDLE_SLOW_INDEX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        launcherTower.setWantedState(LauncherTower.WantedState.SLOW_INDEX);
        // Slow sweep about the aim so incoming fuel cannot pack against the turret.
        turret.setWantedState(Turret.WantedState.AIM_SWEEP);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    private void autonTrackTarget() {
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        dyeRotor.setWantedState(DyeRotor.WantedState.IDLE_SLOW_INDEX);
        intakeExtension.setWantedState(
                agitateInScoreZoneElse(IntakeExtension.WantedState.CONDITIONAL_EXTEND));
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        launcherTower.setWantedState(LauncherTower.WantedState.SLOW_INDEX);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }

    private void autonLaunchWithoutSqueeze() {
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.LAUNCH);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
        applyGatedFeed();
    }

    /**
     * Launch with the extension agitating. The intake rollers run through the whole launch, squeeze
     * included: they cost roughly 23 A, but with them idle the squeeze packs fuel against the
     * bumper instead of moving it toward the feeder. The current has to come from somewhere else.
     */
    private void launchAgitating() {
        fuelIntake.setWantedState(FuelIntake.WantedState.SLOW_INTAKE);
        intakeExtension.setWantedState(IntakeExtension.WantedState.AGITATE);
        launcher.setWantedState(Launcher.WantedState.LAUNCH);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
        applyGatedFeed();
    }

    /** Teleop drive at the shooting or the regular speed limits. */
    private void teleopDrive(boolean shooting) {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(
                shooting
                        ? SHOOTING_TELEOP_TRANSLATION_COEFFICIENT
                        : REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        swerve.setTeleopRotationVelocityCoefficient(
                shooting
                        ? SHOOTING_TELEOP_ROTATION_COEFFICIENT
                        : REGULAR_TELEOP_ROTATION_COEFFICIENT);
    }

    /**
     * Unjam: extension fully out so nothing is pinched, every roller in the fuel path backwards
     * (intake, dye rotor and feeder, tower, flywheel) so fuel moves away from the launcher, and the
     * turret shaking 10 deg either side to free anything tucked against it.
     *
     * @param intakeState how the intake rollers run: fully reversed for the normal unjam, or
     *     reversed with the kicker kept forward for the kicker unjam
     */
    private void unjam(FuelIntake.WantedState intakeState) {
        teleopDrive(false);
        fuelIntake.setWantedState(intakeState);
        dyeRotor.setWantedState(DyeRotor.WantedState.UNJAM);
        intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.REVERSE);
        launcherTower.setWantedState(LauncherTower.WantedState.UNJAM);
        turret.setWantedState(Turret.WantedState.UNJAM_SHAKE);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    private void forceHome() {
        teleopDrive(false);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        dyeRotor.setWantedState(DyeRotor.WantedState.OFF);
        intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_RETRACT);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        launcherTower.setWantedState(LauncherTower.WantedState.OFF);
        turret.setWantedState(Turret.WantedState.IDLE);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    /**
     * Runs one of the turret's pit checks with the rest of the robot quiet.
     *
     * <p>Bound to the pilot D-pad in test mode only, held to run. The flywheel, feeder, rotor and
     * intake are off rather than at their idle states, because these checks are run with people
     * standing at the robot and the only thing that should move is the turret. The hood goes home
     * for the same reason, parked rather than left where it was. The drive stays in ordinary teleop
     * drive, since the follow-a-tag check is worth driving around with.
     *
     * <p>{@link Turret.WantedState#OFF} is the resting member of this family: releasing a check
     * button lands here, so letting go stops the turret where it stands instead of handing it back
     * to {@link #applyIdle()}, which aims at the target. That distinction is the whole reason this
     * state exists, since these are the checks you run when the pose is not to be trusted, and
     * releasing a button a metre from the robot is not the moment to slew across the travel toward
     * a hub the robot is only guessing the direction of.
     */
    private void testTurret(Turret.WantedState turretState) {
        teleopDrive(false);
        fuelIntake.setWantedState(FuelIntake.WantedState.OFF);
        dyeRotor.setWantedState(DyeRotor.WantedState.OFF);
        intakeExtension.setWantedState(IntakeExtension.WantedState.STOPPED);
        launcher.setWantedState(Launcher.WantedState.OFF);
        launcherTower.setWantedState(LauncherTower.WantedState.OFF);
        turret.setWantedState(turretState);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    // Allocation-free boolean checks, for per-loop callers such as ShotCalculator.
    /** True when the robot is in the enemy or neutral zone, the feed zone. */
    public boolean isRobotInFeedZone() {
        return swerve.isInEnemyAllianceZone() || swerve.isInNeutralZone();
    }

    public void setWantedSuperState(WantedSuperState state) {
        this.wantedSuperState = state;
    }

    public Command setStateCommand(WantedSuperState state) {
        return new InstantCommand(() -> setWantedSuperState(state));
    }

    /** Picks a fixed shot and requests {@link WantedSuperState#SET_SHOT} in one command. */
    public Command setShotCommand(ShotCalculator.SetShot shot) {
        return new InstantCommand(
                        () -> {
                            ShotCalculator.selectSetShot(shot);
                            setWantedSuperState(WantedSuperState.SET_SHOT);
                        })
                .withName("SuperStructure.setShot." + shot.label);
    }
}

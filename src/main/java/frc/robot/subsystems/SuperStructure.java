package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import lombok.Getter;

public class SuperStructure extends SubsystemBase {

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
    /** Runs the periodic update. */
    @Override
    public void periodic() {
        currentSuperState = handleStateTransitions();

        // Restart the squeeze timer exactly once when first entering a squeeze state
        if (isSqueezeState(currentSuperState) && !isSqueezeState(previousSuperState)) {
            intakeSqueezeTimer.restart();
        }

        applyStates();

        previousSuperState = currentSuperState;

        Telemetry.log("SuperStructure/WantedSuperState", wantedSuperState.toString());
        Telemetry.log("SuperStructure/CurrentSuperState", currentSuperState.toString());
        Telemetry.log(
                "SuperStructure/IntakeSqueezeTimerElapsed", intakeSqueezeTimer.get(), "seconds");
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
        dyeRotor.setWantedState(DyeRotor.WantedState.INDEX_MAX);
        launcher.setWantedState(Launcher.WantedState.LAUNCH);
        launcherTower.setWantedState(LauncherTower.WantedState.INDEX_MAX);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);

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
        dyeRotor.setWantedState(DyeRotor.WantedState.INDEX_MAX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.AGITATE);
        launcher.setWantedState(Launcher.WantedState.LAUNCH);
        launcherTower.setWantedState(LauncherTower.WantedState.INDEX_MAX);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }
    /** Launches without squeeze. */
    private void launchWithoutSqueeze() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(SHOOTING_TELEOP_TRANSLATION_COEFFICIENT);
        swerve.setTeleopRotationVelocityCoefficient(SHOOTING_TELEOP_ROTATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        dyeRotor.setWantedState(DyeRotor.WantedState.INDEX_MAX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.LAUNCH);
        launcherTower.setWantedState(LauncherTower.WantedState.INDEX_MAX);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }
    /** Launches with brake. */
    private void launchWithBrake() {
        swerve.setWantedState(Swerve.WantedState.X_BRAKE);
        fuelIntake.setWantedState(FuelIntake.WantedState.SLOW_INTAKE);
        dyeRotor.setWantedState(DyeRotor.WantedState.INDEX_MAX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.AGITATE);
        launcher.setWantedState(Launcher.WantedState.LAUNCH);
        launcherTower.setWantedState(LauncherTower.WantedState.INDEX_MAX);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
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
        dyeRotor.setWantedState(DyeRotor.WantedState.INDEX_MAX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.LAUNCH);
        launcherTower.setWantedState(LauncherTower.WantedState.INDEX_MAX);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }
    /** Auton launch with squeeze. */
    private void autonLaunchWithSqueeze() {
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        dyeRotor.setWantedState(DyeRotor.WantedState.INDEX_MAX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.AGITATE);
        launcher.setWantedState(Launcher.WantedState.LAUNCH);
        launcherTower.setWantedState(LauncherTower.WantedState.INDEX_MAX);
        turret.setWantedState(Turret.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
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

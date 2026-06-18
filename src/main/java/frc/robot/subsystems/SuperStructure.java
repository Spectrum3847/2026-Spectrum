package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.fuelIntake.FuelIntake;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexerBed.IndexerBed;
import frc.robot.subsystems.indexerTower.IndexerTower;
import frc.robot.subsystems.intakeExtension.IntakeExtension;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.swerve.Swerve;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.util.Util;
import lombok.Getter;

public class SuperStructure extends SubsystemBase {

    @Getter private final Swerve swerve;
    @Getter private final FuelIntake fuelIntake;
    @Getter private final IntakeExtension intakeExtension;
    @Getter private final IndexerTower indexerTower;
    @Getter private final IndexerBed indexerBed;
    @Getter private final Launcher launcher;
    @Getter private final Hood hood;

    private static final double REGULAR_TELEOP_TRANSLATION_COEFFICIENT = 1.0;
    private static final double SHOOTING_TELEOP_TRANSLATION_COEFFICIENT = 0.1;

    public enum WantedSuperState {
        IDLE,
        INTAKE_FUEL,
        TRACK_TARGET,
        LAUNCH_WITH_SQUEEZE,
        LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY,
        LAUNCH_WITHOUT_SQUEEZE,
        AUTON_TRACK_TARGET,
        AUTON_INTAKE_FUEL,
        UNJAM,
        EJECT,
        FORCE_HOME,
    }

    public enum CurrentSuperState {
        IDLE,
        INTAKE_FUEL,
        TRACK_TARGET,
        LAUNCH_WITH_SQUEEZE,
        LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY,
        LAUNCH_WITHOUT_SQUEEZE,
        AUTON_IDLE,
        AUTON_TRACK_TARGET,
        AUTON_INTAKE_FUEL,
        UNJAM,
        EJECT,
        FORCE_HOME,
    }

    @Getter private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
    @Getter private CurrentSuperState currentSuperState = CurrentSuperState.IDLE;
    private CurrentSuperState previousSuperState = CurrentSuperState.IDLE;

    public SuperStructure(
            Swerve swerve,
            FuelIntake fuelIntake,
            IntakeExtension intakeExtension,
            IndexerTower indexerTower,
            IndexerBed indexerBed,
            Launcher launcher,
            Hood hood) {
        this.swerve = swerve;
        this.fuelIntake = fuelIntake;
        this.intakeExtension = intakeExtension;
        this.indexerTower = indexerTower;
        this.indexerBed = indexerBed;
        this.launcher = launcher;
        this.hood = hood;
    }

    private final Timer intakeSqueezeTimer = new Timer();
    private final double secondsToSqueeze = 1.0;

    private static boolean isSqueezeState(CurrentSuperState state) {
        return state == CurrentSuperState.LAUNCH_WITH_SQUEEZE;
    }

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
            case AUTON_TRACK_TARGET -> CurrentSuperState.AUTON_TRACK_TARGET;
            case AUTON_INTAKE_FUEL -> CurrentSuperState.AUTON_INTAKE_FUEL;
            case UNJAM -> CurrentSuperState.UNJAM;
            case EJECT -> CurrentSuperState.EJECT;
            case FORCE_HOME -> CurrentSuperState.FORCE_HOME;
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
            case AUTON_IDLE:
                applyAutonIdle();
                break;
            case AUTON_INTAKE_FUEL:
                autonIntakeFuel();
                break;
            case AUTON_TRACK_TARGET:
                autonTrackTarget();
                break;
            case UNJAM:
                unjam();
                break;
            case EJECT:
                eject();
                break;
            case FORCE_HOME:
                forceHome();
                break;
        }
    }

    // ── State methods ──────────────────────────────────────────────────────────

    private void applyIdle() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.OFF);
        intakeExtension.setWantedState(IntakeExtension.WantedState.STOPPED);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    private void intakeFuel() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.SLOW_INDEX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    private void trackTarget() {
        swerve.setWantedState(Swerve.WantedState.PILOT_AIM_AT_TARGET);
        swerve.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.OFF);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }

    private void launchWithSqueeze() {
        swerve.setWantedState(Swerve.WantedState.PILOT_AIM_AT_TARGET);
        swerve.setTeleopVelocityCoefficient(SHOOTING_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        indexerTower.setWantedState(IndexerTower.WantedState.INDEX_MAX);
        indexerBed.setWantedState(IndexerBed.WantedState.INDEX_MAX);
        launcher.setWantedState(Launcher.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);

        if (intakeSqueezeTimer.hasElapsed(secondsToSqueeze)) {
            intakeExtension.setWantedState(IntakeExtension.WantedState.SLOW_CLOSE);
            intakeSqueezeTimer.stop();
        } else {
            intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_EXTEND);
        }
    }

    private void launchWithSqueezeWithNoDelay() {
        swerve.setWantedState(Swerve.WantedState.PILOT_AIM_AT_TARGET);
        swerve.setTeleopVelocityCoefficient(SHOOTING_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        indexerTower.setWantedState(IndexerTower.WantedState.INDEX_MAX);
        indexerBed.setWantedState(IndexerBed.WantedState.INDEX_MAX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.SLOW_CLOSE);
        launcher.setWantedState(Launcher.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }

    private void launchWithoutSqueeze() {
        swerve.setWantedState(Swerve.WantedState.PILOT_AIM_AT_TARGET);
        swerve.setTeleopVelocityCoefficient(SHOOTING_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        indexerTower.setWantedState(IndexerTower.WantedState.INDEX_MAX);
        indexerBed.setWantedState(IndexerBed.WantedState.INDEX_MAX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }

    private void applyAutonIdle() {
        swerve.setWantedState(Swerve.WantedState.IDLE);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.OFF);
        intakeExtension.setWantedState(IntakeExtension.WantedState.STOPPED);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    private void autonIntakeFuel() {
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.SLOW_INDEX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    private void autonTrackTarget() {
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.OFF);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }

    private void unjam() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        indexerTower.setWantedState(IndexerTower.WantedState.UNJAM);
        indexerBed.setWantedState(IndexerBed.WantedState.UNJAM);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.OFF);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    private void eject() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.OUTTAKE);
        indexerTower.setWantedState(IndexerTower.WantedState.UNJAM);
        indexerBed.setWantedState(IndexerBed.WantedState.UNJAM);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.OFF);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    private void forceHome() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerve.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.OFF);
        intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_RETRACT);
        launcher.setWantedState(Launcher.WantedState.OFF);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    // ── Public API ─────────────────────────────────────────────────────────────

    public Command coastMechanisms() {
        return Commands.runOnce(
                () -> {
                    intakeExtension.setBrakeMode(false);
                    hood.setBrakeMode(false);
                });
    }

    public Command brakeMechanisms() {
        return Commands.runOnce(
                () -> {
                    intakeExtension.setBrakeMode(true);
                    hood.setBrakeMode(true);
                });
    }

    // Allocation-free boolean checks — use these in per-loop code (e.g. ShotCalculator).
    public boolean isRobotInNeutralZone() {
        return swerve.isInNeutralZone();
    }

    public boolean isRobotInEnemyZone() {
        return swerve.isInEnemyAllianceZone();
    }

    public boolean isRobotInFeedZone() {
        return isRobotInEnemyZone() || isRobotInNeutralZone();
    }

    public boolean isRobotInScoreZone() {
        return !isRobotInFeedZone();
    }

    // Trigger factories — use these for binding-time composition only.
    public Trigger robotInNeutralZone() {
        return new Trigger(this::isRobotInNeutralZone);
    }

    public Trigger robotInEnemyZone() {
        return new Trigger(this::isRobotInEnemyZone);
    }

    public Trigger robotInFeedZone() {
        return new Trigger(this::isRobotInFeedZone);
    }

    public Trigger robotInScoreZone() {
        return new Trigger(this::isRobotInScoreZone);
    }

    public void setWantedSuperState(WantedSuperState state) {
        this.wantedSuperState = state;
    }

    public Command setStateCommand(WantedSuperState state) {
        return new InstantCommand(() -> setWantedSuperState(state));
    }
}

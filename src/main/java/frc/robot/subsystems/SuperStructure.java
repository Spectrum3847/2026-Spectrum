package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
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
import lombok.Getter;

public class SuperStructure extends SubsystemBase {

    private final Swerve swerve;
    private final FuelIntake fuelIntake;
    private final IntakeExtension intakeExtension;
    private final IndexerTower indexerTower;
    private final IndexerBed indexerBed;
    private final Launcher launcher;
    private final Hood hood;

    public enum WantedSuperState {
        IDLE,
        INTAKE_FUEL,
        TRACK_TARGET,
        TRACK_TARGET_WITH_NO_SWERVE,
        LAUNCH_WITH_SQUEEZE,
        LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY,
        LAUNCH_WITHOUT_SQUEEZE,
        AUTON_TRACK_TARGET,
        AUTON_LAUNCH_WITH_SQUEEZE,
        UNJAM,
        FORCE_HOME,
    }

    public enum CurrentSuperState {
        IDLE,
        INTAKE_FUEL,
        TRACK_TARGET,
        TRACK_TARGET_WITH_NO_SWERVE,
        LAUNCH_WITH_SQUEEZE,
        LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY,
        LAUNCH_WITHOUT_SQUEEZE,
        AUTON_TRACK_TARGET,
        AUTON_LAUNCH_WITH_SQUEEZE,
        UNJAM,
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
        return state == CurrentSuperState.LAUNCH_WITH_SQUEEZE
                || state == CurrentSuperState.AUTON_LAUNCH_WITH_SQUEEZE;
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
            case IDLE -> CurrentSuperState.IDLE;
            case INTAKE_FUEL -> CurrentSuperState.INTAKE_FUEL;
            case TRACK_TARGET -> CurrentSuperState.TRACK_TARGET;
            case TRACK_TARGET_WITH_NO_SWERVE -> CurrentSuperState.TRACK_TARGET_WITH_NO_SWERVE;
            case LAUNCH_WITH_SQUEEZE -> CurrentSuperState.LAUNCH_WITH_SQUEEZE;
            case LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY -> CurrentSuperState
                    .LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY;
            case LAUNCH_WITHOUT_SQUEEZE -> CurrentSuperState.LAUNCH_WITHOUT_SQUEEZE;
            case AUTON_TRACK_TARGET -> CurrentSuperState.AUTON_TRACK_TARGET;
            case AUTON_LAUNCH_WITH_SQUEEZE -> CurrentSuperState.AUTON_LAUNCH_WITH_SQUEEZE;
            case UNJAM -> CurrentSuperState.UNJAM;
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
            case TRACK_TARGET_WITH_NO_SWERVE:
                trackTargetWithNoSwerve();
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
            case AUTON_TRACK_TARGET:
                autonTrackTarget();
                break;
            case AUTON_LAUNCH_WITH_SQUEEZE:
                autonLaunchWithSqueeze();
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

    private void applyIdle() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.OFF);
        intakeExtension.setWantedState(IntakeExtension.WantedState.STOPPED);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    private void intakeFuel() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.SLOW_INDEX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.IDLE_PREP);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    private void trackTarget() {
        swerve.setWantedState(Swerve.WantedState.PILOT_AIM_AT_TARGET);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.OFF);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }

    private void trackTargetWithNoSwerve() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.OFF);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }

    private void launchWithSqueeze() {
        swerve.setWantedState(Swerve.WantedState.PILOT_AIM_AT_TARGET);
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
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        indexerTower.setWantedState(IndexerTower.WantedState.INDEX_MAX);
        indexerBed.setWantedState(IndexerBed.WantedState.INDEX_MAX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.SLOW_CLOSE);
        launcher.setWantedState(Launcher.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }

    private void launchWithoutSqueeze() {
        swerve.setWantedState(Swerve.WantedState.PILOT_AIM_AT_TARGET);
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        indexerTower.setWantedState(IndexerTower.WantedState.INDEX_MAX);
        indexerBed.setWantedState(IndexerBed.WantedState.INDEX_MAX);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.AIM_AT_TARGET);
        hood.setWantedState(Hood.WantedState.AIM_AT_TARGET);
    }

    private void autonTrackTarget() {
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.OFF);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.AUTON_AIM);
        hood.setWantedState(Hood.WantedState.AUTON_AIM);
    }

    private void autonLaunchWithSqueeze() {
        fuelIntake.setWantedState(FuelIntake.WantedState.INTAKE);
        indexerTower.setWantedState(IndexerTower.WantedState.INDEX_MAX);
        indexerBed.setWantedState(IndexerBed.WantedState.INDEX_MAX);
        launcher.setWantedState(Launcher.WantedState.AUTON_AIM);
        hood.setWantedState(Hood.WantedState.AUTON_AIM);

        if (intakeSqueezeTimer.hasElapsed(secondsToSqueeze)) {
            intakeExtension.setWantedState(IntakeExtension.WantedState.SLOW_CLOSE);
            intakeSqueezeTimer.stop();
        } else {
            intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_EXTEND);
        }
    }

    private void unjam() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        indexerTower.setWantedState(IndexerTower.WantedState.UNJAM);
        indexerBed.setWantedState(IndexerBed.WantedState.UNJAM);
        intakeExtension.setWantedState(IntakeExtension.WantedState.CONDITIONAL_EXTEND);
        launcher.setWantedState(Launcher.WantedState.OFF);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    private void forceHome() {
        swerve.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        fuelIntake.setWantedState(FuelIntake.WantedState.NEUTRAL);
        indexerTower.setWantedState(IndexerTower.WantedState.OFF);
        indexerBed.setWantedState(IndexerBed.WantedState.OFF);
        intakeExtension.setWantedState(IntakeExtension.WantedState.FULL_RETRACT);
        launcher.setWantedState(Launcher.WantedState.OFF);
        hood.setWantedState(Hood.WantedState.HOME);
    }

    // ── Public API ─────────────────────────────────────────────────────────────
    public Trigger robotInNeutralZone() {
        return swerve.inNeutralZone();
    }

    public Trigger robotInEnemyZone() {
        return swerve.inEnemyAllianceZone();
    }

    public Trigger robotInFeedZone() {
        return robotInEnemyZone().or(robotInNeutralZone());
    }

    public Trigger robotInScoreZone() {
        return robotInFeedZone().negate();
    }

    public void setWantedSuperState(WantedSuperState state) {
        this.wantedSuperState = state;
    }

    public Command setStateCommand(WantedSuperState state) {
        return new InstantCommand(() -> setWantedSuperState(state));
    }
}

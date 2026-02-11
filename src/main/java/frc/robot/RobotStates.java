package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auton.Auton;
import frc.robot.launcher.LauncherStates;
import frc.robot.operator.Operator;
import frc.robot.pilot.Pilot;
import frc.robot.swerve.Swerve;
import frc.spectrumLib.Telemetry;
import lombok.Getter;

/**
 * Manages the high-level robot states.
 * Coordinates multiple subsystems based on the current robot state.
 */
public final class RobotStates {
    private static final Coordinator coordinator = Robot.getCoordinator();
    private static final Pilot pilot = Robot.getPilot();
    private static final Operator operator = Robot.getOperator();
    private static final Swerve swerve = Robot.getSwerve();

    @Getter public static State appliedState = State.IDLE;

    public static final Trigger robotInNeutralZone = swerve.inNeutralZone();
    public static final Trigger robotInEnemyZone = swerve.inEnemyAllianceZone();
    public static final Trigger robotInFeedZone = robotInEnemyZone.or(robotInNeutralZone);
    public static final Trigger robotInScoreZone = robotInFeedZone.not();

    public static final Trigger forceScore = operator.AButton;
    public static final Trigger hopperFull = new Trigger(() -> true);

    //placeholders
    private static final Trigger movementStable = new Trigger(() -> true);
    private static final Trigger visionStable = new Trigger(() -> true);

    public static final Trigger robotReadyScore = (robotInScoreZone).and(movementStable).and(visionStable);

    public static final Trigger robotReadyFeed = (robotInFeedZone).and(hopperFull).and(movementStable).and(visionStable);

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }

    public static void setupStates() {
        pressToState(pilot.AButton, State.INTAKE_FUEL);
        toggleToState(pilot.BButton, State.SNAKE_INTAKE);

        bindAimingWithReadyUpgrade(
                pilot.XButton,
                robotInScoreZone,
                State.TURRET_WITHOUT_TRACK,
                robotReadyScore,
                State.TURRET_WITHOUT_TRACK_WITH_LAUNCH);

        pilot.YButton.toggleOnTrue(LauncherStates.launchFuel());

        bindAimingWithReadyUpgrade(
                pilot.RB,
                robotInScoreZone,
                State.TURRET_TRACK,
                robotReadyScore,
                State.TURRET_TRACK_WITH_LAUNCH);

        bindAimingWithReadyUpgrade(
                pilot.RB,
                robotInFeedZone,
                State.TURRET_FEED_WITH_AIMING,
                robotReadyFeed,
                State.TURRET_FEED_WITH_LAUNCH);

        // Climb
        pilot.startButton.onTrue(applyState(State.L1_CLIMB_PREP));
        pilot.RB.and(pilot.startButton).onTrue(applyState(State.L1_CLIMB_EXECUTE));

        // Clear
        pilot.home_select.onTrue(clearState());

        // Auton
        Auton.autonIntake.onTrue(applyState(State.INTAKE_FUEL));
        Auton.autonShotPrep.onTrue(applyState(State.TURRET_TRACK));
        Auton.autonShoot.onTrue(applyState(State.TURRET_TRACK_WITH_LAUNCH));
        Auton.autonClearState.onTrue(clearState());
    }

    private static void toggleToState(Trigger button, State toggledState) {
        button.onTrue(
                new InstantCommand(
                        () -> {
                            State next = (appliedState == toggledState) ? State.IDLE : toggledState;
                            appliedState = next;
                            applyState(next);
                        })
        );
    }

    private static void pressToState(Trigger button, State pressedState) {
        button.onTrue(applyState(pressedState));
        button.onFalse(applyState(State.IDLE));
    } 

    private static void bindAimingWithReadyUpgrade(Trigger button, Trigger zone, State aimingState, Trigger readyTrigger, State readyState) {
        Trigger active = button.and(zone);

        active.onTrue(applyState(aimingState));
        active.onFalse(applyState(State.IDLE));

        active.and(readyTrigger).onTrue(applyState(readyState));
        active.and(readyTrigger.not()).onTrue(applyState(aimingState));
    }

    public static Command applyState(State state) {
        return new InstantCommand(
                        () -> {
                            appliedState = state;
                            SmartDashboard.putString("APPLIED STATE", state.toString());
                            SmartDashboard.putString("Current Zone", robotInNeutralZone.getAsBoolean() ? "Neutral" : robotInEnemyZone.getAsBoolean() ? "Enemy" : "Alliance");
                            Telemetry.print("Applied State: " + state);
                            coordinator.applyRobotState(state);
                        },
                        swerve)
                .withName("APPLYING STATE: " + state);
    }

    public static Command clearState() {
        return new InstantCommand(
                        () -> {
                            appliedState = State.IDLE;
                            SmartDashboard.putString("APPLIED STATE", "CLEARED TO IDLE");
                            coordinator.applyRobotState(State.IDLE);
                        },
                        swerve)
                .withName("CLEARING STATE TO IDLE");
    }
}

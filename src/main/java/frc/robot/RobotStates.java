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
 * This class coordinates multiple subsystems based on the current robot state.
 */
public class RobotStates {
    private static final Coordinator coordinator = Robot.getCoordinator();
    private static final Pilot pilot = Robot.getPilot();
    private static final Operator operator = Robot.getOperator();
    private static final Swerve swerve = Robot.getSwerve();

    @Getter
    private static State appliedState = State.IDLE;

    /**
     * Define Robot States here and how they can be triggered States should be
     * triggers that command
     * multiple mechanism or can be used in teleop or auton Use onTrue/whileTrue to
     * run a command
     * when entering the state Use onFalse/whileFalse to run a command when leaving
     * the state
     * RobotType Triggers
     */

    // Define triggers here
    public static final Trigger robotInNeutralZone = swerve.inNeutralZone();
    public static final Trigger robotInEnemyZone = swerve.inEnemyAllianceZone();
    public static final Trigger forceScore = operator.AButton;
    public static final Trigger hopperFull = new Trigger(() -> true);
    public static final Trigger robotInScoreZone = robotInEnemyZone.not().and(robotInNeutralZone.not());
    public static final Trigger robotInFeedZone = robotInEnemyZone.or(robotInNeutralZone);
    public static final Trigger robotReadyScore = (pilot.RB.or(pilot.XButton)).and(new Trigger(() -> true), robotInScoreZone); //movement stable + stable vision
    public static final Trigger robotReadyFeed = pilot.RB.and(new Trigger(hopperFull), robotInFeedZone); //movement stable + vision stable + hopper full
    
    public static void setupStates() {
        // Pilot Triggers
        pilot.AButton.onTrue(applyState(State.INTAKE_FUEL));
        pilot.AButton.onFalse(applyState(State.IDLE));

        pilot.BButton.onTrue(applyState(State.SNAKE_INTAKE)); //snake intake toggle
        pilot.BButton.onFalse(applyState(State.IDLE));

        pilot.XButton.onTrue(applyState(State.TURRET_WITHOUT_TRACK));
        pilot.XButton.onFalse(applyState(State.IDLE));
        robotReadyScore.onTrue(applyState(State.TURRET_WITHOUT_TRACK_WITH_LAUNCH));
        robotReadyScore.onFalse(applyState(State.TURRET_WITHOUT_TRACK));

        pilot.YButton.toggleOnTrue(LauncherStates.launchFuel());
        
        pilot.RB.and(robotInScoreZone).onTrue(applyState(State.TURRET_TRACK));
        pilot.RB.or(robotInScoreZone).onFalse(applyState(State.IDLE));
        robotReadyScore.onTrue(applyState(State.TURRET_TRACK_WITH_LAUNCH));
        robotReadyScore.onFalse(applyState(State.TURRET_TRACK));

        pilot.RB.and(robotInFeedZone).onTrue(applyState(State.TURRET_FEED_WITH_AIMING));
        pilot.RB.or(robotInFeedZone).onFalse(applyState(State.IDLE));
        robotReadyFeed.onTrue(applyState(State.TURRET_FEED_WITH_LAUNCH));
        robotReadyFeed.onFalse(applyState(State.TURRET_FEED_WITH_AIMING));

        pilot.startButton.onTrue(applyState(State.L1_CLIMB_PREP));
        pilot.RB.and(pilot.startButton).onTrue(applyState(State.L1_CLIMB_EXECUTE));

        pilot.home_select.onTrue(clearState());

        // Auton Triggers
        Auton.autonIntake.onTrue(applyState(State.INTAKE_FUEL));
        Auton.autonShotPrep.onTrue(applyState(State.TURRET_TRACK));
        Auton.autonShoot.onTrue(applyState(State.TURRET_TRACK_WITH_LAUNCH));
        Auton.autonClearState.onTrue(clearState());
    }

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }

    public static Command applyState(State state) {
        return new InstantCommand(
                () -> {
                    appliedState = state;
                    SmartDashboard.putString("APPLIED STATE", state.toString());
                    Telemetry.print("Applied State: " + state.toString());
                    coordinator.applyRobotState(state);
                })
                .withName("APPLYING STATE: " + state.toString());
    }

    public static Command clearState() {
        return new InstantCommand(
                () -> {
                    appliedState = State.IDLE;
                    SmartDashboard.putString("APPLIED STATE", "CLEARED TO IDLE");
                    coordinator.applyRobotState(State.IDLE);
                })
                .withName("CLEARING STATE TO IDLE");
    }
}

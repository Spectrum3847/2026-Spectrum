package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auton.Auton;
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
    public static final Trigger robotInFeedZone = robotInEnemyZone.or(robotInNeutralZone);
    // public static final Trigger hopperFull = new Trigger(null);

    // Setup any binding to set states
    public static void setupStates() {
        // Pilot Triggers
        pilot.AButton.onTrue(applyState(State.INTAKE_FUEL));
        pilot.AButton.onFalse(applyState(State.IDLE));
        pilot.BButton.onTrue(applyState(State.TURRET_TRACK_WITH_SPINUP));
        pilot.BButton.onFalse(applyState(State.TURRET_TRACK_WITH_LAUNCH));
        pilot.home_select.onTrue(clearState());

        robotInNeutralZone.or(robotInEnemyZone).whileTrue(applyState(State.TURRET_FEED_WITH_SPINUP));

        // Auton Triggers
        Auton.autonIntake.onTrue(applyState(State.INTAKE_FUEL).alongWith(RobotSim.mapleSimIntakeFuel()));
        Auton.autonShotPrep.onTrue(applyState(State.TURRET_TRACK_WITH_SPINUP));
        Auton.autonShoot.onTrue(applyState(State.TURRET_TRACK_WITH_LAUNCH).alongWith(RobotSim.mapleSimLaunchFuel()));
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

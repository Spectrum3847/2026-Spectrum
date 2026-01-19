package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.pilot.Pilot;
import frc.spectrumLib.Telemetry;
import lombok.Getter;

public class RobotStates {
    private static final Coordinator coordinator = Robot.getCoordinator();
    private static final Pilot pilot = Robot.getPilot();

    @Getter private static State appliedState = State.IDLE;

    /**
     * Define Robot States here and how they can be triggered States should be triggers that command
     * multiple mechanism or can be used in teleop or auton Use onTrue/whileTrue to run a command
     * when entering the state Use onFalse/whileFalse to run a command when leaving the state
     * RobotType Triggers
     */

     // Define triggers here

    // Setup any binding to set states
    public static void setupStates() {
        pilot.AButton.onTrue(applyState(State.INTAKING_WITH_INDEXER));
        pilot.AButton.onFalse(applyState(State.IDLE));
        pilot.BButton.onTrue(applyState(State.LAUNCHING_WITH_INDEXER));
        pilot.BButton.onFalse(applyState(State.IDLE));
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

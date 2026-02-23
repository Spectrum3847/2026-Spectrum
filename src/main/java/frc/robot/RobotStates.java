package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.rebuilt.ShiftHelpers;
import frc.robot.auton.Auton;
import frc.robot.pilot.Pilot;
import frc.robot.launcher.LauncherStates;
import frc.robot.swerve.SwerveStates;
import frc.robot.turretRotationalPivot.RotationalPivotStates;
import frc.spectrumLib.Telemetry;
import lombok.Getter;

/**
 * Manages the high-level robot states.
 * This class coordinates multiple subsystems based on the current robot state.
 */
public class RobotStates {
    private static final Coordinator coordinator = Robot.getCoordinator();
    private static final Pilot pilot = Robot.getPilot();

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
    public static final Trigger inNeutralZone = SwerveStates.robotInNeutralZone();
    public static final Trigger inEnemyZone = SwerveStates.robotInEnemyZone();
    public static final Trigger inFeedZone = inEnemyZone.or(inNeutralZone);
    public static final Trigger readyToLaunch = RotationalPivotStates.aimingAtTarget()
            .and(LauncherStates.aimingAtTarget());
    // public static final Trigger hopperFull = new Trigger(null);
    public static final Trigger idle = new Trigger(() -> appliedState == State.IDLE);
    public static final Trigger launchingForLEDS = new Trigger(() -> appliedState == State.TURRET_TRACK_WITH_LAUNCH);

    // Setup any binding to set states
    public static void setupStates() {
        // Pilot Triggers
        pilot.AButton.onTrue(applyState(State.INTAKE_FUEL));
        pilot.AButton.onFalse(applyState(State.IDLE));
        pilot.BButton.onTrue(applyState(State.TURRET_TRACK_WITH_SPINUP));
        pilot.BButton.onFalse(applyState(State.TURRET_TRACK_WITH_LAUNCH));
        pilot.home_select.onTrue(clearState());
        pilot.home_select.onFalse(clearState()); // forces inital state to be cleared on startup

        RotationalPivotStates.aimingAtTarget()
                .onTrue(new InstantCommand(() -> SmartDashboard.putBoolean("TurretOnTarget", true)));
        RotationalPivotStates.aimingAtTarget()
                .onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("TurretOnTarget", false)));
        LauncherStates.aimingAtTarget()
                .onTrue(new InstantCommand(() -> SmartDashboard.putBoolean("LauncherOnTarget", true)));
        LauncherStates.aimingAtTarget()
                .onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("LauncherOnTarget", false)));
        readyToLaunch.onTrue(new InstantCommand(() -> SmartDashboard.putBoolean("ReadyToLaunch", true)));
        readyToLaunch.onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("ReadyToLaunch", false)));

        // robotInNeutralZone.or(robotInEnemyZone).whileTrue(applyState(State.TURRET_FEED_WITH_SPINUP));

        // Auton Triggers
        Auton.autonIntake.onTrue(applyState(State.INTAKE_FUEL));
        Auton.autonShotPrep.onTrue(applyState(State.TURRET_TRACK_WITH_SPINUP));
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
                    Telemetry.print("Applied State: " + state.toString());
                    coordinator.applyRobotState(state);
                })
                .withName("APPLYING STATE: " + state.toString());
    }

    public static Command clearState() {
        return new InstantCommand(
                () -> {
                    appliedState = State.IDLE;
                    coordinator.applyRobotState(State.IDLE);
                })
                .withName("CLEARING STATE TO IDLE");
    }
}

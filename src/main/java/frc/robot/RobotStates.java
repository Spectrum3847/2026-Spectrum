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
import frc.robot.turretRotationalPivot.RotationalPivotStates;
import frc.spectrumLib.Telemetry;
import lombok.Getter;

/**
 * Manages the high-level robot states. This class coordinates multiple subsystems based on the
 * current robot state.
 */
public class RobotStates {
    private static final Coordinator coordinator = Robot.getCoordinator();
    private static final Pilot pilot = Robot.getPilot();
    private static final Operator operator = Robot.getOperator();
    private static final Swerve swerve = Robot.getSwerve();

    @Getter private static State appliedState = State.IDLE;

    /**
     * Define Robot States here and how they can be triggered States should be triggers that command
     * multiple mechanism or can be used in teleop or auton Use onTrue/whileTrue to run a command
     * when entering the state Use onFalse/whileFalse to run a command when leaving the state
     * RobotType Triggers
     */

    // Define triggers here
    public static final Trigger robotInNeutralZone = swerve.inNeutralZone();

    public static final Trigger robotInEnemyZone = swerve.inEnemyAllianceZone();
    public static final Trigger robotInFeedZone = robotInEnemyZone.or(robotInNeutralZone);
    public static final Trigger robotInScoreZone = robotInFeedZone.not();

    public static final Trigger forceScore = operator.AButton;

    public static final Trigger turretOnTarget = RotationalPivotStates.aimingAtTarget();
    public static final Trigger launcherOnTarget = LauncherStates.aimingAtTarget();
    public static final Trigger readyToLaunch = turretOnTarget.and(launcherOnTarget);

    // Setup any binding to set states
    public static void setupStates() {
        // Pilot Triggers
        pilot.RT.onTrue(applyState(State.INTAKE_FUEL));
        pilot.RT.onFalse(applyState(State.IDLE));

        pilot.LT.onTrue(applyState(State.TURRET_TRACK));
        pilot.LT.onFalse(applyState(State.TURRET_TRACK_WITH_LAUNCH));

        pilot.home_select.onTrue(clearState());
        pilot.home_select.onFalse(clearState()); // forces inital state to be cleared on startup

        turretOnTarget.onTrue(
                new InstantCommand(() -> SmartDashboard.putBoolean("TurretOnTarget", true)));
        turretOnTarget.onFalse(
                new InstantCommand(() -> SmartDashboard.putBoolean("TurretOnTarget", false)));
        launcherOnTarget.onTrue(
                new InstantCommand(() -> SmartDashboard.putBoolean("LauncherOnTarget", true)));
        launcherOnTarget.onFalse(
                new InstantCommand(() -> SmartDashboard.putBoolean("LauncherOnTarget", false)));
        readyToLaunch.onTrue(
                new InstantCommand(() -> SmartDashboard.putBoolean("ReadyToLaunch", true)));
        readyToLaunch.onFalse(
                new InstantCommand(() -> SmartDashboard.putBoolean("ReadyToLaunch", false)));

        // robotInNeutralZone.or(robotInEnemyZone).whileTrue(applyState(State.TURRET_FEED_WITH_SPINUP));

        // Auton Triggers
        Auton.autonIntake.onTrue(applyState(State.INTAKE_FUEL));
        Auton.autonShotPrep.onTrue(applyState(State.TURRET_TRACK));
        Auton.autonShoot.onTrue(applyState(State.TURRET_TRACK_WITH_LAUNCH));
        Auton.autonClearState.onTrue(clearState());
    }

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }

    private static void toggleToState(Trigger button, State toggledState) {
        button.onTrue(
                new InstantCommand(
                        () -> {
                            State next = (appliedState == toggledState) ? State.IDLE : toggledState;
                            appliedState = next;
                            SmartDashboard.putString("APPLIED STATE", next.toString());
                            coordinator.applyRobotState(next);
                        }));
    }

    private static void pressToState(Trigger button, State pressedState) {
        button.onTrue(applyState(pressedState));
        button.onFalse(applyState(State.IDLE));
    }

    private static void bindAimingWithReadyUpgrade(
            Trigger button,
            Trigger zone,
            State aimingState,
            Trigger readyTrigger,
            State readyState) {
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
                            Telemetry.print("Applied State: " + state);
                            coordinator.applyRobotState(state);
                        })
                .withName("APPLYING STATE: " + state);
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

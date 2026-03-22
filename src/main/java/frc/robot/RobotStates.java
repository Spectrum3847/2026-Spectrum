package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.rebuilt.ShiftHelpers;
import frc.robot.auton.Auton;
import frc.robot.launcher.LauncherStates;
import frc.robot.operator.Operator;
import frc.robot.pilot.Pilot;
import frc.robot.swerve.Swerve;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.util.Util;
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

    public static final Trigger launcherOnTarget = LauncherStates.aimingAtTarget();

    public static final Trigger autoUpdatePose = Auton.autonPoseUpdate;

    // Setup any binding to set states
    public static void setupStates() {
        // Pilot Triggers
        pilot.RT.whileTrue(applyState(State.INTAKE_FUEL));
        pilot.RT.onFalse(applyState(State.IDLE));

        pilot.XButton.whileTrue(applyState(State.TURRET_TRACK));
        pilot.XButton.onFalse(applyState(State.IDLE));

        pilot.LT.whileTrue(applyState(State.TURRET_TRACK_WITH_LAUNCH));
        pilot.LT.onFalse(applyState(State.IDLE));

        pilot.startButton.whileTrue(applyState(State.CUSTOM_SPEED_TURRET_LAUNCH));
        pilot.startButton.onFalse(applyState(State.IDLE));

        pilot.AButton.whileTrue(applyState(State.UNJAM));
        pilot.AButton.onFalse(applyState(State.IDLE));

        operator.AButton.whileTrue(applyState(State.UNJAM));
        operator.AButton.onFalse(applyState(State.IDLE));

        pilot.home_select.and(pilot.fn).onTrue(applyState(State.FORCE_HOME));
        pilot.home_select.and(pilot.fn).onFalse(applyState(State.IDLE));

        operator.testX.onTrue(applyState(State.TEST_INFINITE_LAUNCH));
        operator.testX.onFalse(applyState(State.TEST_IDLE));

        pilot.home_select.onTrue(clearState());
        pilot.home_select.onFalse(clearState()); // forces inital state to be cleared on startup

        // Telemetry bindings (keep logs in sync with trigger state)
        bindTriggerTelemetry("LauncherPrep/LauncherOnTarget", launcherOnTarget);
        
        // Reset hub shift timer when enabling
        Util.teleop.onTrue(Commands.runOnce(ShiftHelpers::initialize));
        Util.autoMode.onTrue(Commands.runOnce(ShiftHelpers::initialize));
        Util.disabled.onTrue(Commands.runOnce(ShiftHelpers::initialize).ignoringDisable(true));

        // Auton Triggers
        Auton.autonIntake.onTrue(applyState(State.INTAKE_FUEL));
        Auton.autonShotPrep.onTrue(applyState(State.TURRET_TRACK));
        Auton.autonShoot.onTrue(applyState(State.TURRET_TRACK_WITH_LAUNCH));
        Auton.autonUnjam.onTrue(
                Commands.sequence(
                        applyState(State.UNJAM),
                        Commands.waitSeconds(1),
                        applyState(State.TURRET_TRACK_WITH_LAUNCH)));
        Auton.autonClearState.onTrue(clearState());
    }

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }

    private static void bindTriggerTelemetry(String name, Trigger trigger) {
        trigger.onTrue(Commands.runOnce(() -> Telemetry.log(name, true)));
        trigger.onFalse(Commands.runOnce(() -> Telemetry.log(name, false)));
    }

    public static Command applyState(State state) {
        return Commands.runOnce(
                        () -> {
                            appliedState = state;
                            coordinator.applyRobotState(state);
                        })
                .withName("APPLYING STATE: " + state);
    }

    public static Command clearState() {
        return Commands.runOnce(
                        () -> {
                            appliedState = State.IDLE;
                            coordinator.applyRobotState(State.IDLE);
                        })
                .withName("CLEARING STATE TO IDLE");
    }
}

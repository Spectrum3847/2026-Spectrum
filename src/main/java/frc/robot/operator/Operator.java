package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.gamepads.Gamepad;

public class Operator extends Gamepad {

    private static double climberScalerDown = 0.5;
    private static double climberScalerUp = 0.5;

    // Triggers, these would be robot states such as intake, visionAim, etc.
    // If triggers need any of the config values set them in the constructor
    /*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simulation */

    public final Trigger enabled = teleop.or(testMode); // works for both teleop and testMode
    public final Trigger fn = leftBumper;
    public final Trigger noFn = fn.not();
    public final Trigger home_select = select.or(leftStickClick);
    public final Trigger startButton = start;

    public final Trigger RT = rightTrigger;
    public final Trigger LT = leftTrigger;

    public final Trigger maunelOverride = enabled.and(rightStickX.or(rightStickY));

    public final Trigger AButton = A.and(teleop);
    public final Trigger BButton = B.and(teleop);
    public final Trigger XButton = X.and(teleop);
    public final Trigger YButton = Y.and(teleop);

    public final Trigger testA = A.and(testMode);
    public final Trigger testB = B.and(testMode);
    public final Trigger testX = X.and(testMode);
    public final Trigger testY = Y.and(testMode);

    public final Trigger driving = testMode.and(leftStickX.or(leftStickY));
    public final Trigger steer = testMode.and(rightStickX.or(rightStickY));

    public final Trigger coastA = A.and(disabled);
    public final Trigger brakeB = B.and(disabled);

    public final Trigger resetIntakeExtensionPos = Y.and(fn);
    public final Trigger resetTurretPos = start.and(fn);

    public final Trigger moveTurretLeft = LT.and(fn);
    public final Trigger moveTurretRight = RT.and(fn);

    public final Trigger dpadUp = upDpad.and(teleop);
    public final Trigger dpadDown = downDpad.and(teleop);
    public final Trigger dpadLeft = leftDpad.and(teleop);
    public final Trigger dpadRight = rightDpad.and(teleop);

    public final Trigger rightStickTrigger = rightStickX.or(rightStickY);

    // DISABLED TRIGGERS
    public final Trigger coastOn_dB = disabled.and(B);
    public final Trigger coastOff_dA = disabled.and(A);

    public static class OperatorConfig extends Config {

        public OperatorConfig() {
            super("Operator", 1);
            setTriggersDeadzone(0.0);
        }
    }

    @SuppressWarnings("unused")
    private OperatorConfig config;

    public Operator(OperatorConfig config) {
        super(config);
        this.config = config;
        Robot.add(this);
        Telemetry.print("Operator Subsystem Initialized: ");
    }

    @Override
    public void setupStates() {
        // Left Blank so we can bind when the controller is connected
        OperatorStates.setStates();
    }

    @Override
    public void setupDefaultCommand() {
        OperatorStates.setupDefaultCommand();
    }

    public double getTriggerAxis() {
        return ((getRightTriggerAxis() * climberScalerUp)
                - (getLeftTriggerAxis() * climberScalerDown));
    }
}

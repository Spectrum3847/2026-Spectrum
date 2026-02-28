package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.gamepads.Gamepad;

public class Operator extends Gamepad {

    private static double climberScalerDown = 0.15;
    private static double climberScalerUp = 0.35;

    // Triggers, these would be robot states such as intake, visionAim, etc.
    // If triggers need any of the config values set them in the constructor
    /*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simulation */

    public final Trigger enabled = teleop.or(testMode); // works for both teleop and testMode
    public final Trigger fn = leftBumper;
    public final Trigger noFn = fn.not();
    public final Trigger home_select = select.or(leftStickClick);

    public final Trigger AButton = A.and(teleop);
    public final Trigger BButton = B.and(teleop);
    public final Trigger XButton = X.and(teleop);
    public final Trigger YButton = Y.and(teleop);

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

    public void setupStates() {
        // Left Blank so we can bind when the controller is connected
        OperatorStates.setStates();
    }

    public void setupDefaultCommand() {
        OperatorStates.setupDefaultCommand();
    }

    public double getClimberTriggerAxis() {
        return ((getRightTriggerAxis() * climberScalerUp)
                - (getLeftTriggerAxis() * climberScalerDown));
    }
}

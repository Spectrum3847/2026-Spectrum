package frc.robot.pilot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.gamepads.Gamepad;
import frc.spectrumLib.telemetry.Telemetry;

/* A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simulation */
public class Pilot extends Gamepad {
    public final Trigger LB = leftBumper;
    public final Trigger RB = rightBumper;
    public final Trigger LT = leftTrigger;
    public final Trigger RT = rightTrigger;

    public final Trigger AButton = A;
    public final Trigger BButton = B;
    public final Trigger XButton = X;
    public final Trigger YButton = Y;

    public final Trigger startButton = start;
    public final Trigger selectButton = select;

    public final Trigger leftStickPress = leftStickClick;
    public final Trigger rightStickPress = rightStickClick;

    public final Trigger dPadUp = upDpad;
    public final Trigger dPadDown = downDpad;
    public final Trigger dPadLeft = leftDpad;
    public final Trigger dPadRight = rightDpad;

    public static class PilotConfig extends Config {
        private double deadzone = 0.05;

        public PilotConfig() {
            super("Pilot", 0);

            setLeftStickDeadzone(deadzone);
            setLeftStickExp(3);
            // Set Scalar in Constructor from Swerve Config

            setRightStickDeadzone(deadzone);
            setRightStickExp(3.0);
            setRightStickScalar(6 * Math.PI);

            setTriggersDeadzone(deadzone);
            setTriggersExp(1);
            setTriggersScalar(1);
        }
    }

    @SuppressWarnings("unused")
    private PilotConfig config;

    /** Create a new Pilot with the default name and port. */
    public Pilot(PilotConfig config) {
        super(config);
        this.config = config;

        // Set Left stick Scalar from Swerve Config
        config.setLeftStickScalar(Robot.getConfig().swerve.getLinearSpeedAt12Volts().magnitude());
        leftStickCurve.setScalar(config.getLeftStickScalar());

        register();
        Telemetry.print("Pilot Subsystem Initialized: ");
    }

    public void setMaxVelocity(double maxVelocity) {
        leftStickCurve.setScalar(maxVelocity);
    }

    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        rightStickCurve.setScalar(maxRotationalVelocity);
    }

    // Positive is forward, up on the left stick is positive
    public double getDriveFwdPositive() {
        double fwdPositive = leftStickCurve.calculate(-1 * getLeftY());
        return fwdPositive;
    }

    // Positive is left, left on the left stick is positive
    public double getDriveLeftPositive() {
        double leftPositive = -1 * leftStickCurve.calculate(getLeftX());
        return leftPositive;
    }

    // Positive is counter-clockwise, left Trigger is positive
    public double getDriveCCWPositive() {
        double ccwPositive = rightStickCurve.calculate(getRightX());
        return -1 * ccwPositive; // invert the value
    }

    public double getPilotStickAngle() {
        return getLeftStickDirection().getRadians();
    }
}

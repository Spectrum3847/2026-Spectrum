package frc.robot.pilot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.gamepads.Gamepad;
import frc.spectrumLib.telemetry.Telemetry;

/* A, B, X, Y, Left Bumper, Right Bumper, Left Trigger, Right Trigger = Buttons 1 to 8 in simulation */
public class Pilot extends Gamepad {
    public final Trigger LB = leftBumper;
    public final Trigger noLB = LB.negate();
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

    /* LB + dPad reorients the robot front to a cardinal heading */
    public final Trigger upReorient = dPadUp.and(LB).and(teleop);
    public final Trigger leftReorient = dPadLeft.and(LB).and(teleop);
    public final Trigger downReorient = dPadDown.and(LB).and(teleop);
    public final Trigger rightReorient = dPadRight.and(LB).and(teleop);

    /* Coast/brake are pit controls, only live while disabled */
    public final Trigger coastA = AButton.and(disabled);
    public final Trigger brakeB = BButton.and(disabled);

    /* Pose reset works in every mode, so plain Select has to exclude LB to stay unambiguous */
    public final Trigger visionPoseReset_LB_Select = LB.and(selectButton);
    public final Trigger home_select = selectButton.and(noLB);

    public static class PilotConfig extends Config {
        private double deadzone = 0.10;
        /** Creates a new PilotConfig instance. */
        public PilotConfig() {
            super("Pilot", 0);

            setLeftStickDeadzone(deadzone);
            setLeftStickExp(3.0);

            setRightStickDeadzone(deadzone);
            setRightStickExp(3.0);

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

        config.setLeftStickScalar(
                Robot.getConfig().swerve.getLinearSpeedAt12Volts().in(MetersPerSecond));
        config.setRightStickScalar(
                Robot.getConfig().swerve.getAngularSpeedAt12Volts().in(RadiansPerSecond));
        leftStickCurve.setScalar(config.getLeftStickScalar());
        rightStickCurve.setScalar(config.getRightStickScalar());

        setDefaultCommand(rumbleCommand(0, 1).withName("Pilot.noRumble"));

        Telemetry.print("Pilot Subsystem Initialized: ");
    }
    /**
     * Sets the max velocity.
     *
     * @param maxVelocity the max velocity
     */
    public void setMaxVelocity(double maxVelocity) {
        leftStickCurve.setScalar(maxVelocity);
    }
    /**
     * Sets the max rotational velocity.
     *
     * @param maxRotationalVelocity the max rotational velocity
     */
    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        rightStickCurve.setScalar(maxRotationalVelocity);
    }

    // Positive is forward, up on the left stick is positive
    /**
     * Returns the signed chassis forward velocity.
     *
     * @return the forward velocity, positive when driving forward
     */
    public double getDriveFwdPositive() {
        double fwdPositive = leftStickCurve.calculate(-1 * getLeftY());
        return fwdPositive;
    }

    // Positive is left, left on the left stick is positive
    /**
     * Returns the signed chassis left velocity.
     *
     * @return the left velocity, positive when driving left
     */
    public double getDriveLeftPositive() {
        double leftPositive = -1 * leftStickCurve.calculate(getLeftX());
        return leftPositive;
    }

    // Positive is counter-clockwise, left Trigger is positive
    /**
     * Returns the signed chassis rotational velocity.
     *
     * @return the rotational velocity, positive counter-clockwise
     */
    public double getDriveCCWPositive() {
        double ccwPositive = rightStickCurve.calculate(getRightX());
        return -1 * ccwPositive; // invert the value
    }
    /**
     * Returns the pilot stick direction angle in radians.
     *
     * @return the stick angle in radians, not a raw joystick axis
     */
    public double getPilotStickAngle() {
        return getLeftStickDirection().getRadians();
    }
}

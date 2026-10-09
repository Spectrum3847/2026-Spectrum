package frc.robot.pilot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.gamepads.Gamepad;
import frc.spectrumLib.telemetry.Telemetry;

/* A, B, X, Y, Left Bumper, Right Bumper, Left Trigger, Right Trigger = Buttons 1 to 8 in sim */
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

    /* LB + dPad zeroes the robot front to a cardinal heading. */
    public final Trigger upReorient = dPadUp.and(LB).and(teleop);
    public final Trigger leftReorient = dPadLeft.and(LB).and(teleop);
    public final Trigger downReorient = dPadDown.and(LB).and(teleop);
    public final Trigger rightReorient = dPadRight.and(LB).and(teleop);

    /* Coast/brake are pit controls, only live while disabled. */
    public final Trigger coastA = AButton.and(disabled);
    public final Trigger brakeB = BButton.and(disabled);

    /* Pose reset works in every mode, so plain Select has to exclude LB to stay unambiguous. */
    public final Trigger visionPoseReset_LB_Select = LB.and(selectButton);
    public final Trigger home_select = selectButton.and(noLB);

    /*
     * Fixed shots, for when the pose is gone. LB plus a face button, so they can be held while
     * driving: left index on the bumper, right thumb on the face button, left thumb never leaves
     * the drive stick. One per parking spot, see ShotCalculator.SetShot.
     *
     * A, B and X are also bound bare below, so those bare bindings carry noLB and a bare press with
     * LB held would fire both. Release order matters at the margin: let go of LB first with the
     * face button still down and the bare binding fires for the rest of the press. Let go of the
     * face button first, or both together.
     */
    public final Trigger setShotLeftTrench_LB_X = LB.and(XButton).and(teleop);
    public final Trigger setShotRightTrench_LB_B = LB.and(BButton).and(teleop);
    public final Trigger setShotHubFace_LB_Y = LB.and(YButton).and(teleop);
    public final Trigger setShotTower_LB_A = LB.and(AButton).and(teleop);
    public final Trigger anySetShot =
            setShotLeftTrench_LB_X
                    .or(setShotRightTrench_LB_B)
                    .or(setShotHubFace_LB_Y)
                    .or(setShotTower_LB_A);

    /* Bare face buttons, gated so the chords above own the LB-held press. */
    public final Trigger trackTarget_X = XButton.and(noLB);
    public final Trigger unjam_A = AButton.and(noLB);
    public final Trigger kickerUnjam_B = BButton.and(noLB);

    /*
     * Turret pit checks, test mode only, held to run.
     *
     * The bare D-pad is the only part of the pilot nothing else claims: A, B, X, the triggers and
     * Select are all bound without a mode gate and so are still live in test mode, and the LB plus
     * D-pad reorients are teleop only. noLB keeps these and the reorients from ever reading the
     * same press, whatever mode the robot is in.
     */
    public final Trigger testTurretFollowTag_dPadUp = dPadUp.and(noLB).and(testMode);
    public final Trigger testTurretSweep_dPadLeft = dPadLeft.and(noLB).and(testMode);
    public final Trigger testTurretZero_dPadDown = dPadDown.and(noLB).and(testMode);

    public static class PilotConfig extends Config {
        private double deadzone = 0.10;

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

    public Pilot(PilotConfig config) {
        super(config);
        this.config = config;

        config.setLeftStickScalar(
                Robot.getConfig().swerve.getLinearSpeedAt12Volts().in(MetersPerSecond));
        config.setRightStickScalar(
                Robot.getConfig().swerve.getAngularSpeedAt12Volts().in(RadiansPerSecond));
        leftStickCurve.setScalar(config.getLeftStickScalar());
        rightStickCurve.setScalar(config.getRightStickScalar());

        // Hold the gamepad rather than reschedule a zero-intensity rumble every second, whose
        // initialize() and two HAL rumble writes showed up in every loop-overrun epoch print.
        setDefaultCommand(
                Commands.runOnce(() -> rumbleController(0, 0), this)
                        .andThen(Commands.idle(this))
                        .withName("Pilot.noRumble"));

        Telemetry.print("Pilot Subsystem Initialized: ");
    }

    public void setMaxVelocity(double maxVelocity) {
        leftStickCurve.setScalar(maxVelocity);
    }

    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        rightStickCurve.setScalar(maxRotationalVelocity);
    }

    /** Positive is forward, and up on the left stick is positive. */
    public double getDriveFwdPositive() {
        double fwdPositive = leftStickCurve.calculate(-1 * getLeftY());
        return fwdPositive;
    }

    /** Positive is left, and left on the left stick is positive. */
    public double getDriveLeftPositive() {
        double leftPositive = -1 * leftStickCurve.calculate(getLeftX());
        return leftPositive;
    }

    /** Signed chassis rotational velocity, positive counter-clockwise. */
    public double getDriveCCWPositive() {
        double ccwPositive = rightStickCurve.calculate(getRightX());
        return -1 * ccwPositive;
    }

    /** Stick direction angle in radians, not a raw joystick axis. */
    public double getPilotStickAngle() {
        return getLeftStickDirection().getRadians();
    }
}

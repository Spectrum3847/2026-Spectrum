package frc.spectrumLib.gamepads;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.util.ExpCurve;
import frc.spectrumLib.util.Util;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

/**
 * Xbox-compatible controller wrapper. Every button, stick, and D-pad direction has a public
 * trigger, plus composite bumper, trigger, and no-modifier triggers for chorded bindings.
 *
 * <p>With {@link Config#isAttached()} false no HID is opened, the button, stick, and D-pad triggers
 * stay on {@link #kFalse}, and axis reads return 0.0. The four composite triggers {@code
 * leftBumperOnly}, {@code rightBumperOnly}, {@code leftTriggerOnly} and {@code rightTriggerOnly}
 * are only built when a pad is attached, so without one they are null and binding or dereferencing
 * one throws.
 */
public abstract class Gamepad implements Subsystem {
    private Alert disconnectedAlert;

    public static final Trigger kFalse = new Trigger(() -> false);

    private CommandXboxController xboxController;

    protected Trigger A = kFalse; // bottom button, cross on a PlayStation pad

    protected Trigger B = kFalse; // right button, circle on a PlayStation pad

    protected Trigger X = kFalse; // left button, square on a PlayStation pad

    protected Trigger Y = kFalse; // top button, triangle on a PlayStation pad

    protected Trigger leftBumper = kFalse;

    protected Trigger rightBumper = kFalse;

    protected Trigger leftTrigger = kFalse;

    protected Trigger rightTrigger = kFalse;

    protected Trigger leftStickClick = kFalse;

    protected Trigger rightStickClick = kFalse;

    protected Trigger start = kFalse;

    protected Trigger select = kFalse;

    protected Trigger upDpad = kFalse;

    protected Trigger downDpad = kFalse;

    protected Trigger leftDpad = kFalse; // also fires for the up-left and down-left diagonals

    protected Trigger rightDpad = kFalse; // also fires for the up-right and down-right diagonals

    protected Trigger leftStickY = kFalse;

    protected Trigger leftStickX = kFalse;

    protected Trigger rightStickY = kFalse;

    protected Trigger rightStickX = kFalse;

    public Trigger noBumpers = kFalse;

    public Trigger leftBumperOnly;

    public Trigger rightBumperOnly;

    public Trigger bothBumpers = kFalse;

    public Trigger noTriggers = kFalse;

    public Trigger leftTriggerOnly;

    public Trigger rightTriggerOnly;

    public Trigger bothTriggers = kFalse;

    public Trigger noModifiers = kFalse;

    private Rotation2d storedLeftStickDirection = new Rotation2d();

    private Rotation2d storedRightStickDirection = new Rotation2d();

    /** True once the connected message has printed, so it prints once. */
    private boolean configured = false;

    /** True once the "gamepad not connected" message has printed, so it prints once. */
    private boolean printed = false;

    @Getter protected final ExpCurve leftStickCurve;

    @Getter protected final ExpCurve rightStickCurve;

    @Getter protected final ExpCurve triggersCurve;

    protected Trigger teleop = Util.teleop;

    protected Trigger autoMode = Util.autoMode;

    protected Trigger testMode = Util.testMode;

    protected Trigger disabled = Util.disabled;

    /**
     * Controller port, attachment flag, and per-axis curve settings. Every curve applies its
     * deadzone, then the exponent, then the scalar.
     */
    public static class Config {
        @Getter private String name;

        /** Zero-indexed USB port as shown in the DriverStation app. */
        @Getter private int port;

        /** Set false to skip binding this controller's triggers. */
        @Getter @Setter private boolean attached;

        @Getter @Setter double leftStickDeadzone = 0.001;

        @Getter @Setter double leftStickExp = 1.0;

        @Getter @Setter double leftStickScalar = 1.0;

        @Getter @Setter double rightStickDeadzone = 0.001;

        @Getter @Setter double rightStickExp = 1.0;

        @Getter @Setter double rightStickScalar = 1.0;

        @Getter @Setter double triggersDeadzone = 0.002;

        @Getter @Setter double triggersExp = 1.0;

        @Getter @Setter double triggersScalar = 1.0;

        public Config(String name, int port) {
            this.name = name;
            this.port = port;
        }
    }

    private Config config;

    protected Gamepad(Config config) {
        this.config = config;
        disconnectedAlert =
                new Alert(config.name + " Gamepad Disconnected", Alert.AlertType.kError);

        leftStickCurve =
                new ExpCurve(
                        config.getLeftStickExp(),
                        0,
                        config.getLeftStickScalar(),
                        config.getLeftStickDeadzone());
        rightStickCurve =
                new ExpCurve(
                        config.getRightStickExp(),
                        0,
                        config.getRightStickScalar(),
                        config.getRightStickDeadzone());
        triggersCurve =
                new ExpCurve(
                        config.getTriggersExp(),
                        0,
                        config.getTriggersScalar(),
                        config.getTriggersDeadzone());

        if (config.attached) {
            xboxController = new CommandXboxController(config.port);
            A = xboxController.a();
            B = xboxController.b();
            X = xboxController.x();
            Y = xboxController.y();
            leftBumper = xboxController.leftBumper();
            rightBumper = xboxController.rightBumper();
            leftTrigger = xboxController.leftTrigger(config.triggersDeadzone);
            rightTrigger = xboxController.rightTrigger(config.triggersDeadzone);
            leftStickClick = xboxController.leftStick();
            rightStickClick = xboxController.rightStick();
            start = xboxController.start();
            select = xboxController.back();
            upDpad = xboxController.povUp();
            downDpad = xboxController.povDown();
            leftDpad =
                    xboxController
                            .povLeft()
                            .or(xboxController.povUpLeft())
                            .or(xboxController.povDownLeft());
            rightDpad =
                    xboxController
                            .povRight()
                            .or(xboxController.povDownRight())
                            .or(xboxController.povUpRight());
            leftStickY = leftYTrigger(Threshold.ABS_GREATER, config.leftStickDeadzone);
            leftStickX = leftXTrigger(Threshold.ABS_GREATER, config.leftStickDeadzone);
            rightStickY = rightYTrigger(Threshold.ABS_GREATER, config.rightStickDeadzone);
            rightStickX = rightXTrigger(Threshold.ABS_GREATER, config.rightStickDeadzone);

            noBumpers = rightBumper.negate().and(leftBumper.negate());
            leftBumperOnly = leftBumper.and(rightBumper.negate());
            rightBumperOnly = rightBumper.and(leftBumper.negate());
            bothBumpers = rightBumper.and(leftBumper);
            noTriggers = leftTrigger.negate().and(rightTrigger.negate());
            leftTriggerOnly = leftTrigger.and(rightTrigger.negate());
            rightTriggerOnly = rightTrigger.and(leftTrigger.negate());
            bothTriggers = leftTrigger.and(rightTrigger);
            noModifiers = noBumpers.and(noTriggers);
        }

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        configure();
    }

    /** Raises the disconnect alert and prints a one-time message once the controller shows up. */
    public void configure() {
        if (config.isAttached()) {
            disconnectedAlert.set(!isConnected());

            if (!configured) {
                if (!isConnected()) {
                    if (!printed) {
                        Telemetry.print("##" + getName() + ": GAMEPAD NOT CONNECTED ##");
                        printed = true;
                    }
                    return;
                }

                configured = true;
                Telemetry.print("## " + getName() + ": gamepad is connected ##");
            }
        }
    }

    /**
     * Clears the connected flag so the next {@link #configure()} re-checks the gamepad. Pair it
     * with {@code CommandScheduler.getInstance().clearButtons()}, or the scheduler keeps the old
     * trigger bindings.
     */
    public void resetConfig() {
        configured = false;
        configure();
    }

    /**
     * Left stick direction as a {@link Rotation2d}: zero points up, positive is left. Retains the
     * last non-zero direction after the stick recenters.
     */
    public Rotation2d getLeftStickDirection() {
        double x = -1 * getLeftX();
        double y = -1 * getLeftY();
        if (x != 0 || y != 0) {
            Rotation2d angle = new Rotation2d(y, x);
            storedLeftStickDirection = angle;
        }
        return storedLeftStickDirection;
    }

    /** Right stick direction as a {@link Rotation2d}. Retains the last non-zero direction. */
    public Rotation2d getRightStickDirection() {
        double x = getRightX();
        double y = getRightY();
        if (x != 0 || y != 0) {
            Rotation2d angle = new Rotation2d(y, x);
            storedRightStickDirection = angle;
        }
        return storedRightStickDirection;
    }

    /** Snaps the left stick direction to 0, ±π/2, or π radians. */
    public double getLeftStickCardinals() {
        double stickAngle = getLeftStickDirection().getRadians();
        if (stickAngle > -Math.PI / 4 && stickAngle <= Math.PI / 4) {
            return 0;
        } else if (stickAngle > Math.PI / 4 && stickAngle <= 3 * Math.PI / 4) {
            return Math.PI / 2;
        } else if (stickAngle > 3 * Math.PI / 4 || stickAngle <= -3 * Math.PI / 4) {
            return Math.PI;
        } else {
            return -Math.PI / 2;
        }
    }

    /** Snaps the right stick direction to 0, ±π/2, or π radians. */
    public double getRightStickCardinals() {
        double stickAngle = getRightStickDirection().getRadians();
        if (stickAngle > -Math.PI / 4 && stickAngle <= Math.PI / 4) {
            return 0;
        } else if (stickAngle > Math.PI / 4 && stickAngle <= 3 * Math.PI / 4) {
            return Math.PI / 2;
        } else if (stickAngle > 3 * Math.PI / 4 || stickAngle <= -3 * Math.PI / 4) {
            return Math.PI;
        } else {
            return -Math.PI / 2;
        }
    }

    public double getLeftStickMagnitude() {
        double x = -1 * getLeftX();
        double y = -1 * getLeftY();
        return Math.sqrt(x * x + y * y);
    }

    public double getRightStickMagnitude() {
        double x = getRightX();
        double y = getRightY();
        return Math.sqrt(x * x + y * y);
    }

    /** Cardinal headings for the right stick, oriented for the current alliance. */
    public double chooseCardinalDirections() {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return getRedAllianceStickCardinals();
        }
        return getBlueAllianceStickCardinals();
    }

    /** Right stick snapped to 45° increments, forward as 0 radians, Blue alliance perspective. */
    public double getBlueAllianceStickCardinals() {
        double stickAngle = getRightStickDirection().getRadians();
        if (stickAngle > -Math.PI / 8 && stickAngle <= Math.PI / 8) {
            return 0;
        } else if (stickAngle > Math.PI / 8 && stickAngle <= 3 * Math.PI / 8) {
            return Math.PI / 4;
        } else if (stickAngle > 3 * Math.PI / 8 && stickAngle <= 5 * Math.PI / 8) {
            return Math.PI / 2;
        } else if (stickAngle > 5 * Math.PI / 8 && stickAngle <= 7 * Math.PI / 8) {
            return 3 * Math.PI / 4;
        } else if (stickAngle < -Math.PI / 8 && stickAngle >= -3 * Math.PI / 8) {
            return -Math.PI / 4;
        } else if (stickAngle < -3 * Math.PI / 8 && stickAngle >= -5 * Math.PI / 8) {
            return -Math.PI / 2;
        } else if (stickAngle < -5 * Math.PI / 8 && stickAngle >= -7 * Math.PI / 8) {
            return -3 * Math.PI / 4;
        } else {
            return Math.PI;
        }
    }

    /** Right stick snapped to 45° increments, forward as π radians, red alliance perspective. */
    public double getRedAllianceStickCardinals() {
        double stickAngle = getRightStickDirection().getRadians();

        if (stickAngle > -Math.PI / 8 && stickAngle <= Math.PI / 8) {
            return Math.PI;
        } else if (stickAngle > Math.PI / 8 && stickAngle <= 3 * Math.PI / 8) {
            return -3 * Math.PI / 4;
        } else if (stickAngle > 3 * Math.PI / 8 && stickAngle <= 5 * Math.PI / 8) {
            return -Math.PI / 2;
        } else if (stickAngle > 5 * Math.PI / 8 && stickAngle <= 7 * Math.PI / 8) {
            return -Math.PI / 4;
        } else if (stickAngle < -Math.PI / 8 && stickAngle >= -3 * Math.PI / 8) {
            return 3 * Math.PI / 4;
        } else if (stickAngle < -3 * Math.PI / 8 && stickAngle >= -5 * Math.PI / 8) {
            return Math.PI / 2;
        } else if (stickAngle < -5 * Math.PI / 8 && stickAngle >= -7 * Math.PI / 8) {
            return Math.PI / 4;
        } else {
            return 0;
        }
    }

    public Trigger leftYTrigger(Threshold t, double threshold) {
        return axisTrigger(t, threshold, this::getLeftY);
    }

    public Trigger leftXTrigger(Threshold t, double threshold) {
        return axisTrigger(t, threshold, this::getLeftX);
    }

    public Trigger rightYTrigger(Threshold t, double threshold) {
        return axisTrigger(t, threshold, this::getRightY);
    }

    public Trigger rightXTrigger(Threshold t, double threshold) {
        return axisTrigger(t, threshold, this::getRightX);
    }

    public Trigger rightStick(double threshold) {
        return new Trigger(
                () -> Math.abs(getRightX()) >= threshold || Math.abs(getRightY()) >= threshold);
    }

    public Trigger leftStick(double threshold) {
        return new Trigger(
                () -> Math.abs(getLeftX()) >= threshold || Math.abs(getLeftY()) >= threshold);
    }

    private Trigger axisTrigger(Threshold t, double threshold, DoubleSupplier v) {
        return new Trigger(
                () -> {
                    double value = v.getAsDouble();
                    switch (t) {
                        case GREATER:
                            return value > threshold;
                        case LESS:
                            return value < threshold;
                        case ABS_GREATER: // Also called Deadband
                            return Math.abs(value) > threshold;
                        default:
                            return false;
                    }
                });
    }

    public enum Threshold {
        GREATER,
        LESS,
        ABS_GREATER;
    }

    /**
     * Rumbles, then stops after {@code durationSeconds}.
     *
     * @param leftIntensity 0.0 to 1.0
     * @param rightIntensity 0.0 to 1.0
     */
    public Command rumbleCommand(
            double leftIntensity, double rightIntensity, double durationSeconds) {
        return Commands.sequence(
                        new InstantCommand(
                                () -> rumbleController(leftIntensity, rightIntensity), this),
                        Commands.waitSeconds(durationSeconds),
                        new InstantCommand(() -> rumbleController(0, 0), this))
                .ignoringDisable(true)
                .withName("Gamepad.Rumble");
    }

    public Command rumbleCommand(double intensity, double durationSeconds) {
        return rumbleCommand(intensity, intensity, durationSeconds);
    }

    /** Runs {@code command} alongside a half second of full intensity rumble. */
    public Command rumbleCommand(Command command) {
        return command.alongWith(rumbleCommand(1, 0.5)).withName(command.getName());
    }

    public boolean isConnected() {
        if (config.attached) {
            return this.getHID().isConnected();
        } else {
            return false;
        }
    }

    protected double getRightTriggerAxis() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getRightTriggerAxis();
    }

    protected double getLeftTriggerAxis() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getLeftTriggerAxis();
    }

    /** Right trigger minus left trigger, in the range -1 to 1. */
    protected double getTwist() {
        double right = getRightTriggerAxis();
        double left = getLeftTriggerAxis();
        double value = right - left;
        return value;
    }

    protected double getLeftX() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getLeftX();
    }

    protected double getLeftY() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getLeftY();
    }

    protected double getRightX() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getRightX();
    }

    protected double getRightY() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getRightY();
    }

    protected GenericHID getHID() {
        if (!config.attached) {
            return null;
        }
        return xboxController.getHID();
    }

    protected GenericHID getRumbleHID() {
        if (!isConnected()) {
            return null;
        }
        return xboxController.getHID();
    }

    /**
     * Sets both rumble motors now and never stops them. Use {@link #rumbleCommand(double, double,
     * double)} for a rumble that ends on its own.
     *
     * @param leftIntensity 0.0 to 1.0
     * @param rightIntensity 0.0 to 1.0
     */
    public void rumbleController(double leftIntensity, double rightIntensity) {
        if (!isConnected()) {
            return;
        }
        getRumbleHID().setRumble(RumbleType.kLeftRumble, leftIntensity);
        getRumbleHID().setRumble(RumbleType.kRightRumble, rightIntensity);
    }
}

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
 * Base class for the robot's Xbox-compatible gamepads. Wraps a WPILib {@link CommandXboxController}
 * and exposes the buttons, bumper and trigger combinations, stick axis helpers, and rumble commands
 * that bindings need.
 *
 * <p>Subclass once per operator role (pilot, copilot) and bind subsystem commands to the triggers
 * that are set up here.
 *
 * <p>When {@link Config#isAttached()} returns {@code false}, every trigger stays {@code false} for
 * the life of the object and axis reads return {@code 0.0}, because no controller is constructed.
 */
public abstract class Gamepad implements Subsystem {
    private Alert disconnectedAlert;

    /** A trigger that is always {@code false}; the default for every button before wiring. */
    public static final Trigger kFalse = new Trigger(() -> false);

    private CommandXboxController xboxController;

    /** Xbox A, the cross glyph. */
    protected Trigger A = kFalse;

    /** Xbox B, the circle glyph. */
    protected Trigger B = kFalse;

    /** Xbox X, the square glyph. */
    protected Trigger X = kFalse;

    /** Xbox Y, the triangle glyph. */
    protected Trigger Y = kFalse;

    protected Trigger leftBumper = kFalse;

    protected Trigger rightBumper = kFalse;

    /** Fires once the left analog trigger passes the configured trigger deadzone. */
    protected Trigger leftTrigger = kFalse;

    /** Fires once the right analog trigger passes the configured trigger deadzone. */
    protected Trigger rightTrigger = kFalse;

    protected Trigger leftStickClick = kFalse;

    protected Trigger rightStickClick = kFalse;

    /** Xbox Menu, exposed by WPILib as the start button. */
    protected Trigger start = kFalse;

    /** Xbox View, exposed by WPILib as the back button. */
    protected Trigger select = kFalse;

    protected Trigger upDpad = kFalse;

    protected Trigger downDpad = kFalse;

    /** Fires for left, up-left, or down-left on the D-pad. */
    protected Trigger leftDpad = kFalse;

    /** Fires for right, up-right, or down-right on the D-pad. */
    protected Trigger rightDpad = kFalse;

    /** Fires when the left stick's Y axis leaves the configured deadzone in either direction. */
    protected Trigger leftStickY = kFalse;

    /** Fires when the left stick's X axis leaves the configured deadzone in either direction. */
    protected Trigger leftStickX = kFalse;

    /** Fires when the right stick's Y axis leaves the configured deadzone in either direction. */
    protected Trigger rightStickY = kFalse;

    /** Fires when the right stick's X axis leaves the configured deadzone in either direction. */
    protected Trigger rightStickX = kFalse;

    public Trigger noBumpers = kFalse;

    public Trigger leftBumperOnly = kFalse;

    public Trigger rightBumperOnly = kFalse;

    public Trigger bothBumpers = kFalse;

    public Trigger noTriggers = kFalse;

    public Trigger leftTriggerOnly = kFalse;

    public Trigger rightTriggerOnly = kFalse;

    public Trigger bothTriggers = kFalse;

    public Trigger noModifiers = kFalse;

    /** Last non-zero left-stick direction, kept so the value survives the stick recentring. */
    private Rotation2d storedLeftStickDirection = new Rotation2d();

    /** Last non-zero right-stick direction, kept so the value survives the stick recentring. */
    private Rotation2d storedRightStickDirection = new Rotation2d();

    /** Set the first time the gamepad is seen connected. */
    private boolean configured = false;

    private boolean printed = false;

    /** Exponential response curve applied to both left-stick axes. */
    @Getter protected final ExpCurve leftStickCurve;

    /** Exponential response curve applied to both right-stick axes. */
    @Getter protected final ExpCurve rightStickCurve;

    @Getter protected final ExpCurve triggersCurve;

    protected Trigger teleop = Util.teleop;

    protected Trigger autoMode = Util.autoMode;

    protected Trigger testMode = Util.testMode;

    protected Trigger disabled = Util.disabled;

    /**
     * DriverStation USB port, axis curve parameters, and whether this robot uses the controller.
     */
    public static class Config {
        /** Human-readable controller name used in alerts and telemetry. */
        @Getter private String name;

        /** USB port number as shown in the DriverStation application (0-indexed). */
        @Getter private int port;

        /**
         * Whether this controller should be used on the current robot; {@code false} disables it.
         */
        @Getter @Setter private boolean attached;

        /** Deadzone applied to both left-stick axes before the exponential curve. */
        @Getter @Setter double leftStickDeadzone = 0.001;

        /** Exponent for the left-stick exponential response curve (1.0 = linear). */
        @Getter @Setter double leftStickExp = 1.0;

        /** Output scalar applied after the left-stick exponential curve. */
        @Getter @Setter double leftStickScalar = 1.0;

        /** Deadzone applied to both right-stick axes before the exponential curve. */
        @Getter @Setter double rightStickDeadzone = 0.001;

        /** Exponent for the right-stick exponential response curve. */
        @Getter @Setter double rightStickExp = 1.0;

        /** Output scalar applied after the right-stick exponential curve. */
        @Getter @Setter double rightStickScalar = 1.0;

        /** Deadzone applied to both analog trigger axes before the exponential curve. */
        @Getter @Setter double triggersDeadzone = 0.002;

        /** Exponent for the analog-trigger exponential response curve. */
        @Getter @Setter double triggersExp = 1.0;

        /** Output scalar applied after the analog-trigger exponential curve. */
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

    /** Calls {@link #configure()} once per robot loop. */
    @Override
    public void periodic() {
        configure();
    }

    /**
     * Raises the disconnect alert, and the first time the gamepad is seen connected, prints a
     * confirmation. Called from {@link #periodic()}.
     */
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
     * Clears the connection state so {@link #configure()} runs its detection again. Pair it with
     * {@code CommandScheduler.getInstance().clearButtons()}.
     */
    public void resetConfig() {
        configured = false;
        configure();
    }

    /**
     * Direction of the left stick, zero pointing up toward positive Y and 90 degrees pointing left.
     * The last non-zero direction is kept when the stick is released.
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

    /**
     * Direction of the right stick, keeping the last non-zero value when the stick is released.
     * Unlike {@link #getLeftStickDirection()}, the axis values are not negated.
     */
    public Rotation2d getRightStickDirection() {
        double x = getRightX();
        double y = getRightY();
        if (x != 0 || y != 0) {
            Rotation2d angle = new Rotation2d(y, x);
            storedRightStickDirection = angle;
        }
        return storedRightStickDirection;
    }

    /** Snaps the left-stick direction to the nearest of 0, ±π/2 and π radians. */
    public double getLeftStickCardinals() {
        return snapToCardinal(getLeftStickDirection().getRadians());
    }

    /** Snaps the right-stick direction to the nearest of 0, ±π/2 and π radians. */
    public double getRightStickCardinals() {
        return snapToCardinal(getRightStickDirection().getRadians());
    }

    /** Snaps an angle in radians to the nearest of 0, ±π/2 and π. */
    private static double snapToCardinal(double stickAngle) {
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

    /**
     * Euclidean magnitude of the left stick deflection: up to √2 before the curve and scalar, 1.0
     * after.
     */
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

    /** Cardinal stick angles rotated to match the current alliance's driver viewpoint. */
    public double chooseCardinalDirections() {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return getRedAllianceStickCardinals();
        }
        return getBlueAllianceStickCardinals();
    }

    /** Snaps the right stick to the nearest 45 degree increment, with forward as 0 rad. */
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

    /** Returns the blue-alliance cardinals rotated by pi, for red-alliance driving. */
    public double getRedAllianceStickCardinals() {
        double blue = getBlueAllianceStickCardinals();
        return blue > 0 ? blue - Math.PI : blue + Math.PI;
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

    /** Fires when either right-stick axis is at or beyond {@code threshold} in absolute value. */
    public Trigger rightStick(double threshold) {
        return new Trigger(
                () -> Math.abs(getRightX()) >= threshold || Math.abs(getRightY()) >= threshold);
    }

    /** Fires when either left-stick axis is at or beyond {@code threshold} in absolute value. */
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
        /** Fires when the axis value is strictly greater than the threshold. */
        GREATER,
        /** Fires when the axis value is strictly less than the threshold. */
        LESS,
        /** Fires when the absolute axis value is greater than the threshold (deadband check). */
        ABS_GREATER;
    }

    /**
     * Rumble command that holds the given intensities for a fixed time and then stops. It keeps
     * running while the robot is disabled.
     *
     * @param leftIntensity left rumble motor intensity, 0.0 to 1.0
     * @param rightIntensity right rumble motor intensity, 0.0 to 1.0
     * @param durationSeconds how long to rumble, in seconds
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

    /**
     * Rumble command with the same intensity on both motors.
     *
     * @param intensity rumble motor intensity, 0.0 to 1.0
     * @param durationSeconds how long to rumble, in seconds
     */
    public Command rumbleCommand(double intensity, double durationSeconds) {
        return rumbleCommand(intensity, intensity, durationSeconds);
    }

    /**
     * Runs {@code command} alongside a fixed 0.5 s full-strength rumble, under the same name.
     *
     * @param command command to run alongside the rumble
     */
    public Command rumbleCommand(Command command) {
        return command.alongWith(rumbleCommand(1, 0.5)).withName(command.getName());
    }

    public boolean isConnected() {
        return config.attached && getHID().isConnected();
    }

    /** Raw right-trigger axis, 0.0 to 1.0, or 0.0 when the gamepad is not connected. */
    protected double getRightTriggerAxis() {
        return axis(() -> xboxController.getRightTriggerAxis());
    }

    /** Raw left-trigger axis, 0.0 to 1.0, or 0.0 when the gamepad is not connected. */
    protected double getLeftTriggerAxis() {
        return axis(() -> xboxController.getLeftTriggerAxis());
    }

    /** rightTrigger minus leftTrigger, usable as a single twist axis in [-1, 1]. */
    protected double getTwist() {
        return getRightTriggerAxis() - getLeftTriggerAxis();
    }

    /** Raw left-stick X axis, -1.0 to 1.0, or 0.0 when not connected. */
    protected double getLeftX() {
        return axis(() -> xboxController.getLeftX());
    }

    /** Raw left-stick Y axis, -1.0 to 1.0, or 0.0 when not connected. Negative is up. */
    protected double getLeftY() {
        return axis(() -> xboxController.getLeftY());
    }

    /** Raw right-stick X axis, -1.0 to 1.0, or 0.0 when not connected. */
    protected double getRightX() {
        return axis(() -> xboxController.getRightX());
    }

    /** Raw right-stick Y axis, -1.0 to 1.0, or 0.0 when not connected. Negative is up. */
    protected double getRightY() {
        return axis(() -> xboxController.getRightY());
    }

    /**
     * Reads an axis, or returns {@code 0.0} when the controller is not connected. Callers pass a
     * lambda, not a method reference: {@code xboxController} is null when unattached.
     */
    private double axis(DoubleSupplier raw) {
        return isConnected() ? raw.getAsDouble() : 0.0;
    }

    /** The raw HID device for low-level access, or null when this gamepad is not attached. */
    protected GenericHID getHID() {
        if (!config.attached) {
            return null;
        }
        return xboxController.getHID();
    }

    /** The raw HID device for rumble output, or null unless the gamepad is attached and up. */
    protected GenericHID getRumbleHID() {
        if (!isConnected()) {
            return null;
        }
        return xboxController.getHID();
    }

    /**
     * Sets both rumble motors immediately. Use {@link #rumbleCommand(double, double, double)} for a
     * timed burst.
     *
     * @param leftIntensity left rumble motor intensity, 0.0 to 1.0
     * @param rightIntensity right rumble motor intensity, 0.0 to 1.0
     */
    public void rumbleController(double leftIntensity, double rightIntensity) {
        if (!isConnected()) {
            return;
        }
        getRumbleHID().setRumble(RumbleType.kLeftRumble, leftIntensity);
        getRumbleHID().setRumble(RumbleType.kRightRumble, rightIntensity);
    }
}

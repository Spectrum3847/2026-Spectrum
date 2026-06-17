package frc.spectrumLib.mechanism;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.hardware.TalonFXFactory;
import frc.spectrumLib.util.CachedDouble;
import frc.spectrumLib.util.CanDeviceId;
import frc.spectrumLib.util.Conversions;
import java.util.function.DoubleSupplier;
import lombok.*;

/**
 * Abstract base class representing a CTRE TalonFX-driven robot mechanism with common control modes,
 * telemetry, and convenience {@link edu.wpi.first.wpilibj2.command.Command} factories.
 *
 * <p>This class centralizes:
 *
 * <ul>
 *   <li>Motor creation/configuration (leader + optional followers) via the provided {@link Config}
 *   <li>Cached sensor readings (position, velocity, voltage, current) for efficient access
 *   <li>Common unit conversions (rotations/percent/degrees; RPS/RPM)
 *   <li>Standard closed-loop and open-loop control helpers (Motion Magic, velocity, voltage,
 *       percent, torque current)
 *   <li>Convenience {@link edu.wpi.first.wpilibj2.command.button.Trigger} factories (at/above/below
 *       thresholds)
 *   <li>Basic periodic current reporting to a battery/current logger
 * </ul>
 *
 * <h2>Attachment semantics</h2>
 *
 * If {@link Config#isAttached()} is {@code false}, the mechanism will not attempt to construct or
 * command hardware and will return safe default sensor values (typically {@code 0}).
 *
 * <h2>Target tracking</h2>
 *
 * The {@code target} field tracks the last commanded closed-loop setpoint sent by this class. It is
 * used by helper triggers such as {@code atTargetPosition(...)}.
 *
 * <h2>Extending</h2>
 *
 * Concrete mechanisms should provide a {@link Config} describing motor IDs, Talon configuration,
 * follower configuration, and mechanism-specific min/max rotation bounds as needed.
 *
 * <p><b>Note:</b> This class assumes CTRE Phoenix 6 units (rotations, rotations/sec, etc.) and uses
 * {@code Config.talonConfig.Feedback.SensorToMechanismRatio} as the mechanism gearing ratio.
 */
public abstract class Mechanism implements Subsystem {

    // ── Fields ─────────────────────────────────────────────────────────────────

    /** The primary (leader) TalonFX motor controller. */
    @Getter protected TalonFX motor;

    /** Optional follower TalonFX motor controllers that mirror the leader. */
    @Getter protected TalonFX[] followerMotors;

    /** Configuration object holding motor IDs, Talon settings, and mechanism parameters. */
    public Config config;

    /** Alert displayed when an unexpected current reading is detected during diagnostics. */
    Alert currentAlert = new Alert("", AlertType.kWarning);

    /** The last closed-loop position setpoint (in rotations) sent to the motor. */
    private double target = 0;

    /** The last closed-loop velocity setpoint (in rotations per second) sent to the motor. */
    private double velocityTarget = 0;

    // Cached sensor readings — updated lazily each loop via CachedDouble
    private final CachedDouble cachedRotations;
    private final CachedDouble cachedPercentage;
    private final CachedDouble cachedVoltage;
    private final CachedDouble cachedDegrees;
    private final CachedDouble cachedVelocity;
    private final CachedDouble cachedStatorCurrent;
    private final CachedDouble cachedSupplyCurrent;
    private final CachedDouble cachedTemp;

    // ── Constructors ───────────────────────────────────────────────────────────

    /**
     * Creates a Mechanism and, if {@link Config#isAttached()} is {@code true}, initializes the
     * leader TalonFX and any configured follower motors. Sensor caches are always initialized so
     * safe defaults ({@code 0}) are returned even when unattached.
     *
     * @param config the mechanism configuration (motor IDs, Talon settings, follower config, etc.)
     */
    protected Mechanism(Config config) {
        this.config = config;

        if (isAttached()) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
            BaseStatusSignal.setUpdateFrequencyForAll(
                    250,
                    motor.getDutyCycle(),
                    motor.getMotorVoltage(),
                    motor.getTorqueCurrent(),
                    motor.getStatorCurrent(),
                    motor.getSupplyCurrent(),
                    motor.getPosition(),
                    motor.getVelocity(),
                    motor.getDeviceTemp());
            motor.optimizeBusUtilization();

            followerMotors = new TalonFX[config.followerConfigs.length];
            for (int i = 0; i < config.followerConfigs.length; i++) {
                followerMotors[i] =
                        TalonFXFactory.createPermanentFollowerTalon(
                                config.followerConfigs[i].id,
                                motor,
                                config.followerConfigs[i].opposeLeader);
                BaseStatusSignal.setUpdateFrequencyForAll(
                        250,
                        followerMotors[i].getDutyCycle(),
                        followerMotors[i].getMotorVoltage(),
                        followerMotors[i].getTorqueCurrent(),
                        followerMotors[i].getStatorCurrent(),
                        followerMotors[i].getSupplyCurrent(),
                        followerMotors[i].getPosition(),
                        followerMotors[i].getVelocity(),
                        followerMotors[i].getDeviceTemp());
                followerMotors[i].optimizeBusUtilization();
            }
        }

        cachedStatorCurrent = new CachedDouble(this::updateStatorCurrent);
        cachedSupplyCurrent = new CachedDouble(this::updateSupplyCurrent);
        cachedVoltage = new CachedDouble(this::updateVoltage);
        cachedRotations = new CachedDouble(this::updatePositionRotations);
        cachedPercentage = new CachedDouble(this::updatePositionPercentage);
        cachedDegrees = new CachedDouble(this::updatePositionDegrees);
        cachedVelocity = new CachedDouble(this::updateVelocityRPM);
        cachedTemp = new CachedDouble(this::updateTemp);

        this.register();
    }

    /**
     * Creates a Mechanism and explicitly overrides the {@code attached} flag in the config.
     *
     * @param config the mechanism configuration
     * @param attached {@code true} to enable hardware; {@code false} to run in software-only mode
     */
    protected Mechanism(Config config, boolean attached) {
        // The override must be applied BEFORE the delegated constructor runs, since that
        // constructor decides whether to create the motor hardware based on the flag.
        this(applyAttachedOverride(config, attached));
    }

    private static Config applyAttachedOverride(Config config, boolean attached) {
        config.attached = attached;
        return config;
    }

    // ── Subsystem Overrides ────────────────────────────────────────────────────

    /**
     * Called once per scheduler loop. Concrete subclasses should override to implement their
     * periodic state-machine logic, telemetry, and sensor updates.
     */
    @Override
    public void periodic() {}

    /**
     * Called once per simulation loop. Concrete subclasses should override to update simulation
     * state (e.g., physics model inputs).
     */
    @Override
    public void simulationPeriodic() {}

    /**
     * Returns the human-readable name of this mechanism, as defined in its {@link Config}.
     *
     * @return the mechanism name
     */
    @Override
    public String getName() {
        return config.getName();
    }

    // ── Utility ────────────────────────────────────────────────────────────────

    /**
     * Returns {@code true} if physical hardware is attached and motor commands should be sent.
     *
     * @return {@code true} when hardware is available
     */
    public boolean isAttached() {
        return config.isAttached();
    }

    /**
     * Reports the combined supply current draw of the leader motor and all followers to the battery
     * logger. Does nothing if the mechanism is not attached.
     */
    public void logBatteryUsage() {
        if (isAttached()) {
            double motorCurrent = motor.getSupplyCurrent().getValueAsDouble();
            double followersCurrent = 0;
            for (TalonFX follower : followerMotors) {
                followersCurrent += follower.getSupplyCurrent().getValueAsDouble();
            }
            Robot.getBatteryLogger()
                    .reportCurrentUsage("Mechanisms/" + getName(), motorCurrent + followersCurrent);
        }
    }

    /**
     * Returns the name of the command currently scheduled on this subsystem, or {@code "none"} if
     * no command is running.
     *
     * @return the current command name
     */
    protected String getCurrentCommandName() {
        Command currentCommand = this.getCurrentCommand();
        if (currentCommand != null) {
            return currentCommand.getName();
        }
        return "none";
    }

    /**
     * Returns a {@link Trigger} that is active whenever this subsystem's current command is its
     * default command.
     *
     * @return trigger that is {@code true} while the default command is running
     */
    public Trigger runningDefaultCommand() {
        return new Trigger(this::isRunningDefaultCommand);
    }

    private boolean isRunningDefaultCommand() {
        return this.getCurrentCommand() == this.getDefaultCommand();
    }

    /**
     * Returns the last closed-loop setpoint (in rotations) sent to the motor by this class.
     *
     * @return the most recent target position in rotations
     */
    public double getTarget() {
        return target;
    }

    /**
     * Returns the last closed-loop velocity setpoint (in rotations per second) sent to the motor by
     * this class.
     *
     * @return the most recent target velocity in rotations per second
     */
    public double getVelocityTargetRPS() {
        return velocityTarget;
    }

    // ── Triggers ───────────────────────────────────────────────────────────────

    /**
     * Returns a {@link Trigger} that is active when the motor is within {@code tolerance} rotations
     * of the last commanded target position.
     *
     * @param tolerance maximum allowable error in rotations
     * @return trigger that is {@code true} when position error is within tolerance
     */
    public Trigger atTargetPosition(DoubleSupplier tolerance) {
        return new Trigger(() -> isAtTargetPosition(tolerance));
    }

    /**
     * Returns {@code true} when the motor is within {@code tolerance} rotations of the last
     * commanded target position.
     *
     * @param tolerance maximum allowable error in rotations
     * @return {@code true} when position error is within tolerance
     */
    public boolean isAtTargetPosition(DoubleSupplier tolerance) {
        return Math.abs(cachedRotations.getAsDouble() - target) < tolerance.getAsDouble();
    }

    /**
     * Returns a {@link Trigger} that is active when the motor position is within {@code tolerance}
     * of {@code target} (both in rotations).
     *
     * @param target the desired position in rotations
     * @param tolerance maximum allowable error in rotations
     * @return trigger that is {@code true} when position is within tolerance of target
     */
    public Trigger atRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionRotations() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    /**
     * Returns {@code true} when the motor position is within {@code tolerance} of {@code target}
     * (both in rotations).
     *
     * @param target the desired position in rotations
     * @param tolerance maximum allowable error in rotations
     * @return {@code true} when position is within tolerance of target
     */
    public boolean isAtRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return Math.abs(getPositionRotations() - target.getAsDouble()) < tolerance.getAsDouble();
    }

    /**
     * Returns a {@link Trigger} that is active when the motor position is below {@code target +
     * tolerance} rotations.
     *
     * @param target reference position in rotations
     * @param tolerance offset added to target to form the upper bound
     * @return trigger that is {@code true} when position is below the threshold
     */
    public Trigger belowRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionRotations() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} that is active when the motor position is above {@code target -
     * tolerance} rotations.
     *
     * @param target reference position in rotations
     * @param tolerance offset subtracted from target to form the lower bound
     * @return trigger that is {@code true} when position is above the threshold
     */
    public Trigger aboveRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionRotations() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} that is active when the motor position is within {@code tolerance}
     * of {@code target} (both as a percentage of max rotations).
     *
     * @param target the desired position as a percentage of max rotations
     * @param tolerance maximum allowable error as a percentage
     * @return trigger that is {@code true} when position is within tolerance of target
     */
    public Trigger atPercentage(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionPercentage() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    /**
     * Returns a {@link Trigger} that is active when the motor position is below {@code target +
     * tolerance} (as a percentage of max rotations).
     *
     * @param target reference position as a percentage
     * @param tolerance offset added to target to form the upper bound
     * @return trigger that is {@code true} when position is below the threshold
     */
    public Trigger belowPercentage(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionPercentage() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} that is active when the motor position is above {@code target -
     * tolerance} (as a percentage of max rotations).
     *
     * @param target reference position as a percentage
     * @param tolerance offset subtracted from target to form the lower bound
     * @return trigger that is {@code true} when position is above the threshold
     */
    public Trigger abovePercentage(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionPercentage() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} that is active when the motor position is within {@code tolerance}
     * of {@code target} (both in degrees).
     *
     * @param target the desired position in degrees
     * @param tolerance maximum allowable error in degrees
     * @return trigger that is {@code true} when position is within tolerance of target
     */
    public Trigger atDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionDegrees() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    /**
     * Returns a {@link Trigger} that is active when the motor position is below {@code target +
     * tolerance} degrees.
     *
     * @param target reference position in degrees
     * @param tolerance offset added to target to form the upper bound
     * @return trigger that is {@code true} when position is below the threshold
     */
    public Trigger belowDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionDegrees() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} that is active when the motor position is above {@code target -
     * tolerance} degrees.
     *
     * @param target reference position in degrees
     * @param tolerance offset subtracted from target to form the lower bound
     * @return trigger that is {@code true} when position is above the threshold
     */
    public Trigger aboveDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionDegrees() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} that is active when the motor velocity is within {@code tolerance}
     * RPM of {@code target}.
     *
     * @param target the desired velocity in RPM
     * @param tolerance maximum allowable error in RPM
     * @return trigger that is {@code true} when velocity is within tolerance of target
     */
    public Trigger atVelocityRPM(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> Math.abs(getVelocityRPM() - target.getAsDouble()) < tolerance.getAsDouble());
    }

    /**
     * Returns a {@link Trigger} that is active when the motor velocity is below {@code target +
     * tolerance} RPM.
     *
     * @param target reference velocity in RPM
     * @param tolerance offset added to target to form the upper bound
     * @return trigger that is {@code true} when velocity is below the threshold
     */
    public Trigger belowVelocityRPM(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getVelocityRPM() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} that is active when the motor velocity is above {@code target -
     * tolerance} RPM.
     *
     * @param target reference velocity in RPM
     * @param tolerance offset subtracted from target to form the lower bound
     * @return trigger that is {@code true} when velocity is above the threshold
     */
    public Trigger aboveVelocityRPM(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getVelocityRPM() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} that is active when the motor stator current is within {@code
     * tolerance} amps of {@code target}.
     *
     * @param target the desired stator current in amps
     * @param tolerance maximum allowable error in amps
     * @return trigger that is {@code true} when stator current is within tolerance of target
     */
    public Trigger atCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getStatorCurrent() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    /**
     * Returns a {@link Trigger} that is active when the motor stator current is below {@code target
     * + tolerance} amps.
     *
     * @param target reference current in amps
     * @param tolerance offset added to target to form the upper bound
     * @return trigger that is {@code true} when stator current is below the threshold
     */
    public Trigger belowCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getStatorCurrent() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} that is active when the motor stator current is above {@code target
     * - tolerance} amps.
     *
     * @param target reference current in amps
     * @param tolerance offset subtracted from target to form the lower bound
     * @return trigger that is {@code true} when stator current is above the threshold
     */
    public Trigger aboveCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getStatorCurrent() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    // ── Sensor Readings ────────────────────────────────────────────────────────

    /**
     * Reads the stator current directly from the motor hardware.
     *
     * @return motor stator current in amps, or {@code 0} if not attached
     */
    public double updateStatorCurrent() {
        if (config.attached) {
            return motor.getStatorCurrent().getValueAsDouble();
        }
        return 0;
    }

    /**
     * Returns the cached stator current of the motor.
     *
     * @return motor stator current in amps
     */
    public double getStatorCurrent() {
        return cachedStatorCurrent.getAsDouble();
    }

    /**
     * Reads the supply current directly from the motor hardware.
     *
     * @return motor supply current in amps, or {@code 0} if not attached
     */
    public double updateSupplyCurrent() {
        if (config.attached) {
            return motor.getSupplyCurrent().getValueAsDouble();
        }
        return 0;
    }

    /**
     * Returns the cached supply current of the motor.
     *
     * @return motor supply current in amps
     */
    public double getSupplyCurrent() {
        return cachedSupplyCurrent.getAsDouble();
    }

    /**
     * Reads the motor voltage directly from hardware.
     *
     * @return motor voltage in volts, or {@code 0} if not attached
     */
    public double updateVoltage() {
        if (config.attached) {
            return motor.getMotorVoltage().getValueAsDouble();
        }
        return 0;
    }

    /**
     * Returns the cached voltage of the motor.
     *
     * @return motor voltage in volts
     */
    public double getVoltage() {
        return cachedVoltage.getAsDouble();
    }

    /**
     * Reads the motor temperature directly from hardware.
     *
     * @return motor temperature in Celsius, or {@code 0} if not attached
     */
    public double updateTemp() {
        if (config.attached) {
            return motor.getDeviceTemp().getValueAsDouble();
        }
        return 0;
    }

    /**
     * Returns the cached temperature of the motor.
     *
     * @return motor temperature in Celsius
     */
    public double getTemp() {
        return cachedTemp.getAsDouble();
    }

    // ── Unit Conversions ───────────────────────────────────────────────────────

    /**
     * Converts a percentage of the mechanism's maximum range into an absolute rotation count.
     *
     * @param percent position as a percentage of max rotations (0–100)
     * @return the equivalent position in rotations
     */
    public double percentToRotations(DoubleSupplier percent) {
        return (percent.getAsDouble() / 100) * config.maxRotations;
    }

    /**
     * Converts an absolute rotation count into a percentage of the mechanism's maximum range.
     *
     * @param rotations position in rotations
     * @return the equivalent percentage of max rotations (0–100)
     */
    public double rotationsToPercent(DoubleSupplier rotations) {
        return (rotations.getAsDouble() / config.maxRotations) * 100;
    }

    /**
     * Converts degrees to rotations (1 rotation = 360 degrees).
     *
     * @param degrees angle in degrees
     * @return the equivalent position in rotations
     */
    public double degreesToRotations(DoubleSupplier degrees) {
        return (degrees.getAsDouble() / 360);
    }

    /**
     * Converts rotations to degrees (1 rotation = 360 degrees).
     *
     * @param rotations position in rotations
     * @return the equivalent angle in degrees
     */
    public double rotationsToDegrees(DoubleSupplier rotations) {
        return 360 * rotations.getAsDouble();
    }

    // ── Position & Velocity ────────────────────────────────────────────────────

    /**
     * Returns the cached motor position in rotations.
     *
     * @return motor position in rotations
     */
    public double getPositionRotations() {
        return cachedRotations.getAsDouble();
    }

    /**
     * Reads the motor position directly from hardware.
     *
     * @return motor position in rotations, or {@code 0} if not attached
     */
    private double updatePositionRotations() {
        if (config.attached) {
            return motor.getPosition().getValueAsDouble();
        }
        return 0;
    }

    /**
     * Returns the cached motor position as a percentage of max rotations.
     *
     * @return motor position in percentage of max rotations (0–100)
     */
    public double getPositionPercentage() {
        return cachedPercentage.getAsDouble();
    }

    /**
     * Computes the motor position as a percentage of max rotations using the cached rotation value.
     *
     * @return motor position in percentage of max rotations
     */
    private double updatePositionPercentage() {
        return rotationsToPercent(this::getPositionRotations);
    }

    /**
     * Returns the cached motor position in degrees.
     *
     * @return motor position in degrees
     */
    public double getPositionDegrees() {
        return cachedDegrees.getAsDouble();
    }

    /**
     * Computes the motor position in degrees using the cached rotation value.
     *
     * @return motor position in degrees
     */
    private double updatePositionDegrees() {
        return rotationsToDegrees(this::getPositionRotations);
    }

    /**
     * Reads the motor velocity directly from hardware in rotations per second (CTRE native units).
     *
     * @return motor velocity in rotations per second, or {@code 0} if not attached
     */
    private double updateVelocityRPS() {
        if (config.attached) {
            return motor.getVelocity().getValueAsDouble();
        }
        return 0;
    }

    /**
     * Returns the cached motor velocity in RPM.
     *
     * @return motor velocity in revolutions per minute
     */
    public double getVelocityRPM() {
        return cachedVelocity.getAsDouble();
    }

    /**
     * Computes the motor velocity in RPM from the raw RPS sensor reading.
     *
     * @return motor velocity in revolutions per minute
     */
    private double updateVelocityRPM() {
        return Conversions.RPStoRPM(updateVelocityRPS());
    }

    // ── Command Factories ──────────────────────────────────────────────────────

    /**
     * Returns a {@link Command} that continuously drives the mechanism at the specified velocity
     * using closed-loop voltage control.
     *
     * @param velocityRPM the target velocity in revolutions per minute
     * @return a command that runs the mechanism at the given velocity
     */
    public Command runVelocity(DoubleSupplier velocityRPM) {
        return run(() -> setVelocity(() -> Conversions.RPMtoRPS(velocityRPM)))
                .withName(getName() + ".runVelocity");
    }

    /**
     * Returns a {@link Command} that continuously drives the mechanism at the specified velocity
     * using closed-loop Torque Current FOC control (requires Phoenix Pro).
     *
     * @param velocityRPM the target velocity in revolutions per minute
     * @return a command that runs the mechanism at the given velocity using torque current FOC
     */
    public Command runVelocityTcFocRPM(DoubleSupplier velocityRPM) {
        return run(() -> setVelocityTorqueCurrentFOC(() -> Conversions.RPMtoRPS(velocityRPM)))
                .withName(getName() + ".runVelocityTcFocRPM");
    }

    /**
     * Returns a {@link Command} that continuously applies an open-loop percent output to the
     * mechanism using voltage compensation.
     *
     * @param percent fractional output between -1 and +1
     * @return a command that runs the mechanism at the given percent output
     */
    public Command runPercentage(DoubleSupplier percent) {
        return run(() -> setPercentOutput(percent)).withName(getName() + ".runPercentage");
    }

    /**
     * Returns a {@link Command} that continuously applies the specified voltage to the mechanism,
     * bypassing any closed-loop control.
     *
     * @param voltage the desired voltage in volts
     * @return a command that applies the given voltage output
     */
    public Command runVoltage(DoubleSupplier voltage) {
        return run(() -> setVoltageOutput(voltage)).withName(getName() + ".runVoltage");
    }

    /**
     * Returns a {@link Command} that continuously applies the specified voltage to the mechanism,
     * bypassing closed-loop control <em>and</em> ignoring software limit switches.
     *
     * @param voltage the desired voltage in volts
     * @return a command that applies the given voltage output, ignoring software limits
     */
    public Command runVoltageNoSoftLimit(DoubleSupplier voltage) {
        return run(() -> setVoltageOutputNoSoftLimit(voltage))
                .withName(getName() + ".runVoltageNoSoftLimit");
    }

    /**
     * Returns a {@link Command} that continuously drives the mechanism at the specified torque
     * current using FOC control (requires Phoenix Pro).
     *
     * @param current the desired torque current in amps
     * @return a command that runs the mechanism at the given torque current
     */
    public Command runTorqueCurrentFoc(DoubleSupplier current) {
        return run(() -> setTorqueCurrentFoc(current)).withName(getName() + ".runTorqueCurrentFoc");
    }

    /**
     * Returns a {@link Command} that continuously moves the mechanism to the specified position
     * using Motion Magic Torque Current FOC control (requires Phoenix Pro).
     *
     * @param rotations the target position in rotations
     * @return a command that moves the mechanism to the given position
     */
    public Command moveToRotations(DoubleSupplier rotations) {
        return run(() -> setMMPositionFoc(rotations)).withName(getName() + ".runPoseRevolutions");
    }

    /**
     * Returns a {@link Command} that continuously moves the mechanism to the specified position
     * using Motion Magic Torque Current FOC control (requires Phoenix Pro).
     *
     * @param percent the target position as a percentage of max rotations (0–100)
     * @return a command that moves the mechanism to the given percentage position
     */
    public Command moveToPercentage(DoubleSupplier percent) {
        return run(() -> setMMPositionFoc(() -> percentToRotations(percent)))
                .withName(getName() + ".runPosePercentage");
    }

    /**
     * Returns a {@link Command} that continuously moves the mechanism to the specified angular
     * position using Motion Magic Torque Current FOC control (requires Phoenix Pro).
     *
     * @param degrees the target position in degrees
     * @return a command that moves the mechanism to the given position in degrees
     */
    public Command moveToDegrees(DoubleSupplier degrees) {
        return run(() -> setMMPositionFoc(() -> degreesToRotations(degrees)))
                .withName(getName() + ".runPoseDegrees");
    }

    /**
     * Returns a {@link Command} that continuously moves the mechanism to the specified position
     * using Motion Magic Torque Current FOC control (requires Phoenix Pro).
     *
     * <p>Equivalent to {@link #moveToRotations(DoubleSupplier)} — prefer that method for clarity.
     *
     * @param rotations the target position in rotations
     * @return a command that moves the mechanism to the given position
     */
    public Command runFocRotations(DoubleSupplier rotations) {
        return run(() -> setMMPositionFoc(rotations)).withName(getName() + ".runFOCPosition");
    }

    /**
     * Returns a {@link Command} that stops the mechanism and holds it stopped for its duration.
     *
     * @return a command that stops the mechanism
     */
    public Command runStop() {
        return run(this::stop).withName(getName() + ".runStop");
    }

    /**
     * Returns a {@link Command} that sets the mechanism to coast mode while active, then reverts to
     * brake mode when the command ends. Safe to run while the robot is disabled.
     *
     * @return a command that temporarily enables coast mode
     */
    public Command coastMode() {
        return startEnd(() -> setBrakeMode(false), () -> setBrakeMode(true))
                .ignoringDisable(true)
                .withName(getName() + ".coastMode");
    }

    /**
     * Returns a {@link Command} that sets the mechanism to brake mode if it is currently in coast
     * mode. Safe to run while the robot is disabled.
     *
     * @return a command that ensures brake mode is active
     */
    public Command ensureBrakeMode() {
        return runOnce(() -> setBrakeMode(true))
                .onlyIf(
                        () ->
                                config.attached
                                        && config.talonConfig.MotorOutput.NeutralMode
                                                == NeutralModeValue.Coast)
                .ignoringDisable(true)
                .withName(getName() + ".ensureBrakeMode");
    }

    /**
     * Returns a {@link Command} that applies new supply and stator current limits to the mechanism.
     *
     * @param supplyLimit the new supply current limit in amps
     * @param statorLimit the new stator current limit in amps
     * @return a command that updates the current limits
     */
    protected Command runCurrentLimits(DoubleSupplier supplyLimit, DoubleSupplier statorLimit) {
        return Commands.runOnce(() -> setCurrentLimits(supplyLimit, statorLimit));
    }

    // ── Motor Control (Protected) ──────────────────────────────────────────────

    /**
     * Immediately applies new supply and stator current limits to the motor configuration.
     *
     * @param supplyLimit the new supply current limit in amps
     * @param statorLimit the new stator current limit in amps
     */
    protected void setCurrentLimits(DoubleSupplier supplyLimit, DoubleSupplier statorLimit) {
        applyCurrentLimit(supplyLimit, statorLimit);
    }

    /** Stops the motor output. Does nothing if the mechanism is not attached. */
    protected void stop() {
        if (isAttached()) {
            motor.stopMotor();
        }
    }

    /**
     * Sets the mechanism's reported position to zero (tares the motor encoder). Does nothing if the
     * mechanism is not attached.
     */
    protected void tareMotor() {
        if (isAttached()) {
            setMotorPosition(() -> 0);
        }
    }

    /**
     * Sets the motor's internal position register to the specified value without moving the motor.
     *
     * @param rotations the position to write to the motor in rotations
     */
    protected void setMotorPosition(DoubleSupplier rotations) {
        if (isAttached()) {
            motor.setPosition(rotations.getAsDouble());
        }
    }

    /**
     * Closed-loop velocity control using Motion Magic with Torque Current FOC (requires Phoenix
     * Pro).
     *
     * @param velocityRPS the target velocity in rotations per second
     */
    protected void setMMVelocityFOC(DoubleSupplier velocityRPS) {
        if (isAttached()) {
            velocityTarget = velocityRPS.getAsDouble();
            MotionMagicVelocityTorqueCurrentFOC mm =
                    config.mmVelocityFOC.withVelocity(velocityTarget);
            motor.setControl(mm);
        }
    }

    /**
     * Closed-loop velocity control using Torque Current FOC (requires Phoenix Pro).
     *
     * @param velocityRPS the target velocity in rotations per second
     */
    protected void setVelocityTorqueCurrentFOC(DoubleSupplier velocityRPS) {
        if (isAttached()) {
            velocityTarget = velocityRPS.getAsDouble();
            VelocityTorqueCurrentFOC output =
                    config.velocityTorqueCurrentFOC.withVelocity(velocityTarget);
            motor.setControl(output);
        }
    }

    /**
     * Closed-loop velocity control using Torque Current FOC with an RPM input (requires Phoenix
     * Pro). The RPM value is converted to RPS internally before being sent to the motor.
     *
     * @param velocityRPM the target velocity in revolutions per minute
     */
    protected void setVelocityTCFOCrpm(DoubleSupplier velocityRPM) {
        if (isAttached()) {
            velocityTarget = Conversions.RPMtoRPS(velocityRPM.getAsDouble());
            VelocityTorqueCurrentFOC output =
                    config.velocityTorqueCurrentFOC.withVelocity(velocityTarget);
            motor.setControl(output);
        }
    }

    /**
     * Closed-loop velocity control with voltage compensation.
     *
     * @param velocityRPS the target velocity in rotations per second
     */
    protected void setVelocity(DoubleSupplier velocityRPS) {
        if (isAttached()) {
            velocityTarget = velocityRPS.getAsDouble();
            VelocityVoltage output = config.velocityControl.withVelocity(velocityTarget);
            motor.setControl(output);
        }
    }

    /**
     * Closed-loop position control using Motion Magic with Torque Current FOC (requires Phoenix
     * Pro).
     *
     * @param rotations the target position in rotations
     */
    protected void setMMPositionFoc(DoubleSupplier rotations) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            MotionMagicTorqueCurrentFOC mm = config.mmPositionFOC.withPosition(target);
            motor.setControl(mm);
        }
    }

    /**
     * Closed-loop position control using Dynamic Motion Magic with Torque Current FOC (requires
     * Phoenix Pro). Trajectory parameters can be changed every loop cycle.
     *
     * @param rotations the target position in rotations
     * @param velocity the cruise velocity in rotations per second
     * @param acceleration the acceleration in rotations per second squared
     * @param jerk the jerk in rotations per second cubed
     */
    protected void setDynMMPositionFoc(
            DoubleSupplier rotations,
            DoubleSupplier velocity,
            DoubleSupplier acceleration,
            DoubleSupplier jerk) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            DynamicMotionMagicTorqueCurrentFOC mm =
                    config.dynamicMMPositionFOC
                            .withPosition(target)
                            .withVelocity(velocity.getAsDouble())
                            .withAcceleration(acceleration.getAsDouble())
                            .withJerk(jerk.getAsDouble());
            motor.setControl(mm);
        }
    }

    /**
     * Closed-loop position control using Dynamic Motion Magic with voltage compensation. Trajectory
     * parameters can be changed every loop cycle.
     *
     * @param rotations the target position in rotations
     * @param velocity the cruise velocity in rotations per second
     * @param acceleration the acceleration in rotations per second squared
     * @param jerk the jerk in rotations per second cubed
     */
    protected void setDynMMPositionVoltage(
            DoubleSupplier rotations,
            DoubleSupplier velocity,
            DoubleSupplier acceleration,
            DoubleSupplier jerk) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            DynamicMotionMagicVoltage mm =
                    config.dynamicMotionMagicVoltage
                            .withPosition(target)
                            .withVelocity(velocity.getAsDouble())
                            .withAcceleration(acceleration.getAsDouble())
                            .withJerk(jerk.getAsDouble());
            motor.setControl(mm);
        }
    }

    /**
     * Closed-loop position control using Motion Magic with voltage compensation (slot 0).
     *
     * @param rotations the target position in rotations
     */
    protected void setMMPosition(DoubleSupplier rotations) {
        setMMPosition(rotations, 0);
    }

    /**
     * Closed-loop position control using Motion Magic with voltage compensation and an explicit
     * PID/FF gain slot.
     *
     * @param rotations the target position in rotations
     * @param slot the gain slot to use (0, 1, or 2)
     */
    public void setMMPosition(DoubleSupplier rotations, int slot) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            MotionMagicVoltage mm =
                    config.mmPositionVoltageSlot.withSlot(slot).withPosition(target);
            motor.setControl(mm);
        }
    }

    // ── Motor Control (Public) ─────────────────────────────────────────────────

    /**
     * Open-loop percent output control with voltage compensation. The output voltage is {@code
     * percent × voltageCompSaturation}.
     *
     * @param percent fractional output between -1 and +1
     */
    public void setPercentOutput(DoubleSupplier percent) {
        if (isAttached()) {
            VoltageOut output =
                    config.voltageControl.withOutput(
                            config.voltageCompSaturation * percent.getAsDouble());
            motor.setControl(output);
        }
    }

    /**
     * Open-loop voltage control — applies the requested voltage directly without compensation
     * scaling.
     *
     * @param voltage the desired voltage in volts
     */
    public void setVoltageOutput(DoubleSupplier voltage) {
        if (isAttached()) {
            VoltageOut output = config.voltageControl.withOutput(voltage.getAsDouble());
            motor.setControl(output);
        }
    }

    /**
     * Open-loop voltage control that ignores software limit switches. Use with caution — this can
     * drive the mechanism past its configured travel limits.
     *
     * @param voltage the desired voltage in volts
     */
    public void setVoltageOutputNoSoftLimit(DoubleSupplier voltage) {
        if (isAttached()) {
            VoltageOut output =
                    config.voltageControl
                            .withOutput(voltage.getAsDouble())
                            .withIgnoreSoftwareLimits(true);
            motor.setControl(output);
        }
    }

    /**
     * Applies a torque current setpoint using FOC control (requires Phoenix Pro).
     *
     * @param current the desired torque current in amps
     */
    public void setTorqueCurrentFoc(DoubleSupplier current) {
        if (isAttached()) {
            TorqueCurrentFOC output = config.torqueCurrentFOC.withOutput(current.getAsDouble());
            motor.setControl(output);
        }
    }

    // ── Hardware Configuration ─────────────────────────────────────────────────

    /**
     * Sets the motor's neutral mode to brake or coast and immediately applies the change to
     * hardware.
     *
     * @param isInBrake {@code true} to set brake mode; {@code false} to set coast mode
     */
    public void setBrakeMode(boolean isInBrake) {
        if (isAttached()) {
            config.configNeutralBrakeMode(isInBrake);
            config.applyTalonConfig(motor);
        }
    }

    /**
     * Enables or disables the reverse software limit switch and immediately applies the change. The
     * threshold is read from the current configuration.
     *
     * @param enabled {@code true} to enable the reverse soft limit; {@code false} to disable it
     */
    public void toggleReverseSoftLimit(boolean enabled) {
        if (isAttached()) {
            double threshold = config.talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold;
            config.configReverseSoftLimit(threshold, enabled);
            config.applyTalonConfig(motor);
        }
    }

    /**
     * Enables or disables a forward/reverse torque current limit and immediately applies the
     * change. When disabled, the peak torque current is reset to ±300 A (effectively unlimited).
     *
     * @param enabledLimit the torque current limit in amps when {@code enabled} is {@code true}
     * @param enabled {@code true} to apply the limit; {@code false} to remove it
     */
    public void toggleTorqueCurrentLimit(DoubleSupplier enabledLimit, boolean enabled) {
        if (isAttached()) {
            if (enabled) {
                config.configForwardTorqueCurrentLimit(enabledLimit.getAsDouble());
                config.configReverseTorqueCurrentLimit(-1 * enabledLimit.getAsDouble());
                config.configStatorCurrentLimit(enabledLimit.getAsDouble(), true);
                config.applyTalonConfig(motor);
            } else {
                config.configForwardTorqueCurrentLimit(300);
                config.configReverseTorqueCurrentLimit(-300);
                config.applyTalonConfig(motor);
            }
        }
    }

    /**
     * Enables or disables the supply current limit and immediately applies the change.
     *
     * @param enabledLimit the supply current limit in amps
     * @param enabled {@code true} to enable the limit; {@code false} to disable it
     */
    public void toggleSupplyCurrentLimit(DoubleSupplier enabledLimit, boolean enabled) {
        if (isAttached()) {
            if (enabled) {
                config.configSupplyCurrentLimit(enabledLimit.getAsDouble(), true);
                config.applyTalonConfig(motor);
            } else {
                config.configSupplyCurrentLimit(enabledLimit.getAsDouble(), false);
                config.applyTalonConfig(motor);
            }
        }
    }

    /**
     * Applies new supply and stator current limits if the requested values differ from the
     * currently configured limits. The update is retried up to 10 times on failure.
     *
     * @param supplyLimit the new supply current limit in amps
     * @param statorLimit the new stator current limit in amps
     */
    public void applyCurrentLimit(DoubleSupplier supplyLimit, DoubleSupplier statorLimit) {
        if (isAttached()) {
            if (config.talonConfig.CurrentLimits.StatorCurrentLimit != statorLimit.getAsDouble()
                    || config.talonConfig.CurrentLimits.SupplyCurrentLimit
                            != supplyLimit.getAsDouble()) {
                config.configSupplyCurrentLimit(Math.abs(supplyLimit.getAsDouble()), true);
                config.configStatorCurrentLimit(Math.abs(statorLimit.getAsDouble()), true);
                config.configForwardTorqueCurrentLimit(Math.abs(statorLimit.getAsDouble()));
                config.configReverseTorqueCurrentLimit(-1 * Math.abs(statorLimit.getAsDouble()));
                for (int i = 0; i < 10; i++) {
                    StatusCode result = motor.getConfigurator().apply(config.talonConfig);
                    if (!result.isOK()) {
                        System.out.println(
                                "Could not apply config changes to "
                                        + config.getName()
                                        + "\'s motor ");
                    } else {
                        break;
                    }
                }
            }
        }
    }

    // ── Diagnostic Commands ────────────────────────────────────────────────────

    /**
     * Returns a {@link Command} that measures the average stator current over its runtime and fires
     * a warning {@link Alert} if the average deviates from {@code expectedCurrent} by more than
     * {@code tolerance}.
     *
     * @param expectedCurrent the expected average stator current in amps
     * @param tolerance the maximum acceptable deviation in amps
     * @return a diagnostic command that checks average current
     */
    public Command checkAvgCurrent(DoubleSupplier expectedCurrent, DoubleSupplier tolerance) {
        return new Command() {
            double totalCurrent = 0;
            int count = 0;
            String alertText = config.name + " AvgCurrent Error";

            @Override
            public void initialize() {
                totalCurrent = 0;
                count = 0;
            }

            @Override
            public void execute() {
                totalCurrent += getStatorCurrent();
                count++;
            }

            @Override
            public void end(boolean interrupted) {
                double avgCurrent = totalCurrent / count;
                if (Math.abs(avgCurrent - expectedCurrent.getAsDouble())
                        > tolerance.getAsDouble()) {
                    currentAlert.setText(
                            alertText
                                    + " Expected: "
                                    + expectedCurrent.getAsDouble()
                                    + " Actual: "
                                    + avgCurrent);
                    currentAlert.set(true);
                }
            }
        };
    }

    /**
     * Returns a {@link Command} that tracks the peak stator current over its runtime and fires a
     * warning {@link Alert} if the peak exceeds {@code expectedCurrent}.
     *
     * @param expectedCurrent the maximum acceptable peak stator current in amps
     * @return a diagnostic command that checks peak current
     */
    public Command checkMaxCurrent(DoubleSupplier expectedCurrent) {
        return new Command() {
            double maxCurrent = 0;
            String alertText = config.name + " MaxCurrent Error";

            @Override
            public void initialize() {
                maxCurrent = 0;
            }

            @Override
            public void execute() {
                double current = getStatorCurrent();
                if (current > maxCurrent) {
                    maxCurrent = current;
                }
            }

            @Override
            public void end(boolean interrupted) {
                if (maxCurrent > expectedCurrent.getAsDouble()) {
                    currentAlert.setText(
                            alertText
                                    + " Expected: "
                                    + expectedCurrent.getAsDouble()
                                    + " Actual: "
                                    + maxCurrent);
                    currentAlert.set(true);
                }
            }
        };
    }

    /**
     * Returns a {@link Command} that tracks the peak stator current over its runtime and fires a
     * warning {@link Alert} if the peak never reaches {@code expectedCurrent}. Use this to verify
     * that a mechanism drew at least the expected minimum load.
     *
     * @param expectedCurrent the minimum acceptable peak stator current in amps
     * @return a diagnostic command that checks whether a minimum current threshold was reached
     */
    public Command checkMinThresholdCurrent(DoubleSupplier expectedCurrent) {
        return new Command() {
            double maxCurrent = 0;
            String alertText = config.name + " Current Error";

            @Override
            public void initialize() {
                maxCurrent = 0;
            }

            @Override
            public void execute() {
                double current = getStatorCurrent();
                if (current > maxCurrent) {
                    maxCurrent = current;
                }
            }

            @Override
            public void end(boolean interrupted) {
                if (maxCurrent < expectedCurrent.getAsDouble()) {
                    currentAlert.setText(
                            alertText
                                    + " Expected at least: "
                                    + expectedCurrent.getAsDouble()
                                    + " Actual: "
                                    + maxCurrent);
                    currentAlert.set(true);
                }
            }
        };
    }

    // ── Nested Classes ─────────────────────────────────────────────────────────

    /**
     * Configuration for a TalonFX follower motor that mirrors the leader.
     *
     * <p>A follower automatically copies the leader's output. Set {@code opposeLeader} to {@link
     * MotorAlignmentValue#Opposed} when the physical motor is mounted in the opposite direction and
     * must spin in reverse to produce the same mechanism motion.
     */
    public static class FollowerConfig {

        /** Human-readable name for this follower motor (used in alerts and logging). */
        @Getter private String name;

        /** CAN bus device ID and bus name for this follower motor. */
        @Getter private CanDeviceId id;

        /** Whether hardware is attached for this follower. */
        @Getter private boolean attached = true;

        /**
         * Alignment of the follower relative to the leader. Use {@link MotorAlignmentValue#Opposed}
         * when the follower is physically mounted in the opposite direction.
         */
        @Getter private MotorAlignmentValue opposeLeader = MotorAlignmentValue.Aligned;

        /**
         * Creates a follower motor configuration.
         *
         * @param name human-readable name for this follower
         * @param id CAN device ID
         * @param canbus CAN bus name (e.g., {@code "rio"} or {@code "canivore"})
         * @param opposeLeader alignment relative to the leader motor
         */
        public FollowerConfig(
                String name, int id, String canbus, MotorAlignmentValue opposeLeader) {
            this.name = name;
            this.id = new CanDeviceId(id, canbus);
            this.opposeLeader = opposeLeader;
        }
    }

    /**
     * Configuration for a {@link Mechanism}, encapsulating the TalonFX hardware configuration,
     * control request objects, and mechanism-level parameters (gear ratio, soft limits, PID/FF
     * gains, Motion Magic profile, current limits, etc.).
     *
     * <p>Subclass this in each concrete mechanism and call the {@code config*()} helpers in the
     * constructor to set mechanism-specific parameters before passing the config to the {@link
     * Mechanism} constructor.
     */
    public static class Config {

        /** Human-readable name for this mechanism (used in logging and alerts). */
        @Getter private String name;

        /**
         * Whether physical hardware is attached. Set to {@code false} to run in simulation-only
         * mode.
         */
        @Getter @Setter private boolean attached = true;

        /** CAN bus device ID and bus name for the leader motor. */
        @Getter private CanDeviceId id;

        /** Full TalonFX hardware configuration applied to the leader motor on startup. */
        @Getter @Setter protected TalonFXConfiguration talonConfig;

        /** Total number of motors (leader + followers). */
        @Getter private int numMotors = 1;

        /**
         * Voltage compensation saturation value used by {@link Mechanism#setPercentOutput}.
         * Defaults to 12 V.
         */
        @Getter private double voltageCompSaturation = 12.0;

        /** Minimum mechanism position in rotations (used for range calculations). */
        @Getter private double minRotations = 0;

        /** Maximum mechanism position in rotations (used for range calculations). */
        @Getter private double maxRotations = 1;

        /** Configurations for follower motors. Empty by default (no followers). */
        @Getter private FollowerConfig[] followerConfigs = new FollowerConfig[0];

        // Pre-built control request objects — reused each loop to avoid GC pressure.

        @Getter
        private MotionMagicVelocityTorqueCurrentFOC mmVelocityFOC =
                new MotionMagicVelocityTorqueCurrentFOC(0);

        @Getter
        private MotionMagicTorqueCurrentFOC mmPositionFOC = new MotionMagicTorqueCurrentFOC(0);

        @Getter
        private DynamicMotionMagicTorqueCurrentFOC dynamicMMPositionFOC =
                new DynamicMotionMagicTorqueCurrentFOC(0, 0, 0);

        @Getter
        private DynamicMotionMagicVoltage dynamicMotionMagicVoltage =
                new DynamicMotionMagicVoltage(0, 0, 0);

        @Getter
        private MotionMagicVelocityVoltage mmVelocityVoltage = new MotionMagicVelocityVoltage(0);

        @Getter private MotionMagicVoltage mmPositionVoltage = new MotionMagicVoltage(0);

        @Getter
        private MotionMagicVoltage mmPositionVoltageSlot = new MotionMagicVoltage(0).withSlot(1);

        @Getter private VoltageOut voltageControl = new VoltageOut(0);
        @Getter private VelocityVoltage velocityControl = new VelocityVoltage(0);

        @Getter
        private VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

        @Getter private TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0);

        /** Percent (duty-cycle) output control — prefer {@link #voltageControl} in most cases. */
        @Getter private DutyCycleOut percentOutput = new DutyCycleOut(0);

        // ── Constructor ───────────────────────────────────────────────────────

        /**
         * Creates a base mechanism configuration with default Talon settings. Hardware limit
         * switches are disabled by default.
         *
         * @param name human-readable name for this mechanism
         * @param id CAN device ID of the leader motor
         * @param canbus CAN bus name (e.g., {@code "rio"} or {@code "canivore"})
         */
        public Config(String name, int id, String canbus) {
            this.name = name;
            this.id = new CanDeviceId(id, canbus);
            talonConfig = new TalonFXConfiguration();

            /* Put default config settings for all mechanisms here */
            talonConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
            talonConfig.HardwareLimitSwitch.ReverseLimitEnable = false;
        }

        // ── Config Helpers ────────────────────────────────────────────────────

        /**
         * Applies the current {@link TalonFXConfiguration} to the given motor and reports a warning
         * to the DriverStation if the apply fails.
         *
         * @param talon the TalonFX motor to configure
         */
        public void applyTalonConfig(TalonFX talon) {
            StatusCode result = talon.getConfigurator().apply(talonConfig);
            if (!result.isOK()) {
                DriverStation.reportWarning(
                        "Could not apply config changes to " + name + "\'s motor ", false);
            }
        }

        /**
         * Sets the follower motor configurations.
         *
         * @param followers one or more {@link FollowerConfig} objects describing follower motors
         */
        public void setFollowerConfigs(FollowerConfig... followers) {
            followerConfigs = followers;
        }

        /**
         * Sets the voltage compensation saturation voltage used by percent-output control.
         *
         * @param voltageCompSaturation the saturation voltage in volts (typically 12.0)
         */
        public void configVoltageCompensation(double voltageCompSaturation) {
            this.voltageCompSaturation = voltageCompSaturation;
        }

        /**
         * Configures the motor output as counter-clockwise positive (default for most mechanisms
         * when viewed from the shaft end).
         */
        public void configCounterClockwise_Positive() {
            talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        /** Configures the motor output as clockwise positive (inverted relative to the default). */
        public void configClockwise_Positive() {
            talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }

        /**
         * Sets the peak forward output voltage.
         *
         * @param voltageLimit maximum forward voltage in volts
         */
        public void configForwardVoltageLimit(double voltageLimit) {
            talonConfig.Voltage.PeakForwardVoltage = voltageLimit;
        }

        /**
         * Sets the peak reverse output voltage.
         *
         * @param voltageLimit maximum reverse voltage in volts
         */
        public void configReverseVoltageLimit(double voltageLimit) {
            talonConfig.Voltage.PeakReverseVoltage = voltageLimit;
        }

        /**
         * Configures the supply current limit. The absolute value of {@code supplyLimit} is used,
         * so negative values are automatically corrected.
         *
         * @param supplyLimit the supply current limit in amps
         * @param enabled {@code true} to enable the limit
         */
        public void configSupplyCurrentLimit(double supplyLimit, boolean enabled) {
            if (supplyLimit < 0) {
                supplyLimit = -supplyLimit;
            }
            talonConfig.CurrentLimits.SupplyCurrentLimit = supplyLimit;
            talonConfig.CurrentLimits.SupplyCurrentLimitEnable = enabled;
        }

        /**
         * Configures the stator current limit. The absolute value of {@code statorLimit} is used,
         * so negative values are automatically corrected.
         *
         * @param statorLimit the stator current limit in amps
         * @param enabled {@code true} to enable the limit
         */
        public void configStatorCurrentLimit(double statorLimit, boolean enabled) {
            if (statorLimit < 0) {
                statorLimit = -statorLimit;
            }
            talonConfig.CurrentLimits.StatorCurrentLimit = statorLimit;
            talonConfig.CurrentLimits.StatorCurrentLimitEnable = enabled;
        }

        /**
         * Sets the peak forward torque current limit. The absolute value is used so negative inputs
         * are corrected automatically.
         *
         * @param currentLimit peak forward torque current in amps
         */
        public void configForwardTorqueCurrentLimit(double currentLimit) {
            if (currentLimit < 0) {
                currentLimit = -currentLimit;
            }
            talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = currentLimit;
        }

        /**
         * Sets the peak reverse torque current limit. The value is forced negative so positive
         * inputs are corrected automatically.
         *
         * @param currentLimit peak reverse torque current in amps (sign is corrected if positive)
         */
        public void configReverseTorqueCurrentLimit(double currentLimit) {
            if (currentLimit > 0) {
                currentLimit = -currentLimit;
            }
            talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = currentLimit;
        }

        /**
         * Sets the lower supply current limit, used to reduce dissipation after the upper limit has
         * been triggered.
         *
         * @param currentLimit the lower supply current limit in amps
         */
        public void configLowerSupplyCurrentLimit(double currentLimit) {
            talonConfig.CurrentLimits.SupplyCurrentLowerLimit = currentLimit;
        }

        /**
         * Sets the time window for the lower supply current limit.
         *
         * @param time the time in seconds
         */
        public void configLowerSupplyCurrentTime(double time) {
            talonConfig.CurrentLimits.SupplyCurrentLowerTime = time;
        }

        /**
         * Sets the duty-cycle neutral deadband. Outputs below this magnitude are treated as zero.
         *
         * @param deadband deadband as a fraction of full output (e.g., {@code 0.001})
         */
        public void configNeutralDeadband(double deadband) {
            talonConfig.MotorOutput.DutyCycleNeutralDeadband = deadband;
        }

        /**
         * Sets the peak forward and reverse duty-cycle output limits.
         *
         * @param forward maximum forward output (0 to 1)
         * @param reverse maximum reverse output (-1 to 0)
         */
        public void configPeakOutput(double forward, double reverse) {
            talonConfig.MotorOutput.PeakForwardDutyCycle = forward;
            talonConfig.MotorOutput.PeakReverseDutyCycle = reverse;
        }

        /**
         * Configures the forward software limit switch.
         *
         * @param threshold the position threshold in rotations
         * @param enabled {@code true} to enable the limit
         */
        public void configForwardSoftLimit(double threshold, boolean enabled) {
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = threshold;
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = enabled;
        }

        /**
         * Configures the reverse software limit switch.
         *
         * @param threshold the position threshold in rotations
         * @param enabled {@code true} to enable the limit
         */
        public void configReverseSoftLimit(double threshold, boolean enabled) {
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = threshold;
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = enabled;
        }

        /**
         * Enables or disables continuous position wrap-around for closed-loop control. Useful for
         * mechanisms that rotate continuously (e.g., swerve azimuth).
         *
         * @param enabled {@code true} to enable continuous wrap
         */
        public void configContinuousWrap(boolean enabled) {
            talonConfig.ClosedLoopGeneral.ContinuousWrap = enabled;
        }

        /**
         * Configures optional Motion Magic velocity parameters (acceleration and feed-forward) for
         * both FOC and voltage velocity control requests.
         *
         * @param acceleration the velocity acceleration in rotations per second squared
         * @param feedforward the feed-forward term applied during velocity control
         */
        public void configMotionMagicVelocity(double acceleration, double feedforward) {
            mmVelocityFOC =
                    mmVelocityFOC.withAcceleration(acceleration).withFeedForward(feedforward);
            mmVelocityVoltage =
                    mmVelocityVoltage.withAcceleration(acceleration).withFeedForward(feedforward);
        }

        /**
         * Configures the feed-forward term for Motion Magic position control requests.
         *
         * @param feedforward the feed-forward term to apply during position control
         */
        public void configMotionMagicPosition(double feedforward) {
            mmPositionFOC = mmPositionFOC.withFeedForward(feedforward);
            mmPositionVoltage = mmPositionVoltage.withFeedForward(feedforward);
        }

        /**
         * Configures the Motion Magic cruise velocity, acceleration, and jerk limits.
         *
         * @param cruiseVelocity maximum cruise velocity in rotations per second
         * @param acceleration maximum acceleration in rotations per second squared
         * @param jerk maximum jerk in rotations per second cubed
         */
        public void configMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
            talonConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
            talonConfig.MotionMagic.MotionMagicAcceleration = acceleration;
            talonConfig.MotionMagic.MotionMagicJerk = jerk;
        }

        /**
         * Configures the sensor-to-mechanism gear ratio. This is the ratio of rotor rotations to
         * one full mechanism output rotation (or sensor rotations if a remote sensor is used).
         *
         * @param gearRatio the gear ratio (e.g., {@code 11.25} means 11.25 rotor turns per output
         *     rotation)
         */
        public void configGearRatio(double gearRatio) {
            talonConfig.Feedback.SensorToMechanismRatio = gearRatio;
        }

        /**
         * Returns the currently configured sensor-to-mechanism gear ratio.
         *
         * @return the gear ratio
         */
        public double getGearRatio() {
            return talonConfig.Feedback.SensorToMechanismRatio;
        }

        /**
         * Sets the motor neutral mode to brake or coast.
         *
         * @param isInBrake {@code true} for brake mode; {@code false} for coast mode
         */
        public void configNeutralBrakeMode(boolean isInBrake) {
            if (isInBrake) {
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            } else {
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            }
        }

        /**
         * Configures PID gains in slot 0.
         *
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         */
        public void configPIDGains(double kP, double kI, double kD) {
            configPIDGains(0, kP, kI, kD);
        }

        /**
         * Configures PID gains in the specified slot.
         *
         * @param slot the gain slot (0, 1, or 2)
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         */
        public void configPIDGains(int slot, double kP, double kI, double kD) {
            talonConfigFeedbackPID(slot, kP, kI, kD);
        }

        /**
         * Configures feed-forward gains in slot 0.
         *
         * @param kS static friction compensation (volts or amps)
         * @param kV velocity feed-forward gain
         * @param kA acceleration feed-forward gain
         * @param kG gravity/load compensation gain
         */
        public void configFeedForwardGains(double kS, double kV, double kA, double kG) {
            configFeedForwardGains(0, kS, kV, kA, kG);
        }

        /**
         * Configures feed-forward gains in the specified slot.
         *
         * @param slot the gain slot (0, 1, or 2)
         * @param kS static friction compensation (volts or amps)
         * @param kV velocity feed-forward gain
         * @param kA acceleration feed-forward gain
         * @param kG gravity/load compensation gain
         */
        public void configFeedForwardGains(int slot, double kS, double kV, double kA, double kG) {
            talonConfigFeedForward(slot, kV, kA, kS, kG);
        }

        /**
         * Configures the feedback sensor source using a rotorOffset of {@code 0}.
         *
         * @param source the feedback sensor source (e.g., remote CANcoder)
         */
        public void configFeedbackSensorSource(FeedbackSensorSourceValue source) {
            configFeedbackSensorSource(source, 0);
        }

        /**
         * Configures the feedback sensor source and its rotational offset.
         *
         * @param source the feedback sensor source
         * @param offset the feedback rotor offset in rotations
         */
        public void configFeedbackSensorSource(FeedbackSensorSourceValue source, double offset) {
            talonConfig.Feedback.FeedbackSensorSource = source;
            talonConfig.Feedback.FeedbackRotorOffset = offset;
        }

        /**
         * Configures the gravity compensation type in slot 0.
         *
         * @param isArm {@code true} for {@link GravityTypeValue#Arm_Cosine} (rotating arm); {@code
         *     false} for {@link GravityTypeValue#Elevator_Static} (elevator)
         */
        public void configGravityType(boolean isArm) {
            configGravityType(0, isArm);
        }

        /**
         * Configures the gravity compensation type in the specified slot.
         *
         * @param slot the gain slot (0, 1, or 2)
         * @param isArm {@code true} for {@link GravityTypeValue#Arm_Cosine} (rotating arm); {@code
         *     false} for {@link GravityTypeValue#Elevator_Static} (elevator)
         */
        public void configGravityType(int slot, boolean isArm) {
            GravityTypeValue gravityType =
                    isArm ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;
            if (slot == 0) {
                talonConfig.Slot0.GravityType = gravityType;
            } else if (slot == 1) {
                talonConfig.Slot1.GravityType = gravityType;
            } else if (slot == 2) {
                talonConfig.Slot2.GravityType = gravityType;
            } else {
                DriverStation.reportWarning("MechConfig: Invalid slot", false);
            }
        }

        /**
         * Sets the minimum and maximum rotation limits for the mechanism. These bounds are used by
         * unit-conversion helpers such as {@link Mechanism#percentToRotations}.
         *
         * @param minRotation the minimum position in rotations
         * @param maxRotation the maximum position in rotations
         */
        protected void configMinMaxRotations(double minRotation, double maxRotation) {
            this.minRotations = minRotation;
            this.maxRotations = maxRotation;
        }

        // ── Private Helpers ───────────────────────────────────────────────────

        /**
         * Applies feed-forward gains (kV, kA, kS, kG) to the specified TalonFX slot.
         *
         * @param slot the gain slot (0, 1, or 2)
         * @param kV velocity feed-forward
         * @param kA acceleration feed-forward
         * @param kS static friction compensation
         * @param kG gravity compensation
         */
        private void talonConfigFeedForward(int slot, double kV, double kA, double kS, double kG) {
            if (slot == 0) {
                talonConfig.Slot0.kV = kV;
                talonConfig.Slot0.kA = kA;
                talonConfig.Slot0.kS = kS;
                talonConfig.Slot0.kG = kG;
            } else if (slot == 1) {
                talonConfig.Slot1.kV = kV;
                talonConfig.Slot1.kA = kA;
                talonConfig.Slot1.kS = kS;
                talonConfig.Slot1.kG = kG;
            } else if (slot == 2) {
                talonConfig.Slot2.kV = kV;
                talonConfig.Slot2.kA = kA;
                talonConfig.Slot2.kS = kS;
                talonConfig.Slot2.kG = kG;
            } else {
                DriverStation.reportWarning("MechConfig: Invalid FeedForward slot", false);
            }
        }

        /**
         * Applies PID gains (kP, kI, kD) to the specified TalonFX slot.
         *
         * @param slot the gain slot (0, 1, or 2)
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         */
        private void talonConfigFeedbackPID(int slot, double kP, double kI, double kD) {
            if (slot == 0) {
                talonConfig.Slot0.kP = kP;
                talonConfig.Slot0.kI = kI;
                talonConfig.Slot0.kD = kD;
            } else if (slot == 1) {
                talonConfig.Slot1.kP = kP;
                talonConfig.Slot1.kI = kI;
                talonConfig.Slot1.kD = kD;
            } else if (slot == 2) {
                talonConfig.Slot2.kP = kP;
                talonConfig.Slot2.kI = kI;
                talonConfig.Slot2.kD = kD;
            } else {
                DriverStation.reportWarning("MechConfig: Invalid Feedback slot", false);
            }
        }
    }
}

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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.framework.SpectrumRobot;
import frc.spectrumLib.framework.SpectrumSubsystem;
import frc.spectrumLib.hardware.TalonFXFactory;
import frc.spectrumLib.util.CachedDouble;
import frc.spectrumLib.util.CanDeviceId;
import frc.spectrumLib.util.Conversions;
import java.util.function.DoubleSupplier;
import lombok.*;

/**
 * Abstract base class for all CTRE TalonFX-driven robot mechanisms.
 *
 * <p>Provides a unified API covering:
 *
 * <ul>
 *   <li><b>Hardware init</b> — leader + follower TalonFX construction and signal configuration
 *   <li><b>Sensor readings</b> — cached position (rotations, %, degrees), velocity (RPM), voltage,
 *       current, and temperature
 *   <li><b>Unit conversions</b> — rotations ↔ percent ↔ degrees; RPS ↔ RPM
 *   <li><b>Trigger factories</b> — at/above/below thresholds for position, velocity, and current
 *   <li><b>Command factories</b> — open-loop (voltage, percent, torque) and closed-loop (Motion
 *       Magic position, velocity) commands ready to be bound to triggers
 *   <li><b>Low-level setters</b> — direct motor-control calls used inside command lambdas
 *   <li><b>Configuration helpers</b> — runtime brake/coast, current limits, soft limits
 *   <li><b>Diagnostics</b> — current-check commands and battery-logger integration
 * </ul>
 *
 * <h2>Attachment semantics</h2>
 *
 * When {@link Config#isAttached()} returns {@code false}, no hardware is constructed and all sensor
 * methods return {@code 0}. This lets the same codebase run on robots that are missing a mechanism
 * without crashing.
 *
 * <h2>Target tracking</h2>
 *
 * The private {@code target} field stores the most-recently-commanded closed-loop setpoint (in
 * rotations or RPS depending on the control mode). Trigger factories like {@link
 * #atTargetPosition(DoubleSupplier)} compare live position against this value.
 *
 * <h2>Creating a concrete mechanism</h2>
 *
 * <ol>
 *   <li>Subclass {@code Mechanism} and create a matching inner {@link Config} subclass.
 *   <li>In the config constructor call the appropriate {@code config*()} helpers to set PID gains,
 *       gear ratio, current limits, soft limits, etc.
 *   <li>Override {@link #setupDefaultCommand()} and {@link #setupStates()} to bind triggers.
 *   <li>Add mechanism-specific {@code Command} methods that delegate to the protected setters.
 * </ol>
 *
 * <p><b>Units:</b> All CTRE Phoenix 6 signals use <em>rotations</em> for position and
 * <em>rotations-per-second</em> for velocity. Conversions to RPM, degrees, and percent-of-travel
 * are provided by this class.
 */
public abstract class Mechanism implements SpectrumSubsystem {

    // =========================================================================
    // Fields & Construction
    // =========================================================================

    /** The leader TalonFX motor controller. {@code null} when not attached. */
    @Getter protected TalonFX motor;

    /** Follower TalonFX motor controllers. Empty array when not attached. */
    @Getter protected TalonFX[] followerMotors;

    /**
     * Mechanism configuration. Declared {@code public} so subclasses can shadow it with a narrower
     * type (e.g. {@code private MyConfig config}) while still allowing the parent to access common
     * fields.
     */
    public Config config;

    /** WPILib alert raised by current-check diagnostic commands. */
    Alert currentAlert = new Alert("", AlertType.kWarning);

    /**
     * Last commanded closed-loop setpoint sent to the motor. Units depend on control mode:
     * rotations for position modes, RPS for velocity modes.
     */
    private double target = 0;

    // Cached sensor values — refreshed once per accessor call per loop cycle
    private final CachedDouble cachedRotations;
    private final CachedDouble cachedPercentage;
    private final CachedDouble cachedDegrees;
    private final CachedDouble cachedVelocity;
    private final CachedDouble cachedVoltage;
    private final CachedDouble cachedStatorCurrent;
    private final CachedDouble cachedSupplyCurrent;
    private final CachedDouble cachedTemp;

    /**
     * Constructs the mechanism, initializing the leader and all follower TalonFX motors when
     * attached, wiring up cached sensor suppliers, and registering the subsystem with the {@link
     * SpectrumRobot} registry and WPILib {@link edu.wpi.first.wpilibj2.command.CommandScheduler}.
     *
     * @param config the mechanism configuration describing motor IDs, gains, limits, and followers
     */
    protected Mechanism(Config config) {
        this.config = config;

        if (isAttached()) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);

            // Run high-priority signals at 100 Hz to keep cached values fresh
            BaseStatusSignal.setUpdateFrequencyForAll(
                    100,
                    motor.getDutyCycle(),
                    motor.getMotorVoltage(),
                    motor.getTorqueCurrent(),
                    motor.getStatorCurrent(),
                    motor.getSupplyCurrent(),
                    motor.getPosition(),
                    motor.getVelocity());
            motor.optimizeBusUtilization();

            followerMotors = new TalonFX[config.followerConfigs.length];
            for (int i = 0; i < config.followerConfigs.length; i++) {
                followerMotors[i] =
                        TalonFXFactory.createPermanentFollowerTalon(
                                config.followerConfigs[i].id,
                                motor,
                                config.followerConfigs[i].opposeLeader);
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

        SpectrumRobot.add(this);
        this.register();
    }

    /**
     * Constructs the mechanism and immediately overrides the attachment state.
     *
     * <p>Prefer setting {@link Config#setAttached(boolean)} before construction; this constructor
     * exists for cases where attachment must be decided after the config is built.
     *
     * @param config the mechanism configuration
     * @param attached {@code true} to initialize hardware, {@code false} to run detached
     */
    protected Mechanism(Config config, boolean attached) {
        this(config);
        config.attached = attached;
    }

    // =========================================================================
    // SpectrumSubsystem Overrides
    // =========================================================================

    /** Called every scheduler loop. Override in subclasses to log telemetry. */
    @Override
    public void periodic() {}

    /** Called every scheduler loop during simulation. Override to update sim models. */
    @Override
    public void simulationPeriodic() {}

    /**
     * @return the mechanism name as defined in {@link Config}.
     */
    @Override
    public String getName() {
        return config.getName();
    }

    /**
     * Returns whether this mechanism's hardware is connected and should be commanded.
     *
     * @return {@code true} if the motor(s) are attached and safe to command
     */
    public boolean isAttached() {
        return config.isAttached();
    }

    // =========================================================================
    // Sensor Readings
    // =========================================================================

    // ---- Stator Current ----

    /**
     * Reads the leader motor's stator current directly from hardware. Called by the {@link
     * CachedDouble} backing {@link #getStatorCurrent()}.
     *
     * @return stator current in amps, or {@code 0} if not attached
     */
    public double updateStatorCurrent() {
        if (config.attached) {
            return motor.getStatorCurrent().getValueAsDouble();
        }
        return 0;
    }

    /**
     * Returns the leader motor's stator current, cached for the current loop cycle.
     *
     * @return stator current in amps
     */
    public double getStatorCurrent() {
        return cachedStatorCurrent.getAsDouble();
    }

    // ---- Supply Current ----

    /**
     * Reads the leader motor's supply (battery) current directly from hardware. Called by the
     * {@link CachedDouble} backing {@link #getSupplyCurrent()}.
     *
     * @return supply current in amps, or {@code 0} if not attached
     */
    public double updateSupplyCurrent() {
        if (config.attached) {
            return motor.getSupplyCurrent().getValueAsDouble();
        }
        return 0;
    }

    /**
     * Returns the leader motor's supply current, cached for the current loop cycle.
     *
     * @return supply current in amps
     */
    public double getSupplyCurrent() {
        return cachedSupplyCurrent.getAsDouble();
    }

    // ---- Voltage ----

    /**
     * Reads the leader motor's output voltage directly from hardware. Called by the {@link
     * CachedDouble} backing {@link #getVoltage()}.
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
     * Returns the leader motor's output voltage, cached for the current loop cycle.
     *
     * @return motor voltage in volts
     */
    public double getVoltage() {
        return cachedVoltage.getAsDouble();
    }

    // ---- Temperature ----

    /**
     * Reads the leader motor's device temperature directly from hardware. Called by the {@link
     * CachedDouble} backing {@link #getTemp()}.
     *
     * @return temperature in °C, or {@code 0} if not attached
     */
    public double updateTemp() {
        if (config.attached) {
            return motor.getDeviceTemp().getValueAsDouble();
        }
        return 0;
    }

    /**
     * Returns the leader motor's device temperature, cached for the current loop cycle.
     *
     * @return temperature in °C
     */
    public double getTemp() {
        return cachedTemp.getAsDouble();
    }

    // ---- Position ----

    /**
     * Reads the mechanism position in rotations directly from hardware. The value is scaled by
     * {@link Config#configGearRatio(double)} (SensorToMechanismRatio). Called by the {@link
     * CachedDouble} backing {@link #getPositionRotations()}.
     *
     * @return position in rotations, or {@code 0} if not attached
     */
    private double updatePositionRotations() {
        if (config.attached) {
            return motor.getPosition().getValueAsDouble();
        }
        return 0;
    }

    /**
     * Returns the mechanism position in rotations, cached for the current loop cycle.
     *
     * @return position in rotations
     */
    public double getPositionRotations() {
        return cachedRotations.getAsDouble();
    }

    /**
     * Derives the mechanism position as a percentage of {@link Config#getMaxRotations()}. Called by
     * the {@link CachedDouble} backing {@link #getPositionPercentage()}.
     *
     * @return position as a percentage of max rotations (0–100)
     */
    private double updatePositionPercentage() {
        return rotationsToPercent(this::getPositionRotations);
    }

    /**
     * Returns the mechanism position as a percentage of max travel, cached for the current loop
     * cycle.
     *
     * @return position in percent (0 = min, 100 = max)
     */
    public double getPositionPercentage() {
        return cachedPercentage.getAsDouble();
    }

    /**
     * Derives the mechanism position in degrees from the cached rotation value. Called by the
     * {@link CachedDouble} backing {@link #getPositionDegrees()}.
     *
     * @return position in degrees
     */
    private double updatePositionDegrees() {
        return rotationsToDegrees(this::getPositionRotations);
    }

    /**
     * Returns the mechanism position in degrees, cached for the current loop cycle.
     *
     * @return position in degrees
     */
    public double getPositionDegrees() {
        return cachedDegrees.getAsDouble();
    }

    // ---- Velocity ----

    /**
     * Reads the mechanism velocity in rotations-per-second (CTRE native unit) directly from
     * hardware.
     *
     * @return velocity in RPS, or {@code 0} if not attached
     */
    private double updateVelocityRPS() {
        if (config.attached) {
            return motor.getVelocity().getValueAsDouble();
        }
        return 0;
    }

    /**
     * Converts the raw RPS hardware reading to RPM. Called by the {@link CachedDouble} backing
     * {@link #getVelocityRPM()}.
     *
     * @return velocity in RPM
     */
    private double updateVelocityRPM() {
        return Conversions.RPStoRPM(updateVelocityRPS());
    }

    /**
     * Returns the mechanism velocity in RPM, cached for the current loop cycle.
     *
     * @return velocity in RPM
     */
    public double getVelocityRPM() {
        return cachedVelocity.getAsDouble();
    }

    // ---- Unit Conversions ----

    /**
     * Converts a percentage of max travel to rotations.
     *
     * @param percent percentage (0–100), where 100% equals {@link Config#getMaxRotations()}
     * @return equivalent position in rotations
     */
    public double percentToRotations(DoubleSupplier percent) {
        return (percent.getAsDouble() / 100) * config.maxRotations;
    }

    /**
     * Converts rotations to a percentage of max travel.
     *
     * @param rotations position in rotations
     * @return percentage of {@link Config#getMaxRotations()} (0–100)
     */
    public double rotationsToPercent(DoubleSupplier rotations) {
        return (rotations.getAsDouble() / config.maxRotations) * 100;
    }

    /**
     * Converts degrees to rotations (mechanism-side, after gearing).
     *
     * @param degrees angle in degrees
     * @return equivalent position in rotations
     */
    public double degreesToRotations(DoubleSupplier degrees) {
        return (degrees.getAsDouble() / 360);
    }

    /**
     * Converts rotations to degrees (mechanism-side, after gearing).
     *
     * @param rotations position in rotations
     * @return equivalent angle in degrees
     */
    public double rotationsToDegrees(DoubleSupplier rotations) {
        return 360 * rotations.getAsDouble();
    }

    // =========================================================================
    // Triggers
    // =========================================================================

    // ---- Command / Default State ----

    /**
     * Returns a {@link Trigger} that is active whenever the mechanism is running its default
     * command (i.e., no other command has required it).
     *
     * @return trigger active while the default command is running
     */
    public Trigger runningDefaultCommand() {
        return new Trigger(this::isRunningDefaultCommand);
    }

    private boolean isRunningDefaultCommand() {
        return this.getCurrentCommand() == this.getDefaultCommand();
    }

    /**
     * Returns the last closed-loop setpoint commanded to the motor. Units depend on the most recent
     * control mode: rotations for position modes, RPS for velocity modes.
     *
     * @return last commanded setpoint
     */
    public double getTarget() {
        return target;
    }

    /**
     * Returns a {@link Trigger} that is active when the mechanism position is within {@code
     * tolerance} rotations of the last commanded {@link #getTarget() target}.
     *
     * @param tolerance acceptable error in rotations
     * @return trigger active when position is at the last commanded target
     */
    public Trigger atTargetPosition(DoubleSupplier tolerance) {
        return new Trigger(() -> isAtTargetPosition(tolerance));
    }

    private boolean isAtTargetPosition(DoubleSupplier tolerance) {
        return Math.abs(cachedRotations.getAsDouble() - target) < tolerance.getAsDouble();
    }

    // ---- Position Triggers (Rotations) ----

    /**
     * Returns a {@link Trigger} active when the position is within {@code tolerance} rotations of
     * {@code target}.
     *
     * @param target desired position in rotations
     * @param tolerance acceptable error in rotations
     * @return trigger active when position ≈ target
     */
    public Trigger atRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionRotations() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    /**
     * Returns a {@link Trigger} active when the position is below {@code target + tolerance}
     * rotations.
     *
     * @param target reference position in rotations
     * @param tolerance added to target to form the upper bound
     * @return trigger active when position &lt; target + tolerance
     */
    public Trigger belowRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionRotations() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} active when the position is above {@code target - tolerance}
     * rotations.
     *
     * @param target reference position in rotations
     * @param tolerance subtracted from target to form the lower bound
     * @return trigger active when position &gt; target - tolerance
     */
    public Trigger aboveRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionRotations() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    // ---- Position Triggers (Percent) ----

    /**
     * Returns a {@link Trigger} active when the position is within {@code tolerance} percent of
     * {@code target}.
     *
     * @param target desired position as a percentage of max travel (0–100)
     * @param tolerance acceptable error in percent
     * @return trigger active when position ≈ target
     */
    public Trigger atPercentage(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionPercentage() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    /**
     * Returns a {@link Trigger} active when the position is below {@code target + tolerance}
     * percent.
     *
     * @param target reference position in percent
     * @param tolerance added to target to form the upper bound
     * @return trigger active when position &lt; target + tolerance
     */
    public Trigger belowPercentage(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionPercentage() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} active when the position is above {@code target - tolerance}
     * percent.
     *
     * @param target reference position in percent
     * @param tolerance subtracted from target to form the lower bound
     * @return trigger active when position &gt; target - tolerance
     */
    public Trigger abovePercentage(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionPercentage() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    // ---- Position Triggers (Degrees) ----

    /**
     * Returns a {@link Trigger} active when the position is within {@code tolerance} degrees of
     * {@code target}.
     *
     * @param target desired position in degrees
     * @param tolerance acceptable error in degrees
     * @return trigger active when position ≈ target
     */
    public Trigger atDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionDegrees() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    /**
     * Returns a {@link Trigger} active when the position is below {@code target + tolerance}
     * degrees.
     *
     * @param target reference position in degrees
     * @param tolerance added to target to form the upper bound
     * @return trigger active when position &lt; target + tolerance
     */
    public Trigger belowDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionDegrees() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} active when the position is above {@code target - tolerance}
     * degrees.
     *
     * @param target reference position in degrees
     * @param tolerance subtracted from target to form the lower bound
     * @return trigger active when position &gt; target - tolerance
     */
    public Trigger aboveDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionDegrees() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    // ---- Velocity Triggers ----

    /**
     * Returns a {@link Trigger} active when velocity is within {@code tolerance} RPM of {@code
     * target}.
     *
     * @param target desired velocity in RPM
     * @param tolerance acceptable error in RPM
     * @return trigger active when velocity ≈ target
     */
    public Trigger atVelocityRPM(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> Math.abs(getVelocityRPM() - target.getAsDouble()) < tolerance.getAsDouble());
    }

    /**
     * Returns a {@link Trigger} active when velocity is below {@code target + tolerance} RPM.
     *
     * @param target reference velocity in RPM
     * @param tolerance added to target to form the upper bound
     * @return trigger active when velocity &lt; target + tolerance
     */
    public Trigger belowVelocityRPM(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getVelocityRPM() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} active when velocity is above {@code target - tolerance} RPM.
     *
     * @param target reference velocity in RPM
     * @param tolerance subtracted from target to form the lower bound
     * @return trigger active when velocity &gt; target - tolerance
     */
    public Trigger aboveVelocityRPM(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getVelocityRPM() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    // ---- Current Triggers ----

    /**
     * Returns a {@link Trigger} active when stator current is within {@code tolerance} amps of
     * {@code target}.
     *
     * @param target desired stator current in amps
     * @param tolerance acceptable error in amps
     * @return trigger active when current ≈ target
     */
    public Trigger atCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getStatorCurrent() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    /**
     * Returns a {@link Trigger} active when stator current is below {@code target + tolerance}
     * amps.
     *
     * @param target reference current in amps
     * @param tolerance added to target to form the upper bound
     * @return trigger active when current &lt; target + tolerance
     */
    public Trigger belowCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getStatorCurrent() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    /**
     * Returns a {@link Trigger} active when stator current is above {@code target - tolerance}
     * amps.
     *
     * @param target reference current in amps
     * @param tolerance subtracted from target to form the lower bound
     * @return trigger active when current &gt; target - tolerance
     */
    public Trigger aboveCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getStatorCurrent() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    // =========================================================================
    // Command Factories — Open-Loop
    // =========================================================================

    /**
     * Runs the mechanism at a fixed percentage of supply voltage, scaled by {@link
     * Config#getVoltageCompSaturation()}.
     *
     * @param percent output fraction between -1 (full reverse) and +1 (full forward)
     * @return a command that applies the percentage output while scheduled
     */
    public Command runPercentage(DoubleSupplier percent) {
        return run(() -> setPercentOutput(percent)).withName(getName() + ".runPercentage");
    }

    /**
     * Runs the mechanism at a fixed voltage, bypassing closed-loop control.
     *
     * @param voltage output in volts
     * @return a command that applies the voltage while scheduled
     */
    public Command runVoltage(DoubleSupplier voltage) {
        return run(() -> setVoltageOutput(voltage)).withName(getName() + ".runVoltage");
    }

    /**
     * Runs the mechanism at a fixed voltage, bypassing both closed-loop control and software
     * limits. Use with caution — this can drive the mechanism past its configured travel limits.
     *
     * @param voltage output in volts
     * @return a command that applies the voltage while scheduled, ignoring soft limits
     */
    public Command runVoltageNoSoftLimit(DoubleSupplier voltage) {
        return run(() -> setVoltageOutputNoSoftLimit(voltage))
                .withName(getName() + ".runVoltageNoSoftLimit");
    }

    /**
     * Runs the mechanism in open-loop Torque Current FOC mode (requires Phoenix Pro).
     *
     * @param current target torque current in amps
     * @return a command that applies the torque current while scheduled
     */
    public Command runTorqueCurrentFoc(DoubleSupplier current) {
        return run(() -> setTorqueCurrentFoc(current)).withName(getName() + ".runTorqueCurrentFoc");
    }

    // =========================================================================
    // Command Factories — Closed-Loop Velocity
    // =========================================================================

    /**
     * Runs the mechanism at a closed-loop velocity using voltage-compensated control.
     *
     * @param velocityRPM target velocity in RPM
     * @return a command that maintains the velocity while scheduled
     */
    public Command runVelocity(DoubleSupplier velocityRPM) {
        return run(() -> setVelocity(() -> Conversions.RPMtoRPS(velocityRPM)))
                .withName(getName() + ".runVelocity");
    }

    /**
     * Runs the mechanism at a closed-loop velocity using Torque Current FOC control (requires
     * Phoenix Pro). Provides better torque linearity than voltage-mode velocity control.
     *
     * @param velocityRPM target velocity in RPM
     * @return a command that maintains the velocity while scheduled
     */
    public Command runVelocityTcFocRPM(DoubleSupplier velocityRPM) {
        return run(() -> setVelocityTorqueCurrentFOC(() -> Conversions.RPMtoRPS(velocityRPM)))
                .withName(getName() + ".runVelocityFOCrpm");
    }

    // =========================================================================
    // Command Factories — Closed-Loop Position (Motion Magic)
    // =========================================================================

    /**
     * Moves the mechanism to a position in rotations using Motion Magic Torque Current FOC
     * (requires Phoenix Pro).
     *
     * @param rotations target position in rotations
     * @return a command that moves to and holds the position while scheduled
     */
    public Command moveToRotations(DoubleSupplier rotations) {
        return run(() -> setMMPositionFoc(rotations)).withName(getName() + ".moveToRotations");
    }

    /**
     * Moves the mechanism to a position expressed as a percentage of max travel using Motion Magic
     * Torque Current FOC (requires Phoenix Pro).
     *
     * @param percent target position as a percentage of {@link Config#getMaxRotations()} (0–100)
     * @return a command that moves to and holds the position while scheduled
     */
    public Command moveToPercentage(DoubleSupplier percent) {
        return run(() -> setMMPositionFoc(() -> percentToRotations(percent)))
                .withName(getName() + ".moveToPercentage");
    }

    /**
     * Moves the mechanism to a position in degrees using Motion Magic Torque Current FOC (requires
     * Phoenix Pro).
     *
     * @param degrees target position in degrees
     * @return a command that moves to and holds the position while scheduled
     */
    public Command moveToDegrees(DoubleSupplier degrees) {
        return run(() -> setMMPositionFoc(() -> degreesToRotations(degrees)))
                .withName(getName() + ".moveToDegrees");
    }

    /**
     * Moves the mechanism to a position using Motion Magic voltage control (slot 1 gains). Use when
     * FOC is unavailable or when a separate voltage-mode gain set is needed.
     *
     * @param rotations target position in rotations
     * @return a command that moves to and holds the position while scheduled
     */
    public Command runFocRotations(DoubleSupplier rotations) {
        return run(() -> setMMPosition(rotations)).withName(getName() + ".runFOCPosition");
    }

    // =========================================================================
    // Command Factories — Utility
    // =========================================================================

    /**
     * Stops the mechanism by calling {@link TalonFX#stopMotor()}.
     *
     * @return a command that stops the mechanism while scheduled
     */
    public Command runStop() {
        return run(this::stop).withName(getName() + ".runStop");
    }

    /**
     * Temporarily sets the mechanism to coast mode for the duration of the command, then restores
     * brake mode when the command ends. Safe to run while disabled.
     *
     * @return a command that coasts the mechanism while scheduled
     */
    public Command coastMode() {
        return startEnd(() -> setBrakeMode(false), () -> setBrakeMode(true))
                .ignoringDisable(true)
                .withName(getName() + ".coastMode");
    }

    /**
     * Ensures the mechanism is in brake mode. If it is already in brake mode this command does
     * nothing. Safe to run while disabled — useful at match end to lock mechanisms in place.
     *
     * @return a command that switches to brake mode if currently coasting
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
     * Returns a one-shot command that applies new supply and stator current limits immediately.
     *
     * @param supplyLimit new supply current limit in amps (positive magnitude)
     * @param statorLimit new stator current limit in amps (positive magnitude)
     * @return an instant command that updates the limits
     */
    protected Command runCurrentLimits(DoubleSupplier supplyLimit, DoubleSupplier statorLimit) {
        return Commands.runOnce(() -> setCurrentLimits(supplyLimit, statorLimit));
    }

    // =========================================================================
    // Command Factories — Diagnostics
    // =========================================================================

    /**
     * Monitors stator current while scheduled and raises a {@link Alert} when the command ends if
     * the time-averaged current deviates from {@code expectedCurrent} by more than {@code
     * tolerance} amps.
     *
     * @param expectedCurrent expected average stator current in amps
     * @param tolerance acceptable deviation in amps before an alert is raised
     * @return a command that monitors current and alerts on mismatch
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
     * Monitors stator current while scheduled and raises a {@link Alert} when the command ends if
     * the peak current exceeded {@code expectedCurrent} amps. Useful for detecting mechanical jams
     * or binding.
     *
     * @param expectedCurrent maximum expected stator current in amps
     * @return a command that monitors current and alerts if peak is exceeded
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
     * Monitors stator current while scheduled and raises a {@link Alert} when the command ends if
     * the peak current never reached {@code expectedCurrent} amps. Useful for detecting a mechanism
     * that failed to move (broken belt, disconnected motor, etc.).
     *
     * @param expectedCurrent minimum expected peak stator current in amps
     * @return a command that monitors current and alerts if the threshold is not reached
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

    // =========================================================================
    // Low-Level Motor Control (protected setters)
    //
    // These are intended to be called from within Command lambdas in subclasses.
    // Each method is a no-op when isAttached() returns false.
    // =========================================================================

    // ---- State Management ----

    /**
     * Commands the motor to neutral output (stops the motor). Does nothing if the mechanism is not
     * attached.
     */
    protected void stop() {
        if (isAttached()) {
            motor.stopMotor();
        }
    }

    /**
     * Sets the motor's internal position sensor to zero. Used to establish a home reference after a
     * hard-stop zeroing routine.
     */
    protected void tareMotor() {
        if (isAttached()) {
            setMotorPosition(() -> 0);
        }
    }

    /**
     * Seeds the motor's internal position sensor to an arbitrary value without moving the
     * mechanism. Useful for restoring a known position after robot enable.
     *
     * @param rotations the position to write to the motor's sensor, in rotations
     */
    protected void setMotorPosition(DoubleSupplier rotations) {
        if (isAttached()) {
            motor.setPosition(rotations.getAsDouble());
        }
    }

    /**
     * Delegates to {@link #applyCurrentLimit(DoubleSupplier, DoubleSupplier)} to push updated
     * supply and stator limits to hardware.
     *
     * @param supplyLimit new supply current limit in amps (positive magnitude)
     * @param statorLimit new stator current limit in amps (positive magnitude)
     */
    protected void setCurrentLimits(DoubleSupplier supplyLimit, DoubleSupplier statorLimit) {
        applyCurrentLimit(supplyLimit, statorLimit);
    }

    // ---- Open-Loop Setters ----

    /**
     * Applies a percentage output scaled by the voltage compensation saturation voltage. Equivalent
     * to a voltage-mode output clamped to {@code voltageCompSaturation × percent}.
     *
     * @param percent output fraction between -1 and +1
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
     * Applies a direct voltage output, bypassing closed-loop control.
     *
     * @param voltage output in volts
     */
    public void setVoltageOutput(DoubleSupplier voltage) {
        if (isAttached()) {
            VoltageOut output = config.voltageControl.withOutput(voltage.getAsDouble());
            motor.setControl(output);
        }
    }

    /**
     * Applies a direct voltage output, bypassing both closed-loop control and software limits. Use
     * with caution — the mechanism can be driven past its configured travel range.
     *
     * @param voltage output in volts
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
     * Applies an open-loop Torque Current FOC output (requires Phoenix Pro).
     *
     * @param current target torque current in amps (positive = forward)
     */
    public void setTorqueCurrentFoc(DoubleSupplier current) {
        if (isAttached()) {
            TorqueCurrentFOC output = config.torqueCurrentFOC.withOutput(current.getAsDouble());
            motor.setControl(output);
        }
    }

    // ---- Closed-Loop Velocity Setters ----

    /**
     * Runs the mechanism at a closed-loop velocity using voltage-compensated control.
     *
     * @param velocityRPS target velocity in rotations-per-second
     */
    protected void setVelocity(DoubleSupplier velocityRPS) {
        if (isAttached()) {
            target = velocityRPS.getAsDouble();
            VelocityVoltage output = config.velocityControl.withVelocity(target);
            motor.setControl(output);
        }
    }

    /**
     * Runs the mechanism at a closed-loop velocity using Torque Current FOC (requires Phoenix Pro).
     * Accepts RPS directly.
     *
     * @param velocityRPS target velocity in rotations-per-second
     */
    protected void setVelocityTorqueCurrentFOC(DoubleSupplier velocityRPS) {
        if (isAttached()) {
            target = velocityRPS.getAsDouble();
            VelocityTorqueCurrentFOC output = config.velocityTorqueCurrentFOC.withVelocity(target);
            motor.setControl(output);
        }
    }

    /**
     * Runs the mechanism at a closed-loop velocity using Torque Current FOC (requires Phoenix Pro).
     * Accepts RPM and converts internally to RPS.
     *
     * @param velocityRPM target velocity in RPM
     */
    protected void setVelocityTCFOCrpm(DoubleSupplier velocityRPM) {
        if (isAttached()) {
            target = Conversions.RPMtoRPS(velocityRPM.getAsDouble());
            VelocityTorqueCurrentFOC output = config.velocityTorqueCurrentFOC.withVelocity(target);
            motor.setControl(output);
        }
    }

    /**
     * Runs the mechanism at a closed-loop velocity using Motion Magic Velocity Torque Current FOC
     * (requires Phoenix Pro). Motion Magic profiles the velocity ramp using the configured
     * acceleration limit.
     *
     * @param velocityRPS target velocity in rotations-per-second
     */
    protected void setMMVelocityFOC(DoubleSupplier velocityRPS) {
        if (isAttached()) {
            target = velocityRPS.getAsDouble();
            MotionMagicVelocityTorqueCurrentFOC mm = config.mmVelocityFOC.withVelocity(target);
            motor.setControl(mm);
        }
    }

    // ---- Closed-Loop Position Setters ----

    /**
     * Moves the mechanism to a position using Motion Magic Torque Current FOC (requires Phoenix
     * Pro). The motion profile is governed by the cruise velocity, acceleration, and jerk set in
     * {@link Config#configMotionMagic(double, double, double)}.
     *
     * @param rotations target position in rotations
     */
    protected void setMMPositionFoc(DoubleSupplier rotations) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            MotionMagicTorqueCurrentFOC mm = config.mmPositionFOC.withPosition(target);
            motor.setControl(mm);
        }
    }

    /**
     * Moves the mechanism to a position using Dynamic Motion Magic Torque Current FOC (requires
     * Phoenix Pro). Velocity, acceleration, and jerk limits can be changed each loop cycle,
     * allowing adaptive profiling (e.g., slower motion near limits).
     *
     * @param rotations target position in rotations
     * @param velocity maximum profile velocity in rotations-per-second
     * @param acceleration maximum profile acceleration in rotations-per-second²
     * @param jerk maximum profile jerk in rotations-per-second³
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
     * Moves the mechanism to a position using Dynamic Motion Magic voltage control. Velocity,
     * acceleration, and jerk limits can be changed each loop cycle.
     *
     * @param rotations target position in rotations
     * @param velocity maximum profile velocity in rotations-per-second
     * @param acceleration maximum profile acceleration in rotations-per-second²
     * @param jerk maximum profile jerk in rotations-per-second³
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
     * Moves the mechanism to a position using Motion Magic voltage control with slot 0 gains.
     * Delegates to {@link #setMMPosition(DoubleSupplier, int)} with slot 0.
     *
     * @param rotations target position in rotations
     */
    protected void setMMPosition(DoubleSupplier rotations) {
        setMMPosition(rotations, 0);
    }

    /**
     * Moves the mechanism to a position using Motion Magic voltage control with the specified gain
     * slot. Use slot 1 or 2 for alternative gain sets (e.g., a slower profile near soft limits).
     *
     * @param rotations target position in rotations
     * @param slot gain slot index (0, 1, or 2)
     */
    public void setMMPosition(DoubleSupplier rotations, int slot) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            MotionMagicVoltage mm =
                    config.mmPositionVoltageSlot.withSlot(slot).withPosition(target);
            motor.setControl(mm);
        }
    }

    // =========================================================================
    // Configuration & Limits (runtime)
    // =========================================================================

    /**
     * Sets the neutral mode (brake or coast) and immediately pushes the updated configuration to
     * hardware.
     *
     * @param isInBrake {@code true} for brake mode, {@code false} for coast mode
     */
    public void setBrakeMode(boolean isInBrake) {
        if (isAttached()) {
            config.configNeutralBrakeMode(isInBrake);
            config.applyTalonConfig(motor);
        }
    }

    /**
     * Enables or disables the reverse software limit switch without changing its threshold. The
     * threshold is read from the current {@link TalonFXConfiguration}.
     *
     * @param enabled {@code true} to enable the reverse soft limit, {@code false} to disable it
     */
    public void toggleReverseSoftLimit(boolean enabled) {
        if (isAttached()) {
            double threshold = config.talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold;
            config.configReverseSoftLimit(threshold, enabled);
            config.applyTalonConfig(motor);
        }
    }

    /**
     * Overrides the torque current limits at runtime. When {@code enabled} is {@code true}, both
     * forward and reverse torque current limits (and the stator limit) are set to {@code
     * enabledLimit}. When {@code false}, the limits are removed by setting them to 300 A
     * (effectively unlimited for typical FRC use).
     *
     * @param enabledLimit peak torque current in amps (positive magnitude) when limiting is active
     * @param enabled {@code true} to apply the limit, {@code false} to remove it
     */
    public void toggleTorqueCurrentLimit(DoubleSupplier enabledLimit, boolean enabled) {
        if (isAttached()) {
            if (enabled) {
                config.configForwardTorqueCurrentLimit(enabledLimit.getAsDouble());
                config.configReverseTorqueCurrentLimit(enabledLimit.getAsDouble());
                config.configStatorCurrentLimit(enabledLimit.getAsDouble(), true);
                config.applyTalonConfig(motor);
            } else {
                config.configForwardTorqueCurrentLimit(300);
                config.configReverseTorqueCurrentLimit(300);
                config.applyTalonConfig(motor);
            }
        }
    }

    /**
     * Enables or disables the supply current limit at runtime without changing the limit value.
     *
     * @param enabledLimit supply current limit in amps (used as both enabled and disabled value;
     *     only the enable flag changes)
     * @param enabled {@code true} to enable the supply limit, {@code false} to disable it
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
     * Pushes updated supply and stator current limits to hardware only when at least one limit has
     * changed from the currently configured value. Retries up to 10 times on CAN failure.
     *
     * <p>All limit values are treated as positive magnitudes; the reverse torque limit is
     * automatically negated by {@link Config#configReverseTorqueCurrentLimit(double)}.
     *
     * @param supplyLimit new supply current limit in amps (positive magnitude)
     * @param statorLimit new stator current limit in amps (positive magnitude); also applied as the
     *     forward and reverse torque current limit
     */
    public void applyCurrentLimit(DoubleSupplier supplyLimit, DoubleSupplier statorLimit) {
        if (isAttached()) {
            if (config.talonConfig.CurrentLimits.StatorCurrentLimit != statorLimit.getAsDouble()
                    || config.talonConfig.CurrentLimits.SupplyCurrentLimit
                            != supplyLimit.getAsDouble()) {
                config.configSupplyCurrentLimit(Math.abs(supplyLimit.getAsDouble()), true);
                config.configStatorCurrentLimit(Math.abs(statorLimit.getAsDouble()), true);
                config.configForwardTorqueCurrentLimit(Math.abs(statorLimit.getAsDouble()));
                config.configReverseTorqueCurrentLimit(Math.abs(statorLimit.getAsDouble()));
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

    // =========================================================================
    // Telemetry & Diagnostics
    // =========================================================================

    /**
     * Reports the combined supply current of the leader and all follower motors to the {@link
     * frc.spectrumLib.BatteryLogger} under the key {@code "Mechanisms/<name>"}. Call this from
     * {@link #periodic()} in each subclass to include the mechanism in power budgeting.
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
     * Returns the name of the command currently requiring this subsystem, or {@code "none"} if the
     * default command (or no command) is running. Useful for telemetry logs.
     *
     * @return current command name, or {@code "none"}
     */
    protected String getCurrentCommandName() {
        Command currentCommand = this.getCurrentCommand();
        if (currentCommand != null) {
            return currentCommand.getName();
        }
        return "none";
    }

    // =========================================================================
    // Inner Classes
    // =========================================================================

    /**
     * Configuration for a single follower TalonFX that mirrors the leader motor.
     *
     * <p>Followers are configured as permanent followers via {@link
     * TalonFXFactory#createPermanentFollowerTalon} and cannot be commanded independently.
     */
    public static class FollowerConfig {
        /** Human-readable name used for logging. */
        @Getter private String name;

        /** CAN bus device ID of the follower motor. */
        @Getter private CanDeviceId id;

        /** Whether this follower motor is physically installed on the robot. */
        @Getter private boolean attached = true;

        /**
         * Motor alignment relative to the leader. Use {@link MotorAlignmentValue#Opposed} when the
         * follower is mechanically inverted (e.g., motors on opposite sides of a shooter).
         */
        @Getter private MotorAlignmentValue opposeLeader = MotorAlignmentValue.Aligned;

        /**
         * @param name human-readable name for logging
         * @param id CAN device ID
         * @param canbus CAN bus name (e.g., {@link frc.spectrumLib.Rio#CANIVORE})
         * @param opposeLeader {@link MotorAlignmentValue#Opposed} if the follower runs in the
         *     opposite direction from the leader, {@link MotorAlignmentValue#Aligned} otherwise
         */
        public FollowerConfig(
                String name, int id, String canbus, MotorAlignmentValue opposeLeader) {
            this.name = name;
            this.id = new CanDeviceId(id, canbus);
            this.opposeLeader = opposeLeader;
        }
    }

    /**
     * Base configuration class for a {@link Mechanism}.
     *
     * <p>Subclass this inside each concrete mechanism class to add mechanism-specific tuning
     * values, then call the {@code config*()} helpers in the subclass constructor to build the full
     * {@link TalonFXConfiguration} before the motor is constructed.
     *
     * <h2>Sign conventions</h2>
     *
     * <ul>
     *   <li>Always pass <b>positive magnitudes</b> to {@link #configReverseTorqueCurrentLimit}. The
     *       method automatically negates the value before writing it to the hardware register.
     *   <li>All other current/voltage limit helpers accept and store positive values.
     * </ul>
     *
     * <h2>Gain slots</h2>
     *
     * TalonFX supports three independent gain slots (0, 1, 2). Slot 0 is the default for all
     * helpers that do not accept a slot parameter. Use slots 1 and 2 for alternative profiles
     * (e.g., a slower position profile near soft limits).
     */
    public static class Config {

        // ---- Identity ----

        /** Human-readable mechanism name, used in telemetry keys and alerts. */
        @Getter private String name;

        /**
         * Whether the physical hardware for this mechanism is installed on the robot. When {@code
         * false}, motors are not constructed and all commands are no-ops.
         */
        @Getter @Setter private boolean attached = true;

        /** CAN bus device ID of the leader TalonFX. */
        @Getter private CanDeviceId id;

        // ---- TalonFX Configuration ----

        /** Full Phoenix 6 configuration applied to the leader motor at construction time. */
        @Getter @Setter protected TalonFXConfiguration talonConfig;

        /** Total number of motors (leader + followers). Informational only. */
        @Getter private int numMotors = 1;

        /**
         * Voltage compensation saturation used by {@link #setPercentOutput(DoubleSupplier)}.
         * Defaults to 12 V.
         */
        @Getter private double voltageCompSaturation = 12.0;

        /**
         * Minimum mechanism position in rotations. Used by {@link #percentToRotations} and enforced
         * as a reverse soft limit when {@link #configReverseSoftLimit} is called.
         */
        @Getter private double minRotations = 0;

        /**
         * Maximum mechanism position in rotations. Used by {@link #percentToRotations} and enforced
         * as a forward soft limit when {@link #configForwardSoftLimit} is called.
         */
        @Getter private double maxRotations = 1;

        /** Follower motor configurations. Empty by default (no followers). */
        @Getter private FollowerConfig[] followerConfigs = new FollowerConfig[0];

        // ---- Pre-built Control Request Objects ----
        // CTRE control requests are allocated once and mutated with withXxx() to avoid GC pressure.

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

        /** Motion Magic voltage request pre-configured for slot 1. */
        @Getter
        private MotionMagicVoltage mmPositionVoltageSlot = new MotionMagicVoltage(0).withSlot(1);

        @Getter private VoltageOut voltageControl = new VoltageOut(0);
        @Getter private VelocityVoltage velocityControl = new VelocityVoltage(0);

        @Getter
        private VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

        @Getter private TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0);

        /** Duty-cycle output request. Prefer {@link #voltageControl} for reproducible behavior. */
        @Getter private DutyCycleOut percentOutput = new DutyCycleOut(0);

        // ---- Constructor ----

        /**
         * Creates a base mechanism configuration.
         *
         * <p>Hardware limit switches are disabled by default. Call the appropriate {@code
         * config*()} helpers in subclass constructors to configure PID gains, gear ratio, current
         * limits, soft limits, etc.
         *
         * @param name human-readable name (used in telemetry and alerts)
         * @param id CAN device ID of the leader motor
         * @param canbus CAN bus name (e.g., {@link frc.spectrumLib.Rio#CANIVORE})
         */
        public Config(String name, int id, String canbus) {
            this.name = name;
            this.id = new CanDeviceId(id, canbus);
            talonConfig = new TalonFXConfiguration();
            talonConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
            talonConfig.HardwareLimitSwitch.ReverseLimitEnable = false;
        }

        // ---- Hardware Application ----

        /**
         * Applies the current {@link TalonFXConfiguration} to the given motor controller. Logs a DS
         * warning if the CAN write fails.
         *
         * @param talon the motor controller to configure
         */
        public void applyTalonConfig(TalonFX talon) {
            StatusCode result = talon.getConfigurator().apply(talonConfig);
            if (!result.isOK()) {
                DriverStation.reportWarning(
                        "Could not apply config changes to " + name + "\'s motor ", false);
            }
        }

        // ---- Follower Configuration ----

        /**
         * Registers one or more follower motors for this mechanism.
         *
         * @param followers varargs of {@link FollowerConfig} describing each follower
         */
        public void setFollowerConfigs(FollowerConfig... followers) {
            followerConfigs = followers;
        }

        // ---- Motor Output ----

        /**
         * Sets the voltage compensation saturation voltage used by percent-output control.
         *
         * @param voltageCompSaturation saturation voltage in volts (typically 12 V)
         */
        public void configVoltageCompensation(double voltageCompSaturation) {
            this.voltageCompSaturation = voltageCompSaturation;
        }

        /**
         * Sets the positive direction to counter-clockwise (when viewed from the front of the motor
         * shaft).
         */
        public void configCounterClockwise_Positive() {
            talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        /**
         * Sets the positive direction to clockwise (when viewed from the front of the motor shaft).
         * This is the default for most mechanisms mounted on the right side.
         */
        public void configClockwise_Positive() {
            talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }

        /**
         * Sets the duty-cycle neutral deadband. Outputs within this band are treated as zero.
         *
         * @param deadband deadband fraction (0–1); 0.04 is a typical value
         */
        public void configNeutralDeadband(double deadband) {
            talonConfig.MotorOutput.DutyCycleNeutralDeadband = deadband;
        }

        /**
         * Clamps the duty-cycle output to the specified forward and reverse peaks.
         *
         * @param forward maximum forward output fraction (0–1)
         * @param reverse maximum reverse output fraction (-1–0)
         */
        public void configPeakOutput(double forward, double reverse) {
            talonConfig.MotorOutput.PeakForwardDutyCycle = forward;
            talonConfig.MotorOutput.PeakReverseDutyCycle = reverse;
        }

        /**
         * Sets the neutral (idle) mode of the motor.
         *
         * @param isInBrake {@code true} for brake mode, {@code false} for coast mode
         */
        public void configNeutralBrakeMode(boolean isInBrake) {
            talonConfig.MotorOutput.NeutralMode =
                    isInBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        }

        // ---- Voltage Limits ----

        /**
         * Sets the peak forward output voltage. Limits the motor output even in FOC modes.
         *
         * @param voltageLimit peak forward voltage in volts
         */
        public void configForwardVoltageLimit(double voltageLimit) {
            talonConfig.Voltage.PeakForwardVoltage = voltageLimit;
        }

        /**
         * Sets the peak reverse output voltage. Limits the motor output even in FOC modes.
         *
         * @param voltageLimit peak reverse voltage in volts (use a positive value; direction is
         *     implied)
         */
        public void configReverseVoltageLimit(double voltageLimit) {
            talonConfig.Voltage.PeakReverseVoltage = voltageLimit;
        }

        // ---- Current Limits ----

        /**
         * Configures the supply (battery-side) current limit.
         *
         * @param supplyLimit supply current limit in amps (positive magnitude; auto-corrected if
         *     negative is passed)
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
         * Configures the stator (motor-side) current limit.
         *
         * @param statorLimit stator current limit in amps (positive magnitude; auto-corrected if
         *     negative is passed)
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
         * Sets the peak forward torque current limit used in FOC modes.
         *
         * @param currentLimit peak forward torque current in amps (positive magnitude;
         *     auto-corrected if negative is passed)
         */
        public void configForwardTorqueCurrentLimit(double currentLimit) {
            if (currentLimit < 0) {
                currentLimit = -currentLimit;
            }
            talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = currentLimit;
        }

        /**
         * Sets the peak reverse torque current limit used in FOC modes.
         *
         * <p><b>Sign convention:</b> always pass a <em>positive</em> magnitude. This method
         * automatically negates the value before writing it to the hardware register ({@code
         * PeakReverseTorqueCurrent} must be ≤ 0 for Phoenix 6).
         *
         * @param currentLimit peak reverse torque current magnitude in amps (positive)
         */
        public void configReverseTorqueCurrentLimit(double currentLimit) {
            if (currentLimit > 0) {
                currentLimit = -currentLimit;
            }
            talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = currentLimit;
        }

        /**
         * Sets the lower supply current limit, which is applied after {@link
         * #configLowerSupplyCurrentTime(double)} seconds at the higher limit. Useful for
         * stall-prevention: allow a brief high-current spike on startup, then throttle back.
         *
         * @param currentLimit lower supply current limit in amps
         */
        public void configLowerSupplyCurrentLimit(double currentLimit) {
            talonConfig.CurrentLimits.SupplyCurrentLowerLimit = currentLimit;
        }

        /**
         * Sets the time after which the lower supply current limit takes effect.
         *
         * @param time duration in seconds before the lower limit is applied
         * @see #configLowerSupplyCurrentLimit(double)
         */
        public void configLowerSupplyCurrentTime(double time) {
            talonConfig.CurrentLimits.SupplyCurrentLowerTime = time;
        }

        // ---- Soft Limits ----

        /**
         * Configures the forward software limit switch (maximum travel boundary).
         *
         * @param threshold position threshold in rotations; motor output is cut when exceeded
         * @param enabled {@code true} to enforce the limit
         */
        public void configForwardSoftLimit(double threshold, boolean enabled) {
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = threshold;
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = enabled;
        }

        /**
         * Configures the reverse software limit switch (minimum travel boundary).
         *
         * @param threshold position threshold in rotations; motor output is cut when exceeded
         * @param enabled {@code true} to enforce the limit
         */
        public void configReverseSoftLimit(double threshold, boolean enabled) {
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = threshold;
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = enabled;
        }

        /**
         * Stores the minimum and maximum mechanism positions used by percentage-of-travel
         * conversions and the {@code atPercentage} / {@code moveToPercentage} helpers.
         *
         * @param minRotation minimum position in rotations (home / reverse limit)
         * @param maxRotation maximum position in rotations (full extension / forward limit)
         */
        protected void configMinMaxRotations(double minRotation, double maxRotation) {
            this.minRotations = minRotation;
            this.maxRotations = maxRotation;
        }

        // ---- Closed-Loop Gains ----

        /**
         * Sets PID feedback gains for slot 0.
         *
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         */
        public void configPIDGains(double kP, double kI, double kD) {
            configPIDGains(0, kP, kI, kD);
        }

        /**
         * Sets PID feedback gains for the specified slot (0–2).
         *
         * @param slot gain slot index
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         */
        public void configPIDGains(int slot, double kP, double kI, double kD) {
            talonConfigFeedbackPID(slot, kP, kI, kD);
        }

        /**
         * Sets feedforward gains for slot 0.
         *
         * @param kS static friction compensation (volts or amps)
         * @param kV velocity feedforward (output per RPS)
         * @param kA acceleration feedforward (output per RPS²)
         * @param kG gravity feedforward (output; non-zero only for arm/elevator mechanisms)
         */
        public void configFeedForwardGains(double kS, double kV, double kA, double kG) {
            configFeedForwardGains(0, kS, kV, kA, kG);
        }

        /**
         * Sets feedforward gains for the specified slot (0–2).
         *
         * @param slot gain slot index
         * @param kS static friction compensation
         * @param kV velocity feedforward
         * @param kA acceleration feedforward
         * @param kG gravity feedforward
         */
        public void configFeedForwardGains(int slot, double kS, double kV, double kA, double kG) {
            talonConfigFeedForward(slot, kV, kA, kS, kG);
        }

        /**
         * Configures the gravity compensation type for slot 0.
         *
         * @param isArm {@code true} for {@link GravityTypeValue#Arm_Cosine} (rotary arm), {@code
         *     false} for {@link GravityTypeValue#Elevator_Static} (linear elevator)
         */
        public void configGravityType(boolean isArm) {
            configGravityType(0, isArm);
        }

        /**
         * Configures the gravity compensation type for the specified slot.
         *
         * @param slot gain slot index (0–2)
         * @param isArm {@code true} for arm cosine, {@code false} for elevator static
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

        // ---- Motion Magic ----

        /**
         * Sets the Motion Magic trapezoidal profile parameters applied to all position and velocity
         * Motion Magic control requests.
         *
         * @param cruiseVelocity maximum profile velocity in rotations-per-second
         * @param acceleration maximum profile acceleration in rotations-per-second²
         * @param jerk maximum profile jerk in rotations-per-second³ (0 = unlimited)
         */
        public void configMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
            talonConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
            talonConfig.MotionMagic.MotionMagicAcceleration = acceleration;
            talonConfig.MotionMagic.MotionMagicJerk = jerk;
        }

        /**
         * Overrides the acceleration and feedforward on the pre-built velocity Motion Magic control
         * request objects. Call after {@link #configMotionMagic} if velocity-specific acceleration
         * or feedforward tuning is needed.
         *
         * @param acceleration profile acceleration override in rotations-per-second²
         * @param feedforward additional feedforward output added to the velocity controller
         */
        public void configMotionMagicVelocity(double acceleration, double feedforward) {
            mmVelocityFOC =
                    mmVelocityFOC.withAcceleration(acceleration).withFeedForward(feedforward);
            mmVelocityVoltage =
                    mmVelocityVoltage.withAcceleration(acceleration).withFeedForward(feedforward);
        }

        /**
         * Overrides the feedforward on the pre-built position Motion Magic control request objects.
         * Useful when a constant output is needed to overcome a static load (e.g., a pneumatic
         * spring).
         *
         * @param feedforward additional feedforward output added to the position controller
         */
        public void configMotionMagicPosition(double feedforward) {
            mmPositionFOC = mmPositionFOC.withFeedForward(feedforward);
            mmPositionVoltage = mmPositionVoltage.withFeedForward(feedforward);
        }

        // ---- Feedback / Gearing ----

        /**
         * Sets the sensor-to-mechanism gear ratio, i.e., how many rotor rotations correspond to one
         * mechanism rotation. If a remote sensor is used, this is the ratio of sensor rotations to
         * mechanism rotations.
         *
         * <p>Example: a 10:1 gearbox where the sensor is on the rotor → {@code gearRatio = 10}.
         *
         * @param gearRatio rotor-to-mechanism (or sensor-to-mechanism) ratio
         */
        public void configGearRatio(double gearRatio) {
            talonConfig.Feedback.SensorToMechanismRatio = gearRatio;
        }

        /**
         * Returns the currently configured sensor-to-mechanism gear ratio.
         *
         * @return gear ratio (sensor rotations per mechanism rotation)
         */
        public double getGearRatio() {
            return talonConfig.Feedback.SensorToMechanismRatio;
        }

        /**
         * Sets the feedback sensor source and optional rotor offset. Use when a CANcoder or other
         * remote sensor is the primary feedback device.
         *
         * @param source feedback sensor source
         */
        public void configFeedbackSensorSource(FeedbackSensorSourceValue source) {
            configFeedbackSensorSource(source, 0);
        }

        /**
         * Sets the feedback sensor source and rotor offset. The offset is applied as {@code
         * FeedbackRotorOffset} in the TalonFX configuration.
         *
         * @param source feedback sensor source
         * @param offset rotor offset in rotations
         */
        public void configFeedbackSensorSource(FeedbackSensorSourceValue source, double offset) {
            talonConfig.Feedback.FeedbackSensorSource = source;
            talonConfig.Feedback.FeedbackRotorOffset = offset;
        }

        /**
         * Enables or disables continuous wrap, allowing the position sensor to wrap at ±0.5
         * rotations. Useful for mechanisms that rotate continuously (e.g., a swerve azimuth).
         *
         * @param enabled {@code true} to enable continuous wrap
         */
        public void configContinuousWrap(boolean enabled) {
            talonConfig.ClosedLoopGeneral.ContinuousWrap = enabled;
        }

        // ---- Private Helpers ----

        /**
         * Writes feedforward gains to the specified TalonFX gain slot. Note that the parameter
         * order differs from the public API to match the Phoenix 6 field naming convention.
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

        /** Writes PID feedback gains to the specified TalonFX gain slot. */
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

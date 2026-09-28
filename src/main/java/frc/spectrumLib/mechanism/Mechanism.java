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
 * Base class for a TalonFX-driven mechanism. Owns the leader motor and any followers, caches sensor
 * readings, and provides unit conversions, control helpers, and trigger factories for the commands
 * callers build.
 *
 * <p>Units are CTRE Phoenix 6 native: positions in rotations, velocities in rotations per second,
 * unless the name says otherwise. Gearing comes from {@code
 * Config.talonConfig.Feedback.SensorToMechanismRatio}.
 *
 * <p>With {@link Config#isAttached()} false, no hardware is built and every sensor reads 0, so the
 * same mechanism class works with or without a Talon.
 *
 * <p>The target fields hold the last setpoint sent to the motor, which is what {@link
 * #atTargetPosition(DoubleSupplier)} compares against. Subclasses pass a {@link Config} and
 * override {@link #periodic()}.
 */
public abstract class Mechanism implements Subsystem {

    /** Leader motor; the follower motors mirror its output. */
    @Getter protected TalonFX motor;

    /** Motors that mirror the leader. */
    @Getter protected TalonFX[] followerMotors;

    public Config config;

    /** Shows the result of the current diagnostic commands. */
    Alert currentAlert = new Alert("", AlertType.kWarning);

    /** Last closed-loop position setpoint sent to the motor, in rotations. */
    private double target = 0;

    /** Last closed-loop velocity setpoint sent to the motor, in rotations per second. */
    private double velocityTarget = 0;

    // Each cache refreshes at most once per scheduler loop.
    private final CachedDouble cachedRotations;
    private final CachedDouble cachedPercentage;
    private final CachedDouble cachedVoltage;
    private final CachedDouble cachedDegrees;
    private final CachedDouble cachedVelocity;
    private final CachedDouble cachedStatorCurrent;
    private final CachedDouble cachedSupplyCurrent;
    private final CachedDouble cachedTemp;

    /**
     * Creates the leader TalonFX and any follower motors when {@link Config#isAttached()} is {@code
     * true}. The sensor caches are always built, so the getters return 0 when unattached.
     */
    protected Mechanism(Config config) {
        this.config = config;

        if (isAttached()) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
            // optimizeBusUtilization then raises these rates to whatever the bus can sustain.
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

    /** Creates a Mechanism with the config's attached flag overridden. */
    protected Mechanism(Config config, boolean attached) {
        // this(...) has to be the first statement, so the override rides in on the config.
        this(applyAttachedOverride(config, attached));
    }

    private static Config applyAttachedOverride(Config config, boolean attached) {
        config.attached = attached;
        return config;
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

    /** Name from the {@link Config}. This class uses it as the prefix for its command names. */
    @Override
    public String getName() {
        return config.getName();
    }

    public boolean isAttached() {
        return config.isAttached();
    }

    /** Reports leader plus follower supply current to the battery logger. No-op when unattached. */
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

    /** Name of the scheduled command, or {@code "none"}. */
    protected String getCurrentCommandName() {
        Command currentCommand = this.getCurrentCommand();
        if (currentCommand != null) {
            return currentCommand.getName();
        }
        return "none";
    }

    public Trigger runningDefaultCommand() {
        return new Trigger(this::isRunningDefaultCommand);
    }

    private boolean isRunningDefaultCommand() {
        return this.getCurrentCommand() == this.getDefaultCommand();
    }

    /** Last closed-loop position setpoint sent to the motor, in rotations. */
    public double getTarget() {
        return target;
    }

    /** Last closed-loop velocity setpoint sent to the motor, in rotations per second. */
    public double getVelocityTargetRPS() {
        return velocityTarget;
    }

    /**
     * Active while the motor is within {@code tolerance} rotations of the last setpoint sent.
     *
     * @param tolerance maximum position error in rotations
     */
    public Trigger atTargetPosition(DoubleSupplier tolerance) {
        return new Trigger(() -> isAtTargetPosition(tolerance));
    }

    /**
     * True while the motor is within {@code tolerance} rotations of the last setpoint sent.
     *
     * @param tolerance maximum position error in rotations
     */
    public boolean isAtTargetPosition(DoubleSupplier tolerance) {
        return Math.abs(cachedRotations.getAsDouble() - target) < tolerance.getAsDouble();
    }

    public Trigger atRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionRotations() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    public boolean isAtRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return Math.abs(getPositionRotations() - target.getAsDouble()) < tolerance.getAsDouble();
    }

    public Trigger belowRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionRotations() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    public Trigger aboveRotations(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionRotations() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    /**
     * Active while the position is within {@code tolerance} of {@code target}.
     *
     * @param target position as a percentage of {@link Config#getMaxRotations()}
     * @param tolerance maximum deviation in percentage points
     */
    public Trigger atPercentage(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionPercentage() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    /**
     * Active while the position is still under {@code target + tolerance}.
     *
     * @param target position as a percentage of {@link Config#getMaxRotations()}
     */
    public Trigger belowPercentage(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionPercentage() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    /**
     * Active while the position is still over {@code target - tolerance}.
     *
     * @param target position as a percentage of {@link Config#getMaxRotations()}
     */
    public Trigger abovePercentage(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionPercentage() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    public Trigger atDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionDegrees() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    public Trigger belowDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionDegrees() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    public Trigger aboveDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getPositionDegrees() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    public Trigger atVelocityRPM(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> Math.abs(getVelocityRPM() - target.getAsDouble()) < tolerance.getAsDouble());
    }

    public Trigger belowVelocityRPM(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getVelocityRPM() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    public Trigger aboveVelocityRPM(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getVelocityRPM() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    /**
     * Active while the stator current is within {@code tolerance} of {@code target}.
     *
     * @param target stator current in amps
     * @param tolerance maximum deviation in amps
     */
    public Trigger atCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getStatorCurrent() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    /**
     * Active while the stator current is still under {@code target + tolerance}.
     *
     * @param target stator current in amps
     */
    public Trigger belowCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getStatorCurrent() < (target.getAsDouble() + tolerance.getAsDouble()));
    }

    /**
     * Active while the stator current is still over {@code target - tolerance}.
     *
     * @param target stator current in amps
     */
    public Trigger aboveCurrent(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () -> getStatorCurrent() > (target.getAsDouble() - tolerance.getAsDouble()));
    }

    public double updateStatorCurrent() {
        if (config.attached) {
            return motor.getStatorCurrent().getValueAsDouble();
        }
        return 0;
    }

    /** Cached stator current in amps. */
    public double getStatorCurrent() {
        return cachedStatorCurrent.getAsDouble();
    }

    public double updateSupplyCurrent() {
        if (config.attached) {
            return motor.getSupplyCurrent().getValueAsDouble();
        }
        return 0;
    }

    /** Cached supply current in amps. */
    public double getSupplyCurrent() {
        return cachedSupplyCurrent.getAsDouble();
    }

    public double updateVoltage() {
        if (config.attached) {
            return motor.getMotorVoltage().getValueAsDouble();
        }
        return 0;
    }

    /** Cached motor voltage in volts. */
    public double getVoltage() {
        return cachedVoltage.getAsDouble();
    }

    public double updateTemp() {
        if (config.attached) {
            return motor.getDeviceTemp().getValueAsDouble();
        }
        return 0;
    }

    /** Cached motor temperature in degrees Celsius. */
    public double getTemp() {
        return cachedTemp.getAsDouble();
    }

    /**
     * Turns a share of the configured range into absolute rotations.
     *
     * @param percent position as a percentage of {@link Config#getMaxRotations()}
     */
    public double percentToRotations(DoubleSupplier percent) {
        return (percent.getAsDouble() / 100) * config.maxRotations;
    }

    public double rotationsToPercent(DoubleSupplier rotations) {
        return (rotations.getAsDouble() / config.maxRotations) * 100;
    }

    public double degreesToRotations(DoubleSupplier degrees) {
        return (degrees.getAsDouble() / 360);
    }

    public double rotationsToDegrees(DoubleSupplier rotations) {
        return 360 * rotations.getAsDouble();
    }

    public double getPositionRotations() {
        return cachedRotations.getAsDouble();
    }

    private double updatePositionRotations() {
        if (config.attached) {
            return motor.getPosition().getValueAsDouble();
        }
        return 0;
    }

    public double getPositionPercentage() {
        return cachedPercentage.getAsDouble();
    }

    private double updatePositionPercentage() {
        return rotationsToPercent(this::getPositionRotations);
    }

    public double getPositionDegrees() {
        return cachedDegrees.getAsDouble();
    }

    private double updatePositionDegrees() {
        return rotationsToDegrees(this::getPositionRotations);
    }

    private double updateVelocityRPS() {
        if (config.attached) {
            return motor.getVelocity().getValueAsDouble();
        }
        return 0;
    }

    public double getVelocityRPM() {
        return cachedVelocity.getAsDouble();
    }

    private double updateVelocityRPM() {
        return Conversions.RPStoRPM(updateVelocityRPS());
    }

    /** Closed-loop velocity control with voltage compensation. */
    public Command runVelocity(DoubleSupplier velocityRPM) {
        return run(() -> setVelocity(() -> Conversions.RPMtoRPS(velocityRPM)))
                .withName(getName() + ".runVelocity");
    }

    /** Closed-loop velocity control with torque current FOC. Requires Phoenix Pro. */
    public Command runVelocityTcFocRPM(DoubleSupplier velocityRPM) {
        return run(() -> setVelocityTorqueCurrentFOC(() -> Conversions.RPMtoRPS(velocityRPM)))
                .withName(getName() + ".runVelocityTcFocRPM");
    }

    /**
     * Open-loop percent output, scaled to the configured voltage saturation.
     *
     * @param percent fractional output from -1 to 1
     */
    public Command runPercentage(DoubleSupplier percent) {
        return run(() -> setPercentOutput(percent)).withName(getName() + ".runPercentage");
    }

    /** Open-loop voltage output, bypassing closed-loop control. */
    public Command runVoltage(DoubleSupplier voltage) {
        return run(() -> setVoltageOutput(voltage)).withName(getName() + ".runVoltage");
    }

    /**
     * Open-loop voltage output that also ignores the software limit switches, so the mechanism can
     * travel past its configured limits.
     */
    public Command runVoltageNoSoftLimit(DoubleSupplier voltage) {
        return run(() -> setVoltageOutputNoSoftLimit(voltage))
                .withName(getName() + ".runVoltageNoSoftLimit");
    }

    /**
     * Applies a torque current setpoint. Requires Phoenix Pro.
     *
     * @param current torque current in amps
     */
    public Command runTorqueCurrentFoc(DoubleSupplier current) {
        return run(() -> setTorqueCurrentFoc(current)).withName(getName() + ".runTorqueCurrentFoc");
    }

    /** Motion Magic position control with torque current FOC. Requires Phoenix Pro. */
    public Command moveToRotations(DoubleSupplier rotations) {
        return run(() -> setMMPositionFoc(rotations)).withName(getName() + ".runPoseRevolutions");
    }

    /**
     * Motion Magic to a percentage of {@link Config#getMaxRotations()}, torque current FOC.
     * Requires Phoenix Pro.
     */
    public Command moveToPercentage(DoubleSupplier percent) {
        return run(() -> setMMPositionFoc(() -> percentToRotations(percent)))
                .withName(getName() + ".runPosePercentage");
    }

    /** Motion Magic to an angle in degrees, torque current FOC. Requires Phoenix Pro. */
    public Command moveToDegrees(DoubleSupplier degrees) {
        return run(() -> setMMPositionFoc(() -> degreesToRotations(degrees)))
                .withName(getName() + ".runPoseDegrees");
    }

    /** Same as {@link #moveToRotations(DoubleSupplier)}. */
    public Command runFocRotations(DoubleSupplier rotations) {
        return run(() -> setMMPositionFoc(rotations)).withName(getName() + ".runFOCPosition");
    }

    public Command runStop() {
        return run(this::stop).withName(getName() + ".runStop");
    }

    /** Coasts while running, then returns to brake. Safe while the robot is disabled. */
    public Command coastMode() {
        return startEnd(() -> setBrakeMode(false), () -> setBrakeMode(true))
                .ignoringDisable(true)
                .withName(getName() + ".coastMode");
    }

    /** Returns to brake mode if the mechanism is coasting. Safe while the robot is disabled. */
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
     * Applies new current limits.
     *
     * @param supplyLimit supply current limit in amps
     * @param statorLimit stator current limit in amps
     */
    protected Command runCurrentLimits(DoubleSupplier supplyLimit, DoubleSupplier statorLimit) {
        return Commands.runOnce(() -> setCurrentLimits(supplyLimit, statorLimit));
    }

    /**
     * Applies new current limits.
     *
     * @param supplyLimit supply current limit in amps
     * @param statorLimit stator current limit in amps
     */
    protected void setCurrentLimits(DoubleSupplier supplyLimit, DoubleSupplier statorLimit) {
        applyCurrentLimit(supplyLimit, statorLimit);
    }

    protected void stop() {
        if (isAttached()) {
            motor.stopMotor();
        }
    }

    /** Sets the motor's reported position to zero. */
    protected void tareMotor() {
        if (isAttached()) {
            setMotorPosition(() -> 0);
        }
    }

    /**
     * Writes the motor's internal position register without moving the motor.
     *
     * @param rotations the position to write, in rotations
     */
    protected void setMotorPosition(DoubleSupplier rotations) {
        if (isAttached()) {
            motor.setPosition(rotations.getAsDouble());
        }
    }

    /**
     * Motion Magic velocity control with torque current FOC. Requires Phoenix Pro.
     *
     * @param velocityRPS target velocity in rotations per second
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
     * Closed-loop velocity control with torque current FOC. Requires Phoenix Pro.
     *
     * @param velocityRPS target velocity in rotations per second
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
     * Torque current FOC velocity control. Requires Phoenix Pro.
     *
     * @param velocityRPM target velocity in revolutions per minute
     */
    protected void setVelocityTCFOCrpm(DoubleSupplier velocityRPM) {
        if (isAttached()) {
            velocityTarget = Conversions.RPMtoRPS(velocityRPM.getAsDouble());
            VelocityTorqueCurrentFOC output =
                    config.velocityTorqueCurrentFOC.withVelocity(velocityTarget);
            motor.setControl(output);
        }
    }

    protected void setVelocity(DoubleSupplier velocityRPS) {
        if (isAttached()) {
            velocityTarget = velocityRPS.getAsDouble();
            VelocityVoltage output = config.velocityControl.withVelocity(velocityTarget);
            motor.setControl(output);
        }
    }

    /**
     * Motion Magic position control with torque current FOC. Requires Phoenix Pro.
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
     * Dynamic Motion Magic position control with torque current FOC. Requires Phoenix Pro.
     *
     * @param rotations target position in rotations
     * @param velocity cruise velocity in rotations per second
     * @param acceleration in rotations per second squared
     * @param jerk in rotations per second cubed
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
     * Dynamic Motion Magic position control with voltage compensation.
     *
     * @param rotations target position in rotations
     * @param velocity cruise velocity in rotations per second
     * @param acceleration in rotations per second squared
     * @param jerk in rotations per second cubed
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

    protected void setMMPosition(DoubleSupplier rotations) {
        setMMPosition(rotations, 0);
    }

    protected void setMMPosition(DoubleSupplier rotations, int slot) {
        if (isAttached()) {
            target = rotations.getAsDouble();
            MotionMagicVoltage mm =
                    config.mmPositionVoltageSlot.withSlot(slot).withPosition(target);
            motor.setControl(mm);
        }
    }

    /**
     * Open-loop percent output. The voltage sent is {@code percent} times the configured voltage
     * saturation.
     *
     * @param percent fractional output from -1 to 1
     */
    protected void setPercentOutput(DoubleSupplier percent) {
        if (isAttached()) {
            VoltageOut output =
                    config.voltageControl.withOutput(
                            config.voltageCompSaturation * percent.getAsDouble());
            motor.setControl(output);
        }
    }

    /**
     * Open-loop voltage with no compensation scaling.
     *
     * @param voltage the voltage to send, in volts
     */
    protected void setVoltageOutput(DoubleSupplier voltage) {
        if (isAttached()) {
            VoltageOut output = config.voltageControl.withOutput(voltage.getAsDouble());
            motor.setControl(output);
        }
    }

    /**
     * Open-loop voltage that ignores the software limit switches, so the mechanism can travel past
     * its configured limits.
     *
     * @param voltage the voltage to send, in volts
     */
    protected void setVoltageOutputNoSoftLimit(DoubleSupplier voltage) {
        if (isAttached()) {
            VoltageOut output =
                    config.voltageControl
                            .withOutput(voltage.getAsDouble())
                            .withIgnoreSoftwareLimits(true);
            motor.setControl(output);
        }
    }

    /**
     * Applies a torque current setpoint. Requires Phoenix Pro.
     *
     * @param current torque current in amps
     */
    public void setTorqueCurrentFoc(DoubleSupplier current) {
        if (isAttached()) {
            TorqueCurrentFOC output = config.torqueCurrentFOC.withOutput(current.getAsDouble());
            motor.setControl(output);
        }
    }

    /**
     * Switches the motor between brake and coast and applies it right away.
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
     * Enables or disables the reverse software limit, keeping the threshold already in the
     * configuration.
     */
    public void toggleReverseSoftLimit(boolean enabled) {
        if (isAttached()) {
            double threshold = config.talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold;
            config.configReverseSoftLimit(threshold, enabled);
            config.applyTalonConfig(motor);
        }
    }

    /**
     * Applies a forward and reverse torque current limit. Disabling it resets the limit to 300 A.
     *
     * @param enabledLimit torque current limit in amps, used only when the limit is enabled
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
     * Enables or disables the supply current limit. Disabling leaves the limit value in place, so a
     * later enable resumes at the same amps.
     *
     * @param enabledLimit supply current limit in amps
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
     * Applies supply and stator current limits, skipping the apply when both already match. The
     * torque current limits follow the stator limit. A failed apply is retried up to 10 times.
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

    /**
     * Alerts when the average stator current over the run is off by more than {@code tolerance}.
     *
     * @param expectedCurrent the expected average stator current in amps
     * @param tolerance the maximum acceptable deviation in amps
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
     * Alerts when the peak stator current over the run exceeds {@code expectedCurrent}.
     *
     * @param expectedCurrent the maximum acceptable peak stator current in amps
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
     * Alerts when the peak stator current over the run never reaches {@code expectedCurrent}.
     *
     * @param expectedCurrent the minimum acceptable peak stator current in amps
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

    /**
     * One motor that mirrors the leader. Set {@code opposeLeader} to {@link
     * MotorAlignmentValue#Opposed} when the motor is mounted backwards and has to spin the other
     * way to produce the same motion.
     */
    public static class FollowerConfig {

        @Getter private String name;

        @Getter private CanDeviceId id;

        @Getter private boolean attached = true;

        @Getter private MotorAlignmentValue opposeLeader = MotorAlignmentValue.Aligned;

        /**
         * Describes one follower motor.
         *
         * @param canbus CAN bus name, for example {@code "rio"} or {@code "canivore"}
         */
        public FollowerConfig(
                String name, int id, String canbus, MotorAlignmentValue opposeLeader) {
            this.name = name;
            this.id = new CanDeviceId(id, canbus);
            this.opposeLeader = opposeLeader;
        }
    }

    /**
     * Everything a {@link Mechanism} needs: the TalonFX hardware configuration, the control request
     * objects, and the mechanism-level values such as gearing, soft limits, gains, and the Motion
     * Magic profile.
     *
     * <p>Subclass it and call the {@code config*} helpers before passing the config to {@link
     * Mechanism}.
     */
    public static class Config {

        @Getter private String name;

        @Getter @Setter private boolean attached = true;

        @Getter private CanDeviceId id;

        /** Applied to the leader motor on construction. */
        @Getter @Setter protected TalonFXConfiguration talonConfig;

        @Getter private int numMotors = 1;

        /** Voltage ceiling for percent output, 12 V by default. */
        @Getter private double voltageCompSaturation = 12.0;

        @Getter private double minRotations = 0;

        /** Full range for percent output, in rotations. */
        @Getter private double maxRotations = 1;

        /** Empty when the mechanism has no followers. */
        @Getter private FollowerConfig[] followerConfigs = new FollowerConfig[0];

        // Control requests are built once and reused, so the control loop does not allocate.

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

        /** Duty-cycle output control. Prefer {@link #voltageControl}. */
        @Getter private DutyCycleOut percentOutput = new DutyCycleOut(0);

        /**
         * Base config with the default Talon settings and both limit switches off.
         *
         * @param canbus CAN bus name, for example {@code "rio"} or {@code "canivore"}
         */
        public Config(String name, int id, String canbus) {
            this.name = name;
            this.id = new CanDeviceId(id, canbus);
            talonConfig = new TalonFXConfiguration();

            talonConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
            talonConfig.HardwareLimitSwitch.ReverseLimitEnable = false;
        }

        public void applyTalonConfig(TalonFX talon) {
            StatusCode result = talon.getConfigurator().apply(talonConfig);
            if (!result.isOK()) {
                DriverStation.reportWarning(
                        "Could not apply config changes to " + name + "\'s motor ", false);
            }
        }

        public void setFollowerConfigs(FollowerConfig... followers) {
            followerConfigs = followers;
        }

        /**
         * Sets the voltage ceiling that percent output scales against.
         *
         * @param voltageCompSaturation the ceiling in volts, 12.0 on most robots
         */
        public void configVoltageCompensation(double voltageCompSaturation) {
            this.voltageCompSaturation = voltageCompSaturation;
        }

        /** Counter-clockwise positive, the usual sign convention when viewed from the shaft end. */
        public void configCounterClockwise_Positive() {
            talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        public void configClockwise_Positive() {
            talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }

        /**
         * Sets the peak forward output voltage.
         *
         * @param voltageLimit the ceiling in volts
         */
        public void configForwardVoltageLimit(double voltageLimit) {
            talonConfig.Voltage.PeakForwardVoltage = voltageLimit;
        }

        /**
         * Sets the peak reverse output voltage.
         *
         * @param voltageLimit the ceiling in volts; pass a negative value for reverse
         */
        public void configReverseVoltageLimit(double voltageLimit) {
            talonConfig.Voltage.PeakReverseVoltage = voltageLimit;
        }

        /**
         * Takes the absolute value, so a negative input still sets a forward limit.
         *
         * @param supplyLimit the limit in amps
         */
        public void configSupplyCurrentLimit(double supplyLimit, boolean enabled) {
            if (supplyLimit < 0) {
                supplyLimit = -supplyLimit;
            }
            talonConfig.CurrentLimits.SupplyCurrentLimit = supplyLimit;
            talonConfig.CurrentLimits.SupplyCurrentLimitEnable = enabled;
        }

        /**
         * Takes the absolute value, so a negative input still sets a forward limit.
         *
         * @param statorLimit the limit in amps
         */
        public void configStatorCurrentLimit(double statorLimit, boolean enabled) {
            if (statorLimit < 0) {
                statorLimit = -statorLimit;
            }
            talonConfig.CurrentLimits.StatorCurrentLimit = statorLimit;
            talonConfig.CurrentLimits.StatorCurrentLimitEnable = enabled;
        }

        /**
         * Takes the absolute value, so a negative input still sets a forward limit.
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
         * Forces the value negative, so a positive input still sets a reverse limit.
         *
         * @param currentLimit peak reverse torque current in amps
         */
        public void configReverseTorqueCurrentLimit(double currentLimit) {
            if (currentLimit > 0) {
                currentLimit = -currentLimit;
            }
            talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = currentLimit;
        }

        /**
         * Trims the supply current once the upper limit has been exceeded, which cuts dissipation.
         *
         * @param currentLimit the lower limit in amps
         */
        public void configLowerSupplyCurrentLimit(double currentLimit) {
            talonConfig.CurrentLimits.SupplyCurrentLowerLimit = currentLimit;
        }

        /**
         * Sets how long the upper limit may be exceeded first.
         *
         * @param time seconds above the limit before the lower limit engages
         */
        public void configLowerSupplyCurrentTime(double time) {
            talonConfig.CurrentLimits.SupplyCurrentLowerTime = time;
        }

        /**
         * Sets the output band that is treated as zero.
         *
         * @param deadband fraction of full output treated as zero, for example 0.001
         */
        public void configNeutralDeadband(double deadband) {
            talonConfig.MotorOutput.DutyCycleNeutralDeadband = deadband;
        }

        /**
         * Caps the duty cycle in each direction.
         *
         * @param forward 0 to 1
         * @param reverse -1 to 0
         */
        public void configPeakOutput(double forward, double reverse) {
            talonConfig.MotorOutput.PeakForwardDutyCycle = forward;
            talonConfig.MotorOutput.PeakReverseDutyCycle = reverse;
        }

        /**
         * Stops the mechanism at {@code threshold} going forward.
         *
         * @param threshold the position in rotations
         */
        public void configForwardSoftLimit(double threshold, boolean enabled) {
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = threshold;
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = enabled;
        }

        /**
         * Stops the mechanism at {@code threshold} going backward.
         *
         * @param threshold the position in rotations
         */
        public void configReverseSoftLimit(double threshold, boolean enabled) {
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = threshold;
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = enabled;
        }

        /**
         * Wraps the closed-loop target, for mechanisms that spin continuously such as swerve
         * azimuth.
         */
        public void configContinuousWrap(boolean enabled) {
            talonConfig.ClosedLoopGeneral.ContinuousWrap = enabled;
        }

        /**
         * Sets acceleration and feed-forward for velocity control.
         *
         * @param acceleration velocity acceleration in rotations per second squared
         */
        public void configMotionMagicVelocity(double acceleration, double feedforward) {
            mmVelocityFOC =
                    mmVelocityFOC.withAcceleration(acceleration).withFeedForward(feedforward);
            mmVelocityVoltage =
                    mmVelocityVoltage.withAcceleration(acceleration).withFeedForward(feedforward);
        }

        public void configMotionMagicPosition(double feedforward) {
            mmPositionFOC = mmPositionFOC.withFeedForward(feedforward);
            mmPositionVoltage = mmPositionVoltage.withFeedForward(feedforward);
        }

        /**
         * Caps cruise velocity, acceleration, and jerk.
         *
         * @param cruiseVelocity in rotations per second
         * @param acceleration in rotations per second squared
         * @param jerk in rotations per second cubed
         */
        public void configMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
            talonConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
            talonConfig.MotionMagic.MotionMagicAcceleration = acceleration;
            talonConfig.MotionMagic.MotionMagicJerk = jerk;
        }

        /**
         * Sets the mechanism gearing, in sensor turns per output turn. A remote sensor counts as
         * the sensor rather than the rotor.
         */
        public void configGearRatio(double gearRatio) {
            talonConfig.Feedback.SensorToMechanismRatio = gearRatio;
        }

        public double getGearRatio() {
            return talonConfig.Feedback.SensorToMechanismRatio;
        }

        public void configNeutralBrakeMode(boolean isInBrake) {
            if (isInBrake) {
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            } else {
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            }
        }

        public void configPIDGains(double kP, double kI, double kD) {
            configPIDGains(0, kP, kI, kD);
        }

        /**
         * Writes kP, kI, and kD into one slot.
         *
         * @param slot 0, 1, or 2; any other value reports a warning and changes nothing
         */
        public void configPIDGains(int slot, double kP, double kI, double kD) {
            talonConfigFeedbackPID(slot, kP, kI, kD);
        }

        /**
         * Writes the feed-forward gains into slot 0.
         *
         * @param kS static friction compensation, in volts or amps depending on the control mode
         */
        public void configFeedForwardGains(double kS, double kV, double kA, double kG) {
            configFeedForwardGains(0, kS, kV, kA, kG);
        }

        /**
         * Writes the feed-forward gains into the given slot.
         *
         * @param slot 0, 1, or 2; any other value reports a warning and changes nothing
         * @param kS static friction compensation, in volts or amps depending on the control mode
         */
        public void configFeedForwardGains(int slot, double kS, double kV, double kA, double kG) {
            talonConfigFeedForward(slot, kV, kA, kS, kG);
        }

        public void configFeedbackSensorSource(FeedbackSensorSourceValue source) {
            configFeedbackSensorSource(source, 0);
        }

        /**
         * Points the feedback at a sensor, offset by a rotor position.
         *
         * @param offset the feedback rotor offset in rotations
         */
        public void configFeedbackSensorSource(FeedbackSensorSourceValue source, double offset) {
            talonConfig.Feedback.FeedbackSensorSource = source;
            talonConfig.Feedback.FeedbackRotorOffset = offset;
        }

        /**
         * Sets gravity compensation in slot 0.
         *
         * @param isArm {@code true} for {@link GravityTypeValue#Arm_Cosine}, {@code false} for
         *     {@link GravityTypeValue#Elevator_Static}
         */
        public void configGravityType(boolean isArm) {
            configGravityType(0, isArm);
        }

        /**
         * Sets gravity compensation in the given slot.
         *
         * @param slot 0, 1, or 2; any other value reports a warning and changes nothing
         * @param isArm {@code true} for {@link GravityTypeValue#Arm_Cosine}, {@code false} for
         *     {@link GravityTypeValue#Elevator_Static}
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
         * Sets the rotation bounds. Only {@code maxRotations} is read, as the full range for
         * percent output.
         */
        protected void configMinMaxRotations(double minRotation, double maxRotation) {
            this.minRotations = minRotation;
            this.maxRotations = maxRotation;
        }

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

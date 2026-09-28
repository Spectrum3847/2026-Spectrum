package frc.robot.subsystems.intakeExtension;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotSim;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.LinearConfig;
import frc.spectrumLib.sim.LinearSim;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

/**
 * Rack-and-pinion fuel intake deploy, driven by two independent axes: the left, which is this
 * class, and the right ({@link IntakeExtensionRight}). Each side runs its own closed-loop position
 * control rather than one following the other, so a side that skips a tooth can be driven on its
 * own to resync.
 *
 * <p>Resyncing drives each side into the fully extended hard stop and re-zeroes that side's encoder
 * at maxRotations. A skipped tooth otherwise leaves the motor encoder reading a position the rack
 * is no longer at.
 */
public class IntakeExtension extends Mechanism {

    public static class IntakeExtensionConfig extends Config {

        @Getter private final double initPosition = 0;
        @Getter private final double triggerTolerance = 5;

        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18;

        @Getter private final double maxRotations = 2.8;
        @Getter private final double minRotations = 0.0;

        @Getter private final double supplyCurrentLimit = 20;
        @Getter private final double statorCurrentLimit = 40;
        @Getter private final double lowerSupplyCurrentLimit = 20;
        @Getter private final double lowerSupplyCurrentTime = 0;

        @Getter private final double positionKp = 13;
        @Getter private final double positionKi = 0;
        @Getter private final double positionKd = 0;
        @Getter private final double positionKv = 1.0;
        @Getter private final double positionKs = 2.0;
        @Getter private final double positionKa = 0;
        @Getter private final double positionKg = 0;
        @Getter private final double gearRatio = 11.25;
        @Getter private final double mmCruiseVelocity = 100;
        @Getter private final double mmAcceleration = 300;
        @Getter private final double mmJerk = 1000;
        @Getter private final double slowMmCruiseVelocity = 4;
        @Getter private final double slowMmAcceleration = 20;
        @Getter private final double slowMmJerk = 1000;

        @Getter private final double sensorToMechanismRatio = 11.25;
        @Getter private final double rotorToSensorRatio = 1;
        @Getter private final double CANcoderRotorToSensorRatio = 1.7;
        @Getter private final double CANcoderSensorToMechanismRatio = 1;
        @Getter private final double CANcoderOffset = 0;
        @Getter private final boolean CANcoderAttached = false;

        // Resync settings, all driving toward the fully extended hard stop
        @Getter private final double homingVoltage = 6;
        @Getter private final double homingStallRPM = 50.0;
        @Getter private final double homingMinTimeSecs = 0.3;
        @Getter private final double homingStallDebounceSecs = 0.15;
        @Getter private final double homingTimeoutSecs = 3.0;

        @Getter private final double intakeX = Units.inchesToMeters(70);
        @Getter private final double intakeY = Units.inchesToMeters(23);
        @Getter private final double extensionMass = 10.0;
        @Getter private final double drumRadiusMeters = Units.inchesToMeters(0.955 / 2);
        @Getter private final double extensionGearing = 11.25;
        @Getter private final double angle = 180;
        @Getter private final double staticLength = 10;
        @Getter private final double movingLength = 55;
        @Getter private final double lineWidth = 20;
        @Getter private final double maxExtensionHeight = 40;

        public IntakeExtensionConfig() {
            super("IntakeExtension", 4, Rio.CANIVORE);
            configMinMaxRotations(minRotations, maxRotations);
            configPIDGains(0, positionKp, positionKi, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configSupplyCurrentLimit(supplyCurrentLimit, true);
            configStatorCurrentLimit(statorCurrentLimit, true);
            configLowerSupplyCurrentLimit(lowerSupplyCurrentLimit);
            configLowerSupplyCurrentTime(lowerSupplyCurrentTime);
            configGearRatio(gearRatio);
            configForwardTorqueCurrentLimit(statorCurrentLimit);
            configReverseTorqueCurrentLimit(statorCurrentLimit);
            configForwardSoftLimit(maxRotations, true);
            configReverseSoftLimit(minRotations, true);
            configNeutralBrakeMode(true);
            configClockwise_Positive();
        }

        public IntakeExtensionConfig applyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    /**
     * Right deploy axis, a standalone {@link Mechanism} rather than a follower so it can be driven
     * on its own during a resync. Gains, limits, and geometry come from the left config; only the
     * CAN id, name, and inversion differ, since the right gearbox is mounted mirrored. That keeps
     * both axes on "positive extends".
     */
    public static class IntakeExtensionRight extends Mechanism {

        public static class RightConfig extends Config {
            public RightConfig(IntakeExtensionConfig left) {
                super("IntakeExtensionRight", 5, Rio.CANIVORE);
                setAttached(left.isAttached());
                configMinMaxRotations(left.getMinRotations(), left.getMaxRotations());
                configPIDGains(0, left.getPositionKp(), left.getPositionKi(), left.getPositionKd());
                configFeedForwardGains(
                        left.getPositionKs(),
                        left.getPositionKv(),
                        left.getPositionKa(),
                        left.getPositionKg());
                configMotionMagic(
                        left.getMmCruiseVelocity(), left.getMmAcceleration(), left.getMmJerk());
                configSupplyCurrentLimit(left.getSupplyCurrentLimit(), true);
                configStatorCurrentLimit(left.getStatorCurrentLimit(), true);
                configLowerSupplyCurrentLimit(left.getLowerSupplyCurrentLimit());
                configLowerSupplyCurrentTime(left.getLowerSupplyCurrentTime());
                configGearRatio(left.getGearRatio());
                configForwardTorqueCurrentLimit(left.getStatorCurrentLimit());
                configReverseTorqueCurrentLimit(left.getStatorCurrentLimit());
                configForwardSoftLimit(left.getMaxRotations(), true);
                configReverseSoftLimit(left.getMinRotations(), true);
                configNeutralBrakeMode(true);
                configCounterClockwise_Positive();
            }
        }

        @Getter private final RightConfig rightConfig;

        public IntakeExtensionRight(IntakeExtensionConfig leftConfig) {
            super(new RightConfig(leftConfig));
            this.rightConfig = (RightConfig) super.config;
            Telemetry.print(getName() + " Subsystem Initialized");
        }

        /** Closed-loop Motion Magic to an absolute rotation target. */
        public void goToRotations(double rotations) {
            setMMPosition(() -> rotations);
        }

        /** Dynamic Motion Magic voltage move to a rotation target, using the slow profile. */
        public void goToRotationsSlow(
                double rotations, double cruiseVelocity, double acceleration, double jerk) {
            setDynMMPositionVoltage(
                    () -> rotations, () -> cruiseVelocity, () -> acceleration, () -> jerk);
        }

        /** Open-loop voltage that ignores soft limits, so homing can reach the hard stop. */
        public void driveHomingVoltage(double volts) {
            setVoltageOutputNoSoftLimit(() -> volts);
        }

        public void setInitialPosition(double rotations) {
            if (isAttached()) {
                motor.setPosition(rotations);
            }
        }

        public void zeroAtMax() {
            setMotorPosition(() -> rightConfig.getMaxRotations());
        }

        public void stopAxis() {
            stop();
        }

        @Override
        public void periodic() {
            logBatteryUsage();
            Telemetry.log("IntakeExtensionRight/CurrentCommand", getCurrentCommandName());
            Telemetry.log("IntakeExtensionRight/Voltage", getVoltage(), "volts");
            Telemetry.log("IntakeExtensionRight/StatorCurrent", getStatorCurrent(), "amps");
            Telemetry.log("IntakeExtensionRight/SupplyCurrent", getSupplyCurrent(), "amps");
            Telemetry.log("IntakeExtensionRight/Position", getPositionRotations(), "rotations");
            Telemetry.log("IntakeExtensionRight/RPM", getVelocityRPM(), "RPM");
            Telemetry.log("IntakeExtensionRight/Temp", getTemp(), "deg_C");
        }
    }

    @Getter private final IntakeExtensionConfig config;
    @Getter private IntakeExtensionSim sim;
    private final IntakeExtensionRight right;

    public IntakeExtension(IntakeExtensionConfig config) {
        super(config);
        this.config = config;
        this.right = new IntakeExtensionRight(config);

        setInitialPosition();

        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    private void setInitialPosition() {
        if (isAttached()) {
            double initialRotations = config.getInitPosition();
            motor.setPosition(initialRotations);
            right.setInitialPosition(initialRotations);
        }
    }

    public void resetCurrentPositionToMax() {
        if (isAttached()) {
            motor.setPosition(config.getMaxRotations());
        }
        if (right.isAttached()) right.zeroAtMax();
    }

    public Command resetCurrentPositionToMaxCommand() {
        return new InstantCommand(this::resetCurrentPositionToMax);
    }

    public Command resetToInitialPos() {
        return new InstantCommand(this::setInitialPosition);
    }

    /** Sets brake mode on both deploy axes. */
    @Override
    public void setBrakeMode(boolean isInBrake) {
        super.setBrakeMode(isInBrake);
        if (right.isAttached()) right.setBrakeMode(isInBrake);
    }

    public enum WantedState {
        STOPPED,
        FULL_EXTEND,
        CONDITIONAL_EXTEND,
        FULL_RETRACT,
        SLOW_CLOSE,
        RESYNC,
    }

    public enum SystemState {
        STOPPED,
        FULL_EXTEND,
        FULL_RETRACT,
        SLOW_CLOSE,
        HOMING,
    }

    private WantedState wantedState = WantedState.STOPPED;
    private SystemState systemState = SystemState.STOPPED;
    private SystemState previousSystemState = SystemState.STOPPED;
    private boolean sentOutByIntakeState = false;

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case STOPPED -> SystemState.STOPPED;
            case FULL_EXTEND -> {
                sentOutByIntakeState = true;
                yield SystemState.FULL_EXTEND;
            }
            case CONDITIONAL_EXTEND -> sentOutByIntakeState
                    ? SystemState.FULL_EXTEND
                    : SystemState.STOPPED;
            case FULL_RETRACT -> {
                sentOutByIntakeState = false;
                yield SystemState.FULL_RETRACT;
            }
            case SLOW_CLOSE -> SystemState.SLOW_CLOSE;
            case RESYNC -> SystemState.HOMING;
        };
    }

    private void applyStates() {
        switch (systemState) {
            case FULL_EXTEND:
                commandBoth(100, false);
                break;
            case FULL_RETRACT:
                commandBoth(0, false);
                break;
            case SLOW_CLOSE:
                commandBoth(25, true);
                break;
            case HOMING:
                applyHoming();
                break;
            case STOPPED:
                stop();
                if (right.isAttached()) right.stopAxis();
                return;
        }
    }

    /**
     * Commands both axes to the same position, given as a percentage of maxRotations.
     *
     * @param percent 0 to 100
     * @param slow use the slow dynamic Motion Magic voltage profile
     */
    private void commandBoth(double percent, boolean slow) {
        final double rotations = percentToRotations(() -> percent);
        if (slow) {
            setDynMMPositionVoltage(
                    () -> rotations,
                    () -> config.getSlowMmCruiseVelocity(),
                    () -> config.getSlowMmAcceleration(),
                    () -> config.getSlowMmJerk());
            if (right.isAttached()) {
                right.goToRotationsSlow(
                        rotations,
                        config.getSlowMmCruiseVelocity(),
                        config.getSlowMmAcceleration(),
                        config.getSlowMmJerk());
            }
        } else {
            setMMPosition(() -> rotations);
            if (right.isAttached()) right.goToRotations(rotations);
        }
    }

    private final Timer homingTimer = new Timer();
    private boolean leftHomed = false;
    private boolean rightHomed = false;
    // homingTimer seconds when each side was last seen above the stall speed
    private double leftLastMoving = 0;
    private double rightLastMoving = 0;

    /**
     * Drives each side into the fully extended hard stop and re-zeroes that side's encoder at
     * maxRotations once it stalls. After a tooth skip the hard stop is the only position that can
     * be trusted, so homing uses a voltage that bypasses the soft limits and reaches the physical
     * stop even when the stale encoder thinks the soft limit is already reached.
     */
    private void applyHoming() {
        // Re-arm on entry to the HOMING state.
        if (previousSystemState != SystemState.HOMING) {
            homingTimer.restart();
            leftHomed = false;
            rightHomed = false;
            leftLastMoving = 0;
            rightLastMoving = 0;
        }

        boolean timedOut = homingTimer.get() >= config.getHomingTimeoutSecs();

        if (!leftHomed) {
            if (detectLeftStall()) {
                setMotorPosition(() -> config.getMaxRotations());
                stop();
                leftHomed = true;
            } else if (timedOut) {
                Telemetry.print("IntakeExtension: LEFT resync timed out");
                stop();
                leftHomed = true;
            } else {
                setVoltageOutputNoSoftLimit(() -> config.getHomingVoltage());
            }
        } else {
            stop();
        }

        if (right.isAttached()) {
            if (!rightHomed) {
                if (detectRightStall()) {
                    right.zeroAtMax();
                    right.stopAxis();
                    rightHomed = true;
                } else if (timedOut) {
                    Telemetry.print("IntakeExtension: RIGHT resync timed out");
                    right.stopAxis();
                    rightHomed = true;
                } else {
                    right.driveHomingVoltage(config.getHomingVoltage());
                }
            } else {
                right.stopAxis();
            }
        } else {
            rightHomed = true; // no right axis to resync
        }
    }

    private boolean detectLeftStall() {
        double now = homingTimer.get();
        if (Math.abs(getVelocityRPM()) >= config.getHomingStallRPM()) {
            leftLastMoving = now;
        }
        return isStalled(now, leftLastMoving);
    }

    private boolean detectRightStall() {
        double now = homingTimer.get();
        if (Math.abs(right.getVelocityRPM()) >= config.getHomingStallRPM()) {
            rightLastMoving = now;
        }
        return isStalled(now, rightLastMoving);
    }

    /**
     * A side is stalled once it has gone the debounce window without moving above the stall speed,
     * but never before the minimum drive time elapses, so the zero velocity before motion starts
     * does not read as a stall.
     */
    private boolean isStalled(double now, double lastMoving) {
        if (now < config.getHomingMinTimeSecs()) {
            return false;
        }
        return (now - lastMoving) >= config.getHomingStallDebounceSecs();
    }

    /** Whether the most recent resync has finished homing both sides. */
    public boolean isResyncComplete() {
        return systemState == SystemState.HOMING && leftHomed && rightHomed;
    }

    /**
     * Drives both sides into the extended hard stop and re-zeroes each one as it stalls, then
     * returns the subsystem to STOPPED. A homing timeout can end the command with a side that never
     * stalled, and that side is then left where it stopped rather than re-zeroed.
     *
     * @return a command that ends once both sides are re-zeroed, or once the homing timeout hits
     */
    public Command resyncCommand() {
        return startEnd(
                        () -> setWantedState(WantedState.RESYNC),
                        () -> setWantedState(WantedState.STOPPED))
                .until(this::isResyncComplete)
                .withName("IntakeExtension.resync");
    }

    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();
        logBatteryUsage();
        Telemetry.log("IntakeExtension/WantedState", wantedState.toString());
        Telemetry.log("IntakeExtension/SystemState", systemState.toString());
        Telemetry.log("IntakeExtension/CurrentCommand", getCurrentCommandName());
        Telemetry.log("IntakeExtension/Voltage", getVoltage(), "volts");
        Telemetry.log("IntakeExtension/StatorCurrent", getStatorCurrent(), "amps");
        Telemetry.log("IntakeExtension/SupplyCurrent", getSupplyCurrent(), "amps");
        Telemetry.log("IntakeExtension/Position", getPositionRotations(), "rotations");
        Telemetry.log("IntakeExtension/RPM", getVelocityRPM(), "RPM");
        Telemetry.log("IntakeExtension/Temp", getTemp(), "deg_C");
        Telemetry.log("IntakeExtension/LeftHomed", leftHomed);
        Telemetry.log("IntakeExtension/RightHomed", rightHomed);

        previousSystemState = systemState;
    }

    public void simulationInit() {
        if (isAttached()) {
            sim = new IntakeExtensionSim(RobotSim.leftView, motor.getSimState());
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
        }
    }

    class IntakeExtensionSim extends LinearSim {
        public IntakeExtensionSim(Mechanism2d mech, TalonFXSimState intakeExtensionMotorSim) {
            super(
                    new LinearConfig(
                                    config.getIntakeX(),
                                    config.getIntakeY(),
                                    config.getExtensionGearing(),
                                    config.getDrumRadiusMeters())
                            .setAngle(config.getAngle())
                            .setMovingLength(config.getMovingLength())
                            .setStaticLength(config.getStaticLength())
                            .setMaxHeight(config.getMaxExtensionHeight())
                            .setLineWidth(config.getLineWidth())
                            .setColor(new Color8Bit(Color.kLightGray)),
                    mech,
                    intakeExtensionMotorSim,
                    config.getName());
        }
    }
}

package frc.robot.subsystems.intakeExtension;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotSim;
import frc.robot.subsystems.intakeExtension.IntakeExtension.Left.LeftConfig;
import frc.robot.subsystems.intakeExtension.IntakeExtension.Right.RightConfig;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.LinearConfig;
import frc.spectrumLib.sim.LinearSim;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

/**
 * The Intake Extension subsystem. Extends and retracts the fuel intake.
 *
 * <p>The deploy is a rack-and-pinion driven by two independent motors: a left axis (CAN id 4) and a
 * right axis (CAN id 5). Each side runs its own closed-loop position control rather than one
 * following the other, so a side that skips teeth on the rack can be driven on its own to resync.
 *
 * <p>Normal deploy/retract states command both axes to the same setpoint. The {@code RESYNC} state
 * re-establishes truth by driving each side independently into the fully-extended hard stop (using
 * a soft-limit-bypassing voltage), detecting the stall, and re-zeroing that side's encoder at
 * {@code maxRotations}. This recovers from a tooth skip, which otherwise leaves the motor encoder
 * reading a position the rack is no longer at.
 *
 * <p>This is a container for two independent {@link Mechanism}s rather than a {@code Mechanism}
 * itself, so it implements {@link Subsystem} and registers directly. Without that registration the
 * scheduler would never call {@link #periodic()} and the state machine would never run.
 */
public class IntakeExtension implements Subsystem {

    public static class Left extends Mechanism {

        public static class LeftConfig extends Config {

            @Getter private final double initPosition = 0;
            @Getter private final double triggerTolerance = 0.317637;

            @Getter private final double zeroSpeed = -0.1;
            @Getter private final double holdMaxSpeedRPM = 18;

            @Getter private final double maxRotations = 3.652821;
            @Getter private final double minRotations = 0.0;

            @Getter private final double supplyCurrentLimit = 80;
            @Getter private final double statorCurrentLimit = 80;
            @Getter private final double lowerSupplyCurrentLimit = 40;
            @Getter private final double lowerSupplyCurrentTime = 1;

            @Getter private final double positionKp = 4.2;
            @Getter private final double positionKi = 0;
            @Getter private final double positionKd = 0;
            @Getter private final double positionKv = 0.39;
            @Getter private final double positionKs = 0;
            @Getter private final double positionKa = 0;
            @Getter private final double positionKg = -0.017;
            @Getter private final double gearRatio = 3.5;
            @Getter private final double rampPeriod = 0.02;

            @Getter private final double mmCruiseVelocity = 15.246559;
            @Getter private final double mmAcceleration = 76.232794;
            @Getter private final double mmJerk = 0;
            @Getter private final double slowMmCruiseVelocity = 7.623279;
            @Getter private final double slowMmAcceleration = 38.116397;
            @Getter private final double slowMmJerk = 0;

            @Getter private final double homingVoltage = 6;
            @Getter private final double homingStallRPM = 50.0;
            @Getter private final double homingMinTimeSecs = 0.3;
            @Getter private final double homingStallDebounceSecs = 0.15;
            @Getter private final double homingTimeoutSecs = 3.0;

            @Getter private final double intakeX = Units.inchesToMeters(70);
            @Getter private final double intakeY = Units.inchesToMeters(23);
            @Getter private final double extensionMass = 10.0;
            @Getter private final double drumRadiusMeters = Units.inchesToMeters(0.5010597711);
            @Getter private final double extensionGearing = 3.5;
            @Getter private final double angle = 180;
            @Getter private final double staticLength = 10;
            @Getter private final double movingLength = 55;
            @Getter private final double lineWidth = 20;
            @Getter private final double maxExtensionHeight = 40;

            public LeftConfig() {
                super("IntakeExtensionLeft", 4, Rio.CANIVORE);
                configMinMaxRotations(minRotations, maxRotations);
                configPIDGains(0, positionKp, positionKi, positionKd);
                configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
                configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
                configGravityType(false);
                configOpenLoopRamps(rampPeriod);
                configClosedLoopRamps(rampPeriod);
                configSupplyCurrentLimit(supplyCurrentLimit, true);
                configStatorCurrentLimit(statorCurrentLimit, true);
                configLowerSupplyCurrentLimit(lowerSupplyCurrentLimit);
                configLowerSupplyCurrentTime(lowerSupplyCurrentTime);
                configGearRatio(gearRatio);
                configForwardTorqueCurrentLimit(statorCurrentLimit);
                configReverseTorqueCurrentLimit(statorCurrentLimit);
                configForwardSoftLimit(maxRotations, true);
                configReverseSoftLimit(minRotations, true);
                configNeutralBrakeMode(false);
                configClockwise_Positive();
            }
        }

        @Getter private final LeftConfig config;
        @Getter private IntakeExtensionSim sim;

        public Left(LeftConfig config) {
            super(config);
            this.config = config;

            simulationInit();
            Telemetry.print(getName() + " Subsystem Initialized");
        }

        public void goToRotations(double rotations) {
            setMMPosition(() -> rotations);
        }

        public void goToRotationsSlow(
                double rotations, double cruiseVelocity, double acceleration, double jerk) {
            setDynMMPositionVoltage(
                    () -> rotations, () -> cruiseVelocity, () -> acceleration, () -> jerk);
        }

        public void driveHomingVoltage(double volts) {
            setVoltageOutputNoSoftLimit(() -> volts);
        }

        public void setInitialPosition(double rotations) {
            if (isAttached()) {
                motor.setPosition(rotations);
            }
        }

        public void zeroAtMax() {
            setMotorPosition(() -> config.getMaxRotations());
        }

        public void stopAxis() {
            stop();
        }

        @Override
        public void periodic() {
            logBatteryUsage();
            Telemetry.log("IntakeExtensionLeft/CurrentCommand", getCurrentCommandName());
            Telemetry.log("IntakeExtensionLeft/Voltage", getVoltage(), "volts");
            Telemetry.log("IntakeExtensionLeft/StatorCurrent", getStatorCurrent(), "amps");
            Telemetry.log("IntakeExtensionLeft/SupplyCurrent", getSupplyCurrent(), "amps");
            Telemetry.log("IntakeExtensionLeft/Position", getPositionRotations(), "rotations");
            Telemetry.log("IntakeExtensionLeft/RPM", getVelocityRPM(), "RPM");
            Telemetry.log("IntakeExtensionLeft/Temp", getTemp(), "deg_C");
        }

        public void simulationInit() {
            if (isAttached()) {
                sim = new IntakeExtensionSim(RobotSim.leftView, motor.getSimState());
            }
        }

        class IntakeExtensionSim extends LinearSim {
            public IntakeExtensionSim(Mechanism2d mech, TalonFXSimState motorSim) {
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
                        motorSim,
                        config.getName());
            }
        }
    }

    public static class Right extends Mechanism {

        public static class RightConfig extends Config {
            public RightConfig(LeftConfig left) {
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
                configGravityType(false);
                configOpenLoopRamps(left.getRampPeriod());
                configClosedLoopRamps(left.getRampPeriod());
                configSupplyCurrentLimit(left.getSupplyCurrentLimit(), true);
                configStatorCurrentLimit(left.getStatorCurrentLimit(), true);
                configLowerSupplyCurrentLimit(left.getLowerSupplyCurrentLimit());
                configLowerSupplyCurrentTime(left.getLowerSupplyCurrentTime());
                configGearRatio(left.getGearRatio());
                configForwardTorqueCurrentLimit(left.getStatorCurrentLimit());
                configReverseTorqueCurrentLimit(left.getStatorCurrentLimit());
                configForwardSoftLimit(left.getMaxRotations(), true);
                configReverseSoftLimit(left.getMinRotations(), true);
                configNeutralBrakeMode(false);
                configCounterClockwise_Positive();
            }
        }

        @Getter private final RightConfig config;

        public Right(RightConfig config) {
            super(config);
            this.config = config;
            Telemetry.print(getName() + " Subsystem Initialized");
        }

        public void goToRotations(double rotations) {
            setMMPosition(() -> rotations);
        }

        public void goToRotationsSlow(
                double rotations, double cruiseVelocity, double acceleration, double jerk) {
            setDynMMPositionVoltage(
                    () -> rotations, () -> cruiseVelocity, () -> acceleration, () -> jerk);
        }

        public void driveHomingVoltage(double volts) {
            setVoltageOutputNoSoftLimit(() -> volts);
        }

        public void setInitialPosition(double rotations) {
            if (isAttached()) {
                motor.setPosition(rotations);
            }
        }

        public void zeroAtMax() {
            setMotorPosition(() -> config.getMaxRotations());
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

    public static class IntakeExtensionConfig {

        @Getter private final LeftConfig leftConfig;
        @Getter private final RightConfig rightConfig;

        public IntakeExtensionConfig(LeftConfig leftConfig, RightConfig rightConfig) {
            this.leftConfig = leftConfig;
            this.rightConfig = rightConfig;
        }
    }

    // ---- State Machine ----

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
                left.stopAxis();
                right.stopAxis();
                return;
        }
    }

    private void commandBoth(double percent, boolean slow) {
        final double rotations = left.percentToRotations(() -> percent);
        if (slow) {
            left.goToRotationsSlow(
                    rotations,
                    config.getLeftConfig().getSlowMmCruiseVelocity(),
                    config.getLeftConfig().getSlowMmAcceleration(),
                    config.getLeftConfig().getSlowMmJerk());
            right.goToRotationsSlow(
                    rotations,
                    config.getLeftConfig().getSlowMmCruiseVelocity(),
                    config.getLeftConfig().getSlowMmAcceleration(),
                    config.getLeftConfig().getSlowMmJerk());
        } else {
            left.goToRotations(rotations);
            right.goToRotations(rotations);
        }
    }

    // ---- Resync / stall homing ----

    private final Timer homingTimer = new Timer();
    private boolean leftHomed = false;
    private boolean rightHomed = false;
    private double leftLastMoving = 0;
    private double rightLastMoving = 0;

    private void applyHoming() {
        if (previousSystemState != SystemState.HOMING) {
            homingTimer.restart();
            leftHomed = false;
            rightHomed = false;
            leftLastMoving = 0;
            rightLastMoving = 0;
        }

        double homingTimeout = config.getLeftConfig().getHomingTimeoutSecs();
        double homingVoltage = config.getLeftConfig().getHomingVoltage();
        boolean timedOut = homingTimer.get() >= homingTimeout;

        if (!leftHomed) {
            if (detectLeftStall() || timedOut) {
                if (timedOut) Telemetry.print("IntakeExtension: LEFT resync timed out");
                left.zeroAtMax();
                left.stopAxis();
                leftHomed = true;
            } else {
                left.driveHomingVoltage(homingVoltage);
            }
        } else {
            left.stopAxis();
        }

        if (right.isAttached()) {
            if (!rightHomed) {
                if (detectRightStall() || timedOut) {
                    if (timedOut) Telemetry.print("IntakeExtension: RIGHT resync timed out");
                    right.zeroAtMax();
                    right.stopAxis();
                    rightHomed = true;
                } else {
                    right.driveHomingVoltage(homingVoltage);
                }
            } else {
                right.stopAxis();
            }
        } else {
            rightHomed = true;
        }
    }

    private boolean detectLeftStall() {
        double now = homingTimer.get();
        double stallRPM = config.getLeftConfig().getHomingStallRPM();
        if (Math.abs(left.getVelocityRPM()) >= stallRPM) {
            leftLastMoving = now;
        }
        return isStalled(now, leftLastMoving);
    }

    private boolean detectRightStall() {
        double now = homingTimer.get();
        double stallRPM = config.getLeftConfig().getHomingStallRPM();
        if (Math.abs(right.getVelocityRPM()) >= stallRPM) {
            rightLastMoving = now;
        }
        return isStalled(now, rightLastMoving);
    }

    private boolean isStalled(double now, double lastMoving) {
        double minTime = config.getLeftConfig().getHomingMinTimeSecs();
        double debounce = config.getLeftConfig().getHomingStallDebounceSecs();
        if (now < minTime) {
            return false;
        }
        return (now - lastMoving) >= debounce;
    }

    public boolean isResyncComplete() {
        return systemState == SystemState.HOMING && leftHomed && rightHomed;
    }

    public Command resyncCommand() {
        return startEnd(
                        () -> setWantedState(WantedState.RESYNC),
                        () -> setWantedState(WantedState.STOPPED))
                .until(this::isResyncComplete)
                .withName("IntakeExtension.resync");
    }

    // ---- Subsystem plumbing ----

    @Getter private final Left left;
    @Getter private final Right right;
    @Getter private final IntakeExtensionConfig config;

    public IntakeExtension(IntakeExtensionConfig config) {
        this.config = config;
        this.left = new Left(config.getLeftConfig());
        this.right = new Right(config.getRightConfig());

        setInitialPosition();

        this.register();
        Telemetry.print("Intake Extension Subsystem Initialized");
    }

    private void setInitialPosition() {
        double initialRotations =
                left.degreesToRotations(() -> config.getLeftConfig().getInitPosition());
        left.setInitialPosition(initialRotations);
        right.setInitialPosition(initialRotations);
    }

    public void resetCurrentPositionToMax() {
        left.zeroAtMax();
        right.zeroAtMax();
    }

    public Command resetCurrentPositionToMaxCommand() {
        return new InstantCommand(this::resetCurrentPositionToMax);
    }

    public Command resetToInitialPos() {
        return new InstantCommand(this::setInitialPosition);
    }

    public void setBrakeMode(boolean isInBrake) {
        left.setBrakeMode(isInBrake);
        right.setBrakeMode(isInBrake);
    }

    public Left.IntakeExtensionSim getSim() {
        return left.getSim();
    }

    public double getPositionPercentage() {
        return left.getPositionPercentage();
    }

    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();

        Telemetry.log("IntakeExtension/WantedState", wantedState.toString());
        Telemetry.log("IntakeExtension/SystemState", systemState.toString());
        Telemetry.log("IntakeExtension/LeftHomed", leftHomed);
        Telemetry.log("IntakeExtension/RightHomed", rightHomed);

        previousSystemState = systemState;
    }
}

package frc.robot.subsystems.hood;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.rebuilt.ShotCalculator;
import frc.robot.RobotSim;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

public class Hood extends Mechanism {

    public static class HoodConfig extends Config {

        @Getter private final double initPosition = 0.0;

        /* 34.5 deg of travel */
        @Getter private final double maxRotations = 0.095833;
        @Getter private final double minRotations = 0.0;

        /** Position error (degrees) within which the hood counts as on target for a shot. */
        @Getter private final double aimToleranceDegrees = 0.5;

        /**
         * Below this angle (degrees) the hood is considered to be resting on its hard stop at home,
         * and output is cut instead of holding position 0. The hard stop sits fractionally above
         * the encoder zero, so holding 0 against it stalled the motor at 75 A stator continuously
         * on the bench (2026-09-04 logs) and heated it 19 C in 30 s of idle. Brake mode holds it.
         */
        @Getter private final double homeRestToleranceDegrees = 1.0;

        /* Hood config values */
        @Getter private final double supplyCurrentLimit = 80;
        @Getter private final double statorCurrentLimit = 80;
        @Getter private final double lowerSupplyCurrentLimit = 40;
        @Getter private final double lowerSupplyCurrentTime = 1;
        @Getter private final double positionKp = 2750;
        @Getter private final double positionKi = 0;
        @Getter private final double positionKd = 0;
        @Getter private final double positionKv = 10.22819093986847;
        @Getter private final double positionKs = 0.49;
        @Getter private final double positionKa = 0;
        @Getter private final double positionKg = 0;

        @Getter private final double gearRatio = 59.4;
        @Getter private final double mmCruiseVelocity = 0.1;
        @Getter private final double mmAcceleration = 0.4;
        @Getter private final double mmJerk = 0;
        @Getter private final double peakVoltage = 3;

        /* Sim Configs */
        @Getter private final double hoodX = Units.inchesToMeters(45);

        @Getter private final double hoodY = Units.inchesToMeters(52.5);
        @Getter private final double simRatio = gearRatio;
        @Getter private final double length = Units.inchesToMeters(7.735);

        /** Creates a new HoodConfig instance. */
        public HoodConfig() {
            super("Hood", 19, Rio.CANIVORE);
            configMinMaxRotations(minRotations, maxRotations);
            configPIDGains(0, positionKp, positionKi, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configForwardVoltageLimit(peakVoltage);
            configReverseVoltageLimit(-peakVoltage);
            configGearRatio(gearRatio);
            configSupplyCurrentLimit(supplyCurrentLimit, true);
            configStatorCurrentLimit(statorCurrentLimit, true);
            configLowerSupplyCurrentLimit(lowerSupplyCurrentLimit);
            configLowerSupplyCurrentTime(lowerSupplyCurrentTime);
            configForwardTorqueCurrentLimit(statorCurrentLimit);
            configReverseTorqueCurrentLimit(statorCurrentLimit);
            configForwardSoftLimit(maxRotations, true);
            configReverseSoftLimit(minRotations, true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
        }
    }

    public enum WantedState {
        HOME,
        STOPPED,
        AIM_AT_TARGET
    }

    public enum SystemState {
        HOME,
        STOPPED,
        AIM_AT_TARGET
    }

    private WantedState wantedState = WantedState.HOME;
    private SystemState systemState = SystemState.HOME;
    /**
     * Sets the wanted state.
     *
     * @param state the wanted state
     */
    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }
    /** Handles the state transition. */
    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case HOME -> SystemState.HOME;
            case STOPPED -> SystemState.STOPPED;
            case AIM_AT_TARGET -> SystemState.AIM_AT_TARGET;
        };
    }
    /** Hood angle commanded this loop (degrees). */
    @Getter private double commandedDegrees = 0;

    /** Applies the states. */
    private void applyStates() {
        double wantedDegrees = 0;
        switch (systemState) {
            case HOME:
                wantedDegrees = 0.0;
                if (getPositionDegrees() <= config.getHomeRestToleranceDegrees()) {
                    // Resting on the hard stop: stop pushing into it (see
                    // homeRestToleranceDegrees).
                    commandedDegrees = 0.0;
                    stop();
                    return;
                }
                break;
            case STOPPED:
                stop();
                return;
            case AIM_AT_TARGET:
                var params = ShotCalculator.getInstance().getParameters();
                wantedDegrees = params.hoodAngle();
                break;
        }
        commandedDegrees = wantedDegrees;
        final double finalWantedDegrees = wantedDegrees;
        final double finalWantedPosition = degreesToRotations(() -> finalWantedDegrees);
        // setMMPositionFOC
        setPosition(() -> finalWantedPosition);
    }

    /**
     * Returns {@code true} when the hood is aiming and within the configured tolerance of the
     * commanded shot angle. Gates feeding into the flywheel.
     */
    public boolean isAtAngle() {
        return isAtAngle(config.getAimToleranceDegrees());
    }

    /**
     * Same check as {@link #isAtAngle()} against a caller-supplied tolerance. The feeder gate uses
     * a wider tolerance to decide whether to <em>keep</em> feeding than to start.
     *
     * @param toleranceDegrees allowed angle error in degrees
     * @return true when aiming and within {@code toleranceDegrees} of the commanded angle
     */
    public boolean isAtAngle(double toleranceDegrees) {
        return systemState == SystemState.AIM_AT_TARGET
                && Math.abs(getPositionDegrees() - commandedDegrees) <= toleranceDegrees;
    }

    @Getter private final HoodConfig config;

    @Getter private HoodSim sim;

    /**
     * Creates a new Hood instance.
     *
     * @param config the config
     */
    public Hood(HoodConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }
    /** Runs the periodic update. */
    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();
        logBatteryUsage();
        Telemetry.log("Hood/WantedState", wantedState.toString());
        Telemetry.log("Hood/SystemState", systemState.toString());
        Telemetry.log("Hood/CurrentCommand", getCurrentCommandName());
        logDiagnostics("Hood", true);
        Telemetry.log("Hood/RPM", getVelocityRPM(), "RPM");
        Telemetry.logDash("Hood/PositionDegrees", getPositionDegrees(), "deg");
        Telemetry.log("Hood/CommandedDegrees", commandedDegrees, "deg");
        Telemetry.log("Hood/AtAngle", isAtAngle());
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    /** Simulation init. */
    public void simulationInit() {
        if (isAttached()) {
            sim = new HoodSim(RobotSim.leftView, motor);
        }
    }

    class HoodSim extends ArmSim {
        /**
         * Creates a new HoodSim instance.
         *
         * @param mech the mech
         * @param motor the motor
         */
        public HoodSim(Mechanism2d mech, TalonFX motor) {
            super(
                    new ArmConfig(
                                    config.hoodX,
                                    config.hoodY,
                                    config.simRatio,
                                    config.length,
                                    180 - config.getMaxRotations() * 360,
                                    180 - config.getMinRotations() * 360,
                                    180 - 9)
                            .setSimulatedGravity(false),
                    mech,
                    motor,
                    config.getName());
        }
    }
}

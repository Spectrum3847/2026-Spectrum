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
        // TODO: tune
        @Getter private final double initPosition = 0.0;

        /* 34.5 deg of travel */
        @Getter private final double maxRotations = 0.095833;
        @Getter private final double minRotations = 0.0;

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
        @Getter private final double hoodX = Units.inchesToMeters(50);
        @Getter private final double hoodY = Units.inchesToMeters(65);
        @Getter private final double simRatio = 8.49;
        @Getter private final double length = Units.inchesToMeters(7.735);

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
            configClockwise_Positive();
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

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case HOME -> SystemState.HOME;
            case STOPPED -> SystemState.STOPPED;
            case AIM_AT_TARGET -> SystemState.AIM_AT_TARGET;
        };
    }

    private void applyStates() {
        double wantedDegrees = 0;
        switch (systemState) {
            case HOME:
                wantedDegrees = 0.0;
                break;
            case STOPPED:
                stop();
                return;
            case AIM_AT_TARGET:
                var params = ShotCalculator.getInstance().getParameters();
                wantedDegrees = params.hoodAngle();
                break;
        }
        final double finalWantedDegrees = wantedDegrees;
        final double finalWantedPosition = degreesToRotations(() -> finalWantedDegrees);
        setPosition(() -> finalWantedPosition);
    }

    @Getter private final HoodConfig config;
    @Getter private HoodSim sim;

    public Hood(HoodConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();
        logBatteryUsage();
        Telemetry.log("Hood/WantedState", wantedState.toString());
        Telemetry.log("Hood/SystemState", systemState.toString());
        Telemetry.log("Hood/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Hood/Voltage", getVoltage(), "volts");
        Telemetry.log("Hood/StatorCurrent", getStatorCurrent(), "amps");
        Telemetry.log("Hood/SupplyCurrent", getSupplyCurrent(), "amps");
        Telemetry.log("Hood/RPM", getVelocityRPM(), "RPM");
        Telemetry.log("Hood/Temp", getTemp(), "deg_C");
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    public void simulationInit() {
        if (isAttached()) {
            sim = new HoodSim(RobotSim.leftView, motor);
        }
    }

    class HoodSim extends ArmSim {
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

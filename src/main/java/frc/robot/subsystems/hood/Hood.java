package frc.robot.subsystems.hood;

import com.ctre.phoenix6.sim.TalonFXSimState;
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
import lombok.Setter;

/** The Hood subsystem. Positions the hood that sets the fuel launch angle. */
public class Hood extends Mechanism {

    public static class HoodConfig extends Config {

        @Getter private final double initPosition = 9;

        @Getter @Setter private double maxRotations = 0.137;
        @Getter @Setter private double minRotations = 0.024;

        /* Hood config values */
        @Getter private final double currentLimit = 40;
        @Getter private final double torqueCurrentLimit = 80;
        @Getter private final double positionKp = 3000;
        @Getter private final double positionKi = 0;
        @Getter private final double positionKd = 220;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 25;
        @Getter private final double positionKa = 0;
        @Getter private final double positionKg = 0;

        @Getter private final double gearRatio = 51.667;
        @Getter private final double mmCruiseVelocity = 50;
        @Getter private final double mmAcceleration = 200;
        @Getter private final double mmJerk = 1000;
        @Getter private final double holdMaxSpeedRPM = 18;

        /* Sim Configs */
        @Getter private final double hoodX = Units.inchesToMeters(62.5);
        @Getter private final double hoodY = Units.inchesToMeters(50);
        @Getter private final double simRatio = 51.667;
        @Getter private final double length = Units.inchesToMeters(10);

        public HoodConfig() {
            super("Hood", 15, Rio.CANIVORE);
            configMinMaxRotations(minRotations, maxRotations);
            configPIDGains(0, positionKp, positionKi, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(gearRatio);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configForwardSoftLimit(maxRotations, true);
            configReverseSoftLimit(minRotations, true);
            configNeutralBrakeMode(true);
            configClockwise_Positive();
        }
    }

    // ---- State Machine ----

    public enum WantedState {
        HOME,
        STOPPED,
        AIM_AT_TARGET,
    }

    public enum SystemState {
        HOME,
        STOPPED,
        AIM_AT_TARGET,
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
        double wantedDegrees = 9;
        switch (systemState) {
            case HOME:
                wantedDegrees = 9.0;
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
        setMMPositionFoc(() -> finalWantedPosition);
    }

    @Getter private final HoodConfig config;
    @Getter private HoodSim sim;

    public Hood(HoodConfig config) {
        super(config);
        this.config = config;

        setInitialPosition();

        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    private void setInitialPosition() {
        if (isAttached()) {
            motor.setPosition(degreesToRotations(() -> config.getInitPosition()));
        }
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
        Telemetry.log("Hood/PositionDegrees", getPositionDegrees(), "degrees");
        Telemetry.log("Hood/RPM", getVelocityRPM(), "RPM");
        Telemetry.log("Hood/Temp", getTemp(), "deg_C");
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    public void simulationInit() {
        if (isAttached()) {
            sim = new HoodSim(RobotSim.leftView, motor.getSimState());
        }
    }

    // Must be called to enable the simulation
    // if roller position changes configure x and y to set position.
    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
        }
    }

    class HoodSim extends ArmSim {
        public HoodSim(Mechanism2d mech, TalonFXSimState armMotorSim) {
            super(
                    new ArmConfig(
                                    config.hoodX,
                                    config.hoodY,
                                    config.simRatio,
                                    config.length,
                                    90,
                                    180 - 9,
                                    180 - 9)
                            .setSimulatedGravity(false),
                    mech,
                    armMotorSim,
                    config.getName());
        }
    }
}

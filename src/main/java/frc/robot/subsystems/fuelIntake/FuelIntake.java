package frc.robot.subsystems.fuelIntake;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.util.Units;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

/** The Fuel Intake subsystem. Responsible for intake and handling of fuel elements. */
public class FuelIntake extends Mechanism {

    public static class FuelIntakeConfig extends Config {

        /* Intake config values */
        @Getter private final double supplyCurrentLimit = 70;
        @Getter private final double statorCurrentLimit = 180;
        @Getter private final double velocityKp = 5;
        @Getter private final double velocityKv = 0;
        @Getter private final double velocityKs = 4;

        /* Sim Configs */
        @Getter private final double intakeX = Units.inchesToMeters(15);
        @Getter private final double intakeY = Units.inchesToMeters(23);
        @Getter private final double wheelDiameter = 6;

        public FuelIntakeConfig() {
            super("Intake Left", 5, Rio.RIO_CANBUS);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1);
            configSupplyCurrentLimit(supplyCurrentLimit, true);
            configStatorCurrentLimit(statorCurrentLimit, true);
            configForwardTorqueCurrentLimit(statorCurrentLimit);
            configReverseTorqueCurrentLimit(statorCurrentLimit);
            configNeutralBrakeMode(false);
            configCounterClockwise_Positive();
            setFollowerConfigs(
                    new FollowerConfig(
                            "Intake Right", 6, Rio.RIO_CANBUS, MotorAlignmentValue.Opposed));
        }
    }

    // ---- State Machine ----

    public enum WantedState {
        NEUTRAL,
        OFF,
        INTAKE,
        SLOW_INTAKE,
    }

    public enum SystemState {
        NEUTRAL,
        OFF,
        INTAKE,
        SLOW_INTAKE,
    }

    private WantedState wantedState = WantedState.NEUTRAL;
    private SystemState systemState = SystemState.NEUTRAL;

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case NEUTRAL -> SystemState.NEUTRAL;
            case INTAKE -> SystemState.INTAKE;
            case SLOW_INTAKE -> SystemState.SLOW_INTAKE;
            case OFF -> SystemState.OFF;
        };
    }

    private void applyStates() {
        double wantedTorqueCurrent = 0;
        switch (systemState) {
            case NEUTRAL:
                wantedTorqueCurrent = 0;
                break;
            case INTAKE:
                wantedTorqueCurrent = 130;
                break;
            case SLOW_INTAKE:
                wantedTorqueCurrent = 45;
                break;
            case OFF:
                stop();
                return;
        }
        final double finalWantedTorqueCurrent = wantedTorqueCurrent;
        setTorqueCurrentFoc(() -> finalWantedTorqueCurrent);
    }

    @Getter private final FuelIntakeConfig config;
    // @Getter private FuelIntakeSim sim;

    public FuelIntake(FuelIntakeConfig config) {
        super(config);
        this.config = config;

        // simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();
        logBatteryUsage();
        Telemetry.log("FuelIntake/WantedState", wantedState.toString());
        Telemetry.log("FuelIntake/SystemState", systemState.toString());
        Telemetry.log("FuelIntake/CurrentCommand", getCurrentCommandName());
        Telemetry.log("FuelIntake/Voltage", getVoltage(), "volts");
        Telemetry.log("FuelIntake/StatorCurrent", getStatorCurrent(), "amps");
        Telemetry.log("FuelIntake/SupplyCurrent", getSupplyCurrent(), "amps");
        Telemetry.log("FuelIntake/RPM", getVelocityRPM(), "RPM");
        Telemetry.log("FuelIntake/Temp", getTemp(), "deg_C");
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    // public void simulationInit() {
    //     if (isAttached()) {
    //         // Create a new RollerSim with the left view, the motor's sim state, and a 6 in diameter
    //         sim = new FuelIntakeSim(RobotSim.leftView, motor.getSimState());
    //     }
    // }

    // // Must be called to enable the simulation
    // // if roller position changes configure x and y to set position.
    // @Override
    // public void simulationPeriodic() {
    //     if (isAttached()) {
    //         sim.simulationPeriodic();
    //     }
    // }

    // class FuelIntakeSim extends RollerSim {
    //     public FuelIntakeSim(Mechanism2d mech, TalonFXSimState rollerMotorSim) {
    //         super(
    //                 new RollerConfig(config.getWheelDiameter())
    //                         .setPosition(config.getIntakeX(), config.getIntakeY())
    //                         .setMount(Robot.getIntakeExtension().getSim()),
    //                 mech,
    //                 rollerMotorSim,
    //                 config.getName());
    //     }
    // }
}

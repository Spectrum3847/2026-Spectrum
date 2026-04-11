package frc.robot.indexerTower;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

public class IndexerTower extends Mechanism {

    public static class IndexerTowerConfig extends Config {

        // Intake Voltages and Current
        @Getter @Setter private double indexVoltageOut = 10;
        @Getter @Setter private double unjamVoltageOut = -10;
        @Getter @Setter private double indexerTorqueCurrent = 80;
        @Getter @Setter private double indexerVelocityRPM = 3000;
        @Getter @Setter private double indexerSlowVelocityRPM = 1000;
        @Getter @Setter private double indexerUnjamRPM = -1500;

        @Getter
        private final DoubleSubscriber indexerTowerFeedRPM =
                Telemetry.tunable("Tunable/IndexerTowerFeedRPM", indexerVelocityRPM);

        /* Intake config values */
        @Getter @Setter private double currentLimit = 80;
        @Getter @Setter private double torqueCurrentLimit = 140;
        @Getter @Setter private double lowerCurrentLimit = 60;
        @Getter @Setter private double timeUntilLowerCurrent = 1;
        @Getter @Setter private double velocityKp = 35;
        @Getter @Setter private double velocityKv = 0;
        @Getter @Setter private double velocityKs = 8;

        /* Sim Configs */
        @Getter private double intakeX = Units.inchesToMeters(60);
        @Getter private double intakeY = Units.inchesToMeters(75);
        @Getter private double wheelDiameter = 12;

        public IndexerTowerConfig() {
            super("IndexerTower", 51, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configLowerSupplyCurrentLimit(lowerCurrentLimit);
            configLowerSupplyCurrentTime(timeUntilLowerCurrent);
            configNeutralBrakeMode(false);
            configClockwise_Positive();
            setFollowerConfigs(
                    new FollowerConfig(
                            "IndexerTower Follower",
                            52,
                            Rio.CANIVORE,
                            MotorAlignmentValue.Aligned));
        }
    }

    // private IndexerTowerConfig config;
    // private IndexerSim sim;

    public IndexerTower(IndexerTowerConfig config) {
        super(config);
        this.config = config;

        // simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {
        logBatteryUsage();
        Telemetry.log("IndexerTower/CurrentCommand", getCurrentCommandName());
        Telemetry.log("IndexerTower/Voltage", getVoltage());
        Telemetry.log("IndexerTower/Current", getStatorCurrent());
        Telemetry.log("IndexerTower/RPM", getVelocityRPM());
    }

    @Override
    public void setupStates() {}

    @Override
    public void setupDefaultCommand() {
        IndexerTowerStates.setupDefaultCommand();
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    public Command runTorqueFOC(DoubleSupplier torque) {
        return run(() -> setTorqueCurrentFoc(torque));
    }

    public void setVoltageAndCurrentLimits(
            DoubleSupplier voltage, DoubleSupplier supply, DoubleSupplier torque) {
        setVoltageOutput(voltage);
        setCurrentLimits(supply, torque);
    }

    public Command runVoltageCurrentLimits(
            DoubleSupplier voltage, DoubleSupplier supplyCurrent, DoubleSupplier torqueCurrent) {
        return runVoltage(voltage).alongWith(runCurrentLimits(supplyCurrent, torqueCurrent));
    }

    public Command runTCcurrentLimits(DoubleSupplier torqueCurrent, DoubleSupplier supplyCurrent) {
        return runTorqueCurrentFoc(torqueCurrent)
                .alongWith(runCurrentLimits(supplyCurrent, torqueCurrent));
    }

    public Command stopMotor() {
        return run(() -> stop());
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    // public void simulationInit() {
    //     if (isAttached()) {
    //         // Create a new RollerSim with the left view, the motor's sim state, and a 6 in
    // diameter
    //         sim = new IndexerSim(RobotSim.topView, motor.getSimState());
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

    // class IndexerSim extends RollerSim {
    //     public IndexerSim(Mechanism2d mech, TalonFXSimState rollerMotorSim) {
    //         super(
    //                 new RollerConfig(config.getWheelDiameter())
    //                         .setPosition(config.getIntakeX(), config.getIntakeY()),
    //                 mech,
    //                 rollerMotorSim,
    //                 config.getName());
    //     }
    // }
}

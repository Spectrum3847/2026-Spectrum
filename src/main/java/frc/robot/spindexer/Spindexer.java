package frc.robot.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSim;
import frc.robot.RobotStates;
import frc.robot.State;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.LinearConfig;
import frc.spectrumLib.sim.LinearSim;
import frc.spectrumLib.util.Util;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

public class Spindexer extends Mechanism {
    
    public static class SpindexerConfig extends Config {

        // TODO: Get ID and values when robot is built
        private int spindexerId;

        @Getter @Setter private double spindexerVoltageOut;
        @Getter @Setter private double spindexerTorqueCurrent;
        @Getter @Setter private double spindexerVelocityRPM;

        /* Spindexer config values */
        @Getter @Setter private double currentLimit;
        @Getter @Setter private double torqueCurrentLimit;
        @Getter @Setter private double velocityKp;
        @Getter @Setter private double velocityKv;
        @Getter @Setter private double velocityKs;

        public SpindexerConfig() {
            super("Spindexer", spindexerId, Rio.CANIVORE);
            /*
              Must add the following:
              - configPIDGains()
              - configFeedForwardGains()
              - configSupplyCurrentLimits()
              - configStatorCurrentLimit()
              - configForwardTorqueCurrent();
              - configClockwise_Positive()
              - follower?
            */
        }
    }

    public Spindexer(SpindexerConfig config) {
        super(config);
        this.config = config;

        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {
        logBatteryUsage();
        Telemetry.log("Spindexer/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Spindexer/Voltage", getVoltage(), "volts");
        Telemetry.log("Spindexer/StatorCurrent", getStatorCurrent(), "amps");
        Telemetry.log("Spindexer/SupplyCurrent", getSupplyCurrent(), "amps");
        Telemetry.log("Spindexer/RPM", getVelocityRPM(), "RPM");
        Telemetry.log("Spindexer/Temp", getTemp(), "deg_C");
    }

    @Override
    public void setupStates() {}

    @Override
    public void setupDefaultCommand() {
        // SpindexerStates.setupDefaultCommand();
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    // From IndexerBed, let's continue to think about what will be different in a Spindexer vs Indexer
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
}

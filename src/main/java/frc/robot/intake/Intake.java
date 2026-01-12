package frc.robot.intake;

import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

public class Intake extends Mechanism {

    public static class IntakeConfig extends Config {

        // Intake Voltages and Current
        @Getter @Setter private double fuelIntakeVoltage = 9.0;
        @Getter @Setter private double fuelIntakeSupplyCurrent = 30.0;
        @Getter @Setter private double fuelIntakeTorqueCurrent = 85.0;

        /* Intake config values */
        @Getter private double currentLimit = 44;
        @Getter private double torqueCurrentLimit = 200;
        @Getter private double velocityKp = 12;
        @Getter private double velocityKv = 0.2;
        @Getter private double velocityKs = 14;

        /* Sim Configs */
        @Getter private double intakeX = 0.8;
        @Getter private double intakeY = 1.3;
        @Getter private double wheelDiameter = 5.0;

        public IntakeConfig() {
            super("Intake", 5, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
        }
    }

    private IntakeConfig config;

    public Intake(IntakeConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    @Override
    public void setupStates() {}

    @Override
    public void setupDefaultCommand() {
        IntakeStates.setupDefaultCommand();
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/

    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addStringProperty("CurrentCommand", this::getCurrentCommandName, null);
            builder.addDoubleProperty("Motor Voltage", this::getVoltage, null);
            // builder.addDoubleProperty("Rotations", this::getPositionRotations, null);
            builder.addDoubleProperty("Velocity RPM", this::getVelocityRPM, null);
            builder.addDoubleProperty("StatorCurrent", this::getStatorCurrent, null);
        }
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
    public void simulationInit() {
        if (isAttached()) {
            // Create a new RollerSim with the left view, the motor's sim state, and a 6 in diameter
            // sim = new CoralIntakeSim(RobotSim.leftView, motor.getSimState());
        }
    }

    // Must be called to enable the simulation
    // if roller position changes configure x and y to set position.
    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            // sim.simulationPeriodic();
        }
    }

    // class CoralIntakeSim extends RollerSim {
        // public CoralIntakeSim(Mechanism2d mech, TalonFXSimState coralRollerMotorSim) {
        //     super(
        //             new RollerConfig(config.wheelDiameter)
        //                     .setPosition(config.intakeX, config.intakeY)
        //                     .setMount(Robot.getElbow().getSim()),
        //             mech,
        //             coralRollerMotorSim,
        //             config.getName());
        // }
    // }
}

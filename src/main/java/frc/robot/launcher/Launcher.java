package frc.robot.launcher;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.rebuilt.ShotCalculator;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.RollerConfig;
import frc.spectrumLib.sim.RollerSim;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

public class Launcher extends Mechanism {

    public static class LauncherConfig extends Config {

        // Intake Voltages and Current
        @Getter @Setter private double LauncherVoltage = 9.0;
        @Getter @Setter private double LauncherSupplyCurrent = 30.0;
        @Getter @Setter private double LauncherTorqueCurrent = 85.0;

        @Getter @Setter private double idlingRPM = 700;
        @Getter @Setter private double slowLaunchSpeed = 400;
        @Getter @Setter private double autoTrenchLaunch = 1800;

        @Getter
        private final DoubleSubscriber onTheFlySpeed =
                Telemetry.tunable("Launcher/OnTheFlySpeed", 0.0);

        /* Launcher config values */
        @Getter private double currentLimit = 80;
        @Getter private double torqueCurrentLimit = 160;
        @Getter private double forwardTorqueCurrentLimit = torqueCurrentLimit;
        @Getter private double reverseTorqueCurrentLimit = -20;
        @Getter private double lowerCurrentLimit = 60;
        @Getter private double timeUntilLowerCurrent = 1;
        @Getter private double nominalVoltage = 16;
        @Getter private double velocityKp = 10;
        @Getter private double velocityKv = 0;
        @Getter private double velocityKs = 20;

        @Getter private double onTargetToleranceRPM = 100;

        /* Sim Configs */
        @Getter private double launcherX = Units.inchesToMeters(62.5);
        @Getter private double launcherY = Units.inchesToMeters(60);
        @Getter private double wheelDiameter = 4;

        public LauncherConfig() {
            super("Launcher Top Left", 46, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1);
            configLowerSupplyCurrentLimit(lowerCurrentLimit);
            configLowerSupplyCurrentTime(timeUntilLowerCurrent);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(forwardTorqueCurrentLimit);
            configReverseTorqueCurrentLimit(reverseTorqueCurrentLimit);
            configNeutralBrakeMode(false);
            configForwardVoltageLimit(nominalVoltage);
            configReverseVoltageLimit(nominalVoltage);
            configClockwise_Positive();
            setFollowerConfigs(
                    new FollowerConfig(
                            "Launcher Top Right", 47, Rio.CANIVORE, MotorAlignmentValue.Opposed),
                    new FollowerConfig(
                            "Launcher Bottom Left", 48, Rio.CANIVORE, MotorAlignmentValue.Aligned),
                    new FollowerConfig(
                            "Launcher Bottom Right",
                            49,
                            Rio.CANIVORE,
                            MotorAlignmentValue.Opposed));
        }
    }

    private LauncherConfig config;
    private LauncherSim sim;

    public Launcher(LauncherConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {
        logBatteryUsage();
        Telemetry.log("Launcher/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Launcher/Voltage", getVoltage(), "volts");
        Telemetry.log("Launcher/Current", getStatorCurrent(), "amps");
        Telemetry.log("Launcher/RPM", getVelocityRPM(), "RPM");
    }

    @Override
    public void setupStates() {}

    @Override
    public void setupDefaultCommand() {
        LauncherStates.setupDefaultCommand();
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

    public Command trackTargetCommand() {
        return run(() -> {
                    var params = ShotCalculator.getInstance().getParameters();
                    setVelocityTCFOCrpm(() -> params.flywheelSpeed());
                })
                .withName("Launcher.trackTargetCommand");
    }

    public Command onTheFlyLaunch() {
        return run(() -> {
                    setVelocityTCFOCrpm(() -> config.getOnTheFlySpeed().get());
                })
                .withName("Launcher.onTheFlyLaunch");
    }

    public Trigger aimingAtTarget() {
        return new Trigger(
                () -> {
                    var params = ShotCalculator.getInstance().getParameters();

                    double targetRPM = params.flywheelSpeed();
                    double currentRPM = getVelocityRPM();

                    double errorRPM = currentRPM - targetRPM;

                    return Math.abs(errorRPM) < config.getOnTargetToleranceRPM();
                });
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    public void simulationInit() {
        if (isAttached()) {
            sim = new LauncherSim(RobotSim.leftView, motor.getSimState());
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

    class LauncherSim extends RollerSim {
        public LauncherSim(Mechanism2d mech, TalonFXSimState rollerMotorSim) {
            super(
                    new RollerConfig(config.getWheelDiameter())
                            .setPosition(config.getLauncherX(), config.getLauncherY())
                            .setMount(Robot.getHood().getSim()),
                    mech,
                    rollerMotorSim,
                    config.getName());
        }
    }
}

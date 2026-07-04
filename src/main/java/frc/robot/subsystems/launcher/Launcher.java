package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.RobotSim;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.RollerConfig;
import frc.spectrumLib.sim.RollerSim;
import frc.spectrumLib.telemetry.*;
import lombok.Getter;
import lombok.Setter;

public class Launcher extends Mechanism {

    public static class LauncherConfig extends Config {

        // Intake Voltages and Current
        @Getter @Setter private double LauncherVoltage = 9.0;
        @Getter @Setter private double LauncherSupplyCurrent = 30.0;
        @Getter @Setter private double LauncherStatorCurrent = 85.0;

        @Getter @Setter private double idlingRPM = 700;
        @Getter @Setter private double slowLaunchSpeed = 400;
        @Getter @Setter private double autoTrenchLaunch = 1800;

        @Getter
        private final DoubleSubscriber onTheFlySpeed =
                Telemetry.tunable("Launcher/OnTheFlySpeed", 0.0);

        /* Launcher config values */
        @Getter private double supplyCurrentLimit = 80;
        @Getter private double statorCurrentLimit = 100;
        @Getter private double forwardStatorCurrentLimit = statorCurrentLimit;
        @Getter private double reverseStatorCurrentLimit = -10;
        @Getter private double lowerSupplyCurrentLimit = 60;
        @Getter private double timeUntilLowerCurrent = 1;
        @Getter private double nominalVoltage = 16;
        @Getter private double velocityKp = 10;
        @Getter private double velocityKv = 0;
        @Getter private double velocityKs = 20;

        @Getter private double onTargetToleranceRPM = 100;

        /* Sim Configs */
        @Getter private double launcherX = Units.inchesToMeters(50);
        @Getter private double launcherY = Units.inchesToMeters(63);
        @Getter private double wheelDiameter = 4;

        public LauncherConfig() {
            super("Launcher", 46, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1);
            configLowerSupplyCurrentLimit(lowerSupplyCurrentLimit);
            configLowerSupplyCurrentTime(timeUntilLowerCurrent);
            configSupplyCurrentLimit(supplyCurrentLimit, true);
            configStatorCurrentLimit(statorCurrentLimit, true);
            configForwardTorqueCurrentLimit(forwardStatorCurrentLimit);
            configReverseTorqueCurrentLimit(reverseStatorCurrentLimit);
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

    // ---- State Machine ----

    public enum WantedState {
        OFF,
        IDLE_PREP,
        SLOW_LAUNCH,
        AIM_AT_TARGET,
    }

    public enum SystemState {
        OFF,
        IDLE_PREP,
        SLOW_LAUNCH,
        AIM_AT_TARGET,
    }

    private WantedState wantedState = WantedState.OFF;
    private SystemState systemState = SystemState.OFF;

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    @SuppressWarnings("unused")
    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case OFF -> SystemState.OFF;
            case IDLE_PREP -> SystemState.IDLE_PREP;
            case SLOW_LAUNCH -> SystemState.SLOW_LAUNCH;
            case AIM_AT_TARGET -> SystemState.AIM_AT_TARGET;
        };
    }

    // TODO: add actual values when robot is built
    @SuppressWarnings("unused")
    private void applyStates() {
        double wantedRPM = 0;
        switch (systemState) {
            case OFF:
                stop();
                return;
            case IDLE_PREP:
                wantedRPM = 5000;
                break;
            case SLOW_LAUNCH:
                wantedRPM = 1000;
                break;
            case AIM_AT_TARGET:
                wantedRPM = -2000;
                break;
        }
        final double finalWantedRPM = wantedRPM;
        setVelocityTCFOCrpm(() -> finalWantedRPM);
    }

    @Getter private LauncherConfig config;
    @Getter private LauncherSim sim;

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
        Telemetry.log("Launcher/StatorCurrent", getStatorCurrent(), "amps");
        Telemetry.log("Launcher/SupplyCurrent", getSupplyCurrent(), "amps");
        Telemetry.log("Launcher/RPM", getVelocityRPM(), "RPM");
        Telemetry.log("Launcher/Temp", getTemp(), "deg_C");
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // // --------------------------------------------------------------------------------
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
                            .setPosition(config.getLauncherX(), config.getLauncherY()),
                    mech,
                    rollerMotorSim,
                    config.getName());
        }
    }
}

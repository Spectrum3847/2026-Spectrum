package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.rebuilt.ShotCalculator;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.RollerConfig;
import frc.spectrumLib.sim.RollerSim;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

/** The Launcher subsystem. Four-motor flywheel that launches fuel at the hub. */
public class Launcher extends Mechanism {

    public static class LauncherConfig extends Config {

        /* Launcher config values */
        @Getter private final double supplyCurrentLimit = 80;
        @Getter private final double statorCurrentLimit = 100;
        @Getter private final double lowerSupplyCurrentLimit = 80;
        @Getter private final double lowerSupplyCurrentTime = 0;
        @Getter private final double forwardTorqueCurrentLimit = statorCurrentLimit;
        @Getter private final double reverseTorqueCurrentLimit = 10;
        @Getter private final double voltageLimit = 12;
        @Getter private final double velocityKp = 10;
        @Getter private final double velocityKv = 0;
        @Getter private final double velocityKs = 20;

        @Getter private final double onTargetToleranceRPM = 100;

        /* Sim Configs */
        @Getter private final double launcherX = Units.inchesToMeters(62.5);
        @Getter private final double launcherY = Units.inchesToMeters(60);
        @Getter private final double wheelDiameter = 4;

        public LauncherConfig() {
            super("Launcher", 46, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1);
            configLowerSupplyCurrentLimit(lowerSupplyCurrentLimit);
            configLowerSupplyCurrentTime(lowerSupplyCurrentTime);
            configSupplyCurrentLimit(supplyCurrentLimit, true);
            configStatorCurrentLimit(statorCurrentLimit, true);
            configForwardTorqueCurrentLimit(forwardTorqueCurrentLimit);
            configReverseTorqueCurrentLimit(reverseTorqueCurrentLimit);
            configNeutralBrakeMode(false);
            configForwardVoltageLimit(voltageLimit);
            configReverseVoltageLimit(-voltageLimit);
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

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case OFF -> SystemState.OFF;
            case IDLE_PREP -> SystemState.IDLE_PREP;
            case SLOW_LAUNCH -> SystemState.SLOW_LAUNCH;
            case AIM_AT_TARGET -> SystemState.AIM_AT_TARGET;
        };
    }

    private void applyStates() {
        double wantedRPM = 0;
        switch (systemState) {
            case OFF:
                stop();
                return;
            case IDLE_PREP:
                wantedRPM = 700;
                break;
            case SLOW_LAUNCH:
                wantedRPM = 400;
                break;
            case AIM_AT_TARGET:
                var params = ShotCalculator.getInstance().getParameters();
                wantedRPM = params.flywheelSpeed();
                break;
        }
        final double finalWantedRPM = wantedRPM;
        setVelocityTCFOCrpm(() -> finalWantedRPM);
    }

    @Getter private final LauncherConfig config;
    @Getter private LauncherSim sim;

    public Launcher(LauncherConfig config) {
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
        Telemetry.log("Launcher/WantedState", wantedState.toString());
        Telemetry.log("Launcher/SystemState", systemState.toString());
        Telemetry.log("Launcher/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Launcher/Voltage", getVoltage(), "volts");
        Telemetry.log("Launcher/StatorCurrent", getStatorCurrent(), "amps");
        Telemetry.log("Launcher/SupplyCurrent", getSupplyCurrent(), "amps");
        Telemetry.log("Launcher/RPM", getVelocityRPM(), "RPM");
        Telemetry.log("Launcher/Temp", getTemp(), "deg_C");
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

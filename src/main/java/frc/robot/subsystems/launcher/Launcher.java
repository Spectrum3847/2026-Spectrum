package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.rebuilt.ShotCalculator;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.RollerConfig;
import frc.spectrumLib.sim.RollerSim;
import frc.spectrumLib.telemetry.*;
import lombok.Getter;

public class Launcher extends Mechanism {

    public static class LauncherConfig extends Config {

        // tune
        @Getter private final double idlingRPM = 700;
        @Getter private final double slowLaunchSpeed = 400;
        @Getter private final double autoTrenchLaunch = 1800;

        @Getter
        private final DoubleSubscriber onTheFlySpeed =
                Telemetry.tunable("Launcher/OnTheFlySpeed", 0.0);

        /* Launcher config values */
        @Getter private final double supplyCurrentLimit = 80;
        @Getter private final double statorCurrentLimit = 80;
        @Getter private final double forwardStatorCurrentLimit = statorCurrentLimit;
        @Getter private final double reverseStatorCurrentLimit = -10;
        @Getter private final double lowerSupplyCurrentLimit = 40;
        @Getter private final double timeUntilLowerCurrent = 1;
        @Getter private final double nominalVoltage = 16;
        // TODO: tune
        @Getter private double velocityKp = 0.5;
        @Getter private double velocityKv = 0.1425;
        @Getter private double velocityKs = 0;

        @Getter private double onTargetToleranceRPM = 200;

        @Getter private double gearRatio = 1.38;

        /* Sim Configs */
        @Getter private final double launcherX = Units.inchesToMeters(50);

        @Getter private final double launcherY = Units.inchesToMeters(65);
        @Getter private final double wheelDiameter = 4;

        /** Creates a new LauncherConfig instance. */
        public LauncherConfig() {
            super("Launcher Front Left", 15, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(gearRatio);
            configLowerSupplyCurrentLimit(lowerSupplyCurrentLimit);
            configLowerSupplyCurrentTime(timeUntilLowerCurrent);
            configSupplyCurrentLimit(supplyCurrentLimit, true);
            configStatorCurrentLimit(statorCurrentLimit, true);
            configForwardTorqueCurrentLimit(forwardStatorCurrentLimit);
            configReverseTorqueCurrentLimit(reverseStatorCurrentLimit);
            configNeutralBrakeMode(false);
            configForwardVoltageLimit(nominalVoltage);
            configReverseVoltageLimit(-nominalVoltage);
            configClockwise_Positive();
            setFollowerConfigs(
                    new FollowerConfig(
                            "Launcher Front Right", 16, Rio.CANIVORE, MotorAlignmentValue.Opposed));
        }
    }

    // ---- State Machine ----

    public enum WantedState {
        OFF,
        IDLE_PREP,
        LAUNCH,
    }

    public enum SystemState {
        OFF,
        IDLE_PREP,
        LAUNCH,
    }

    private WantedState wantedState = WantedState.OFF;
    private SystemState systemState = SystemState.OFF;
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
            case OFF -> SystemState.OFF;
            case IDLE_PREP -> SystemState.IDLE_PREP;
            case LAUNCH -> SystemState.LAUNCH;
        };
    }
    /** Applies the states. */
    private void applyStates() {
        double wantedRPM = 0;
        switch (systemState) {
            case OFF:
                stop();
                return;
            case IDLE_PREP:
                wantedRPM = 700;
                break;
            case LAUNCH:
                var params = ShotCalculator.getInstance().getParameters();
                wantedRPM = params.flywheelSpeed();
                break;
        }
        final double finalWantedRPM = wantedRPM;
        setVelocityRPM(() -> finalWantedRPM);
    }

    @Getter private final LauncherConfig config;

    @Getter private LauncherSim sim;

    /**
     * Creates a new Launcher instance.
     *
     * @param config the config
     */
    public Launcher(LauncherConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }
    /** Runs the periodic update. */
    @Override
    public void periodic() {
        systemState = handleStateTransition();
        logBatteryUsage();
        applyStates();
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
    /** Simulation init. */
    public void simulationInit() {
        if (isAttached()) {
            sim = new LauncherSim(RobotSim.leftView, motor);
        }
    }

    class LauncherSim extends RollerSim {
        /**
         * Creates a new LauncherSim instance.
         *
         * @param mech the mech
         * @param motor the motor
         */
        public LauncherSim(Mechanism2d mech, TalonFX motor) {
            super(
                    new RollerConfig(config.getWheelDiameter())
                            .setPosition(config.getLauncherX(), config.getLauncherY())
                            .setGearRatio(config.getGearRatio())
                            .setMount(Robot.getHood().getSim()),
                    mech,
                    motor,
                    config.getName());
        }
    }
}

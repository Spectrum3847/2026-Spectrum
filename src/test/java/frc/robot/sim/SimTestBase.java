package frc.robot.sim;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.configs.OM2026;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.dyeRotor.DyeRotor;
import frc.robot.subsystems.fuelIntake.FuelIntake;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakeExtension.IntakeExtension;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherTower;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.Turret;
import frc.spectrumLib.sim.SimLoop;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;

/** Base test fixture for WPILib simulation integration tests. */
public abstract class SimTestBase {

    /**
     * Helper class to hold the full robot stack components for simulation tests.
     * Resources are owned by the caller and must be closed appropriately.
     */
    public static class RobotStack {
        public final Robot.Config config;
        public final Swerve swerve;
        public final FuelIntake fuelIntake;
        public final IntakeExtension intakeExtension;
        public final DyeRotor dyeRotor;
        public final Launcher launcher;
        public final LauncherTower launcherTower;
        public final Turret turret;
        public final Hood hood;
        public final SuperStructure superStructure;

        /**
         * Creates a robot stack containing the configuration and robot subsystems.
         *
         * @param config the robot configuration
         * @param swerve the swerve drivetrain
         * @param fuelIntake the fuel intake
         * @param intakeExtension the intake extension
         * @param dyeRotor the dye rotor
         * @param launcher the launcher
         * @param launcherTower the launcher tower
         * @param turret the turret
         * @param hood the hood
         * @param superStructure the superstructure
         */
        public RobotStack(
                Robot.Config config,
                Swerve swerve,
                FuelIntake fuelIntake,
                IntakeExtension intakeExtension,
                DyeRotor dyeRotor,
                Launcher launcher,
                LauncherTower launcherTower,
                Turret turret,
                Hood hood,
                SuperStructure superStructure) {
            this.config = config;
            this.swerve = swerve;
            this.fuelIntake = fuelIntake;
            this.intakeExtension = intakeExtension;
            this.dyeRotor = dyeRotor;
            this.launcher = launcher;
            this.launcherTower = launcherTower;
            this.turret = turret;
            this.hood = hood;
            this.superStructure = superStructure;
        }
    }

    /**
     * Creates a full robot stack with OM2026 configuration for simulation testing.
     * The caller is responsible for closing the Swerve resource (use try-with-resources).
     *
     * @return a RobotStack containing all configured subsystems and SuperStructure
     */
    protected static RobotStack createRobotStack() {
        Robot.Config config = new OM2026();

        Swerve swerve = new Swerve(config.swerve);
        FuelIntake fuelIntake = new FuelIntake(config.fuelIntake);
        IntakeExtension intakeExtension = new IntakeExtension(config.intakeExtension);
        DyeRotor dyeRotor = new DyeRotor(config.dyeRotor);
        Launcher launcher = new Launcher(config.launcher);
        LauncherTower launcherTower = new LauncherTower(config.launcherTower);
        Turret turret = new Turret(config.turret);
        Hood hood = new Hood(config.hood);

        SuperStructure superStructure =
                new SuperStructure(
                        swerve,
                        fuelIntake,
                        intakeExtension,
                        dyeRotor,
                        launcher,
                        launcherTower,
                        turret,
                        hood);

        return new RobotStack(
                config,
                swerve,
                fuelIntake,
                intakeExtension,
                dyeRotor,
                launcher,
                launcherTower,
                turret,
                hood,
                superStructure);
    }

    /** Initializes WPILib HAL for simulation. */
    @BeforeAll
    static void initHAL() {
        assert HAL.initialize(50, 0) : "Failed to initialize WPILib HAL for simulation";
    }

    /** Prepares clean simulation state before each test. */
    @BeforeEach
    void setupSim() {
        SimHooks.pauseTiming();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        SimLoop.reset();

        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(false);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
    }

    /** Cleans up simulation state after each test. */
    @AfterEach
    void teardownSim() {
        SimHooks.resumeTiming();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        SimLoop.reset();
    }

    /**
     * Steps simulation time forward and runs the CommandScheduler.
     *
     * @param dtSeconds time step per loop (typically 0.020 for 50 Hz)
     * @param durationSeconds total simulation time to advance
     */
    protected void stepSim(double dtSeconds, double durationSeconds) {
        int steps = (int) Math.round(durationSeconds / dtSeconds);
        for (int i = 0; i < steps; i++) {
            SimHooks.stepTiming(dtSeconds);
            CommandScheduler.getInstance().run();
        }
    }

    /** Sets the simulated DriverStation to Autonomous Enabled. */
    protected void enableAutonomousSim() {
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    /** Sets the simulated DriverStation to Teleop Enabled. */
    protected void enableTeleopSim() {
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }
}

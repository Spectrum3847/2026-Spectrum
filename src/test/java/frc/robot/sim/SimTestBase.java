package frc.robot.sim;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.spectrumLib.sim.SimLoop;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;

/** Base test fixture for WPILib simulation integration tests. */
public abstract class SimTestBase {

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

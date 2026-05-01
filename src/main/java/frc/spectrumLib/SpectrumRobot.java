package frc.spectrumLib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.reflect.Field;
import java.util.ArrayList;
import org.littletonrobotics.junction.LoggedRobot;

/**
 * The base robot class for Spectrum robots. Extends WPILib's TimedRobot and manages a collection of
 * SpectrumSubsystems.
 */
public class SpectrumRobot extends LoggedRobot {

    /** Create a single static instance of all of your subsystems */
    private static final ArrayList<SpectrumSubsystem> subsystems = new ArrayList<>();

    /**
     * Add a subsystem to the global list of subsystems.
     *
     * @param subsystem The subsystem to add.
     */
    public static void add(SpectrumSubsystem subsystem) {
        subsystems.add(subsystem);
    }

    public SpectrumRobot() {
        super();
        DriverStation.silenceJoystickConnectionWarning(true);

        // Adjust loop overrun warning timeout
        try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(0.20);
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
        }
        CommandScheduler.getInstance().setPeriod(0.20);
    }

    /**
     * Set up default commands for all registered subsystems. Should be called during robot
     * initialization.
     */
    protected void setupDefaultCommands() {
        // Setup Default Commands for all subsystems
        subsystems.forEach(SpectrumSubsystem::setupDefaultCommand);
    }

    /**
     * Set up states and triggers for all registered subsystems. Should be called during robot
     * initialization.
     */
    protected void setupStates() {
        // Bind Triggers for all subsystems
        subsystems.forEach(SpectrumSubsystem::setupStates);
    }
}

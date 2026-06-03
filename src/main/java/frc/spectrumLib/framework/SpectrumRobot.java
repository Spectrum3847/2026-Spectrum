package frc.spectrumLib.framework;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.reflect.Field;

/**
 * The base robot class for Spectrum robots. Extends WPILib's TimedRobot and manages a collection of
 * SpectrumSubsystems.
 */
public class SpectrumRobot extends TimedRobot {

    /**
     * Constructs a SpectrumRobot, silencing joystick connection warnings and extending the loop
     * overrun watchdog timeout to 200 ms to accommodate longer periodic loops.
     */
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
}

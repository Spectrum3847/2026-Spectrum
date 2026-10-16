package frc.spectrumLib.framework;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.reflect.Field;

/**
 * Base robot class for Spectrum robots. The constructor silences joystick connection warnings and
 * sets the loop overrun watchdog to {@link #LOOP_OVERRUN_WARNING_SECONDS}.
 */
public class SpectrumRobot extends TimedRobot {

    /**
     * Loop length past which WPILib prints "Loop time of Xs overrun" and a per-section epoch
     * breakdown to the Driver Station. WPILib's own default is 0.5 s.
     *
     * <p>Every overrun below this value stays off the console. In the 2026-09-05 logs 60 to 90
     * percent of enabled loops ran over 25 ms. The {@code Scheduler/*} timers in the wpilog are the
     * record to read instead. Drop this to 0.04 for a diagnostic build that wants the per-section
     * print, keeping in mind the print is itself work the loop has to do.
     */
    public static final double LOOP_OVERRUN_WARNING_SECONDS = 0.20;

    public SpectrumRobot() {
        super();
        DriverStation.silenceJoystickConnectionWarning(true);

        // WPILib exposes no setter for the watchdog timeout, so reach the private field.
        try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(LOOP_OVERRUN_WARNING_SECONDS);
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
        }
        CommandScheduler.getInstance().setPeriod(LOOP_OVERRUN_WARNING_SECONDS);
    }
}

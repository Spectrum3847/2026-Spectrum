package frc.spectrumLib.framework;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.reflect.Field;

/**
 * Base robot class. Silences joystick connection warnings and raises the loop overrun watchdog to
 * 200 ms, so long periodic loops do not trip it.
 */
public class SpectrumRobot extends TimedRobot {

    public SpectrumRobot() {
        super();
        DriverStation.silenceJoystickConnectionWarning(true);

        // The watchdog timeout is a private field with no setter.
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

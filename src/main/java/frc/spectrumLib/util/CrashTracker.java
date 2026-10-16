package frc.spectrumLib.util;

import edu.wpi.first.wpilibj.RobotBase;
import frc.spectrumLib.telemetry.Telemetry;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;

/**
 * Appends caught crashes to a file on the roboRIO that never rolls over, so a fault survives a
 * reboot.
 */
public class CrashTracker {

    private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();
    private static String filePath = "/home/lvuser/crash_tracking.txt";

    /** Appends the run UUID, the marker, the date, and the stack trace to the crash file. */
    public static void logThrowableCrash(Throwable throwable) {
        logMarker("Exception", throwable);
    }

    @SuppressWarnings("unused")
    private static void logMarker(String mark) {
        logMarker(mark, null);
    }

    private static void logMarker(String mark, Throwable nullableException) {
        try (PrintWriter writer = new PrintWriter(new FileWriter(filePath, true))) {
            writer.print(RUN_INSTANCE_UUID.toString());
            writer.print(", ");
            writer.print(mark);
            writer.print(", ");
            writer.print(new Date().toString());

            if (nullableException != null) {
                writer.print(", ");
                nullableException.printStackTrace(writer);
            }

            writer.println();
        } catch (IOException e) {
            if (e instanceof FileNotFoundException) {
                if (RobotBase.isSimulation()) {
                    Telemetry.print(
                            "CrashTracker failed to save crash file to robot: running in simulation mode");
                } else {
                    Telemetry.print(
                            "CrashTracker failed to save crash file to robot: path `/home/lvuser/crash_tracking.txt` not found");
                }
            } else {
                e.printStackTrace();
            }
        }
    }
}

package frc.spectrumLib.telemetry;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**
 * Telemetry and logging utility. Extends DogLog to provide structured logging and console output
 * with priority levels.
 */
public class Telemetry extends DogLog implements Subsystem {

    /**
     * Tracks the most recent set of active alerts for each severity key to avoid duplicate log
     * entries.
     */
    private static final Map<String, String[]> previousAlerts = new HashMap<>();

    /** Named fault conditions that can be surfaced as structured log entries. */
    public enum Fault {
        CAMERA_OFFLINE,
        AUTO_SHOT_TIMEOUT_TRIGGERED,
        BROWNOUT,
    }

    /**
     * Priority levels for printing to the console.
     *
     * <ul>
     *   <li>{@link #NORMAL} — only printed when the global priority is also {@code NORMAL}.
     *   <li>{@link #HIGH} — always printed regardless of the global priority setting.
     * </ul>
     */
    public enum PrintPriority {
        NORMAL,
        HIGH
    }

    /** Minimum priority level a message must have to be written to the console. */
    private static PrintPriority priority = PrintPriority.HIGH;

    /**
     * Creates a Telemetry instance and registers it as a WPILib subsystem so its {@link
     * #periodic()} method is called every loop cycle.
     */
    public Telemetry() {
        super();
        register();
    }

    /** Called every robot loop cycle. Logs any newly active alerts from NetworkTables. */
    @Override
    public void periodic() {
        logAlerts();
    }

    /**
     * Start the telemetry system.
     *
     * @param ntPublish Whether to publish to NetworkTables.
     * @param captureNt Whether to capture NetworkTables entries in the log.
     * @param captureDs Whether to capture SmartDashboard entries in the log.
     * @param captureConsole Whether to capture console output in the log.
     * @param logExtras Whether to log extra data, like PDH currents, CAN usage, radio connection
     *     status, etc.
     * @param tunableOnFMS Whether tunable values should be read from NetworkTables.
     * @param priority The minimum priority level for console output.
     */
    public static void start(
            boolean ntPublish,
            boolean captureDs,
            boolean captureNt,
            boolean captureConsole,
            boolean logExtras,
            boolean tunableOnFMS,
            PrintPriority priority) {
        setPriority(priority);
        Telemetry.setOptions(
                new DogLogOptions()
                        .withNtPublish(ntPublish)
                        .withCaptureDs(captureDs)
                        .withCaptureNt(captureNt)
                        .withCaptureConsole(captureConsole)
                        .withNtTunables(tunableOnFMS)
                        .withLogExtras(logExtras));
        Telemetry.setPdh(new PowerDistribution());
        /* Display the currently running commands on SmartDashboard*/
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    private static void setPriority(PrintPriority priority) {
        Telemetry.priority = priority;
    }

    /**
     * Wraps a command so that its initialization and end are logged to the "Commands" key.
     *
     * @param cmd The command to wrap
     * @return a decorated command that logs lifecycle events and preserves the original name
     */
    public static Command log(Command cmd) {
        return cmd.deadlineFor(
                        Commands.startEnd(
                                () -> log("Commands", "Init: " + cmd.getName()),
                                () -> log("Commands", "End: " + cmd.getName())))
                .ignoringDisable(cmd.runsWhenDisabled())
                .withName(cmd.getName());
    }

    /** Print a statement if they are enabled */
    public static void print(String output, PrintPriority priority) {
        String out = "TIME: " + String.format("%.3f", Timer.getFPGATimestamp()) + " || " + output;
        if (priority == PrintPriority.HIGH || Telemetry.priority == PrintPriority.NORMAL) {
            System.out.println(out);
        }
        log("Prints", out);
    }

    /**
     * Prints a message at {@link PrintPriority#NORMAL} priority. The message is always written to
     * the DogLog "Prints" key but only echoed to stdout when the global priority allows it.
     *
     * @param output The string to print
     */
    public static void print(String output) {
        print(output, PrintPriority.NORMAL);
    }

    /**
     * Reads all active alerts from the SmartDashboard NetworkTable and logs any that are new since
     * the last call under the "Alerts" DogLog key.
     */
    public static void logAlerts() {
        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
        logAlertType(ntInstance, "errors", "ERROR");
        logAlertType(ntInstance, "warnings", "WARNING");
        logAlertType(ntInstance, "infos", "INFO");
    }

    private static void logAlertType(NetworkTableInstance ntInstance, String key, String prefix) {
        String[] alertStrings =
                ntInstance
                        .getTable("SmartDashboard/Alerts")
                        .getEntry(key)
                        .getStringArray(new String[0]);

        String[] previousAlertStrings = previousAlerts.getOrDefault(key, new String[0]);

        for (String alert : alertStrings) {
            if (!Arrays.asList(previousAlertStrings).contains(alert)) {
                log("Alerts", prefix + ": " + alert);
            }
        }

        previousAlerts.put(key, alertStrings);
    }
}

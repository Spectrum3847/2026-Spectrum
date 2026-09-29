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

/** Structured logging and console output, built on DogLog. */
public class Telemetry extends DogLog implements Subsystem {

    /** Last alerts seen per severity, so {@link #logAlerts()} logs each one once. */
    private static final Map<String, String[]> previousAlerts = new HashMap<>();

    public enum Fault {
        CAMERA_OFFLINE,
        AUTO_SHOT_TIMEOUT_TRIGGERED,
        BROWNOUT,
    }

    /**
     * {@code NORMAL} messages print whenever the global priority is {@code NORMAL}, {@code HIGH}
     * messages always print.
     */
    public enum PrintPriority {
        NORMAL,
        HIGH
    }

    private static PrintPriority priority = PrintPriority.HIGH;

    /** Registers as a WPILib subsystem so {@link #periodic()} runs each loop. */
    public Telemetry() {
        super();
        register();
    }

    @Override
    public void periodic() {
        logAlerts();
    }

    /**
     * Sets up logging for the match.
     *
     * @param ntPublish publish to NetworkTables
     * @param captureDs capture SmartDashboard entries in the log
     * @param captureNt capture NetworkTables entries in the log
     * @param captureConsole capture console output in the log
     * @param logExtras PDH currents, CAN usage, and radio status
     * @param tunableOnFMS read tunable values from NetworkTables
     * @param priority lowest priority that still reaches the console
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
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    private static void setPriority(PrintPriority priority) {
        Telemetry.priority = priority;
    }

    /** Logs a command's start and end to the "Commands" key, keeping the command's own name. */
    public static Command log(Command cmd) {
        return cmd.deadlineFor(
                        Commands.startEnd(
                                () -> log("Commands", "Init: " + cmd.getName()),
                                () -> log("Commands", "End: " + cmd.getName())))
                .ignoringDisable(cmd.runsWhenDisabled())
                .withName(cmd.getName());
    }

    public static void print(String output, PrintPriority priority) {
        String out = "TIME: " + String.format("%.3f", Timer.getFPGATimestamp()) + " || " + output;
        if (priority == PrintPriority.HIGH || Telemetry.priority == PrintPriority.NORMAL) {
            System.out.println(out);
        }
        log("Prints", out);
    }

    /** Prints at {@link PrintPriority#NORMAL} and always logs to the "Prints" key. */
    public static void print(String output) {
        print(output, PrintPriority.NORMAL);
    }

    /** Logs any SmartDashboard alert that has appeared since the last call, and remembers it. */
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

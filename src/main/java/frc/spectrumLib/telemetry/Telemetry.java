package frc.spectrumLib.telemetry;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.spectrumLib.framework.RobotLoop;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**
 * Telemetry and logging utility. Extends DogLog to provide structured logging and console output
 * with priority levels.
 *
 * <p>Three tiers:
 *
 * <ul>
 *   <li>{@link #log} writes to the wpilog only. DogLog skips a record when the value has not
 *       changed, so a boolean or a state string costs nothing while it is steady; a double that
 *       moves every loop costs a record every loop.
 *   <li>{@link #logDash} also publishes the value to NetworkTables at 10 Hz, for the Driver Station
 *       dashboard and the robot app. Use it for exactly the keys a dashboard shows; the whole log
 *       is not mirrored to NetworkTables unless the mirror switch is on (see {@link #start}).
 *   <li>{@link #slowLogThisLoop()} is true every fifth loop. Wrap logs that do not need loop-rate
 *       resolution (currents, temperatures, vision status) in it.
 * </ul>
 *
 * <p>On 2026-09-05 a roboRIO ran at 92 to 95 percent CPU on a 30 ms loop while every record was
 * mirrored to NetworkTables and flushed every 20 ms, which cost a core. These tiers cut the volume
 * to what is actually read.
 */
public class Telemetry extends DogLog implements Subsystem {

    /** Most recent set of active alerts per severity key, to avoid duplicate log entries. */
    private static final Map<String, String[]> previousAlerts = new HashMap<>();

    /** Live subscribers for each alert severity, created on first use. */
    private static final Map<String, StringArraySubscriber> alertSubscribers = new HashMap<>();

    public enum Fault {
        CAMERA_OFFLINE,
        AUTO_SHOT_TIMEOUT_TRIGGERED,
        BROWNOUT,
    }

    /**
     * Priority levels for printing to the console.
     *
     * <ul>
     *   <li>{@link #NORMAL} prints only when the global priority is also {@code NORMAL}.
     *   <li>{@link #HIGH} always prints, whatever the global priority is.
     * </ul>
     */
    public enum PrintPriority {
        NORMAL,
        HIGH
    }

    /** Global console filter: at {@link PrintPriority#HIGH} only HIGH messages print. */
    private static PrintPriority priority = PrintPriority.HIGH;

    /**
     * SmartDashboard key of the switch that mirrors every log entry to NetworkTables, for live
     * AdvantageScope sessions in the shop. Off by default; always off when the FMS is attached.
     */
    public static final String NT_MIRROR_SWITCH_KEY = "Telemetry/MirrorLogsToNT";

    private static BooleanEntry ntMirrorSwitch;

    /**
     * Cached switch position, read once per loop on the main thread, consumed on the log thread.
     */
    private static volatile boolean ntMirrorEnabled = false;

    /** Loops between slow-tier publishes: every fifth 20 ms loop is 10 Hz. */
    public static final int SLOW_LOG_EVERY_LOOPS = 5;

    /**
     * True on the loops the slow telemetry tier publishes on. Every caller in the same loop agrees,
     * so related values land on the same records.
     */
    public static boolean slowLogThisLoop() {
        return RobotLoop.every(SLOW_LOG_EVERY_LOOPS);
    }

    public Telemetry() {
        super();
        register();
    }

    @Override
    public void periodic() {
        refreshNtMirrorSwitch();
        logAlerts();
    }

    private static void refreshNtMirrorSwitch() {
        if (ntMirrorSwitch != null) {
            ntMirrorEnabled = ntMirrorSwitch.get();
        }
    }

    /**
     * Entries the log queue may hold before it starts dropping them.
     *
     * <p>The queue absorbs the gap between a robot thread that produces records in bursts and a log
     * thread that has to get scheduled to drain them. DogLog's default of 1000 was about a third of
     * a second at the 3000 records per second these logs were running at; 5000 covers a stall of
     * about one and a half seconds for roughly 400 KB of heap in the worst case, and only while a
     * burst is actually queued.
     *
     * <p>Headroom, not a fix. The queue filled on 2026-09-05 because the log thread was starved of
     * CPU, not because the disk was slow; cutting what is logged is the real lever.
     */
    private static final int LOG_ENTRY_QUEUE_CAPACITY = 5000;

    /**
     * Starts the telemetry system.
     *
     * <p>Values a dashboard needs are published to NetworkTables individually through {@link
     * #logDash}; everything else stays in the wpilog. Mirroring every entry is opt-in through the
     * {@value #NT_MIRROR_SWITCH_KEY} switch, and is forced off whenever the FMS is attached
     * regardless of the switch.
     *
     * @param ntMirrorDefault initial position of the mirror-everything switch. {@code false} for
     *     the robot; flip it in Elastic when AdvantageScope needs the full live stream
     */
    public static void start(
            boolean ntMirrorDefault,
            boolean captureDs,
            boolean captureNt,
            boolean captureConsole,
            boolean logExtras,
            boolean tunableOnFMS,
            PrintPriority priority) {
        Telemetry.priority = priority;

        ntMirrorEnabled = ntMirrorDefault;
        ntMirrorSwitch =
                NetworkTableInstance.getDefault()
                        .getTable("SmartDashboard")
                        .getBooleanTopic(NT_MIRROR_SWITCH_KEY)
                        .getEntry(ntMirrorDefault);
        ntMirrorSwitch.set(ntMirrorDefault);

        Telemetry.setOptions(
                new DogLogOptions()
                        .withNtPublish(Telemetry::mirrorLogsToNt)
                        .withCaptureDs(captureDs)
                        .withCaptureNt(captureNt)
                        .withCaptureConsole(captureConsole)
                        .withNtTunables(tunableOnFMS)
                        .withLogEntryQueueCapacity(LOG_ENTRY_QUEUE_CAPACITY)
                        .withLogExtras(logExtras));
        Telemetry.setPdh(new PowerDistribution());
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    /**
     * Whether DogLog should mirror log entries to NetworkTables right now. Evaluated by the log
     * thread for every record, so it reads two cached booleans and nothing else.
     */
    private static boolean mirrorLogsToNt() {
        return ntMirrorEnabled && !DriverStation.isFMSAttached();
    }

    //
    // logDash writes the record on every call and publishes it to NetworkTables on slow-tier
    // loops, so a value that moves every loop reaches the dashboard at 10 Hz and the log at loop
    // rate. The NetworkTables topic only changes when a slow-tier call publishes, so a boolean or a
    // string that flips between ticks still arrives within 100 ms. logDashAlways publishes on every
    // call, for values logged on their own slower cadence that might never land on a tick.

    public static void logDash(String key, double value) {
        if (slowLogThisLoop()) {
            forceNt.log(key, value);
        } else {
            log(key, value);
        }
    }

    public static void logDash(String key, double value, String unit) {
        if (slowLogThisLoop()) {
            forceNt.log(key, value, unit);
        } else {
            log(key, value, unit);
        }
    }

    public static void logDash(String key, boolean value) {
        if (slowLogThisLoop()) {
            forceNt.log(key, value);
        } else {
            log(key, value);
        }
    }

    public static void logDash(String key, String value) {
        if (slowLogThisLoop()) {
            forceNt.log(key, value);
        } else {
            log(key, value);
        }
    }

    public static void logDash(String key, long value) {
        if (slowLogThisLoop()) {
            forceNt.log(key, value);
        } else {
            log(key, value);
        }
    }

    /**
     * Logs a double and publishes it to the dashboard on this very call. For values logged on their
     * own cadence (a timer, a state change) that must not wait for a slow-tier loop.
     */
    public static void logDashAlways(String key, double value) {
        forceNt.log(key, value);
    }

    public static void logDashAlways(String key, double value, String unit) {
        forceNt.log(key, value, unit);
    }

    public static void logDashAlways(String key, boolean value) {
        forceNt.log(key, value);
    }

    public static void logDashAlways(String key, String value) {
        forceNt.log(key, value);
    }

    public static void logDashAlways(String key, long value) {
        forceNt.log(key, value);
    }

    //
    // A state is logged on the loop it changes and not while it holds. The wpilog ends up the same
    // as logging every loop, because DogLog drops a value equal to the last one, but every-loop
    // calls still cost a queue entry each for the log thread to throw away.

    /**
     * Last state logged per key, so {@link #logState} can skip unchanged ones. Main thread only.
     */
    private static final Map<String, Enum<?>> lastLoggedStates = new HashMap<>();

    /**
     * Logs a state machine's state when it changes: the exact loop of every transition, nothing
     * while it holds.
     */
    public static void logState(String key, Enum<?> state) {
        if (lastLoggedStates.put(key, state) != state) {
            log(key, state.name());
        }
    }

    /**
     * {@link #logState}, also published to the dashboard on every change (a change can land between
     * slow-tier ticks, so {@link #logDash} could miss it).
     */
    public static void logStateDash(String key, Enum<?> state) {
        if (lastLoggedStates.put(key, state) != state) {
            logDashAlways(key, state.name());
        }
    }

    /** Start times of open {@link #time} spans, in FPGA microseconds. */
    private static final Map<String, Long> epochStartMicros = new HashMap<>();

    /**
     * Starts a timed span. Pair with {@link #timeEnd(String)} on the same key.
     *
     * <p>Replaces DogLog's timer so the result reaches the dashboard: the {@code Scheduler/*} spans
     * are on the Elastic layout and are the primary record of loop time.
     */
    public static void time(String key) {
        epochStartMicros.put(key, RobotController.getFPGATime());
    }

    /**
     * Ends a timed span and logs its length in seconds, matching the units DogLog's timer used, so
     * existing analysis of the {@code Scheduler/*} keys keeps working.
     */
    public static void timeEnd(String key) {
        Long start = epochStartMicros.remove(key);
        if (start == null) {
            return;
        }
        logDash(key, (RobotController.getFPGATime() - start) / 1_000_000.0, "seconds");
    }

    /**
     * Wraps a command so that its initialization and end are logged to the "Commands" key. The
     * wrapper keeps the original name.
     */
    public static Command log(Command cmd) {
        return cmd.deadlineFor(
                        Commands.startEnd(
                                () -> log("Commands", "Init: " + cmd.getName()),
                                () -> log("Commands", "End: " + cmd.getName())))
                .ignoringDisable(cmd.runsWhenDisabled())
                .withName(cmd.getName());
    }

    /** Echoes to the console when the priority allows, and always logs to the "Prints" key. */
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
     */
    public static void print(String output) {
        print(output, PrintPriority.NORMAL);
    }

    /**
     * Logs any alert newly published under {@code SmartDashboard/Alerts} (errors, warnings, infos)
     * to the "Alerts" DogLog key.
     *
     * <p>Reads the subscribers' change queues rather than the current arrays, so a loop on which
     * nothing changed does no NetworkTables read and allocates nothing.
     */
    public static void logAlerts() {
        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
        logAlertType(ntInstance, "errors", "ERROR");
        logAlertType(ntInstance, "warnings", "WARNING");
        logAlertType(ntInstance, "infos", "INFO");
    }

    private static void logAlertType(NetworkTableInstance ntInstance, String key, String prefix) {
        StringArraySubscriber subscriber =
                alertSubscribers.computeIfAbsent(
                        key,
                        k ->
                                ntInstance
                                        .getTable("SmartDashboard/Alerts")
                                        .getStringArrayTopic(k)
                                        .subscribe(new String[0]));

        for (String[] alertStrings : subscriber.readQueueValues()) {
            String[] previousAlertStrings = previousAlerts.getOrDefault(key, new String[0]);
            for (String alert : alertStrings) {
                if (!Arrays.asList(previousAlertStrings).contains(alert)) {
                    log("Alerts", prefix + ": " + alert);
                }
            }
            previousAlerts.put(key, alertStrings);
        }
    }
}

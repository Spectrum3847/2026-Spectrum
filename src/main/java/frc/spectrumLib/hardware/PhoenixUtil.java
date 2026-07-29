package frc.spectrumLib.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

/**
 * Shared helpers for Phoenix 6 devices: configurator retries and batched status-signal refresh.
 *
 * <h2>Signal tiers</h2>
 *
 * Devices register their {@link BaseStatusSignal}s in one of two tiers. The fast tier is for
 * signals in the control path (position, velocity) and refreshes every loop. The slow tier is for
 * telemetry-only signals (currents, applied voltage, device temperature) and refreshes every
 * {@value #SLOW_TIER_PERIOD_LOOPS} loops. {@link #refreshAll()} drives both from one place, so
 * every value read during a loop shares a single refresh instant.
 *
 * <p>Registering a signal does not set its update frequency — the owning device still calls {@link
 * BaseStatusSignal#setUpdateFrequencyForAll} (see {@link #FAST_SIGNAL_HZ}, {@link #SLOW_SIGNAL_HZ},
 * {@link #TEMP_SIGNAL_HZ}) followed by {@code optimizeBusUtilization()}. The frequency controls how
 * often the device puts the frame on the bus; the tier controls how often the locally cached value
 * is re-read.
 */
public final class PhoenixUtil {

    /** Update frequency for signals used by the control path (position, velocity). */
    public static final double FAST_SIGNAL_HZ = 250.0;

    /** Update frequency for telemetry-only signals (currents, applied voltage). */
    public static final double SLOW_SIGNAL_HZ = 10.0;

    /** Update frequency for device temperature. */
    public static final double TEMP_SIGNAL_HZ = 4.0;

    /**
     * Scheduler loops between slow-tier refreshes. At the default 50 Hz robot period this works out
     * to 10 Hz, matching {@link #SLOW_SIGNAL_HZ}.
     */
    public static final int SLOW_TIER_PERIOD_LOOPS = 5;

    /** Signals refreshed every loop, grouped by CAN bus name. */
    private static final Map<String, List<BaseStatusSignal>> fastSignals = new LinkedHashMap<>();

    /** Signals refreshed every {@link #SLOW_TIER_PERIOD_LOOPS} loops, grouped by CAN bus name. */
    private static final Map<String, List<BaseStatusSignal>> slowSignals = new LinkedHashMap<>();

    /** Flattened views of the maps above, rebuilt whenever a new signal is registered. */
    private static BaseStatusSignal[][] fastGroups = null;

    private static BaseStatusSignal[][] slowGroups = null;

    private static int refreshCounter = 0;

    private PhoenixUtil() {}

    // ── Config retries ─────────────────────────────────────────────────────────

    /**
     * Runs a configurator call until it reports success, up to {@code maxAttempts} times. Reports
     * an error to the Driver Station if every attempt fails.
     *
     * @param description what is being configured, used in the failure message (e.g. {@code "Turret
     *     leader config"})
     * @param maxAttempts maximum number of attempts before giving up
     * @param command the configurator call to run, e.g. {@code () ->
     *     talon.getConfigurator().apply(config, 0.25)}
     * @return the final {@link StatusCode} — {@link StatusCode#isOK()} is {@code false} if every
     *     attempt failed
     */
    public static StatusCode tryUntilOk(
            String description, int maxAttempts, Supplier<StatusCode> command) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int attempt = 0; attempt < maxAttempts; attempt++) {
            status = command.get();
            if (status.isOK()) {
                return status;
            }
        }
        DriverStation.reportError(
                "[PhoenixUtil] "
                        + description
                        + " failed after "
                        + maxAttempts
                        + " attempts: "
                        + status,
                false);
        return status;
    }

    // ── Batched signal refresh ─────────────────────────────────────────────────

    /**
     * Registers control-path signals for refresh on every loop.
     *
     * @param canbus CAN bus name the signals live on, used to group refresh calls per bus
     * @param signals the signals to register; {@code null} entries are ignored
     */
    public static void registerSignals(String canbus, BaseStatusSignal... signals) {
        if (register(fastSignals, canbus, signals)) {
            fastGroups = null;
        }
    }

    /**
     * Registers telemetry-only signals for throttled refresh, every {@link #SLOW_TIER_PERIOD_LOOPS}
     * loops.
     *
     * @param canbus CAN bus name the signals live on, used to group refresh calls per bus
     * @param signals the signals to register; {@code null} entries are ignored
     */
    public static void registerSlowSignals(String canbus, BaseStatusSignal... signals) {
        if (register(slowSignals, canbus, signals)) {
            slowGroups = null;
        }
    }

    /**
     * Refreshes every registered signal. Call this once per loop, before any code reads a signal
     * value — in practice at the top of {@code robotPeriodic()}, ahead of the command scheduler.
     */
    public static void refreshAll() {
        if (fastGroups == null) {
            fastGroups = flatten(fastSignals);
        }
        for (BaseStatusSignal[] group : fastGroups) {
            BaseStatusSignal.refreshAll(group);
        }

        if (++refreshCounter >= SLOW_TIER_PERIOD_LOOPS) {
            refreshCounter = 0;
            if (slowGroups == null) {
                slowGroups = flatten(slowSignals);
            }
            for (BaseStatusSignal[] group : slowGroups) {
                BaseStatusSignal.refreshAll(group);
            }
        }
    }

    /**
     * @return the number of signals registered in the fast (every-loop) tier
     */
    public static int getFastSignalCount() {
        return countSignals(fastSignals);
    }

    /**
     * @return the number of signals registered in the slow (throttled) tier
     */
    public static int getSlowSignalCount() {
        return countSignals(slowSignals);
    }

    private static boolean register(
            Map<String, List<BaseStatusSignal>> target,
            String canbus,
            BaseStatusSignal... signals) {
        boolean added = false;
        List<BaseStatusSignal> busSignals =
                target.computeIfAbsent(canbus, key -> new ArrayList<>());
        for (BaseStatusSignal signal : signals) {
            if (signal != null) {
                busSignals.add(signal);
                added = true;
            }
        }
        return added;
    }

    private static BaseStatusSignal[][] flatten(Map<String, List<BaseStatusSignal>> source) {
        return source.values().stream()
                .filter(list -> !list.isEmpty())
                .map(list -> list.toArray(new BaseStatusSignal[0]))
                .toArray(BaseStatusSignal[][]::new);
    }

    private static int countSignals(Map<String, List<BaseStatusSignal>> source) {
        return source.values().stream().mapToInt(List::size).sum();
    }
}

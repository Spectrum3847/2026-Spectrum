package frc.spectrumLib.hardware;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleFunction;

/**
 * A cap on how long boot may spend on CAN configuration that is not working.
 *
 * <p>Every Phoenix configuration call blocks until the device answers or the timeout expires. With
 * a healthy bus that costs nothing. With a dead one, each of the roughly two dozen CANivore devices
 * is configured with several such calls, plus retry loops on top, and every one waits out its full
 * timeout against a device that is not there.
 *
 * <p>On 2026-09-19 at Chezy the CANivore bus went down mid-match on a cut wire. After a Driver
 * Station restart, robot init took 64.0 s against 16.9 s the match before, and the roboRIO then sat
 * at 93-99 % CPU with 58-77 % of loops over 25 ms. It had come back, and the match was over before
 * anyone could tell the difference.
 *
 * <p>Callers run their configuration through {@link #run} and consult {@link #exhausted()} before
 * retrying or before making calls that are merely nice to have. Once the time lost to failed
 * configuration passes {@link #BUDGET_SECONDS}, retries stop, optional calls are skipped, and an
 * alert names the mechanism. Boot then finishes with a dead bus in roughly the time it takes to
 * fail once per device instead of ten times.
 *
 * <p>Open question from the same match, still unanswered. This bounds the boot, but the load that
 * followed is a separate thing. With the bus already dead, CPU sat at a normal 40 to 50 percent for
 * 20 s after init, then went to 95 to 100 percent at exactly 30.0 s after init completed and stayed
 * there. Nothing in the Java loop changed at that moment (vision and the scheduler were each under
 * 20 ms) and the DataLog writer thread was starved too, so the load is native or in a background
 * service. To find it, unplug the CANivore, boot, watch {@code System/CpuPercent} at plus 30 s,
 * then disable the Phoenix diagnostics server with {@code
 * Unmanaged.setPhoenixDiagnosticsStartTime(-1)} and the SignalLogger to see which one it is. Log:
 * {@code logs/matches/FRC_20260919_034444.wpilog}.
 *
 * <p>The cap is deliberately cause-agnostic. A cut wire, a powered-down bus, a wrong bus name and a
 * missing CANivore all present the same way, and the right response to each is the same: stop
 * waiting and come up.
 */
public final class CanConfigBudget {

    private CanConfigBudget() {}

    /**
     * Cumulative seconds of failed configuration allowed before retries are abandoned. Big enough
     * that one slow device, or a few first-call timeouts on a healthy bus, never reaches it. Small
     * enough that a dead bus costs seconds rather than most of a minute.
     */
    public static final double BUDGET_SECONDS = 3.0;

    /**
     * Timeout for a single boot configuration call, in seconds. The same as the Phoenix default on
     * purpose: a shorter individual wait would risk failing on a healthy but busy bus, and the fix
     * here is to stop repeating the wait once nothing is answering.
     */
    public static final double CALL_TIMEOUT_SECONDS = 0.050;

    public static final int MAX_ATTEMPTS = 10;

    private static double spentSeconds = 0;
    private static int failedCalls = 0;

    private static final Alert exhaustedAlert = new Alert("", AlertType.kError);

    /** Callers check this before retrying, and before calls that are only nice to have. */
    public static boolean exhausted() {
        return spentSeconds >= BUDGET_SECONDS;
    }

    /**
     * @return {@link #MAX_ATTEMPTS} while the budget holds, otherwise 1
     */
    public static int maxAttempts() {
        return exhausted() ? 1 : MAX_ATTEMPTS;
    }

    /**
     * Runs one configuration call with the bounded timeout, charging its cost to the budget only if
     * it fails. A successful call costs nothing however long it took, because time spent talking to
     * a device that is really there is time well spent, and only failures repeat.
     *
     * @param name the device or mechanism being configured, for the alert text
     * @param call the configuration call, taking the timeout in seconds
     */
    public static StatusCode run(String name, DoubleFunction<StatusCode> call) {
        double start = Timer.getFPGATimestamp();
        StatusCode result = call.apply(CALL_TIMEOUT_SECONDS);
        if (!result.isOK()) {
            boolean wasExhausted = exhausted();
            spentSeconds += Timer.getFPGATimestamp() - start;
            failedCalls++;
            if (!wasExhausted && exhausted()) {
                exhaustedAlert.setText(
                        String.format(
                                "CAN config budget spent: %.1f s lost over %d failed device"
                                        + " configs (first noticed at %s). Retries are now off so"
                                        + " boot can finish -- expect mechanisms to be"
                                        + " unconfigured. Check the CAN bus wiring and power.",
                                spentSeconds, failedCalls, name));
                exhaustedAlert.set(true);
            }
        }
        return result;
    }

    /**
     * Seconds charged so far. Logged so a boot that nearly reached the cap shows up before the boot
     * that reaches it.
     */
    public static double getSpentSeconds() {
        return spentSeconds;
    }

    public static int getFailedCalls() {
        return failedCalls;
    }
}

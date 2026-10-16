package frc.spectrumLib.framework;

/**
 * Counts robot loops so work can run once per loop, or every Nth loop, without a timer.
 *
 * <p>Main robot thread only. Nothing advances the count in a unit test, so a once-per-loop guard
 * there refreshes exactly once.
 */
public final class RobotLoop {
    private static long count = 0;

    private RobotLoop() {}

    /**
     * Advances the count. {@link frc.robot.Robot#robotPeriodic()} calls this first, so the count
     * starts at 0 inside the first loop.
     */
    public static void next() {
        count++;
    }

    public static long count() {
        return count;
    }

    /**
     * True on every {@code n}th loop, so a 20 ms loop with {@code n = 5} is 10 Hz. Every caller in
     * a loop gets the same answer, which keeps related values on the same record.
     *
     * @param n loops between true results, where 1 or less means every loop
     */
    public static boolean every(int n) {
        return n <= 1 || count % n == 0;
    }
}

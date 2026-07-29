package frc.spectrumLib.telemetry;

import java.util.Arrays;

/**
 * Rolling-window statistics for loop durations, reporting p50, p95 and max alongside the latest
 * sample.
 *
 * <p>Note that {@code SpectrumRobot} raises the watchdog timeout to 200 ms, so WPILib's built-in
 * loop-overrun warning does not fire for an overrun that still misses the 20 ms period several
 * times over.
 */
public class LoopTimingStats {

    private final String logKey;
    private final double[] samples;
    private int nextIndex = 0;
    private int sampleCount = 0;
    private double latestMS = 0.0;

    /** Scratch buffer for percentile sorting, so {@link #log()} does not allocate each loop. */
    private final double[] sortScratch;

    /**
     * Creates a rolling window of loop-duration samples.
     *
     * @param logKey telemetry key prefix, e.g. {@code "Scheduler/robotPeriodic"}
     * @param windowSamples number of samples to retain; at a 50 Hz loop, 250 is five seconds
     */
    public LoopTimingStats(String logKey, int windowSamples) {
        this.logKey = logKey;
        this.samples = new double[windowSamples];
        this.sortScratch = new double[windowSamples];
    }

    /**
     * Records one loop duration.
     *
     * @param durationMS the duration of the loop just completed, in milliseconds
     */
    public void update(double durationMS) {
        latestMS = durationMS;
        samples[nextIndex] = durationMS;
        nextIndex = (nextIndex + 1) % samples.length;
        if (sampleCount < samples.length) {
            sampleCount++;
        }
    }

    /**
     * @return the most recently recorded duration, in milliseconds
     */
    public double getLatestMS() {
        return latestMS;
    }

    /**
     * @return the median duration over the window, in milliseconds
     */
    public double getP50MS() {
        return percentile(0.50);
    }

    /**
     * @return the 95th-percentile duration over the window, in milliseconds
     */
    public double getP95MS() {
        return percentile(0.95);
    }

    /**
     * @return the worst duration over the window, in milliseconds
     */
    public double getMaxMS() {
        return percentile(1.0);
    }

    /** Logs the latest sample plus p50, p95 and max for the current window. */
    public void log() {
        Telemetry.log(logKey + "/LatestMS", latestMS, "ms");
        Telemetry.log(logKey + "/P50MS", getP50MS(), "ms");
        Telemetry.log(logKey + "/P95MS", getP95MS(), "ms");
        Telemetry.log(logKey + "/MaxMS", getMaxMS(), "ms");
    }

    private double percentile(double percentile) {
        if (sampleCount == 0) {
            return 0.0;
        }
        System.arraycopy(samples, 0, sortScratch, 0, sampleCount);
        Arrays.sort(sortScratch, 0, sampleCount);
        int index = (int) Math.ceil(percentile * sampleCount) - 1;
        return sortScratch[Math.max(0, Math.min(index, sampleCount - 1))];
    }
}

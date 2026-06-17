// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.spectrumLib.telemetry;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.HashMap;
import java.util.Map;
import lombok.Getter;
import lombok.Setter;

/**
 * Tracks and logs current draw, power, cumulative energy consumption, peak current/power, and
 * sustained current windows useful for breaker thermal analysis.
 */
public class BatteryLogger {
    /** Duration of one robot loop in seconds, used to convert power (W) to energy (J). */
    private static final double LOOP_PERIOD_SECS = 0.02;

    /** Rolling-window sample counts. */
    private static final int WINDOW_20S_SAMPLES = (int) (20.0 / LOOP_PERIOD_SECS);
    private static final int WINDOW_45S_SAMPLES = (int) (45.0 / LOOP_PERIOD_SECS);
    private static final int WINDOW_60S_SAMPLES = (int) (60.0 / LOOP_PERIOD_SECS);

    /** When {@code false} all methods are no-ops, allowing the logger to be disabled at runtime. */
    @Setter private boolean enabled = false;

    /**
     * Running total of current draw accumulated since the last {@link #logPower()} call, in amps.
     */
    @Getter private double totalCurrent = 0.0;

    /** Running total of power accumulated since the last {@link #logPower()} call, in watts. */
    @Getter private double totalPower = 0.0;

    /** Cumulative energy consumed over the entire enabled session, in joules. */
    @Getter private double totalEnergy = 0.0;

    /** Peak instantaneous current observed since reset, in amps. */
    @Getter private double maxCurrent = 0.0;

    /** Peak instantaneous power observed since reset, in watts. */
    @Getter private double maxPower = 0.0;

    /** Highest rolling-average current over a 20-second window, in amps. */
    @Getter private double max20sCurrentA = 0.0;

    /** Highest rolling-average current over a 45-second window, in amps. */
    @Getter private double max45sCurrentA = 0.0;

    /** Highest rolling-average current over a 60-second window, in amps. */
    @Getter private double max60sCurrentA = 0.0;

    /** Battery terminal voltage used to convert current to power, in volts. */
    @Setter private double batteryVoltage = 12.6;

    /** Estimated current drawn by the RoboRIO itself, in amps. */
    @Setter private double rioCurrent = 0.0;

    private final Map<String, Double> subsystemCurrents = new HashMap<>();
    private final Map<String, Double> subsystemPowers = new HashMap<>();
    private final Map<String, Double> subsystemEnergies = new HashMap<>();
    private final Map<String, Double> maxSubsystemCurrents = new HashMap<>();

    /** Rolling-current history buffers. */
    private final Deque<Double> currentHistory20s = new ArrayDeque<>();

    private final Deque<Double> currentHistory45s = new ArrayDeque<>();
    private final Deque<Double> currentHistory60s = new ArrayDeque<>();

    /** Running sums for efficient rolling-average calculations. */
    private double rollingCurrent20s = 0.0;

    private double rollingCurrent45s = 0.0;
    private double rollingCurrent60s = 0.0;

    /**
     * Records the current draw for a named subsystem channel and accumulates it into the running
     * totals. The {@code key} may use "/" or "-" as separators; parent keys are automatically
     * aggregated.
     *
     * @param key Hierarchical name for the current consumer (e.g. {@code "Drive/FrontLeft"})
     * @param amps One or more current readings in amps; absolute values are summed
     */
    public void reportCurrentUsage(String key, double... amps) {
        if (!enabled) {
            return;
        }

        double totalAmps = 0.0;
        for (double amp : amps) {
            totalAmps += Math.abs(amp);
        }

        double power = totalAmps * batteryVoltage;
        double energy = power * LOOP_PERIOD_SECS;

        totalCurrent += totalAmps;
        totalPower += power;
        totalEnergy += energy;

        subsystemCurrents.put(key, totalAmps);
        subsystemPowers.put(key, power);
        subsystemEnergies.merge(key, energy, Double::sum);

        maxSubsystemCurrents.merge(key, totalAmps, Math::max);

        String[] keys = key.split("/|-");
        if (keys.length < 2) {
            return;
        }

        String subkey = "";
        for (int i = 0; i < keys.length - 1; i++) {
            subkey += keys[i];

            if (i < keys.length - 2) {
                subkey += "/";
            }

            subsystemCurrents.merge(subkey, totalAmps, Double::sum);
            subsystemPowers.merge(subkey, power, Double::sum);
            subsystemEnergies.merge(subkey, energy, Double::sum);
            maxSubsystemCurrents.merge(subkey, totalAmps, Math::max);
        }
    }

    /**
     * Appends control-overhead current consumers (roboRIO, CANcoders, Pigeon, CANivore, radio),
     * updates rolling-current windows, logs all telemetry, and resets per-loop accumulators.
     */
    public void logPower() {
        if (!enabled) {
            return;
        }

        // Controls overhead is added here so it is included in total current.
        reportCurrentUsage("Controls/roboRIO", rioCurrent);
        reportCurrentUsage("Controls/CANcoders", 0.05 * 4);
        reportCurrentUsage("Controls/Pigeon", 0.04);
        reportCurrentUsage("Controls/CANivore", 0.03);
        reportCurrentUsage("Controls/Radio", 0.5);

        // Track instantaneous peaks.
        maxCurrent = Math.max(maxCurrent, totalCurrent);
        maxPower = Math.max(maxPower, totalPower);

        // Update rolling windows and sustained-current peaks.
        updateRollingWindows(totalCurrent);

        // Total metrics.
        Telemetry.log("BatteryLogger/Current", totalCurrent, "amps");
        Telemetry.log("BatteryLogger/Power", totalPower, "watts");
        Telemetry.log("BatteryLogger/Energy", joulesToWattHours(totalEnergy), "wh");
        Telemetry.log("BatteryLogger/BatteryVoltage", batteryVoltage, "volts");

        // Peak metrics.
        Telemetry.log("BatteryLogger/MaxCurrent", maxCurrent, "amps");
        Telemetry.log("BatteryLogger/MaxPower", maxPower, "watts");

        // Sustained-current metrics.
        Telemetry.log("BatteryLogger/Max20sCurrent", max20sCurrentA, "amps");
        Telemetry.log("BatteryLogger/Max45sCurrent", max45sCurrentA, "amps");
        Telemetry.log("BatteryLogger/Max60sCurrent", max60sCurrentA, "amps");

        // Per-subsystem current.
        for (var entry : subsystemCurrents.entrySet()) {
            Telemetry.log("BatteryLogger/Current/" + entry.getKey(), entry.getValue(), "amps");
            subsystemCurrents.put(entry.getKey(), 0.0);
        }

        // Per-subsystem max current.
        for (var entry : maxSubsystemCurrents.entrySet()) {
            Telemetry.log("BatteryLogger/MaxCurrent/" + entry.getKey(), entry.getValue(), "amps");
        }

        // Per-subsystem power.
        for (var entry : subsystemPowers.entrySet()) {
            Telemetry.log("BatteryLogger/Power/" + entry.getKey(), entry.getValue(), "watts");
            subsystemPowers.put(entry.getKey(), 0.0);
        }

        // Per-subsystem cumulative energy.
        for (var entry : subsystemEnergies.entrySet()) {
            Telemetry.log(
                    "BatteryLogger/Energy/" + entry.getKey(),
                    joulesToWattHours(entry.getValue()),
                    "wh");
        }

        // Reset per-loop totals.
        totalCurrent = 0.0;
        totalPower = 0.0;
    }

    private void updateRollingWindows(double currentSample) {
        rollingCurrent20s =
                updateRollingWindow(
                        currentHistory20s, rollingCurrent20s, currentSample, WINDOW_20S_SAMPLES);

        rollingCurrent45s =
                updateRollingWindow(
                        currentHistory45s, rollingCurrent45s, currentSample, WINDOW_45S_SAMPLES);

        rollingCurrent60s =
                updateRollingWindow(
                        currentHistory60s, rollingCurrent60s, currentSample, WINDOW_60S_SAMPLES);

        double avg20s = rollingCurrent20s / currentHistory20s.size();
        double avg45s = rollingCurrent45s / currentHistory45s.size();
        double avg60s = rollingCurrent60s / currentHistory60s.size();

        max20sCurrentA = Math.max(max20sCurrentA, avg20s);
        max45sCurrentA = Math.max(max45sCurrentA, avg45s);
        max60sCurrentA = Math.max(max60sCurrentA, avg60s);
    }

    private double updateRollingWindow(
            Deque<Double> history, double runningSum, double sample, int maxSamples) {

        history.addLast(sample);
        runningSum += sample;

        if (history.size() > maxSamples) {
            runningSum -= history.removeFirst();
        }

        return runningSum;
    }

    /** Resets all peak and rolling-window metrics. */
    public void resetMaximums() {
        maxCurrent = 0.0;
        maxPower = 0.0;

        max20sCurrentA = 0.0;
        max45sCurrentA = 0.0;
        max60sCurrentA = 0.0;

        rollingCurrent20s = 0.0;
        rollingCurrent45s = 0.0;
        rollingCurrent60s = 0.0;

        currentHistory20s.clear();
        currentHistory45s.clear();
        currentHistory60s.clear();

        maxSubsystemCurrents.clear();
    }

    private double joulesToWattHours(double joules) {
        return joules / 3600.0;
    }
}

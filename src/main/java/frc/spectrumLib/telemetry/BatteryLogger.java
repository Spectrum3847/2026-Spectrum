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
 * Tracks current, power, energy, peaks, and rolling current windows per subsystem, for spotting
 * circuits that will trip a breaker.
 */
public class BatteryLogger {
    /**
     * One robot loop in seconds, used to turn watts into joules. Must match the scheduler period.
     */
    private static final double LOOP_PERIOD_SECS = 0.02;

    private static final int WINDOW_20S_SAMPLES = (int) (20.0 / LOOP_PERIOD_SECS);
    private static final int WINDOW_45S_SAMPLES = (int) (45.0 / LOOP_PERIOD_SECS);
    private static final int WINDOW_60S_SAMPLES = (int) (60.0 / LOOP_PERIOD_SECS);

    /** Set false to make every method a no-op. */
    @Setter private boolean enabled = false;

    /** Amps accumulated since the last {@link #logPower()}. */
    @Getter private double totalCurrent = 0.0;

    /** Watts accumulated since the last {@link #logPower()}. */
    @Getter private double totalPower = 0.0;

    /** Joules accumulated since the logger was enabled. */
    @Getter private double totalEnergy = 0.0;

    /** Peak current, amps, since the last {@link #resetMaximums()}. */
    @Getter private double maxCurrent = 0.0;

    /** Peak power, watts, since the last {@link #resetMaximums()}. */
    @Getter private double maxPower = 0.0;

    /** Highest 20 s rolling average current, amps. */
    @Getter private double max20sCurrentA = 0.0;

    /** Highest 45 s rolling average current, amps. */
    @Getter private double max45sCurrentA = 0.0;

    /** Highest 60 s rolling average current, amps. */
    @Getter private double max60sCurrentA = 0.0;

    /** Battery terminal voltage, volts, used to turn amps into watts. */
    @Setter private double batteryVoltage = 12.6;

    /** Estimated RoboRIO current draw, amps. */
    @Setter private double rioCurrent = 0.0;

    private final Map<String, Double> subsystemCurrents = new HashMap<>();
    private final Map<String, Double> subsystemPowers = new HashMap<>();
    private final Map<String, Double> subsystemEnergies = new HashMap<>();
    private final Map<String, Double> maxSubsystemCurrents = new HashMap<>();

    private final Deque<Double> currentHistory20s = new ArrayDeque<>();

    private final Deque<Double> currentHistory45s = new ArrayDeque<>();
    private final Deque<Double> currentHistory60s = new ArrayDeque<>();

    /** Running sums, so a rolling average costs no traversal. */
    private double rollingCurrent20s = 0.0;

    private double rollingCurrent45s = 0.0;
    private double rollingCurrent60s = 0.0;

    /**
     * Records a subsystem's current draw and accumulates it into the running totals. A key split on
     * {@code "/"} or {@code "-"} also aggregates the reading under each parent key.
     *
     * @param key hierarchical name for the consumer, such as Drive/FrontLeft
     * @param amps readings in amps, summed by absolute value
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

    /** Logs one loop's totals, then clears them, so call it once per loop. */
    public void logPower() {
        if (!enabled) {
            return;
        }

        // Overhead current estimates in amps, added here so they count toward the total.
        reportCurrentUsage("Controls/roboRIO", rioCurrent);
        reportCurrentUsage("Controls/CANcoders", 0.05 * 4);
        reportCurrentUsage("Controls/Pigeon", 0.04);
        reportCurrentUsage("Controls/CANivore", 0.03);
        reportCurrentUsage("Controls/Radio", 0.5);

        maxCurrent = Math.max(maxCurrent, totalCurrent);
        maxPower = Math.max(maxPower, totalPower);

        updateRollingWindows(totalCurrent);

        Telemetry.log("BatteryLogger/Current", totalCurrent, "amps");
        Telemetry.log("BatteryLogger/Power", totalPower, "watts");
        Telemetry.log("BatteryLogger/Energy", joulesToWattHours(totalEnergy), "wh");
        Telemetry.log("BatteryLogger/BatteryVoltage", batteryVoltage, "volts");

        Telemetry.log("BatteryLogger/MaxCurrent", maxCurrent, "amps");
        Telemetry.log("BatteryLogger/MaxPower", maxPower, "watts");

        Telemetry.log("BatteryLogger/Max20sCurrent", max20sCurrentA, "amps");
        Telemetry.log("BatteryLogger/Max45sCurrent", max45sCurrentA, "amps");
        Telemetry.log("BatteryLogger/Max60sCurrent", max60sCurrentA, "amps");

        for (var entry : subsystemCurrents.entrySet()) {
            Telemetry.log("BatteryLogger/Current/" + entry.getKey(), entry.getValue(), "amps");
            subsystemCurrents.put(entry.getKey(), 0.0);
        }

        for (var entry : maxSubsystemCurrents.entrySet()) {
            Telemetry.log("BatteryLogger/MaxCurrent/" + entry.getKey(), entry.getValue(), "amps");
        }

        for (var entry : subsystemPowers.entrySet()) {
            Telemetry.log("BatteryLogger/Power/" + entry.getKey(), entry.getValue(), "watts");
            subsystemPowers.put(entry.getKey(), 0.0);
        }

        for (var entry : subsystemEnergies.entrySet()) {
            Telemetry.log(
                    "BatteryLogger/Energy/" + entry.getKey(),
                    joulesToWattHours(entry.getValue()),
                    "wh");
        }

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

    /** Clears the peaks, the rolling window histories, and the per subsystem peaks. */
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

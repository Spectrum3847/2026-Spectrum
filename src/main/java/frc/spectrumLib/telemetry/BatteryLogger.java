// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.spectrumLib.telemetry;

import java.util.HashMap;
import java.util.Map;
import lombok.Getter;
import lombok.Setter;

/**
 * Tracks and logs current draw, power, and cumulative energy consumption across subsystems each
 * robot loop cycle.
 *
 * <p>Key derivation (splitting a channel name into its parent keys) and log-key construction are
 * cached per channel name, so steady-state operation performs no string splitting or concatenation.
 * The set of channel names is small and fixed after the first loop.
 */
public class BatteryLogger {
    /** Duration of one robot loop in seconds, used to convert power (W) to energy (J). */
    private final double loopPeriodSecs = 0.02;

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
    /** Battery terminal voltage used to convert current to power, in volts. */
    @Setter private double batteryVoltage = 12.6;
    /** Estimated current drawn by the RoboRIO itself, in amps. */
    @Setter private double rioCurrent = 0.0;

    private final Map<String, Double> subsystemCurrents = new HashMap<>();
    private final Map<String, Double> subsystemPowers = new HashMap<>();
    private final Map<String, Double> subsystemEnergies = new HashMap<>();

    /** Channel name to its parent keys, derived once per channel by {@link #deriveParentKeys}. */
    private final Map<String, String[]> parentKeys = new HashMap<>();

    /** Channel or parent name to its fully qualified log key, built once per name. */
    private final Map<String, String> currentLogKeys = new HashMap<>();

    private final Map<String, String> powerLogKeys = new HashMap<>();
    private final Map<String, String> energyLogKeys = new HashMap<>();

    /**
     * Splits a channel name on "/" or "-" and returns every parent prefix joined with "/", from the
     * root down. {@code "A/B/C"} yields {@code ["A", "A/B"]}; a name with no separator yields an
     * empty array.
     */
    private static String[] deriveParentKeys(String key) {
        String[] keys = key.split("/|-");
        if (keys.length < 2) {
            return new String[0];
        }
        String[] parents = new String[keys.length - 1];
        StringBuilder subkey = new StringBuilder();
        for (int i = 0; i < keys.length - 1; i++) {
            if (i > 0) {
                subkey.append('/');
            }
            subkey.append(keys[i]);
            parents[i] = subkey.toString();
        }
        return parents;
    }

    /**
     * Records the current draw for a named subsystem channel and accumulates it into the running
     * totals. The {@code key} may use "/" or "-" as separators; parent keys are automatically
     * aggregated.
     *
     * @param key Hierarchical name for the current consumer (e.g. {@code "Drive/FrontLeft"})
     * @param amps One or more current readings in amps; absolute values are summed
     */
    public void reportCurrentUsage(String key, double... amps) {
        if (enabled) {
            double totalAmps = 0.0;
            for (double amp : amps) totalAmps += Math.abs(amp);

            double power = totalAmps * batteryVoltage;
            double energy = power * loopPeriodSecs;

            totalCurrent += totalAmps;
            totalPower += power;
            totalEnergy += energy;

            subsystemCurrents.put(key, totalAmps);
            subsystemPowers.put(key, power);
            subsystemEnergies.merge(key, energy, Double::sum);

            for (String parent : parentKeys.computeIfAbsent(key, BatteryLogger::deriveParentKeys)) {
                subsystemCurrents.merge(parent, totalAmps, Double::sum);
                subsystemPowers.merge(parent, power, Double::sum);
                subsystemEnergies.merge(parent, energy, Double::sum);
            }
        }
    }

    /**
     * Appends control-overhead current consumers (roboRIO, CANcoders, Pigeon, CANivore, radio),
     * then logs total and per-subsystem current, power, and energy to DogLog under the {@code
     * BatteryLogger/} key hierarchy. Resets per-loop current and power accumulators afterward;
     * cumulative energy is preserved across calls.
     */
    public void logPower() {
        if (enabled) {
            // Controls overhead is added here so it is included in the totalCurrent log below.
            // Subsystem currents have already been accumulated via logBatteryUsage() in periodic().
            reportCurrentUsage("Controls/roboRIO", rioCurrent);
            reportCurrentUsage("Controls/CANcoders", 0.05 * 4);
            reportCurrentUsage("Controls/Pigeon", 0.04);
            reportCurrentUsage("Controls/CANivore", 0.03);
            reportCurrentUsage("Controls/Radio", 0.5);

            // Log total (subsystems + controls overhead) and per-subsystem energy usage
            Telemetry.log("BatteryLogger/Current", totalCurrent, "amps");
            Telemetry.log("BatteryLogger/Power", totalPower, "watts");
            Telemetry.log("BatteryLogger/Energy", joulesToWattHours(totalEnergy), "wh");
            Telemetry.log("BatteryLogger/BatteryVoltage", batteryVoltage, "volts");

            for (var entry : subsystemCurrents.entrySet()) {
                Telemetry.log(
                        currentLogKeys.computeIfAbsent(
                                entry.getKey(), k -> "BatteryLogger/Current/" + k),
                        entry.getValue(),
                        "amps");
                entry.setValue(0.0);
            }
            for (var entry : subsystemPowers.entrySet()) {
                Telemetry.log(
                        powerLogKeys.computeIfAbsent(
                                entry.getKey(), k -> "BatteryLogger/Power/" + k),
                        entry.getValue(),
                        "watts");
                entry.setValue(0.0);
            }
            for (var entry : subsystemEnergies.entrySet()) {
                Telemetry.log(
                        energyLogKeys.computeIfAbsent(
                                entry.getKey(), k -> "BatteryLogger/Energy/" + k),
                        joulesToWattHours(entry.getValue()),
                        "wh");
            }

            // Reset power and current totals, before next loop
            totalPower = 0.0;
            totalCurrent = 0.0;
        }
    }

    /**
     * Current accumulated this loop for a channel or parent key, in amps. Package-private for
     * tests.
     */
    double getSubsystemCurrent(String key) {
        return subsystemCurrents.getOrDefault(key, 0.0);
    }

    /** Joules to watt hours. */
    private double joulesToWattHours(double joules) {
        return joules / 3600.0;
    }
}

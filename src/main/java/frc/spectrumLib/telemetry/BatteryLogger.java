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

    private Map<String, Double> subsystemCurrents = new HashMap<>();
    private Map<String, Double> subsystemPowers = new HashMap<>();
    private Map<String, Double> subsystemEnergies = new HashMap<>();

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
                Telemetry.log("BatteryLogger/Current/" + entry.getKey(), entry.getValue(), "amps");
                subsystemCurrents.put(entry.getKey(), 0.0);
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

            // Reset power and current totals, before next loop
            totalPower = 0.0;
            totalCurrent = 0.0;
        }
    }

    private double joulesToWattHours(double joules) {
        return joules / 3600.0;
    }
}

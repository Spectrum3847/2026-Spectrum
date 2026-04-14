// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.spectrumLib;

import java.util.HashMap;
import java.util.Map;
import lombok.Getter;
import lombok.Setter;

/** Class for logging current, power, and energy usage. */
public class BatteryLogger {
    private final double loopPeriodSecs = 0.02;

    @Setter private boolean enabled = false;
    @Getter private double totalCurrent = 0.0;
    @Getter private double totalPower = 0.0;
    @Getter private double totalEnergy = 0.0;
    @Setter private double batteryVoltage = 12.6;
    @Setter private double rioCurrent = 0.0;

    private Map<String, Double> subsytemCurrents = new HashMap<>();
    private Map<String, Double> subsytemPowers = new HashMap<>();
    private Map<String, Double> subsytemEnergies = new HashMap<>();

    public void reportCurrentUsage(String key, double... amps) {
        if (enabled) {
            double totalAmps = 0.0;
            for (double amp : amps) totalAmps += Math.abs(amp);

            double power = totalAmps * batteryVoltage;
            double energy = power * loopPeriodSecs;

            totalCurrent += totalAmps;
            totalPower += power;
            totalEnergy += energy;

            subsytemCurrents.put(key, totalAmps);
            subsytemPowers.put(key, power);
            subsytemEnergies.merge(key, energy, Double::sum);

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
                subsytemCurrents.merge(subkey, totalAmps, Double::sum);
                subsytemPowers.merge(subkey, power, Double::sum);
                subsytemEnergies.merge(subkey, energy, Double::sum);
            }
        }
    }

    public void logPower() {
        if (enabled) {
            reportCurrentUsage("Controls/roboRIO", rioCurrent);
            reportCurrentUsage("Controls/CANcoders", 0.05 * 4);
            reportCurrentUsage("Controls/Pigeon", 0.04);
            reportCurrentUsage("Controls/CANivore", 0.03);
            reportCurrentUsage("Controls/Radio", 0.5);

            // Log total and subsystem energy usage
            Telemetry.log("BatteryLogger/Current", totalCurrent, "amps");
            Telemetry.log("BatteryLogger/Power", totalPower, "watts");
            Telemetry.log("BatteryLogger/Energy", joulesToWattHours(totalEnergy), "wh");
            Telemetry.log("BatteryLogger/BatteryVoltage", batteryVoltage, "volts");

            for (var entry : subsytemCurrents.entrySet()) {
                Telemetry.log("BatteryLogger/Current/" + entry.getKey(), entry.getValue(), "amps");
                subsytemCurrents.put(entry.getKey(), 0.0);
            }
            for (var entry : subsytemPowers.entrySet()) {
                Telemetry.log("BatteryLogger/Power/" + entry.getKey(), entry.getValue(), "watts");
                subsytemPowers.put(entry.getKey(), 0.0);
            }
            for (var entry : subsytemEnergies.entrySet()) {
                Telemetry.log(
                        "BatteryLogger/Energy/" + entry.getKey(),
                        joulesToWattHours(entry.getValue()),
                        "wh");
            }

            // Reset power and curren totals, before next loop
            totalPower = 0.0;
            totalCurrent = 0.0;
        }
    }

    private double joulesToWattHours(double joules) {
        return joules / 3600.0;
    }
}

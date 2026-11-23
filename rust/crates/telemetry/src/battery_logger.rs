// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

//! Tracks and logs current draw, power, and cumulative energy consumption across subsystems
//! each robot loop cycle.
//!
//! Key derivation (splitting a channel name into its parent keys) and log-key construction
//! are cached per channel name, so steady-state operation does no string splitting or
//! concatenation. The set of channel names is small and fixed after the first loop.
//!
//! Current is published at 10 Hz ([`slow_log_this_loop`]) and power and energy every
//! [`SLOW_LOG_PERIOD_SECONDS`], because this class was the largest single producer of records
//! on the robot and the loop period tracks that volume closely. In the 2026-09-05 18:38 log
//! it wrote 32.3 of 94 records per loop, a third of everything. Current is the channel worth
//! watching live; power is current times pack voltage and energy is its integral, so neither
//! needs 50 Hz to stay useful.

use crate::telemetry::{
    log_dash_always_with_unit, log_dash_with_unit, log_with_unit, slow_log_this_loop,
};
use std::cell::{Cell, RefCell};
use std::collections::HashMap;
use wpilib::timer;
use wpimath::math_util;

/// Seconds between power and energy publications.
///
/// At a 20 ms loop this drops their combined ~24 records per loop to about one, without
/// removing either channel from the log.
const SLOW_LOG_PERIOD_SECONDS: f64 = 0.5;

/// Ceiling on one integration step, in seconds.
///
/// Bounds what a stall or a pause between enables can bank into the energy total. Normal
/// loops are far below it; anything above it is not time the mechanisms actually spent
/// drawing the current we just sampled.
const MAX_INTEGRATION_SECONDS: f64 = 0.25;

/// Splits a channel name on "/" or "-" and returns every parent prefix joined with "/", from
/// the root down. `"A/B/C"` yields `["A", "A/B"]`; a name with no separator yields none.
fn derive_parent_keys(key: &str) -> Vec<String> {
    let mut keys: Vec<&str> = key.split(['/', '-']).collect();
    // Java: String.split drops trailing empty strings, so "A/" has one part, not two.
    while keys.len() > 1 && keys.last() == Some(&"") {
        keys.pop();
    }
    if keys.len() < 2 {
        return Vec::new();
    }
    let mut parents = Vec::with_capacity(keys.len() - 1);
    let mut subkey = String::new();
    for (i, part) in keys[..keys.len() - 1].iter().enumerate() {
        if i > 0 {
            subkey.push('/');
        }
        subkey.push_str(part);
        parents.push(subkey.clone());
    }
    parents
}

fn joules_to_watt_hours(joules: f64) -> f64 {
    joules / 3600.0
}

#[derive(Default)]
struct State {
    /// Running total of current draw accumulated since the last `log_power` call, in amps.
    total_current: f64,
    /// Total power drawn on the most recent `log_power` call, in watts.
    total_power: f64,
    /// Cumulative energy consumed over the entire enabled session, in joules.
    total_energy: f64,

    subsystem_currents: HashMap<String, f64>,
    subsystem_energies: HashMap<String, f64>,

    /// Channel name to its parent keys, derived once per channel by `derive_parent_keys`.
    parent_keys: HashMap<String, Vec<String>>,

    /// Channel or parent name to its fully qualified log key, built once per name.
    current_log_keys: HashMap<String, String>,
    power_log_keys: HashMap<String, String>,
    energy_log_keys: HashMap<String, String>,

    /// FPGA time of the previous `log_power` call, if any.
    last_integration_seconds: Option<f64>,

    /// FPGA time of the previous power/energy publication, if any.
    last_slow_log_seconds: Option<f64>,
}

pub struct BatteryLogger {
    /// When false all methods are no-ops, allowing the logger to be disabled at runtime.
    enabled: Cell<bool>,
    /// Battery terminal voltage used to convert current to power, in volts.
    battery_voltage: Cell<f64>,
    /// Estimated current drawn by the roboRIO itself, in amps.
    rio_current: Cell<f64>,
    state: RefCell<State>,
}

impl Default for BatteryLogger {
    fn default() -> Self {
        Self::new()
    }
}

impl BatteryLogger {
    pub fn new() -> Self {
        Self {
            enabled: Cell::new(false),
            battery_voltage: Cell::new(12.6),
            rio_current: Cell::new(0.0),
            state: RefCell::new(State::default()),
        }
    }

    pub fn set_enabled(&self, enabled: bool) {
        self.enabled.set(enabled);
    }

    pub fn set_battery_voltage(&self, volts: f64) {
        self.battery_voltage.set(volts);
    }

    pub fn set_rio_current(&self, amps: f64) {
        self.rio_current.set(amps);
    }

    pub fn total_current(&self) -> f64 {
        self.state.borrow().total_current
    }

    pub fn total_power(&self) -> f64 {
        self.state.borrow().total_power
    }

    pub fn total_energy(&self) -> f64 {
        self.state.borrow().total_energy
    }

    /// Records the current draw for a named subsystem channel and accumulates it into the
    /// running totals. The `key` may use "/" or "-" as separators; parent keys are
    /// automatically aggregated.
    ///
    /// Power and energy are computed from these currents in [`Self::log_power`], which knows
    /// this loop's pack voltage and its true elapsed time.
    ///
    /// `amps` are one or more current readings; absolute values are summed.
    pub fn report_current_usage(&self, key: &str, amps: &[f64]) {
        if !self.enabled.get() {
            return;
        }
        let total_amps: f64 = amps.iter().map(|a| a.abs()).sum();
        let mut s = self.state.borrow_mut();
        s.total_current += total_amps;
        s.subsystem_currents.insert(key.to_owned(), total_amps);

        if !s.parent_keys.contains_key(key) {
            s.parent_keys
                .insert(key.to_owned(), derive_parent_keys(key));
        }
        let State {
            parent_keys,
            subsystem_currents,
            ..
        } = &mut *s;
        for parent in &parent_keys[key] {
            *subsystem_currents.entry(parent.clone()).or_insert(0.0) += total_amps;
        }
    }

    /// Appends control-overhead current consumers (roboRIO, CANcoders, Pigeon, CANivore,
    /// radio), integrates energy over the elapsed loop, then logs current on the slow tier
    /// and power and energy every [`SLOW_LOG_PERIOD_SECONDS`]. Resets the per-loop current
    /// accumulators afterward; cumulative energy is preserved across calls.
    pub fn log_power(&self) {
        self.log_power_at(timer::fpga_timestamp());
    }

    fn log_power_at(&self, now: f64) {
        if !self.enabled.get() {
            return;
        }
        self.report_current_usage("Controls/roboRIO", &[self.rio_current.get()]);
        self.report_current_usage("Controls/CANcoders", &[0.05 * 4.0]);
        self.report_current_usage("Controls/Pigeon", &[0.04]);
        self.report_current_usage("Controls/CANivore", &[0.03]);
        self.report_current_usage("Controls/Radio", &[0.5]);

        let voltage = self.battery_voltage.get();
        let mut guard = self.state.borrow_mut();
        let s = &mut *guard;

        // Integrate over the loop that actually elapsed, not an assumed 20 ms: the
        // 2026-09-05 logs ran a 20.2 to 27.4 ms median period, which under-reported every
        // energy total by roughly a fifth to a third.
        let elapsed = match s.last_integration_seconds {
            None => 0.0,
            Some(last) => math_util::clamp(now - last, 0.0, MAX_INTEGRATION_SECONDS),
        };
        s.last_integration_seconds = Some(now);

        s.total_power = s.total_current * voltage;
        s.total_energy += s.total_power * elapsed;
        for (key, amps) in &s.subsystem_currents {
            *s.subsystem_energies.entry(key.clone()).or_insert(0.0) += amps * voltage * elapsed;
        }

        let log_slow = s
            .last_slow_log_seconds
            .is_none_or(|last| now - last >= SLOW_LOG_PERIOD_SECONDS);
        if log_slow {
            s.last_slow_log_seconds = Some(now);
        }

        // Current at 10 Hz. Total current, pack voltage, energy, and the per-mechanism
        // currents are read on the Driver Station dashboard and in the robot app, hence
        // log_dash.
        let log_current = slow_log_this_loop();
        if log_current {
            log_dash_with_unit("BatteryLogger/Current", s.total_current, "amps");
            log_dash_with_unit("BatteryLogger/BatteryVoltage", voltage, "volts");
        }
        if log_slow {
            log_with_unit("BatteryLogger/Power", s.total_power, "watts");
            log_dash_always_with_unit(
                "BatteryLogger/Energy",
                joules_to_watt_hours(s.total_energy),
                "wh",
            );
        }

        let State {
            subsystem_currents,
            subsystem_energies,
            current_log_keys,
            power_log_keys,
            energy_log_keys,
            ..
        } = s;
        for (key, amps) in subsystem_currents.iter_mut() {
            if log_current {
                let log_key = current_log_keys
                    .entry(key.clone())
                    .or_insert_with(|| format!("BatteryLogger/Current/{key}"));
                log_dash_with_unit(log_key, *amps, "amps");
            }
            if log_slow {
                let log_key = power_log_keys
                    .entry(key.clone())
                    .or_insert_with(|| format!("BatteryLogger/Power/{key}"));
                log_with_unit(log_key, *amps * voltage, "watts");
            }
            *amps = 0.0;
        }
        if log_slow {
            for (key, joules) in subsystem_energies.iter() {
                let log_key = energy_log_keys
                    .entry(key.clone())
                    .or_insert_with(|| format!("BatteryLogger/Energy/{key}"));
                log_with_unit(log_key, joules_to_watt_hours(*joules), "wh");
            }
        }

        s.total_current = 0.0;
    }

    /// Current accumulated this loop for a channel or parent key, in amps.
    #[cfg(test)]
    fn subsystem_current(&self, key: &str) -> f64 {
        self.state
            .borrow()
            .subsystem_currents
            .get(key)
            .copied()
            .unwrap_or(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::{set_loop_count, testing};

    #[test]
    fn parent_keys() {
        assert_eq!(derive_parent_keys("A/B/C"), ["A", "A/B"]);
        assert_eq!(derive_parent_keys("Mechanisms/Hood"), ["Mechanisms"]);
        assert_eq!(derive_parent_keys("Drive-FrontLeft"), ["Drive"]);
        assert_eq!(derive_parent_keys("A/B-C"), ["A", "A/B"]);
        assert!(derive_parent_keys("Solo").is_empty());
        assert!(derive_parent_keys("").is_empty());
        // Java: trailing separators add no part.
        assert!(derive_parent_keys("A/").is_empty());
        assert_eq!(derive_parent_keys("A//B"), ["A", "A/"]);
    }

    #[test]
    fn disabled_logger_does_nothing() {
        let _g = testing::serial();
        let b = BatteryLogger::new();
        b.report_current_usage("A/B", &[5.0]);
        assert_eq!(b.total_current(), 0.0);
        let out = testing::capture(|| b.log_power_at(1.0));
        assert!(out.is_empty());
    }

    #[test]
    fn currents_sum_absolute_values_and_roll_up_to_parents() {
        let b = BatteryLogger::new();
        b.set_enabled(true);
        b.report_current_usage("Mechanisms/Hood", &[3.0, -2.0]);
        b.report_current_usage("Mechanisms/Intake", &[1.0]);
        b.report_current_usage("Swerve-Drive-FL", &[4.0]);
        assert_eq!(b.subsystem_current("Mechanisms/Hood"), 5.0);
        assert_eq!(b.subsystem_current("Mechanisms"), 6.0);
        assert_eq!(b.subsystem_current("Swerve"), 4.0);
        assert_eq!(b.subsystem_current("Swerve/Drive"), 4.0);
        assert_eq!(b.total_current(), 10.0);
    }

    fn by_key(out: &[testing::Logged]) -> HashMap<String, Vec<f64>> {
        let mut map: HashMap<String, Vec<f64>> = HashMap::new();
        for l in out {
            map.entry(l.key.clone()).or_default().push(l.double());
        }
        map
    }

    #[test]
    fn power_and_energy_integrate_over_the_real_elapsed_time() {
        let _g = testing::serial();
        let b = BatteryLogger::new();
        b.set_enabled(true);
        b.set_battery_voltage(12.0);
        b.set_rio_current(0.0);
        // Overhead channels: 0.2 + 0.04 + 0.03 + 0.5 = 0.77 A, plus 10 A of mechanisms.
        let overhead = 0.05 * 4.0 + 0.04 + 0.03 + 0.5;

        set_loop_count(1);
        let first = testing::capture(|| {
            b.report_current_usage("Mechanisms/Hood", &[10.0]);
            b.log_power_at(100.0);
        });
        let m = by_key(&first);
        // The first call integrates nothing.
        assert_eq!(b.total_energy(), 0.0);
        assert!((m["BatteryLogger/Power"][0] - (10.0 + overhead) * 12.0).abs() < 1e-9);
        assert_eq!(m["BatteryLogger/Energy"], [0.0]);
        assert!(
            !m.contains_key("BatteryLogger/Current"),
            "loop 1 is not a slow loop"
        );
        assert_eq!(b.total_current(), 0.0, "accumulators reset");

        // 30 ms later, within the slow-log period: energy integrates, nothing is published.
        set_loop_count(2);
        let second = testing::capture(|| {
            b.report_current_usage("Mechanisms/Hood", &[10.0]);
            b.log_power_at(100.03);
        });
        assert!(second.is_empty());
        let want = (10.0 + overhead) * 12.0 * 0.03;
        assert!((b.total_energy() - want).abs() < 1e-9);
        assert!((b.total_power() - (10.0 + overhead) * 12.0).abs() < 1e-9);

        // 0.6 s after the first publication, on a slow loop: everything publishes.
        set_loop_count(5);
        let third = testing::capture(|| {
            b.report_current_usage("Mechanisms/Hood", &[10.0]);
            b.log_power_at(100.6);
        });
        let m = by_key(&third);
        // The step is clamped to 0.25 s: the loop was not really that long.
        let want = want + (10.0 + overhead) * 12.0 * MAX_INTEGRATION_SECONDS;
        assert!((b.total_energy() - want).abs() < 1e-9);
        assert!((m["BatteryLogger/Energy"][0] - want / 3600.0).abs() < 1e-12);
        assert!((m["BatteryLogger/Current"][0] - (10.0 + overhead)).abs() < 1e-9);
        assert_eq!(m["BatteryLogger/BatteryVoltage"], [12.0]);
        assert_eq!(m["BatteryLogger/Current/Mechanisms/Hood"], [10.0]);
        assert_eq!(m["BatteryLogger/Current/Mechanisms"], [10.0]);
        assert!((m["BatteryLogger/Power/Mechanisms/Hood"][0] - 120.0).abs() < 1e-9);
        assert!(m.contains_key("BatteryLogger/Energy/Controls/Radio"));
        assert!(m.contains_key("BatteryLogger/Energy/Mechanisms"));
        let by = |k: &str| third.iter().find(|l| l.key == k).unwrap();
        assert!(by("BatteryLogger/Current").force_nt);
        assert!(
            by("BatteryLogger/Energy").force_nt,
            "energy is always published"
        );
        assert!(!by("BatteryLogger/Power").force_nt);
        assert_eq!(by("BatteryLogger/Power").unit.as_deref(), Some("watts"));
        assert_eq!(by("BatteryLogger/Energy").unit.as_deref(), Some("wh"));
        assert_eq!(by("BatteryLogger/Current").unit.as_deref(), Some("amps"));
    }

    #[test]
    fn channels_that_stop_reporting_log_zero() {
        let _g = testing::serial();
        let b = BatteryLogger::new();
        b.set_enabled(true);
        set_loop_count(0);
        testing::capture(|| {
            b.report_current_usage("Mechanisms/Hood", &[10.0]);
            b.log_power_at(10.0);
        });
        set_loop_count(5);
        let out = testing::capture(|| b.log_power_at(10.6));
        let m = by_key(&out);
        assert_eq!(m["BatteryLogger/Current/Mechanisms/Hood"], [0.0]);
    }
}

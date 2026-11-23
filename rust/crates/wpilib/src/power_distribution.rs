//! Java `PowerDistribution`: PDH or PDP readings. Reads report HAL failures to the driver
//! station and return zero.

use crate::hal::{self, HalError, or_report};
use std::panic::Location;

/// `HAL_PowerDistributionType` values the robot can ask for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModuleType {
    Ctre,
    Rev,
}

impl ModuleType {
    fn hal_value(self) -> i32 {
        match self {
            Self::Ctre => 1,
            Self::Rev => 2,
        }
    }

    fn from_hal(value: i32) -> Self {
        if value == 2 { Self::Rev } else { Self::Ctre }
    }
}

const AUTOMATIC_TYPE: i32 = 0;
const DEFAULT_MODULE: i32 = -1;

#[derive(Debug)]
pub struct PowerDistribution {
    handle: i32,
    module: i32,
}

impl PowerDistribution {
    /// The module on the default CAN ID, of whichever type is found.
    #[track_caller]
    pub fn new() -> Result<Self, HalError> {
        Self::init(DEFAULT_MODULE, AUTOMATIC_TYPE, Location::caller())
    }

    #[track_caller]
    pub fn with_module(module: i32, kind: ModuleType) -> Result<Self, HalError> {
        Self::init(module, kind.hal_value(), Location::caller())
    }

    fn init(module: i32, kind: i32, at: &Location<'_>) -> Result<Self, HalError> {
        let location = format!("{}:{}", at.file(), at.line());
        let handle = hal::initialize_power_distribution(module, kind, &location)?;
        // Built before the fallible reads so Drop frees the handle on error.
        let mut pd = Self { handle, module: 0 };
        pd.module = hal::pd_module_number(handle)?;
        let instance = match ModuleType::from_hal(hal::pd_type(handle)?) {
            ModuleType::Ctre => hal::INSTANCE_PDP_CTRE,
            ModuleType::Rev => hal::INSTANCE_PDP_REV,
        };
        hal::report_usage(hal::RESOURCE_PDP, instance);
        Ok(pd)
    }

    pub fn module(&self) -> i32 {
        self.module
    }

    #[track_caller]
    pub fn module_type(&self) -> ModuleType {
        ModuleType::from_hal(or_report(hal::pd_type(self.handle)))
    }

    #[track_caller]
    pub fn num_channels(&self) -> usize {
        usize::try_from(or_report(hal::pd_num_channels(self.handle))).unwrap_or(0)
    }

    #[track_caller]
    pub fn voltage(&self) -> f64 {
        or_report(hal::pd_voltage(self.handle))
    }

    #[track_caller]
    pub fn temperature(&self) -> f64 {
        or_report(hal::pd_temperature(self.handle))
    }

    #[track_caller]
    pub fn current(&self, channel: i32) -> f64 {
        or_report(hal::pd_channel_current(self.handle, channel))
    }

    /// One current per channel, amps.
    #[track_caller]
    pub fn all_currents(&self) -> Vec<f64> {
        let mut currents = vec![0.0; self.num_channels()];
        or_report(hal::pd_all_channel_currents(self.handle, &mut currents));
        currents
    }

    #[track_caller]
    pub fn total_current(&self) -> f64 {
        or_report(hal::pd_total_current(self.handle))
    }

    #[track_caller]
    pub fn total_power(&self) -> f64 {
        or_report(hal::pd_total_power(self.handle))
    }

    #[track_caller]
    pub fn total_energy(&self) -> f64 {
        or_report(hal::pd_total_energy(self.handle))
    }

    #[track_caller]
    pub fn reset_total_energy(&self) {
        or_report(hal::pd_reset_total_energy(self.handle));
    }

    #[track_caller]
    pub fn clear_sticky_faults(&self) {
        or_report(hal::pd_clear_sticky_faults(self.handle));
    }

    #[track_caller]
    pub fn switchable_channel(&self) -> bool {
        or_report(hal::pd_switchable_channel(self.handle))
    }

    #[track_caller]
    pub fn set_switchable_channel(&self, enabled: bool) {
        or_report(hal::pd_set_switchable_channel(self.handle, enabled));
    }
}

impl Drop for PowerDistribution {
    fn drop(&mut self) {
        hal::clean_power_distribution(self.handle);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn module_type_round_trips() {
        assert_eq!(
            ModuleType::from_hal(ModuleType::Rev.hal_value()),
            ModuleType::Rev
        );
        assert_eq!(
            ModuleType::from_hal(ModuleType::Ctre.hal_value()),
            ModuleType::Ctre
        );
        assert_eq!(ModuleType::from_hal(0), ModuleType::Ctre);
    }
}

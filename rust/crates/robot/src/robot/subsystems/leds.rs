//! LED strip for the robot. Wraps [`SpectrumLEDs`] for the pattern library and the CANdle
//! hardware. Bind the pattern methods from `Robot` or `SuperStructure`.

use crate::spectrum_lib::hardware::rio::Rio;
use crate::spectrum_lib::leds::spectrum_leds::{Config, SpectrumLEDs};
use commands::{CommandScheduler, Subsystem, SubsystemId};
use phoenix6::CanBus;
use phoenix6::signals::{LossOfSignalBehaviorValue, StripTypeValue};
use std::ops::Deref;
use std::rc::Rc;
use telemetry::telemetry;

/// Length of the external LED strip, which the CANdle addresses from index 8.
pub const NUM_LEDS: i32 = 20;

/// Hardware config, already set to address the external strip only. Set `start_idx` to 0 to
/// span the onboard LEDs as well. Java builds this once in a static block. A function here,
/// because the config holds non-`Sync` handles.
pub fn leds_config() -> Config {
    let mut leds_config = Config::new("Leds", 1, NUM_LEDS, CanBus::new(Rio::CANIVORE));
    leds_config.strip_type = StripTypeValue::RGB;
    leds_config.brightness = 0.5;
    leds_config.loss_of_signal_behavior = LossOfSignalBehaviorValue::DisableLEDs;
    leds_config
}

pub struct Leds {
    leds: Rc<SpectrumLEDs>,
}

impl Deref for Leds {
    type Target = SpectrumLEDs;

    fn deref(&self) -> &SpectrumLEDs {
        &self.leds
    }
}

impl Leds {
    pub fn new() -> Rc<Leds> {
        let spectrum_leds = SpectrumLEDs::new(leds_config());

        // Java's `Leds` overrides `periodic`. `SpectrumLEDs::new` registered the base object,
        // so swap it for this one under the same id.
        let scheduler = CommandScheduler::get_instance();
        scheduler.unregister_subsystem(spectrum_leds.id());
        let leds = Rc::new(Leds {
            leds: spectrum_leds,
        });
        scheduler.register_subsystem(leds.clone());

        leds.set_default_command(
            leds.set_pattern(&leds.breathe(leds.purple, 2.0), -1)
                .with_name("Leds.idle"),
        );

        telemetry::print(&format!("{} Subsystem Initialized", leds.name()));
        leds
    }
}

impl Subsystem for Leds {
    fn id(&self) -> SubsystemId {
        self.leds.id()
    }

    fn name(&self) -> String {
        "Leds".to_owned()
    }

    fn periodic(&self) {
        telemetry::log("Leds/CurrentCommand", self.get_current_command_name());
        telemetry::log("Leds/CommandPriority", self.get_command_priority());
        telemetry::log("Leds/IsAnimating", self.is_animating());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_config_addresses_the_external_strip_only() {
        let config = leds_config();
        assert_eq!(config.name, "Leds");
        assert_eq!(config.device_id, 1);
        assert_eq!(config.num_leds, NUM_LEDS);
        assert_eq!(config.start_idx, 8);
        assert_eq!(config.can_bus.as_ref().map(CanBus::name), Some("*"));
        assert_eq!(config.strip_type, StripTypeValue::RGB);
        assert_eq!(config.brightness, 0.5);
        assert_eq!(
            config.loss_of_signal_behavior,
            LossOfSignalBehaviorValue::DisableLEDs
        );
    }
}

use crate::spectrum_lib::hardware::can_config_budget;
use crate::spectrum_lib::hardware::spectrum_cancoder_config::SpectrumCANcoderConfig;
use crate::spectrum_lib::mechanism::mechanism::Config;
use phoenix6::signals::{FeedbackSensorSourceValue, SensorDirectionValue};
use phoenix6::{CANcoder, CANcoderConfiguration, StatusCode, TalonFX, TalonFXConfiguration};
use telemetry::telemetry;

/// Selects how the TalonFX reads position data from the remote CANcoder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CANCoderFeedbackType {
    /// Position is read remotely; motor encoder is used for velocity.
    RemoteCANcoder,
    /// CANcoder position is fused with the motor encoder for high-bandwidth feedback.
    FusedCANcoder,
    /// Motor encoder is synchronized to the CANcoder position on enable.
    SyncCANcoder,
}

impl CANCoderFeedbackType {
    pub fn sensor_source(self) -> FeedbackSensorSourceValue {
        match self {
            Self::RemoteCANcoder => FeedbackSensorSourceValue::RemoteCANcoder,
            Self::FusedCANcoder => FeedbackSensorSourceValue::FusedCANcoder,
            Self::SyncCANcoder => FeedbackSensorSourceValue::SyncCANcoder,
        }
    }
}

/// Wraps a CTRE CANcoder and applies Spectrum configuration. When the config says the
/// encoder is attached, the constructor also points the supplied TalonFX's feedback at it.
#[derive(Debug)]
pub struct SpectrumCANcoder {
    can_coder: Option<CANcoder>,
    config: SpectrumCANcoderConfig,
    feedback_source: CANCoderFeedbackType,
}

impl SpectrumCANcoder {
    /// Configures the CANcoder, and when it is attached points the motor's feedback at it.
    /// Nothing else happens if the config marks the encoder unattached.
    ///
    /// Takes the config by value and stores `cancoder_id` in it, where Java stored it in the
    /// caller's object. `mech_config` is the mechanism's config, whose Talon settings change
    /// in place.
    pub fn new(
        cancoder_id: i32,
        mut config: SpectrumCANcoderConfig,
        motor: &TalonFX,
        mech_config: &mut Config,
        feedback_source: CANCoderFeedbackType,
    ) -> Self {
        config.cancoder_id = cancoder_id;
        let mut cancoder = Self {
            can_coder: None,
            config,
            feedback_source,
        };

        if cancoder.config.attached {
            // Fused/Sync feedback requires the CANcoder to be on the same bus as the motor.
            let can_coder = CANcoder::new(cancoder_id, motor.network().clone())
                .unwrap_or_else(|code| panic!("Could not create CANcoder {cancoder_id}: {code}"));
            let can_coder_configs = cancoder_configuration(&cancoder.config);
            let response = can_config_budget::run(&format!("CANcoder {cancoder_id}"), |timeout| {
                can_coder
                    .configurator()
                    .apply_with_timeout(&can_coder_configs, timeout)
            });
            cancoder.can_coder = Some(can_coder);
            if cancoder.can_coder_response_ok(response) {
                cancoder.modify_motor_config(motor, mech_config);
            }
        }
        cancoder
    }

    pub fn get_can_coder(&self) -> Option<&CANcoder> {
        self.can_coder.as_ref()
    }

    pub fn is_attached(&self) -> bool {
        self.config.attached
    }

    /// Points the motor's feedback at this CANcoder using the configured source and ratios.
    /// Modifies `mech_config`'s stored TalonFX config in place, then applies it to the motor.
    pub fn modify_motor_config(&self, motor: &TalonFX, mech_config: &mut Config) -> &Self {
        let talon_config_mod = &mut mech_config.talon_config;
        point_feedback_at_cancoder(talon_config_mod, &self.config, self.feedback_source);
        motor.configurator().apply(&*talon_config_mod);
        self
    }

    /// True when the configurator's response came back OK, and prints the failure to the
    /// telemetry log when it did not.
    pub fn can_coder_response_ok(&self, response: StatusCode) -> bool {
        if !response.is_ok() {
            telemetry::print(&format!(
                "CANcoder ID {} failed config with error {}",
                self.config.cancoder_id, response
            ));
            return false;
        }
        true
    }
}

/// The CANcoder's own settings: magnet offset, direction, and a 0.5 rotation wrap point.
fn cancoder_configuration(config: &SpectrumCANcoderConfig) -> CANcoderConfiguration {
    let mut can_coder_configs = CANcoderConfiguration::default();
    can_coder_configs.magnet_sensor.magnet_offset = config.offset;
    can_coder_configs.magnet_sensor.sensor_direction = if config.inverted {
        SensorDirectionValue::Clockwise_Positive
    } else {
        SensorDirectionValue::CounterClockwise_Positive
    };
    can_coder_configs
        .magnet_sensor
        .absolute_sensor_discontinuity_point = 0.5;
    can_coder_configs
}

/// The Talon feedback settings that read the CANcoder.
fn point_feedback_at_cancoder(
    talon_config: &mut TalonFXConfiguration,
    config: &SpectrumCANcoderConfig,
    feedback_source: CANCoderFeedbackType,
) {
    talon_config.feedback.feedback_remote_sensor_id = config.cancoder_id;
    talon_config.feedback.feedback_sensor_source = feedback_source.sensor_source();
    talon_config.feedback.rotor_to_sensor_ratio = config.rotor_to_sensor_ratio;
    talon_config.feedback.sensor_to_mechanism_ratio = config.sensor_to_mechanism_ratio;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn feedback_types_map_to_the_matching_sensor_source() {
        assert_eq!(
            CANCoderFeedbackType::RemoteCANcoder.sensor_source(),
            FeedbackSensorSourceValue::RemoteCANcoder
        );
        assert_eq!(
            CANCoderFeedbackType::FusedCANcoder.sensor_source(),
            FeedbackSensorSourceValue::FusedCANcoder
        );
        assert_eq!(
            CANCoderFeedbackType::SyncCANcoder.sensor_source(),
            FeedbackSensorSourceValue::SyncCANcoder
        );
    }

    #[test]
    fn config_constructor_keeps_the_ratios_and_flags() {
        let config = SpectrumCANcoderConfig::new(1.7, 1.0, 0.25, true, true);
        assert_eq!(config.rotor_to_sensor_ratio, 1.7);
        assert_eq!(config.sensor_to_mechanism_ratio, 1.0);
        assert_eq!(config.offset, 0.25);
        assert!(config.attached);
        assert!(config.inverted);
    }

    #[test]
    fn an_unattached_cancoder_reports_it_and_has_no_device() {
        let cancoder = SpectrumCANcoder {
            can_coder: None,
            config: SpectrumCANcoderConfig::new(1.7, 1.0, 0.0, false, false),
            feedback_source: CANCoderFeedbackType::FusedCANcoder,
        };
        assert!(!cancoder.is_attached());
        assert!(cancoder.get_can_coder().is_none());
    }

    #[test]
    fn cancoder_configuration_carries_offset_direction_and_wrap_point() {
        let normal =
            cancoder_configuration(&SpectrumCANcoderConfig::new(1.0, 1.0, 0.125, true, false));
        assert_eq!(normal.magnet_sensor.magnet_offset, 0.125);
        assert_eq!(
            normal.magnet_sensor.sensor_direction,
            SensorDirectionValue::CounterClockwise_Positive
        );
        assert_eq!(
            normal.magnet_sensor.absolute_sensor_discontinuity_point,
            0.5
        );

        let inverted =
            cancoder_configuration(&SpectrumCANcoderConfig::new(1.0, 1.0, 0.0, true, true));
        assert_eq!(
            inverted.magnet_sensor.sensor_direction,
            SensorDirectionValue::Clockwise_Positive
        );
    }

    #[test]
    fn motor_feedback_points_at_the_cancoder_with_the_configured_ratios() {
        let mut config = SpectrumCANcoderConfig::new(1.7, 2.0, 0.0, true, false);
        config.cancoder_id = 9;
        let mut talon_config = TalonFXConfiguration::default();
        point_feedback_at_cancoder(
            &mut talon_config,
            &config,
            CANCoderFeedbackType::SyncCANcoder,
        );
        let feedback = &talon_config.feedback;
        assert_eq!(feedback.feedback_remote_sensor_id, 9);
        assert_eq!(
            feedback.feedback_sensor_source,
            FeedbackSensorSourceValue::SyncCANcoder
        );
        assert_eq!(feedback.rotor_to_sensor_ratio, 1.7);
        assert_eq!(feedback.sensor_to_mechanism_ratio, 2.0);
    }

    #[test]
    fn pointing_feedback_leaves_other_talon_settings_alone() {
        let config = SpectrumCANcoderConfig::new(1.0, 1.0, 0.0, true, false);
        let mut talon_config = TalonFXConfiguration::default();
        talon_config.current_limits.supply_current_limit = 33.0;
        point_feedback_at_cancoder(
            &mut talon_config,
            &config,
            CANCoderFeedbackType::FusedCANcoder,
        );
        assert_eq!(talon_config.current_limits.supply_current_limit, 33.0);
    }
}

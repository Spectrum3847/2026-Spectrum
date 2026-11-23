use crate::spectrum_lib::hardware::can_config_budget;
use crate::spectrum_lib::util::can_device_id::CanDeviceId;
use phoenix6::controls::Follower;
use phoenix6::signals::{
    FeedbackSensorSourceValue, ForwardLimitSourceValue, ForwardLimitTypeValue, InvertedValue,
    MotorAlignmentValue, NeutralModeValue, ReverseLimitSourceValue, ReverseLimitTypeValue,
};
use phoenix6::{CanBus, TalonFX, TalonFXConfiguration};
use wpilib::driver_station;

const NEUTRAL_MODE: NeutralModeValue = NeutralModeValue::Brake;
const INVERT_VALUE: InvertedValue = InvertedValue::CounterClockwise_Positive;
/// Fraction of full output.
const NEUTRAL_DEADBAND: f64 = 0.04;
/// Amps.
const SUPPLY_CURRENT_LIMIT: f64 = 40.0;

/// Builds TalonFX objects and applies the Spectrum defaults. Closed-loop and sensor
/// parameters are left for the application to set.
pub fn create_default_talon(id: &CanDeviceId) -> TalonFX {
    create_config_talon(id, &get_default_config())
}

pub fn create_config_talon(id: &CanDeviceId, config: &TalonFXConfiguration) -> TalonFX {
    let talon = create_talon(id);
    let result = can_config_budget::run(&format!("Talon {}", id.get_device_number()), |timeout| {
        talon.configurator().apply_with_timeout(config, timeout)
    });
    if !result.is_ok() {
        driver_station::report_warning(&format!(
            "Could not apply config to Talon {}: {}",
            id.get_device_number(),
            result
        ));
    }
    talon
}

/// Builds a motor that mirrors the leader's output.
///
/// `motor_alignment` is `Aligned` for a follower mounted the same way round as the leader,
/// `Opposed` for one mounted backwards. Panics if the buses differ, where Java throws
/// `IllegalArgumentException`.
pub fn create_permanent_follower_talon(
    follower_id: &CanDeviceId,
    leader_talon_fx: &TalonFX,
    motor_alignment: MotorAlignmentValue,
) -> TalonFX {
    let leader_can_bus = leader_talon_fx.network().name();
    let leader_id = leader_talon_fx.device_id();
    assert!(
        follower_id.get_bus() == leader_can_bus,
        "Leader and Follower Talons must be on the same CAN bus"
    );

    let mut follower_config = get_default_config();
    leader_talon_fx.configurator().refresh(&mut follower_config);
    let talon = create_config_talon(follower_id, &follower_config);

    talon.set_control(&Follower::new(leader_id, motor_alignment));
    talon
}

pub fn get_default_config() -> TalonFXConfiguration {
    let mut config = TalonFXConfiguration::default();

    config.motor_output.neutral_mode = NEUTRAL_MODE;
    config.motor_output.inverted = INVERT_VALUE;
    config.motor_output.duty_cycle_neutral_deadband = NEUTRAL_DEADBAND;
    config.motor_output.peak_forward_duty_cycle = 1.0;
    config.motor_output.peak_reverse_duty_cycle = -1.0;

    config.current_limits.supply_current_limit = SUPPLY_CURRENT_LIMIT;
    config.current_limits.supply_current_limit_enable = true;
    config.current_limits.stator_current_limit_enable = false;

    config.software_limit_switch.forward_soft_limit_enable = false;
    config.software_limit_switch.forward_soft_limit_threshold = 0.0;
    config.software_limit_switch.reverse_soft_limit_enable = false;
    config.software_limit_switch.reverse_soft_limit_threshold = 0.0;

    config.feedback.feedback_sensor_source = FeedbackSensorSourceValue::RotorSensor;
    config.feedback.feedback_rotor_offset = 0.0;
    config.feedback.sensor_to_mechanism_ratio = 1.0;

    config.hardware_limit_switch.forward_limit_enable = false;
    config
        .hardware_limit_switch
        .forward_limit_autoset_position_enable = false;
    config.hardware_limit_switch.forward_limit_source = ForwardLimitSourceValue::LimitSwitchPin;
    config.hardware_limit_switch.forward_limit_type = ForwardLimitTypeValue::NormallyOpen;
    config.hardware_limit_switch.reverse_limit_enable = false;
    config
        .hardware_limit_switch
        .reverse_limit_autoset_position_enable = false;
    config.hardware_limit_switch.reverse_limit_source = ReverseLimitSourceValue::LimitSwitchPin;
    config.hardware_limit_switch.reverse_limit_type = ReverseLimitTypeValue::NormallyOpen;

    config.audio.beep_on_boot = true;
    config.audio.allow_music_dur_disable = true;
    config.audio.beep_on_config = true;

    config
}

/// Panics for a device ID outside 0 to 62, where Java throws.
fn create_talon(id: &CanDeviceId) -> TalonFX {
    let talon =
        TalonFX::new(id.get_device_number(), CanBus::new(id.get_bus())).unwrap_or_else(|code| {
            panic!(
                "Could not create Talon {} on bus \"{}\": {}",
                id.get_device_number(),
                id.get_bus(),
                code
            )
        });
    // Blocking, and worthless on a bus with nothing on it.
    if !can_config_budget::exhausted() {
        talon.clear_sticky_faults();
    }

    talon
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_is_brake_with_a_forty_amp_supply_limit() {
        let config = get_default_config();
        assert_eq!(config.motor_output.neutral_mode, NeutralModeValue::Brake);
        assert_eq!(
            config.motor_output.inverted,
            InvertedValue::CounterClockwise_Positive
        );
        assert_eq!(config.motor_output.duty_cycle_neutral_deadband, 0.04);
        assert_eq!(config.motor_output.peak_forward_duty_cycle, 1.0);
        assert_eq!(config.motor_output.peak_reverse_duty_cycle, -1.0);
        assert_eq!(config.current_limits.supply_current_limit, 40.0);
        assert!(config.current_limits.supply_current_limit_enable);
        assert!(!config.current_limits.stator_current_limit_enable);
    }

    #[test]
    fn default_config_disables_every_limit_switch() {
        let config = get_default_config();
        assert!(!config.software_limit_switch.forward_soft_limit_enable);
        assert!(!config.software_limit_switch.reverse_soft_limit_enable);
        assert_eq!(
            config.software_limit_switch.forward_soft_limit_threshold,
            0.0
        );
        assert_eq!(
            config.software_limit_switch.reverse_soft_limit_threshold,
            0.0
        );
        assert!(!config.hardware_limit_switch.forward_limit_enable);
        assert!(!config.hardware_limit_switch.reverse_limit_enable);
        assert!(
            !config
                .hardware_limit_switch
                .forward_limit_autoset_position_enable
        );
        assert!(
            !config
                .hardware_limit_switch
                .reverse_limit_autoset_position_enable
        );
    }

    #[test]
    fn default_config_uses_the_rotor_sensor_one_to_one() {
        let config = get_default_config();
        assert_eq!(
            config.feedback.feedback_sensor_source,
            FeedbackSensorSourceValue::RotorSensor
        );
        assert_eq!(config.feedback.feedback_rotor_offset, 0.0);
        assert_eq!(config.feedback.sensor_to_mechanism_ratio, 1.0);
    }

    #[test]
    fn default_config_beeps_and_plays_music_while_disabled() {
        let config = get_default_config();
        assert!(config.audio.beep_on_boot);
        assert!(config.audio.beep_on_config);
        assert!(config.audio.allow_music_dur_disable);
    }
}

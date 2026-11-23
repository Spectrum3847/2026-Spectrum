//! Pure tests: enum values, config items, control defaults. Nothing here loads the natives.
//! Values come from the Java sources and `SpnEnums.hpp` of 26.3.0.

use crate::configs::*;
use crate::controls::*;
use crate::signals::*;
use crate::spn;
use std::collections::{HashMap, HashSet};

#[test]
fn enum_values_match_java() {
    assert_eq!(InvertedValue::CounterClockwise_Positive.value(), 0);
    assert_eq!(InvertedValue::Clockwise_Positive.value(), 1);
    assert_eq!(NeutralModeValue::Coast.value(), 0);
    assert_eq!(NeutralModeValue::Brake.value(), 1);
    assert_eq!(MotorAlignmentValue::Aligned.value(), 0);
    assert_eq!(MotorAlignmentValue::Opposed.value(), 1);
    assert_eq!(GravityTypeValue::Elevator_Static.value(), 0);
    assert_eq!(GravityTypeValue::Arm_Cosine.value(), 1);
    assert_eq!(StaticFeedforwardSignValue::UseVelocitySign.value(), 0);
    assert_eq!(StaticFeedforwardSignValue::UseClosedLoopSign.value(), 1);
    assert_eq!(FeedbackSensorSourceValue::RotorSensor.value(), 0);
    assert_eq!(FeedbackSensorSourceValue::RemoteCANcoder.value(), 1);
    assert_eq!(FeedbackSensorSourceValue::FusedCANcoder.value(), 5);
    assert_eq!(FeedbackSensorSourceValue::SyncCANcoder.value(), 6);
    assert_eq!(ForwardLimitTypeValue::NormallyOpen.value(), 0);
    assert_eq!(ForwardLimitTypeValue::NormallyClosed.value(), 1);
    assert_eq!(ForwardLimitSourceValue::LimitSwitchPin.value(), 0);
    assert_eq!(ForwardLimitSourceValue::Disabled.value(), 3);
    assert_eq!(ReverseLimitTypeValue::NormallyOpen.value(), 0);
    assert_eq!(ReverseLimitSourceValue::LimitSwitchPin.value(), 0);
    assert_eq!(SensorDirectionValue::CounterClockwise_Positive.value(), 0);
    assert_eq!(SensorDirectionValue::Clockwise_Positive.value(), 1);
    assert_eq!(LossOfSignalBehaviorValue::KeepRunning.value(), 0);
    assert_eq!(LossOfSignalBehaviorValue::DisableLEDs.value(), 1);
    assert_eq!(StripTypeValue::GRB.value(), 0);
    assert_eq!(StripTypeValue::RGB.value(), 1);
    assert_eq!(StripTypeValue::BRG.value(), 2);
    assert_eq!(StripTypeValue::GRBW.value(), 6);
    assert_eq!(StripTypeValue::RGBW.value(), 7);
    assert_eq!(StripTypeValue::BRGW.value(), 8);
    assert_eq!(LarsonBounceValue::Front.value(), 0);
    assert_eq!(LarsonBounceValue::Center.value(), 1);
    assert_eq!(LarsonBounceValue::Back.value(), 2);
    assert_eq!(AnimationDirectionValue::Forward.value(), 0);
    assert_eq!(AnimationDirectionValue::Backward.value(), 1);
}

#[test]
fn from_value_round_trips_and_unknown_means_first_variant() {
    assert_eq!(StripTypeValue::from_value(7), StripTypeValue::RGBW);
    assert_eq!(StripTypeValue::from_value(3), StripTypeValue::GRB);
    assert_eq!(
        FeedbackSensorSourceValue::from_value(5),
        FeedbackSensorSourceValue::FusedCANcoder
    );
    assert_eq!(
        FeedbackSensorSourceValue::from_value(99),
        FeedbackSensorSourceValue::RotorSensor
    );
    assert_eq!(
        ForwardLimitSourceValue::from_value(3),
        ForwardLimitSourceValue::Disabled
    );
    assert_eq!(
        MagnetHealthValue::from_value(3),
        MagnetHealthValue::Magnet_Green
    );
}

/// A fake device text: every item by SPN.
struct MapSource(HashMap<i32, ConfigValue>);

impl MapSource {
    fn of(config: &impl ParentConfiguration) -> Self {
        Self(config.items().iter().map(|i| (i.spn, i.value)).collect())
    }
}

impl ConfigSource for MapSource {
    fn double(&self, spn: i32) -> Option<f64> {
        match self.0.get(&spn)? {
            ConfigValue::Double(v) => Some(*v),
            _ => None,
        }
    }

    fn int(&self, spn: i32) -> Option<i32> {
        match self.0.get(&spn)? {
            ConfigValue::Int(v) => Some(*v),
            _ => None,
        }
    }

    fn boolean(&self, spn: i32) -> Option<bool> {
        match self.0.get(&spn)? {
            ConfigValue::Bool(v) => Some(*v),
            _ => None,
        }
    }
}

#[test]
fn group_item_counts_match_java_serialize() {
    // One ConfigJNI.Serialize call per field in each Java class.
    assert_eq!(MotorOutputConfigs::default().items().len(), 6);
    assert_eq!(CurrentLimitsConfigs::default().items().len(), 6);
    assert_eq!(VoltageConfigs::default().items().len(), 3);
    assert_eq!(TorqueCurrentConfigs::default().items().len(), 3);
    assert_eq!(FeedbackConfigs::default().items().len(), 6);
    assert_eq!(DifferentialSensorsConfigs::default().items().len(), 4);
    assert_eq!(DifferentialConstantsConfigs::default().items().len(), 3);
    assert_eq!(OpenLoopRampsConfigs::default().items().len(), 3);
    assert_eq!(ClosedLoopRampsConfigs::default().items().len(), 3);
    assert_eq!(HardwareLimitSwitchConfigs::default().items().len(), 12);
    assert_eq!(AudioConfigs::default().items().len(), 3);
    assert_eq!(SoftwareLimitSwitchConfigs::default().items().len(), 4);
    assert_eq!(MotionMagicConfigs::default().items().len(), 5);
    assert_eq!(CustomParamsConfigs::default().items().len(), 2);
    assert_eq!(ClosedLoopGeneralConfigs::default().items().len(), 4);
    assert_eq!(Slot0Configs::default().items().len(), 11);
    assert_eq!(Slot1Configs::default().items().len(), 11);
    assert_eq!(Slot2Configs::default().items().len(), 11);
    assert_eq!(MagnetSensorConfigs::default().items().len(), 3);
    assert_eq!(MountPoseConfigs::default().items().len(), 3);
    assert_eq!(GyroTrimConfigs::default().items().len(), 3);
    assert_eq!(Pigeon2FeaturesConfigs::default().items().len(), 3);
    assert_eq!(LEDConfigs::default().items().len(), 3);
    assert_eq!(CANdleFeaturesConfigs::default().items().len(), 3);
}

#[test]
fn configuration_item_counts_are_the_sum_of_their_groups() {
    assert_eq!(TalonFXConfiguration::default().items().len(), 100);
    assert_eq!(CANcoderConfiguration::default().items().len(), 5);
    assert_eq!(Pigeon2Configuration::default().items().len(), 11);
    assert_eq!(CANdleConfiguration::default().items().len(), 8);
}

#[test]
fn every_spn_in_a_configuration_is_distinct() {
    let talon = TalonFXConfiguration::default().items();
    let unique: HashSet<i32> = talon.iter().map(|i| i.spn).collect();
    assert_eq!(unique.len(), talon.len());
}

#[test]
fn talon_items_start_with_motor_output_in_java_order() {
    let items = TalonFXConfiguration::default().items();
    let expected = [
        (spn::Config_Inverted, ConfigValue::Int(0)),
        (spn::Config_NeutralMode, ConfigValue::Int(0)),
        (spn::Config_DutyCycleNeutralDB, ConfigValue::Double(0.0)),
        (spn::Config_PeakForwardDC, ConfigValue::Double(1.0)),
        (spn::Config_PeakReverseDC, ConfigValue::Double(-1.0)),
        (spn::Config_ControlTimesyncFreq, ConfigValue::Double(0.0)),
        (spn::Config_StatorCurrentLimit, ConfigValue::Double(120.0)),
        (spn::Config_StatorCurrLimitEn, ConfigValue::Bool(true)),
        (spn::Config_SupplyCurrentLimit, ConfigValue::Double(70.0)),
        (spn::Config_SupplyCurrLimitEn, ConfigValue::Bool(true)),
        (
            spn::Config_SupplyCurrentLowerLimit,
            ConfigValue::Double(40.0),
        ),
        (spn::Config_SupplyCurrentLowerTime, ConfigValue::Double(1.0)),
    ];
    for (item, (spn, value)) in items.iter().zip(expected) {
        assert_eq!(item.spn, spn);
        assert_eq!(item.value, value);
    }
}

#[test]
fn talon_items_end_with_slot2() {
    let items = TalonFXConfiguration::default().items();
    let last = items.last().expect("items");
    assert_eq!(last.spn, spn::Slot2_GainSchedBehavior);
    assert_eq!(last.value, ConfigValue::Int(0));
    let slot0_kp = items
        .iter()
        .position(|i| i.spn == spn::Slot0_kP)
        .expect("slot 0");
    assert_eq!(slot0_kp, 100 - 33);
}

#[test]
fn java_defaults() {
    let motor = MotorOutputConfigs::default();
    assert_eq!(motor.inverted, InvertedValue::CounterClockwise_Positive);
    assert_eq!(motor.neutral_mode, NeutralModeValue::Coast);
    assert_eq!(motor.peak_reverse_duty_cycle, -1.0);

    let limits = CurrentLimitsConfigs::default();
    assert_eq!(limits.stator_current_limit, 120.0);
    assert!(limits.stator_current_limit_enable);
    assert_eq!(limits.supply_current_limit, 70.0);
    assert!(limits.supply_current_limit_enable);

    let feedback = FeedbackConfigs::default();
    assert_eq!(feedback.sensor_to_mechanism_ratio, 1.0);
    assert_eq!(feedback.rotor_to_sensor_ratio, 1.0);
    assert_eq!(feedback.feedback_remote_sensor_id, 0);

    let switches = HardwareLimitSwitchConfigs::default();
    assert!(switches.forward_limit_enable);
    assert!(switches.reverse_limit_enable);

    let audio = AudioConfigs::default();
    assert!(audio.beep_on_boot && audio.beep_on_config && !audio.allow_music_dur_disable);

    let magic = MotionMagicConfigs::default();
    assert_eq!(magic.motion_magic_expo_k_v, 0.12);
    assert_eq!(magic.motion_magic_expo_k_a, 0.1);

    let slot = Slot1Configs::default();
    assert_eq!(slot.gravity_type, GravityTypeValue::Elevator_Static);
    assert_eq!(
        slot.static_feedforward_sign,
        StaticFeedforwardSignValue::UseVelocitySign
    );

    let magnet = MagnetSensorConfigs::default();
    assert_eq!(magnet.absolute_sensor_discontinuity_point, 0.5);

    let led = LEDConfigs::default();
    assert_eq!(led.strip_type, StripTypeValue::GRB);
    assert_eq!(led.brightness_scalar, 1.0);
    assert_eq!(
        led.loss_of_signal_behavior,
        LossOfSignalBehaviorValue::KeepRunning
    );

    let pigeon = Pigeon2Configuration::default();
    assert!(pigeon.future_proof_configs);
    assert!(pigeon.future_proof());
    assert!(!MotorOutputConfigs::default().future_proof());
}

#[test]
fn builders_set_one_field_each() {
    let slot = Slot0Configs::default()
        .with_k_p(500.0)
        .with_k_d(20.0)
        .with_k_s(0.15)
        .with_static_feedforward_sign(StaticFeedforwardSignValue::UseClosedLoopSign);
    assert_eq!(slot.k_p, 500.0);
    assert_eq!(slot.k_d, 20.0);
    assert_eq!(slot.k_s, 0.15);
    assert_eq!(slot.k_i, 0.0);
    assert_eq!(
        slot.static_feedforward_sign,
        StaticFeedforwardSignValue::UseClosedLoopSign
    );

    let config = TalonFXConfiguration::default()
        .with_current_limits(CurrentLimitsConfigs::default().with_supply_current_limit(40.0))
        .with_future_proof_configs(false);
    assert_eq!(config.current_limits.supply_current_limit, 40.0);
    assert!(!config.future_proof());
}

#[test]
fn loading_collected_items_reproduces_a_changed_configuration() {
    let mut changed = TalonFXConfiguration::default();
    changed.motor_output.inverted = InvertedValue::Clockwise_Positive;
    changed.motor_output.neutral_mode = NeutralModeValue::Brake;
    changed.motor_output.duty_cycle_neutral_deadband = 0.04;
    changed.current_limits.supply_current_limit = 40.0;
    changed.current_limits.stator_current_limit_enable = false;
    changed.feedback.feedback_sensor_source = FeedbackSensorSourceValue::FusedCANcoder;
    changed.feedback.feedback_remote_sensor_id = 12;
    changed.feedback.rotor_to_sensor_ratio = 7.5;
    changed.hardware_limit_switch.forward_limit_enable = false;
    changed.hardware_limit_switch.reverse_limit_source = ReverseLimitSourceValue::RemoteCANcoder;
    changed.motion_magic.motion_magic_cruise_velocity = 80.0;
    changed.closed_loop_general.continuous_wrap = true;
    changed.slot0.k_p = 1.5;
    changed.slot1.gravity_type = GravityTypeValue::Arm_Cosine;
    changed.slot2.k_g = 0.25;

    let mut loaded = TalonFXConfiguration::default();
    loaded.load(&MapSource::of(&changed));
    assert_eq!(loaded, changed);
}

#[test]
fn loading_missing_items_keeps_current_values() {
    let mut config = Slot0Configs::default().with_k_p(3.0);
    config.load(&MapSource(HashMap::new()));
    assert_eq!(config.k_p, 3.0);
}

#[test]
fn loading_picks_each_group_by_spn() {
    let mut source = MapSource(HashMap::new());
    source.0.insert(spn::Slot1_kP, ConfigValue::Double(9.0));
    let mut config = TalonFXConfiguration::default();
    config.load(&source);
    assert_eq!(config.slot1.k_p, 9.0);
    assert_eq!(config.slot0.k_p, 0.0);
    assert_eq!(config.slot2.k_p, 0.0);
}

#[test]
fn control_defaults_match_java() {
    let duty = DutyCycleOut::new(0.5);
    assert_eq!(duty.output, 0.5);
    assert!(duty.enable_foc);
    assert_eq!(duty.update_freq_hz, 100.0);
    assert_eq!(duty.name(), "DutyCycleOut");

    let volts = VoltageOut::new(6.0);
    assert!(volts.enable_foc);
    assert!(!volts.ignore_software_limits);
    assert_eq!(volts.update_freq_hz, 100.0);
    assert!(
        VoltageOut::new(1.0)
            .with_ignore_software_limits(true)
            .ignore_software_limits
    );

    let torque = TorqueCurrentFOC::new(10.0);
    assert_eq!(torque.max_abs_duty_cycle, 1.0);
    assert_eq!(torque.deadband, 0.0);
    assert_eq!(torque.update_freq_hz, 100.0);

    let position = PositionVoltage::new(2.0);
    assert_eq!(
        (position.velocity, position.feed_forward, position.slot),
        (0.0, 0.0, 0)
    );
    assert!(position.enable_foc);
    assert_eq!(PositionTorqueCurrentFOC::new(1.0).update_freq_hz, 100.0);

    let velocity = VelocityVoltage::new(50.0);
    assert_eq!(velocity.acceleration, 0.0);
    assert!(velocity.enable_foc);
    assert_eq!(VelocityTorqueCurrentFOC::new(1.0).slot, 0);

    let magic = MotionMagicVoltage::new(1.0).with_slot(1);
    assert_eq!(magic.slot, 1);
    assert!(magic.enable_foc);
    assert_eq!(MotionMagicTorqueCurrentFOC::new(1.0).feed_forward, 0.0);
    assert_eq!(MotionMagicVelocityVoltage::new(3.0).acceleration, 0.0);
    assert_eq!(
        MotionMagicVelocityTorqueCurrentFOC::new(3.0).update_freq_hz,
        100.0
    );

    let dynamic = DynamicMotionMagicVoltage::new(1.0, 2.0, 3.0);
    assert_eq!(
        (dynamic.position, dynamic.velocity, dynamic.acceleration),
        (1.0, 2.0, 3.0)
    );
    assert_eq!(dynamic.jerk, 0.0);
    assert!(dynamic.enable_foc);
    assert_eq!(
        DynamicMotionMagicTorqueCurrentFOC::new(0.0, 0.0, 0.0).jerk,
        0.0
    );
}

#[test]
fn slow_requests_default_to_20_hz_and_zero_resends_once() {
    let follower = Follower::new(3, MotorAlignmentValue::Opposed);
    assert_eq!(follower.leader_id, 3);
    assert_eq!(follower.motor_alignment, MotorAlignmentValue::Opposed);
    assert_eq!(follower.update_freq_hz, 20.0);
    assert_eq!(NeutralOut::new().update_freq_hz, 20.0);
    assert_eq!(CoastOut::new().update_freq_hz, 20.0);
    assert_eq!(StaticBrake::new().update_freq_hz, 20.0);
    assert!(!NeutralOut::new().use_timesync);
    assert_eq!(NeutralOut::new().name(), "NeutralOut");
}

#[test]
fn animation_defaults_match_java() {
    let rainbow = RainbowAnimation::new(8, 15);
    assert_eq!((rainbow.led_start_index, rainbow.led_end_index), (8, 15));
    assert_eq!(rainbow.slot, 0);
    assert_eq!(rainbow.brightness, 1.0);
    assert_eq!(rainbow.direction, AnimationDirectionValue::Forward);
    assert_eq!(rainbow.frame_rate, 100.0);
    assert_eq!(rainbow.update_freq_hz, 20.0);

    assert_eq!(RgbFadeAnimation::new(0, 1).frame_rate, 100.0);
    assert_eq!(RgbFadeAnimation::new(0, 1).brightness, 1.0);
    assert_eq!(SingleFadeAnimation::new(0, 1).frame_rate, 100.0);
    assert_eq!(StrobeAnimation::new(0, 1).frame_rate, 4.0);
    assert_eq!(StrobeAnimation::new(0, 1).color, RgbwColor::default());

    let larson = LarsonAnimation::new(0, 1);
    assert_eq!(larson.size, 3);
    assert_eq!(larson.bounce_mode, LarsonBounceValue::Front);
    assert_eq!(larson.frame_rate, 25.0);

    let fire = FireAnimation::new(0, 1);
    assert_eq!(
        (fire.brightness, fire.sparking, fire.cooling),
        (1.0, 0.6, 0.3)
    );
    assert_eq!(fire.frame_rate, 60.0);
    assert_eq!(fire.direction, AnimationDirectionValue::Forward);

    let flow = ColorFlowAnimation::new(0, 1);
    assert_eq!(flow.frame_rate, 25.0);
    assert_eq!(flow.direction, AnimationDirectionValue::Forward);

    let empty = EmptyAnimation::new(3);
    assert_eq!(empty.slot, 3);
    assert_eq!(empty.update_freq_hz, 0.0);

    let solid = SolidColor::new(8, 40).with_color(RgbwColor::new(255, 0, 10));
    assert_eq!(solid.color, RgbwColor::new_rgbw(255, 0, 10, 0));
}

#[test]
fn control_names_are_the_java_class_names() {
    assert_eq!(
        MotionMagicTorqueCurrentFOC::new(0.0).name(),
        "MotionMagicTorqueCurrentFOC"
    );
    assert_eq!(
        DynamicMotionMagicVoltage::new(0.0, 0.0, 0.0).name(),
        "DynamicMotionMagicVoltage"
    );
    assert_eq!(SolidColor::new(0, 0).name(), "SolidColor");
    assert_eq!(EmptyControl.name(), "EmptyControl");
}

#[test]
fn rgbw_color_has_no_white_by_default() {
    let color = RgbwColor::new(1, 2, 3);
    assert_eq!(
        (color.red, color.green, color.blue, color.white),
        (1, 2, 3, 0)
    );
    assert_eq!(RgbwColor::default(), RgbwColor::new_rgbw(0, 0, 0, 0));
    assert_ne!(RgbwColor::new(1, 2, 3), RgbwColor::new_rgbw(1, 2, 3, 4));
}

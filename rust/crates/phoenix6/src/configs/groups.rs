//! Generated from the Phoenix 6 26.3.0 Java configs (com.ctre.phoenix6.configs). Field order is
//! the order of Java's `serialize()`.

use super::config_group;
use crate::signals::*;

config_group! {
    /// Configs that directly affect motor output.
    pub struct MotorOutputConfigs {
        inverted / with_inverted: InvertedValue = InvertedValue::CounterClockwise_Positive => Config_Inverted,
        neutral_mode / with_neutral_mode: NeutralModeValue = NeutralModeValue::Coast => Config_NeutralMode,
        /// Units: fractional.
        duty_cycle_neutral_deadband / with_duty_cycle_neutral_deadband: f64 = 0.0 => Config_DutyCycleNeutralDB,
        /// Units: fractional.
        peak_forward_duty_cycle / with_peak_forward_duty_cycle: f64 = 1.0 => Config_PeakForwardDC,
        /// Units: fractional.
        peak_reverse_duty_cycle / with_peak_reverse_duty_cycle: f64 = -1.0 => Config_PeakReverseDC,
        /// Units: Hz.
        control_timesync_freq_hz / with_control_timesync_freq_hz: f64 = 0.0 => Config_ControlTimesyncFreq,
    }
}

config_group! {
    /// Configs that directly affect current limiting features.
    pub struct CurrentLimitsConfigs {
        /// Units: A.
        stator_current_limit / with_stator_current_limit: f64 = 120.0 => Config_StatorCurrentLimit,
        stator_current_limit_enable / with_stator_current_limit_enable: bool = true => Config_StatorCurrLimitEn,
        /// Units: A.
        supply_current_limit / with_supply_current_limit: f64 = 70.0 => Config_SupplyCurrentLimit,
        supply_current_limit_enable / with_supply_current_limit_enable: bool = true => Config_SupplyCurrLimitEn,
        /// Units: A.
        supply_current_lower_limit / with_supply_current_lower_limit: f64 = 40.0 => Config_SupplyCurrentLowerLimit,
        /// Units: seconds.
        supply_current_lower_time / with_supply_current_lower_time: f64 = 1.0 => Config_SupplyCurrentLowerTime,
    }
}

config_group! {
    /// Configs that affect Voltage control types.
    pub struct VoltageConfigs {
        /// Units: seconds.
        supply_voltage_time_constant / with_supply_voltage_time_constant: f64 = 0.0 => Config_SupplyVLowpassTau,
        /// Units: V.
        peak_forward_voltage / with_peak_forward_voltage: f64 = 16.0 => Config_PeakForwardV,
        /// Units: V.
        peak_reverse_voltage / with_peak_reverse_voltage: f64 = -16.0 => Config_PeakReverseV,
    }
}

config_group! {
    /// Configs that affect Torque Current control types.
    pub struct TorqueCurrentConfigs {
        /// Units: A.
        peak_forward_torque_current / with_peak_forward_torque_current: f64 = 800.0 => Config_PeakForTorqCurr,
        /// Units: A.
        peak_reverse_torque_current / with_peak_reverse_torque_current: f64 = -800.0 => Config_PeakRevTorqCurr,
        /// Units: A.
        torque_neutral_deadband / with_torque_neutral_deadband: f64 = 0.0 => Config_TorqueNeutralDB,
    }
}

config_group! {
    /// Configs that affect the feedback of this motor controller.
    pub struct FeedbackConfigs {
        /// Units: rotations.
        feedback_rotor_offset / with_feedback_rotor_offset: f64 = 0.0 => Config_FeedbackRotorOffset,
        /// Units: scalar.
        sensor_to_mechanism_ratio / with_sensor_to_mechanism_ratio: f64 = 1.0 => Config_SensorToMechanismRatio,
        /// Units: scalar.
        rotor_to_sensor_ratio / with_rotor_to_sensor_ratio: f64 = 1.0 => Config_RotorToSensorRatio,
        feedback_sensor_source / with_feedback_sensor_source: FeedbackSensorSourceValue = FeedbackSensorSourceValue::RotorSensor => Config_FeedbackSensorSource,
        feedback_remote_sensor_id / with_feedback_remote_sensor_id: i32 = 0 => Config_FeedbackRemoteSensorID,
        /// Units: seconds.
        velocity_filter_time_constant / with_velocity_filter_time_constant: f64 = 0.0 => Config_VelocityFilterTimeConstant,
    }
}

config_group! {
    /// Configs related to sensors used for differential control of a mechanism.
    pub struct DifferentialSensorsConfigs {
        differential_sensor_source / with_differential_sensor_source: DifferentialSensorSourceValue = DifferentialSensorSourceValue::Disabled => Config_DifferentialSensorSource,
        differential_talon_fx_sensor_id / with_differential_talon_fx_sensor_id: i32 = 0 => Config_DifferentialTalonFXSensorID,
        differential_remote_sensor_id / with_differential_remote_sensor_id: i32 = 0 => Config_DifferentialRemoteSensorID,
        /// Units: scalar.
        sensor_to_differential_ratio / with_sensor_to_differential_ratio: f64 = 1.0 => Config_SensorToDifferentialRatio,
    }
}

config_group! {
    /// Configs related to constants used for differential control of a mechanism.
    pub struct DifferentialConstantsConfigs {
        /// Units: fractional.
        peak_differential_duty_cycle / with_peak_differential_duty_cycle: f64 = 1.0 => Config_PeakDiffDC,
        /// Units: V.
        peak_differential_voltage / with_peak_differential_voltage: f64 = 16.0 => Config_PeakDiffV,
        /// Units: A.
        peak_differential_torque_current / with_peak_differential_torque_current: f64 = 800.0 => Config_PeakDiffTorqCurr,
    }
}

config_group! {
    /// Configs that affect the open-loop control of this motor controller.
    pub struct OpenLoopRampsConfigs {
        /// Units: seconds.
        duty_cycle_open_loop_ramp_period / with_duty_cycle_open_loop_ramp_period: f64 = 0.0 => Config_DutyCycleOpenLoopRampPeriod,
        /// Units: seconds.
        voltage_open_loop_ramp_period / with_voltage_open_loop_ramp_period: f64 = 0.0 => Config_VoltageOpenLoopRampPeriod,
        /// Units: seconds.
        torque_open_loop_ramp_period / with_torque_open_loop_ramp_period: f64 = 0.0 => Config_TorqueOpenLoopRampPeriod,
    }
}

config_group! {
    /// Configs that affect the closed-loop control of this motor controller.
    pub struct ClosedLoopRampsConfigs {
        /// Units: seconds.
        duty_cycle_closed_loop_ramp_period / with_duty_cycle_closed_loop_ramp_period: f64 = 0.0 => Config_DutyCycleClosedLoopRampPeriod,
        /// Units: seconds.
        voltage_closed_loop_ramp_period / with_voltage_closed_loop_ramp_period: f64 = 0.0 => Config_VoltageClosedLoopRampPeriod,
        /// Units: seconds.
        torque_closed_loop_ramp_period / with_torque_closed_loop_ramp_period: f64 = 0.0 => Config_TorqueClosedLoopRampPeriod,
    }
}

config_group! {
    /// Configs that change how the motor controller behaves under different limit switch states.
    pub struct HardwareLimitSwitchConfigs {
        forward_limit_type / with_forward_limit_type: ForwardLimitTypeValue = ForwardLimitTypeValue::NormallyOpen => Config_ForwardLimitType,
        forward_limit_autoset_position_enable / with_forward_limit_autoset_position_enable: bool = false => Config_ForwardLimitAutosetPosEnable,
        /// Units: rotations.
        forward_limit_autoset_position_value / with_forward_limit_autoset_position_value: f64 = 0.0 => Config_ForwardLimitAutosetPosValue,
        forward_limit_enable / with_forward_limit_enable: bool = true => Config_ForwardLimitEnable,
        forward_limit_source / with_forward_limit_source: ForwardLimitSourceValue = ForwardLimitSourceValue::LimitSwitchPin => Config_ForwardLimitSource,
        forward_limit_remote_sensor_id / with_forward_limit_remote_sensor_id: i32 = 0 => Config_ForwardLimitRemoteSensorID,
        reverse_limit_type / with_reverse_limit_type: ReverseLimitTypeValue = ReverseLimitTypeValue::NormallyOpen => Config_ReverseLimitType,
        reverse_limit_autoset_position_enable / with_reverse_limit_autoset_position_enable: bool = false => Config_ReverseLimitAutosetPosEnable,
        /// Units: rotations.
        reverse_limit_autoset_position_value / with_reverse_limit_autoset_position_value: f64 = 0.0 => Config_ReverseLimitAutosetPosValue,
        reverse_limit_enable / with_reverse_limit_enable: bool = true => Config_ReverseLimitEnable,
        reverse_limit_source / with_reverse_limit_source: ReverseLimitSourceValue = ReverseLimitSourceValue::LimitSwitchPin => Config_ReverseLimitSource,
        reverse_limit_remote_sensor_id / with_reverse_limit_remote_sensor_id: i32 = 0 => Config_ReverseLimitRemoteSensorID,
    }
}

config_group! {
    /// Configs that affect audible components of the device.
    pub struct AudioConfigs {
        beep_on_boot / with_beep_on_boot: bool = true => Config_BeepOnBoot,
        beep_on_config / with_beep_on_config: bool = true => Config_BeepOnConfig,
        allow_music_dur_disable / with_allow_music_dur_disable: bool = false => Config_AllowMusicDurDisable,
    }
}

config_group! {
    /// Configs that affect how software-limit switches behave.
    pub struct SoftwareLimitSwitchConfigs {
        forward_soft_limit_enable / with_forward_soft_limit_enable: bool = false => Config_ForwardSoftLimitEnable,
        reverse_soft_limit_enable / with_reverse_soft_limit_enable: bool = false => Config_ReverseSoftLimitEnable,
        /// Units: rotations.
        forward_soft_limit_threshold / with_forward_soft_limit_threshold: f64 = 0.0 => Config_ForwardSoftLimitThreshold,
        /// Units: rotations.
        reverse_soft_limit_threshold / with_reverse_soft_limit_threshold: f64 = 0.0 => Config_ReverseSoftLimitThreshold,
    }
}

config_group! {
    /// Configs for Motion Magic.
    pub struct MotionMagicConfigs {
        /// Units: rot per sec.
        motion_magic_cruise_velocity / with_motion_magic_cruise_velocity: f64 = 0.0 => Config_MotionMagicCruiseVelocity,
        /// Units: rot per sec^2.
        motion_magic_acceleration / with_motion_magic_acceleration: f64 = 0.0 => Config_MotionMagicAcceleration,
        /// Units: rot per sec^3.
        motion_magic_jerk / with_motion_magic_jerk: f64 = 0.0 => Config_MotionMagicJerk,
        /// Units: V/rps.
        motion_magic_expo_k_v / with_motion_magic_expo_k_v: f64 = 0.12 => Config_MotionMagicExpo_kV,
        /// Units: V/rps^2.
        motion_magic_expo_k_a / with_motion_magic_expo_k_a: f64 = 0.1 => Config_MotionMagicExpo_kA,
    }
}

config_group! {
    /// Custom Params.
    pub struct CustomParamsConfigs {
        custom_param0 / with_custom_param0: i32 = 0 => CustomParam0,
        custom_param1 / with_custom_param1: i32 = 0 => CustomParam1,
    }
}

config_group! {
    /// Configs that affect general behavior during closed-looping.
    pub struct ClosedLoopGeneralConfigs {
        continuous_wrap / with_continuous_wrap: bool = false => Config_ContinuousWrap,
        differential_continuous_wrap / with_differential_continuous_wrap: bool = false => Config_DifferentialContinuousWrap,
        /// Units: rotations.
        gain_sched_error_threshold / with_gain_sched_error_threshold: f64 = 0.0 => Config_GainSchedErrorThreshold,
        gain_sched_kp_behavior / with_gain_sched_kp_behavior: GainSchedKpBehaviorValue = GainSchedKpBehaviorValue::Continuous => Config_GainSchedKpBehavior,
    }
}

config_group! {
    /// Gains for the specified slot.
    pub struct Slot0Configs {
        k_p / with_k_p: f64 = 0.0 => Slot0_kP,
        k_i / with_k_i: f64 = 0.0 => Slot0_kI,
        k_d / with_k_d: f64 = 0.0 => Slot0_kD,
        k_s / with_k_s: f64 = 0.0 => Slot0_kS,
        k_v / with_k_v: f64 = 0.0 => Slot0_kV,
        k_a / with_k_a: f64 = 0.0 => Slot0_kA,
        k_g / with_k_g: f64 = 0.0 => Slot0_kG,
        gravity_type / with_gravity_type: GravityTypeValue = GravityTypeValue::Elevator_Static => Slot0_kG_Type,
        static_feedforward_sign / with_static_feedforward_sign: StaticFeedforwardSignValue = StaticFeedforwardSignValue::UseVelocitySign => Slot0_kS_Sign,
        /// Units: rotations.
        gravity_arm_position_offset / with_gravity_arm_position_offset: f64 = 0.0 => Slot0_kG_PosOffset,
        gain_sched_behavior / with_gain_sched_behavior: GainSchedBehaviorValue = GainSchedBehaviorValue::Inactive => Slot0_GainSchedBehavior,
    }
}

config_group! {
    /// Gains for the specified slot.
    pub struct Slot1Configs {
        k_p / with_k_p: f64 = 0.0 => Slot1_kP,
        k_i / with_k_i: f64 = 0.0 => Slot1_kI,
        k_d / with_k_d: f64 = 0.0 => Slot1_kD,
        k_s / with_k_s: f64 = 0.0 => Slot1_kS,
        k_v / with_k_v: f64 = 0.0 => Slot1_kV,
        k_a / with_k_a: f64 = 0.0 => Slot1_kA,
        k_g / with_k_g: f64 = 0.0 => Slot1_kG,
        gravity_type / with_gravity_type: GravityTypeValue = GravityTypeValue::Elevator_Static => Slot1_kG_Type,
        static_feedforward_sign / with_static_feedforward_sign: StaticFeedforwardSignValue = StaticFeedforwardSignValue::UseVelocitySign => Slot1_kS_Sign,
        /// Units: rotations.
        gravity_arm_position_offset / with_gravity_arm_position_offset: f64 = 0.0 => Slot1_kG_PosOffset,
        gain_sched_behavior / with_gain_sched_behavior: GainSchedBehaviorValue = GainSchedBehaviorValue::Inactive => Slot1_GainSchedBehavior,
    }
}

config_group! {
    /// Gains for the specified slot.
    pub struct Slot2Configs {
        k_p / with_k_p: f64 = 0.0 => Slot2_kP,
        k_i / with_k_i: f64 = 0.0 => Slot2_kI,
        k_d / with_k_d: f64 = 0.0 => Slot2_kD,
        k_s / with_k_s: f64 = 0.0 => Slot2_kS,
        k_v / with_k_v: f64 = 0.0 => Slot2_kV,
        k_a / with_k_a: f64 = 0.0 => Slot2_kA,
        k_g / with_k_g: f64 = 0.0 => Slot2_kG,
        gravity_type / with_gravity_type: GravityTypeValue = GravityTypeValue::Elevator_Static => Slot2_kG_Type,
        static_feedforward_sign / with_static_feedforward_sign: StaticFeedforwardSignValue = StaticFeedforwardSignValue::UseVelocitySign => Slot2_kS_Sign,
        /// Units: rotations.
        gravity_arm_position_offset / with_gravity_arm_position_offset: f64 = 0.0 => Slot2_kG_PosOffset,
        gain_sched_behavior / with_gain_sched_behavior: GainSchedBehaviorValue = GainSchedBehaviorValue::Inactive => Slot2_GainSchedBehavior,
    }
}

config_group! {
    /// Configs that affect the magnet sensor and how to interpret it.
    pub struct MagnetSensorConfigs {
        sensor_direction / with_sensor_direction: SensorDirectionValue = SensorDirectionValue::CounterClockwise_Positive => CANcoder_SensorDirection,
        /// Units: rotations.
        magnet_offset / with_magnet_offset: f64 = 0.0 => CANCoder_MagnetOffset,
        /// Units: rotations.
        absolute_sensor_discontinuity_point / with_absolute_sensor_discontinuity_point: f64 = 0.5 => Config_AbsoluteSensorDiscontinuityPoint,
    }
}

config_group! {
    /// Configs for Pigeon 2's Mount Pose configuration.
    pub struct MountPoseConfigs {
        /// Units: deg.
        mount_pose_yaw / with_mount_pose_yaw: f64 = 0.0 => Pigeon2MountPoseYaw,
        /// Units: deg.
        mount_pose_pitch / with_mount_pose_pitch: f64 = 0.0 => Pigeon2MountPosePitch,
        /// Units: deg.
        mount_pose_roll / with_mount_pose_roll: f64 = 0.0 => Pigeon2MountPoseRoll,
    }
}

config_group! {
    /// Configs to trim the Pigeon2's gyroscope.
    pub struct GyroTrimConfigs {
        /// Units: deg per rotation.
        gyro_scalar_x / with_gyro_scalar_x: f64 = 0.0 => Pigeon2GyroScalarX,
        /// Units: deg per rotation.
        gyro_scalar_y / with_gyro_scalar_y: f64 = 0.0 => Pigeon2GyroScalarY,
        /// Units: deg per rotation.
        gyro_scalar_z / with_gyro_scalar_z: f64 = 0.0 => Pigeon2GyroScalarZ,
    }
}

config_group! {
    /// Configs to enable/disable various features of the Pigeon2.
    pub struct Pigeon2FeaturesConfigs {
        enable_compass / with_enable_compass: bool = false => Pigeon2UseCompass,
        disable_temperature_compensation / with_disable_temperature_compensation: bool = false => Pigeon2DisableTemperatureCompensation,
        disable_no_motion_calibration / with_disable_no_motion_calibration: bool = false => Pigeon2DisableNoMotionCalibration,
    }
}

config_group! {
    /// Configs related to CANdle LED control.
    pub struct LEDConfigs {
        strip_type / with_strip_type: StripTypeValue = StripTypeValue::GRB => Config_LED_StripType,
        /// Units: scalar.
        brightness_scalar / with_brightness_scalar: f64 = 1.0 => Config_LED_BrightnessScalar,
        loss_of_signal_behavior / with_loss_of_signal_behavior: LossOfSignalBehaviorValue = LossOfSignalBehaviorValue::KeepRunning => Config_LED_LossOfSignalBehavior,
    }
}

config_group! {
    /// Configs related to general CANdle features.
    pub struct CANdleFeaturesConfigs {
        enable_5v_rail / with_enable_5v_rail: Enable5VRailValue = Enable5VRailValue::Enabled => Config_CANdle_5VRail,
        vbat_output_mode / with_vbat_output_mode: VBatOutputModeValue = VBatOutputModeValue::On => Config_CANdle_VBatOutputMode,
        status_led_when_active / with_status_led_when_active: StatusLedWhenActiveValue = StatusLedWhenActiveValue::Enabled => Config_CANdle_StatusLedWhenActive,
    }
}

//! Generated from the Phoenix 6 26.3.0 Java controls (com.ctre.phoenix6.controls). Defaults,
//! constructor arguments and native argument order match Java.

use super::control;
use crate::signals::*;

control! {
    /// Request a specified motor duty cycle.
    pub struct DutyCycleOut("DutyCycleOut", c_ctre_phoenix6_RequestControlDutyCycleOut, TalonFx) {
        new {
            output / with_output: f64,
        }
        defaults {
            enable_foc / with_enable_foc: bool = true,
            override_brake_dur_neutral / with_override_brake_dur_neutral: bool = false,
            limit_forward_motion / with_limit_forward_motion: bool = false,
            limit_reverse_motion / with_limit_reverse_motion: bool = false,
            ignore_hardware_limits / with_ignore_hardware_limits: bool = false,
            ignore_software_limits / with_ignore_software_limits: bool = false,
            use_timesync / with_use_timesync: bool = false,
            update_freq_hz / with_update_freq_hz: f64 = 100.0,
        }
        send(this) [this.update_freq_hz, this.output, this.enable_foc, this.override_brake_dur_neutral, this.limit_forward_motion, this.limit_reverse_motion, this.ignore_hardware_limits, this.ignore_software_limits, this.use_timesync]
    }
}

control! {
    /// Requires Phoenix Pro; Request a specified motor current (field oriented control).
    pub struct TorqueCurrentFOC("TorqueCurrentFOC", c_ctre_phoenix6_RequestControlTorqueCurrentFOC, TalonFx) {
        new {
            output / with_output: f64,
        }
        defaults {
            max_abs_duty_cycle / with_max_abs_duty_cycle: f64 = 1.0,
            deadband / with_deadband: f64 = 0.0,
            override_coast_dur_neutral / with_override_coast_dur_neutral: bool = false,
            limit_forward_motion / with_limit_forward_motion: bool = false,
            limit_reverse_motion / with_limit_reverse_motion: bool = false,
            ignore_hardware_limits / with_ignore_hardware_limits: bool = false,
            ignore_software_limits / with_ignore_software_limits: bool = false,
            use_timesync / with_use_timesync: bool = false,
            update_freq_hz / with_update_freq_hz: f64 = 100.0,
        }
        send(this) [this.update_freq_hz, this.output, this.max_abs_duty_cycle, this.deadband, this.override_coast_dur_neutral, this.limit_forward_motion, this.limit_reverse_motion, this.ignore_hardware_limits, this.ignore_software_limits, this.use_timesync]
    }
}

control! {
    /// Request a specified voltage.
    pub struct VoltageOut("VoltageOut", c_ctre_phoenix6_RequestControlVoltageOut, TalonFx) {
        new {
            output / with_output: f64,
        }
        defaults {
            enable_foc / with_enable_foc: bool = true,
            override_brake_dur_neutral / with_override_brake_dur_neutral: bool = false,
            limit_forward_motion / with_limit_forward_motion: bool = false,
            limit_reverse_motion / with_limit_reverse_motion: bool = false,
            ignore_hardware_limits / with_ignore_hardware_limits: bool = false,
            ignore_software_limits / with_ignore_software_limits: bool = false,
            use_timesync / with_use_timesync: bool = false,
            update_freq_hz / with_update_freq_hz: f64 = 100.0,
        }
        send(this) [this.update_freq_hz, this.output, this.enable_foc, this.override_brake_dur_neutral, this.limit_forward_motion, this.limit_reverse_motion, this.ignore_hardware_limits, this.ignore_software_limits, this.use_timesync]
    }
}

control! {
    /// Request PID to target position with voltage feedforward
    pub struct PositionVoltage("PositionVoltage", c_ctre_phoenix6_RequestControlPositionVoltage, TalonFx) {
        new {
            position / with_position: f64,
        }
        defaults {
            velocity / with_velocity: f64 = 0.0,
            enable_foc / with_enable_foc: bool = true,
            feed_forward / with_feed_forward: f64 = 0.0,
            slot / with_slot: i32 = 0,
            override_brake_dur_neutral / with_override_brake_dur_neutral: bool = false,
            limit_forward_motion / with_limit_forward_motion: bool = false,
            limit_reverse_motion / with_limit_reverse_motion: bool = false,
            ignore_hardware_limits / with_ignore_hardware_limits: bool = false,
            ignore_software_limits / with_ignore_software_limits: bool = false,
            use_timesync / with_use_timesync: bool = false,
            update_freq_hz / with_update_freq_hz: f64 = 100.0,
        }
        send(this) [this.update_freq_hz, this.position, this.velocity, this.enable_foc, this.feed_forward, this.slot, this.override_brake_dur_neutral, this.limit_forward_motion, this.limit_reverse_motion, this.ignore_hardware_limits, this.ignore_software_limits, this.use_timesync]
    }
}

control! {
    /// Requires Phoenix Pro; Request PID to target position with torque current feedforward.
    pub struct PositionTorqueCurrentFOC("PositionTorqueCurrentFOC", c_ctre_phoenix6_RequestControlPositionTorqueCurrentFOC, TalonFx) {
        new {
            position / with_position: f64,
        }
        defaults {
            velocity / with_velocity: f64 = 0.0,
            feed_forward / with_feed_forward: f64 = 0.0,
            slot / with_slot: i32 = 0,
            override_coast_dur_neutral / with_override_coast_dur_neutral: bool = false,
            limit_forward_motion / with_limit_forward_motion: bool = false,
            limit_reverse_motion / with_limit_reverse_motion: bool = false,
            ignore_hardware_limits / with_ignore_hardware_limits: bool = false,
            ignore_software_limits / with_ignore_software_limits: bool = false,
            use_timesync / with_use_timesync: bool = false,
            update_freq_hz / with_update_freq_hz: f64 = 100.0,
        }
        send(this) [this.update_freq_hz, this.position, this.velocity, this.feed_forward, this.slot, this.override_coast_dur_neutral, this.limit_forward_motion, this.limit_reverse_motion, this.ignore_hardware_limits, this.ignore_software_limits, this.use_timesync]
    }
}

control! {
    /// Request PID to target velocity with voltage feedforward.
    pub struct VelocityVoltage("VelocityVoltage", c_ctre_phoenix6_RequestControlVelocityVoltage, TalonFx) {
        new {
            velocity / with_velocity: f64,
        }
        defaults {
            acceleration / with_acceleration: f64 = 0.0,
            enable_foc / with_enable_foc: bool = true,
            feed_forward / with_feed_forward: f64 = 0.0,
            slot / with_slot: i32 = 0,
            override_brake_dur_neutral / with_override_brake_dur_neutral: bool = false,
            limit_forward_motion / with_limit_forward_motion: bool = false,
            limit_reverse_motion / with_limit_reverse_motion: bool = false,
            ignore_hardware_limits / with_ignore_hardware_limits: bool = false,
            ignore_software_limits / with_ignore_software_limits: bool = false,
            use_timesync / with_use_timesync: bool = false,
            update_freq_hz / with_update_freq_hz: f64 = 100.0,
        }
        send(this) [this.update_freq_hz, this.velocity, this.acceleration, this.enable_foc, this.feed_forward, this.slot, this.override_brake_dur_neutral, this.limit_forward_motion, this.limit_reverse_motion, this.ignore_hardware_limits, this.ignore_software_limits, this.use_timesync]
    }
}

control! {
    /// Requires Phoenix Pro; Request PID to target velocity with torque current feedforward.
    pub struct VelocityTorqueCurrentFOC("VelocityTorqueCurrentFOC", c_ctre_phoenix6_RequestControlVelocityTorqueCurrentFOC, TalonFx) {
        new {
            velocity / with_velocity: f64,
        }
        defaults {
            acceleration / with_acceleration: f64 = 0.0,
            feed_forward / with_feed_forward: f64 = 0.0,
            slot / with_slot: i32 = 0,
            override_coast_dur_neutral / with_override_coast_dur_neutral: bool = false,
            limit_forward_motion / with_limit_forward_motion: bool = false,
            limit_reverse_motion / with_limit_reverse_motion: bool = false,
            ignore_hardware_limits / with_ignore_hardware_limits: bool = false,
            ignore_software_limits / with_ignore_software_limits: bool = false,
            use_timesync / with_use_timesync: bool = false,
            update_freq_hz / with_update_freq_hz: f64 = 100.0,
        }
        send(this) [this.update_freq_hz, this.velocity, this.acceleration, this.feed_forward, this.slot, this.override_coast_dur_neutral, this.limit_forward_motion, this.limit_reverse_motion, this.ignore_hardware_limits, this.ignore_software_limits, this.use_timesync]
    }
}

control! {
    /// Requests Motion Magic to target a final position using a motion profile. Users can optionally provide a voltage feedforward.
    pub struct MotionMagicVoltage("MotionMagicVoltage", c_ctre_phoenix6_RequestControlMotionMagicVoltage, TalonFx) {
        new {
            position / with_position: f64,
        }
        defaults {
            enable_foc / with_enable_foc: bool = true,
            feed_forward / with_feed_forward: f64 = 0.0,
            slot / with_slot: i32 = 0,
            override_brake_dur_neutral / with_override_brake_dur_neutral: bool = false,
            limit_forward_motion / with_limit_forward_motion: bool = false,
            limit_reverse_motion / with_limit_reverse_motion: bool = false,
            ignore_hardware_limits / with_ignore_hardware_limits: bool = false,
            ignore_software_limits / with_ignore_software_limits: bool = false,
            use_timesync / with_use_timesync: bool = false,
            update_freq_hz / with_update_freq_hz: f64 = 100.0,
        }
        send(this) [this.update_freq_hz, this.position, this.enable_foc, this.feed_forward, this.slot, this.override_brake_dur_neutral, this.limit_forward_motion, this.limit_reverse_motion, this.ignore_hardware_limits, this.ignore_software_limits, this.use_timesync]
    }
}

control! {
    /// Requires Phoenix Pro; Requests Motion Magic to target a final position using a motion profile. Users can optionally provide a torque current feedforward.
    pub struct MotionMagicTorqueCurrentFOC("MotionMagicTorqueCurrentFOC", c_ctre_phoenix6_RequestControlMotionMagicTorqueCurrentFOC, TalonFx) {
        new {
            position / with_position: f64,
        }
        defaults {
            feed_forward / with_feed_forward: f64 = 0.0,
            slot / with_slot: i32 = 0,
            override_coast_dur_neutral / with_override_coast_dur_neutral: bool = false,
            limit_forward_motion / with_limit_forward_motion: bool = false,
            limit_reverse_motion / with_limit_reverse_motion: bool = false,
            ignore_hardware_limits / with_ignore_hardware_limits: bool = false,
            ignore_software_limits / with_ignore_software_limits: bool = false,
            use_timesync / with_use_timesync: bool = false,
            update_freq_hz / with_update_freq_hz: f64 = 100.0,
        }
        send(this) [this.update_freq_hz, this.position, this.feed_forward, this.slot, this.override_coast_dur_neutral, this.limit_forward_motion, this.limit_reverse_motion, this.ignore_hardware_limits, this.ignore_software_limits, this.use_timesync]
    }
}

control! {
    /// Requests Motion Magic to target a final velocity using a motion profile. This allows smooth transitions between velocity set points.  Users can optionally provide a voltage feedforward.
    pub struct MotionMagicVelocityVoltage("MotionMagicVelocityVoltage", c_ctre_phoenix6_RequestControlMotionMagicVelocityVoltage, TalonFx) {
        new {
            velocity / with_velocity: f64,
        }
        defaults {
            acceleration / with_acceleration: f64 = 0.0,
            enable_foc / with_enable_foc: bool = true,
            feed_forward / with_feed_forward: f64 = 0.0,
            slot / with_slot: i32 = 0,
            override_brake_dur_neutral / with_override_brake_dur_neutral: bool = false,
            limit_forward_motion / with_limit_forward_motion: bool = false,
            limit_reverse_motion / with_limit_reverse_motion: bool = false,
            ignore_hardware_limits / with_ignore_hardware_limits: bool = false,
            ignore_software_limits / with_ignore_software_limits: bool = false,
            use_timesync / with_use_timesync: bool = false,
            update_freq_hz / with_update_freq_hz: f64 = 100.0,
        }
        send(this) [this.update_freq_hz, this.velocity, this.acceleration, this.enable_foc, this.feed_forward, this.slot, this.override_brake_dur_neutral, this.limit_forward_motion, this.limit_reverse_motion, this.ignore_hardware_limits, this.ignore_software_limits, this.use_timesync]
    }
}

control! {
    /// Requests Motion Magic to target a final velocity using a motion profile. This allows smooth transitions between velocity set points.  Users can optionally provide a torque feedforward.
    pub struct MotionMagicVelocityTorqueCurrentFOC("MotionMagicVelocityTorqueCurrentFOC", c_ctre_phoenix6_RequestControlMotionMagicVelocityTorqueCurrentFOC, TalonFx) {
        new {
            velocity / with_velocity: f64,
        }
        defaults {
            acceleration / with_acceleration: f64 = 0.0,
            feed_forward / with_feed_forward: f64 = 0.0,
            slot / with_slot: i32 = 0,
            override_coast_dur_neutral / with_override_coast_dur_neutral: bool = false,
            limit_forward_motion / with_limit_forward_motion: bool = false,
            limit_reverse_motion / with_limit_reverse_motion: bool = false,
            ignore_hardware_limits / with_ignore_hardware_limits: bool = false,
            ignore_software_limits / with_ignore_software_limits: bool = false,
            use_timesync / with_use_timesync: bool = false,
            update_freq_hz / with_update_freq_hz: f64 = 100.0,
        }
        send(this) [this.update_freq_hz, this.velocity, this.acceleration, this.feed_forward, this.slot, this.override_coast_dur_neutral, this.limit_forward_motion, this.limit_reverse_motion, this.ignore_hardware_limits, this.ignore_software_limits, this.use_timesync]
    }
}

control! {
    /// Requires Phoenix Pro and CANivore; Requests Motion Magic to target a final position using a motion profile. This dynamic request allows runtime changes to Cruise Velocity, Acceleration, and (optional) Jerk.  Users can optionally provide a voltage feedforward.
    pub struct DynamicMotionMagicVoltage("DynamicMotionMagicVoltage", c_ctre_phoenix6_RequestControlDynamicMotionMagicVoltage, TalonFx) {
        new {
            position / with_position: f64,
            velocity / with_velocity: f64,
            acceleration / with_acceleration: f64,
        }
        defaults {
            jerk / with_jerk: f64 = 0.0,
            enable_foc / with_enable_foc: bool = true,
            feed_forward / with_feed_forward: f64 = 0.0,
            slot / with_slot: i32 = 0,
            override_brake_dur_neutral / with_override_brake_dur_neutral: bool = false,
            limit_forward_motion / with_limit_forward_motion: bool = false,
            limit_reverse_motion / with_limit_reverse_motion: bool = false,
            ignore_hardware_limits / with_ignore_hardware_limits: bool = false,
            ignore_software_limits / with_ignore_software_limits: bool = false,
            use_timesync / with_use_timesync: bool = false,
            update_freq_hz / with_update_freq_hz: f64 = 100.0,
        }
        send(this) [this.update_freq_hz, this.position, this.velocity, this.acceleration, this.jerk, this.enable_foc, this.feed_forward, this.slot, this.override_brake_dur_neutral, this.limit_forward_motion, this.limit_reverse_motion, this.ignore_hardware_limits, this.ignore_software_limits, this.use_timesync]
    }
}

control! {
    /// Requires Phoenix Pro and CANivore; Requests Motion Magic to target a final position using a motion profile. This dynamic request allows runtime changes to Cruise Velocity, Acceleration, and (optional) Jerk.  Users can optionally provide a torque current feedforward.
    pub struct DynamicMotionMagicTorqueCurrentFOC("DynamicMotionMagicTorqueCurrentFOC", c_ctre_phoenix6_RequestControlDynamicMotionMagicTorqueCurrentFOC, TalonFx) {
        new {
            position / with_position: f64,
            velocity / with_velocity: f64,
            acceleration / with_acceleration: f64,
        }
        defaults {
            jerk / with_jerk: f64 = 0.0,
            feed_forward / with_feed_forward: f64 = 0.0,
            slot / with_slot: i32 = 0,
            override_coast_dur_neutral / with_override_coast_dur_neutral: bool = false,
            limit_forward_motion / with_limit_forward_motion: bool = false,
            limit_reverse_motion / with_limit_reverse_motion: bool = false,
            ignore_hardware_limits / with_ignore_hardware_limits: bool = false,
            ignore_software_limits / with_ignore_software_limits: bool = false,
            use_timesync / with_use_timesync: bool = false,
            update_freq_hz / with_update_freq_hz: f64 = 100.0,
        }
        send(this) [this.update_freq_hz, this.position, this.velocity, this.acceleration, this.jerk, this.feed_forward, this.slot, this.override_coast_dur_neutral, this.limit_forward_motion, this.limit_reverse_motion, this.ignore_hardware_limits, this.ignore_software_limits, this.use_timesync]
    }
}

control! {
    /// Follow the motor output of another Talon.
    pub struct Follower("Follower", c_ctre_phoenix6_RequestControlFollower, TalonFx) {
        new {
            leader_id / with_leader_id: i32,
            motor_alignment / with_motor_alignment: MotorAlignmentValue,
        }
        defaults {
            update_freq_hz / with_update_freq_hz: f64 = 20.0,
        }
        send(this) [this.update_freq_hz, this.leader_id, this.motor_alignment.value()]
    }
}

control! {
    /// Request neutral output of actuator. The applied brake type is determined by the NeutralMode configuration.
    pub struct NeutralOut("NeutralOut", c_ctre_phoenix6_RequestControlNeutralOut, TalonFx) {
        new {
        }
        defaults {
            use_timesync / with_use_timesync: bool = false,
            update_freq_hz / with_update_freq_hz: f64 = 20.0,
        }
        send(this) [this.update_freq_hz, this.use_timesync]
    }
}

control! {
    /// Request coast neutral output of actuator.  The bridge is disabled and the rotor is allowed to coast.
    pub struct CoastOut("CoastOut", c_ctre_phoenix6_RequestControlCoastOut, TalonFx) {
        new {
        }
        defaults {
            use_timesync / with_use_timesync: bool = false,
            update_freq_hz / with_update_freq_hz: f64 = 20.0,
        }
        send(this) [this.update_freq_hz, this.use_timesync]
    }
}

control! {
    /// Applies full neutral-brake by shorting motor leads together.
    pub struct StaticBrake("StaticBrake", c_ctre_phoenix6_RequestControlStaticBrake, TalonFx) {
        new {
        }
        defaults {
            use_timesync / with_use_timesync: bool = false,
            update_freq_hz / with_update_freq_hz: f64 = 20.0,
        }
        send(this) [this.update_freq_hz, this.use_timesync]
    }
}

control! {
    /// Sets LEDs to a solid color.
    pub struct SolidColor("SolidColor", c_ctre_phoenix6_RequestControlSolidColor, Candle) {
        new {
            led_start_index / with_led_start_index: i32,
            led_end_index / with_led_end_index: i32,
        }
        defaults {
            color / with_color: RgbwColor = RgbwColor::default(),
        }
        send(this) [0.0, this.led_start_index, this.led_end_index, this.color.red, this.color.green, this.color.blue, this.color.white]
    }
}

control! {
    /// Animation that creates a rainbow throughout all the LEDs.
    pub struct RainbowAnimation("RainbowAnimation", c_ctre_phoenix6_RequestControlRainbowAnimation, Candle) {
        new {
            led_start_index / with_led_start_index: i32,
            led_end_index / with_led_end_index: i32,
        }
        defaults {
            slot / with_slot: i32 = 0,
            brightness / with_brightness: f64 = 1.0,
            direction / with_direction: AnimationDirectionValue = AnimationDirectionValue::Forward,
            frame_rate / with_frame_rate: f64 = 100.0,
            update_freq_hz / with_update_freq_hz: f64 = 20.0,
        }
        send(this) [this.update_freq_hz, this.led_start_index, this.led_end_index, this.slot, this.brightness, this.direction.value(), this.frame_rate]
    }
}

control! {
    /// Animation that fades all the LEDs of a strip simultaneously between Red, Green, and Blue.
    pub struct RgbFadeAnimation("RgbFadeAnimation", c_ctre_phoenix6_RequestControlRgbFadeAnimation, Candle) {
        new {
            led_start_index / with_led_start_index: i32,
            led_end_index / with_led_end_index: i32,
        }
        defaults {
            slot / with_slot: i32 = 0,
            brightness / with_brightness: f64 = 1.0,
            frame_rate / with_frame_rate: f64 = 100.0,
            update_freq_hz / with_update_freq_hz: f64 = 20.0,
        }
        send(this) [this.update_freq_hz, this.led_start_index, this.led_end_index, this.slot, this.brightness, this.frame_rate]
    }
}

control! {
    /// Animation that fades into and out of a specified color.
    pub struct SingleFadeAnimation("SingleFadeAnimation", c_ctre_phoenix6_RequestControlSingleFadeAnimation, Candle) {
        new {
            led_start_index / with_led_start_index: i32,
            led_end_index / with_led_end_index: i32,
        }
        defaults {
            slot / with_slot: i32 = 0,
            color / with_color: RgbwColor = RgbwColor::default(),
            frame_rate / with_frame_rate: f64 = 100.0,
            update_freq_hz / with_update_freq_hz: f64 = 20.0,
        }
        send(this) [this.update_freq_hz, this.led_start_index, this.led_end_index, this.slot, this.color.red, this.color.green, this.color.blue, this.color.white, this.frame_rate]
    }
}

control! {
    /// Animation that strobes the LEDs a specified color.
    pub struct StrobeAnimation("StrobeAnimation", c_ctre_phoenix6_RequestControlStrobeAnimation, Candle) {
        new {
            led_start_index / with_led_start_index: i32,
            led_end_index / with_led_end_index: i32,
        }
        defaults {
            slot / with_slot: i32 = 0,
            color / with_color: RgbwColor = RgbwColor::default(),
            frame_rate / with_frame_rate: f64 = 4.0,
            update_freq_hz / with_update_freq_hz: f64 = 20.0,
        }
        send(this) [this.update_freq_hz, this.led_start_index, this.led_end_index, this.slot, this.color.red, this.color.green, this.color.blue, this.color.white, this.frame_rate]
    }
}

control! {
    /// Animation that bounces a pocket of light across the LED strip.
    pub struct LarsonAnimation("LarsonAnimation", c_ctre_phoenix6_RequestControlLarsonAnimation, Candle) {
        new {
            led_start_index / with_led_start_index: i32,
            led_end_index / with_led_end_index: i32,
        }
        defaults {
            slot / with_slot: i32 = 0,
            color / with_color: RgbwColor = RgbwColor::default(),
            size / with_size: i32 = 3,
            bounce_mode / with_bounce_mode: LarsonBounceValue = LarsonBounceValue::Front,
            frame_rate / with_frame_rate: f64 = 25.0,
            update_freq_hz / with_update_freq_hz: f64 = 20.0,
        }
        send(this) [this.update_freq_hz, this.led_start_index, this.led_end_index, this.slot, this.color.red, this.color.green, this.color.blue, this.color.white, this.size, this.bounce_mode.value(), this.frame_rate]
    }
}

control! {
    /// Animation that looks similar to a flame flickering.
    pub struct FireAnimation("FireAnimation", c_ctre_phoenix6_RequestControlFireAnimation, Candle) {
        new {
            led_start_index / with_led_start_index: i32,
            led_end_index / with_led_end_index: i32,
        }
        defaults {
            slot / with_slot: i32 = 0,
            brightness / with_brightness: f64 = 1.0,
            direction / with_direction: AnimationDirectionValue = AnimationDirectionValue::Forward,
            sparking / with_sparking: f64 = 0.6,
            cooling / with_cooling: f64 = 0.3,
            frame_rate / with_frame_rate: f64 = 60.0,
            update_freq_hz / with_update_freq_hz: f64 = 20.0,
        }
        send(this) [this.update_freq_hz, this.led_start_index, this.led_end_index, this.slot, this.brightness, this.direction.value(), this.sparking, this.cooling, this.frame_rate]
    }
}

control! {
    /// Animation that gradually lights the entire LED strip one LED at a time.
    pub struct ColorFlowAnimation("ColorFlowAnimation", c_ctre_phoenix6_RequestControlColorFlowAnimation, Candle) {
        new {
            led_start_index / with_led_start_index: i32,
            led_end_index / with_led_end_index: i32,
        }
        defaults {
            slot / with_slot: i32 = 0,
            color / with_color: RgbwColor = RgbwColor::default(),
            direction / with_direction: AnimationDirectionValue = AnimationDirectionValue::Forward,
            frame_rate / with_frame_rate: f64 = 25.0,
            update_freq_hz / with_update_freq_hz: f64 = 20.0,
        }
        send(this) [this.update_freq_hz, this.led_start_index, this.led_end_index, this.slot, this.color.red, this.color.green, this.color.blue, this.color.white, this.direction.value(), this.frame_rate]
    }
}

control! {
    /// An empty animation, clearing any animation in the specified slot.
    pub struct EmptyAnimation("EmptyAnimation", c_ctre_phoenix6_RequestControlEmptyAnimation, Candle) {
        new {
            slot / with_slot: i32,
        }
        defaults {
            update_freq_hz / with_update_freq_hz: f64 = 0.0,
        }
        send(this) [this.update_freq_hz, this.slot]
    }
}

//! Tests that need the simulation natives. Run with:
//! `LD_LIBRARY_PATH=$PWD/native/linuxarm64 cargo test -p phoenix6 -- --ignored`
//! The gate skips them because the natives are not on its library path.

use phoenix6::configs::*;
use phoenix6::controls::*;
use phoenix6::error_report::native_status_name;
use phoenix6::signals::*;
use phoenix6::{CANcoder, CANdle, CanBus, Pigeon2, StatusCode, TalonFX};

fn setup() {
    wpilib::hal::initialize();
}

#[test]
#[ignore = "needs natives"]
fn default_talon_configuration_serializes_to_ctre_text() {
    setup();
    let text = TalonFXConfiguration::default().serialize();
    let lines: Vec<&str> = text.lines().collect();
    assert_eq!(lines.len(), 100);
    assert_eq!(
        &lines[..5],
        ["1901,s_0", "1904,s_0", "1905,f_0", "1910,f_1", "1911,f_-1"]
    );
    assert!(text.ends_with('\n'));
}

#[test]
#[ignore = "needs natives"]
fn serialized_kinds_follow_the_value_type() {
    setup();
    let slot = Slot0Configs::default()
        .with_k_p(1.5)
        .with_gravity_type(GravityTypeValue::Arm_Cosine);
    let text = slot.serialize();
    assert!(text.contains("1877,f_1.5\n"), "{text}");
    assert!(text.contains("s_1\n"));
    let audio = AudioConfigs::default().serialize();
    assert!(audio.lines().all(|l| l.contains(",b_")), "{audio}");
}

#[test]
#[ignore = "needs natives"]
fn applied_configuration_reads_back_from_a_simulated_device() {
    setup();
    let talon = TalonFX::new(30, CanBus::roborio()).expect("talon");
    let mut changed = TalonFXConfiguration::default();
    changed.motor_output.inverted = InvertedValue::Clockwise_Positive;
    changed.motor_output.neutral_mode = NeutralModeValue::Brake;
    changed.slot1.k_v = 0.5;
    changed.slot0.gravity_type = GravityTypeValue::Arm_Cosine;
    changed.hardware_limit_switch.forward_limit_enable = false;
    changed.feedback.feedback_remote_sensor_id = 9;
    changed.feedback.feedback_sensor_source = FeedbackSensorSourceValue::FusedCANcoder;
    assert_eq!(talon.configurator().apply(&changed), StatusCode::OK);

    let mut loaded = TalonFXConfiguration::default();
    assert_eq!(talon.configurator().refresh(&mut loaded), StatusCode::OK);
    // The device holds doubles as f32.
    for (a, b) in changed.items().iter().zip(loaded.items()) {
        match (a.value, b.value) {
            (ConfigValue::Double(x), ConfigValue::Double(y)) => {
                assert!((x - y).abs() < 1e-6, "{a:?} vs {b:?}")
            }
            (x, y) => assert_eq!(x, y, "{a:?}"),
        }
    }
}

#[test]
#[ignore = "needs natives"]
fn status_table_names_match_the_c_layer() {
    setup();
    for code in [
        StatusCode::OK,
        StatusCode::RxTimeout,
        StatusCode::InvalidDeviceId,
        StatusCode::CanMessageStale,
        StatusCode::FirmwareTooOld,
        StatusCode::NotSupported,
        StatusCode::LoggerNotRunning,
    ] {
        assert_eq!(native_status_name(code), code.name());
    }
}

#[test]
#[ignore = "needs natives"]
fn every_request_is_accepted_by_a_simulated_device() {
    setup();
    let bus = CanBus::roborio();
    let talon = TalonFX::new(11, bus.clone()).expect("talon");
    let requests: Vec<Box<dyn TalonFxControl>> = vec![
        Box::new(DutyCycleOut::new(0.1)),
        Box::new(TorqueCurrentFOC::new(1.0)),
        Box::new(VoltageOut::new(1.0)),
        Box::new(PositionVoltage::new(1.0)),
        Box::new(PositionTorqueCurrentFOC::new(1.0)),
        Box::new(VelocityVoltage::new(1.0)),
        Box::new(VelocityTorqueCurrentFOC::new(1.0)),
        Box::new(MotionMagicVoltage::new(1.0)),
        Box::new(MotionMagicTorqueCurrentFOC::new(1.0)),
        Box::new(MotionMagicVelocityVoltage::new(1.0)),
        Box::new(MotionMagicVelocityTorqueCurrentFOC::new(1.0)),
        Box::new(DynamicMotionMagicVoltage::new(1.0, 2.0, 3.0)),
        Box::new(DynamicMotionMagicTorqueCurrentFOC::new(1.0, 2.0, 3.0)),
        Box::new(Follower::new(12, MotorAlignmentValue::Opposed)),
        Box::new(NeutralOut::new()),
        Box::new(CoastOut::new()),
        Box::new(StaticBrake::new()),
    ];
    for request in &requests {
        assert_eq!(
            talon.set_control(request.as_ref()),
            StatusCode::OK,
            "{}",
            request.name()
        );
    }

    let candle = CANdle::new(11, bus).expect("candle");
    let animations: Vec<Box<dyn CandleControl>> = vec![
        Box::new(SolidColor::new(0, 7)),
        Box::new(RainbowAnimation::new(8, 15)),
        Box::new(RgbFadeAnimation::new(8, 15)),
        Box::new(SingleFadeAnimation::new(8, 15)),
        Box::new(StrobeAnimation::new(8, 15)),
        Box::new(LarsonAnimation::new(8, 15)),
        Box::new(FireAnimation::new(8, 15)),
        Box::new(ColorFlowAnimation::new(8, 15)),
        Box::new(EmptyAnimation::new(0)),
    ];
    for request in &animations {
        assert_eq!(
            candle.set_control(request.as_ref()),
            StatusCode::OK,
            "{}",
            request.name()
        );
    }
}

#[test]
#[ignore = "needs natives"]
fn sim_inputs_reach_the_signals() {
    setup();
    let bus = CanBus::roborio();
    let talon = TalonFX::new(21, bus.clone()).expect("talon");
    let cancoder = CANcoder::new(22, bus.clone()).expect("cancoder");
    let pigeon = Pigeon2::new(20, bus).expect("pigeon");
    std::thread::sleep(std::time::Duration::from_millis(250));

    let sim = talon.sim_state();
    sim.set_raw_rotor_position(2.5);
    sim.set_rotor_velocity(10.0);
    cancoder.sim_state().set_raw_position(0.25);
    pigeon.sim_state().set_raw_yaw(90.0);
    std::thread::sleep(std::time::Duration::from_millis(150));

    assert_eq!(talon.position(true).value(), 2.5);
    assert_eq!(talon.velocity(true).value(), 10.0);
    assert_eq!(cancoder.absolute_position(true).value(), 0.25);
    assert_eq!(pigeon.yaw(true).value(), 90.0);

    let mut reversed = talon.sim_state();
    reversed.orientation = phoenix6::sim::ChassisReference::Clockwise_Positive;
    reversed.set_raw_rotor_position(1.0);
    std::thread::sleep(std::time::Duration::from_millis(150));
    assert_eq!(talon.position(true).value(), -1.0);
}

#[test]
#[ignore = "needs natives"]
fn invalid_device_ids_are_rejected() {
    setup();
    assert_eq!(
        TalonFX::new(63, CanBus::roborio()).err(),
        Some(StatusCode::InvalidDeviceId)
    );
    assert_eq!(
        Pigeon2::new(-1, CanBus::roborio()).err(),
        Some(StatusCode::InvalidDeviceId)
    );
}

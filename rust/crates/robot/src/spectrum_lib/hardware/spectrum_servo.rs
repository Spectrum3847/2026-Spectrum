use commands::{CommandScheduler, Subsystem, SubsystemId};
use std::ops::Deref;
use std::rc::Rc;
use wpilib::pwm::Servo;

/// A [`Servo`] that is also a [`Subsystem`], so commands can require it. Dereferences to
/// the servo.
pub struct SpectrumServo {
    servo: Servo,
    id: SubsystemId,
}

impl SpectrumServo {
    /// Connects to a PWM port on the roboRIO and registers with the scheduler. Panics for
    /// a bad channel, where Java throws.
    ///
    /// `port` is the PWM channel, 0 to 9.
    pub fn new(port: i32) -> Rc<Self> {
        let servo = Servo::new(port)
            .unwrap_or_else(|e| panic!("Could not create a servo on PWM port {port}: {e}"));
        let spectrum_servo = Rc::new(Self {
            servo,
            id: SubsystemId::new(),
        });
        CommandScheduler::get_instance().register_subsystem(spectrum_servo.clone());
        spectrum_servo
    }
}

impl Deref for SpectrumServo {
    type Target = Servo;

    fn deref(&self) -> &Servo {
        &self.servo
    }
}

impl Subsystem for SpectrumServo {
    fn id(&self) -> SubsystemId {
        self.id
    }

    fn name(&self) -> String {
        "SpectrumServo".to_owned()
    }
}

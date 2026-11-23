/// Lombok getters and setters on the Java class are public fields.
#[derive(Debug, Clone)]
pub struct SpectrumCANcoderConfig {
    pub cancoder_id: i32,
    /// Rotor turns per CANcoder turn.
    pub rotor_to_sensor_ratio: f64,
    /// CANcoder turns per mechanism turn.
    pub sensor_to_mechanism_ratio: f64,
    /// Offset applied to the reading, in rotations.
    pub offset: f64,
    /// `false` builds no CANcoder, for sim or a robot without one.
    pub attached: bool,
    /// `true` makes clockwise rotation positive.
    pub inverted: bool,
}

impl SpectrumCANcoderConfig {
    pub fn new(
        rotor_to_sensor_ratio: f64,
        sensor_to_mechanism_ratio: f64,
        offset: f64,
        attached: bool,
        inverted: bool,
    ) -> Self {
        Self {
            cancoder_id: 0,
            rotor_to_sensor_ratio,
            sensor_to_mechanism_ratio,
            offset,
            attached,
            inverted,
        }
    }
}

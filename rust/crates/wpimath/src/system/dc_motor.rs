use crate::units;

/// DC motor model. Fields keep WPILib's names and SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DcMotor {
    pub nominal_voltage_volts: f64,
    pub stall_torque_newton_meters: f64,
    pub stall_current_amps: f64,
    pub free_current_amps: f64,
    pub free_speed_rad_per_sec: f64,
    pub r_ohms: f64,
    pub kv_rad_per_sec_per_volt: f64,
    pub kt_nm_per_amp: f64,
}

impl DcMotor {
    /// Torque and currents are per motor and are scaled by `num_motors`.
    pub fn new(
        nominal_voltage_volts: f64,
        stall_torque_newton_meters: f64,
        stall_current_amps: f64,
        free_current_amps: f64,
        free_speed_rad_per_sec: f64,
        num_motors: i32,
    ) -> Self {
        let n = f64::from(num_motors);
        let stall_torque_newton_meters = stall_torque_newton_meters * n;
        let stall_current_amps = stall_current_amps * n;
        let free_current_amps = free_current_amps * n;
        let r_ohms = nominal_voltage_volts / stall_current_amps;
        Self {
            nominal_voltage_volts,
            stall_torque_newton_meters,
            stall_current_amps,
            free_current_amps,
            free_speed_rad_per_sec,
            r_ohms,
            kv_rad_per_sec_per_volt: free_speed_rad_per_sec
                / (nominal_voltage_volts - r_ohms * free_current_amps),
            kt_nm_per_amp: stall_torque_newton_meters / stall_current_amps,
        }
    }

    pub fn kraken_x60_foc(num_motors: i32) -> Self {
        Self::new(
            12.0,
            9.37,
            483.0,
            2.0,
            units::rotations_per_minute_to_radians_per_second(5800.0),
            num_motors,
        )
    }

    pub fn current(&self, speed_rad_per_sec: f64, voltage_input_volts: f64) -> f64 {
        -1.0 / self.kv_rad_per_sec_per_volt / self.r_ohms * speed_rad_per_sec
            + 1.0 / self.r_ohms * voltage_input_volts
    }

    pub fn current_from_torque(&self, torque_nm: f64) -> f64 {
        torque_nm / self.kt_nm_per_amp
    }

    pub fn torque(&self, current_amps: f64) -> f64 {
        current_amps * self.kt_nm_per_amp
    }

    pub fn voltage(&self, torque_nm: f64, speed_rad_per_sec: f64) -> f64 {
        1.0 / self.kv_rad_per_sec_per_volt * speed_rad_per_sec
            + 1.0 / self.kt_nm_per_amp * self.r_ohms * torque_nm
    }

    pub fn speed(&self, torque_nm: f64, voltage_input_volts: f64) -> f64 {
        voltage_input_volts * self.kv_rad_per_sec_per_volt
            - 1.0 / self.kt_nm_per_amp * torque_nm * self.r_ohms * self.kv_rad_per_sec_per_volt
    }

    /// Motor behind a gearbox, seen from the output shaft.
    pub fn with_reduction(&self, gearbox_reduction: f64) -> Self {
        Self::new(
            self.nominal_voltage_volts,
            self.stall_torque_newton_meters * gearbox_reduction,
            self.stall_current_amps,
            self.free_current_amps,
            self.free_speed_rad_per_sec / gearbox_reduction,
            1,
        )
    }
}

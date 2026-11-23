//! DogLog's extras (`withLogExtras`): roboRIO stats, CAN bus status and the power
//! distribution every 20 ms, and the radio status every 5.81 s.

use crate::native;
use crate::value::{Entry, Value};
use std::io::{Read, Write};
use std::net::{SocketAddr, TcpStream, ToSocketAddrs};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::Duration;
use wpilib::notifier::Notifier;
use wpilib::power_distribution::PowerDistribution;
use wpilib::robot_controller;

const LOOP_PERIOD_SECONDS: f64 = 0.02;
const RADIO_LOG_PERIOD_SECONDS: f64 = 5.81;
const REQUEST_TIMEOUT: Duration = Duration::from_secs(1);

// WPILib unit names, which DogLog writes as the unit text.
const VOLTS: &str = "Volt";
const AMPS: &str = "Amp";
const CELSIUS: &str = "Celsius";
const WATTS: &str = "Watt";
const JOULES: &str = "Joule";
const MICROSECONDS: &str = "Microsecond";

/// Queues one record.
pub type Emit = Arc<dyn Fn(Entry) + Send + Sync>;

/// Runs `emit` for each roboRIO stat.
pub fn log_system(emit: &dyn Fn(&str, Value, Option<&str>)) {
    let int = |v: i64| Value::Integer(v);
    let flag = Value::Boolean;
    emit(
        "SystemStats/FPGAVersion",
        int(i64::from(native::fpga_version())),
        None,
    );
    emit(
        "SystemStats/FPGARevision",
        int(native::fpga_revision()),
        None,
    );
    emit(
        "SystemStats/SerialNumber",
        Value::String(robot_controller::serial_number()),
        None,
    );
    emit(
        "SystemStats/Comments",
        Value::String(native::comments()),
        None,
    );
    emit(
        "SystemStats/TeamNumber",
        int(i64::from(native::team_number())),
        None,
    );
    emit("SystemStats/FPGAButton", flag(native::fpga_button()), None);
    emit(
        "SystemStats/SystemActive",
        flag(native::system_active()),
        None,
    );
    emit(
        "SystemStats/BrownedOut",
        flag(robot_controller::is_browned_out()),
        None,
    );
    emit("SystemStats/RSLState", flag(native::rsl_state()), None);
    emit(
        "SystemStats/SystemTimeValid",
        flag(native::system_time_valid()),
        None,
    );
    emit(
        "SystemStats/BatteryVoltage",
        Value::Double(robot_controller::battery_voltage()),
        Some(VOLTS),
    );
    emit(
        "SystemStats/BatteryCurrent",
        Value::Double(robot_controller::input_current()),
        Some(AMPS),
    );
    let rails: [(&str, f64, f64, bool, i32); 3] = [
        (
            "3v3Rail",
            native::voltage_3v3(),
            native::current_3v3(),
            native::active_3v3(),
            native::current_faults_3v3(),
        ),
        (
            "5vRail",
            native::voltage_5v(),
            native::current_5v(),
            native::active_5v(),
            native::current_faults_5v(),
        ),
        (
            "6vRail",
            native::voltage_6v(),
            native::current_6v(),
            native::active_6v(),
            native::current_faults_6v(),
        ),
    ];
    for (name, volts, amps, active, faults) in rails {
        emit(
            &format!("SystemStats/{name}/Voltage"),
            Value::Double(volts),
            Some(VOLTS),
        );
        emit(
            &format!("SystemStats/{name}/Current"),
            Value::Double(amps),
            Some(AMPS),
        );
        emit(&format!("SystemStats/{name}/Active"), flag(active), None);
        emit(
            &format!("SystemStats/{name}/CurrentFaults"),
            int(i64::from(faults)),
            None,
        );
    }
    emit(
        "SystemStats/BrownoutVoltage",
        Value::Double(robot_controller::brownout_voltage()),
        Some(VOLTS),
    );
    emit(
        "SystemStats/CPUTempCelcius",
        Value::Double(native::cpu_temp()),
        Some(CELSIUS),
    );
}

pub fn log_can(emit: &dyn Fn(&str, Value, Option<&str>)) {
    let status = native::can_status();
    // Java logs the utilization as a float; this logs it as a double.
    emit(
        "SystemStats/CANBus/Utilization",
        Value::Double(f64::from(status.percent_bus_utilization)),
        None,
    );
    let int = |v: u32| Value::Integer(i64::from(v));
    emit(
        "SystemStats/CANBus/OffCount",
        int(status.bus_off_count),
        None,
    );
    emit(
        "SystemStats/CANBus/TxFullCount",
        int(status.tx_full_count),
        None,
    );
    emit(
        "SystemStats/CANBus/ReceiveErrorCount",
        int(status.receive_error_count),
        None,
    );
    emit(
        "SystemStats/CANBus/TransmitErrorCount",
        int(status.transmit_error_count),
        None,
    );
    emit(
        "SystemStats/EpochTimeMicros",
        Value::Integer(robot_controller::fpga_time() as i64),
        Some(MICROSECONDS),
    );
}

pub fn log_pdh(pdh: &PowerDistribution, emit: &dyn Fn(&str, Value, Option<&str>)) {
    let d = Value::Double;
    emit(
        "SystemStats/PowerDistribution/Temperature",
        d(pdh.temperature()),
        Some(CELSIUS),
    );
    emit(
        "SystemStats/PowerDistribution/Voltage",
        d(pdh.voltage()),
        Some(VOLTS),
    );
    emit(
        "SystemStats/PowerDistribution/ChannelCurrent",
        Value::DoubleArray(pdh.all_currents()),
        Some(AMPS),
    );
    emit(
        "SystemStats/PowerDistribution/TotalCurrent",
        d(pdh.total_current()),
        Some(AMPS),
    );
    emit(
        "SystemStats/PowerDistribution/TotalPower",
        d(pdh.total_power()),
        Some(WATTS),
    );
    emit(
        "SystemStats/PowerDistribution/TotalEnergy",
        d(pdh.total_energy()),
        Some(JOULES),
    );
    emit(
        "SystemStats/PowerDistribution/ChannelCount",
        Value::Integer(pdh.num_channels() as i64),
        None,
    );
}

/// `10.TE.AM.1` for team `TEAM`.
pub fn radio_host(team: i32) -> String {
    format!("10.{}.{}.1", team / 100, team % 100)
}

/// The body of `GET /status`, or `None` when the radio does not answer within a second.
/// HTTP/1.0 keeps the reply free of chunking.
pub fn fetch_radio_status(addr: impl ToSocketAddrs, host: &str) -> Option<String> {
    let addr: SocketAddr = addr.to_socket_addrs().ok()?.next()?;
    let mut stream = TcpStream::connect_timeout(&addr, REQUEST_TIMEOUT).ok()?;
    stream.set_read_timeout(Some(REQUEST_TIMEOUT)).ok()?;
    stream.set_write_timeout(Some(REQUEST_TIMEOUT)).ok()?;
    // One write: a server that answers and closes before reading a second piece resets us.
    let request = format!("GET /status HTTP/1.0\r\nHost: {host}\r\nConnection: close\r\n\r\n");
    stream.write_all(request.as_bytes()).ok()?;
    let mut reply = Vec::new();
    stream.read_to_end(&mut reply).ok()?;
    let split = reply.windows(4).position(|w| w == b"\r\n\r\n")?;
    Some(String::from_utf8_lossy(&reply[split + 4..]).into_owned())
}

fn log_radio(emit: &dyn Fn(&str, Value, Option<&str>)) {
    let host = radio_host(native::team_number());
    let status = fetch_radio_status((host.as_str(), 80), &host);
    emit(
        "RadioStatus/Connected",
        Value::Boolean(status.is_some()),
        None,
    );
    emit(
        "RadioStatus/StatusJson",
        Value::CustomString {
            value: status.unwrap_or_default(),
            type_name: "json".to_owned(),
        },
        None,
    );
}

/// The periodic extras. Started and stopped by the log options.
pub struct Extras {
    emit: Emit,
    pdh: Arc<Mutex<Option<PowerDistribution>>>,
    notifiers: Mutex<Option<[Notifier; 2]>>,
}

impl Extras {
    pub fn new(emit: Emit) -> Self {
        Self {
            emit,
            pdh: Arc::new(Mutex::new(None)),
            notifiers: Mutex::new(None),
        }
    }

    pub fn set_pdh(&self, pdh: Option<PowerDistribution>) {
        *self.pdh.lock().unwrap_or_else(PoisonError::into_inner) = pdh;
    }

    /// Starts or stops the notifiers. Without a loaded HAL there is nothing to time them.
    pub fn set_enabled(&self, enabled: bool) {
        let mut notifiers = self
            .notifiers
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        if !enabled {
            *notifiers = None;
            return;
        }
        if notifiers.is_some() || wpilib::hal::try_hal().is_none() {
            return;
        }
        let (emit, pdh) = (self.emit.clone(), self.pdh.clone());
        let main = Notifier::new(move || {
            // One timestamp for the whole tick, as in Java.
            let now = robot_controller::fpga_time();
            let sink = |k: &str, v: Value, u: Option<&str>| emit(entry(k, now, v, u));
            log_system(&sink);
            log_can(&sink);
            if let Some(pdh) = pdh.lock().unwrap_or_else(PoisonError::into_inner).as_ref() {
                log_pdh(pdh, &sink);
            }
        });
        let emit = self.emit.clone();
        let radio = Notifier::new(move || {
            let now = robot_controller::fpga_time();
            log_radio(&|k: &str, v: Value, u: Option<&str>| emit(entry(k, now, v, u)));
        });
        match (main, radio) {
            (Ok(main), Ok(radio)) => {
                main.set_name("DogLog extras logger");
                radio.set_name("DogLog radio logger");
                radio.start_periodic(RADIO_LOG_PERIOD_SECONDS);
                main.start_periodic(LOOP_PERIOD_SECONDS);
                *notifiers = Some([main, radio]);
            }
            (Err(e), _) | (_, Err(e)) => e.report(),
        }
    }
}

/// Builds the entry an extras record becomes.
pub fn entry(key: &str, ts: u64, value: Value, unit: Option<&str>) -> Entry {
    Entry {
        key: key.to_owned(),
        force_nt: false,
        timestamp: ts,
        value,
        unit: unit.map(str::to_owned),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;
    use std::thread;

    #[test]
    fn radio_address_follows_the_team_number() {
        assert_eq!(radio_host(3847), "10.38.47.1");
        assert_eq!(radio_host(254), "10.2.54.1");
        assert_eq!(radio_host(12), "10.0.12.1");
    }

    #[test]
    fn radio_status_is_the_response_body() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let server = thread::spawn(move || {
            let (mut conn, _) = listener.accept().unwrap();
            let mut request = [0u8; 256];
            let n = conn.read(&mut request).unwrap();
            let text = String::from_utf8_lossy(&request[..n]).into_owned();
            conn.write_all(b"HTTP/1.0 200 OK\r\nContent-Type: application/json\r\n\r\n{\"a\":1}")
                .unwrap();
            text
        });
        assert_eq!(
            fetch_radio_status(addr, "10.38.47.1").as_deref(),
            Some("{\"a\":1}")
        );
        let request = server.join().unwrap();
        assert!(request.starts_with("GET /status HTTP/1.0\r\nHost: 10.38.47.1\r\n"));
    }

    #[test]
    fn radio_status_is_none_when_nothing_listens() {
        let addr = {
            let listener = TcpListener::bind("127.0.0.1:0").unwrap();
            listener.local_addr().unwrap()
        };
        assert_eq!(fetch_radio_status(addr, "x"), None);
    }
}

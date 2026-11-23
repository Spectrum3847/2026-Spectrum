use std::collections::hash_map::RandomState;
use std::fmt::Display;
use std::fs::OpenOptions;
use std::hash::BuildHasher;
use std::io::{self, Write};
use std::sync::OnceLock;
use std::time::{SystemTime, UNIX_EPOCH};
use telemetry::telemetry;

const FILE_PATH: &str = "/home/lvuser/crash_tracking.txt";

/// The run id Java drew with `UUID.randomUUID()`, drawn once per run.
static RUN_INSTANCE_UUID: OnceLock<String> = OnceLock::new();

/// Appends caught crashes to a file on the roboRIO that never rolls over, so a fault
/// survives a reboot. Appends the run UUID, the marker, the date and the crash text.
///
/// Java takes a `Throwable` and prints its stack trace. Rust has no throwable type, so
/// callers pass the panic message or the error text.
pub fn log_throwable_crash(throwable: &dyn Display) {
    log_marker(FILE_PATH, "Exception", Some(&throwable.to_string()));
}

fn log_marker(path: &str, mark: &str, throwable: Option<&str>) {
    // Java: FileWriter reports any failure to open the file as FileNotFoundException.
    let file = match OpenOptions::new().append(true).create(true).open(path) {
        Ok(file) => file,
        Err(_) => {
            if wpilib::is_simulation() {
                telemetry::print(
                    "CrashTracker failed to save crash file to robot: running in simulation mode",
                );
            } else {
                telemetry::print(
                    "CrashTracker failed to save crash file to robot: path `/home/lvuser/crash_tracking.txt` not found",
                );
            }
            return;
        }
    };
    if let Err(e) = write_marker(file, mark, throwable, unix_seconds()) {
        eprintln!("{e}");
    }
}

fn write_marker(
    mut writer: impl Write,
    mark: &str,
    throwable: Option<&str>,
    unix_seconds: i64,
) -> io::Result<()> {
    write!(
        writer,
        "{}, {}, {}",
        run_instance_uuid(),
        mark,
        java_date_string(unix_seconds)
    )?;
    if let Some(throwable) = throwable {
        write!(writer, ", {throwable}")?;
    }
    writeln!(writer)
}

fn unix_seconds() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs() as i64)
}

fn run_instance_uuid() -> &'static str {
    RUN_INSTANCE_UUID.get_or_init(random_uuid)
}

/// A random version 4 UUID from the keys std seeds its hashers with.
fn random_uuid() -> String {
    let state = RandomState::new();
    let mut bytes = [0_u8; 16];
    bytes[..8].copy_from_slice(&state.hash_one(0_u64).to_be_bytes());
    bytes[8..].copy_from_slice(&state.hash_one(1_u64).to_be_bytes());
    bytes[6] = (bytes[6] & 0x0f) | 0x40;
    bytes[8] = (bytes[8] & 0x3f) | 0x80;
    let hex: String = bytes.iter().map(|b| format!("{b:02x}")).collect();
    format!(
        "{}-{}-{}-{}-{}",
        &hex[0..8],
        &hex[8..12],
        &hex[12..16],
        &hex[16..20],
        &hex[20..32]
    )
}

/// `Date.toString()` in UTC, which is the roboRIO's time zone: `Sat Nov 22 18:30:05 UTC 2026`.
fn java_date_string(unix_seconds: i64) -> String {
    const DAYS: [&str; 7] = ["Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"];
    const MONTHS: [&str; 12] = [
        "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec",
    ];
    let days = unix_seconds.div_euclid(86_400);
    let seconds_of_day = unix_seconds.rem_euclid(86_400);
    let (year, month, day) = civil_from_days(days);
    let weekday = (days + 4).rem_euclid(7) as usize;
    format!(
        "{} {} {:02} {:02}:{:02}:{:02} UTC {}",
        DAYS[weekday],
        MONTHS[(month - 1) as usize],
        day,
        seconds_of_day / 3600,
        seconds_of_day % 3600 / 60,
        seconds_of_day % 60,
        year
    )
}

/// Days since 1970-01-01 to a proleptic Gregorian (year, month, day).
fn civil_from_days(days: i64) -> (i64, i64, i64) {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let day_of_era = z.rem_euclid(146_097);
    let year_of_era =
        (day_of_era - day_of_era / 1460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
    let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
    let month_prime = (5 * day_of_year + 2) / 153;
    let day = day_of_year - (153 * month_prime + 2) / 5 + 1;
    let month = if month_prime < 10 {
        month_prime + 3
    } else {
        month_prime - 9
    };
    let year = year_of_era + era * 400 + i64::from(month <= 2);
    (year, month, day)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn date_string_matches_java_format() {
        assert_eq!(java_date_string(0), "Thu Jan 01 00:00:00 UTC 1970");
        assert_eq!(
            java_date_string(1_700_000_000),
            "Tue Nov 14 22:13:20 UTC 2023"
        );
        assert_eq!(
            java_date_string(951_782_400),
            "Tue Feb 29 00:00:00 UTC 2000"
        );
        assert_eq!(java_date_string(-1), "Wed Dec 31 23:59:59 UTC 1969");
    }

    #[test]
    fn uuid_is_a_version_4_uuid_and_stable_per_run() {
        let uuid = random_uuid();
        let parts: Vec<&str> = uuid.split('-').collect();
        let lengths: Vec<usize> = parts.iter().map(|p| p.len()).collect();
        assert_eq!(lengths, [8, 4, 4, 4, 12]);
        assert!(parts[2].starts_with('4'));
        assert!(matches!(parts[3].as_bytes()[0], b'8' | b'9' | b'a' | b'b'));
        assert_eq!(run_instance_uuid(), run_instance_uuid());
        assert_ne!(random_uuid(), random_uuid());
    }

    #[test]
    fn marker_line_has_uuid_mark_date_and_crash_text() {
        let mut out = Vec::new();
        write_marker(&mut out, "Exception", Some("boom"), 0).unwrap();
        let line = String::from_utf8(out).unwrap();
        assert_eq!(
            line,
            format!(
                "{}, Exception, Thu Jan 01 00:00:00 UTC 1970, boom\n",
                run_instance_uuid()
            )
        );
    }

    #[test]
    fn marker_without_a_throwable_has_no_trailing_field() {
        let mut out = Vec::new();
        write_marker(&mut out, "Mark", None, 0).unwrap();
        let line = String::from_utf8(out).unwrap();
        assert!(line.ends_with("Thu Jan 01 00:00:00 UTC 1970\n"));
    }

    #[test]
    fn crash_file_is_appended_not_replaced() {
        let path = std::env::temp_dir().join(format!("crash_tracker_test_{}.txt", random_uuid()));
        let path_str = path.to_str().unwrap();
        log_marker(path_str, "First", Some("one"));
        log_marker(path_str, "Second", Some("two"));
        let text = std::fs::read_to_string(&path).unwrap();
        std::fs::remove_file(&path).unwrap();
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 2);
        assert!(lines[0].contains(", First, ") && lines[0].ends_with(", one"));
        assert!(lines[1].contains(", Second, ") && lines[1].ends_with(", two"));
    }
}

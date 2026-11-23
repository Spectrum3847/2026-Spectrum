use std::fs;
use std::net::ToSocketAddrs;
use wpilib::driver_station;

const UNKNOWN: &str = "UNKNOWN";
const ATTEMPTS: usize = 10;

/// Reads the robot's own addresses and resolves hostnames to IP addresses. Each lookup
/// retries ten times, reporting a warning on every failure, before giving up.
///
/// Nothing in the robot code calls this. std has no network interface API, so the MAC
/// comes from Linux sysfs and is `UNKNOWN` elsewhere.
///
/// Returns the robot's MAC address as uppercase colon-separated hex, or `UNKNOWN` if it
/// cannot be read.
pub fn get_mac_address() -> String {
    for _ in 0..ATTEMPTS {
        match read_mac_address() {
            Some(mac) => return mac,
            None => driver_station::report_warning("Failed to get MAC, retrying"),
        }
    }
    driver_station::report_warning("Failed to get MAC");
    UNKNOWN.to_owned()
}

/// Returns the robot's own IP address, or `UNKNOWN` if it cannot be resolved.
pub fn get_ip_address() -> String {
    for _ in 0..ATTEMPTS {
        match local_host_name().and_then(|host| resolve(&host)) {
            Some(ip) => return ip,
            None => driver_station::report_warning("Failed to get IP, retrying"),
        }
    }
    driver_station::report_warning("Failed to get IP");
    UNKNOWN.to_owned()
}

/// Resolves a hostname or mDNS name such as `limelight.local`. Returns `UNKNOWN` if the
/// name never resolves.
pub fn get_ip_address_of(device_name_address: &str) -> String {
    for _ in 0..ATTEMPTS {
        match resolve(device_name_address) {
            Some(ip) => return ip,
            None => driver_station::report_warning("Failed to get IP, retrying"),
        }
    }
    driver_station::report_warning("Failed to get IP");
    UNKNOWN.to_owned()
}

fn resolve(host: &str) -> Option<String> {
    (host, 0)
        .to_socket_addrs()
        .ok()?
        .next()
        .map(|addr| addr.ip().to_string())
}

fn local_host_name() -> Option<String> {
    let name = fs::read_to_string("/proc/sys/kernel/hostname")
        .ok()
        .or_else(|| std::env::var("HOSTNAME").ok())
        .or_else(|| std::env::var("COMPUTERNAME").ok())?;
    let name = name.trim();
    (!name.is_empty()).then(|| name.to_owned())
}

/// The first non-loopback interface with a real address, wired interfaces first.
fn read_mac_address() -> Option<String> {
    let mut interfaces: Vec<(String, String)> = fs::read_dir("/sys/class/net")
        .ok()?
        .filter_map(Result::ok)
        .filter_map(|entry| {
            let name = entry.file_name().to_string_lossy().into_owned();
            let address = fs::read_to_string(entry.path().join("address")).ok()?;
            Some((name, normalize_mac(&address)?))
        })
        .collect();
    interfaces.sort_by_key(|(name, _)| {
        (
            !name.starts_with("eth") && !name.starts_with("en"),
            name.clone(),
        )
    });
    interfaces.into_iter().map(|(_, mac)| mac).next()
}

/// Uppercases a sysfs address. An all-zero address (loopback) is no address.
fn normalize_mac(text: &str) -> Option<String> {
    let mac = text.trim().to_uppercase();
    let valid = mac.split(':').count() == 6
        && mac
            .split(':')
            .all(|byte| byte.len() == 2 && byte.chars().all(|c| c.is_ascii_hexdigit()));
    (valid && mac.chars().any(|c| c != '0' && c != ':')).then_some(mac)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mac_is_uppercase_colon_separated() {
        assert_eq!(
            normalize_mac("aa:bb:cc:0d:ee:0f\n").as_deref(),
            Some("AA:BB:CC:0D:EE:0F")
        );
    }

    #[test]
    fn loopback_and_malformed_addresses_are_rejected() {
        assert_eq!(normalize_mac("00:00:00:00:00:00\n"), None);
        assert_eq!(normalize_mac("aa:bb:cc"), None);
        assert_eq!(normalize_mac("zz:bb:cc:dd:ee:ff"), None);
        assert_eq!(normalize_mac(""), None);
    }

    #[test]
    fn numeric_hosts_resolve_to_themselves() {
        assert_eq!(resolve("127.0.0.1").as_deref(), Some("127.0.0.1"));
    }
}

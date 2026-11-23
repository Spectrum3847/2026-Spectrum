//! Host detection and per-platform native library rules.

use crate::Result;
use std::path::{Path, PathBuf};

pub const ROBORIO: &str = "linuxathena";
pub const PLATFORMS: [&str; 5] = [
    ROBORIO,
    "linuxarm64",
    "linuxx86-64",
    "windowsx86-64",
    "osxuniversal",
];

pub fn rust_dir() -> PathBuf {
    let xtask = Path::new(env!("CARGO_MANIFEST_DIR"));
    xtask.parent().expect("xtask has a parent").to_path_buf()
}

/// Cargo's output folder. `build.target-dir` in a cargo config is not read.
pub fn target_dir() -> PathBuf {
    std::env::var_os("CARGO_TARGET_DIR").map_or_else(|| rust_dir().join("target"), PathBuf::from)
}

pub fn cargo() -> String {
    std::env::var("CARGO").unwrap_or_else(|_| "cargo".into())
}

pub fn host_platform() -> Result<&'static str> {
    platform_for(std::env::consts::OS, std::env::consts::ARCH)
}

pub fn platform_for(os: &str, arch: &str) -> Result<&'static str> {
    match (os, arch) {
        ("linux", "aarch64") => Ok("linuxarm64"),
        ("linux", "x86_64") => Ok("linuxx86-64"),
        ("windows", "x86_64") => Ok("windowsx86-64"),
        ("macos", _) => Ok("osxuniversal"),
        _ => Err(format!("unsupported host {os}-{arch}; pass a platform").into()),
    }
}

/// True for the shared libraries to keep from a native zip: no JNI, no debug files.
pub fn is_runtime_lib(platform: &str, file: &str) -> bool {
    if file.contains("jni") || file.ends_with(".debug") {
        return false;
    }
    if platform.starts_with("windows") {
        file.ends_with(".dll")
    } else if platform.starts_with("osx") {
        file.ends_with(".dylib")
    } else {
        file.contains(".so")
    }
}

/// Environment variable the dynamic loader reads for extra library folders.
pub fn search_path_var(os: &str) -> &'static str {
    match os {
        "windows" => "PATH",
        "macos" => "DYLD_LIBRARY_PATH",
        _ => "LD_LIBRARY_PATH",
    }
}

/// Shared library file name for a base name, same rules as `wpilib::native::file_name`.
pub fn lib_file(os: &str, base: &str) -> String {
    match os {
        "windows" => format!("{base}.dll"),
        "macos" => format!("lib{base}.dylib"),
        _ => format!("lib{base}.so"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn host_platforms() {
        assert_eq!(platform_for("linux", "aarch64").unwrap(), "linuxarm64");
        assert_eq!(platform_for("linux", "x86_64").unwrap(), "linuxx86-64");
        assert_eq!(platform_for("windows", "x86_64").unwrap(), "windowsx86-64");
        assert_eq!(platform_for("macos", "aarch64").unwrap(), "osxuniversal");
        assert_eq!(platform_for("macos", "x86_64").unwrap(), "osxuniversal");
        assert!(platform_for("windows", "aarch64").is_err());
    }

    #[test]
    fn lib_filter_per_platform() {
        assert!(is_runtime_lib("linuxarm64", "libwpiHal.so"));
        assert!(is_runtime_lib("linuxathena", "libntcore.so.2026"));
        assert!(!is_runtime_lib("linuxarm64", "libwpiHaljni.so"));
        assert!(!is_runtime_lib("linuxarm64", "libwpiHal.so.debug"));
        assert!(is_runtime_lib("windowsx86-64", "wpiHal.dll"));
        assert!(!is_runtime_lib("windowsx86-64", "wpiHal.lib"));
        assert!(!is_runtime_lib("windowsx86-64", "wpiHal.pdb"));
        assert!(!is_runtime_lib("windowsx86-64", "wpiHaljni.dll"));
        assert!(is_runtime_lib("osxuniversal", "libwpiHal.dylib"));
        assert!(!is_runtime_lib("osxuniversal", "libwpiHaljni.dylib"));
        assert!(!is_runtime_lib("osxuniversal", "LICENSE.md"));
    }

    #[test]
    fn lib_file_names() {
        assert_eq!(lib_file("linux", "wpiHal"), "libwpiHal.so");
        assert_eq!(lib_file("windows", "wpiHal"), "wpiHal.dll");
        assert_eq!(lib_file("macos", "wpiHal"), "libwpiHal.dylib");
    }

    #[test]
    fn search_path_vars() {
        assert_eq!(search_path_var("windows"), "PATH");
        assert_eq!(search_path_var("macos"), "DYLD_LIBRARY_PATH");
        assert_eq!(search_path_var("linux"), "LD_LIBRARY_PATH");
    }
}

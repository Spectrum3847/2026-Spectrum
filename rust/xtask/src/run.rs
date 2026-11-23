//! Tasks that run cargo: gate, build-roborio and sim.

use crate::Result;
use crate::deploy::TARGET;
use crate::fetch::ensure_toolchain;
use crate::platform::{cargo, host_platform, lib_file, rust_dir, search_path_var, target_dir};
use std::ffi::OsString;
use std::path::Path;
use std::process::Command;

fn run(mut cmd: Command) -> Result<()> {
    let status = cmd
        .status()
        .map_err(|e| format!("cannot run {:?}: {e}", cmd.get_program()))?;
    if status.success() {
        Ok(())
    } else {
        Err(format!("{:?} failed: {status}", cmd.get_program()).into())
    }
}

/// The checks every commit must pass. Stops at the first failure.
pub fn gate() -> Result<()> {
    for args in GATE_STEPS {
        let mut cmd = Command::new(cargo());
        cmd.current_dir(rust_dir()).args(*args);
        run(cmd)?;
    }
    Ok(())
}

const GATE_STEPS: &[&[&str]] = &[
    &["fmt", "--all", "--check"],
    &[
        "clippy",
        "--workspace",
        "--all-targets",
        "--",
        "-D",
        "warnings",
    ],
    &["test", "--workspace"],
];

#[cfg(test)]
mod gate_tests {
    #[test]
    fn gate_runs_format_lint_and_tests_in_order() {
        let names: Vec<&str> = super::GATE_STEPS.iter().map(|s| s[0]).collect();
        assert_eq!(names, ["fmt", "clippy", "test"]);
        assert!(super::GATE_STEPS[1].ends_with(&["-D", "warnings"]));
    }
}

/// Output: `<target dir>/armv7-unknown-linux-gnueabi/release/robot`.
pub fn build_roborio(extra: &[String]) -> Result<()> {
    let gcc = ensure_toolchain()?;
    let mut rustup = Command::new("rustup");
    rustup.args(["target", "add", TARGET]);
    run(rustup)?;

    // The stock target uses soft-float calls. Turning soft-float off emits VFP
    // instructions and keeps the soft calling convention the roboRIO libraries use.
    let mut cmd = Command::new(cargo());
    cmd.current_dir(rust_dir())
        .args(["build", "--release", "--target", TARGET, "-p", "robot"])
        .args(extra)
        .env("CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABI_LINKER", gcc)
        .env(
            "CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABI_RUSTFLAGS",
            "-C target-cpu=cortex-a9 -C target-feature=-soft-float",
        );
    run(cmd)?;
    println!(
        "built {}",
        target_dir().join(TARGET).join("release/robot").display()
    );
    Ok(())
}

/// `existing` with `dir` in front, in the host's path list format.
fn prepend_path(dir: &Path, existing: Option<OsString>) -> Result<OsString> {
    let old = existing.unwrap_or_default();
    let paths = std::iter::once(dir.to_path_buf()).chain(std::env::split_paths(&old));
    Ok(std::env::join_paths(paths)?)
}

/// What cargo runs for `sim`: the robot, or one example (optionally in one package).
pub fn sim_args(name: Option<&str>, package: Option<&str>) -> Vec<String> {
    let mut args = vec!["run".to_string()];
    match name {
        None | Some("robot") => args.extend(["-p".into(), package.unwrap_or("robot").into()]),
        Some(example) => {
            args.extend(["--example".into(), example.into()]);
            if let Some(p) = package {
                args.extend(["-p".into(), p.into()]);
            }
        }
    }
    args
}

/// Which HAL simulation extensions `sim` loads.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SimOptions {
    pub gui: bool,
    pub ds: bool,
}

impl Default for SimOptions {
    fn default() -> Self {
        Self {
            gui: true,
            ds: false,
        }
    }
}

/// Value of `HALSIM_EXTENSIONS` (the HAL splits it on `;` on Windows, `:` elsewhere), or
/// `None` when no extension is wanted. The libraries are found through the library path.
pub fn extensions_env(os: &str, options: SimOptions) -> Option<OsString> {
    let libs: Vec<String> = [
        (options.gui, "halsim_gui"),
        (options.ds, "halsim_ds_socket"),
    ]
    .iter()
    .filter(|(wanted, _)| *wanted)
    .map(|(_, base)| lib_file(os, base))
    .collect();
    let sep = if os == "windows" { ";" } else { ":" };
    (!libs.is_empty()).then(|| libs.join(sep).into())
}

pub fn sim(name: Option<&str>, package: Option<&str>, options: SimOptions) -> Result<()> {
    let platform = host_platform()?;
    let natives = rust_dir().join("native").join(platform);
    if !natives.is_dir() {
        return Err(format!(
            "missing {}; run `cargo xtask fetch-natives`",
            natives.display()
        )
        .into());
    }
    let var = search_path_var(std::env::consts::OS);
    let mut cmd = Command::new(cargo());
    cmd.current_dir(rust_dir())
        .args(sim_args(name, package))
        .env(var, prepend_path(&natives, std::env::var_os(var))?);
    match extensions_env(std::env::consts::OS, options) {
        Some(value) => cmd.env("HALSIM_EXTENSIONS", value),
        None => cmd.env_remove("HALSIM_EXTENSIONS"),
    };
    run(cmd)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sim_runs_robot_by_default() {
        assert_eq!(sim_args(None, None), ["run", "-p", "robot"]);
        assert_eq!(sim_args(Some("robot"), None), ["run", "-p", "robot"]);
    }

    #[test]
    fn sim_runs_named_example() {
        assert_eq!(
            sim_args(Some("hal_smoke"), None),
            ["run", "--example", "hal_smoke"]
        );
        assert_eq!(
            sim_args(Some("pathplanner_telemetry"), Some("pathplanner")),
            [
                "run",
                "--example",
                "pathplanner_telemetry",
                "-p",
                "pathplanner"
            ]
        );
    }

    #[test]
    fn extensions_per_os() {
        let both = SimOptions {
            gui: true,
            ds: true,
        };
        let value = |os, o| extensions_env(os, o).map(|v| v.into_string().unwrap());
        assert_eq!(
            value("linux", SimOptions::default()).as_deref(),
            Some("libhalsim_gui.so")
        );
        assert_eq!(
            value("linux", both).as_deref(),
            Some("libhalsim_gui.so:libhalsim_ds_socket.so")
        );
        assert_eq!(
            value("windows", both).as_deref(),
            Some("halsim_gui.dll;halsim_ds_socket.dll")
        );
        assert_eq!(
            value("macos", both).as_deref(),
            Some("libhalsim_gui.dylib:libhalsim_ds_socket.dylib")
        );
        let headless = SimOptions {
            gui: false,
            ds: true,
        };
        assert_eq!(
            value("linux", headless).as_deref(),
            Some("libhalsim_ds_socket.so")
        );
        let none = SimOptions {
            gui: false,
            ds: false,
        };
        assert_eq!(value("linux", none), None);
    }

    #[test]
    fn prepend_keeps_existing_entries() {
        let sep = if cfg!(windows) { ";" } else { ":" };
        let joined = prepend_path(Path::new("natives"), Some(format!("a{sep}b").into())).unwrap();
        let parts: Vec<_> = std::env::split_paths(&joined).collect();
        assert_eq!(
            parts,
            [Path::new("natives"), Path::new("a"), Path::new("b")]
        );
        let only = prepend_path(Path::new("natives"), None).unwrap();
        assert_eq!(
            std::env::split_paths(&only).next().unwrap(),
            Path::new("natives")
        );
    }
}

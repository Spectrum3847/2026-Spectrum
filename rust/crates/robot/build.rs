//! Writes `build_constants.rs` into `OUT_DIR`: Java's gversion `BuildConstants`. Works
//! without git, where the git fields read "unknown".

mod build_info;

use build_info::BuildInfo;
use std::path::PathBuf;
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

fn main() {
    let manifest_dir = PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").unwrap_or_default());
    let git = |args: &[&str]| {
        let output = Command::new("git")
            .args(args)
            .current_dir(&manifest_dir)
            .output()
            .ok()?;
        output
            .status
            .success()
            .then(|| String::from_utf8_lossy(&output.stdout).trim().to_owned())
    };

    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |elapsed| elapsed.as_secs() as i64);
    let fallback_name = std::env::var("CARGO_PKG_NAME").unwrap_or_default();
    let info = BuildInfo::collect(&fallback_name, now, git);

    let out_dir = PathBuf::from(std::env::var_os("OUT_DIR").unwrap_or_default());
    std::fs::write(out_dir.join("build_constants.rs"), info.render())
        .expect("write build_constants.rs");

    // Commits and edits change the SHA and the dirty flag.
    println!("cargo:rerun-if-changed=src");
    println!("cargo:rerun-if-changed=build_info.rs");
    for name in ["HEAD", "index"] {
        if let Some(path) = git(&["rev-parse", "--git-path", name]) {
            println!("cargo:rerun-if-changed={path}");
        }
    }
}

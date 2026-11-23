//! Deploy that follows GradleRIO's steps: stream a tar over `ssh` and unpack it on the roboRIO.

use crate::Result;
use crate::platform::{ROBORIO, rust_dir, target_dir};
use flate2::{Compression, write::GzEncoder};
use std::fs::{self, File};
use std::io::Write;
use std::path::Path;
use std::process::{Command, Stdio};

pub const TARGET: &str = "armv7-unknown-linux-gnueabi";

/// Runs on the roboRIO with the tar on stdin.
const REMOTE_SCRIPT: &str = r##"set -e
. /etc/profile.d/natinst-path.sh
/usr/local/frc/bin/frcKillRobot.sh -t 2>/dev/null || true
work=$(mktemp -d)
tar -xz -C "$work"
lib=/usr/local/frc/third-party/lib
mkdir -p "$lib"
cp "$work"/lib/* "$lib"/
chmod -R 777 "$lib" || true
chown -R lvuser:ni "$lib"
ldconfig
cp "$work"/frcUserProgram /home/lvuser/frcUserProgram
chmod +x /home/lvuser/frcUserProgram
chown lvuser /home/lvuser/frcUserProgram
setcap cap_sys_nice+eip /home/lvuser/frcUserProgram
mkdir -p /home/lvuser/deploy
cp -r "$work"/deploy/. /home/lvuser/deploy/
chown -R lvuser /home/lvuser/deploy
echo "\"/home/lvuser/frcUserProgram\" " > /home/lvuser/robotCommand
chmod +x /home/lvuser/robotCommand
chown lvuser /home/lvuser/robotCommand
rm -rf "$work"
sync
noapp=$(/usr/local/natinst/bin/nirtcfg --file=/etc/natinst/share/ni-rt.ini --get section=systemsettings,token=NoApp.enabled,value=unknown)
if [ "$noapp" = "true" ]; then
    echo "NoApp is set; disable it with the imaging tool or hold the User button 5 s" >&2
    exit 1
fi
/usr/local/frc/bin/frcKillRobot.sh -t -r 2>/dev/null
echo deployed"##;

fn tar_file(
    tar: &mut tar::Builder<GzEncoder<Vec<u8>>>,
    path: &Path,
    name: &str,
    mode: u32,
) -> Result<()> {
    let mut file = File::open(path)?;
    let mut header = tar::Header::new_gnu();
    header.set_size(file.metadata()?.len());
    header.set_mode(mode);
    tar.append_data(&mut header, name, &mut file)?;
    Ok(())
}

/// The archive the remote script unpacks: `frcUserProgram`, `lib/*`, `deploy/**`.
pub fn build_tar(bin: &Path, natives: &Path, deploy: &Path) -> Result<Vec<u8>> {
    let mut tar = tar::Builder::new(GzEncoder::new(Vec::new(), Compression::default()));
    tar_file(&mut tar, bin, "frcUserProgram", 0o755)?;
    let mut libs: Vec<_> = fs::read_dir(natives)?
        .filter_map(|e| e.ok())
        .filter(|e| e.file_name().to_string_lossy().contains(".so"))
        .collect();
    libs.sort_by_key(|e| e.file_name());
    for lib in libs {
        let name = format!("lib/{}", lib.file_name().to_string_lossy());
        tar_file(&mut tar, &lib.path(), &name, 0o644)?;
    }
    tar.append_dir_all("deploy", deploy)?;
    Ok(tar.into_inner()?.finish()?)
}

fn null_device(os: &str) -> &'static str {
    if os == "windows" { "NUL" } else { "/dev/null" }
}

// roboRIO host keys change on every reimage, so skip known_hosts.
fn ssh_args(host: &str, os: &str) -> Vec<String> {
    vec![
        "-o".into(),
        "StrictHostKeyChecking=no".into(),
        "-o".into(),
        format!("UserKnownHostsFile={}", null_device(os)),
        format!("admin@{host}"),
        REMOTE_SCRIPT.into(),
    ]
}

/// The roboRIO admin account has an empty password: press Enter if ssh asks.
pub fn deploy(host: &str) -> Result<()> {
    let rust = rust_dir();
    let bin = target_dir().join(TARGET).join("release/robot");
    let natives = rust.join("native").join(ROBORIO);
    let deploy_dir = rust.join("../src/main/deploy");
    if !bin.is_file() {
        return Err(format!("missing {}; run `cargo xtask build-roborio`", bin.display()).into());
    }
    if !natives.is_dir() {
        return Err(format!(
            "missing {}; run `cargo xtask fetch-natives {ROBORIO}`",
            natives.display()
        )
        .into());
    }
    let tar = build_tar(&bin, &natives, &deploy_dir)?;

    let mut child = Command::new("ssh")
        .args(ssh_args(host, std::env::consts::OS))
        .stdin(Stdio::piped())
        .spawn()
        .map_err(|e| format!("cannot run ssh (install the OpenSSH client): {e}"))?;
    let sent = child.stdin.take().expect("piped stdin").write_all(&tar);
    let status = child.wait()?;
    if !status.success() {
        return Err(format!("deploy failed: ssh {status}").into());
    }
    sent?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use flate2::read::GzDecoder;
    use std::io::Read;

    #[test]
    fn tar_has_program_libs_and_deploy_files() {
        let dir = crate::scratch_dir("deploy-tar");
        let (bin, natives, deploy) = (dir.join("robot"), dir.join("natives"), dir.join("deploy"));
        fs::create_dir_all(natives.as_path()).unwrap();
        fs::create_dir_all(deploy.join("pathplanner")).unwrap();
        fs::write(&bin, b"elf").unwrap();
        fs::write(natives.join("libwpiHal.so"), b"a").unwrap();
        fs::write(natives.join("libCTRE_PhoenixTools.so"), b"b").unwrap();
        fs::write(natives.join("notes.txt"), b"skip").unwrap();
        fs::write(deploy.join("pathplanner/a.path"), b"{}").unwrap();

        let bytes = build_tar(&bin, &natives, &deploy).unwrap();
        fs::remove_dir_all(&dir).unwrap();

        let mut archive = tar::Archive::new(GzDecoder::new(bytes.as_slice()));
        let mut entries = Vec::new();
        for entry in archive.entries().unwrap() {
            let mut entry = entry.unwrap();
            let name = entry.path().unwrap().to_string_lossy().into_owned();
            let mode = entry.header().mode().unwrap();
            let mut body = String::new();
            let _ = entry.read_to_string(&mut body);
            entries.push((name, mode, body));
        }
        let names: Vec<_> = entries.iter().map(|e| e.0.as_str()).collect();
        assert_eq!(
            names,
            [
                "frcUserProgram",
                "lib/libCTRE_PhoenixTools.so",
                "lib/libwpiHal.so",
                "deploy/",
                "deploy/pathplanner",
                "deploy/pathplanner/a.path",
            ]
        );
        assert_eq!(entries[0].1 & 0o777, 0o755);
        assert_eq!(entries[0].2, "elf");
    }

    #[test]
    fn remote_script_runs_every_step() {
        for step in [
            "frcKillRobot.sh -t 2>/dev/null || true",
            "tar -xz -C \"$work\"",
            "ldconfig",
            "setcap cap_sys_nice+eip /home/lvuser/frcUserProgram",
            "> /home/lvuser/robotCommand",
            "NoApp.enabled",
            "frcKillRobot.sh -t -r",
        ] {
            assert!(REMOTE_SCRIPT.contains(step), "missing {step}");
        }
    }

    #[test]
    fn ssh_args_per_os() {
        let unix = ssh_args("10.38.47.2", "linux");
        assert!(unix.contains(&"UserKnownHostsFile=/dev/null".to_string()));
        assert!(unix.contains(&"admin@10.38.47.2".to_string()));
        assert!(ssh_args("h", "windows").contains(&"UserKnownHostsFile=NUL".to_string()));
        assert_eq!(unix.last().unwrap(), REMOTE_SCRIPT);
    }
}

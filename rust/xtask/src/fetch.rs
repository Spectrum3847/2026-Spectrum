//! Downloads: WPILib and CTRE natives, and the roboRIO cross toolchain.

use crate::Result;
use crate::platform::{PLATFORMS, ROBORIO, is_runtime_lib, rust_dir};
use flate2::read::GzDecoder;
use std::fs::{self, File};
use std::io::{Read, Seek};
use std::path::{Path, PathBuf};

const WPILIB: &str = "2026.2.1";
const PHOENIX: &str = "26.3.0";
const WPI_MAVEN: &str = "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first";
const CTRE_MAVEN: &str = "https://maven.ctr-electronics.com/release/com/ctre/phoenix6";

const TOOLCHAIN_RELEASE: &str = "v2025-2";
const TOOLCHAIN_GCC: &str = "12.1.0";

// tools-sim links against every sim device library.
const CTRE_SIM: [&str; 11] = [
    "tools-sim",
    "simTalonSRX",
    "simVictorSPX",
    "simPigeonIMU",
    "simProTalonFX",
    "simProTalonFXS",
    "simProCANcoder",
    "simProPigeon2",
    "simProCANrange",
    "simProCANdi",
    "simProCANdle",
];

pub fn native_urls(platform: &str) -> Vec<String> {
    let wpi = |group_artifact: &str| {
        let name = group_artifact.rsplit('/').next().unwrap_or(group_artifact);
        format!("{WPI_MAVEN}/{group_artifact}/{WPILIB}/{name}-{WPILIB}-{platform}.zip")
    };
    let mut urls: Vec<String> = [
        "wpiutil/wpiutil-cpp",
        "wpinet/wpinet-cpp",
        "ntcore/ntcore-cpp",
        "hal/hal-cpp",
    ]
    .iter()
    .map(|a| wpi(a))
    .collect();
    if platform == ROBORIO {
        urls.push(format!(
            "{CTRE_MAVEN}/tools/{PHOENIX}/tools-{PHOENIX}-{platform}.zip"
        ));
    } else {
        // halsim_gui links against wpimath.
        for a in [
            "wpimath/wpimath-cpp",
            "halsim/halsim_ds_socket",
            "halsim/halsim_gui",
        ] {
            urls.push(wpi(a));
        }
        for a in CTRE_SIM {
            urls.push(format!(
                "{CTRE_MAVEN}/sim/{a}/{PHOENIX}/{a}-{PHOENIX}-{platform}.zip"
            ));
        }
    }
    urls
}

fn open_url(url: &str) -> Result<impl Read + use<>> {
    eprintln!("downloading {url}");
    Ok(ureq::get(url).call()?.into_body().into_reader())
}

/// Copies the runtime libraries of one zip into `out`, flat. Returns the count.
fn extract_libs(zip: impl Read + Seek, platform: &str, out: &Path) -> Result<usize> {
    let mut archive = zip::ZipArchive::new(zip)?;
    let mut count = 0;
    for i in 0..archive.len() {
        let mut entry = archive.by_index(i)?;
        let Some(name) = entry
            .enclosed_name()
            .and_then(|p| p.file_name().map(|n| n.to_string_lossy().into_owned()))
        else {
            continue;
        };
        if entry.is_dir() || !is_runtime_lib(platform, &name) {
            continue;
        }
        std::io::copy(&mut entry, &mut File::create(out.join(name))?)?;
        count += 1;
    }
    Ok(count)
}

pub fn fetch_natives(platform: &str) -> Result<()> {
    if !PLATFORMS.contains(&platform) {
        return Err(format!("unknown platform {platform}; use one of {PLATFORMS:?}").into());
    }
    let out = rust_dir().join("native").join(platform);
    fs::create_dir_all(&out)?;
    let tmp = out.join(".fetch.zip");
    let result = native_urls(platform).iter().try_for_each(|url| {
        std::io::copy(&mut open_url(url)?, &mut File::create(&tmp)?)?;
        extract_libs(File::open(&tmp)?, platform, &out).map(drop)
    });
    let _ = fs::remove_file(&tmp);
    result?;

    println!("natives for {platform} in {}:", out.display());
    let mut names: Vec<_> = fs::read_dir(&out)?
        .filter_map(|e| e.ok()?.file_name().into_string().ok())
        .collect();
    names.sort();
    names.iter().for_each(|n| println!("{n}"));
    Ok(())
}

/// The toolchain archive name for a host, and whether it is a zip.
fn toolchain_asset(os: &str, arch: &str) -> Result<(String, bool)> {
    let (host, zip) = match (os, arch) {
        ("linux", "aarch64") => ("aarch64-bookworm-linux-gnu", false),
        ("linux", "x86_64") => ("x86_64-linux-gnu", false),
        ("macos", "aarch64") => ("arm64-apple-darwin", false),
        ("macos", "x86_64") => ("x86_64-apple-darwin", false),
        ("windows", "x86_64") => ("x86_64-w64-mingw32", true),
        _ => return Err(format!("unsupported host {os}-{arch}").into()),
    };
    let ext = if zip { "zip" } else { "tgz" };
    Ok((
        format!("cortexa9_vfpv3-roborio-academic-2025-{host}-Toolchain-{TOOLCHAIN_GCC}.{ext}"),
        zip,
    ))
}

pub fn toolchain_url(os: &str, arch: &str) -> Result<String> {
    let (name, _) = toolchain_asset(os, arch)?;
    Ok(format!(
        "https://github.com/wpilibsuite/opensdk/releases/download/{TOOLCHAIN_RELEASE}/{name}"
    ))
}

/// Path of the roboRIO cross compiler, downloaded into `native/toolchain` on first use.
pub fn ensure_toolchain() -> Result<PathBuf> {
    let (os, arch) = (std::env::consts::OS, std::env::consts::ARCH);
    let dir = rust_dir().join("native").join("toolchain");
    let gcc = dir.join("roborio-academic/bin").join(format!(
        "arm-frc2025-linux-gnueabi-gcc{}",
        std::env::consts::EXE_SUFFIX
    ));
    if gcc.exists() {
        return Ok(gcc);
    }
    let url = toolchain_url(os, arch)?;
    fs::create_dir_all(&dir)?;
    if toolchain_asset(os, arch)?.1 {
        let tmp = dir.join(".toolchain.zip");
        let result = (|| -> Result<()> {
            std::io::copy(&mut open_url(&url)?, &mut File::create(&tmp)?)?;
            zip::ZipArchive::new(File::open(&tmp)?)?.extract(&dir)?;
            Ok(())
        })();
        let _ = fs::remove_file(&tmp);
        result?;
    } else {
        tar::Archive::new(GzDecoder::new(open_url(&url)?)).unpack(&dir)?;
    }
    if !gcc.exists() {
        return Err(format!("{} missing after extracting the toolchain", gcc.display()).into());
    }
    Ok(gcc)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Cursor, Write};

    #[test]
    fn roborio_urls() {
        let urls = native_urls("linuxathena");
        assert_eq!(urls.len(), 5);
        assert_eq!(
            urls[0],
            "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/wpiutil/wpiutil-cpp/2026.2.1/wpiutil-cpp-2026.2.1-linuxathena.zip"
        );
        assert_eq!(
            urls[4],
            "https://maven.ctr-electronics.com/release/com/ctre/phoenix6/tools/26.3.0/tools-26.3.0-linuxathena.zip"
        );
    }

    #[test]
    fn desktop_urls_include_sim() {
        let urls = native_urls("windowsx86-64");
        assert_eq!(urls.len(), 4 + 3 + 11);
        assert!(
            urls.iter()
                .any(|u| u.ends_with("halsim_ds_socket-2026.2.1-windowsx86-64.zip"))
        );
        assert!(
            urls.iter()
                .any(|u| u.ends_with("halsim_gui-2026.2.1-windowsx86-64.zip"))
        );
        assert!(
            urls.iter()
                .any(|u| u.ends_with("wpimath-cpp-2026.2.1-windowsx86-64.zip"))
        );
        assert!(
            urls.iter()
                .any(|u| u.ends_with("tools-sim-26.3.0-windowsx86-64.zip"))
        );
    }

    #[test]
    fn toolchain_per_host() {
        let url = |os, arch| toolchain_url(os, arch).unwrap();
        assert!(
            url("linux", "x86_64")
                .ends_with("roborio-academic-2025-x86_64-linux-gnu-Toolchain-12.1.0.tgz")
        );
        assert!(
            url("linux", "aarch64")
                .ends_with("2025-aarch64-bookworm-linux-gnu-Toolchain-12.1.0.tgz")
        );
        assert!(url("macos", "aarch64").ends_with("2025-arm64-apple-darwin-Toolchain-12.1.0.tgz"));
        assert!(url("macos", "x86_64").ends_with("2025-x86_64-apple-darwin-Toolchain-12.1.0.tgz"));
        assert!(url("windows", "x86_64").ends_with("2025-x86_64-w64-mingw32-Toolchain-12.1.0.zip"));
        assert!(toolchain_url("windows", "aarch64").is_err());
    }

    #[test]
    fn extract_keeps_only_runtime_libs_flat() {
        let mut buf = Cursor::new(Vec::new());
        {
            let mut zip = zip::ZipWriter::new(&mut buf);
            let opts = zip::write::SimpleFileOptions::default();
            for name in [
                "LICENSE.md",
                "windows/x86-64/shared/wpiHal.dll",
                "windows/x86-64/shared/wpiHal.lib",
                "windows/x86-64/shared/wpiHal.pdb",
                "windows/x86-64/shared/wpiHaljni.dll",
            ] {
                zip.start_file(name, opts).unwrap();
                zip.write_all(b"x").unwrap();
            }
            zip.finish().unwrap();
        }
        let dir = crate::scratch_dir("extract");
        let n = extract_libs(Cursor::new(buf.into_inner()), "windowsx86-64", &dir).unwrap();
        let names: Vec<_> = fs::read_dir(&dir)
            .unwrap()
            .map(|e| e.unwrap().file_name().into_string().unwrap())
            .collect();
        fs::remove_dir_all(&dir).unwrap();
        assert_eq!((n, names), (1, vec!["wpiHal.dll".to_string()]));
    }
}

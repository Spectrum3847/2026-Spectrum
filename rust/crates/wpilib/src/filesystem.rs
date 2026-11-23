//! Java `Filesystem`: where deploy files live.

use crate::hal;
use std::env;
use std::ffi::OsString;
use std::path::{Path, PathBuf};

const ROBORIO_DEPLOY: &str = "/home/lvuser/deploy";

/// Env var that overrides the deploy folder in simulation.
pub const DEPLOY_DIR_VAR: &str = "SPECTRUM_DEPLOY_DIR";

/// The deploy folder: `/home/lvuser/deploy` on the roboRIO. In simulation, `$SPECTRUM_DEPLOY_DIR`,
/// else the nearest `src/main/deploy` above the working directory (the repo's folder).
pub fn deploy_directory() -> PathBuf {
    deploy_directory_for(
        hal::is_simulation(),
        env::var_os(DEPLOY_DIR_VAR),
        env::current_dir().ok(),
    )
}

fn deploy_directory_for(
    simulation: bool,
    override_dir: Option<OsString>,
    cwd: Option<PathBuf>,
) -> PathBuf {
    if !simulation {
        return PathBuf::from(ROBORIO_DEPLOY);
    }
    if let Some(dir) = override_dir.filter(|d| !d.is_empty()) {
        return PathBuf::from(dir);
    }
    let cwd = cwd.unwrap_or_default();
    let relative = Path::new("src").join("main").join("deploy");
    cwd.ancestors()
        .map(|dir| dir.join(&relative))
        .find(|dir| dir.is_dir())
        .unwrap_or_else(|| cwd.join(&relative))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    struct TempDir(PathBuf);

    impl TempDir {
        fn new(name: &str) -> Self {
            let dir = env::temp_dir().join(format!("wpilib-fs-{name}-{}", std::process::id()));
            let _ = fs::remove_dir_all(&dir);
            fs::create_dir_all(&dir).unwrap();
            Self(dir)
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn roborio_ignores_the_override() {
        let dir = deploy_directory_for(false, Some("/elsewhere".into()), Some("/x".into()));
        assert_eq!(dir, Path::new(ROBORIO_DEPLOY));
    }

    #[test]
    fn simulation_prefers_the_override() {
        let dir = deploy_directory_for(true, Some("/custom/deploy".into()), Some("/x".into()));
        assert_eq!(dir, Path::new("/custom/deploy"));
    }

    #[test]
    fn empty_override_is_ignored() {
        let temp = TempDir::new("empty");
        let dir = deploy_directory_for(true, Some(OsString::new()), Some(temp.0.clone()));
        assert_eq!(dir, temp.0.join("src/main/deploy"));
    }

    #[test]
    fn simulation_finds_the_repo_deploy_folder_from_a_subfolder() {
        let temp = TempDir::new("walk");
        fs::create_dir_all(temp.0.join("src/main/deploy")).unwrap();
        fs::create_dir_all(temp.0.join("rust/crates")).unwrap();
        let dir = deploy_directory_for(true, None, Some(temp.0.join("rust/crates")));
        assert_eq!(dir, temp.0.join("src/main/deploy"));
    }

    #[test]
    fn simulation_without_a_deploy_folder_falls_back_to_the_working_directory() {
        let temp = TempDir::new("none");
        let dir = deploy_directory_for(true, None, Some(temp.0.clone()));
        assert_eq!(dir, temp.0.join("src/main/deploy"));
    }
}

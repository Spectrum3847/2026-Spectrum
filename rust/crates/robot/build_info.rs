//! Git and clock facts that `build.rs` turns into `BuildConstants`, Java's gversion output.
//! The lib compiles this file too, for its tests.

pub const UNKNOWN: &str = "unknown";

/// Java's `BuildConstants.DIRTY` is 0 or 1. -1 means git could not say.
pub const DIRTY_UNKNOWN: i32 = -1;

#[derive(Debug, PartialEq, Eq)]
pub struct BuildInfo {
    pub project_name: String,
    pub build_date: String,
    pub git_sha: String,
    pub git_date: String,
    pub git_branch: String,
    pub dirty: i32,
}

impl BuildInfo {
    /// `git` runs one git command in the repo and returns its trimmed stdout, or `None` when
    /// git is missing or the command fails. Every git field then falls back to [`UNKNOWN`].
    pub fn collect(
        fallback_project_name: &str,
        now_unix_seconds: i64,
        git: impl Fn(&[&str]) -> Option<String>,
    ) -> Self {
        let project_name = git(&["rev-parse", "--show-toplevel"])
            .and_then(|top| {
                top.rsplit(['/', '\\'])
                    .next()
                    .filter(|name| !name.is_empty())
                    .map(str::to_owned)
            })
            .unwrap_or_else(|| fallback_project_name.to_owned());
        let git_date = git(&["log", "-1", "--format=%ct"])
            .and_then(|seconds| seconds.parse().ok())
            .map_or_else(|| UNKNOWN.to_owned(), format_utc);
        let dirty = git(&["status", "--porcelain"])
            .map_or(DIRTY_UNKNOWN, |status| i32::from(!status.is_empty()));
        Self {
            project_name,
            build_date: format_utc(now_unix_seconds),
            git_sha: git(&["rev-parse", "HEAD"]).unwrap_or_else(|| UNKNOWN.to_owned()),
            git_date,
            git_branch: git(&["rev-parse", "--abbrev-ref", "HEAD"])
                .unwrap_or_else(|| UNKNOWN.to_owned()),
            dirty,
        }
    }

    /// Rust source for `robot/build_constants.rs` to include.
    pub fn render(&self) -> String {
        format!(
            "pub const MAVEN_NAME: &str = {:?};\n\
             pub const BUILD_DATE: &str = {:?};\n\
             pub const GIT_SHA: &str = {:?};\n\
             pub const GIT_DATE: &str = {:?};\n\
             pub const GIT_BRANCH: &str = {:?};\n\
             pub const DIRTY: i32 = {};\n",
            self.project_name,
            self.build_date,
            self.git_sha,
            self.git_date,
            self.git_branch,
            self.dirty
        )
    }
}

/// `yyyy-MM-dd HH:mm:ss UTC`. Java's gversion used New York time; UTC needs no tz database.
pub fn format_utc(unix_seconds: i64) -> String {
    let days = unix_seconds.div_euclid(86_400);
    let secs = unix_seconds.rem_euclid(86_400);
    // Civil date from a day count (Howard Hinnant's algorithm).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let day_of_era = z.rem_euclid(146_097);
    let year_of_era =
        (day_of_era - day_of_era / 1_460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
    let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
    let month_index = (5 * day_of_year + 2) / 153;
    let day = day_of_year - (153 * month_index + 2) / 5 + 1;
    let month = if month_index < 10 {
        month_index + 3
    } else {
        month_index - 9
    };
    let year = year_of_era + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}-{month:02}-{day:02} {:02}:{:02}:{:02} UTC",
        secs / 3_600,
        secs % 3_600 / 60,
        secs % 60
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn formats_known_instants_in_utc() {
        assert_eq!(format_utc(0), "1970-01-01 00:00:00 UTC");
        assert_eq!(format_utc(951_782_400), "2000-02-29 00:00:00 UTC");
        assert_eq!(format_utc(1_767_225_599), "2025-12-31 23:59:59 UTC");
        assert_eq!(format_utc(-1), "1969-12-31 23:59:59 UTC");
    }

    #[test]
    fn reads_every_field_from_git() {
        let info = BuildInfo::collect("fallback", 0, |args| {
            Some(
                match args {
                    ["rev-parse", "--show-toplevel"] => "/home/team/2026-Spectrum",
                    ["log", "-1", "--format=%ct"] => "1767225599",
                    ["status", "--porcelain"] => " M file.rs",
                    ["rev-parse", "--abbrev-ref", "HEAD"] => "main",
                    ["rev-parse", "HEAD"] => "abc123",
                    other => panic!("unexpected git call {other:?}"),
                }
                .to_owned(),
            )
        });
        assert_eq!(info.project_name, "2026-Spectrum");
        assert_eq!(info.git_sha, "abc123");
        assert_eq!(info.git_date, "2025-12-31 23:59:59 UTC");
        assert_eq!(info.git_branch, "main");
        assert_eq!(info.dirty, 1);
    }

    #[test]
    fn a_clean_tree_is_not_dirty() {
        let info = BuildInfo::collect("fallback", 0, |args| match args {
            ["status", "--porcelain"] => Some(String::new()),
            _ => None,
        });
        assert_eq!(info.dirty, 0);
    }

    #[test]
    fn missing_git_falls_back_to_unknown() {
        let info = BuildInfo::collect("robot", 86_400, |_| None);
        assert_eq!(
            info,
            BuildInfo {
                project_name: "robot".to_owned(),
                build_date: "1970-01-02 00:00:00 UTC".to_owned(),
                git_sha: UNKNOWN.to_owned(),
                git_date: UNKNOWN.to_owned(),
                git_branch: UNKNOWN.to_owned(),
                dirty: DIRTY_UNKNOWN,
            }
        );
    }

    #[test]
    fn rendered_source_declares_every_constant() {
        let source = BuildInfo::collect("robot", 0, |_| None).render();
        for name in [
            "MAVEN_NAME",
            "BUILD_DATE",
            "GIT_SHA",
            "GIT_DATE",
            "GIT_BRANCH",
            "DIRTY",
        ] {
            assert!(source.contains(&format!("pub const {name}:")), "{name}");
        }
        assert!(source.contains("DIRTY: i32 = -1;"));
    }
}

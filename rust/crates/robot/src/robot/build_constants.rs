//! Port of the generated `frc.robot.BuildConstants`. `build.rs` writes the constants from git.

include!(concat!(env!("OUT_DIR"), "/build_constants.rs"));

/// The `BuildConstants/GitDirty` text `Robot` logs.
pub fn dirty_description() -> &'static str {
    match DIRTY {
        0 => "All changes committed",
        1 => "Uncommitted changes",
        _ => "Unknown",
    }
}

#[cfg(test)]
#[path = "../../build_info.rs"]
mod build_info;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generated_constants_are_filled_in() {
        assert!(!MAVEN_NAME.is_empty());
        assert!(!BUILD_DATE.is_empty());
        assert!(!GIT_SHA.is_empty());
        assert!(!GIT_DATE.is_empty());
        assert!(!GIT_BRANCH.is_empty());
        assert!(matches!(DIRTY, -1..=1));
    }

    #[test]
    fn the_dirty_flag_has_a_description() {
        assert!(
            ["All changes committed", "Uncommitted changes", "Unknown"]
                .contains(&dirty_description())
        );
    }
}

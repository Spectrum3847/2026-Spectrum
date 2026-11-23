//! Field dimensions and reference points, in meters. Poses use a blue alliance origin.

use commands::Trigger;
use wpilib::driver_station::{self, Alliance};
use wpimath::geometry::{Pose2d, Rotation2d, Translation2d, Translation3d};

/// `Units.inchesToMeters`, usable in constants.
const fn inches(value: f64) -> f64 {
    value * 0.0254
}

pub const FIELD_LENGTH: f64 = inches(651.2);
pub const HALF_LENGTH: f64 = FIELD_LENGTH / 2.0;
pub const FIELD_WIDTH: f64 = inches(317.7);
pub const HALF_WIDTH: f64 = FIELD_WIDTH / 2.0;

pub const CENTER_FIELD: Pose2d = Pose2d::new(
    Translation2d::new(HALF_LENGTH, HALF_WIDTH),
    Rotation2d::ZERO,
);

/// Inside of the starting line.
pub const STARTING_LINE_X: f64 = inches(299.438);

pub const TAG26_X: f64 = inches(158.61);

pub const NORMAL_FEED_BLUE_LEFT: Translation2d = Translation2d::new(1.0, FIELD_WIDTH - 1.0);
pub const NORMAL_FEED_BLUE_RIGHT: Translation2d = Translation2d::new(1.0, 1.0);
pub const NORMAL_FEED_RED_LEFT: Translation2d = Translation2d::new(FIELD_LENGTH - 1.0, 1.0);
pub const NORMAL_FEED_RED_RIGHT: Translation2d =
    Translation2d::new(FIELD_LENGTH - 1.0, FIELD_WIDTH - 1.0);

pub const DEEP_FEED_BLUE_LEFT: Translation2d = Translation2d::new(1.0, FIELD_WIDTH - 2.5);
pub const DEEP_FEED_BLUE_RIGHT: Translation2d = Translation2d::new(1.0, 2.5);
pub const DEEP_FEED_RED_LEFT: Translation2d = Translation2d::new(FIELD_LENGTH - 1.0, 2.5);
pub const DEEP_FEED_RED_RIGHT: Translation2d =
    Translation2d::new(FIELD_LENGTH - 1.0, FIELD_WIDTH - 2.5);

pub mod blue_hub {
    use super::*;

    pub const WIDTH: f64 = inches(47.0);
    pub const TOTAL_HEIGHT: f64 = inches(72.0);
    pub const INNER_OPENING_WIDTH: f64 = inches(41.7);
    pub const INNER_OPENING_HEIGHT: f64 = inches(56.5);

    pub const CENTER_X: f64 = TAG26_X + WIDTH / 2.0;
    pub const CENTER_Y: f64 = FIELD_WIDTH / 2.0;

    pub const TOP_CENTER: Translation3d = Translation3d::new(CENTER_X, CENTER_Y, TOTAL_HEIGHT);
    pub const INNER_CENTER: Translation3d =
        Translation3d::new(CENTER_X, CENTER_Y, INNER_OPENING_HEIGHT);
    pub const NEAR_LEFT_CORNER: Translation2d = Translation2d::new(TAG26_X, CENTER_Y + WIDTH / 2.0);
    pub const NEAR_RIGHT_CORNER: Translation2d =
        Translation2d::new(TAG26_X, CENTER_Y - WIDTH / 2.0);
    pub const FAR_LEFT_CORNER: Translation2d =
        Translation2d::new(TAG26_X + WIDTH, CENTER_Y + WIDTH / 2.0);
    pub const FAR_RIGHT_CORNER: Translation2d =
        Translation2d::new(TAG26_X + WIDTH, CENTER_Y - WIDTH / 2.0);
}

pub mod blue_bumps {
    use super::*;

    pub const WIDTH: f64 = inches(73.0);
    pub const DEPTH: f64 = inches(44.4);
    pub const HEIGHT: f64 = inches(6.513);
}

/// Average of four corners, as Java computes `centerPose`.
const fn corner_center(
    a: Translation2d,
    b: Translation2d,
    c: Translation2d,
    d: Translation2d,
) -> Translation2d {
    Translation2d::new((a.x + b.x + c.x + d.x) / 4.0, (a.y + b.y + c.y + d.y) / 4.0)
}

pub mod left_blue_bump {
    use super::*;

    pub const NEAR_LEFT_CORNER: Translation2d = Translation2d::new(
        blue_hub::CENTER_X - blue_bumps::DEPTH / 2.0,
        FIELD_WIDTH - blue_trench::OPENING_WIDTH,
    );
    pub const NEAR_RIGHT_CORNER: Translation2d = blue_hub::NEAR_LEFT_CORNER;
    pub const FAR_LEFT_CORNER: Translation2d = Translation2d::new(
        blue_hub::CENTER_X + blue_bumps::DEPTH / 2.0,
        FIELD_WIDTH - blue_trench::OPENING_WIDTH,
    );
    pub const FAR_RIGHT_CORNER: Translation2d = blue_hub::FAR_LEFT_CORNER;

    pub const CENTER_POSE: Translation2d = corner_center(
        NEAR_LEFT_CORNER,
        NEAR_RIGHT_CORNER,
        FAR_LEFT_CORNER,
        FAR_RIGHT_CORNER,
    );
}

pub mod right_blue_bump {
    use super::*;

    pub const NEAR_LEFT_CORNER: Translation2d = blue_hub::NEAR_RIGHT_CORNER;
    pub const NEAR_RIGHT_CORNER: Translation2d = Translation2d::new(
        blue_hub::CENTER_X - blue_bumps::DEPTH / 2.0,
        blue_trench::OPENING_WIDTH,
    );
    pub const FAR_LEFT_CORNER: Translation2d = blue_hub::FAR_RIGHT_CORNER;
    pub const FAR_RIGHT_CORNER: Translation2d = Translation2d::new(
        blue_hub::CENTER_X + blue_bumps::DEPTH / 2.0,
        blue_trench::OPENING_WIDTH,
    );

    pub const CENTER_POSE: Translation2d = corner_center(
        NEAR_LEFT_CORNER,
        NEAR_RIGHT_CORNER,
        FAR_LEFT_CORNER,
        FAR_RIGHT_CORNER,
    );
}

pub mod blue_trench {
    use super::*;

    pub const WIDTH: f64 = inches(65.65);
    pub const DEPTH: f64 = inches(47.0);
    pub const HEIGHT: f64 = inches(40.25);
    pub const OPENING_WIDTH: f64 = inches(50.34);
    pub const OPENING_HEIGHT: f64 = inches(22.25);
}

pub mod left_blue_trench {
    use super::*;

    pub const OPENING_TOP_LEFT: Translation3d =
        Translation3d::new(blue_hub::CENTER_X, FIELD_WIDTH, blue_trench::OPENING_HEIGHT);
    pub const OPENING_TOP_RIGHT: Translation3d = Translation3d::new(
        blue_hub::CENTER_X,
        FIELD_WIDTH - blue_trench::OPENING_WIDTH,
        blue_trench::OPENING_HEIGHT,
    );
}

pub mod right_blue_trench {
    use super::*;

    pub const OPENING_TOP_LEFT: Translation3d = Translation3d::new(
        blue_hub::CENTER_X,
        blue_trench::OPENING_WIDTH,
        blue_trench::OPENING_HEIGHT,
    );
    pub const OPENING_TOP_RIGHT: Translation3d =
        Translation3d::new(blue_hub::CENTER_X, 0.0, blue_trench::OPENING_HEIGHT);
}

pub mod blue_tower {
    use super::*;

    pub const WIDTH: f64 = inches(49.25);
    pub const DEPTH: f64 = inches(45.0);
    pub const HEIGHT: f64 = inches(78.25);
    pub const INNER_OPENING_WIDTH: f64 = inches(32.25);
    pub const UPRIGHT_HEIGHT: f64 = inches(72.1);

    pub const LOW_RUNG_Z: f64 = inches(27.0);
    pub const MID_RUNG_Z: f64 = inches(45.0);
    pub const HIGH_RUNG_Z: f64 = inches(63.0);

    pub const TAG31_Y: f64 = FIELD_WIDTH / 2.0;

    pub const FRONT_FACE_X: f64 = inches(43.51);

    pub const CENTER: Translation2d = Translation2d::new(FRONT_FACE_X, TAG31_Y);

    pub const LEFT_UPRIGHT: Translation2d = Translation2d::new(
        FRONT_FACE_X,
        TAG31_Y + INNER_OPENING_WIDTH / 2.0 + inches(0.75),
    );
    pub const RIGHT_UPRIGHT: Translation2d = Translation2d::new(
        FRONT_FACE_X,
        TAG31_Y - INNER_OPENING_WIDTH / 2.0 - inches(0.75),
    );
}

pub mod blue_depot {
    use super::*;

    pub const WIDTH: f64 = inches(42.0);
    pub const DEPTH: f64 = inches(27.0);
    pub const HEIGHT: f64 = inches(1.125);
    pub const DISTANCE_FROM_CENTER_Y: f64 = inches(75.93);

    pub const CENTER: Translation3d =
        Translation3d::new(DEPTH, FIELD_WIDTH / 2.0 + DISTANCE_FROM_CENTER_Y, HEIGHT);
    pub const LEFT_CORNER: Translation3d = Translation3d::new(
        DEPTH,
        FIELD_WIDTH / 2.0 + DISTANCE_FROM_CENTER_Y + WIDTH / 2.0,
        HEIGHT,
    );
    pub const RIGHT_CORNER: Translation3d = Translation3d::new(
        DEPTH,
        FIELD_WIDTH / 2.0 + DISTANCE_FROM_CENTER_Y - WIDTH / 2.0,
        HEIGHT,
    );
}

pub mod blue_outpost {
    use super::*;

    pub const OPENING_WIDTH: f64 = inches(31.8);
    pub const OPENING_HEIGHT: f64 = inches(7.0);
    pub const OPENING_Z: f64 = inches(28.1);

    pub const TAG29_Y: f64 = inches(26.22);

    pub const CENTER: Translation2d = Translation2d::new(0.0, TAG29_Y);
}

pub const fn blue_to_red_translation3d(translation: Translation3d) -> Translation3d {
    Translation3d::new(FIELD_LENGTH - translation.x, translation.y, translation.z)
}

pub const fn blue_to_red_translation2d(translation: Translation2d) -> Translation2d {
    Translation2d::new(FIELD_LENGTH - translation.x, translation.y)
}

pub const fn blue_to_red(translation: f64) -> f64 {
    FIELD_LENGTH - translation
}

pub const BLUE_HUB_CENTER: Translation3d = blue_hub::TOP_CENTER;
pub const RED_HUB_CENTER: Translation3d = blue_to_red_translation3d(blue_hub::TOP_CENTER);

/// An unknown alliance counts as blue.
pub fn is_blue() -> bool {
    is_blue_for(driver_station::alliance())
}

pub fn is_blue_for(alliance: Option<Alliance>) -> bool {
    alliance.unwrap_or(Alliance::Blue) == Alliance::Blue
}

pub fn is_red() -> bool {
    !is_blue()
}

thread_local! {
    static RED: Trigger = Trigger::new(is_red);
    static BLUE: Trigger = Trigger::new(is_blue);
}

pub fn red() -> Trigger {
    RED.with(Trigger::clone)
}

pub fn blue() -> Trigger {
    BLUE.with(Trigger::clone)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dimensions_match_the_manual_in_meters() {
        assert!((FIELD_LENGTH - 16.540_48).abs() < 1e-5);
        assert!((FIELD_WIDTH - 8.069_58).abs() < 1e-5);
        assert_eq!(HALF_LENGTH * 2.0, FIELD_LENGTH);
    }

    #[test]
    fn red_hub_mirrors_the_blue_hub_in_x() {
        assert!((RED_HUB_CENTER.x - (FIELD_LENGTH - BLUE_HUB_CENTER.x)).abs() < 1e-12);
        assert_eq!(RED_HUB_CENTER.y, BLUE_HUB_CENTER.y);
        assert_eq!(RED_HUB_CENTER.z, BLUE_HUB_CENTER.z);
    }

    #[test]
    fn bump_centers_sit_between_the_hub_and_the_trench() {
        let (left, right) = (left_blue_bump::CENTER_POSE, right_blue_bump::CENTER_POSE);
        assert!(std::hint::black_box(left.y) > HALF_WIDTH);
        assert!(std::hint::black_box(right.y) < HALF_WIDTH);
        assert!((left.x - right.x).abs() < 1e-12);
    }

    #[test]
    fn unknown_alliance_counts_as_blue() {
        assert!(is_blue_for(None));
        assert!(is_blue_for(Some(Alliance::Blue)));
        assert!(!is_blue_for(Some(Alliance::Red)));
    }
}

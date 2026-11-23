use crate::context;
use crate::rebuilt::field;
use wpimath::geometry::Translation2d;
use wpimath::interpolation::InterpolatingDoubleTreeMap;
use wpimath::units::inches_to_meters;

thread_local! {
    static DISTANCE_OFFSET_MAP: InterpolatingDoubleTreeMap = {
        let mut map = InterpolatingDoubleTreeMap::new();
        map.put(0.0, inches_to_meters(0.0));
        map
    };
}

fn x_distance_offset() -> f64 {
    inches_to_meters(0.0)
}

/// Where to feed fuel to from the current robot pose and zone.
pub fn generate() -> Translation2d {
    let swerve = context::swerve();
    let in_field_left = swerve.in_field_left().get_as_boolean();
    let in_opposing_alliance_zone = swerve.in_enemy_alliance_zone().get_as_boolean();
    generate_for(
        field::is_red(),
        in_field_left,
        in_opposing_alliance_zone,
        swerve.get_robot_pose().translation,
    )
}

/// [`generate`] with the alliance, zone flags and robot position given.
pub fn generate_for(
    red: bool,
    in_field_left: bool,
    in_opposing_alliance_zone: bool,
    robot_translation: Translation2d,
) -> Translation2d {
    let blue = !red;
    let mut feed_target = if in_opposing_alliance_zone {
        if in_field_left {
            if blue {
                field::DEEP_FEED_BLUE_LEFT
            } else {
                field::DEEP_FEED_RED_RIGHT
            }
        } else if blue {
            field::DEEP_FEED_BLUE_RIGHT
        } else {
            field::DEEP_FEED_RED_LEFT
        }
    } else if in_field_left {
        if blue {
            field::NORMAL_FEED_BLUE_LEFT
        } else {
            field::NORMAL_FEED_RED_RIGHT
        }
    } else if blue {
        field::NORMAL_FEED_BLUE_RIGHT
    } else {
        field::NORMAL_FEED_RED_LEFT
    };

    let distance = Translation2d::new(feed_target.x, feed_target.y).distance(robot_translation);

    let distance_offset = DISTANCE_OFFSET_MAP.with(|map| map.get(distance).unwrap_or(0.0));
    // Do math in blue alliance, we flip for red.
    let mut off_set = Translation2d::new(x_distance_offset(), -distance_offset);

    if red {
        off_set = Translation2d::new(-off_set.x, off_set.y);
    }

    feed_target = Translation2d::new(feed_target.x + off_set.x, feed_target.y + off_set.y);
    feed_target
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blue_and_red_pick_mirrored_corners() {
        let robot = Translation2d::new(8.0, 4.0);
        assert_eq!(
            generate_for(false, true, false, robot),
            field::NORMAL_FEED_BLUE_LEFT
        );
        assert_eq!(
            generate_for(true, true, false, robot),
            field::NORMAL_FEED_RED_RIGHT
        );
        assert_eq!(
            generate_for(false, false, true, robot),
            field::DEEP_FEED_BLUE_RIGHT
        );
        assert_eq!(
            generate_for(true, false, true, robot),
            field::DEEP_FEED_RED_LEFT
        );
    }
}

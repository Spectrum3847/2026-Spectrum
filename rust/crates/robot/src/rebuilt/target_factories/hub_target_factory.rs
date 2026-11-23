use crate::context;
use crate::rebuilt::field;
use wpimath::geometry::{Translation2d, Translation3d};
use wpimath::interpolation::InterpolatingDoubleTreeMap;
use wpimath::units::inches_to_meters;

struct Maps {
    height: InterpolatingDoubleTreeMap,
    distance_offset: InterpolatingDoubleTreeMap,
}

thread_local! {
    static MAPS: Maps = {
        let mut height = InterpolatingDoubleTreeMap::new();
        height.put(0.0, 0.0);
        let mut distance_offset = InterpolatingDoubleTreeMap::new();
        distance_offset.put(0.0, inches_to_meters(0.0));
        Maps { height, distance_offset }
    };
}

fn x_distance_offset() -> f64 {
    inches_to_meters(0.0)
}

/// Where to aim for the alliance hub from the current robot pose.
pub fn generate() -> Translation3d {
    generate_for(
        field::is_red(),
        context::swerve().get_robot_pose().translation,
    )
}

/// [`generate`] with the alliance and robot position given.
pub fn generate_for(red: bool, robot_translation: Translation2d) -> Translation3d {
    let mut hub_pose = if red {
        field::RED_HUB_CENTER
    } else {
        field::BLUE_HUB_CENTER
    };

    let distance = Translation2d::new(hub_pose.x, hub_pose.y).distance(robot_translation);

    let (distance_offset, height) = MAPS.with(|maps| {
        (
            maps.distance_offset.get(distance).unwrap_or(0.0),
            maps.height.get(distance).unwrap_or(0.0),
        )
    });
    // Do math in blue alliance, we flip for red.
    let mut off_set = Translation2d::new(x_distance_offset(), -distance_offset);

    if red {
        off_set = Translation2d::new(-off_set.x, off_set.y);
    }

    hub_pose = Translation3d::new(
        hub_pose.x + off_set.x,
        hub_pose.y + off_set.y,
        hub_pose.z + height,
    );
    hub_pose
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_empty_offset_tables_aim_at_the_hub_top_center() {
        let blue = generate_for(false, Translation2d::new(3.0, 4.0));
        assert_eq!(blue, field::BLUE_HUB_CENTER);
        let red = generate_for(true, Translation2d::new(12.0, 4.0));
        assert_eq!(red, field::RED_HUB_CENTER);
    }
}

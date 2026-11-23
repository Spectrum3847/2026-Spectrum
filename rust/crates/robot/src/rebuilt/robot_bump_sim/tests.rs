use super::*;
use wpimath::geometry::Rotation2d;

const TOL: f64 = 1e-9;

fn square() -> [Translation2d; 4] {
    [
        Translation2d::new(0.3, 0.3),
        Translation2d::new(0.3, -0.3),
        Translation2d::new(-0.3, 0.3),
        Translation2d::new(-0.3, -0.3),
    ]
}

#[test]
fn matches_java_golden_fixture() {
    let mut sim: Option<RobotBumpSim> = None;
    golden::check("robot_bump_sim", TOL, |op, i| match op {
        "new" => {
            let offsets = [0, 2, 4, 6].map(|k| Translation2d::new(i[k], i[k + 1]));
            sim = Some(RobotBumpSim::new(offsets));
            Some(Vec::new())
        }
        "update" => {
            let sim = sim.as_mut()?;
            let pose = Pose2d::from_xy(i[0], i[1], Rotation2d::new(i[2]));
            let pose3d = sim.update(pose, ChassisSpeeds::new(i[3], i[4], 0.0), i[6] as i32);
            let world = sim.get_sim_world_pose(pose);
            let q = pose3d.rotation.quaternion();
            Some(vec![
                pose3d.x(),
                pose3d.y(),
                pose3d.z(),
                q.w,
                q.x,
                q.y,
                q.z,
                f64::from(sim.is_on_ramp()),
                world.x(),
                world.y(),
                world.rotation.radians(),
            ])
        }
        _ => None,
    });
}

/// Drives straight at a speed with the sim owning X on the ramp. Returns the final X and
/// whether the robot was ever on the ramp.
fn drive(x0: f64, vx0: f64, steps: usize) -> (f64, bool) {
    let mut sim = RobotBumpSim::new(square());
    let (mut x, mut vx, dt) = (x0, vx0, 0.02);
    let mut touched = false;
    for _ in 0..steps {
        let pose = Pose2d::from_xy(x, 2.5, Rotation2d::ZERO);
        sim.update(pose, ChassisSpeeds::new(vx, 0.0, 0.0), 5);
        touched |= sim.is_on_ramp();
        let next = if sim.is_on_ramp() {
            sim.get_sim_world_pose(pose).x()
        } else {
            x + vx * dt
        };
        if sim.is_on_ramp() {
            vx = (next - x) / dt;
        }
        x = next;
    }
    (x, touched)
}

#[test]
fn a_fast_robot_crosses_the_bump() {
    let (x, touched) = drive(3.0, 3.0, 150);
    assert!(touched);
    assert!(x > 5.5, "x = {x}");
}

#[test]
fn a_slow_robot_slides_back() {
    let (x, touched) = drive(3.0, 1.0, 150);
    assert!(touched);
    assert!(x < 3.96, "x = {x}");
}

#[test]
fn off_the_bump_lane_nothing_happens() {
    let mut sim = RobotBumpSim::new(square());
    let pose = Pose2d::from_xy(4.3, 0.8, Rotation2d::ZERO);
    let pose3d = sim.update(pose, ChassisSpeeds::new(3.0, 0.0, 0.0), 5);
    assert!(!sim.is_on_ramp());
    assert!(pose3d.z() < 1e-9 && pose3d.x() == 4.3);
}

#[test]
fn bump_tables_pair_up() {
    for (start, end) in BUMP_LINE_STARTS.iter().zip(&BUMP_LINE_ENDS) {
        assert!(start.y < end.y);
        assert!((start.x - end.x).abs() > 0.5);
    }
}

use super::*;
use golden::{close, load};
use std::cell::Cell;
use std::rc::Rc;

const TOL: f64 = 1e-9;
const ZONES: usize = 2;

const HEADER: [&str; 27] = [
    "balls",
    "launched",
    "scored",
    "intaked",
    "blue",
    "red",
    "sleeping",
    "ke",
    "pe",
    "mx",
    "my",
    "mz",
    "contacts",
    "callbacks0",
    "callbacks1",
    "in_flight",
    "on_ground",
    "last_launch_speed",
    "sum_px",
    "sum_py",
    "sum_pz",
    "sum_vx",
    "sum_vy",
    "sum_vz",
    "sum_wx",
    "sum_wy",
    "sum_wz",
];
const BALL_FIELDS: [&str; 9] = ["px", "py", "pz", "vx", "vy", "vz", "wx", "wy", "wz"];

fn config_from(i: &[f64]) -> PhysicsConfig {
    PhysicsConfig {
        drag_enabled: i[0] != 0.0,
        magnus_enabled: i[1] != 0.0,
        friction_enabled: i[2] != 0.0,
        spin_transfer_enabled: i[3] != 0.0,
        sleeping_enabled: i[4] != 0.0,
        ccd_enabled: i[5] != 0.0,
        velocity_dependent_cor: i[6] != 0.0,
        spin_decay_enabled: i[7] != 0.0,
        solver_iterations: i[8] as i32,
        subticks: i[9] as i32,
        spin_decay_tau: i[10],
        sleep_velocity_threshold: i[11],
        sleep_frame_threshold: i[12] as i32,
        ccd_speed_threshold: i[13],
        baumgarte_beta: i[14],
        baumgarte_slop: i[15],
        deterministic: i[16] != 0.0,
        deterministic_seed: i[17] as i64,
        conservation_monitor: i[18] != 0.0,
    }
}

fn v3(i: &[f64]) -> Translation3d {
    Translation3d::new(i[0], i[1], i[2])
}

fn flatten(points: &[Translation3d]) -> Vec<f64> {
    points.iter().flat_map(|p| [p.x, p.y, p.z]).collect()
}

/// Replays the Java action log against one sim.
struct Replay {
    sim: Option<FuelPhysicsSim>,
    robot: Rc<Cell<[f64; 6]>>,
    zone_active: Rc<[Cell<bool>; ZONES]>,
    callbacks: Rc<[Cell<i32>; ZONES]>,
    zones: usize,
}

impl Replay {
    fn new() -> Self {
        Self {
            sim: None,
            robot: Rc::default(),
            zone_active: Rc::default(),
            callbacks: Rc::default(),
            zones: 0,
        }
    }

    fn sim(&mut self) -> &mut FuelPhysicsSim {
        self.sim.as_mut().expect("`new` comes first")
    }

    fn apply(&mut self, op: &str, i: &[f64]) -> Option<Vec<f64>> {
        let none = Some(Vec::new());
        match op {
            "rng" => {
                let mut rng = JavaRandom::new(i[0] as i64);
                Some((0..8).map(|_| rng.next_double()).collect())
            }
            "new" => {
                self.sim = Some(FuelPhysicsSim::headless(config_from(i)));
                self.robot.set([0.0; 6]);
                for k in 0..ZONES {
                    self.zone_active[k].set(false);
                    self.callbacks[k].set(0);
                }
                self.zones = 0;
                none
            }
            "seed" => {
                self.sim().set_deterministic(i[0] as i64);
                none
            }
            "launch_rpm" => {
                let sim = self.sim();
                sim.launch_ball(v3(&i[0..3]), v3(&i[3..6]), i[6]);
                Some(flatten(sim.get_last_shot_arc()))
            }
            "launch_omega" => {
                let sim = self.sim();
                sim.launch_ball_omega(v3(&i[0..3]), v3(&i[3..6]), v3(&i[6..9]));
                Some(flatten(sim.get_last_shot_arc()))
            }
            "spawn" => {
                self.sim().spawn_ball(v3(i));
                none
            }
            "spawn_vel" => {
                self.sim()
                    .spawn_ball_with_velocity(v3(&i[0..3]), v3(&i[3..6]));
                none
            }
            "clear" => {
                self.sim().clear_balls();
                none
            }
            "place_field_balls" => {
                self.sim().place_field_balls();
                none
            }
            "reset_counters" => {
                self.sim().reset_counters();
                none
            }
            "robot_config" => {
                let (pose_robot, speeds_robot) = (Rc::clone(&self.robot), Rc::clone(&self.robot));
                self.sim().configure_robot(
                    i[0],
                    i[1],
                    i[2],
                    i[3] as i32,
                    move || {
                        let r = pose_robot.get();
                        Pose2d::from_xy(r[0], r[1], Rotation2d::new(r[2]))
                    },
                    move || {
                        let r = speeds_robot.get();
                        ChassisSpeeds::new(r[3], r[4], r[5])
                    },
                );
                none
            }
            "intake_zone" => {
                let k = self.zones;
                self.zones += 1;
                let (active, callbacks) =
                    (Rc::clone(&self.zone_active), Rc::clone(&self.callbacks));
                self.sim().add_intake_zone_with_callback(
                    i[0],
                    i[1],
                    i[2],
                    i[3],
                    move || active[k].get(),
                    move || callbacks[k].set(callbacks[k].get() + 1),
                );
                none
            }
            "intake_active" => {
                self.zone_active[i[0] as usize].set(i[1] != 0.0);
                none
            }
            "robot_state" => {
                self.robot.set([i[0], i[1], i[2], i[3], i[4], i[5]]);
                none
            }
            "advance" => {
                let sim = self.sim();
                for _ in 0..i[1] as usize {
                    sim.advance_physics(i[0]);
                }
                none
            }
            "step" => {
                self.sim().advance_physics(i[0]);
                Some(self.snapshot())
            }
            _ => None,
        }
    }

    fn snapshot(&mut self) -> Vec<f64> {
        let callbacks = [self.callbacks[0].get(), self.callbacks[1].get()];
        let sim = self.sim();
        let ke = sim.get_total_kinetic_energy();
        let pe = sim.get_total_potential_energy();
        let momentum = sim.get_total_momentum();
        let balls = sim.get_balls();

        let mut out = vec![
            balls.len() as f64,
            f64::from(sim.get_total_launched()),
            f64::from(sim.get_total_scored()),
            f64::from(sim.get_total_intaked()),
            f64::from(sim.get_blue_score()),
            f64::from(sim.get_red_score()),
            sim.get_sleeping_ball_count() as f64,
            ke,
            pe,
            momentum.x,
            momentum.y,
            momentum.z,
            sim.get_contact_count() as f64,
            f64::from(callbacks[0]),
            f64::from(callbacks[1]),
            sim.get_balls_in_flight() as f64,
            sim.get_balls_on_ground() as f64,
            sim.get_last_launch_speed(),
        ];
        let mut sums = [0.0; 9];
        for b in balls {
            for (sum, v) in sums.iter_mut().zip([
                b.pos.x, b.pos.y, b.pos.z, b.vel.x, b.vel.y, b.vel.z, b.omega.x, b.omega.y,
                b.omega.z,
            ]) {
                *sum += v;
            }
        }
        out.extend(sums);
        for b in balls.iter().take(40) {
            for t in [b.pos, b.vel, b.omega] {
                out.extend([t.x, t.y, t.z]);
            }
        }
        out
    }
}

fn label(op: &str, index: usize) -> String {
    match op {
        "step" if index < HEADER.len() => HEADER[index].to_owned(),
        "step" => {
            let k = index - HEADER.len();
            format!("ball{}.{}", k / 9, BALL_FIELDS[k % 9])
        }
        _ => format!("out[{index}]"),
    }
}

#[test]
fn matches_java_golden_fixture() {
    let cases = load("fuel_physics_sim");
    let mut replay = Replay::new();
    let mut steps = 0;
    let mut scenario = 0;
    for (n, case) in cases.iter().enumerate() {
        if case.op == "new" {
            scenario += 1;
        }
        let got = replay
            .apply(&case.op, &case.input)
            .unwrap_or_else(|| panic!("#{n}: unknown op {}", case.op));
        assert_eq!(
            got.len(),
            case.out.len(),
            "#{n} {} (scenario {scenario}): output length",
            case.op
        );
        for (index, (g, w)) in got.iter().zip(&case.out).enumerate() {
            assert!(
                close(*g, *w, TOL),
                "#{n} {} (scenario {scenario}) {}: want {w}, got {g}",
                case.op,
                label(&case.op, index)
            );
        }
        steps += usize::from(case.op == "step");
    }
    assert!(steps > 300, "fixture has too few steps: {steps}");
}

fn headless() -> FuelPhysicsSim {
    let config = PhysicsConfig {
        deterministic: true,
        ..PhysicsConfig::default()
    };
    FuelPhysicsSim::headless(config)
}

#[test]
fn java_random_matches_the_jdk() {
    // java.util.Random(42).nextDouble()
    assert_eq!(JavaRandom::new(42).next_double(), 0.7275636800328681);
}

#[test]
fn grid_covers_the_field() {
    assert_eq!((GRID_COLS, GRID_ROWS), (67, 33));
    assert!(GRID_COLS as f64 * CELL_SIZE >= FIELD_LENGTH);
    assert!(GRID_ROWS as f64 * CELL_SIZE >= FIELD_WIDTH);
}

#[test]
fn field_layout_is_408_balls() {
    let mut sim = headless();
    sim.place_field_balls();
    assert_eq!(sim.get_ball_count(), 12 * 30 + 2 * 4 * 6);
}

#[test]
fn ball_count_is_capped() {
    let mut sim = headless();
    for _ in 0..MAX_BALLS + 5 {
        sim.spawn_ball(Translation3d::new(1.0, 1.0, 1.0));
    }
    assert_eq!(sim.get_ball_count(), MAX_BALLS);
    sim.launch_ball(Translation3d::ZERO, Translation3d::ZERO, 0.0);
    assert_eq!(sim.get_total_launched(), 0);
}

#[test]
fn launch_never_takes_intaked_below_zero() {
    let mut sim = headless();
    sim.launch_ball(
        Translation3d::new(2.0, 2.0, 1.0),
        Translation3d::new(1.0, 0.0, 1.0),
        0.0,
    );
    assert_eq!((sim.get_total_launched(), sim.get_total_intaked()), (1, 0));
    assert_eq!(
        sim.get_last_shot_arc()[0],
        Translation3d::new(2.0, 2.0, 1.0)
    );
}

#[test]
fn tick_does_nothing_until_enabled() {
    let mut sim = headless();
    sim.spawn_ball(Translation3d::new(2.0, 2.0, 2.0));
    sim.tick();
    assert_eq!(sim.get_balls()[0].pos().z, 2.0);
    sim.enable();
    sim.tick();
    assert!(sim.get_balls()[0].pos().z < 2.0);
    sim.disable();
    assert!(!sim.is_running());
}

#[test]
fn ball_falling_into_the_blue_hub_scores() {
    let mut sim = headless();
    sim.spawn_ball_with_velocity(
        Translation3d::new(4.5974, 4.035, 3.0),
        Translation3d::new(0.0, 0.0, -2.0),
    );
    for _ in 0..100 {
        sim.advance_physics(0.02);
    }
    assert_eq!(
        (
            sim.get_blue_score(),
            sim.get_red_score(),
            sim.get_total_scored()
        ),
        (1, 0, 1)
    );
    assert!(
        sim.get_balls()[0].pos().x > 5.0,
        "exits toward the red side"
    );
}

#[test]
fn ball_rising_through_the_hub_does_not_score_on_the_way_up() {
    let mut sim = headless();
    sim.spawn_ball_with_velocity(
        Translation3d::new(11.938, 4.035, 1.0),
        Translation3d::new(0.0, 0.0, 6.0),
    );
    for _ in 0..30 {
        sim.advance_physics(0.02);
        assert_eq!(sim.get_red_score(), 0, "still rising");
        if sim.get_balls()[0].vel().z < 0.0 {
            break;
        }
    }
}

#[test]
fn ball_over_the_alliance_wall_leaves_the_field() {
    let mut sim = headless();
    sim.spawn_ball_with_velocity(
        Translation3d::new(1.5, 2.0, 1.4),
        Translation3d::new(-7.0, 0.0, 6.0),
    );
    for _ in 0..40 {
        sim.advance_physics(0.02);
    }
    assert_eq!(sim.get_ball_count(), 0);
}

#[test]
fn a_nan_ball_is_removed() {
    let mut sim = headless();
    sim.spawn_ball(Translation3d::new(f64::NAN, 1.0, 1.0));
    sim.spawn_ball(Translation3d::new(5.0, 5.0, 1.0));
    sim.advance_physics(0.02);
    assert_eq!(sim.get_ball_count(), 1);
}

#[test]
fn a_resting_ball_goes_to_sleep() {
    let mut sim = headless();
    sim.spawn_ball(Translation3d::new(6.0, 3.0, BALL_RADIUS));
    for _ in 0..5 {
        sim.advance_physics(0.02);
    }
    assert_eq!(sim.get_sleeping_ball_count(), 1);
}

#[test]
fn contacts_reuse_their_buffer() {
    let mut sim = headless();
    sim.place_field_balls();
    sim.advance_physics(0.02);
    let capacity = sim.contacts.capacity();
    sim.advance_physics(0.02);
    assert_eq!(sim.contacts.capacity(), capacity);
}

#[test]
#[ignore = "needs the WPILib natives: LD_LIBRARY_PATH=native/linuxarm64"]
fn tick_publishes_to_networktables() {
    let mut sim = FuelPhysicsSim::new("TestFuelPhysicsSim");
    sim.enable();
    for k in 0..3 {
        sim.spawn_ball(Translation3d::new(2.0 + f64::from(k), 2.0, BALL_RADIUS));
    }
    sim.tick();

    let inst = Instance::default_instance();
    let options = Options::default();
    let positions = inst
        .topic::<StructArrayKind<Translation3d>>("TestFuelPhysicsSim/Positions")
        .subscribe(Vec::new(), &options);
    let count = inst
        .topic::<Integer>("TestFuelPhysicsSim/Stats/BallCount")
        .subscribe(0, &options);
    assert_eq!(positions.get().len(), 3);
    assert_eq!(count.get(), 3);
}

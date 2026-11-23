//! Starts an NT4 server on a spare port, connects a client instance in the same process, and
//! reads back a double, a string array, a Pose2d struct, a chooser, a Field2d, a Mechanism2d.
//!
//! Run from `rust/` with the natives on the library path:
//! `cargo xtask sim nt_smoke -p wpilib`

use std::thread::sleep;
use std::time::{Duration, Instant};
use wpilib::dashboard::{
    Field2d, Ligament, Mechanism2d, Sendable, SendableBuilder, SendableChooser, io::NtIo,
};
use wpilib::nt::{
    Double, Instance, Options, StringArray, StructArrayKind, StructKind, Text, Value,
};
use wpimath::geometry::{Pose2d, Rotation2d, Translation3d};

const PORT: u32 = 5899;

fn wait_for<T>(what: &str, mut read: impl FnMut() -> Option<T>) -> T {
    let start = Instant::now();
    loop {
        if let Some(value) = read() {
            return value;
        }
        assert!(
            start.elapsed() < Duration::from_secs(5),
            "timed out waiting for {what}"
        );
        sleep(Duration::from_millis(20));
    }
}

fn main() {
    let server = Instance::create();
    let client = Instance::create();
    server.start_server("", "127.0.0.1", 0, PORT);
    client.start_client4("nt_smoke");
    client.set_server("127.0.0.1", PORT);

    // Publish through the server instance.
    let numbers = server.table("Smoke");
    let double = numbers
        .topic::<Double>("speed")
        .publish(&Options::default());
    double.set(3.25);
    let names = numbers
        .topic::<StringArray>("names")
        .publish(&Options::default());
    let list = ["alpha".to_owned(), "beta".to_owned()];
    names.set(&list);
    let pose = numbers
        .topic::<StructKind<Pose2d>>("pose")
        .publish(&Options::default());
    pose.set(&Pose2d::from_xy(1.5, 2.5, Rotation2d::from_degrees(90.0)));
    let arc = numbers
        .topic::<StructArrayKind<Translation3d>>("arc")
        .publish(&Options::default());
    arc.set(&[
        Translation3d::new(1.0, 2.0, 3.0),
        Translation3d::new(4.0, 5.0, 6.0),
    ]);

    // The dashboard objects publish through the server instance too.
    let io: wpilib::dashboard::SharedIo =
        std::rc::Rc::new(std::cell::RefCell::new(NtIo::new(server)));
    let chooser = SendableChooser::<i32>::new();
    chooser.set_default_option("Leave", 1);
    chooser.add_option("Score", 2);
    let mut builder = SendableBuilder::new("/SmartDashboard/Auto Chooser", io.clone());
    chooser.init_sendable(&mut builder);
    builder.start_listeners();
    builder.update();

    let field = Field2d::new();
    field.set_robot_pose(Pose2d::from_xy(4.0, 2.0, Rotation2d::from_degrees(45.0)));
    let mut field_builder = SendableBuilder::new("/SmartDashboard/Field2d", io.clone());
    field.init_sendable(&mut field_builder);
    field
        .get_object("cam")
        .set_pose(Pose2d::from_xy(7.0, 1.0, Rotation2d::ZERO));

    let mech = Mechanism2d::new(3.0, 2.0);
    let arm = mech
        .get_root("Root", 1.0, 0.5)
        .append(Ligament::new("Arm", 0.4, 90.0));
    let mut mech_builder = SendableBuilder::new("/SmartDashboard/Mech", io);
    mech.init_sendable(&mut mech_builder);
    arm.set_angle(30.0);
    server.flush();

    // Read everything back through the client instance.
    let cn = client.table("Smoke");
    let speed = cn
        .topic::<Double>("speed")
        .subscribe(0.0, &Options::default());
    let got = wait_for("speed", || {
        let v = speed.get_atomic();
        (v.time != 0).then_some(v.value)
    });
    println!("double: {got}");
    assert_eq!(got, 3.25);

    let sub = cn
        .topic::<StringArray>("names")
        .subscribe(vec![], &Options::default());
    let got = wait_for("names", || Some(sub.get()).filter(|v| !v.is_empty()));
    println!("string[]: {got:?}");
    assert_eq!(got, list);

    let sub = cn
        .topic::<StructKind<Pose2d>>("pose")
        .subscribe(Pose2d::ZERO, &Options::default());
    let got = wait_for("pose", || Some(sub.get()).filter(|p| *p != Pose2d::ZERO));
    println!(
        "Pose2d: x={} y={} deg={:.1}",
        got.x(),
        got.y(),
        got.rotation.degrees()
    );
    assert_eq!(
        got,
        Pose2d::from_xy(1.5, 2.5, Rotation2d::from_degrees(90.0))
    );

    let sub = cn
        .topic::<StructArrayKind<Translation3d>>("arc")
        .subscribe(vec![], &Options::default());
    let got = wait_for("arc", || Some(sub.get()).filter(|v| !v.is_empty()));
    println!("Translation3d[]: {} points, last z={}", got.len(), got[1].z);
    assert_eq!(got[1], Translation3d::new(4.0, 5.0, 6.0));

    let schema = client.entry("/.schema/struct:Pose2d");
    let schema = wait_for("Pose2d schema", || match schema.get_value() {
        Some(Value::Raw(bytes)) => Some(bytes),
        _ => None,
    });
    println!(
        "schema struct:Pose2d = {}",
        String::from_utf8_lossy(&schema)
    );
    assert_eq!(schema, b"Translation2d translation;Rotation2d rotation");
    assert!(client.has_schema("struct:Pose2d") || server.has_schema("struct:Pose2d"));

    let dash = client.table("SmartDashboard");
    let text = |table: &str, key: &str| {
        let sub = dash
            .get_sub_table(table)
            .topic::<Text>(key)
            .subscribe(String::new(), &Options::default());
        wait_for(key, || Some(sub.get()).filter(|v| !v.is_empty()))
    };
    let chooser_table = dash.get_sub_table("Auto Chooser");
    let kind = text("Auto Chooser", ".type");
    let default = text("Auto Chooser", "default");
    let active = text("Auto Chooser", "active");
    let options = chooser_table
        .topic::<StringArray>("options")
        .subscribe(vec![], &Options::default());
    let options = wait_for("options", || Some(options.get()).filter(|v| !v.is_empty()));
    println!("chooser: type={kind:?} default={default:?} active={active:?} options={options:?}");
    assert_eq!(kind, "String Chooser");
    assert_eq!(options, ["Leave", "Score"]);

    // The dashboard picks an option; the server applies it on the next update.
    let selected = chooser_table
        .topic::<Text>("selected")
        .publish(&Options::default());
    selected.set("Score");
    client.flush();
    wait_for("chooser selection", || {
        builder.update();
        chooser.get_selected().filter(|v| *v == 2)
    });
    println!(
        "chooser selected after client write: {:?}",
        chooser.get_selected()
    );

    let field_obj = dash
        .get_sub_table("Field2d")
        .topic::<wpilib::nt::DoubleArray>("Robot")
        .subscribe(vec![], &Options::default());
    let got = wait_for("Field2d Robot", || {
        Some(field_obj.get()).filter(|v| !v.is_empty())
    });
    println!("Field2d Robot: {got:?}");
    assert_eq!(got, [4.0, 2.0, 45.0]);

    let angle = dash
        .get_sub_table("Mech")
        .get_sub_table("Root")
        .get_sub_table("Arm")
        .topic::<Double>("angle")
        .subscribe(0.0, &Options::default());
    let got = wait_for("Mechanism2d angle", || {
        Some(angle.get()).filter(|v| *v == 30.0)
    });
    println!("Mechanism2d Root/Arm angle: {got}");

    client.stop_client();
    server.stop_server();
    println!("nt_smoke ok");
}

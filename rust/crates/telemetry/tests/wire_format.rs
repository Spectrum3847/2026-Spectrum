//! One record of every data type, byte by byte against `wpiutil/doc/datalog.adoc`, then a file
//! with every type written through the background writer and read back.

use std::fs;
use std::time::Duration;
use telemetry::datalog::{Config, DataLog};
use telemetry::reader::{Log, decode};
use telemetry::wpilog::{self, payload, write_record};
use wpilib::nt::Struct;
use wpimath::geometry::{Pose2d, Rotation2d};

/// A record with entry id 1 at 1 second: `20 01 <size> 40 42 0f <payload>`.
fn record(payload: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    write_record(&mut out, 1, 1_000_000, payload);
    out
}

fn expect(size: u8, tail: &[u8]) -> Vec<u8> {
    [&[0x20, 0x01, size, 0x40, 0x42, 0x0f][..], tail].concat()
}

fn encode(f: impl FnOnce(&mut Vec<u8>)) -> Vec<u8> {
    let mut p = Vec::new();
    f(&mut p);
    record(&p)
}

#[test]
fn boolean() {
    assert_eq!(encode(|p| payload::boolean(p, true)), expect(1, &[1]));
    assert_eq!(encode(|p| payload::boolean(p, false)), expect(1, &[0]));
}

#[test]
fn int64() {
    assert_eq!(
        encode(|p| payload::int64(p, 3)),
        expect(8, &[3, 0, 0, 0, 0, 0, 0, 0])
    );
    assert_eq!(
        encode(|p| payload::int64(p, -2)),
        expect(8, &[0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff])
    );
}

#[test]
fn float() {
    assert_eq!(
        encode(|p| payload::float(p, 1.5)),
        expect(4, &[0x00, 0x00, 0xc0, 0x3f])
    );
}

#[test]
fn double() {
    assert_eq!(
        encode(|p| payload::double(p, 1.5)),
        expect(8, &[0, 0, 0, 0, 0, 0, 0xf8, 0x3f])
    );
}

#[test]
fn string() {
    assert_eq!(encode(|p| payload::string(p, "hi")), expect(2, b"hi"));
    assert_eq!(encode(|p| payload::string(p, "")), expect(0, &[]));
}

#[test]
fn raw() {
    assert_eq!(record(&[9, 8, 7]), expect(3, &[9, 8, 7]));
}

#[test]
fn boolean_array() {
    assert_eq!(
        encode(|p| payload::boolean_array(p, &[true, false, true])),
        expect(3, &[1, 0, 1])
    );
}

#[test]
fn int64_array() {
    assert_eq!(
        encode(|p| payload::int64_array(p, &[1, -2])),
        expect(
            16,
            &[
                1, 0, 0, 0, 0, 0, 0, 0, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
            ]
        )
    );
}

#[test]
fn float_array() {
    assert_eq!(
        encode(|p| payload::float_array(p, &[0.5, -0.25])),
        expect(8, &[0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x80, 0xbe])
    );
}

#[test]
fn double_array() {
    assert_eq!(
        encode(|p| payload::double_array(p, &[1.0, 2.0])),
        expect(
            16,
            &[0, 0, 0, 0, 0, 0, 0xf0, 0x3f, 0, 0, 0, 0, 0, 0, 0, 0x40]
        )
    );
}

#[test]
fn string_array() {
    assert_eq!(
        encode(|p| payload::string_array(p, &["a", "bc"])),
        expect(15, &[2, 0, 0, 0, 1, 0, 0, 0, b'a', 2, 0, 0, 0, b'b', b'c'])
    );
}

#[test]
fn struct_value() {
    let mut p = Vec::new();
    Pose2d::from_xy(1.0, 2.0, Rotation2d::new(0.5)).pack(&mut p);
    assert_eq!(
        record(&p),
        expect(
            24,
            &[
                0, 0, 0, 0, 0, 0, 0xf0, 0x3f, // 1.0
                0, 0, 0, 0, 0, 0, 0, 0x40, // 2.0
                0, 0, 0, 0, 0, 0, 0xe0, 0x3f, // 0.5
            ]
        )
    );
}

#[test]
fn header_with_extra_text_is_spec_layout() {
    let mut out = Vec::new();
    wpilog::write_header(&mut out, "abc");
    assert_eq!(
        out,
        [
            b'W', b'P', b'I', b'L', b'O', b'G', 0, 1, 3, 0, 0, 0, b'a', b'b', b'c'
        ]
    );
}

#[test]
fn every_type_round_trips_through_a_file() {
    let dir = std::env::temp_dir().join(format!("telemetry-wire-{}", std::process::id()));
    let _ = fs::remove_dir_all(&dir);
    fs::create_dir_all(&dir).unwrap();

    let mut config = Config::directory(&dir, "all.wpilog");
    config.period = Duration::from_millis(5);
    config.extra_header = "written by test".to_owned();
    let log = DataLog::new(config);

    let mut expected: Vec<(&str, &str, Vec<u8>)> = Vec::new();
    let mut add = |name: &'static str, type_name: &'static str, f: &dyn Fn(&mut Vec<u8>)| {
        let mut p = Vec::new();
        f(&mut p);
        expected.push((name, type_name, p));
    };
    add("boolean", "boolean", &|p| payload::boolean(p, true));
    add("int", "int64", &|p| payload::int64(p, i64::MIN));
    add("float", "float", &|p| payload::float(p, -0.125));
    add("double", "double", &|p| payload::double(p, 6.02e23));
    add("string", "string", &|p| {
        payload::string(p, "multi\nline \u{e9}")
    });
    add("raw", "raw", &|p| p.extend_from_slice(&[0, 255, 1]));
    add("bools", "boolean[]", &|p| {
        payload::boolean_array(p, &[true; 9])
    });
    add("ints", "int64[]", &|p| {
        payload::int64_array(p, &[i64::MAX, 0])
    });
    add("floats", "float[]", &|p| {
        payload::float_array(p, &[1.0, 2.0, 3.0])
    });
    add("doubles", "double[]", &|p| payload::double_array(p, &[]));
    add("strings", "string[]", &|p| {
        payload::string_array(p, &["x", "", "yz"])
    });

    for (name, type_name, p) in &expected {
        let id = log.start(name, type_name, "{}", 10).unwrap();
        // Two records, the second after a gap that needs a longer timestamp field.
        log.append(id, 20, p);
        log.append(id, 5_000_000_000, p);
    }
    drop(log);

    let parsed = Log::parse(&fs::read(dir.join("all.wpilog")).unwrap()).unwrap();
    assert_eq!(parsed.extra_header, "written by test");
    assert_eq!(parsed.entries.len(), expected.len());
    for (name, type_name, p) in &expected {
        let entry = parsed.entry(name).unwrap();
        assert_eq!(entry.type_name, *type_name);
        assert_eq!(entry.metadata, "{}");
        assert_eq!(entry.records, [(20, p.clone()), (5_000_000_000, p.clone())]);
    }
    assert_eq!(
        decode::int64(&parsed.entry("int").unwrap().records[0].1),
        Some(i64::MIN)
    );
    assert_eq!(
        decode::string(&parsed.entry("string").unwrap().records[0].1),
        "multi\nline \u{e9}"
    );
    assert_eq!(
        decode::string_array(&parsed.entry("strings").unwrap().records[0].1),
        Some(vec!["x".to_owned(), String::new(), "yz".to_owned()])
    );
    let _ = fs::remove_dir_all(&dir);
}

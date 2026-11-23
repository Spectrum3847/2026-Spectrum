//! WPILib DataLog file format 1.0, as specified in `wpiutil/doc/datalog.adoc` (WPILib
//! v2026.2.1). All integers are little endian.

pub const VERSION: u16 = 0x0100;

pub const CONTROL_START: u8 = 0;
pub const CONTROL_FINISH: u8 = 1;
pub const CONTROL_SET_METADATA: u8 = 2;

/// Largest possible record header: bitfield, 4 byte id, 4 byte size, 8 byte timestamp.
pub const MAX_RECORD_HEADER: usize = 17;

/// `WPILOG`, the version, then the length and text of the extra header.
pub fn write_header(out: &mut Vec<u8>, extra_header: &str) {
    out.extend_from_slice(b"WPILOG");
    out.extend_from_slice(&VERSION.to_le_bytes());
    out.extend_from_slice(&(extra_header.len() as u32).to_le_bytes());
    out.extend_from_slice(extra_header.as_bytes());
}

/// Bytes needed for `value`, at least one.
fn var_len(value: u64) -> usize {
    ((64 - value.leading_zeros()).div_ceil(8) as usize).max(1)
}

fn put_var(out: &mut Vec<u8>, value: u64, len: usize) {
    out.extend_from_slice(&value.to_le_bytes()[..len]);
}

/// Appends one record with the shortest header that holds its fields.
pub fn write_record(out: &mut Vec<u8>, entry: u32, timestamp: u64, payload: &[u8]) {
    write_record_header(out, entry, timestamp, payload.len());
    out.extend_from_slice(payload);
}

/// Appends only the header of a record whose payload is `payload_len` bytes.
pub fn write_record_header(out: &mut Vec<u8>, entry: u32, timestamp: u64, payload_len: usize) {
    let entry_len = var_len(u64::from(entry));
    let size_len = var_len(payload_len as u64);
    let time_len = var_len(timestamp);
    out.push((((time_len - 1) << 4) | ((size_len - 1) << 2) | (entry_len - 1)) as u8);
    put_var(out, u64::from(entry), entry_len);
    put_var(out, payload_len as u64, size_len);
    put_var(out, timestamp, time_len);
}

fn put_str(out: &mut Vec<u8>, text: &str) {
    out.extend_from_slice(&(text.len() as u32).to_le_bytes());
    out.extend_from_slice(text.as_bytes());
}

/// Start control record: names `entry`, its type string and its metadata.
pub fn write_start(
    out: &mut Vec<u8>,
    entry: u32,
    name: &str,
    type_name: &str,
    metadata: &str,
    timestamp: u64,
) {
    let mut payload = Vec::with_capacity(13 + name.len() + type_name.len() + metadata.len());
    payload.push(CONTROL_START);
    payload.extend_from_slice(&entry.to_le_bytes());
    put_str(&mut payload, name);
    put_str(&mut payload, type_name);
    put_str(&mut payload, metadata);
    write_record(out, 0, timestamp, &payload);
}

pub fn write_finish(out: &mut Vec<u8>, entry: u32, timestamp: u64) {
    let mut payload = vec![CONTROL_FINISH];
    payload.extend_from_slice(&entry.to_le_bytes());
    write_record(out, 0, timestamp, &payload);
}

/// Replaces the metadata of `entry`.
pub fn write_set_metadata(out: &mut Vec<u8>, entry: u32, metadata: &str, timestamp: u64) {
    let mut payload = vec![CONTROL_SET_METADATA];
    payload.extend_from_slice(&entry.to_le_bytes());
    put_str(&mut payload, metadata);
    write_record(out, 0, timestamp, &payload);
}

/// Record payloads of the standard data types. Each function appends to `out`.
pub mod payload {
    pub const BOOLEAN: &str = "boolean";
    pub const INT64: &str = "int64";
    pub const FLOAT: &str = "float";
    pub const DOUBLE: &str = "double";
    pub const STRING: &str = "string";
    pub const BOOLEAN_ARRAY: &str = "boolean[]";
    pub const INT64_ARRAY: &str = "int64[]";
    pub const FLOAT_ARRAY: &str = "float[]";
    pub const DOUBLE_ARRAY: &str = "double[]";
    pub const STRING_ARRAY: &str = "string[]";
    pub const RAW: &str = "raw";

    pub fn boolean(out: &mut Vec<u8>, value: bool) {
        out.push(u8::from(value));
    }

    pub fn int64(out: &mut Vec<u8>, value: i64) {
        out.extend_from_slice(&value.to_le_bytes());
    }

    pub fn float(out: &mut Vec<u8>, value: f32) {
        out.extend_from_slice(&value.to_le_bytes());
    }

    pub fn double(out: &mut Vec<u8>, value: f64) {
        out.extend_from_slice(&value.to_le_bytes());
    }

    pub fn string(out: &mut Vec<u8>, value: &str) {
        out.extend_from_slice(value.as_bytes());
    }

    pub fn boolean_array(out: &mut Vec<u8>, values: &[bool]) {
        out.extend(values.iter().map(|&v| u8::from(v)));
    }

    pub fn int64_array(out: &mut Vec<u8>, values: &[i64]) {
        for v in values {
            int64(out, *v);
        }
    }

    pub fn float_array(out: &mut Vec<u8>, values: &[f32]) {
        for v in values {
            float(out, *v);
        }
    }

    pub fn double_array(out: &mut Vec<u8>, values: &[f64]) {
        for v in values {
            double(out, *v);
        }
    }

    /// A 4 byte count, then each string as a 4 byte length and its bytes.
    pub fn string_array<S: AsRef<str>>(out: &mut Vec<u8>, values: &[S]) {
        out.extend_from_slice(&(values.len() as u32).to_le_bytes());
        for v in values {
            out.extend_from_slice(&(v.as_ref().len() as u32).to_le_bytes());
            out.extend_from_slice(v.as_ref().as_bytes());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_without_extra_text() {
        let mut out = Vec::new();
        write_header(&mut out, "");
        assert_eq!(
            out,
            [0x57, 0x50, 0x49, 0x4c, 0x4f, 0x47, 0x00, 0x01, 0, 0, 0, 0]
        );
    }

    #[test]
    fn header_with_extra_text() {
        let mut out = Vec::new();
        write_header(&mut out, "hi");
        assert_eq!(out[8..], [2, 0, 0, 0, b'h', b'i']);
    }

    // The examples in datalog.adoc, byte for byte.

    #[test]
    fn spec_integer_record() {
        let mut out = Vec::new();
        let mut payload = Vec::new();
        payload::int64(&mut payload, 3);
        write_record(&mut out, 1, 1_000_000, &payload);
        assert_eq!(
            out,
            [
                0x20, 0x01, 0x08, 0x40, 0x42, 0x0f, 0x03, 0, 0, 0, 0, 0, 0, 0
            ]
        );
    }

    #[test]
    fn spec_start_record() {
        let mut out = Vec::new();
        write_start(&mut out, 1, "test", "int64", "", 1_000_000);
        let want: Vec<u8> = [
            &[0x20, 0x00, 0x1a, 0x40, 0x42, 0x0f, 0x00][..],
            &[1, 0, 0, 0, 4, 0, 0, 0],
            b"test",
            &[5, 0, 0, 0],
            b"int64",
            &[0, 0, 0, 0],
        ]
        .concat();
        assert_eq!(out, want);
        assert_eq!(out.len(), 32);
    }

    #[test]
    fn spec_finish_record() {
        let mut out = Vec::new();
        write_finish(&mut out, 1, 1_000_000);
        assert_eq!(out, [0x20, 0x00, 0x05, 0x40, 0x42, 0x0f, 0x01, 1, 0, 0, 0]);
    }

    #[test]
    fn spec_set_metadata_record() {
        let mut out = Vec::new();
        write_set_metadata(&mut out, 1, "{\"source\":\"NT\"}", 1_000_000);
        let want: Vec<u8> = [
            &[0x20, 0x00, 0x18, 0x40, 0x42, 0x0f, 0x02][..],
            &[1, 0, 0, 0, 0x0f, 0, 0, 0],
            b"{\"source\":\"NT\"}",
        ]
        .concat();
        assert_eq!(out, want);
        assert_eq!(out.len(), 30);
    }

    #[test]
    fn header_fields_use_the_shortest_length() {
        let mut out = Vec::new();
        write_record(&mut out, 0x0102_0304, u64::MAX, &[0u8; 300]);
        // 4 byte id, 2 byte size, 8 byte timestamp
        assert_eq!(out[0], (7 << 4) | (1 << 2) | 3);
        assert_eq!(out[1..5], [4, 3, 2, 1]);
        assert_eq!(out[5..7], [0x2c, 0x01]);
        assert_eq!(out[7..15], [0xff; 8]);
        assert_eq!(out.len(), 15 + 300);

        out.clear();
        write_record(&mut out, 0, 0, &[]);
        assert_eq!(out, [0, 0, 0, 0]);
        assert_eq!(var_len(0xff), 1);
        assert_eq!(var_len(0x100), 2);
        assert_eq!(var_len(0xff_ffff), 3);
        assert_eq!(var_len(0x100_0000), 4);
    }

    #[test]
    fn payload_layouts() {
        let mut out = Vec::new();
        payload::boolean_array(&mut out, &[true, false, true]);
        assert_eq!(out, [1, 0, 1]);
        out.clear();
        payload::string_array(&mut out, &["a", "bc"]);
        assert_eq!(out, [2, 0, 0, 0, 1, 0, 0, 0, b'a', 2, 0, 0, 0, b'b', b'c']);
        out.clear();
        payload::float(&mut out, 1.0);
        assert_eq!(out, [0, 0, 0x80, 0x3f]);
        out.clear();
        payload::double_array(&mut out, &[1.0, 2.0]);
        assert_eq!(out.len(), 16);
        out.clear();
        payload::int64_array(&mut out, &[-1]);
        assert_eq!(out, [0xff; 8]);
    }
}

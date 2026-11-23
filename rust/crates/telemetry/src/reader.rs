//! Reads wpilog files back: raw records, control records and per entry value lists.

use crate::wpilog::{CONTROL_FINISH, CONTROL_SET_METADATA, CONTROL_START};
use std::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReadError {
    /// Missing `WPILOG` magic or a header cut short.
    BadHeader,
    /// The version is not 1.x.
    UnsupportedVersion(u16),
    /// The data ends inside a record.
    Truncated,
}

impl fmt::Display for ReadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BadHeader => write!(f, "not a wpilog file"),
            Self::UnsupportedVersion(v) => write!(f, "unsupported wpilog version {v:#06x}"),
            Self::Truncated => write!(f, "wpilog ends inside a record"),
        }
    }
}

impl std::error::Error for ReadError {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Control {
    Start {
        entry: u32,
        name: String,
        type_name: String,
        metadata: String,
    },
    Finish {
        entry: u32,
    },
    SetMetadata {
        entry: u32,
        metadata: String,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Record<'a> {
    pub entry: u32,
    pub timestamp: u64,
    pub payload: &'a [u8],
}

impl Record<'_> {
    /// The control record this is, or `None` for a data record or a malformed control record.
    pub fn control(&self) -> Option<Control> {
        if self.entry != 0 {
            return None;
        }
        let (&kind, rest) = self.payload.split_first()?;
        let entry = u32::from_le_bytes(rest.get(..4)?.try_into().ok()?);
        let rest = &rest[4..];
        match kind {
            CONTROL_START => {
                let (name, rest) = take_str(rest)?;
                let (type_name, rest) = take_str(rest)?;
                let (metadata, rest) = take_str(rest)?;
                rest.is_empty().then_some(Control::Start {
                    entry,
                    name,
                    type_name,
                    metadata,
                })
            }
            CONTROL_FINISH => rest.is_empty().then_some(Control::Finish { entry }),
            CONTROL_SET_METADATA => {
                let (metadata, rest) = take_str(rest)?;
                rest.is_empty()
                    .then_some(Control::SetMetadata { entry, metadata })
            }
            _ => None,
        }
    }
}

fn take_str(bytes: &[u8]) -> Option<(String, &[u8])> {
    let len = u32::from_le_bytes(bytes.get(..4)?.try_into().ok()?) as usize;
    let text = bytes.get(4..4usize.checked_add(len)?)?;
    Some((
        String::from_utf8_lossy(text).into_owned(),
        &bytes[4 + len..],
    ))
}

/// Walks the records of a log.
pub struct Reader<'a> {
    data: &'a [u8],
    pos: usize,
    extra_header: String,
}

impl<'a> Reader<'a> {
    pub fn new(data: &'a [u8]) -> Result<Self, ReadError> {
        if data.len() < 12 || &data[..6] != b"WPILOG" {
            return Err(ReadError::BadHeader);
        }
        let version = u16::from_le_bytes([data[6], data[7]]);
        if version >> 8 != 1 {
            return Err(ReadError::UnsupportedVersion(version));
        }
        let extra_len = u32::from_le_bytes([data[8], data[9], data[10], data[11]]) as usize;
        let extra = data.get(12..12 + extra_len).ok_or(ReadError::BadHeader)?;
        Ok(Self {
            data,
            pos: 12 + extra_len,
            extra_header: String::from_utf8_lossy(extra).into_owned(),
        })
    }

    pub fn extra_header(&self) -> &str {
        &self.extra_header
    }

    fn var(&mut self, len: usize) -> Result<u64, ReadError> {
        let bytes = self
            .data
            .get(self.pos..self.pos + len)
            .ok_or(ReadError::Truncated)?;
        self.pos += len;
        let mut word = [0u8; 8];
        word[..len].copy_from_slice(bytes);
        Ok(u64::from_le_bytes(word))
    }
}

impl<'a> Iterator for Reader<'a> {
    type Item = Result<Record<'a>, ReadError>;

    fn next(&mut self) -> Option<Self::Item> {
        let bits = *self.data.get(self.pos)?;
        self.pos += 1;
        let result = (|| {
            let entry = self.var(usize::from(bits & 3) + 1)? as u32;
            let size = self.var(usize::from((bits >> 2) & 3) + 1)? as usize;
            let timestamp = self.var(usize::from((bits >> 4) & 7) + 1)?;
            let payload = self
                .data
                .get(self.pos..self.pos.checked_add(size).ok_or(ReadError::Truncated)?)
                .ok_or(ReadError::Truncated)?;
            self.pos += size;
            Ok(Record {
                entry,
                timestamp,
                payload,
            })
        })();
        if result.is_err() {
            self.pos = self.data.len();
        }
        Some(result)
    }
}

/// One started entry and the values logged to it.
#[derive(Debug, Clone, PartialEq)]
pub struct Entry {
    pub id: u32,
    pub name: String,
    pub type_name: String,
    /// Metadata after the last set metadata record.
    pub metadata: String,
    pub finished: bool,
    /// `(timestamp, payload)` in file order.
    pub records: Vec<(u64, Vec<u8>)>,
    /// Metadata values in the order they were set, starting with the Start record's.
    pub metadata_history: Vec<String>,
}

/// A whole log, entries in the order they started.
#[derive(Debug, Clone, Default)]
pub struct Log {
    pub extra_header: String,
    pub entries: Vec<Entry>,
}

impl Log {
    /// Parses `data`. A log cut short keeps the records before the cut and reports the error
    /// through [`Log::parse_lossy`].
    pub fn parse(data: &[u8]) -> Result<Self, ReadError> {
        let (log, error) = Self::parse_lossy(data)?;
        match error {
            Some(e) => Err(e),
            None => Ok(log),
        }
    }

    /// Like [`Log::parse`], returning what was read before a truncation.
    pub fn parse_lossy(data: &[u8]) -> Result<(Self, Option<ReadError>), ReadError> {
        let reader = Reader::new(data)?;
        let mut log = Self {
            extra_header: reader.extra_header().to_owned(),
            entries: Vec::new(),
        };
        let mut live: std::collections::HashMap<u32, usize> = std::collections::HashMap::new();
        for record in reader {
            let record = match record {
                Ok(r) => r,
                Err(e) => return Ok((log, Some(e))),
            };
            if record.entry != 0 {
                if let Some(&at) = live.get(&record.entry) {
                    log.entries[at]
                        .records
                        .push((record.timestamp, record.payload.to_vec()));
                }
                continue;
            }
            match record.control() {
                Some(Control::Start {
                    entry,
                    name,
                    type_name,
                    metadata,
                }) => {
                    live.insert(entry, log.entries.len());
                    log.entries.push(Entry {
                        id: entry,
                        name,
                        type_name,
                        metadata_history: vec![metadata.clone()],
                        metadata,
                        finished: false,
                        records: Vec::new(),
                    });
                }
                Some(Control::Finish { entry }) => {
                    if let Some(at) = live.remove(&entry) {
                        log.entries[at].finished = true;
                    }
                }
                Some(Control::SetMetadata { entry, metadata }) => {
                    if let Some(&at) = live.get(&entry) {
                        log.entries[at].metadata_history.push(metadata.clone());
                        log.entries[at].metadata = metadata;
                    }
                }
                None => {}
            }
        }
        Ok((log, None))
    }

    pub fn entry(&self, name: &str) -> Option<&Entry> {
        self.entries.iter().find(|e| e.name == name)
    }

    /// Entry names, in start order.
    pub fn names(&self) -> Vec<&str> {
        self.entries.iter().map(|e| e.name.as_str()).collect()
    }
}

/// Decoders for the standard payloads. `None` when the length does not fit the type.
pub mod decode {
    pub fn boolean(p: &[u8]) -> Option<bool> {
        (p.len() == 1).then(|| p[0] != 0)
    }

    pub fn int64(p: &[u8]) -> Option<i64> {
        Some(i64::from_le_bytes(p.try_into().ok()?))
    }

    pub fn float(p: &[u8]) -> Option<f32> {
        Some(f32::from_le_bytes(p.try_into().ok()?))
    }

    pub fn double(p: &[u8]) -> Option<f64> {
        Some(f64::from_le_bytes(p.try_into().ok()?))
    }

    pub fn string(p: &[u8]) -> String {
        String::from_utf8_lossy(p).into_owned()
    }

    pub fn boolean_array(p: &[u8]) -> Vec<bool> {
        p.iter().map(|&b| b != 0).collect()
    }

    pub fn int64_array(p: &[u8]) -> Option<Vec<i64>> {
        p.len().is_multiple_of(8).then(|| {
            p.chunks_exact(8)
                .map(|c| i64::from_le_bytes(c.try_into().unwrap_or_default()))
                .collect()
        })
    }

    pub fn float_array(p: &[u8]) -> Option<Vec<f32>> {
        p.len().is_multiple_of(4).then(|| {
            p.chunks_exact(4)
                .map(|c| f32::from_le_bytes(c.try_into().unwrap_or_default()))
                .collect()
        })
    }

    pub fn double_array(p: &[u8]) -> Option<Vec<f64>> {
        p.len().is_multiple_of(8).then(|| {
            p.chunks_exact(8)
                .map(|c| f64::from_le_bytes(c.try_into().unwrap_or_default()))
                .collect()
        })
    }

    pub fn string_array(p: &[u8]) -> Option<Vec<String>> {
        let count = u32::from_le_bytes(p.get(..4)?.try_into().ok()?) as usize;
        let mut rest = &p[4..];
        let mut out = Vec::new();
        for _ in 0..count {
            let len = u32::from_le_bytes(rest.get(..4)?.try_into().ok()?) as usize;
            out.push(String::from_utf8_lossy(rest.get(4..4 + len)?).into_owned());
            rest = &rest[4 + len..];
        }
        rest.is_empty().then_some(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wpilog::{self, payload};

    fn log_with_one_entry() -> Vec<u8> {
        let mut out = Vec::new();
        wpilog::write_header(&mut out, "extra");
        wpilog::write_start(&mut out, 1, "a", "double", "{}", 10);
        let mut p = Vec::new();
        payload::double(&mut p, 2.5);
        wpilog::write_record(&mut out, 1, 20, &p);
        wpilog::write_set_metadata(&mut out, 1, "{\"u\":1}", 30);
        wpilog::write_finish(&mut out, 1, 40);
        out
    }

    #[test]
    fn parses_entries_and_controls() {
        let log = Log::parse(&log_with_one_entry()).unwrap();
        assert_eq!(log.extra_header, "extra");
        let entry = log.entry("a").unwrap();
        assert_eq!(entry.type_name, "double");
        assert_eq!(entry.metadata, "{\"u\":1}");
        assert_eq!(entry.metadata_history, ["{}", "{\"u\":1}"]);
        assert!(entry.finished);
        assert_eq!(decode::double(&entry.records[0].1), Some(2.5));
        assert_eq!(entry.records[0].0, 20);
    }

    #[test]
    fn rejects_bad_headers() {
        assert_eq!(Log::parse(b"nope").unwrap_err(), ReadError::BadHeader);
        let mut data = log_with_one_entry();
        data[7] = 2;
        assert_eq!(
            Log::parse(&data).unwrap_err(),
            ReadError::UnsupportedVersion(0x0200)
        );
    }

    #[test]
    fn truncation_keeps_earlier_records() {
        let mut data = log_with_one_entry();
        data.pop();
        assert_eq!(Log::parse(&data).unwrap_err(), ReadError::Truncated);
        let (log, error) = Log::parse_lossy(&data).unwrap();
        assert_eq!(error, Some(ReadError::Truncated));
        assert_eq!(log.entry("a").unwrap().records.len(), 1);
    }

    #[test]
    fn decoders_check_lengths() {
        assert_eq!(decode::double(&[0; 7]), None);
        assert_eq!(decode::boolean(&[]), None);
        assert_eq!(decode::string_array(&[1, 0, 0, 0, 9, 0, 0, 0]), None);
        assert_eq!(decode::double_array(&[0; 9]), None);
    }
}

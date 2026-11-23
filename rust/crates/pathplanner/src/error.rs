use std::fmt;

#[derive(Debug)]
pub enum Error {
    Io(std::io::Error),
    Parse(serde_json::Error),
    /// A file whose `version` field is not `2025.X`.
    Version {
        found: String,
        file: String,
    },
    Invalid(String),
    Unsupported(&'static str),
}

pub type Result<T> = std::result::Result<T, Error>;

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(e) => write!(f, "{e}"),
            Error::Parse(e) => write!(f, "{e}"),
            Error::Version { found, file } => {
                write!(f, "{file} has version {found}, expected 2025.X")
            }
            Error::Invalid(message) => f.write_str(message),
            Error::Unsupported(what) => write!(f, "{what} is not supported"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Io(e) => Some(e),
            Error::Parse(e) => Some(e),
            _ => None,
        }
    }
}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Error::Io(e)
    }
}

impl From<serde_json::Error> for Error {
    fn from(e: serde_json::Error) -> Self {
        Error::Parse(e)
    }
}

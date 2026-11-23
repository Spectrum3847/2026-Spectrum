use std::error::Error;
use std::fmt;

/// Raised to stop robot code when continuing would be unsafe. Java's is a
/// `RuntimeException`, so callers that must stop the program `panic!` with it.
#[derive(Debug)]
pub struct KillRobotException {
    message: Option<String>,
    cause: Option<Box<dyn Error + Send + Sync>>,
}

impl KillRobotException {
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: Some(message.into()),
            cause: None,
        }
    }

    /// The message is the cause's text, as `RuntimeException(Throwable)` sets it.
    pub fn from_cause(cause: impl Into<Box<dyn Error + Send + Sync>>) -> Self {
        let cause = cause.into();
        Self {
            message: Some(cause.to_string()),
            cause: Some(cause),
        }
    }

    pub fn with_cause(
        message: impl Into<String>,
        cause: impl Into<Box<dyn Error + Send + Sync>>,
    ) -> Self {
        Self {
            message: Some(message.into()),
            cause: Some(cause.into()),
        }
    }

    pub fn get_message(&self) -> Option<&str> {
        self.message.as_deref()
    }
}

impl fmt::Display for KillRobotException {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.message.as_deref().unwrap_or(""))
    }
}

impl Error for KillRobotException {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        self.cause.as_deref().map(|c| c as &(dyn Error + 'static))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn message_constructor_keeps_the_message() {
        let e = KillRobotException::new("stop");
        assert_eq!(e.get_message(), Some("stop"));
        assert_eq!(e.to_string(), "stop");
        assert!(e.source().is_none());
    }

    #[test]
    fn cause_constructor_uses_the_cause_text_as_the_message() {
        let e = KillRobotException::from_cause("inner failure");
        assert_eq!(e.get_message(), Some("inner failure"));
        assert_eq!(
            e.source().map(ToString::to_string).as_deref(),
            Some("inner failure")
        );
    }

    #[test]
    fn message_and_cause_constructor_keeps_both() {
        let e = KillRobotException::with_cause("outer", "inner");
        assert_eq!(e.to_string(), "outer");
        assert_eq!(
            e.source().map(ToString::to_string).as_deref(),
            Some("inner")
        );
    }
}

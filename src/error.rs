use std::fmt;

#[derive(Debug)]
pub enum Error {
    InvalidInput(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::InvalidInput(err) => write!(f, "InvalidInput: {}", err),
        }
    }
}

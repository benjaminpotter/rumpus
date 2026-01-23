use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Error {
    OddImgDim((usize, usize)),
    EmptyRange,
    NonFinite,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::OddImgDim((x, y)) => write!(f, "odd image dimensions: {}x{}", x, y),
            Error::EmptyRange => write!(f, "bounds define empty range"),
            Error::NonFinite => write!(f, "float cannot be infinite or NaN"),
        }
    }
}

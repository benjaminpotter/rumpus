#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Describes the intensity ratio of polarized light in a ray.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Dop {
    degree: f64,
}

impl Dop {
    /// Create a new `Dop` from `degree`.
    ///
    /// Panics if `degree` is not between 0.0 and 1.0.
    pub fn new(degree: f64) -> Self {
        assert!((0.0..=1.0).contains(&degree));
        Self { degree }
    }

    /// Create a new `Dop` of zero.
    pub fn zero() -> Self {
        Self { degree: 0. }
    }

    /// Returns a new `Dop` clamp between 0.0 and `max`.
    ///
    /// Panics if `max` is not between 0.0 and 1.0.
    pub fn clamp(self, max: f64) -> Self {
        assert!((0.0..=1.0).contains(&max));
        Self {
            degree: self.degree.clamp(0.0, max),
        }
    }

    pub fn into_inner(self) -> f64 {
        self.degree
    }
}

impl From<f64> for Dop {
    fn from(degree: f64) -> Self {
        Self { degree }
    }
}

impl Default for Dop {
    fn default() -> Self {
        Self::zero()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[should_panic]
    fn create_invalid_dop() {
        assert_eq!(-1.0, Dop::new(-1.0).degree);
    }
}

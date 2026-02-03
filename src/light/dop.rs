use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::light::LightError;

/// Describes the intensity ratio of polarized light in a ray.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Dop {
    inner: f64,
}

impl Dop {
    /// Create a new `Dop` from `degree`.
    #[must_use]
    #[deprecated]
    pub fn new(degree: f64) -> Option<Self> {
        if (0.0..=1.0).contains(&degree) {
            Some(Self { inner: degree })
        } else {
            None
        }
    }

    /// Create a new `Dop` from `degree`.
    #[must_use]
    pub fn try_new(degree: f64) -> Result<Self, LightError> {
        if (0.0..=1.0).contains(&degree) {
            Ok(Self { inner: degree })
        } else {
            Err(LightError::DegreeOutOfBounds { degree })
        }
    }

    #[must_use]
    pub fn clamped(degree: f64) -> Self {
        Self {
            inner: degree.clamp(0., 1.),
        }
    }

    /// Create a new `Dop` of zero.
    #[must_use]
    pub fn zero() -> Self {
        Self { inner: 0. }
    }
}

impl Default for Dop {
    fn default() -> Self {
        Self::zero()
    }
}

impl From<Dop> for f64 {
    fn from(dop: Dop) -> Self {
        dop.inner
    }
}

impl<Rhs: Into<f64>> Add<Rhs> for Dop {
    type Output = Self;

    fn add(self, rhs: Rhs) -> Self::Output {
        Self::clamped(self.inner + rhs.into())
    }
}

impl<Rhs: Into<f64>> AddAssign<Rhs> for Dop {
    fn add_assign(&mut self, rhs: Rhs) {
        // self + rhs is guaranteed to be clamped to [0, 1]
        *self = *self + rhs;
    }
}

impl<Rhs: Into<f64>> Sub<Rhs> for Dop {
    type Output = Self;

    fn sub(self, rhs: Rhs) -> Self::Output {
        Self::clamped(self.inner - rhs.into())
    }
}

impl<Rhs: Into<f64>> SubAssign<Rhs> for Dop {
    fn sub_assign(&mut self, rhs: Rhs) {
        // self + rhs is guaranteed to be clamped to [0, 1]
        *self = *self - rhs;
    }
}

// Multiply a Dop by some f64-like type and get a Dop back.
impl<Rhs: Into<f64>> Mul<Rhs> for Dop {
    type Output = Self;

    fn mul(self, rhs: Rhs) -> Self::Output {
        Self::clamped(self.inner * rhs.into())
    }
}

impl<Rhs: Into<f64>> MulAssign<Rhs> for Dop {
    fn mul_assign(&mut self, rhs: Rhs) {
        *self = *self * rhs;
    }
}

// Multiply an f64 by a Dop and get an f64 back.
// Useful when weighting f64 values by a Dop.
impl Mul<Dop> for f64 {
    type Output = Self;

    fn mul(self, rhs: Dop) -> Self::Output {
        self * rhs.inner
    }
}

impl MulAssign<Dop> for f64 {
    fn mul_assign(&mut self, rhs: Dop) {
        *self = *self * rhs;
    }
}

impl<Rhs: Into<f64>> Div<Rhs> for Dop {
    type Output = Self;

    fn div(self, rhs: Rhs) -> Self::Output {
        Self::clamped(self.inner / rhs.into())
    }
}

impl<Rhs: Into<f64>> DivAssign<Rhs> for Dop {
    fn div_assign(&mut self, rhs: Rhs) {
        *self = *self / rhs;
    }
}

impl Div<Dop> for f64 {
    type Output = Self;

    fn div(self, rhs: Dop) -> Self::Output {
        self / rhs.inner
    }
}

impl DivAssign<Dop> for f64 {
    fn div_assign(&mut self, rhs: Dop) {
        *self = *self / rhs;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[should_panic]
    fn create_invalid_dop() {
        Dop::new(-1.0).unwrap();
    }
}

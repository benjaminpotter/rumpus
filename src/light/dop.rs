/// Describes the intensity ratio of polarized light in a ray.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Dop {
    degree: f64,
}

impl Dop {
    /// Create a new `Dop` from `degree`.
    ///
    /// Panics if `degree` is not between 0.0 and 1.0.
    pub fn new(degree: f64) -> Self {
        assert!(0.0 <= degree && degree <= 1.0);
        Self { degree }
    }

    /// Returns a new `Dop` clamp between 0.0 and `max`.
    ///
    /// Panics if `max` is not between 0.0 and 1.0.
    pub fn clamp(self, max: f64) -> Self {
        assert!(0.0 <= max && max <= 1.0);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[should_panic]
    fn create_invalid_dop() {
        assert_eq!(-1.0, Dop::new(-1.0).degree);
    }
}

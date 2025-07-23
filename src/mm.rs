use std::f64::consts::FRAC_PI_2;

/// Describes the linear polarization of a ray.
#[derive(Debug, PartialEq)]
pub struct StokesVec {
    inner: [f64; 3],
}

impl StokesVec {
    pub fn new(s0: f64, s1: f64, s2: f64) -> Self {
        StokesVec {
            inner: [s0, s1, s2],
        }
    }

    /// Compute the AoP of the ray.
    fn aop(&self) -> Aop {
        Aop {
            angle: (self.inner[2].atan2(self.inner[1]) / 2.).to_degrees(),
        }
    }

    /// Compute the DoP of the ray.
    fn dop(&self) -> Dop {
        Dop {
            degree: (self.inner[1].powf(2.) + self.inner[2].powf(2.)).sqrt() / self.inner[0],
        }
    }
}

/// Describes the e-vector orientation of a ray.
#[derive(Debug, PartialEq)]
pub struct Aop {
    /// The angle of the e-vector of the ray in degrees.
    angle: f64,
}

impl Aop {
    /// Creates a new `Aop` from `angle` given in radians.
    ///
    /// Panics if `angle` is not between -PI/2 and PI/2.
    pub fn from_rad(angle: f64) -> Self {
        assert!(-FRAC_PI_2 <= angle && angle <= FRAC_PI_2);
        Self {
            angle: angle.to_degrees(),
        }
    }

    /// Creates a new `Aop` from `angle` given in degrees.
    ///
    /// Panics if `angle` is not between -90.0 and 90.0.
    pub fn from_deg(angle: f64) -> Self {
        assert!(-90.0 <= angle && angle <= 90.0);
        Self { angle }
    }
}

/// Describes the intensity ratio of polarized light in a ray.
#[derive(Debug, PartialEq, PartialOrd)]
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
}

impl From<f64> for Dop {
    fn from(degree: f64) -> Self {
        Self { degree }
    }
}

/// Describes the angle and degree of polarization for a single ray.
#[derive(Debug, PartialEq)]
pub struct Ray {
    /// Location of pixel that measured the ray.
    loc: (u32, u32),
    angle: Aop,
    degree: Dop,
}

impl Ray {
    pub fn new(loc: (u32, u32), angle: Aop, degree: Dop) -> Self {
        Self { loc, angle, degree }
    }

    pub fn from_stokes(loc: (u32, u32), stokes: StokesVec) -> Self {
        Self {
            loc,
            angle: stokes.aop(),
            degree: stokes.dop(),
        }
    }

    pub fn get_loc(&self) -> &(u32, u32) {
        &self.loc
    }

    pub fn get_dop(&self) -> &Dop {
        &self.degree
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_4, PI};

    #[test]
    fn create_aop_from_rad() {
        assert_eq!(45.0, Aop::from_rad(FRAC_PI_4).angle);
    }

    #[test]
    #[should_panic]
    fn create_invalid_aop() {
        assert_eq!(180.0, Aop::from_rad(PI).angle);
    }

    #[test]
    #[should_panic]
    fn create_invalid_dop() {
        assert_eq!(-1.0, Dop::new(-1.0).degree);
    }
}

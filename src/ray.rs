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
///
/// The angle of the e-vector must be between -90.0 and 90.0.
#[derive(Clone, Copy, Debug, PartialEq)]
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

    fn from_deg_wrap(mut angle: f64) -> Self {
        if angle < -90.0 {
            while angle < 0.0 {
                angle += 180.0;
            }
        } else if angle > 90.0 {
            while angle > 0.0 {
                angle -= 180.0;
            }
        }

        Self::from_deg(angle)
    }

    /// Returns true if `other` is within `thres` of `self` inclusive and
    /// handling wrapping.
    pub fn in_thres(&self, other: &Aop, thres: f64) -> bool {
        (*self - *other).angle.abs() <= thres
    }
}

impl std::ops::Sub for Aop {
    type Output = Self;

    fn sub(self, other: Self) -> Self::Output {
        Self::from_deg_wrap(self.angle - other.angle)
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

// Ray should probably operate with respect to the ENU frame.
// Right now it uses the image frame.
//
// Motivation:
// The estimation algorithms require the global frame to handle non-zero pitch
// and roll values.
//
// - Used in ht to compute angle of ray wrt east
// - Making the assumption that matching the transform to the zenith pixel
// - zenith position is only constrained by three angles
//
// SensorParams should be called CameraParams
// Should hold a state
// State should hold a position time and pose
// Each ray should get a Vector3 that corresponds to the global frame
// Shoots from the pixel location into the sky
// Directly corresponds to the sensor's ray in enu frame
//
// Image rays needs to take a CameraParams in order to generate this

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

    pub fn get_aop(&self) -> &Aop {
        &self.angle
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

    #[test]
    fn sub_aop() {
        assert_eq!(
            Aop::from_deg(1.0),
            Aop::from_deg(-90.0) - Aop::from_deg(89.0)
        );

        assert_eq!(
            Aop::from_deg(0.0),
            Aop::from_deg(-90.0) - Aop::from_deg(90.0)
        );

        assert_eq!(
            Aop::from_deg(0.0),
            Aop::from_deg(-90.0) - Aop::from_deg(-90.0)
        );
    }

    #[test]
    fn threshold_aop() {
        const THRES: f64 = 0.1;
        let center = Aop::from_deg(90.0);

        for ref case in vec![
            Aop::from_deg(89.90),
            Aop::from_deg(-90.0),
            Aop::from_deg(-89.90),
        ] {
            assert_eq!(true, center.in_thres(case, THRES));
        }

        for ref case in vec![
            Aop::from_deg(89.89),
            Aop::from_deg(45.0),
            Aop::from_deg(-89.89),
        ] {
            assert_eq!(false, center.in_thres(case, THRES));
        }
    }
}

use super::ray::RayFrame;
use std::f64::consts::FRAC_PI_2;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Describes the e-vector orientation of a ray.
///
/// The angle of the e-vector must be between -90.0 and 90.0.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Aop<Frame: RayFrame> {
    /// The angle of the e-vector of the ray in degrees.
    angle: f64,
    _phan: std::marker::PhantomData<Frame>,
}

impl<Frame: RayFrame> Aop<Frame> {
    /// Creates a new `Aop` from `angle` given in radians.
    ///
    /// Panics if `angle` is not between -PI/2 and PI/2.
    pub fn from_rad(angle: f64) -> Self {
        assert!(-FRAC_PI_2 <= angle && angle <= FRAC_PI_2);
        Self::from_deg(angle.to_degrees())
    }

    /// Creates a new `Aop` from `angle` given in degrees.
    ///
    /// Panics if `angle` is not between -90.0 and 90.0.
    pub fn from_deg(angle: f64) -> Self {
        assert!(-90.0 <= angle && angle <= 90.0);
        Self {
            angle,
            _phan: std::marker::PhantomData,
        }
    }

    /// Creates a new `Aop` from `angle` given in degrees causing values not
    /// between -90.0 and 90.0 to be wrapped.
    pub fn from_deg_wrap(mut angle: f64) -> Self {
        while angle < -90.0 {
            angle += 180.0;
        }

        while angle > 90.0 {
            angle -= 180.0;
        }

        Self::from_deg(angle)
    }

    /// Returns true if `other` is within `thres` of `self` inclusive and
    /// handling wrapping.
    pub fn in_thres(&self, other: &Aop<Frame>, thres: f64) -> bool {
        (*self - *other).angle.abs() <= thres
    }

    /// Returns a raw `f64` angle in degrees.
    pub fn degrees(&self) -> f64 {
        self.angle
    }

    /// Returns a raw `f64` angle in radians.
    pub fn radians(&self) -> f64 {
        self.angle.to_radians()
    }

    pub fn into_inner(self) -> f64 {
        self.angle
    }
}

impl<Frame: RayFrame> std::ops::Add for Aop<Frame> {
    type Output = Self;

    fn add(self, other: Self) -> Self::Output {
        Self::from_deg_wrap(self.angle + other.angle)
    }
}

impl<Frame: RayFrame> std::ops::Add<f64> for Aop<Frame> {
    type Output = Self;

    fn add(self, other: f64) -> Self::Output {
        Self::from_deg_wrap(self.angle + other)
    }
}

impl<Frame: RayFrame> std::ops::Sub for Aop<Frame> {
    type Output = Self;

    fn sub(self, other: Self) -> Self::Output {
        Self::from_deg_wrap(self.angle - other.angle)
    }
}

impl<Frame: RayFrame> std::ops::Sub<f64> for Aop<Frame> {
    type Output = Self;

    fn sub(self, other: f64) -> Self::Output {
        Self::from_deg_wrap(self.angle - other)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::light::ray::GlobalFrame;
    use std::f64::consts::{FRAC_PI_4, PI};

    #[test]
    fn create_aop_from_rad() {
        assert_eq!(45.0, Aop::<GlobalFrame>::from_rad(FRAC_PI_4).angle);
    }

    #[test]
    #[should_panic]
    fn create_invalid_aop() {
        assert_eq!(180.0, Aop::<GlobalFrame>::from_rad(PI).angle);
    }

    #[test]
    fn add_aop() {
        assert_eq!(
            Aop::<GlobalFrame>::from_deg(1.0),
            Aop::<GlobalFrame>::from_deg(90.0) + Aop::from_deg(-89.0)
        );
    }

    #[test]
    fn add_aop_with_f64() {
        assert_eq!(
            Aop::<GlobalFrame>::from_deg(1.0),
            Aop::<GlobalFrame>::from_deg(-90.0) - 89.0
        );
    }

    #[test]
    fn sub_aop() {
        assert_eq!(
            Aop::<GlobalFrame>::from_deg(1.0),
            Aop::<GlobalFrame>::from_deg(-90.0) - Aop::from_deg(89.0)
        );

        assert_eq!(
            Aop::<GlobalFrame>::from_deg(0.0),
            Aop::<GlobalFrame>::from_deg(-90.0) - Aop::from_deg(90.0)
        );

        assert_eq!(
            Aop::<GlobalFrame>::from_deg(0.0),
            Aop::<GlobalFrame>::from_deg(-90.0) - Aop::from_deg(-90.0)
        );
    }

    #[test]
    fn sub_aop_with_f64() {
        assert_eq!(
            Aop::<GlobalFrame>::from_deg(1.0),
            Aop::<GlobalFrame>::from_deg(-90.0) - 89.0
        );
    }

    #[test]
    fn threshold_aop() {
        const THRES: f64 = 0.1;
        let center = Aop::<GlobalFrame>::from_deg(90.0);

        for ref case in vec![
            Aop::<GlobalFrame>::from_deg(89.90),
            Aop::<GlobalFrame>::from_deg(-90.0),
            Aop::<GlobalFrame>::from_deg(-89.90),
        ] {
            assert_eq!(true, center.in_thres(case, THRES));
        }

        for ref case in vec![
            Aop::<GlobalFrame>::from_deg(89.89),
            Aop::<GlobalFrame>::from_deg(45.0),
            Aop::<GlobalFrame>::from_deg(-89.89),
        ] {
            assert_eq!(false, center.in_thres(case, THRES));
        }
    }
}

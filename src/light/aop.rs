use crate::light::ray::{GlobalFrame, RayFrame, SensorFrame};
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use uom::si::f64::Angle;

/// Describes the e-vector orientation of a ray.
///
/// The angle of the e-vector must be between -90.0 and 90.0.
#[derive(Clone, Copy, Debug, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Aop<Frame: RayFrame> {
    /// The angle of the e-vector of the ray.
    angle: Angle,
    _phan: std::marker::PhantomData<Frame>,
}

impl<Frame: RayFrame> Aop<Frame> {
    /// Creates a new `Aop` from `angle`.
    ///
    /// Returns `None` if `angle` is not between -90 and 90.
    pub fn from_angle(angle: Angle) -> Option<Self> {
        if !is_valid(angle) {
            return None;
        }

        Some(Self {
            angle,
            _phan: std::marker::PhantomData,
        })
    }

    /// Creates a new `Aop` from `angle` wrapping into -90.0 and 90.0 to be wrapped.
    pub fn from_angle_wrapped(mut angle: Angle) -> Self {
        while angle > Angle::HALF_TURN / 2. {
            angle -= Angle::HALF_TURN;
        }

        while angle < -Angle::HALF_TURN / 2. {
            angle += Angle::HALF_TURN;
        }

        // Expect is enforced by the while loops above.
        Self::from_angle(angle).expect("angle is within range -90 to 90")
    }

    /// Returns true if `other` is within `thres` of `self` inclusive and
    /// handling wrapping.
    pub fn in_thres(&self, other: &Aop<Frame>, thres: Angle) -> bool {
        (*self - *other).angle.abs() <= thres
    }

    pub fn into_inner(self) -> Angle {
        self.angle
    }
}

/// Returns `true` if `angle` is between -90 and 90, `false` otherwise.
fn is_valid(angle: Angle) -> bool {
    -Angle::HALF_TURN / 2. <= angle && angle <= Angle::HALF_TURN / 2.
}

impl Aop<GlobalFrame> {
    /// Transforms the `Aop` from the GlobalFrame into the SensorFrame.
    pub fn into_sensor_frame(self, shift: Angle) -> Aop<SensorFrame> {
        Aop::from_angle_wrapped(self.angle + shift)
    }
}

impl Aop<SensorFrame> {
    /// Transforms the `Aop` from the SensorFrame into the GlobalFrame.
    pub fn into_global_frame(self, shift: Angle) -> Aop<GlobalFrame> {
        Aop::from_angle_wrapped(self.angle - shift)
    }
}

impl<Frame: RayFrame> std::ops::Add for Aop<Frame> {
    type Output = Self;

    fn add(self, other: Self) -> Self::Output {
        Self::from_angle_wrapped(self.angle + other.angle)
    }
}

impl<Frame: RayFrame> std::ops::Sub for Aop<Frame> {
    type Output = Self;

    fn sub(self, other: Self) -> Self::Output {
        Self::from_angle_wrapped(self.angle - other.angle)
    }
}

impl<Frame: RayFrame> std::cmp::PartialEq for Aop<Frame> {
    fn eq(&self, other: &Aop<Frame>) -> bool {
        match self.angle.abs() == Angle::HALF_TURN / 2.
            && other.angle.abs() == Angle::HALF_TURN / 2.
        {
            // Handle the case that -90 is the same as 90.
            true => true,
            false => self.angle == other.angle,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::light::ray::GlobalFrame;
    use approx::assert_relative_eq;
    use quickcheck::quickcheck;
    use rstest::rstest;
    use uom::si::angle::{degree, radian};

    fn a(angle: f64) -> Angle {
        Angle::new::<degree>(angle)
    }

    quickcheck! {
        fn aop_from_wrapped(angle: i8) -> bool {
            // Will panic if it tries to create an invalid Aop.
            // Should never panic due to wrapping.
            Aop::<GlobalFrame>::from_angle_wrapped(a(angle as f64));

            // If we didn't panic, call this test a success.
            true
        }
    }

    #[rstest]
    #[case(a(180.0))]
    #[case(a(91.0))]
    fn invalid_aop(#[case] angle: Angle) {
        assert_eq!(Aop::<GlobalFrame>::from_angle(angle), None,);
    }

    #[rstest]
    #[case(a(90.0), a(-89.0), a(1.0))]
    fn add_aop(#[case] lhs: Angle, #[case] rhs: Angle, #[case] sum: Angle) {
        let result = Aop::<GlobalFrame>::from_angle(lhs).unwrap() + Aop::from_angle(rhs).unwrap();
        assert_relative_eq!(result.into_inner().get::<radian>(), sum.get::<radian>(),);
    }

    #[rstest]
    #[case(a(-90.0), a(89.0), a(1.0))]
    #[case(a(-90.0), a(90.0), a(0.0))]
    #[case(a(-90.0), a(-90.0), a(0.0))]
    fn sub_aop(#[case] lhs: Angle, #[case] rhs: Angle, #[case] dif: Angle) {
        let result = Aop::<GlobalFrame>::from_angle(lhs).unwrap() - Aop::from_angle(rhs).unwrap();
        assert_relative_eq!(result.into_inner().get::<radian>(), dif.get::<radian>());
    }

    #[rstest]
    #[case(a(90.0), a(89.9), a(0.1), true)]
    #[case(a(90.0), a(-90.0), a(0.1), true)]
    #[case(a(90.0), a(-89.9), a(0.1), true)]
    #[case(a(90.0), a(89.89), a(0.1), false)]
    #[case(a(90.0), a(45.0), a(0.1), false)]
    #[case(a(90.0), a(-89.89), a(0.1), false)]
    fn threshold_aop(
        #[case] center: Angle,
        #[case] case: Angle,
        #[case] thres: Angle,
        #[case] in_thres: bool,
    ) {
        assert_eq!(
            Aop::<GlobalFrame>::from_angle(center)
                .unwrap()
                .in_thres(&Aop::<GlobalFrame>::from_angle(case).unwrap(), thres,),
            in_thres
        );
    }

    #[rstest]
    #[case(a(0.0), a(0.0))]
    #[case(a(0.0), a(90.0))]
    #[case(a(1.0), a(90.0))]
    #[case(a(-1.0), a(180.0))]
    fn frame_reversible(#[case] angle: Angle, #[case] offset: Angle) {
        assert_relative_eq!(
            Aop::<SensorFrame>::from_angle_wrapped(angle)
                .into_global_frame(offset.clone())
                .into_sensor_frame(offset)
                .into_inner()
                .get::<radian>(),
            angle.get::<radian>(),
        );
    }
}

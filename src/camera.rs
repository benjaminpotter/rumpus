use std::marker::PhantomData;

use nalgebra::Vector2;
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use sguaba::{
    Bearing, Coordinate, Vector, coordinate, engineering::Orientation, math::RigidBodyTransform,
};
use uom::{ConstZero, si::f64::*};

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SensorCoordinate<In> {
    coordinate: Vector2<f64>,
    _in: PhantomData<In>,
}

impl<In> SensorCoordinate<In> {
    pub fn new() -> Self {
        Self {
            // TODO: This needs to be passed in somehow.
            coordinate: Vector2::default(),
            _in: PhantomData,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RayBearing<In> {
    // TODO: This needs to hold actual data.
    // TODO: RayBearing should always be above the horizon.
    _in: PhantomData<In>,
}

impl<In> RayBearing<In> {
    pub fn new() -> Self {
        Self { _in: PhantomData }
    }
}

pub trait Camera<In> {
    fn trace_backward(&self, coord: SensorCoordinate<In>) -> RayBearing<In>;
    fn trace_forward(&self, bearing: RayBearing<In>) -> SensorCoordinate<In>;
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PinholeCamera {
    focal_length: Length,
}

impl PinholeCamera {
    pub fn from_focal_length(focal_length: Length) -> Self {
        if focal_length <= Length::ZERO {
            panic!("focal length must be greater than zero");
        }

        Self { focal_length }
    }
}

impl<In> Camera<In> for PinholeCamera {
    fn trace_backward(&self, coord: SensorCoordinate<In>) -> RayBearing<In> {
        // Trace a ray from the physical pixel location through the focal point.
        // This approach uses the pinhole camera model.
        // let ray_in_frd: Coordinate<CameraFrd> = coord
        //     + Vector::<CameraFrd>::builder()
        //         .frd_front(Length::ZERO)
        //         .frd_right(Length::ZERO)
        //         .frd_down(self.lens.focal_length)
        //         .build();

        // TODO: Implement
        RayBearing::new()
    }

    fn trace_forward(&self, bearing: RayBearing<In>) -> SensorCoordinate<In> {
        // let len = -self.lens.focal_length / bearing_frd.elevation().tan();
        // Some(
        //     coordinate! { f = len * bearing_frd.azimuth().cos(), r = len * bearing_frd.azimuth().sin(), d = Length::ZERO },
        // )

        // TODO: Implement
        SensorCoordinate::new()
    }
}

pub struct Simulation<Cam, CamIn, In> {
    camera: Cam,
    transform: RigidBodyTransform<CamIn, In>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::relative_eq;
    use quickcheck::quickcheck;
    use uom::si::{
        angle::degree,
        length::{micron, millimeter},
    };

    quickcheck! {
        fn trace_roundtrip(
            x_seed: i16,
            y_seed: i16,
            roll_seed: i8,
            pitch_seed: i8,
            yaw_seed: u16
        ) -> quickcheck::TestResult {
            // Construct pixel coordinates from the random seed passed by quickcheck.
            // Aim to have pixel coordinates on range -5000 to 5000 microns.
            let x = Length::new::<micron>(x_seed as f64 * 5000. / i16::MAX as f64);
            let y = Length::new::<micron>(y_seed as f64 * 5000. / i16::MAX as f64);
            let px = Coordinate::<CameraFrd>::builder()
                .frd_front(x)
                .frd_right(y)
                .frd_down(Length::ZERO)
                .build();

            // Construct orientation from the random seed passed by quickcheck.
            // Aim to have roll and pitch on the range -30 to 30.
            // Aim to have yaw on the range 0 to 360.
            let roll = Angle::new::<degree>(roll_seed as f64 * 30. / i8::MAX as f64);
            let pitch = Angle::new::<degree>(pitch_seed as f64 * 30. / i8::MAX as f64);
            let yaw = Angle::new::<degree>(yaw_seed as f64 * 360. / u16::MAX as f64);
            let ort = Orientation::<CameraEnu>::tait_bryan_builder()
                .yaw(yaw)
                .pitch(pitch)
                .roll(roll)
                .build();

            let focal_length = Length::new::<millimeter>(8.0);
            let lens =
                Lens::from_focal_length(focal_length).expect("focal_length is greater than zero");

            let cam = Camera::new(lens, ort);
            match cam.trace_from_sky(
                cam.trace_from_sensor(px.clone())
                    .expect("px has a z value of zero"),
            ) {
                Some(result) =>
                    quickcheck::TestResult::from_bool(relative_eq!(px, result)),
                // `trace_from_sensor` may produce a bearing that is below the horizon.
                None => quickcheck::TestResult::discard(),
            }
        }
    }
}

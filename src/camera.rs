use super::{CameraEnu, CameraFrd};
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use sguaba::{coordinate, engineering::Orientation, system, Bearing, Coordinate, Vector};
use uom::{si::f64::*, ConstZero};

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Lens {
    focal_length: Length,
}

impl Lens {
    pub fn from_focal_length(focal_length: Length) -> Option<Self> {
        if focal_length > Length::ZERO {
            return Some(Self { focal_length });
        }

        None
    }
}

/// Represents a simulated sensor in the world.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Camera {
    lens: Lens,
    ort: Orientation<CameraEnu>,
}

impl Camera {
    pub fn new(lens: Lens, ort: Orientation<CameraEnu>) -> Self {
        Self { lens, ort }
    }

    /// Trace a point on an imaginary sensor through the lens and into the ENU
    /// frame.
    pub fn trace_from_sensor(&self, coord: Coordinate<CameraFrd>) -> Option<Bearing<CameraEnu>> {
        // Point must be on sensor.
        if coord.frd_down() != Length::ZERO {
            return None;
        }

        // Trace a ray from the physical pixel location through the focal point.
        // This approach uses the pinhole camera model.
        let mut ray_in_frd: Coordinate<CameraFrd> = coord
            + Vector::<CameraFrd>::builder()
                .frd_front(Length::ZERO)
                .frd_right(Length::ZERO)
                .frd_down(self.lens.focal_length)
                .build();

        // Since we are mapping the FRD's zero to the ENU orientation, the down
        // vector will match the ENU up vector.
        // SAFETY: The CameraFrd system only differs from the CameraEnu system
        // by its rotation *not* any translation.
        let camera_frd_to_enu = unsafe { self.ort.map_as_zero_in::<CameraFrd>() }.inverse();
        let ray_in_enu = camera_frd_to_enu.transform(ray_in_frd);

        Some(
            ray_in_enu
                .bearing_from_origin()
                // Enforced by check that point.z() == Length::ZERO.
                .expect("ray_in_enu should never be colocated with CameraEnu's origin"),
        )
    }

    /// Trace a sky point from the ENU frame into the body frame and through the
    /// lens.
    pub fn trace_from_sky(&self, bearing: Bearing<CameraEnu>) -> Option<Coordinate<CameraFrd>> {
        // Bearing should be above the horizon.
        if bearing.elevation() < Angle::ZERO {
            return None;
        }

        // Since we are mapping the FRD's zero to the ENU orientation, the down
        // vector will match the ENU up vector.
        // SAFETY: The CameraFrd system only differs from the CameraEnu system
        // by rotation *not* translation.
        let camera_enu_to_frd = unsafe { self.ort.map_as_zero_in::<CameraFrd>() };
        let bearing_frd = camera_enu_to_frd.transform(bearing);

        // Project the bearing onto the sensor plane.
        // Bearing in FRD has azimuth taken CCW from X (forward) when looking
        // up from positive Z (down) and elevation is positive toward negative Z.
        // See: https://docs.rs/sguaba/latest/sguaba/systems/trait.BearingDefined.html
        let len = -self.lens.focal_length / bearing_frd.elevation().tan();
        Some(
            coordinate! { f = len * bearing_frd.azimuth().cos(), r = len * bearing_frd.azimuth().sin(), d = Length::ZERO },
        )
    }
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

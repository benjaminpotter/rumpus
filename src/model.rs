use crate::{CameraEnu, light::aop::Aop, ray::GlobalFrame};
use chrono::prelude::*;
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use sguaba::{Bearing, systems::Wgs84};
use uom::{
    ConstZero,
    si::{angle::degree, f64::Angle},
};

/// Describes the skylight polarization pattern for a given earth centered
/// (`Wgs84`) position and a UTC timepoint.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SkyModel {
    /// The location of the sun's center for an observer on the ground.
    solar_bearing: Bearing<CameraEnu>,
}

impl SkyModel {
    /// Create a `SkyModel` from a `solar_bearing`.
    pub fn from_solar_bearing(solar_bearing: Bearing<CameraEnu>) -> Self {
        Self { solar_bearing }
    }

    /// Create a new `SkyModel` from a `pos` and a `time`.
    pub fn from_wgs84_and_time(pos: Wgs84, time: DateTime<Utc>) -> Self {
        // Given a lon, lat, and time, compute the solar azimuth and zenith angle.
        let solar_pos = spa::solar_position::<spa::StdFloatOps>(
            time,
            pos.latitude().get::<degree>(),
            pos.longitude().get::<degree>(),
        )
        // Using `Wgs84` should enforce this.
        .expect("latitude and longitude are valid");

        Self::from_solar_bearing(
            Bearing::<CameraEnu>::builder()
                .azimuth(Angle::new::<degree>(solar_pos.azimuth))
                // Convert the zenith angle into an elevation angle.
                // The elevation is taken from the XY plane towards positive Z.
                .elevation(Angle::HALF_TURN / 2. - Angle::new::<degree>(solar_pos.zenith_angle))
                .expect("solar zenith should be on the range 0 to 180")
                .build(),
        )
    }

    /// Returns the `Bearing<CameraEnu>` towards the sun.
    pub fn solar_bearing(&self) -> Bearing<CameraEnu> {
        self.solar_bearing
    }

    /// Use the `SkyModel` to compute an `Aop` in the `GlobalFrame` at `bearing`.
    ///
    /// Returns `None` if `bearing` is below the horizon ie it has elevation
    /// less than zero.
    pub fn aop(&self, bearing: Bearing<CameraEnu>) -> Option<Aop<GlobalFrame>> {
        if bearing.elevation() < Angle::ZERO {
            return None;
        }

        let solar_azimuth = self.solar_bearing.azimuth();
        let solar_zenith = Angle::HALF_TURN / 2. - self.solar_bearing.elevation();
        let azimuth = bearing.azimuth();
        let zenith = Angle::HALF_TURN / 2. - bearing.elevation();
        let angle = (zenith.sin() * solar_zenith.cos()
            - zenith.cos() * (azimuth - solar_azimuth).cos() * solar_zenith.sin())
        .atan2((azimuth - solar_azimuth).sin() * solar_zenith.sin());

        Some(Aop::from_angle_wrapped(angle))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::relative_eq;
    use quickcheck::quickcheck;
    use uom::si::angle::degree;

    quickcheck! {
        fn solar_meridian_ortho_aop(flip_azimuth: bool, elevation_seed: u8) -> bool {
            let elevation = Angle::new::<degree>(elevation_seed as f64 * 90. / u8::MAX as f64);
            let azimuth = match flip_azimuth {
                true => Angle::new::<degree>(180.0),
                false => Angle::new::<degree>(0.0),
            };

            relative_eq!(
                SkyModel::from_solar_bearing(
                    Bearing::<CameraEnu>::builder()
                        .azimuth(Angle::new::<degree>(0.0))
                        .elevation(Angle::new::<degree>(45.0))
                        .expect("solar elevation should be on the range -90 to 90")
                        .build(),
                )
                .aop(
                    Bearing::<CameraEnu>::builder()
                        .azimuth(azimuth)
                        .elevation(elevation)
                        .expect("elevation should be on the range -90 to 90")
                        .build(),
                )
                .expect("bearing is above the horizon")
                .into_inner()
                .get::<degree>()
                .abs(),
                90.0
            )
        }
    }
}

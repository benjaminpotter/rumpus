use crate::light::dop::Dop;
use crate::{light::aop::Aop, ray::GlobalFrame};
use chrono::prelude::*;
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use sguaba::CoordinateSystem;
use sguaba::systems::EnuLike;
use sguaba::{Bearing, systems::Wgs84};
use uom::{
    ConstZero,
    si::{angle::degree, f64::Angle, ratio::ratio},
};

/// Describes the skylight polarization pattern for a given earth centered
/// (`Wgs84`) position and a UTC timepoint.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SkyModel<In> {
    /// The location of the sun's center for an observer on the ground.
    solar_bearing: Bearing<In>,
}

impl<In> SkyModel<In> {
    /// Create a `SkyModel` from a `solar_bearing`.
    pub fn from_solar_bearing(solar_bearing: Bearing<In>) -> Self {
        Self { solar_bearing }
    }

    /// Create a new [`SkyModel`] from a position and a time.
    ///
    /// # Safety
    /// This function only produces a valid [`SkyModel`] if the origin of `In` is coincident with
    /// `position`. Otherwise, the model will interpret the solar bearing from `position`, but
    /// return results that interpret bearings from the origin of `In`.
    pub unsafe fn from_position_and_time(
        position: impl Into<Wgs84>,
        time: impl Into<DateTime<Utc>>,
    ) -> Self
    where
        In: CoordinateSystem<Convention = EnuLike>,
    {
        // Given a lon, lat, and time, compute the solar azimuth and zenith angle.
        let position = position.into();
        let solar_pos = spa::solar_position::<spa::StdFloatOps>(
            time.into(),
            position.latitude().get::<degree>(),
            position.longitude().get::<degree>(),
        )
        // Using `Wgs84` should enforce this.
        .expect("latitude and longitude are valid");

        Self::from_solar_bearing(
            Bearing::<In>::builder()
                .azimuth(Angle::new::<degree>(solar_pos.azimuth))
                // Convert the zenith angle into an elevation angle.
                // The elevation is taken from the XY plane towards positive Z.
                .elevation(Angle::HALF_TURN / 2. - Angle::new::<degree>(solar_pos.zenith_angle))
                .expect("solar zenith should be on the range 0 to 180")
                .build(),
        )
    }

    /// Returns the [`Bearing`] towards the sun.
    pub fn solar_bearing(&self) -> Bearing<In> {
        self.solar_bearing
    }

    /// Use the [`SkyModel`] to compute an [`Aop`] in the [`GlobalFrame`] at `bearing`.
    ///
    /// Returns `None` if `bearing` is below the horizon ie it has elevation
    /// less than zero.
    pub fn aop(&self, bearing: Bearing<In>) -> Option<Aop<GlobalFrame>> {
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

    /// Use the `SkyModel` to compute a `Dop` at `bearing`.
    ///
    /// Returns `None` if `bearing` is below the horizon ie it has elevation
    /// less than zero.
    pub fn dop(&self, bearing: Bearing<In>) -> Option<Dop> {
        if bearing.elevation() < Angle::ZERO {
            return None;
        }

        let max_dop = 1.0;
        let solar_azimuth = self.solar_bearing.azimuth();
        let solar_zenith = Angle::HALF_TURN / 2. - self.solar_bearing.elevation();
        let azimuth = bearing.azimuth();
        let zenith = Angle::HALF_TURN / 2. - bearing.elevation();
        let scattering_angle = (zenith.cos() * solar_zenith.cos()
            + zenith.sin() * solar_zenith.sin() * (azimuth - solar_azimuth).cos())
        .acos();
        let deg = max_dop * scattering_angle.sin().get::<ratio>().powf(2.0)
            / (1.0 + scattering_angle.cos().get::<ratio>().powf(2.0));

        Some(Dop::new(deg).unwrap())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::relative_eq;
    use quickcheck::quickcheck;
    use sguaba::system;
    use uom::si::angle::degree;

    system!(struct ModelEnu using ENU);

    quickcheck! {
        fn solar_meridian_ortho_aop(flip_azimuth: bool, elevation_seed: u8) -> bool {
            let elevation = Angle::new::<degree>(elevation_seed as f64 * 90. / u8::MAX as f64);
            let azimuth = match flip_azimuth {
                true => Angle::new::<degree>(180.0),
                false => Angle::new::<degree>(0.0),
            };

            relative_eq!(
                Into::<Angle>::into(SkyModel::from_solar_bearing(
                    Bearing::<ModelEnu>::builder()
                        .azimuth(Angle::new::<degree>(0.0))
                        .elevation(Angle::new::<degree>(45.0))
                        .expect("solar elevation should be on the range -90 to 90")
                        .build(),
                )
                .aop(
                    Bearing::<ModelEnu>::builder()
                        .azimuth(azimuth)
                        .elevation(elevation)
                        .expect("elevation should be on the range -90 to 90")
                        .build(),
                )
                .expect("bearing is above the horizon"))
                .get::<degree>()
                .abs(),
                90.0
            )
        }
    }
}

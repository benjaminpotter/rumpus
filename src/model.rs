use super::{
    light::{
        aop::Aop,
        dop::Dop,
        ray::{GlobalFrame, Ray, RayLocation},
    },
    state::Position,
};
use chrono::prelude::*;
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use spa::{SolarPos, StdFloatOps};

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SkyPoint {
    /// Azimuth angle clock-wise from +Y.
    azi: f64,

    /// Zenith angle from +Z in plane defined by +Z and azimuth.
    zen: f64,
}

impl SkyPoint {
    pub fn new(azi: f64, zen: f64) -> Self {
        Self { azi, zen }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RayleighModel {
    sun_pos: SkyPoint,
}

impl RayleighModel {
    pub fn new(pos: Position, time: DateTime<Utc>) -> Self {
        // Given a lon, lat, and time, compute the solar azimuth and zenith angle.
        let solar_pos: SolarPos =
            spa::solar_position::<StdFloatOps>(time, pos.lat, pos.lon).unwrap();

        Self {
            sun_pos: SkyPoint {
                // Measured CW from north.
                azi: solar_pos.azimuth,

                // Measured between zenith and sun's center.
                zen: solar_pos.zenith_angle,
            },
        }
    }

    pub fn aop(&self, sky_point: SkyPoint) -> Aop<GlobalFrame> {
        Aop::from_rad(
            ((sky_point.zen.sin() * self.sun_pos.zen.cos()
                - sky_point.zen.cos()
                    * (sky_point.azi - self.sun_pos.azi).cos()
                    * self.sun_pos.zen.sin())
                / (sky_point.azi - self.sun_pos.azi).sin()
                / self.sun_pos.zen.sin())
            .atan(),
        )
    }
}

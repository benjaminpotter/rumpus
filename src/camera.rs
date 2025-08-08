use super::{
    light::{
        aop::Aop,
        dop::Dop,
        ray::{GlobalFrame, Ray, RayLocation},
    },
    state::{Orientation, Position},
};
use chrono::{DateTime, Utc};
use nalgebra::{Vector2, Vector3};
use spa::{SolarPos, StdFloatOps};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

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
pub struct Lens {
    focal_length: f64,
}

impl Lens {
    pub fn new(focal_length: f64) -> Self {
        Self { focal_length }
    }
}

/// Represents a simulated sensor in the world.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Camera {
    lens: Lens,
    ort: Orientation,
}

impl Camera {
    pub fn new(lens: Lens, ort: Orientation) -> Self {
        Self { lens, ort }
    }

    /// Simulate a ray from `ray_loc` using `model`.
    pub fn simulate_ray(&self, ray_loc: RayLocation, model: &RayleighModel) -> Ray<GlobalFrame> {
        let sp = self.trace_from_sensor(ray_loc.as_vec2());
        Ray::new(
            ray_loc,
            model.aop(sp),
            Dop::new(0.0),
            // model.dop(ray_azimuth_rad, ray_zenith_rad),
        )
    }

    /// Trace a point on an imaginary sensor through the lens and into the ENU
    /// frame.
    pub fn trace_from_sensor(&self, point: &Vector2<f64>) -> SkyPoint {
        // Trace a ray from the physical pixel location through the focal point.
        // This approach uses the pinhole camera model.
        let mut ray: Vector3<_> = Vector3::new(point.x, point.y, self.lens.focal_length);

        // Transform ray from body (sensor) frame into the ENU frame.
        ray = self.ort.as_rot() * ray;

        // Compute the zenith and azimuth angles of the ray.
        SkyPoint {
            // Azimuth is CW from +Y.
            azi: ray.x.atan2(ray.y),
            // Zenith is angle from +Z in the plane defined by +Z and the ray.
            zen: ray.normalize().z.acos(),
        }
    }

    /// Trace a sky point from the ENU frame into the body frame and through the
    /// lens.
    pub fn trace_from_sky(&self, sky_point: &SkyPoint) -> Vector2<f64> {
        // Create a vector pointing to sky_point with arbitrary length.
        // Let ||sp|| = 1
        //
        // Then,
        // a = sin(zen)
        //
        // So,
        // x = a * sin(azi)
        // y = a * cos(azi)
        // z = cos(zen)
        let a = sky_point.zen.sin();
        let sp = Vector3::new(
            a * sky_point.azi.sin(),
            a * sky_point.azi.cos(),
            sky_point.zen.cos(),
        );

        // Rotate vector into body frame.
        let mut sp_body: Vector3<_> = self.ort.as_rot().transpose() * sp;

        // Set vector length based on focal length.
        sp_body.set_magnitude(self.lens.focal_length / sky_point.zen.cos());

        // Return the sensor point.
        Vector2::new(sp_body.x, sp_body.y)
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

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Rotation3, Vector2};

    #[test]
    fn trace_from_optical_center() {
        let cam = Camera::new(
            Lens::new(8.0),
            Orientation::new(Rotation3::from_euler_angles(0.0, 0.0, 0.0)),
        );

        let point = Vector2::new(0.0, 0.0);
        assert_eq!(
            cam.trace_from_sensor(&point),
            SkyPoint { azi: 0.0, zen: 0.0 }
        );
    }

    #[test]
    fn trace_from_zenith() {
        let cam = Camera::new(
            Lens::new(8.0),
            Orientation::new(Rotation3::from_euler_angles(0.0, 0.0, 0.0)),
        );
        let point = SkyPoint { azi: 0.0, zen: 0.0 };
        assert_eq!(cam.trace_from_sky(&point), Vector2::new(0.0, 0.0));
    }

    #[test]
    fn trace_reversible() {
        let cam = Camera::new(
            Lens::new(8.0),
            Orientation::new(Rotation3::from_euler_angles(0.0, 0.0, 0.0)),
        );

        let point = Vector2::new(0.0, 0.0);
        assert_eq!(cam.trace_from_sky(&cam.trace_from_sensor(&point)), point);
    }
}

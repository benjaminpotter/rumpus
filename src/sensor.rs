use nalgebra::{Rotation3, Vector3};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::fmt;

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct SensorParams {
    pub pixel_size_um: (f32, f32),
    pub sensor_size_px: (u32, u32),
    pub focal_length_mm: f32,

    /// Follows east, north, up reference frame.
    pub enu_pose_deg: (f32, f32, f32),

    // TODO: Specify lon, lat, time to determine solar vector.
    /// Follows zenith angle from up and azimuth angle from north.
    pub solar_vector_deg: (f32, f32),
}

impl fmt::Display for SensorParams {
    fn fmt(&self, _f: &mut fmt::Formatter<'_>) -> fmt::Result {
        unimplemented!();
    }
}

/// Represents a simulated sensor in the world.
pub struct Sensor {
    pixel_size_mm: Vector3<f32>,
    sensor_size_px: Vector3<f32>,
    focal_point_mm: Vector3<f32>,
    enu_to_body: Rotation3<f32>,
    body_to_enu: Rotation3<f32>,
    solar_vector_rad: (f32, f32),
}

impl From<SensorParams> for Sensor {
    fn from(params: SensorParams) -> Self {
        // Convert pixel size to mm.
        let pixel_size_mm = Vector3::new(
            params.pixel_size_um.0 / 1000.,
            0.,
            params.pixel_size_um.1 / 1000.,
        );

        // Convert sensor size to floating point.
        let sensor_size_px = Vector3::new(
            (params.sensor_size_px.0) as f32,
            0.,
            (params.sensor_size_px.1) as f32,
        );

        let focal_point_mm = Vector3::new(0., params.focal_length_mm, 0.);

        let (roll_rad, pitch_rad, yaw_rad) = (
            params.enu_pose_deg.0.to_radians(),
            params.enu_pose_deg.1.to_radians(),
            params.enu_pose_deg.2.to_radians(),
        );

        // Given roll, pitch, yaw of the body frame wrt the ENU frame.
        // Given a vector U in the ENU frame, U in the body frame can be calculated using the rotation matrix.
        let enu_to_body = Rotation3::from_euler_angles(roll_rad, pitch_rad, yaw_rad);

        // Given a vector V in the body frame, V in the ENU frame can be calculated using the rotation matrix.
        let body_to_enu = enu_to_body.transpose();

        let solar_vector_rad = (
            params.solar_vector_deg.0.to_radians(),
            params.solar_vector_deg.1.to_radians(),
        );

        Self {
            pixel_size_mm,
            sensor_size_px,
            focal_point_mm,
            enu_to_body,
            body_to_enu,
            solar_vector_rad,
        }
    }
}

impl Sensor {
    /// Simulate a pixel using the Rayleigh sky model.
    pub fn simulate_pixel(&self, pixel: &(u32, u32)) -> (f32, f32) {
        // Compute physical pixel location on image sensor.
        let pixel = Vector3::new(pixel.0 as f32, 0., pixel.1 as f32);
        let pixel = pixel - self.sensor_size_px * 0.5;
        let phys_loc = self.pixel_size_mm.component_mul(&pixel);

        // Trace a ray from the physical pixel location through the focal point.
        // This approach uses the pinhole camera model.
        let body_ray = phys_loc + self.focal_point_mm;

        // Transform ray from body (sensor) frame into the solar frame.
        let enu_ray = self.body_to_enu * body_ray;
        let enu_ray = enu_ray.normalize();

        // Compute the zenith and azimuth angles of the ray.
        let ray_azimuth_rad = enu_ray.x.atan2(enu_ray.y);
        let ray_zenith_rad = enu_ray.z.acos();

        // Apply Rayleigh sky model using zenith and azimuth of ray to compute AoP and DoP.
        let (solar_azimuth_rad, solar_zenith_rad) = self.solar_vector_rad;

        let dop = 0.;
        let aop_rad = ((ray_zenith_rad.sin() * solar_zenith_rad.cos()
            - ray_zenith_rad.cos()
                * (ray_azimuth_rad - solar_azimuth_rad).cos()
                * solar_zenith_rad.sin())
            / (ray_azimuth_rad - solar_azimuth_rad).sin()
            / solar_zenith_rad.sin())
        .atan();

        (aop_rad.to_degrees(), dop)
    }

    /// Simulates the specified pixels in parallel.
    /// Returns a row major vector of simulated measurements.
    pub fn par_simulate_pixels(&self, pixels: &Vec<(u32, u32)>) -> Vec<(f32, f32)> {
        pixels
            .par_iter()
            .map(|pixel| self.simulate_pixel(pixel))
            .collect()
    }
}

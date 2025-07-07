use chrono::{DateTime, Utc};
use nalgebra::{Rotation3, Vector3};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use spa::{SolarPos, StdFloatOps};
use std::fmt;

/// A serializable data structure used to construct a simulated sensor.
#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct SensorParams {
    /// Size of a pixel on the simulated sensor in micrometers.
    pub pixel_size_um: (f64, f64),

    /// The dimensions of the simulated sensor in number of pixels.
    pub sensor_size_px: (u32, u32),

    /// The distance between the simulated sensor and the focal point along the +Z axis.
    pub focal_length_mm: f64,

    /// The Euler angles of the simulated sensor in the ENU reference frame.
    pub enu_pose_deg: (f64, f64, f64),

    pub lon: f64,
    pub lat: f64,
    pub time: DateTime<Utc>,
}

impl fmt::Display for SensorParams {
    fn fmt(&self, _f: &mut fmt::Formatter<'_>) -> fmt::Result {
        unimplemented!();
    }
}

impl Default for SensorParams {
    fn default() -> Self {
        Self {
            pixel_size_um: (3.45, 3.45),
            sensor_size_px: (2448, 2048),
            focal_length_mm: 8.0,
            enu_pose_deg: (0., 0., 0.),
            lat: 44.2187,
            lon: -76.4747,
            // TODO: Maybe do not use the UTC now?
            time: "2025-06-13T16:26:47+00:00"
                .parse::<DateTime<Utc>>()
                .unwrap(),
        }
    }
}

impl SensorParams {
    pub fn num_pixels(&self) -> usize {
        (self.sensor_size_px.0 * self.sensor_size_px.1) as usize
    }

    pub fn pixels(&self) -> Vec<(u32, u32)> {
        let width = self.sensor_size_px.0;
        let height = self.sensor_size_px.1;

        (0..height)
            .into_iter()
            .map(|row| (0..width).into_iter().map(move |col| (col, row)))
            .flatten()
            .collect()
    }

    pub fn to_pose(&self, pose_deg: (f64, f64, f64)) -> Self {
        let mut copy = self.clone();
        copy.enu_pose_deg = pose_deg;

        copy
    }
}

/// Represents a simulated sensor in the world.
pub struct Sensor {
    pixel_size_mm: Vector3<f64>,
    sensor_size_px: Vector3<f64>,
    focal_point_mm: Vector3<f64>,
    enu_to_body: Rotation3<f64>,
    body_to_enu: Rotation3<f64>,
    solar_vector_rad: (f64, f64),
}

impl From<&SensorParams> for Sensor {
    fn from(params: &SensorParams) -> Self {
        // Convert pixel size to mm.
        let pixel_size_mm = Vector3::new(
            params.pixel_size_um.0 / 1000.,
            params.pixel_size_um.1 / 1000.,
            0.,
        );

        // Convert sensor size to floating point.
        let sensor_size_px = Vector3::new(
            (params.sensor_size_px.0) as f64,
            (params.sensor_size_px.1) as f64,
            0.,
        );

        // Focal point is in the +z direction (optical axis).
        let focal_point_mm = Vector3::new(0., 0., params.focal_length_mm);

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

        // Given a lon, lat, and time, compute the solar azimuth and zenith angle.
        let solar_pos: SolarPos =
            spa::solar_position::<StdFloatOps>(params.time, params.lat, params.lon).unwrap();
        let solar_vector_rad: (f64, f64) = (
            // Measured CW from north.
            solar_pos.azimuth,
            // Measured between zenith and sun's center.
            solar_pos.zenith_angle,
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
    pub fn simulate_pixel(&self, pixel: &(u32, u32)) -> (f64, f64) {
        // Compute physical pixel location on image sensor.
        let pixel = Vector3::new(pixel.0 as f64, pixel.1 as f64, 0.);
        let pixel = pixel - self.sensor_size_px * 0.5;
        let phys_loc = self.pixel_size_mm.component_mul(&pixel);

        // Trace a ray from the physical pixel location through the focal point.
        // This approach uses the pinhole camera model.
        let body_ray = phys_loc + self.focal_point_mm;

        // Transform ray from body (sensor) frame into the ENU frame.
        let enu_ray = self.body_to_enu * body_ray;
        let enu_ray = enu_ray.normalize();

        // Compute the zenith and azimuth angles of the ray.
        // Azimuth is CW from +Y.
        // Zenith is angle from +Z in the plane defined by +Z and the ray.
        let ray_azimuth_rad = enu_ray.x.atan2(enu_ray.y);
        let ray_zenith_rad = enu_ray.z.acos();

        // Apply Rayleigh sky model using zenith and azimuth of ray to compute AoP and DoP.
        let (solar_azimuth_rad, solar_zenith_rad) = self.solar_vector_rad;

        // TODO: Compute DoP using scattering angle.
        let dop = 0.;
        let aop_rad = ((ray_zenith_rad.sin() * solar_zenith_rad.cos()
            - ray_zenith_rad.cos()
                * (ray_azimuth_rad - solar_azimuth_rad).cos()
                * solar_zenith_rad.sin())
            / (ray_azimuth_rad - solar_azimuth_rad).sin()
            / solar_zenith_rad.sin())
        .atan();

        // let z = solar_zenith_rad.cos();
        // let a = (1. - z.powf(2.)).sqrt();
        // let x = a * solar_azimuth_rad.sin();
        // let y = a * solar_azimuth_rad.cos();
        // let enu_solar_vector = Vector3::new(x, y, z);
        // let enu_e_vector = enu_ray.cross(&enu_solar_vector).normalize();
        // let body_e_vector = self.enu_to_body * enu_e_vector;

        // let beta_rad = phys_loc.y.atan2(phys_loc.x);
        // let z_mer = -body_ray;
        // let y_mer = Vector3::new(-beta_rad.sin(), beta_rad.cos(), 0.);
        // let x_mer = y_mer.cross(&z_mer);

        // let aop_rad = (body_e_vector.dot(&y_mer) / body_e_vector.dot(&x_mer)).atan();

        (aop_rad.to_degrees(), dop)
    }

    /// Simulates the specified pixels in parallel.
    /// Returns a vector of pixels in the same order they were provided.
    pub fn par_simulate_pixels(&self, pixels: &Vec<(u32, u32)>) -> Vec<(f64, f64)> {
        pixels
            .par_iter()
            .map(|pixel| self.simulate_pixel(pixel))
            .collect()
    }
}

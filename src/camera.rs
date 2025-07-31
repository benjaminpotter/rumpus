use crate::{
    ray::{Aop, Dop, Ray},
    state::State,
};
use nalgebra::{Rotation3, Vector3};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use spa::{SolarPos, StdFloatOps};
use std::fmt;

/// A serializable data structure used to construct a simulated camera.
#[derive(Clone, Copy, Serialize, Deserialize, Debug)]
pub struct CameraParams {
    /// Size of a pixel on the simulated sensor in micrometers.
    pub pixel_size_um: (f64, f64),

    /// The dimensions of the simulated sensor in number of pixels.
    pub sensor_size_px: (u32, u32),

    /// The distance between the simulated sensor and the focal point along the +Z axis.
    pub focal_length_mm: f64,
}

impl fmt::Display for CameraParams {
    fn fmt(&self, _f: &mut fmt::Formatter<'_>) -> fmt::Result {
        unimplemented!();
    }
}

impl Default for CameraParams {
    fn default() -> Self {
        Self {
            pixel_size_um: (3.45, 3.45),
            sensor_size_px: (2448, 2048),
            focal_length_mm: 8.0,
        }
    }
}

impl CameraParams {
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
}

/// Represents a simulated sensor in the world.
pub struct Camera {
    pixel_size_mm: Vector3<f64>,
    sensor_size_px: Vector3<f64>,
    focal_point_mm: Vector3<f64>,
    body_to_enu: Rotation3<f64>,
    solar_vector_rad: (f64, f64),
}

impl Camera {
    pub fn new(params: &CameraParams, state: State) -> Self {
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

        let (pose, position, time) = state.into_inner();

        // Given roll, pitch, yaw of the body frame wrt the ENU frame.
        // Given a vector U in the ENU frame, U in the body frame can be calculated using the rotation matrix.
        let enu_to_body: Rotation3<_> = pose.into();

        // Given a vector V in the body frame, V in the ENU frame can be calculated using the rotation matrix.
        let body_to_enu = enu_to_body.transpose();

        // Given a lon, lat, and time, compute the solar azimuth and zenith angle.
        let solar_pos: SolarPos =
            spa::solar_position::<StdFloatOps>(time, position.lat, position.lon).unwrap();
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
            body_to_enu,
            solar_vector_rad,
        }
    }
}

impl Camera {
    /// Simulate a pixel using the Rayleigh sky model.
    pub fn simulate_pixel(&self, pixel_location: &(u32, u32)) -> Ray {
        // Compute physical pixel location on image sensor.
        let pixel = Vector3::new(pixel_location.0 as f64, pixel_location.1 as f64, 0.);
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

        Ray::new(*pixel_location, Aop::from_rad(aop_rad), Dop::new(dop))
    }

    /// Simulates the specified pixels in parallel.
    /// Returns a vector of pixels in the same order they were provided.
    pub fn par_simulate_pixels(&self, pixels: &Vec<(u32, u32)>) -> Vec<Ray> {
        pixels
            .par_iter()
            .map(|pixel| self.simulate_pixel(pixel))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::{Pose, Position};
    use chrono::prelude::*;

    /// Test simulation algorithm using regression.
    /// Using commit with hash 4019279...
    #[test]
    fn regress_simulate() {
        let params = CameraParams {
            pixel_size_um: (3.45, 3.45),
            sensor_size_px: (2448, 2048),
            focal_length_mm: 8.0,
        };

        let state = State::new(
            Pose::zeros(),
            Position {
                lat: 44.2187,
                lon: -76.4747,
            },
            "2025-06-13T16:26:47+00:00"
                .parse::<DateTime<Utc>>()
                .unwrap(),
        );

        let rays = vec![
            Ray::new((0, 0), Aop::from_deg(-37.420423616444666), Dop::new(0.0)),
            Ray::new((2448, 0), Aop::from_deg(-77.361022793184380), Dop::new(0.0)),
            Ray::new((0, 2048), Aop::from_deg(40.030473803771110), Dop::new(0.0)),
            Ray::new(
                (2448, 2048),
                Aop::from_deg(62.284743078589490),
                Dop::new(0.0),
            ),
        ];

        let cam = Camera::new(&params, state);
        let pixels: Vec<(u32, u32)> = rays.iter().map(|ray| *ray.get_loc()).collect();
        assert_eq!(rays, cam.par_simulate_pixels(&pixels));
    }
}

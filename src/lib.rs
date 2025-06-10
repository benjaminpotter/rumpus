use nalgebra::{Matrix3, Vector3};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::fmt;

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct SensorParams {
    pub pixel_size_um: (f32, f32),
    pub sensor_size_px: (u32, u32),
    pub focal_length_mm: f32,
    pub solar_pose_deg: (f32, f32, f32),
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
    solar_to_body: Matrix3<f32>,
    body_to_solar: Matrix3<f32>,
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

        let (yaw_deg, pitch_deg, roll_deg) = params.solar_pose_deg;
        let solar_pose_rad = (
            yaw_deg.to_radians(),
            pitch_deg.to_radians(),
            roll_deg.to_radians(),
        );
        let solar_to_body = Sensor::solar_to_body_from_pose(solar_pose_rad);
        let body_to_solar = solar_to_body.transpose();

        Self {
            pixel_size_mm,
            sensor_size_px,
            focal_point_mm,
            solar_to_body,
            body_to_solar,
        }
    }
}

impl Sensor {
    /// Solar to Body Frame Rotation Matrix
    ///
    /// This rotation matrix transforms vectors from the solar vector coordinate system
    /// to the body coordinate system using three Euler angles.
    ///
    /// # Parameters
    ///
    /// * ψ - yaw angle (radians)
    /// * α - pitch angle (radians)
    /// * β - roll angle (radians)
    ///
    /// # Matrix Structure
    ///
    /// ```text
    /// [R₁₁  R₁₂  R₁₃]
    /// [R₂₁  R₂₂  R₂₃]
    /// [R₃₁  R₃₂  R₃₃]
    /// ```
    ///
    /// # Components
    ///
    /// ## First Row:
    /// - **R₁₁** = `cos(β)cos(ψ) + sin(β)sin(α)sin(ψ)`
    /// - **R₁₂** = `-cos(β)sin(ψ) + sin(β)sin(α)cos(ψ)`
    /// - **R₁₃** = `-sin(β)cos(α)`
    ///
    /// ## Second Row:
    /// - **R₂₁** = `cos(α)sin(ψ)`
    /// - **R₂₂** = `cos(α)cos(ψ)`
    /// - **R₂₃** = `sin(α)`
    ///
    /// ## Third Row:
    /// - **R₃₁** = `sin(β)cos(ψ) - cos(β)sin(α)sin(ψ)`
    /// - **R₃₂** = `-sin(β)sin(ψ) - cos(β)sin(α)cos(ψ)`
    /// - **R₃₃** = `cos(β)cos(α)`
    fn solar_to_body_from_pose(pose_solar_rad: (f32, f32, f32)) -> Matrix3<f32> {
        let (yaw, pitch, roll) = pose_solar_rad;

        let r11 = roll.cos() * yaw.cos() + roll.sin() * pitch.sin() * yaw.sin();
        let r12 = -roll.cos() * yaw.sin() + roll.sin() * pitch.sin() * yaw.cos();
        let r13 = -roll.sin() * pitch.cos();

        let r21 = pitch.cos() * yaw.sin();
        let r22 = pitch.cos() * yaw.cos();
        let r23 = pitch.sin();

        let r31 = roll.sin() * yaw.cos() - roll.cos() * pitch.sin() * yaw.sin();
        let r32 = -roll.sin() * yaw.sin() - roll.cos() * pitch.sin() * yaw.cos();
        let r33 = roll.cos() * pitch.cos();

        // nalgebra iterator fills column-wise.
        let pose_iter = vec![r11, r21, r31, r12, r22, r32, r13, r23, r33];
        Matrix3::from_iterator(pose_iter.into_iter())
    }

    /// Simulate a pixel using the Rayleigh sky model.
    pub fn simulate_pixel(&self, pixel: (u32, u32)) -> (f32, f32) {
        // Compute physical pixel location on image sensor.
        let pixel = Vector3::new(pixel.0 as f32, 0., pixel.1 as f32);
        let pixel = pixel - self.sensor_size_px * 0.5;
        let phys_loc = self.pixel_size_mm.component_mul(&pixel);

        // Trace a ray from the physical pixel location through the focal point.
        // This approach uses the pinhole camera model.
        let ray_body = phys_loc + self.focal_point_mm;

        // Transform ray from body (sensor) frame into the solar frame.
        let ray_solar = self.body_to_solar * ray_body;

        // Compute the zenith and azimuth angles of the ray.
        let zenith = (ray_solar.z / ray_solar.norm()).acos();
        let azimuth = ray_solar.y.atan2(ray_solar.x);

        // Apply Rayleigh sky model using zenith and azimuth of ray to compute AoP and DoP.
        let dop_max = 1.0;
        let dop = dop_max * zenith.sin().powf(2.) / (1. + zenith.cos().powf(2.));

        let mut aop_rad = 0.;

        // Ignore invalid solution at DoP == 0.
        if dop != 0. {
            let evector_solar = Vector3::new(azimuth.sin(), -azimuth.cos(), 0.);
            let evector_body = self.solar_to_body * evector_solar;

            aop_rad = (evector_body.x / evector_body.z).atan();
        }

        (aop_rad.to_degrees(), dop)
    }

    /// Simulates the specified pixels in parallel.
    /// Returns a row major vector of simulated measurements.
    pub fn par_simulate_pixels(&self, pixels: Vec<(u32, u32)>) -> Vec<(f32, f32)> {
        pixels
            .par_iter()
            .map(|pixel| self.simulate_pixel(*pixel))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // TODO: How do we test something like this?

    #[test]
    fn test_center_pixel() {
        let (_params, sensor) = make_sensor();
        let (_aop, _dop) = sensor.simulate_pixel((1224, 1024));
    }

    #[test]
    fn test_image() {
        let (params, sensor) = make_sensor();
        let pixels: Vec<(u32, u32)> = (0..params.sensor_size_px.1)
            .into_iter()
            .map(|row| {
                (0..params.sensor_size_px.0)
                    .into_iter()
                    .map(move |col| (col, row))
            })
            .flatten()
            .collect();

        let _simulated_image = sensor.par_simulate_pixels(pixels);
    }

    fn make_sensor() -> (SensorParams, Sensor) {
        let params = SensorParams {
            pixel_size_um: (3.45, 3.45),
            sensor_size_px: (2448, 2048),
            focal_length_mm: 3.5,
            solar_pose_deg: (78.9, -65.2, 278.3),
        };

        (params.clone(), Sensor::from(params))
    }
}

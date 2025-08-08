use super::{aop::Aop, dop::Dop, stokes::StokesVec};
use nalgebra::Vector2;

pub trait RayFrame: Copy + Clone {}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GlobalFrame;
impl RayFrame for GlobalFrame {}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorFrame;
impl RayFrame for SensorFrame {}

/// Describes the angle and degree of polarization for a single ray.
#[derive(Debug, PartialEq)]
pub struct Ray<Frame: RayFrame> {
    /// Location of pixel that measured the ray.
    loc: RayLocation,
    angle: Aop<Frame>,
    degree: Dop,
    _phan: std::marker::PhantomData<Frame>,
}

impl<Frame: RayFrame> Ray<Frame> {
    /// Creates a new `Ray` from a polarization `angle` and `degree`.
    pub fn new(loc: RayLocation, angle: Aop<Frame>, degree: Dop) -> Self {
        Self {
            loc,
            angle,
            degree,
            _phan: std::marker::PhantomData,
        }
    }

    pub fn from_stokes(loc: RayLocation, stokes: StokesVec<Frame>) -> Self {
        Self::new(loc, stokes.aop(), stokes.dop())
    }

    pub fn get_loc(&self) -> &RayLocation {
        &self.loc
    }

    pub fn get_aop(&self) -> &Aop<Frame> {
        &self.angle
    }

    pub fn get_dop(&self) -> &Dop {
        &self.degree
    }
}

impl Ray<GlobalFrame> {
    /// Transforms the Ray from the GlobalFrame into the SensorFrame.
    ///
    /// Requires a zenith location because the transform occurs with reference
    /// to the global up direction.
    pub fn into_sensor_frame(self, zenith_loc: &RayLocation) -> Ray<SensorFrame> {
        let loc_wrt_zen = self.loc.as_vec2() - zenith_loc.as_vec2();
        let offset = loc_wrt_zen.y.atan2(loc_wrt_zen.x).to_degrees();
        let angle = self.angle.into_inner() + offset;
        Ray::new(self.loc, Aop::from_deg_wrap(angle), self.degree)
    }
}

impl Ray<SensorFrame> {
    /// Transforms the Ray from the SensorFrame into the GlobalFrame.
    ///
    /// Requires a zenith location because the transform occurs with reference
    /// to the global up direction.
    pub fn into_global_frame(self, zenith_loc: &RayLocation) -> Ray<GlobalFrame> {
        let loc_wrt_zen = self.loc.as_vec2() - zenith_loc.as_vec2();
        let offset = loc_wrt_zen.y.atan2(loc_wrt_zen.x).to_degrees();
        let angle = self.angle.into_inner() - offset;
        Ray::new(self.loc, Aop::from_deg_wrap(angle), self.degree)
    }
}

/// Represents the location that a `Ray` was measured.
///
/// Location is with reference to the camera frame.
#[derive(Clone, Debug, PartialEq)]
pub struct RayLocation {
    inner: Vector2<f64>,
}

impl RayLocation {
    pub fn new(inner: Vector2<f64>) -> Self {
        Self { inner }
    }

    /// Computes a `RayLocation` from a pixel location and a `RaySensor`.
    pub fn at_pixel(loc: (u32, u32), sensor: &RaySensor) -> Self {
        let pixel = Vector2::new(loc.0 as f64, loc.1 as f64);
        let pixel = pixel - sensor.sensor_dim * 0.5;
        let phys_loc = sensor.pixel_size.component_mul(&pixel);

        Self { inner: phys_loc }
    }

    pub fn as_vec2(&self) -> &Vector2<f64> {
        &self.inner
    }
}

pub struct RaySensor {
    /// Size of a pixel on the simulated sensor in millimeters.
    pixel_size: Vector2<f64>,

    /// The dimensions of the simulated sensor in number of pixels.
    sensor_dim: Vector2<f64>,
}

impl RaySensor {
    pub fn new(pixel_size: Vector2<f64>, sensor_dim: Vector2<f64>) -> Self {
        Self {
            pixel_size,
            sensor_dim,
        }
    }
}

impl Default for RaySensor {
    fn default() -> Self {
        Self {
            // 0.001 is micron to mm
            pixel_size: Vector2::new(3.45 * 0.001, 3.45 * 0.001),
            sensor_dim: Vector2::new(2448.0, 2048.0),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::image::IntensityImage;
    use image::{GrayImage, ImageReader};
    use nalgebra::Vector2;

    #[test]
    fn regress_into_global_frame() {
        let sensor = RaySensor::new(
            Vector2::new(3.45 * 0.001 * 2.0, 3.45 * 0.001 * 2.0),
            Vector2::new(2448.0 / 2.0, 2048.0 / 2.0),
        );
        let zenith_loc = RayLocation::at_pixel((612, 512), &sensor);

        let image = read_image();
        let (width, height) = image.dimensions();
        let angles: Vec<f64> = IntensityImage::from_bytes(width, height, &image.into_raw())
            .unwrap()
            .rays(&sensor)
            .skip(1224 * 256)
            .take(3)
            .map(|ray| ray.into_global_frame(&zenith_loc))
            .map(|ray| ray.get_aop().clone().degrees())
            .collect();

        assert_eq!(
            angles,
            // Values taken from commit with hash ....
            vec![42.134330551943975, 42.235225489055665, 41.56346889997212]
        );
    }

    #[test]
    fn frame_reversible() {
        let sensor = RaySensor::new(
            Vector2::new(3.45 * 0.001 * 2.0, 3.45 * 0.001 * 2.0),
            Vector2::new(2448.0 / 2.0, 2048.0 / 2.0),
        );
        let zenith_loc = RayLocation::at_pixel((612, 512), &sensor);

        let image = read_image();
        let (width, height) = image.dimensions();
        let reference = vec![64.83392055917022, 64.96819157348496, 64.32990412704505];
        let angles: Vec<f64> = IntensityImage::from_bytes(width, height, &image.into_raw())
            .unwrap()
            .rays(&sensor)
            .skip(1224 * 256)
            .take(3)
            .map(|ray| ray.into_global_frame(&zenith_loc))
            .map(|ray| ray.into_sensor_frame(&zenith_loc))
            .map(|ray| ray.get_aop().clone().degrees())
            .collect();

        // Problematic due to floating point error accumulation.
        for (angle, refr) in angles.into_iter().zip(reference.into_iter()) {
            assert!((angle - refr).abs() < 0.01);
        }
    }

    fn read_image() -> GrayImage {
        ImageReader::open("testing/intensity.png")
            .unwrap()
            .decode()
            .unwrap()
            .into_luma8()
    }
}

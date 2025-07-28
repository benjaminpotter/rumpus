use crate::{
    estimator::Estimator,
    filter::AopFilter,
    ray::{Aop, Ray, RayIterator},
};
use std::ops::RangeInclusive;

/// Estimates azimuth using a 1D Hough transform.
pub struct HoughTransform {
    /// The location of the zenith in the sensor plane represented as
    /// co-ordinates in image space.
    zenith: (f64, f64),

    /// The azimuth angle of the solar meridian.
    ///
    /// Taken CW from north.
    saz: f64,

    /// The AoP threshold used to isolate the solar meridian.
    thres: f64,

    /// The minimum resolution for esimates in degrees.
    res: f64,
}

impl HoughTransform {
    pub fn new(saz: f64, zenith: (f64, f64), res: f64, thres: f64) -> Self {
        Self {
            saz,
            zenith,
            thres,
            res,
        }
    }

    fn ray_angle(&self, ray: Ray) -> f64 {
        let (col, row) = *ray.get_loc();
        let (x, y) = (col as f64 - self.zenith.0, row as f64 - self.zenith.1);
        (y / x).atan().to_degrees()
    }
}

impl Estimator for HoughTransform {
    type Output = f64;

    fn estimate<I: RayIterator>(&self, rays: I) -> Self::Output {
        let mut acc = Accumulator::new(self.res, -90.0..=90.0);
        for ray in rays.ray_filter(AopFilter::new(Aop::from_deg(90.0), self.thres)) {
            let angle = self.ray_angle(ray);

            // TODO: make collect from iterator of votes.
            acc.vote(angle);
        }

        // TODO: disambiguate from (anti-) solar meridian
        // TODO: use self.saz to make this wrt ENU frame.
        acc.winner()
    }
}

struct Accumulator {
    resolution: f64,
    range: RangeInclusive<f64>,
    buffer: Vec<u32>,
}

impl Accumulator {
    fn new(resolution: f64, range: RangeInclusive<f64>) -> Self {
        let len = ((range.end() - range.start()) / resolution) as usize;
        let buffer = vec![0; len];

        Self {
            resolution,
            range,
            buffer,
        }
    }

    fn value_to_index(&self, value: f64) -> usize {
        ((value - self.range.start()) / self.resolution).floor() as usize
    }

    fn index_to_value(&self, index: usize) -> f64 {
        index as f64 * self.resolution + self.range.start()
    }

    fn vote(&mut self, value: f64) {
        let index = self.value_to_index(value);
        self.buffer[index] += 1;
    }

    fn winner(&self) -> f64 {
        let (index, _) = self
            .buffer
            .iter()
            .enumerate()
            .max_by_key(|&(_, count)| count)
            .unwrap();

        self.index_to_value(index)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::image::IntensityImage;
    use image::{GrayImage, ImageReader};

    #[test]
    fn hough_transform() {
        let image = read_image();
        let (width, height) = image.dimensions();
        let est = IntensityImage::from_bytes(width, height, &image.into_raw())
            .unwrap()
            .rays()
            .estimate(HoughTransform::new(0.0, (612.0, 512.0), 0.1, 0.2));

        assert_eq!(est, -40.5);
    }

    fn read_image() -> GrayImage {
        ImageReader::open("testing/intensity.png")
            .unwrap()
            .decode()
            .unwrap()
            .into_luma8()
    }
}

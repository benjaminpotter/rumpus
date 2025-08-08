use super::Estimator;
use crate::{
    iter::RayIterator,
    light::{
        aop::Aop,
        filter::{AopFilter, RayPredicate},
        ray::{GlobalFrame, Ray, RayLocation, SensorFrame},
    },
};
use std::ops::RangeInclusive;

/// Estimates azimuth using a 1D Hough transform.
pub struct HoughTransform {
    /// The location of the zenith in the sensor plane.
    zenith: RayLocation,

    /// The azimuth angle of the solar meridian.
    ///
    /// Taken CW from north.
    saz: f64,

    /// The minimum resolution for esimates in degrees.
    res: f64,

    /// The AoP threshold used to isolate the solar meridian.
    filter: AopFilter<GlobalFrame>,
}

impl HoughTransform {
    pub fn new(saz: f64, zenith: RayLocation, res: f64, thres: f64) -> Self {
        let filter = AopFilter::new(Aop::from_deg(90.0), thres);
        Self {
            saz,
            zenith,
            res,
            filter,
        }
    }

    fn ray_angle(&self, ray: Ray<GlobalFrame>) -> f64 {
        let loc = *ray.get_loc().as_vec2();
        (loc.y / loc.x).atan().to_degrees()
    }
}

impl Estimator<SensorFrame> for &HoughTransform {
    type Output = f64;

    fn estimate<I: RayIterator<SensorFrame>>(self, rays: I) -> Self::Output {
        let mut acc = Accumulator::new(self.res, -90.0..=90.0);
        for ray in rays
            .map(|ray| ray.into_global_frame(&self.zenith))
            .filter(|ray| self.filter.eval(&ray))
        {
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

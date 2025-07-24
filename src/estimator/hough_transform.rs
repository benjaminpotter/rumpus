use crate::{
    estimator::Estimator,
    filter::AopFilter,
    image::Rays,
    ray::{Aop, Ray},
    state::{Position, Positioned, Timed},
};
use chrono::prelude::*;
use std::ops::RangeInclusive;

/// Estimates azimuth using a 1D Hough transform.
pub struct HoughTransform<S> {
    state: S,

    /// The location of the zenith in the sensor plane represented as
    /// co-ordinates in image space.
    zenith: (f64, f64),
    thres: f64,
    acc: Accumulator,
}

impl<S> HoughTransform<S> {
    pub fn new(state: S, res: f64, zenith: (f64, f64), thres: f64) -> Self {
        let acc = Accumulator::new(res, -90.0..=90.0);
        Self {
            state,
            zenith,
            thres,
            acc,
        }
    }

    fn ray_angle(&self, ray: Ray) -> f64 {
        let (col, row) = *ray.get_loc();
        let (x, y) = (col as f64 - self.zenith.0, row as f64 - self.zenith.1);
        (y / x).atan().to_degrees()
    }
}

impl<S> Estimator for HoughTransform<S>
where
    S: Positioned + Timed,
{
    type Output = f64;

    fn estimate(&mut self, rays: Rays) -> Self::Output {
        for ray in rays.ray_filter(AopFilter::new(Aop::from_deg(90.0), self.thres)) {
            let angle = self.ray_angle(ray);

            // TODO: make collect from iterator of votes.
            self.acc.vote(angle);
        }

        // TODO: use state to make this wrt ENU frame.
        self.acc.winner()
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

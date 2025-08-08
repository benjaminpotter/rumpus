use super::Estimator;
use crate::{
    camera::{Camera, Lens, RayleighModel, SkyPoint},
    error::Error,
    iter::RayIterator,
    light::ray::{Ray, RayLocation, SensorFrame},
    state::Orientation,
};
use rand::{
    distr::{Distribution, Uniform},
    Rng,
};
use rayon::prelude::*;

/// Estimates ort using simulated images and a loss function.
pub struct PatternMatch<S> {
    lens: Lens,
    model: RayleighModel,
    searcher: S,
    max_iters: usize,
}

impl<S> PatternMatch<S> {
    pub fn new(lens: Lens, model: RayleighModel, searcher: S, max_iters: usize) -> Self {
        Self {
            lens,
            model,
            searcher,
            max_iters,
        }
    }
}

impl<S: Searcher> Estimator<SensorFrame> for PatternMatch<S> {
    type Output = Orientation;

    fn estimate<I: RayIterator<SensorFrame>>(self, rays: I) -> Self::Output {
        let rays: Vec<Ray<SensorFrame>> = rays.collect();
        self.searcher
            .search_iter()
            .take(self.max_iters)
            .map(|ort| {
                let camera = Camera::new(self.lens.clone(), ort.clone());
                let zen_loc = RayLocation::new(camera.trace_from_sky(&SkyPoint::new(0.0, 0.0)));
                let loss = rays
                    .par_iter()
                    .map(|ray| {
                        (*ray.get_aop()
                            - *camera
                                .simulate_ray(ray.get_loc().clone(), &self.model)
                                .into_sensor_frame(&zen_loc)
                                .get_aop())
                        .into_inner()
                        .powf(2.)
                            / (*ray.get_dop()).into_inner()
                    })
                    .sum::<f64>()
                    / rays.len() as f64;

                Estimate { ort, loss }
            })
            .reduce(Estimate::min)
            .unwrap()
            .ort
    }
}

struct Estimate {
    ort: Orientation,
    loss: f64,
}

impl Estimate {
    fn min(self, other: Self) -> Self {
        match self.loss < other.loss {
            true => self,
            false => other,
        }
    }
}

/// Can be converted to an iterator over orts.
///
/// Used by `PatternMatcher` to generate simulated images.
pub trait Searcher {
    type Iter: Iterator<Item = Orientation>;
    fn search_iter(self) -> Self::Iter;
}

/// A searcher implementation that provides random orts within a bounded range.
pub struct StochasticSearch<R> {
    sampler: Uniform<Orientation>,
    rng: R,
}

impl<R> StochasticSearch<R> {
    /// Creates a new `StochasticSearch` bounded by `low` and `high`.
    pub fn try_new(low: Orientation, high: Orientation, rng: R) -> Result<Self, Error> {
        let sampler = Uniform::new(low, high).map_err(|err| match err {
            rand::distr::uniform::Error::EmptyRange => Error::EmptyRange,
            rand::distr::uniform::Error::NonFinite => Error::NonFinite,
        })?;
        Ok(Self { sampler, rng })
    }
}

impl<R: Rng + Clone> Searcher for StochasticSearch<R> {
    type Iter = rand::distr::Iter<Uniform<Orientation>, R, Orientation>;

    fn search_iter(self) -> Self::Iter {
        self.sampler.sample_iter(self.rng)
    }
}

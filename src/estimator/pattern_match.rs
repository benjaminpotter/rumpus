use crate::{
    camera::{Camera, CameraParams},
    error::Error,
    estimator::Estimator,
    ray::{Ray, RayIterator},
    state::{Pose, Position, State},
};
use chrono::prelude::*;
use rand::{
    distr::{Distribution, Uniform},
    Rng,
};
use rayon::prelude::*;

/// Estimates pose using simulated images and a loss function.
pub struct PatternMatch<S> {
    cam_params: CameraParams,
    position: Position,
    time: DateTime<Utc>,
    searcher: S,
}

impl<S> PatternMatch<S> {
    pub fn new(
        cam_params: CameraParams,
        position: Position,
        time: DateTime<Utc>,
        searcher: S,
    ) -> Self {
        Self {
            cam_params,
            position,
            time,
            searcher,
        }
    }
}

impl<S: Searcher> Estimator for PatternMatch<S> {
    type Output = Pose;

    fn estimate<I: RayIterator>(self, rays: I) -> Self::Output {
        const MAX_ITERS: usize = 10;
        let rays: Vec<Ray> = rays.collect();
        self.searcher
            .search_iter()
            .take(MAX_ITERS)
            .map(|pose| {
                let camera = Camera::new(
                    &self.cam_params,
                    State::new(pose, self.position.clone(), self.time.clone()),
                );

                let loss = rays
                    .par_iter()
                    .map(|ray| {
                        (*ray.get_aop() - *camera.simulate_pixel(ray.get_loc()).get_aop())
                            .into_inner()
                            .powf(2.)
                            / (*ray.get_dop()).into_inner()
                    })
                    .sum::<f64>()
                    / rays.len() as f64;

                Estimate { pose, loss }
            })
            .reduce(Estimate::min)
            .unwrap()
            .pose
    }
}

struct Estimate {
    pose: Pose,
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

/// Can be converted to an iterator over poses.
///
/// Used by `PatternMatcher` to generate simulated images.
pub trait Searcher {
    type Iter: Iterator<Item = Pose>;
    fn search_iter(self) -> Self::Iter;
}

/// A searcher implementation that provides random poses within a bounded range.
pub struct StochasticSearch<R> {
    sampler: Uniform<Pose>,
    rng: R,
}

impl<R> StochasticSearch<R> {
    /// Creates a new `StochasticSearch` bounded by `low` and `high`.
    pub fn try_new(low: Pose, high: Pose, rng: R) -> Result<Self, Error> {
        let sampler = Uniform::new(low, high).map_err(|err| match err {
            rand::distr::uniform::Error::EmptyRange => Error::EmptyRange,
            rand::distr::uniform::Error::NonFinite => Error::NonFinite,
        })?;
        Ok(Self { sampler, rng })
    }
}

impl<R: Rng + Clone> Searcher for StochasticSearch<R> {
    type Iter = rand::distr::Iter<Uniform<Pose>, R, Pose>;

    fn search_iter(self) -> Self::Iter {
        self.sampler.sample_iter(self.rng)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::image::IntensityImage;
    use image::{GrayImage, ImageReader};
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    #[test]
    fn regress_pattern_match() {
        let image = read_image();
        let (width, height) = image.dimensions();
        let searcher = VecSearch::new(vec![Pose::zeros(), Pose::new(0.0, 0.0, 180.0)]);
        let pose = IntensityImage::from_bytes(width, height, &image.into_raw())
            .unwrap()
            .rays()
            .estimate(PatternMatch::new(
                CameraParams::default(),
                Position::kingston(),
                "2025-06-13T16:26:47+00:00"
                    .parse::<DateTime<Utc>>()
                    .unwrap(),
                searcher,
            ));

        assert_eq!(pose, Pose::new(0.0, 0.0, 180.0));
    }

    #[test]
    fn stochastic_search() {
        const SEED: u64 = 0;
        let searcher = StochasticSearch::try_new(
            Pose::new(-1.0, -1.0, 0.0),
            Pose::new(1.0, 1.0, 360.0),
            ChaCha8Rng::seed_from_u64(SEED),
        )
        .unwrap();

        assert_eq!(
            // Might fail due to reproducibility of the rng.
            // https://rust-random.github.io/book/crate-reprod.html
            searcher.search_iter().next().unwrap(),
            Pose::new(0.41815083085312343, -0.0681565554207797, 251.6915673629034)
        );
    }

    struct VecSearch {
        buffer: Vec<Pose>,
    }

    impl VecSearch {
        fn new(buffer: Vec<Pose>) -> Self {
            Self { buffer }
        }
    }

    impl Searcher for VecSearch {
        type Iter = std::vec::IntoIter<Pose>;

        fn search_iter(self) -> Self::Iter {
            self.buffer.into_iter()
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

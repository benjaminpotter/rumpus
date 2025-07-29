use crate::{
    camera::{Camera, CameraParams},
    estimator::Estimator,
    ray::{Ray, RayIterator},
    state::{Pose, Position, State},
};
use chrono::prelude::*;
use rand::{
    distr::{Distribution, StandardUniform},
    rngs::ThreadRng,
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

    fn create_camera(&self, pose: Pose) -> Camera {
        Camera::new(
            self.cam_params.clone(),
            State::new(pose, self.position.clone(), self.time.clone()),
        )
    }
}

impl<S: Searcher> Estimator for PatternMatch<S> {
    type Output = Pose;

    fn estimate<I: RayIterator>(self, rays: I) -> Self::Output {
        let rays: Vec<Ray> = rays.collect();
        self.searcher
            .search_iter()
            .take(10)
            .map(|pose| {
                let camera = self.create_camera(pose.clone());
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

pub trait Searcher {
    type Iter: Iterator<Item = Pose>;
    fn search_iter(&self) -> Self::Iter;
}

pub struct StochasticSearch {}

impl StochasticSearch {
    pub fn new() -> Self {
        Self {}
    }
}

impl Searcher for StochasticSearch {
    type Iter = rand::distr::Iter<StandardUniform, ThreadRng, Pose>;

    fn search_iter(&self) -> Self::Iter {
        StandardUniform.sample_iter(rand::rng())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::image::IntensityImage;
    use image::{GrayImage, ImageReader};

    #[test]
    fn pattern_match() {
        let image = read_image();
        let (width, height) = image.dimensions();
        let pose = IntensityImage::from_bytes(width, height, &image.into_raw())
            .unwrap()
            .rays()
            .estimate(PatternMatch::new(
                CameraParams::default(),
                Position::kingston(),
                Utc::now(),
                StochasticSearch::new(),
            ));

        assert_eq!(pose, Pose::up());
    }

    fn read_image() -> GrayImage {
        ImageReader::open("testing/intensity.png")
            .unwrap()
            .decode()
            .unwrap()
            .into_luma8()
    }
}

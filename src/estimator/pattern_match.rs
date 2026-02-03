use crate::{
    CameraEnu,
    camera::{Camera, Lens},
    estimator::Estimator,
    iter::RayIterator,
    light::dop::Dop,
    model::SkyModel,
    ray::{Ray, SensorFrame},
};
use rayon::prelude::*;
use sguaba::{Bearing, engineering::Orientation};
use uom::{
    ConstZero,
    si::{angle::radian, f64::Angle},
};

/// Estimates ort using simulated images and a loss function.
pub struct PatternMatch<S> {
    lens: Lens,
    model: SkyModel,
    searcher: S,
    max_iters: usize,
}

impl<S> PatternMatch<S> {
    pub fn new(lens: Lens, model: SkyModel, searcher: S, max_iters: usize) -> Self {
        Self {
            lens,
            model,
            searcher,
            max_iters,
        }
    }
}

impl<S: Searcher> Estimator<SensorFrame> for PatternMatch<S> {
    type Output = Orientation<CameraEnu>;

    fn estimate<I: RayIterator<SensorFrame>>(self, rays: I) -> Self::Output {
        let rays: Vec<Ray<SensorFrame>> = rays.collect();

        self.searcher
            .orientations()
            .take(self.max_iters)
            .map(|ort| {
                // Construct a camera at the new orientation.
                let cam = Camera::new(self.lens.clone(), ort.clone());

                // Find the zenith coordinate in CameraFrd.
                let zenith_coord = cam
                    .trace_from_sky(
                        Bearing::<CameraEnu>::builder()
                            .azimuth(Angle::ZERO)
                            .elevation(Angle::HALF_TURN / 2.)
                            .expect("elevation is on range -90 to 90")
                            .build(),
                    )
                    .expect("zenith is always above the horizon");

                let loss = rays
                    .par_iter()
                    .filter_map(|ray| {
                        // Model a ray with the same CameraFrd coordinate as the
                        // measured ray.
                        let ray_bearing = cam
                            .trace_from_sensor(*ray.coord())
                            .expect("ray coordinate should always have Z of zero");
                        // Ignore rays from below the horizon.
                        let modelled_aop = self.model.aop(ray_bearing)?;
                        let modelled_ray_global = Ray::new(*ray.coord(), modelled_aop, Dop::zero());

                        // Transform the modelled ray from the global frame into
                        // the sensor frame.
                        let modelled_ray_sensor = modelled_ray_global
                            .into_sensor_frame(zenith_coord.clone())
                            // Camera trace_from_sky always returns a coordinate
                            // with a zenith of zero which enforces this expect.
                            .expect("zenith coord is has Z of zero");

                        // Compute the weighted, squared difference between the
                        // modelled ray and the measured ray.
                        let delta = *ray.aop() - *modelled_ray_sensor.aop();
                        let sq_diff = Into::<Angle>::into(delta).get::<radian>().powf(2.);
                        let weight = 1. / *ray.dop();
                        let weighted_sq_diff = weight * sq_diff;

                        Some(weighted_sq_diff)
                    })
                    // Take the mean of the weighted, squared differences.
                    .sum::<f64>()
                    / rays.len() as f64;

                Estimate { ort, loss }
            })
            // Find the estimate with the smallest weighted, squared difference.
            .reduce(Estimate::min)
            .expect("at least one orientation from searcher")
            // Return the orientation of the camera used to generate the model.
            .ort
    }
}

struct Estimate {
    ort: Orientation<CameraEnu>,
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

/// Can be converted to an iterator over orientations.
///
/// Used by `PatternMatcher` to generate simulated images.
pub trait Searcher {
    type Iter: Iterator<Item = Orientation<CameraEnu>>;

    fn orientations(self) -> Self::Iter;
}

pub struct VecSearch {
    orts: Vec<Orientation<CameraEnu>>,
}

impl VecSearch {
    pub fn new(orts: Vec<Orientation<CameraEnu>>) -> Self {
        Self { orts }
    }
}

impl Searcher for VecSearch {
    type Iter = std::vec::IntoIter<Orientation<CameraEnu>>;

    fn orientations(self) -> Self::Iter {
        self.orts.into_iter()
    }
}

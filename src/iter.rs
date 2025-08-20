use super::{
    estimator::Estimator,
    filter::{RayFilter, RayPredicate},
    ray::{Ray, RayFrame},
};

/// A `Iterator` wrapper for `Ray`.
/// This trait exposes additional functions on an `Iterator` over `Ray`.
pub trait RayIterator<Frame: RayFrame>: Iterator<Item = Ray<Frame>> {
    fn ray_filter<P: RayPredicate<Frame>>(self, pred: P) -> RayFilter<Self, P>
    where
        Self: Sized,
    {
        RayFilter::new(self, pred)
    }

    fn estimate<E, O>(self, estimator: E) -> O
    where
        Self: Sized,
        E: Estimator<Frame, Output = O>,
    {
        estimator.estimate(self)
    }
}

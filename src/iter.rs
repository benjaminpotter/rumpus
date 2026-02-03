use super::{
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
}

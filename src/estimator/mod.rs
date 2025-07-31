mod hough_transform;
pub mod pattern_match;

use crate::ray::RayIterator;
pub use hough_transform::HoughTransform;

pub trait Estimator {
    type Output;
    fn estimate<I: RayIterator>(self, rays: I) -> Self::Output;
}

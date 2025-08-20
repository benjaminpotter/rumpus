use crate::{iter::RayIterator, ray::RayFrame};

// pub mod hough_transform;
pub mod pattern_match;

pub trait Estimator<Frame: RayFrame> {
    type Output;
    fn estimate<I: RayIterator<Frame>>(self, rays: I) -> Self::Output;
}

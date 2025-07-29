//! ```ignore
//! struct State;
//! impl Positioned for State;
//! impl Timed for State;
//!
//! let mut state = State;
//! while let Some(image) = next_image() {
//!     let (w, h) = image.dimensions()
//!     let ht = HoughTransform::new(state, (w/2, h/2));
//!     let estimate = IntensityImage::from_bytes(w, h, &image.into_raw())
//!         .rays()
//!         .estimator(ht);
//!     
//!     state = next_state(estimate);
//! }
//! ```

mod hough_transform;
pub mod pattern_match;

use crate::ray::RayIterator;
pub use hough_transform::HoughTransform;

pub trait Estimator {
    type Output;
    fn estimate<I: RayIterator>(self, rays: I) -> Self::Output;
}

// struct Estimates<Et, I, Er>
// - iter: I
// - estimator: Er
// - _phantom: Et
//
// - impl<Et, I, Er> Iterator for Estimates<Et, I, Er> where I: Iterator<Item = Et>, Er: Estimator<Estimate = Et>
// - type Item = I::Item
// - fn next(&mut self) -> Option<Self::Item> calls self.estimator.estimate(self.iter.next()?)

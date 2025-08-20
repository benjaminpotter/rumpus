// #![warn(missing_docs)]

//! Skylight Polarization Utilities

pub mod camera;
pub mod error;
pub mod estimator;
pub mod filter;
pub mod image;
pub mod iter;
pub mod light;
pub mod model;
pub mod ray;

pub mod prelude {
    pub use crate::camera::{Camera, Lens};
    pub use crate::error::Error;
    pub use crate::estimator::pattern_match::{PatternMatch, VecSearch};
    pub use crate::filter::{AopFilter, DopFilter, RayFilter};
    pub use crate::image::{ImageSensor, IntensityImage, RayImage};
    pub use crate::iter::RayIterator;
    pub use crate::light::{aop::Aop, dop::Dop};
    pub use crate::model::SkyModel;
    pub use crate::ray::{GlobalFrame, Ray, SensorFrame};
    pub use crate::{CameraEnu, CameraFrd};
}

use sguaba::system;

// A coordinate system with its origin at the optical center of a camera and
// its X, Y, Z axes along East, North, and Up, respectively.
system!(pub struct CameraEnu using ENU);

// A coordinate system with its origin at the optical center of a camera and
// its X, Y, Z axes pointing forward in the direction of travel, right of the
// optical center, and down into lens through the sensor plane, respectively.
system!(pub struct CameraFrd using FRD);

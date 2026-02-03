// #![warn(missing_docs)]

//! Skylight Polarization Utilities

pub mod error;
pub mod filter;
pub mod image;
pub mod iter;
pub mod light;
pub mod model;
pub mod optic;
pub mod ray;
pub mod simulation;

pub mod prelude {
    pub use crate::error::Error;
    pub use crate::filter::{AopFilter, DopFilter, RayFilter};
    pub use crate::image::{IntensityImage, RayImage};
    pub use crate::iter::RayIterator;
    pub use crate::light::{aop::Aop, dop::Dop};
    pub use crate::model::SkyModel;
    pub use crate::ray::{GlobalFrame, Ray, SensorFrame};
}

use thiserror::Error;
use uom::si::f64::Angle;

pub mod aop;
pub mod dop;
pub mod stokes;

#[derive(Debug, Error)]
pub enum LightError {
    #[error("expected angle in range [-PI, PI] but got: {angle:#?}")]
    AngleOutOfBounds { angle: Angle },
    #[error("expected degree in range [0, 1] but got: {degree}")]
    DegreeOutOfBounds { degree: f64 },
}

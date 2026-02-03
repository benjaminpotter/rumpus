use crate::light::{LightError, aop::Aop, dop::Dop, stokes::StokesVec};
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use thiserror::Error;
use uom::si::f64::Angle;

#[derive(Debug, Error)]
pub enum RayError {
    #[error("failed to parse stokes vector")]
    InvalidStokes(#[from] LightError),
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct GlobalFrame;

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SensorFrame;

/// Describes the angle and degree of polarization for a single ray.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Ray<Frame> {
    /// Angle of polarization of the `Ray`.
    ///
    /// This refers to the e-vector angle with respect to `Frame`.
    angle: Aop<Frame>,

    /// Degree of polarization of the `Ray`.
    degree: Dop,

    _phan: std::marker::PhantomData<Frame>,
}

impl<Frame> Ray<Frame> {
    /// Creates a new `Ray` from a polarization `angle` and `degree`.
    #[must_use]
    pub fn new(angle: Aop<Frame>, degree: Dop) -> Self {
        Self {
            angle,
            degree,
            _phan: std::marker::PhantomData,
        }
    }

    #[must_use]
    pub fn aop(&self) -> Aop<Frame>
    where
        Frame: Copy,
    {
        self.angle
    }

    #[must_use]
    pub fn dop(&self) -> Dop {
        self.degree
    }
}

impl Ray<GlobalFrame> {
    /// Transforms the Ray from the `GlobalFrame` into the `SensorFrame`.
    #[must_use]
    pub fn into_sensor_frame(self, shift: Angle) -> Ray<SensorFrame> {
        Ray::new(self.angle.into_sensor_frame(shift), self.degree)
    }
}

impl Ray<SensorFrame> {
    /// Transforms the Ray from the `SensorFrame` into the `GlobalFrame`.
    #[must_use]
    pub fn into_global_frame(self, shift: Angle) -> Ray<GlobalFrame> {
        Ray::new(self.angle.into_global_frame(shift), self.degree)
    }
}

impl<Frame> TryFrom<StokesVec<Frame>> for Ray<Frame> {
    type Error = RayError;

    fn try_from(stokes: StokesVec<Frame>) -> Result<Self, Self::Error> {
        Ok(Self::new(stokes.aop()?, stokes.dop()?))
    }
}

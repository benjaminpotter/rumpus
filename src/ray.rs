use crate::light::{aop::Aop, dop::Dop, stokes::StokesVec};
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use uom::si::f64::Angle;

pub trait RayFrame: Copy + Clone + std::fmt::Debug + PartialEq {}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct GlobalFrame;
impl RayFrame for GlobalFrame {}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SensorFrame;
impl RayFrame for SensorFrame {}

/// Describes the angle and degree of polarization for a single ray.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Ray<Frame: RayFrame> {
    /// Angle of polarization of the `Ray`.
    ///
    /// This refers to the e-vector angle with respect to `Frame`.
    angle: Aop<Frame>,

    /// Degree of polarization of the `Ray`.
    degree: Dop,

    _phan: std::marker::PhantomData<Frame>,
}

impl<Frame: RayFrame> Ray<Frame> {
    /// Creates a new `Ray` from a polarization `angle` and `degree`.
    pub fn new(angle: Aop<Frame>, degree: Dop) -> Self {
        Self {
            angle,
            degree,
            _phan: std::marker::PhantomData,
        }
    }

    pub fn from_stokes(stokes: StokesVec<Frame>) -> Self {
        Self::new(stokes.aop(), stokes.dop())
    }

    pub fn aop(&self) -> &Aop<Frame> {
        &self.angle
    }

    pub fn dop(&self) -> &Dop {
        &self.degree
    }
}

impl Ray<GlobalFrame> {
    /// Transforms the Ray from the GlobalFrame into the SensorFrame.
    pub fn into_sensor_frame(self, shift: Angle) -> Ray<SensorFrame> {
        Ray::new(self.angle.into_sensor_frame(shift), self.degree)
    }
}

impl Ray<SensorFrame> {
    /// Transforms the Ray from the SensorFrame into the GlobalFrame.
    pub fn into_global_frame(self, shift: Angle) -> Ray<GlobalFrame> {
        Ray::new(self.angle.into_global_frame(shift), self.degree)
    }
}

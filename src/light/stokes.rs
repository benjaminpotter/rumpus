use crate::light::{LightError, aop::Aop, dop::Dop};
use uom::si::{angle::radian, f64::Angle};

/// Describes the linear polarization of a ray.
#[derive(Debug, PartialEq)]
pub struct StokesVec<Frame> {
    inner: [f64; 3],
    _phan: std::marker::PhantomData<Frame>,
}

impl<Frame> StokesVec<Frame> {
    #[must_use]
    pub fn new(s0: f64, s1: f64, s2: f64) -> Self {
        StokesVec {
            inner: [s0, s1, s2],
            _phan: std::marker::PhantomData,
        }
    }

    /// Compute the `AoP` of the ray.
    ///
    /// # Errors
    /// Will return an `Err` if the Stokes vector encodes an [`Aop`] outside of [-90, 90].
    pub fn aop(&self) -> Result<Aop<Frame>, LightError> {
        let angle = Angle::new::<radian>(self.inner[2].atan2(self.inner[1]) / 2.);
        Aop::try_from_angle(angle)
    }

    /// Compute the `DoP` of the ray.
    ///
    /// # Errors
    /// Will return `Err` if the Stokes vector encodes a [`Dop`] outside of [0, 1].
    pub fn dop(&self) -> Result<Dop, LightError> {
        Dop::try_new((self.inner[1].powf(2.) + self.inner[2].powf(2.)).sqrt() / self.inner[0])
    }
}

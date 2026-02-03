use crate::light::{aop::Aop, dop::Dop};
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
    #[must_use] 
    pub fn aop(&self) -> Aop<Frame> {
        let angle = Angle::new::<radian>(self.inner[2].atan2(self.inner[1]) / 2.);
        Aop::from_angle(angle).expect("aop from stokes equations should return a valid angle")
    }

    /// Compute the `DoP` of the ray.
    #[must_use] 
    pub fn dop(&self) -> Dop {
        Dop::new((self.inner[1].powf(2.) + self.inner[2].powf(2.)).sqrt() / self.inner[0])
            .expect("dop within [0, 1]")
    }
}

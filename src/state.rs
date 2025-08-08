use nalgebra::Rotation3;
use rand::{
    distr::uniform::{Error, SampleBorrow, SampleUniform, UniformFloat, UniformSampler},
    Rng,
};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Orientation {
    inner: Rotation3<f64>,
}

impl Orientation {
    /// Create a new `Orientation` from `roll`, `pitch`, and `yaw` in degrees.
    pub fn new(inner: Rotation3<f64>) -> Self {
        Self { inner }
    }

    pub fn as_rot(&self) -> &Rotation3<f64> {
        &self.inner
    }

    /// Returns the `Orientation` as a roll, pitch, yaw tuple.
    pub fn euler_angles(&self) -> (f64, f64, f64) {
        self.inner.euler_angles()
    }
}

/// Sample random orientations on a range.
///
/// Implemented following the (https://docs.rs/rand/latest/rand/distr/uniform/index.html)[documentation] for `rand`.
///
/// ```
/// use rumpus::state::Orientation;
/// use nalgebra::Rotation3;
/// use rand::distr::{Distribution, Uniform};
///
/// let low = Orientation::new(Rotation3::from_euler_angles(-1.0, -1.0, 0.0));
/// let high =  Orientation::new(Rotation3::from_euler_angles(1.0, 1.0, 360.0));
/// let uniform = Uniform::new(low, high).unwrap();
/// let orientation= uniform.sample(&mut rand::rng());
/// ```
#[derive(Clone)]
pub struct UniformOrientation {
    roll: UniformFloat<f64>,
    pitch: UniformFloat<f64>,
    yaw: UniformFloat<f64>,
}

impl UniformSampler for UniformOrientation {
    type X = Orientation;

    fn new<B1, B2>(low: B1, high: B2) -> Result<Self, Error>
    where
        B1: SampleBorrow<Self::X> + Sized,
        B2: SampleBorrow<Self::X> + Sized,
    {
        let (lr, lp, ly) = low.borrow().as_rot().euler_angles();
        let (hr, hp, hy) = high.borrow().as_rot().euler_angles();

        Ok(UniformOrientation {
            roll: UniformFloat::<f64>::new(lr, hr)?,
            pitch: UniformFloat::<f64>::new(lp, hp)?,
            yaw: UniformFloat::<f64>::new(ly, hy)?,
        })
    }

    fn new_inclusive<B1, B2>(low: B1, high: B2) -> Result<Self, Error>
    where
        B1: SampleBorrow<Self::X> + Sized,
        B2: SampleBorrow<Self::X> + Sized,
    {
        let (lr, lp, ly) = low.borrow().as_rot().euler_angles();
        let (hr, hp, hy) = high.borrow().as_rot().euler_angles();

        Ok(UniformOrientation {
            roll: UniformFloat::<f64>::new_inclusive(lr, hr)?,
            pitch: UniformFloat::<f64>::new_inclusive(lp, hp)?,
            yaw: UniformFloat::<f64>::new_inclusive(ly, hy)?,
        })
    }

    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> Self::X {
        Orientation::new(Rotation3::from_euler_angles(
            self.roll.sample(rng).to_radians(),
            self.pitch.sample(rng).to_radians(),
            self.yaw.sample(rng).to_radians(),
        ))
    }
}

impl SampleUniform for Orientation {
    type Sampler = UniformOrientation;
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Position {
    pub lat: f64,
    pub lon: f64,
}

impl Position {
    pub fn kingston() -> Self {
        Self {
            lat: 44.2187,
            lon: -76.4747,
        }
    }
}

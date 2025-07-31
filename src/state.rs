use chrono::prelude::*;
use nalgebra::Rotation3;
use rand::{
    distr::uniform::{Error, SampleBorrow, SampleUniform, UniformFloat, UniformSampler},
    Rng,
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Serialize, Deserialize, Debug)]
pub struct State {
    pose: Pose,
    position: Position,
    time: DateTime<Utc>,
}

impl State {
    pub fn new(pose: Pose, position: Position, time: DateTime<Utc>) -> Self {
        Self {
            pose,
            position,
            time,
        }
    }

    pub fn into_inner(self) -> (Pose, Position, DateTime<Utc>) {
        (self.pose, self.position, self.time)
    }
}

#[derive(Clone, Copy, Serialize, Deserialize, Debug, PartialEq)]
pub struct Pose {
    roll: f64,
    pitch: f64,
    yaw: f64,
}

impl Pose {
    /// Create a new `Pose` from `roll`, `pitch`, and `yaw`.
    pub fn new(roll: f64, pitch: f64, yaw: f64) -> Self {
        Self { roll, pitch, yaw }
    }

    pub fn zeros() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        }
    }

    // TODO: Properly handle radians...
    pub fn to_radians(&self) -> Self {
        Self {
            roll: self.roll.to_radians(),
            pitch: self.pitch.to_radians(),
            yaw: self.yaw.to_radians(),
        }
    }
}

/// Sample random poses on a range.
///
/// Implemented following the (https://docs.rs/rand/latest/rand/distr/uniform/index.html)[documentation] for `rand`.
///
/// ```
/// use rumpus::state::Pose;
/// use rand::distr::{Distribution, Uniform};
///
/// let (low, high) = (Pose::new(-1.0, -1.0, 0.0), Pose::new(1.0, 1.0, 360.0));
/// let uniform = Uniform::new(low, high).unwrap();
/// let pose = uniform.sample(&mut rand::rng());
/// ```
#[derive(Clone)]
pub struct UniformPose {
    roll: UniformFloat<f64>,
    pitch: UniformFloat<f64>,
    yaw: UniformFloat<f64>,
}

impl UniformSampler for UniformPose {
    type X = Pose;

    fn new<B1, B2>(low: B1, high: B2) -> Result<Self, Error>
    where
        B1: SampleBorrow<Self::X> + Sized,
        B2: SampleBorrow<Self::X> + Sized,
    {
        let (lr, lp, ly) = low.borrow().clone().into();
        let (hr, hp, hy) = high.borrow().clone().into();

        Ok(UniformPose {
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
        let (lr, lp, ly) = low.borrow().clone().into();
        let (hr, hp, hy) = high.borrow().clone().into();

        Ok(UniformPose {
            roll: UniformFloat::<f64>::new_inclusive(lr, hr)?,
            pitch: UniformFloat::<f64>::new_inclusive(lp, hp)?,
            yaw: UniformFloat::<f64>::new_inclusive(ly, hy)?,
        })
    }

    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> Self::X {
        Pose::new(
            self.roll.sample(rng),
            self.pitch.sample(rng),
            self.yaw.sample(rng),
        )
    }
}

impl SampleUniform for Pose {
    type Sampler = UniformPose;
}

impl From<(f64, f64, f64)> for Pose {
    fn from(tuple: (f64, f64, f64)) -> Self {
        let (roll, pitch, yaw) = tuple;
        Self { roll, pitch, yaw }
    }
}

impl Into<(f64, f64, f64)> for Pose {
    fn into(self) -> (f64, f64, f64) {
        (self.roll, self.pitch, self.yaw)
    }
}

impl Into<Rotation3<f64>> for Pose {
    fn into(self) -> Rotation3<f64> {
        let (roll_rad, pitch_rad, yaw_rad) = self.to_radians().into();
        Rotation3::from_euler_angles(roll_rad, pitch_rad, yaw_rad)
    }
}

#[derive(Clone, Copy, Serialize, Deserialize, Debug)]
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

use chrono::prelude::*;
use nalgebra::Rotation3;
use rand::{
    distr::{Distribution, StandardUniform},
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

// In degrees
// ENU reference frame
// Euler angles
#[derive(Clone, Copy, Serialize, Deserialize, Debug, PartialEq)]
pub struct Pose {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

impl Pose {
    pub fn up() -> Self {
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

impl Distribution<Pose> for StandardUniform {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> Pose {
        Pose {
            roll: rng.random_range(0.0..360.0),
            pitch: rng.random_range(0.0..360.0),
            yaw: rng.random_range(0.0..360.0),
        }
    }
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

use crate::{
    light::{aop::Aop, dop::Dop, stokes::StokesVec},
    CameraFrd,
};
use sguaba::Coordinate;
use uom::{
    si::f64::{Angle, Length},
    ConstZero,
};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub trait RayFrame: Copy + Clone {}

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
    /// Coordinate that ray was measured.
    coord: Coordinate<CameraFrd>,

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
    pub fn new(coord: Coordinate<CameraFrd>, angle: Aop<Frame>, degree: Dop) -> Self {
        Self {
            coord,
            angle,
            degree,
            _phan: std::marker::PhantomData,
        }
    }

    pub fn from_stokes(coord: Coordinate<CameraFrd>, stokes: StokesVec<Frame>) -> Self {
        Self::new(coord, stokes.aop(), stokes.dop())
    }

    pub fn coord(&self) -> &Coordinate<CameraFrd> {
        &self.coord
    }

    pub fn aop(&self) -> &Aop<Frame> {
        &self.angle
    }

    pub fn dop(&self) -> &Dop {
        &self.degree
    }

    /// Returns the angle between `self.coord` taken from `origin` with respect
    /// to the FRD front direction.
    ///
    /// Returns `None` if `zenith_loc` is not on the sensor ie it does not have
    /// a Z component of zero.
    fn angle_to(&self, origin: Coordinate<CameraFrd>) -> Option<Angle> {
        if origin.frd_down() != Length::ZERO {
            return None;
        }

        let coord_from_origin = self.coord - origin;
        Some(
            coord_from_origin
                .frd_right()
                .atan2(coord_from_origin.frd_front()),
        )
    }
}

impl Ray<GlobalFrame> {
    /// Transforms the Ray from the GlobalFrame into the SensorFrame.
    ///
    /// Returns `None` if `zenith_coord` is not on the sensor ie it does not have
    /// a Z component of zero.
    /// Requires a zenith location because the transform occurs with reference
    /// to the global up direction.
    pub fn into_sensor_frame(
        self,
        zenith_coord: Coordinate<CameraFrd>,
    ) -> Option<Ray<SensorFrame>> {
        let offset = self.angle_to(zenith_coord)?;
        Some(Ray::new(
            self.coord,
            self.angle.into_sensor_frame(offset),
            self.degree,
        ))
    }
}

impl Ray<SensorFrame> {
    /// Transforms the Ray from the SensorFrame into the GlobalFrame.
    ///
    /// Returns `None` if `zenith_coord` is not on the sensor ie it does not have
    /// a Z component of zero.
    /// Requires a zenith location because the transform occurs with reference
    /// to the global up direction.
    pub fn into_global_frame(
        self,
        zenith_coord: Coordinate<CameraFrd>,
    ) -> Option<Ray<GlobalFrame>> {
        let offset = self.angle_to(zenith_coord)?;
        Some(Ray::new(
            self.coord,
            self.angle.into_global_frame(offset),
            self.degree,
        ))
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RaySensor {
    pixel_width: Length,
    pixel_height: Length,
    cols: u16,
    rows: u16,
}

impl RaySensor {
    pub fn new(pixel_width: Length, pixel_height: Length, cols: u16, rows: u16) -> Self {
        Self {
            pixel_width,
            pixel_height,
            cols,
            rows,
        }
    }

    /// Computes a `Coordinate` in `CameraFrd` from a pixel location.
    ///
    /// `row` maps to the FRD right direction.
    /// `col` maps to the FRD front direction.
    /// Returns `None` if the `row` and `col` are out of bounds.
    pub fn at_pixel(&self, row: u16, col: u16) -> Option<Coordinate<CameraFrd>> {
        if row > self.rows || col > self.cols {
            return None;
        }

        Some(
            Coordinate::<CameraFrd>::builder()
                .frd_front(self.pixel_width * (col as f64 - self.cols as f64 / 2.0))
                .frd_right(self.pixel_height * (row as f64 - self.rows as f64 / 2.0))
                .frd_down(Length::ZERO)
                .build(),
        )
    }
}

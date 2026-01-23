use std::marker::PhantomData;

use nalgebra::Vector2;
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use sguaba::{
    Bearing, Coordinate, Vector, coordinate, engineering::Orientation, math::RigidBodyTransform,
};
use uom::{
    ConstZero,
    si::{f64::*, ratio::ratio},
};

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SensorCoordinate {
    x: Length,
    y: Length,
}

impl SensorCoordinate {
    pub fn new(x: Length, y: Length) -> Self {
        Self { x, y }
    }

    pub fn x(&self) -> Length {
        todo!()
    }

    pub fn y(&self) -> Length {
        todo!()
    }
}

impl AsRef<SensorCoordinate> for SensorCoordinate {
    fn as_ref(&self) -> &SensorCoordinate {
        &self
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PixelCoordinate {
    row: usize,
    col: usize,
}

impl PixelCoordinate {
    pub fn new(row: usize, col: usize) -> Self {
        Self { row, col }
    }

    pub fn row(&self) -> usize {
        self.row
    }

    pub fn col(&self) -> usize {
        self.col
    }
}

impl AsRef<PixelCoordinate> for PixelCoordinate {
    fn as_ref(&self) -> &PixelCoordinate {
        &self
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ImageSensor {
    pixel_size: Length,
    rows: usize,
    cols: usize,
}

impl ImageSensor {
    pub fn new(pixel_size: Length, rows: usize, cols: usize) -> Self {
        Self {
            pixel_size,
            rows,
            cols,
        }
    }

    pub fn pixel_count(&self) -> usize {
        self.cols * self.rows
    }

    pub fn rows(&self) -> usize {
        self.rows
    }

    pub fn cols(&self) -> usize {
        self.cols
    }

    pub fn contains_pixel(&self, coord: impl AsRef<PixelCoordinate>) -> bool {
        (0..self.rows).contains(&coord.as_ref().row())
            && (0..self.cols).contains(&coord.as_ref().col())
    }

    pub fn pixel_from_sensor(
        &self,
        coord: impl AsRef<SensorCoordinate>,
    ) -> Option<PixelCoordinate> {
        let result = PixelCoordinate::new(
            ((coord.as_ref().y() / self.pixel_size).get::<ratio>()
                + self.rows.checked_sub(1)? as f64 / 2.0)
                .round() as usize,
            ((coord.as_ref().x() / self.pixel_size).get::<ratio>()
                + self.cols.checked_sub(1)? as f64 / 2.0)
                .round() as usize,
        );

        match self.contains_pixel(result) {
            true => Some(result),
            false => None,
        }
    }

    pub fn sensor_from_pixel(
        &self,
        pixel: impl AsRef<PixelCoordinate>,
    ) -> Option<SensorCoordinate> {
        match self.contains_pixel(&pixel) {
            true => Some(SensorCoordinate::new(
                self.pixel_size * (pixel.as_ref().col() as f64 - (self.cols - 1) as f64 / 2.0),
                self.pixel_size * (pixel.as_ref().row() as f64 - (self.rows - 1) as f64 / 2.0),
            )),
            false => None,
        }
    }

    fn pixels(&self) -> impl Iterator<Item = PixelCoordinate> {
        (0..self.rows).flat_map(|row| (0..self.cols).map(move |col| PixelCoordinate::new(row, col)))
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RayBearing {
    // TODO: This needs to hold actual data.
    // TODO: RayBearing should always be above the horizon.
}

impl RayBearing {
    pub fn new() -> Self {
        Self {}
    }

    pub(crate) fn azimuth(&self) -> _ {
        todo!()
    }

    pub(crate) fn elevation(&self) -> _ {
        todo!()
    }
}

pub trait Optic {
    fn trace_backward(&self, coord: SensorCoordinate) -> RayBearing;
    fn trace_forward(&self, bearing: RayBearing) -> SensorCoordinate;
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PinholeOptic {
    focal_length: Length,
}

impl PinholeOptic {
    pub fn from_focal_length(focal_length: Length) -> Self {
        if focal_length <= Length::ZERO {
            panic!("focal length must be greater than zero");
        }

        Self { focal_length }
    }
}

impl Optic for PinholeOptic {
    fn trace_backward(&self, coord: SensorCoordinate) -> RayBearing {
        // Trace a ray from the physical pixel location through the focal point.
        // This approach uses the pinhole camera model.
        // let ray_in_frd: Coordinate<CameraFrd> = coord
        //     + Vector::<CameraFrd>::builder()
        //         .frd_front(Length::ZERO)
        //         .frd_right(Length::ZERO)
        //         .frd_down(self.lens.focal_length)
        //         .build();

        // TODO: Implement
        todo!()
    }

    fn trace_forward(&self, bearing: RayBearing) -> SensorCoordinate {
        // let len = -self.lens.focal_length / bearing_frd.elevation().tan();
        // Some(
        //     coordinate! { f = len * bearing_frd.azimuth().cos(), r = len * bearing_frd.azimuth().sin(), d = Length::ZERO },
        // )

        // TODO: Implement
        todo!()
    }
}

pub struct Camera<O> {
    optic: O,
    sensor: ImageSensor,
}

impl<O> Camera<O> {
    pub fn new(optic: O, pixel_size: Length, rows: usize, cols: usize) -> Self {
        Self {
            optic,
            sensor: ImageSensor::new(pixel_size, rows, cols),
        }
    }

    pub fn pixels(&self) -> impl Iterator<Item = PixelCoordinate> {
        self.sensor.pixels()
    }

    pub fn trace_from_pixel(&self, pixel: impl AsRef<PixelCoordinate>) -> Option<RayBearing>
    where
        O: Optic,
    {
        Some(
            self.optic
                .trace_backward(self.sensor.sensor_from_pixel(pixel)?),
        )
    }

    pub fn trace_from_bearing(&self, bearing: impl AsRef<RayBearing>) -> PixelCoordinate {
        todo!()
    }

    pub fn rows(&self) -> usize {
        self.sensor.rows()
    }

    pub fn cols(&self) -> usize {
        self.sensor.cols()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::{AbsDiffEq, relative_eq};
    use quickcheck::quickcheck;
    use rstest::rstest;
    use uom::si::length::{meter, micron, millimeter};

    impl AbsDiffEq for SensorCoordinate {
        type Epsilon = f64;

        fn default_epsilon() -> Self::Epsilon {
            f64::EPSILON
        }

        fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
            // Meter is the base unit for the Length quantity.
            (self.x() - other.x()).abs().get::<meter>() <= epsilon
                && (self.y() - other.y()).abs().get::<meter>() <= epsilon
        }
    }

    quickcheck! {
        fn pinhole_trace_roundtrip(
            x_seed: i16,
            y_seed: i16
        ) -> bool {
            // Construct pixel coordinates from the random seed passed by quickcheck.
            // Aim to have pixel coordinates on range -5000 to 5000 microns.
            let x = Length::new::<micron>(x_seed as f64 * 5000. / i16::MAX as f64);
            let y = Length::new::<micron>(y_seed as f64 * 5000. / i16::MAX as f64);
            let px = SensorCoordinate::new(x, y);

            let focal_length = Length::new::<millimeter>(8.0);
            let cam = PinholeOptic::from_focal_length(focal_length);

            let result = cam.trace_forward(cam.trace_backward(px.clone()));

            // FIXME: Why doesn't this want to work??
            // relative_eq!(px, result)

            px.abs_diff_eq(&result, f64::EPSILON)
        }
    }

    #[rstest]
    #[case(0, 0)]
    #[case(512, 612)]
    #[case(106, 0)]
    #[case(0, 292)]
    fn pixel_to_coord_roundtrip(#[case] row: usize, #[case] col: usize) {
        const ROWS: usize = 1024;
        const COLS: usize = 1224;
        const PIXEL_SIZE_UM: f64 = 3.45 * 2.;

        let sensor = ImageSensor::new(Length::new::<micron>(PIXEL_SIZE_UM), ROWS, COLS);
        let px = PixelCoordinate::new(row, col);

        assert_eq!(
            px,
            sensor
                .pixel_from_sensor(sensor.sensor_from_pixel(px).expect("pixel is on sensor"))
                .expect("coord is on sensor")
        );
    }
}

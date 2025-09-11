use crate::{
    error::Error,
    iter::RayIterator,
    light::stokes::StokesVec,
    ray::{Ray, RayFrame, SensorFrame},
    CameraFrd,
};
use rayon::prelude::*;
use sguaba::Coordinate;
use uom::{si::f64::Length, si::ratio::ratio, ConstZero};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IntensityPixel {
    row: u16,
    col: u16,
    /// A metapixel is a group of four intensity pixels that have two sets of orthogonal linear polarizing filters.
    /// Each element in this buffer stores an intensity value in 0, 45, 90, 135 order.
    inner: [f64; 4],
}

impl IntensityPixel {
    /// The Stokes vectors are computed by:
    /// ```text
    /// S_0 = (I_0 + I_45 + I_90 + I_135) / 2
    /// S_1 = I_0 - I_90
    /// S_2 = I_45 - I_135
    /// ```
    fn stokes(&self) -> StokesVec<SensorFrame> {
        StokesVec::new(
            (self.inner[0] + self.inner[1] + self.inner[2] + self.inner[3]) / 2.,
            self.inner[0] - self.inner[2],
            self.inner[1] - self.inner[3],
        )
    }
}

/// A polarized intensity image.
///
/// Represents an image where each pixel measures light intensity through a
/// linear polarizing filter. This measurement can determine properties about
/// the polarization state of incident rays.
#[derive(Clone, Debug, PartialEq)]
pub struct IntensityImage {
    /// Buffer of metapixels.
    metapixels: Vec<IntensityPixel>,
    width: u16,
    height: u16,
}

impl IntensityImage {
    /// Create an intensity image from an array of bytes.
    ///
    /// A division of focal plane (DoFP) polarized camera has a micro-polarizer
    /// array in between the lens and the sensor. The micro-polarizer array
    /// allows the camera to image the intensity of light through two sets of
    /// orthogonal linear polarizing filters. The pattern of the
    /// micro-polarizer array is shown below.
    ///
    /// ```text
    /// +-----+-----+-----+-----+-----+-----+
    /// | 090 | 135 | 090 | ... | 090 | 135 |
    /// +-----+-----+-----+-----+-----+-----+
    /// | 045 | 000 | 045 | ... | 045 | 000 |
    /// +-----+-----+-----+-----+-----+-----+
    /// | 090 | 135 | ... |
    /// ```
    ///
    /// The intensity information of two set of orthogonal linear polarizing
    /// filters measures the angle and degree of linear polarization. This
    /// function expects a list of bytes organized by row.
    ///
    /// ```text
    /// +--------+--------+--------+-----+--------+--------+
    /// |      0 |      1 |      2 | ... |    w-2 |    w-1 |
    /// +--------+--------+--------+-----+--------+--------+
    /// |      w |    w+1 |    w+2 | ... |   2w-2 |   2w-1 |
    /// +--------+--------+--------+-----+--------+--------+
    /// |    ... |    ... |
    /// +--------+
    /// | w(h-1) |
    /// ```
    pub fn from_bytes(width: u16, height: u16, bytes: &[u8]) -> Result<Self, Error> {
        let meta_width = width
            .checked_div(2)
            .ok_or(Error::OddImgDim((width, height)))?;
        let meta_height = height
            .checked_div(2)
            .ok_or(Error::OddImgDim((width, height)))?;

        let coords: Vec<(u16, u16)> = (0..meta_height)
            .flat_map(|y| (0..meta_width).map(move |x| (x, y)))
            .collect();

        let metapixels: Vec<IntensityPixel> = coords
            .into_par_iter()
            .map(|(x, y)| {
                let i000 = (x as usize * 2 + 1) + (y as usize * 2 + 1) * width as usize;
                let i045 = (x as usize * 2) + (y as usize * 2 + 1) * width as usize;
                let i090 = (x as usize * 2) + (y as usize * 2) * width as usize;
                let i135 = (x as usize * 2 + 1) + (y as usize * 2) * width as usize;

                // FIXME: Catch problems with the size of `bytes`.
                IntensityPixel {
                    row: y,
                    col: x,
                    inner: [
                        bytes[i000] as f64,
                        bytes[i045] as f64,
                        bytes[i090] as f64,
                        bytes[i135] as f64,
                    ],
                }
            })
            .collect();

        Ok(Self {
            metapixels,
            width: meta_width,
            height: meta_height,
        })
    }

    pub fn width(&self) -> u16 {
        self.width
    }

    pub fn height(&self) -> u16 {
        self.height
    }

    pub fn rays<'a>(&'a self, pixel_width: Length, pixel_height: Length) -> Rays<'a> {
        Rays {
            inner: self.metapixels.iter(),
            // Constructs a `ImageSensor` using the dimensions of the `IntensityImage`.
            sensor: ImageSensor::new(pixel_width, pixel_height, self.height, self.width),
        }
    }
}

/// An iterator over rays.
#[derive(Clone, Debug)]
pub struct Rays<'a> {
    inner: std::slice::Iter<'a, IntensityPixel>,
    sensor: ImageSensor,
}

impl<'a> Iterator for Rays<'a> {
    type Item = Ray<SensorFrame>;
    fn next(&mut self) -> Option<Self::Item> {
        let px = self.inner.next()?;
        Some(Ray::from_stokes(
            self.sensor
                .at_pixel(px.row, px.col)
                // This iterator is created with a ImageSensor that matches the
                // parent IntensityImage which guarantees that all pixel
                // locations are inside the ImageSensor bounds.
                .expect("sensor dimensions should match image dimensions"),
            px.stokes(),
        ))
    }
}

// All of RayIterator's functions are defined using Iterator.
impl<'a> RayIterator<SensorFrame> for Rays<'a> {}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ImageSensor {
    pixel_width: Length,
    pixel_height: Length,
    rows: usize,
    cols: usize,
}

impl ImageSensor {
    pub fn new(pixel_width: Length, pixel_height: Length, rows: u16, cols: u16) -> Self {
        Self {
            pixel_width,
            pixel_height,
            rows: rows as usize,
            cols: cols as usize,
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

    pub fn at_coord(&self, coord: Coordinate<CameraFrd>) -> Option<(u16, u16)> {
        let row = ((coord.frd_right() / self.pixel_height).get::<ratio>()
            + self.rows.checked_sub(1)? as f64 / 2.0)
            .round() as usize;
        let col = ((coord.frd_front() / self.pixel_width).get::<ratio>()
            + self.cols.checked_sub(1)? as f64 / 2.0)
            .round() as usize;

        if (0..self.rows).contains(&row) && (0..self.cols).contains(&col) {
            return Some((row as u16, col as u16));
        }

        None
    }

    /// Computes a `Coordinate` in `CameraFrd` from a pixel location.
    ///
    /// `row` maps to the FRD right direction.
    /// `col` maps to the FRD front direction.
    /// Returns `None` if the `row` and `col` are out of bounds.
    pub fn at_pixel(&self, row: u16, col: u16) -> Option<Coordinate<CameraFrd>> {
        if row as usize > self.rows || col as usize > self.cols {
            return None;
        }

        Some(
            Coordinate::<CameraFrd>::builder()
                .frd_front(self.pixel_width * (col as f64 - (self.cols - 1) as f64 / 2.0))
                .frd_right(self.pixel_height * (row as f64 - (self.rows - 1) as f64 / 2.0))
                .frd_down(Length::ZERO)
                .build(),
        )
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct RayImage<Frame: RayFrame> {
    pixels: Vec<Option<Ray<Frame>>>,
    _phan: std::marker::PhantomData<Frame>,
}

impl<Frame: RayFrame> RayImage<Frame> {
    pub fn new() -> Self {
        Self {
            pixels: Vec::new(),
            _phan: std::marker::PhantomData,
        }
    }

    pub fn from_pixels(pixels: Vec<Option<Ray<Frame>>>) -> Self {
        Self {
            pixels,
            _phan: std::marker::PhantomData,
        }
    }

    pub fn from_rays_with_sensor<I>(rays: I, sensor: &ImageSensor) -> Option<Self>
    where
        I: IntoIterator<Item = Ray<Frame>>,
    {
        let mut pixels = vec![None; sensor.pixel_count()];
        for ray in rays {
            if let Some((row, col)) = sensor.at_coord(*ray.coord()) {
                let index = row as usize * sensor.cols() + col as usize;
                if pixels[index].is_some() {
                    return None;
                }
                pixels[index] = Some(ray);
            } else {
                return None;
            }
        }

        Some(Self::from_pixels(pixels))
    }

    pub fn ray_pixels<'a>(&'a self) -> RayPixels<'a, Frame> {
        RayPixels::new(self.pixels.iter())
    }
}

#[derive(Clone, Debug)]
pub struct RayPixels<'a, Frame: RayFrame> {
    inner: std::slice::Iter<'a, Option<Ray<Frame>>>,
}

impl<'a, Frame: RayFrame> RayPixels<'a, Frame> {
    fn new(inner: std::slice::Iter<'a, Option<Ray<Frame>>>) -> Self {
        Self { inner }
    }
}

impl<'a, Frame: RayFrame> Iterator for RayPixels<'a, Frame> {
    type Item = &'a Option<Ray<Frame>>;

    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;
    use uom::si::length::micron;

    #[rstest]
    #[case(0, 0)]
    #[case(512, 612)]
    #[case(106, 0)]
    #[case(0, 292)]
    fn pixel_to_coord_roundtrip(#[case] row: u16, #[case] col: u16) {
        const ROWS: u16 = 1024;
        const COLS: u16 = 1224;
        const PIXEL_SIZE_UM: f64 = 3.45 * 2.;

        let sensor = ImageSensor::new(
            Length::new::<micron>(PIXEL_SIZE_UM),
            Length::new::<micron>(PIXEL_SIZE_UM),
            ROWS,
            COLS,
        );

        let coord = sensor.at_pixel(row, col).expect("pixel is on sensor");
        let pixel = sensor.at_coord(coord).expect("coord is on sensor");

        assert_eq!(pixel, (row, col));
    }
}

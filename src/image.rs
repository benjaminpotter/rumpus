use super::{
    error::Error,
    iter::RayIterator,
    light::{
        ray::{Ray, RaySensor, SensorFrame},
        stokes::StokesVec,
    },
};
use rayon::prelude::*;
use uom::si::f64::Length;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IntensityPixel {
    loc: (u16, u16),
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
                let i000 = ((x as usize * 2 + 1) + (y as usize * 2 + 1) * width as usize);
                let i045 = ((x as usize * 2) + (y as usize * 2 + 1) * width as usize);
                let i090 = ((x as usize * 2) + (y as usize * 2) * width as usize);
                let i135 = ((x as usize * 2 + 1) + (y as usize * 2) * width as usize);

                // FIXME: Catch problems with the size of `bytes`.
                IntensityPixel {
                    loc: (x, y),
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

    pub fn rays<'a>(&'a self, pixel_width: Length, pixel_height: Length) -> Rays<'a> {
        Rays {
            inner: self.metapixels.iter(),
            // Constructs a `RaySensor` using the dimensions of the `IntensityImage`.
            sensor: RaySensor::new(pixel_width, pixel_height, self.width, self.height),
        }
    }
}

/// An iterator over rays.
#[derive(Clone, Debug)]
pub struct Rays<'a> {
    inner: std::slice::Iter<'a, IntensityPixel>,
    sensor: RaySensor,
}

impl<'a> Iterator for Rays<'a> {
    type Item = Ray<SensorFrame>;
    fn next(&mut self) -> Option<Self::Item> {
        let px = self.inner.next()?;
        Some(Ray::from_stokes(
            self.sensor
                .at_pixel(px.loc.1, px.loc.0)
                // This iterator is created with a RaySensor that matches the
                // parent IntensityImage which guarantees that all pixel
                // locations are inside the RaySensor bounds.
                .expect("sensor dimensions should match image dimensions"),
            px.stokes(),
        ))
    }
}

// All of RayIterator's functions are defined using Iterator.
impl<'a> RayIterator<SensorFrame> for Rays<'a> {}

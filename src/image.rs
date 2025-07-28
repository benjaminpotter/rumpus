use crate::{
    error::Error,
    ray::{Ray, RayIterator, StokesVec},
};
use rayon::prelude::*;

pub struct IntensityPixel {
    loc: (u32, u32),
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
    fn to_stokes(&self) -> StokesVec {
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
pub struct IntensityImage {
    /// Buffer of metapixels.
    metapixels: Vec<IntensityPixel>,
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
    pub fn from_bytes(width: u32, height: u32, bytes: &[u8]) -> Result<Self, Error> {
        // TODO:
        // - Allow dimensions to mismatch the bytes and do interpolation.
        // - Maybe make another function for users that want this functionality.

        let dims = (
            width
                .checked_div(2)
                .ok_or(Error::InvalidInput("image width is not even".into()))?,
            height
                .checked_div(2)
                .ok_or(Error::InvalidInput("image height is not even".into()))?,
        );

        let coords: Vec<(u32, u32)> = (0..dims.1)
            .into_iter()
            .map(|y| (0..dims.0).into_iter().map(move |x| (x, y)))
            .flatten()
            .collect();

        let metapixels: Vec<IntensityPixel> = coords
            .into_par_iter()
            .map(|(x, y)| {
                let i000 = ((x * 2 + 1) + (y * 2 + 1) * width) as usize;
                let i045 = ((x * 2 + 0) + (y * 2 + 1) * width) as usize;
                let i090 = ((x * 2 + 0) + (y * 2 + 0) * width) as usize;
                let i135 = ((x * 2 + 1) + (y * 2 + 0) * width) as usize;

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

        Ok(Self { metapixels })
    }

    pub fn rays(&self) -> ImageRays {
        ImageRays {
            inner: self.metapixels.iter(),
        }
    }
}

/// An iterator over rays.
pub struct ImageRays<'a> {
    inner: std::slice::Iter<'a, IntensityPixel>,
}

impl<'a> Iterator for ImageRays<'a> {
    type Item = Ray;
    fn next(&mut self) -> Option<Self::Item> {
        let px = self.inner.next()?;
        let loc = px.loc.clone();
        Some(Ray::from_stokes(loc, px.to_stokes()))
    }
}

// All of RayIterator's functions are defined using Iterator.
impl<'a> RayIterator for ImageRays<'a> {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ray::{Aop, Dop, Ray};
    use image::{GrayImage, ImageReader};

    #[test]
    fn first_ray() {
        let image = read_image();
        let (width, height) = image.dimensions();
        let ray = IntensityImage::from_bytes(width, height, &image.into_raw())
            .unwrap()
            .rays()
            .next()
            .unwrap();

        assert_eq!(
            ray,
            Ray::new((0, 0), Aop::from_deg(90.0), Dop::new(0.2222222222222222))
        );
    }

    fn read_image() -> GrayImage {
        ImageReader::open("testing/intensity.png")
            .unwrap()
            .decode()
            .unwrap()
            .into_luma8()
    }
}

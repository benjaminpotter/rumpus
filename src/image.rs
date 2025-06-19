use crate::error::Error;
use rayon::prelude::*;

/// A polarized intensity image.
///
/// Represents an image where each pixel measures light intensity through a linear polarizing filter.
/// This measurement can determine properties about the polarization state of incident rays.
pub struct IntensityImage {
    /// The dimensions of the metaimage.
    dims: (u32, u32),

    /// Buffer of metapixels.
    ///
    /// A metapixel is a group of four intensity pixels that have two sets of orthogonal linear polarizing filters.
    /// Each element in this buffer stores an intensity value in 0, 45, 90, 135 order.
    metapixels: Vec<[f64; 4]>,
}

impl IntensityImage {
    /// Create an intensity image from an array of bytes.
    ///
    /// A division of focal plane (DoFP) polarized camera has a micro-polarizer array in between the lens and the sensor.
    /// The micro-polarizer array allows the camera to image the intensity of light through two sets of orthogonal linear polarizing filters.
    /// The pattern of the micro-polarizer array is shown below.
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
    /// The intensity information of two set of orthogonal linear polarizing filters measures the angle and degree of linear polarization.
    /// This function expects a list of bytes organized by row.
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

        let metapixels: Vec<[f64; 4]> = coords
            .par_iter()
            .map(|(x, y)| {
                let i000 = ((x * 2 + 1) + (y * 2 + 1) * width) as usize;
                let i045 = ((x * 2 + 0) + (y * 2 + 1) * width) as usize;
                let i090 = ((x * 2 + 0) + (y * 2 + 0) * width) as usize;
                let i135 = ((x * 2 + 1) + (y * 2 + 0) * width) as usize;

                // FIXME: Catch problems with the size of `bytes`.
                [
                    bytes[i000] as f64,
                    bytes[i045] as f64,
                    bytes[i090] as f64,
                    bytes[i135] as f64,
                ]
            })
            .collect();

        Ok(Self { dims, metapixels })
    }

    /// Convert an owned intensity image to a StokesImage by computing Stokes vectors.
    pub fn into_stokes_image(self) -> StokesImage {
        let dims = self.dims;
        let frame = StokesReferenceFrame::Sensor;
        let pixels: Vec<[f64; 3]> = self
            .metapixels
            .into_iter()
            .map(|mp| {
                [
                    (mp[0] + mp[1] + mp[2] + mp[3]) / 2.,
                    mp[0] - mp[2],
                    mp[1] - mp[3],
                ]
            })
            .collect();

        StokesImage {
            dims,
            frame,
            pixels,
        }
    }
}

/// Describes the reference frame for the S_1 and S_2 Stokes parameters.
#[derive(Debug, PartialEq)]
pub enum StokesReferenceFrame {
    /// S_1 and S_2 parameters are with reference to the 0 degree linear polarizer of the image sensor.
    Sensor,

    /// S_1 and S_2 parameters are with reference to the incident ray which changes with each pixel.
    Pixel,
}

pub struct StokesImage {
    dims: (u32, u32),
    frame: StokesReferenceFrame,
    pixels: Vec<[f64; 3]>,
}

impl StokesImage {
    pub fn dimensions(&self) -> (u32, u32) {
        self.dims
    }

    pub fn par_transform_frame(mut self, frame: StokesReferenceFrame) -> Self {
        if frame == self.frame {
            return self;
        }

        let origin = (self.dims.0 as f64 / 2., self.dims.1 as f64 / 2.);
        let coords: Vec<(f64, f64)> = (0..self.dims.1)
            .into_iter()
            .map(|y| (0..self.dims.0).into_iter().map(move |x| (x, y)))
            .flatten()
            .map(|(x, y)| (x as f64, y as f64))
            .map(|(x, y)| (x - origin.0, y - origin.1))
            .collect();

        match frame {
            StokesReferenceFrame::Sensor => unimplemented!(),
            StokesReferenceFrame::Pixel => coords
                .par_iter()
                .zip(self.pixels.par_iter_mut())
                .for_each(|((x, y), pixel)| {
                    let beta = y.atan2(*x) * 2.;
                    let s1 = pixel[1] * beta.cos() + pixel[2] * beta.sin();
                    let s2 = pixel[2] * beta.cos() - pixel[1] * beta.sin();
                    pixel[1] = s1;
                    pixel[2] = s2;
                }),
        };

        self.frame = frame;
        self
    }

    pub fn par_compute_aop_image(&self) -> Vec<f64> {
        self.pixels
            .par_iter()
            .map(|sv| sv[2].atan2(sv[1]) / 2.)
            .map(|aop| aop.to_degrees())
            .collect()
    }

    pub fn par_compute_dop_image(&self, max: f64) -> Vec<f64> {
        self.pixels
            .par_iter()
            .map(|sv| (sv[1].powf(2.) + sv[2].powf(2.)).sqrt() / sv[0])
            .map(|dop| dop.clamp(0., max))
            .collect()
    }
}

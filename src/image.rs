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
    ///
    /// The Stokes vectors are computed by:
    /// ```text
    /// S_0 = (I_0 + I_45 + I_90 + I_135) / 2
    /// S_1 = I_0 - I_90
    /// S_2 = I_45 - I_135
    /// ```
    pub fn into_stokes_image(self) -> StokesImage {
        let dims = self.dims;
        let frame = StokesReferenceFrame::Sensor;
        let pixels: Vec<[f64; 3]> = self
            .metapixels
            .par_iter()
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
#[derive(Debug, PartialEq, Clone)]
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

    /// Returns a reference to a Stokes vector if `pixel` is within `self.dims`, otherwise returns None.
    fn get_pixel(&self, pixel: (u32, u32)) -> Option<&[f64; 3]> {
        let index = (pixel.1 * self.dims.0 + pixel.0) as usize;
        self.pixels.get(index)
    }

    /// Compute an AoP from a Stokes vector.
    fn sv_to_aop(sv: &[f64; 3]) -> f64 {
        (sv[2].atan2(sv[1]) / 2.).to_degrees()
    }

    /// Compute a DoP from a Stokes vector.
    fn sv_to_dop(sv: &[f64; 3]) -> f64 {
        (sv[1].powf(2.) + sv[2].powf(2.)).sqrt() / sv[0]
    }

    /// Returns an AoP if `pixel` is within `self.dims`, otherwise returns None.
    fn aop_at(&self, pixel: (u32, u32)) -> Option<f64> {
        let sv = self.get_pixel(pixel)?;
        Some(StokesImage::sv_to_aop(sv))
    }

    /// Returns a DoP if `pixel` is within `self.dims`, otherwise returns None.
    fn dop_at(&self, pixel: (u32, u32)) -> Option<f64> {
        let sv = self.get_pixel(pixel)?;
        Some(StokesImage::sv_to_dop(sv))
    }

    /// Convert an owned Stokes image to a AopDopImage by computing AoP and DoP values.
    pub fn into_aop_dop_image(self) -> (AopImage, DopImage) {
        let frame = self.frame.clone();
        let dims = self.dims.clone();
        let (aop_pixels, dop_pixels): (Vec<_>, Vec<_>) = self
            .pixels
            .par_iter()
            .map(|sv| (StokesImage::sv_to_aop(sv), StokesImage::sv_to_dop(sv)))
            .unzip();

        (
            AopImage {
                pixels: aop_pixels,
                frame,
                dims,
            },
            DopImage {
                pixels: dop_pixels,
                dims,
            },
        )
    }
}

pub struct AopImage {
    pixels: Vec<f64>,
    frame: StokesReferenceFrame,
    dims: (u32, u32),
}

impl AopImage {
    pub fn dimensions(&self) -> (u32, u32) {
        self.dims
    }

    pub fn as_slice(&self) -> &[f64] {
        self.pixels.as_slice()
    }

    pub fn into_vec(self) -> Vec<f64> {
        self.pixels
    }
}

pub struct DopImage {
    pixels: Vec<f64>,
    dims: (u32, u32),
}

impl DopImage {
    pub fn dimensions(&self) -> (u32, u32) {
        self.dims
    }

    pub fn as_slice(&self) -> &[f64] {
        self.pixels.as_slice()
    }

    pub fn into_vec(self) -> Vec<f64> {
        self.pixels
    }
}

/// Map an f64 on the interval [x_min, x_max] to an RGB color.
pub fn to_rgb(x: f64, x_min: f64, x_max: f64) -> Option<[u8; 3]> {
    if x < x_min || x > x_max {
        return None;
    }

    let interval_width = x_max - x_min;
    let x_norm = ((x - x_min) / interval_width * 255.).floor() as u8;

    let r = vec![
        255,
        x_norm
            .checked_sub(96)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
        255 - x_norm
            .checked_sub(224)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
    ]
    .into_iter()
    .min()
    .unwrap();

    let g = vec![
        255,
        x_norm
            .checked_sub(32)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
        255 - x_norm
            .checked_sub(160)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
    ]
    .into_iter()
    .min()
    .unwrap();

    let b = vec![
        255,
        x_norm
            .checked_add(127)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
        255 - x_norm
            .checked_sub(96)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
    ]
    .into_iter()
    .min()
    .unwrap();

    Some([r, g, b])
}

#[cfg(test)]
mod tests {
    use super::*;
}

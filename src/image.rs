use crate::{
    error::Error,
    iter::RayIterator,
    light::stokes::StokesVec,
    ray::{Ray, RayFrame, SensorFrame},
};
use rayon::prelude::*;
use sguaba::Coordinate;
use thiserror::Error;
use uom::{
    ConstZero,
    si::{angle::degree, f64::Length, ratio::ratio},
};

#[derive(Debug, Error)]
pub enum ImageError {
    #[error("length of data does not match size of extents: expected {} found {len}", rows * cols)]
    SizeMismatch {
        rows: usize,
        cols: usize,
        len: usize,
    },

    #[error(
        "intensity image reader requires even numbered image dimensions: found {}x{}",
        width,
        height
    )]
    InvalidDimensions { width: usize, height: usize },
}

#[derive(Clone, Debug, PartialEq)]
struct Matrix<T> {
    elements: Vec<T>,
    rows: usize,
    cols: usize,
}

impl<T> Matrix<T> {
    fn from_elements(
        elements: impl IntoIterator<Item = T>,
        rows: usize,
        cols: usize,
    ) -> Result<Self, ImageError> {
        let elements: Vec<_> = elements.into_iter().collect();
        let len = elements.len();
        if rows * cols != len {
            Err(ImageError::SizeMismatch { rows, cols, len })
        } else {
            Ok(Self {
                elements,
                rows,
                cols,
            })
        }
    }

    fn iter_elements(&self) -> impl Iterator<Item = &T> {
        self.elements.iter()
    }

    fn map_by<U>(&self, map: impl Fn(&T) -> U) -> Matrix<U> {
        let elements: Vec<_> = self.iter_elements().map(|elem| map(elem)).collect();
        Matrix::from_elements(elements, self.rows, self.cols)
            // This is fine as long as the type continues to maintain the invariant that the length
            // of the elements list matches the size of the extents.
            .expect("len of elements matches extents")
    }

    fn index(&self, row: usize, col: usize) -> usize {
        row * self.cols + col
    }

    fn get(&self, row: usize, col: usize) -> &T {
        let index = self.index(row, col);
        &self.elements[index]
    }

    fn get_mut(&mut self, row: usize, col: usize) -> &mut T {
        let index = self.index(row, col);
        &mut self.elements[index]
    }

    fn rows(&self) -> usize {
        self.rows
    }

    fn cols(&self) -> usize {
        self.cols
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IntensityPixel {
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
    width: usize,
    height: usize,
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
    pub fn from_bytes(width: usize, height: usize, bytes: &[u8]) -> Result<Self, ImageError> {
        let meta_width = width
            .checked_div(2)
            .ok_or(ImageError::InvalidDimensions { width, height })?;
        let meta_height = height
            .checked_div(2)
            .ok_or(ImageError::InvalidDimensions { width, height })?;

        let coords: Vec<(usize, usize)> = (0..meta_height)
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

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn rays<'a>(&'a self) -> Rays<'a> {
        Rays {
            inner: self.metapixels.iter(),
        }
    }
}

/// An iterator over rays.
#[derive(Clone, Debug)]
pub struct Rays<'a> {
    inner: std::slice::Iter<'a, IntensityPixel>,
}

impl<'a> Iterator for Rays<'a> {
    type Item = Ray<SensorFrame>;
    fn next(&mut self) -> Option<Self::Item> {
        let px = self.inner.next()?;
        Some(Ray::from_stokes(px.stokes()))
    }
}

// All of RayIterator's functions are defined using Iterator.
impl<'a> RayIterator<SensorFrame> for Rays<'a> {}

#[derive(Clone, Debug, PartialEq)]
pub struct RayImage<Frame: RayFrame> {
    inner: Matrix<Option<Ray<Frame>>>,
    _phan: std::marker::PhantomData<Frame>,
}

impl<Frame: RayFrame> RayImage<Frame> {
    fn from_matrix(matrix: Matrix<Option<Ray<Frame>>>) -> Self {
        Self {
            inner: matrix,
            _phan: std::marker::PhantomData,
        }
    }

    pub fn from_pixels(
        pixels: Vec<Option<Ray<Frame>>>,
        rows: usize,
        cols: usize,
    ) -> Result<Self, ImageError> {
        let matrix = Matrix::from_elements(pixels, rows, cols)?;
        Ok(Self::from_matrix(matrix))
    }

    pub fn rows(&self) -> usize {
        self.inner.rows()
    }

    pub fn cols(&self) -> usize {
        self.inner.cols()
    }

    pub fn pixels(&self) -> impl Iterator<Item = Option<&Ray<Frame>>> {
        self.inner.iter_elements().map(|elem| elem.as_ref())
    }

    pub fn aop_bytes<M: ColorMap>(&self, color_map: &M) -> Vec<u8> {
        self.pixels()
            .map(|pixel| {
                pixel
                    .map(|ray| ray.aop().get::<degree>())
                    .unwrap_or(f64::NAN)
            })
            .flat_map(|value| color_map.map(value, -90.0, 90.0))
            .collect()
    }

    pub fn dop_bytes<M: ColorMap>(&self, color_map: &M) -> Vec<u8> {
        self.pixels()
            .map(|pixel| pixel.map(|ray| ray.dop().into_inner()).unwrap_or(f64::NAN))
            .flat_map(|value| color_map.map(value, 0.0, 1.0))
            .collect()
    }
}

pub trait ColorMap {
    fn map(&self, value: f64, min: f64, max: f64) -> [u8; 3];
}

pub struct Jet;
impl ColorMap for Jet {
    fn map(&self, value: f64, min: f64, max: f64) -> [u8; 3] {
        if value < min || value > max {
            return [255, 255, 255];
        }

        let interval_width = max - min;
        let x_norm = ((value - min) / interval_width * 255.).floor() as u8;

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

        [r, g, b]
    }
}

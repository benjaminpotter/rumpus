use crate::{
    iter::RayIterator,
    light::stokes::StokesVec,
    ray::{Ray, SensorFrame},
};
use rayon::prelude::*;
use thiserror::Error;
use uom::si::{angle::degree, f64::Angle};

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
        if rows * cols == len {
            Ok(Self {
                elements,
                rows,
                cols,
            })
        } else {
            Err(ImageError::SizeMismatch { rows, cols, len })
        }
    }

    fn iter(&self) -> impl Iterator<Item = &T> {
        self.elements.iter()
    }

    fn cells(&self) -> Cells<'_, T> {
        Cells::new(&self.elements, self.rows, self.cols)
    }

    fn rows(&self) -> usize {
        self.rows
    }

    fn cols(&self) -> usize {
        self.cols
    }

    fn index(&self, row: usize, col: usize) -> usize {
        row * self.cols() + col
    }

    fn cell(&self, row: usize, col: usize) -> &T {
        &self.elements[self.index(row, col)]
    }
}

struct Cells<'a, T> {
    elements: std::vec::IntoIter<&'a T>,
    index: usize,
    rows: usize,
    cols: usize,
}

#[derive(Clone, Debug, PartialEq)]
struct MatrixCell<'a, T> {
    element: &'a T,
    row: usize,
    col: usize,
}

impl<'a, T> Cells<'a, T> {
    fn new(elements: impl IntoIterator<Item = &'a T>, rows: usize, cols: usize) -> Self {
        let elements: Vec<_> = elements.into_iter().collect();
        Self {
            elements: elements.into_iter(),
            index: 0,
            rows,
            cols,
        }
    }
}

impl<'a, T> Iterator for Cells<'a, T> {
    type Item = MatrixCell<'a, T>;

    fn next(&mut self) -> Option<Self::Item> {
        let row = self.index / self.cols;
        let col = self.index % self.cols;
        let Some(element) = self.elements.next() else {
            // If there are no more elements to yield:
            assert_eq!(self.rows, row);
            assert_eq!(col, 0);
            return None;
        };

        self.index += 1;

        Some(MatrixCell { element, row, col })
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
    /// Create an [`IntensityImage`] from an array of bytes.
    ///
    /// A division of focal plane polarized camera has a micro-polarizer
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
    ///
    /// # Errors
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
                let i000 = (x * 2 + 1) + (y * 2 + 1) * width;
                let i045 = (x * 2) + (y * 2 + 1) * width;
                let i090 = (x * 2) + (y * 2) * width;
                let i135 = (x * 2 + 1) + (y * 2) * width;

                // FIXME: Catch problems with the size of `bytes`.
                IntensityPixel {
                    inner: [
                        f64::from(bytes[i000]),
                        f64::from(bytes[i045]),
                        f64::from(bytes[i090]),
                        f64::from(bytes[i135]),
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

    #[must_use]
    pub fn width(&self) -> usize {
        self.width
    }

    #[must_use]
    pub fn height(&self) -> usize {
        self.height
    }

    #[must_use]
    pub fn rays(&self) -> Rays<'_> {
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

impl Iterator for Rays<'_> {
    type Item = Ray<SensorFrame>;
    fn next(&mut self) -> Option<Self::Item> {
        let px = self.inner.next()?;
        // TODO: Might want to propagate this error..
        Ray::try_from(px.stokes()).ok()
    }
}

// All of RayIterator's functions are defined using Iterator.
impl RayIterator<SensorFrame> for Rays<'_> {}

#[derive(Clone, Debug, PartialEq)]
pub struct RayImage<Frame> {
    inner: Matrix<Option<Ray<Frame>>>,
    _phan: std::marker::PhantomData<Frame>,
}

impl<Frame> RayImage<Frame> {
    fn from_matrix(matrix: Matrix<Option<Ray<Frame>>>) -> Self {
        Self {
            inner: matrix,
            _phan: std::marker::PhantomData,
        }
    }

    ///
    /// # Errors
    pub fn from_rays(
        rays: impl IntoIterator<Item = Option<Ray<Frame>>>,
        rows: usize,
        cols: usize,
    ) -> Result<Self, ImageError> {
        let matrix = Matrix::from_elements(rays, rows, cols)?;
        Ok(Self::from_matrix(matrix))
    }

    #[must_use]
    pub fn rows(&self) -> usize {
        self.inner.rows()
    }

    #[must_use]
    pub fn cols(&self) -> usize {
        self.inner.cols()
    }

    #[must_use]
    pub fn ray(&self, row: usize, col: usize) -> Option<&Ray<Frame>> {
        self.inner.cell(row, col).as_ref()
    }

    pub fn rays(&self) -> impl Iterator<Item = Option<&Ray<Frame>>> {
        self.inner.iter().map(|elem| elem.as_ref())
    }

    pub fn pixels(&self) -> impl Iterator<Item = RayPixel<'_, Frame>> {
        self.inner.cells().map(|cell| RayPixel {
            ray: cell.element.as_ref(),
            row: cell.row,
            col: cell.col,
        })
    }

    pub fn aop_bytes<M>(&self, color_map: &M) -> Vec<u8>
    where
        Frame: Copy,
        M: ColorMap,
        M::Output: IntoIterator<Item = u8>,
    {
        self.rays()
            .map(|pixel| pixel.map_or(f64::NAN, |ray| Angle::from(ray.aop()).get::<degree>()))
            .flat_map(|value| color_map.map(value, -90.0, 90.0))
            .collect()
    }

    pub fn dop_bytes<M>(&self, color_map: &M) -> Vec<u8>
    where
        M: ColorMap,
        M::Output: IntoIterator<Item = u8>,
    {
        self.rays()
            .map(|pixel| pixel.map_or(f64::NAN, |ray| f64::from(ray.dop())))
            .flat_map(|value| color_map.map(value, 0.0, 1.0))
            .collect()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RayPixel<'a, Frame> {
    ray: Option<&'a Ray<Frame>>,
    row: usize,
    col: usize,
}

impl<'a, Frame> RayPixel<'a, Frame> {
    #[must_use]
    pub fn ray(&self) -> Option<&'a Ray<Frame>> {
        self.ray
    }

    #[must_use]
    pub fn row(&self) -> usize {
        self.row
    }

    #[must_use]
    pub fn col(&self) -> usize {
        self.col
    }
}

pub trait ColorMap {
    type Output;

    fn map(&self, value: f64, min: f64, max: f64) -> Self::Output;
}

pub struct Jet;
impl ColorMap for Jet {
    type Output = [u8; 3];

    fn map(&self, value: f64, min: f64, max: f64) -> Self::Output {
        if value < min || value > max {
            return [255, 255, 255];
        }

        let interval_width = max - min;

        #[allow(clippy::cast_possible_truncation)]
        #[allow(clippy::cast_sign_loss)]
        let x_norm = ((value - min) / interval_width * 255.).floor() as u8;

        let r = vec![
            255,
            x_norm.saturating_sub(96).saturating_mul(4),
            255 - x_norm.saturating_sub(224).saturating_mul(4),
        ]
        .into_iter()
        .min()
        .unwrap();

        let g = vec![
            255,
            x_norm.saturating_sub(32).saturating_mul(4),
            255 - x_norm.saturating_sub(160).saturating_mul(4),
        ]
        .into_iter()
        .min()
        .unwrap();

        let b = vec![
            255,
            x_norm.saturating_add(127).saturating_mul(4),
            255 - x_norm.saturating_sub(96).saturating_mul(4),
        ]
        .into_iter()
        .min()
        .unwrap();

        [r, g, b]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matrix_cells() {
        let elements = vec![10, 20, 30, 1, 2, 3];
        let matrix = Matrix {
            elements: elements.clone(),
            rows: 2,
            cols: 3,
        };

        assert_eq!(
            matrix.cells().nth(3),
            Some(MatrixCell {
                element: &elements[3],
                row: 1,
                col: 0,
            })
        );
    }
}

use crate::ray::{Aop, Dop, Ray};

/// A predicate over a ray.
///
/// Implementors of this `trait` are used with [`RayFilter`].
///
/// [`RayFilter`]: RayFilter
pub trait RayPredicate {
    fn eval(&self, ray: &Ray) -> bool;
}

/// A predicate that holds on rays with
/// `center - thres <= Aop <= center + thres` and handles wrapping.
pub struct AopFilter {
    center: Aop,
    thres: f64,
}

impl AopFilter {
    pub fn new(center: Aop, thres: f64) -> Self {
        Self { center, thres }
    }
}

impl RayPredicate for AopFilter {
    fn eval(&self, ray: &Ray) -> bool {
        self.center.in_thres(ray.get_aop(), self.thres)
    }
}

/// A predicate that holds on rays with `Dop >= min`.
pub struct DopFilter {
    min: Dop,
}

impl DopFilter {
    pub fn new(min: f64) -> Self {
        Self { min: Dop::new(min) }
    }
}

impl RayPredicate for DopFilter {
    fn eval(&self, ray: &Ray) -> bool {
        self.min <= *ray.get_dop()
    }
}

// struct CircleFilter
//   - radius
//   - center
//   - impl MeasurementFilter
// - Includes the measurement if inside circle

// struct Filter<I, F>
// - iter: I
// - mm_filter: F
//
// - impl<I, F> Iterator for Filter<I, F> where I: Iterator<Item = Measurement>, F:MeasurementFilter
// - type Item = I::Item
// - fn next(&mut self) -> Option<Self::Item> calls self.iter.find(|mm| self.mm_filter.filter(mm))

/// An iterator that filters rays from `iter` with `pred.eval`.
pub struct RayFilter<I, P> {
    iter: I,
    pred: P,
}

impl<I, P> RayFilter<I, P> {
    pub fn new(iter: I, pred: P) -> Self {
        Self { iter, pred }
    }
}

impl<I, P> Iterator for RayFilter<I, P>
where
    I: Iterator<Item = Ray>,
    P: RayPredicate,
{
    type Item = Ray;
    fn next(&mut self) -> Option<Self::Item> {
        self.iter.find(|ray| self.pred.eval(&ray))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{image::IntensityImage, ray::Ray};
    use image::{GrayImage, ImageReader};

    #[test]
    fn dop_filter() {
        const MIN: f64 = 0.65;
        let rays = parse_rays();
        let filt = DopFilter::new(MIN);
        for ray in rays {
            assert_eq!(Dop::new(MIN) <= *ray.get_dop(), filt.eval(&ray));
        }
    }

    fn read_image() -> GrayImage {
        ImageReader::open("testing/intensity.png")
            .unwrap()
            .decode()
            .unwrap()
            .into_luma8()
    }

    fn parse_rays() -> Vec<Ray> {
        let image = read_image();
        let (width, height) = image.dimensions();
        IntensityImage::from_bytes(width, height, &image.into_raw())
            .unwrap()
            .rays()
            .collect()
    }
}

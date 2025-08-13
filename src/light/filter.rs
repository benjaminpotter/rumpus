use super::{
    aop::Aop,
    dop::Dop,
    ray::{Ray, RayFrame},
};
use uom::si::f64::Angle;

/// A predicate over a ray.
///
/// Implementors of this `trait` are used with [`RayFilter`].
///
/// [`RayFilter`]: RayFilter
pub trait RayPredicate<Frame: RayFrame> {
    fn eval(&self, ray: &Ray<Frame>) -> bool;
}

/// A predicate that holds on rays with
/// `center - thres <= Aop <= center + thres` and handles wrapping.
pub struct AopFilter<Frame: RayFrame> {
    center: Aop<Frame>,
    thres: Angle,
}

impl<Frame: RayFrame> AopFilter<Frame> {
    pub fn new(center: Aop<Frame>, thres: Angle) -> Self {
        Self { center, thres }
    }
}

impl<Frame: RayFrame> RayPredicate<Frame> for AopFilter<Frame> {
    fn eval(&self, ray: &Ray<Frame>) -> bool {
        self.center.in_thres(ray.aop(), self.thres)
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

impl<Frame: RayFrame> RayPredicate<Frame> for DopFilter {
    fn eval(&self, ray: &Ray<Frame>) -> bool {
        self.min <= *ray.dop()
    }
}

// struct CircleFilter
//   - radius
//   - center
//   - impl MeasurementFilter
// - Includes the measurement if inside circle

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

impl<I, P, Frame: RayFrame> Iterator for RayFilter<I, P>
where
    I: Iterator<Item = Ray<Frame>>,
    P: RayPredicate<Frame>,
{
    type Item = Ray<Frame>;
    fn next(&mut self) -> Option<Self::Item> {
        self.iter.find(|ray| self.pred.eval(&ray))
    }
}

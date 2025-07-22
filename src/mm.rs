#[derive(Debug, PartialEq)]
pub struct Measurement {
    pixel_location: (u32, u32),
    aop: f64,
    dop: f64,
}

impl Measurement {
    pub fn new(pixel_location: (u32, u32), aop: f64, dop: f64) -> Self {
        Self {
            pixel_location,
            aop,
            dop,
        }
    }

    pub fn with_dop_max(mut self, max: f64) -> Self {
        self.dop = self.dop.clamp(0.0, max);
        self
    }

    pub fn get_pixel_location(&self) -> &(u32, u32) {
        &self.pixel_location
    }

    pub fn get_aop(&self) -> &f64 {
        &self.aop
    }

    pub fn get_dop(&self) -> &f64 {
        &self.dop
    }
}

// struct Measurements
// - impl Iterator for Measurements
// - type Item = Measurement
//
// - fn filter<I: Iterator<Item = Measurement, F: MeasurementFilter>(self, filter: F) -> Filter<I, F>
// - fn estimates<Et, I: Iterator<Item = Et>, Er: Estimator<Estimate = Et>>(self, estimator: Er) -> Estimates<Et, I, Er>

// struct Filter<I, F>
// - iter: I
// - mm_filter: F
//
// - impl<I, F> Iterator for Filter<I, F> where I: Iterator<Item = Measurement>, F:MeasurementFilter
// - type Item = I::Item
// - fn next(&mut self) -> Option<Self::Item> calls self.iter.find(|mm| self.mm_filter.filter(mm))

// struct Estimates<Et, I, Er>
// - iter: I
// - estimator: Er
// - _phantom: Et
//
// - impl<Et, I, Er> Iterator for Estimates<Et, I, Er> where I: Iterator<Item = Et>, Er: Estimator<Estimate = Et>
// - type Item = I::Item
// - fn next(&mut self) -> Option<Self::Item> calls self.estimator.estimate(self.iter.next()?)

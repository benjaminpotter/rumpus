// trait Estimator
// - type Estimate;
// - fn estimate(mms: &Vec<Measurement>) -> Self::Estimate

// Hough Transform
// - type Yaw = f64;
// - impl Estimator<Estimate = Yaw>

// Pattern Matching
// - impl Estimator<Estimate = Pose>
//
// Configurations
// - Search algorithm
//   - stochastic
//   - seeded
// - Search boundaries
//   - root params
//   - resolution
//   - defines the search space
// - Loss function
//
// Inputs:
// - Series of AoP and DoP measurements
// - Must be in same "format" as the simulation
//   - Same reference frame
// - Include known state with the measurement
//
// Outputs:
// - Estimated pose

// Example Usage
//
// while let Some(image) = iter_images()
//   let estimate = image
//      .rays()
//      .filter(DopFilter::new())
//      .filter(CircleFilter::new())
//      .estimates(Estimator::new())
//      .collect();
//
//
//
// struct Measurements
// - impl Iterator for Measurements
// - type Item = Measurement
//
// - fn filter<I: Iterator<Item = Measurement, F: MeasurementFilter>(self, filter: F) -> Filter<I, F>
// - fn estimates<Et, I: Iterator<Item = Et>, Er: Estimator<Estimate = Et>>(self, estimator: Er) -> Estimates<Et, I, Er>

// struct Estimates<Et, I, Er>
// - iter: I
// - estimator: Er
// - _phantom: Et
//
// - impl<Et, I, Er> Iterator for Estimates<Et, I, Er> where I: Iterator<Item = Et>, Er: Estimator<Estimate = Et>
// - type Item = I::Item
// - fn next(&mut self) -> Option<Self::Item> calls self.estimator.estimate(self.iter.next()?)

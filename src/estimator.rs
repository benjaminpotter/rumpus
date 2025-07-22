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
//      .into_measurements()
//      .filter(DopFilter::new())
//      .filter(CircleFilter::new())
//      .estimates(Estimator::new())
//      .collect();
//
//

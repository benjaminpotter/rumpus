// trait MeasurementFilter
//   - fn filter(&self, mm: &Measurement) -> bool;
//
// struct AoPFilter
//   - threshold
//   - impl MeasurementFilter
// - Includes the measurement if inside range
// - Used for hough and pattern match
//
// struct DoPFilter
//   - min
//   - impl MeasurementFilter
// - Includes the measurement if greather than min
//
// struct CircleFilter
//   - radius
//   - impl MeasurementFilter
// - Includes the measurement if inside circle

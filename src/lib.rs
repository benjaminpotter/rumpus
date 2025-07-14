// #![warn(missing_docs)]

//! Skylight Polarization Utilities

#[allow(missing_docs)]
pub mod error;
pub mod image;
pub mod sensor;

#[cfg(test)]
mod tests {
    use crate::sensor::*;

    // TODO: How do we test something like this?

    #[test]
    fn test_center_pixel() {
        let (_params, sensor) = make_sensor();
        let pixel = (1224, 1024);
        let (_aop, _dop) = sensor.simulate_pixel(&pixel);
    }

    #[test]
    fn test_image() {
        let (params, sensor) = make_sensor();
        let pixels: Vec<(u32, u32)> = (0..params.sensor_size_px.1)
            .into_iter()
            .map(|row| {
                (0..params.sensor_size_px.0)
                    .into_iter()
                    .map(move |col| (col, row))
            })
            .flatten()
            .collect();

        let _simulated_image = sensor.par_simulate_pixels(&pixels);
    }

    fn make_sensor() -> (SensorParams, Sensor) {
        let params = SensorParams::default();
        (params, Sensor::from(&params))
    }
}

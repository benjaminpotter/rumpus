pub mod sensor;

#[cfg(test)]
mod tests {
    use crate::sensor::*;

    // TODO: How do we test something like this?

    #[test]
    fn test_center_pixel() {
        let (_params, sensor) = make_sensor();
        let (_aop, _dop) = sensor.simulate_pixel((1224, 1024));
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

        let _simulated_image = sensor.par_simulate_pixels(pixels);
    }

    fn make_sensor() -> (SensorParams, Sensor) {
        let params = SensorParams {
            pixel_size_um: (3.45, 3.45),
            sensor_size_px: (2448, 2048),
            focal_length_mm: 3.5,
            solar_pose_deg: (78.9, -65.2, 278.3),
        };

        (params.clone(), Sensor::from(params))
    }
}

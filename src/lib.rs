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

    #[test]
    fn test_solar_azimuth_estimation() {
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

        let simulated_image = sensor.par_simulate_pixels(&pixels);

        let aop_threshold_deg = 0.2;
        let estimate_resolution_deg = 0.1;
        let acc_size = (180. / estimate_resolution_deg) as usize;
        let mut accumulator: Vec<u32> = vec![0; acc_size];
        let image_center_px = (
            params.sensor_size_px.0 as f32 / 2.,
            params.sensor_size_px.1 as f32 / 2.,
        );

        for (i, (x, y)) in pixels
            .iter()
            .map(|(col, row)| (*col as f32, *row as f32))
            .map(|(col, row)| (col - image_center_px.0, -(row - image_center_px.1)))
            .enumerate()
        {
            let (aop, _) = simulated_image[i];
            let aop_offset_deg = (aop - 90.).abs();
            if aop_offset_deg < aop_threshold_deg {
                // On the range [-90.0, 90.0]
                let azimuth_vote = (y / x).atan();

                // Map vote to range [0, acc_size].
                let acc_idx = ((azimuth_vote + 90.) / estimate_resolution_deg).floor() as usize;
                accumulator[acc_idx] += 1;
            }
        }

        let mut azimuth_estimate_idx = 0;
        for (i, votes) in accumulator.iter().enumerate().skip(1) {
            if accumulator[azimuth_estimate_idx] < *votes {
                azimuth_estimate_idx = i;
            }
        }

        // Map [0, acc_size] index to [-90.0, 90.0] estimate.
        let azimuth_estimate_deg = ((azimuth_estimate_idx as f32) * estimate_resolution_deg) - 90.;
        let azimuth_deg = 90.;
        let azimuth_estimate_err = (azimuth_estimate_deg - azimuth_deg).abs();

        println!(
            "estimate={}, err={}",
            azimuth_estimate_deg, azimuth_estimate_err
        );
        assert!(azimuth_estimate_err <= estimate_resolution_deg);
    }

    fn make_sensor() -> (SensorParams, Sensor) {
        let params = SensorParams {
            pixel_size_um: (3.45, 3.45),
            sensor_size_px: (2448, 2048),
            focal_length_mm: 3.5,
            solar_pose_deg: (0., 90., 180.),
        };

        (params.clone(), Sensor::from(params))
    }
}

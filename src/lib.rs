// #![warn(missing_docs)]

//! Skylight Polarization Utilities

#[allow(missing_docs)]
pub mod error;

pub mod image;
pub mod sensor;

use crate::error::Error;
use std::ops::RangeInclusive;

struct Accumulator {
    resolution: f64,
    range: RangeInclusive<f64>,
    buffer: Vec<u32>,
}

impl Accumulator {
    fn new(resolution: f64, range: RangeInclusive<f64>) -> Self {
        let len = Accumulator::len_from_params(resolution, &range);
        let buffer = vec![0; len];

        Self {
            resolution,
            range,
            buffer,
        }
    }

    fn len_from_params(resolution: f64, range: &RangeInclusive<f64>) -> usize {
        ((range.end() - range.start()) / resolution) as usize
    }

    fn value_to_index(&self, value: f64) -> usize {
        ((value - self.range.start()) / self.resolution).floor() as usize
    }

    fn index_to_value(&self, index: usize) -> f64 {
        index as f64 * self.resolution + self.range.start()
    }

    fn vote(&mut self, value: f64) {
        let index = self.value_to_index(value);
        self.buffer[index] += 1;
    }

    fn into_winner(self) -> f64 {
        let (index, _) = self
            .buffer
            .iter()
            .enumerate()
            .max_by_key(|&(_, count)| count)
            .unwrap();

        self.index_to_value(index)
    }
}

#[cfg(test)]
mod tests {
    use crate::{sensor::*, Accumulator};
    use chrono::Utc;
    use rayon::prelude::*;

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
        let image_center_px = (
            params.sensor_size_px.0 as f64 / 2.,
            params.sensor_size_px.1 as f64 / 2.,
        );

        let voters: Vec<(usize, &(u32, u32))> = pixels
            .par_iter()
            .enumerate()
            // Apply binary threshold to select pixels close to +/- 90 deg.
            .filter(|(i, _)| {
                let (aop, _) = simulated_image[*i];
                let aop_offset_deg = (aop.abs() - 90.).abs();
                aop_offset_deg < aop_threshold_deg
            })
            .collect();

        // let size = params.num_pixels();
        // let mut bytes: Vec<u8> = vec![0u8; size];
        // voters.iter().for_each(|(i, _)| bytes[*i] = 255u8);
        // let _ = image::save_buffer(
        //     "voters.png",
        //     &bytes,
        //     params.sensor_size_px.0,
        //     params.sensor_size_px.1,
        //     image::ExtendedColorType::L8,
        // );

        let votes: Vec<f64> = voters
            .par_iter()
            .map(|(_, loc)| loc)
            // Map pixel locations to have origin at optical center.
            .map(|(col, row)| {
                (
                    *col as f64 - image_center_px.0,
                    (*row as f64 - image_center_px.1) * -1.,
                )
            })
            // Map pixel location to an azimuth angle.
            // On the range [-90.0, 90.0].
            .map(|(x, y)| (y / x).atan().to_degrees())
            .collect();

        let estimate_resolution_deg = 0.1;
        let mut acc = Accumulator::new(estimate_resolution_deg, -90.0..=90.0);
        votes
            .into_iter()
            // Record vote in accumulator.
            .for_each(|azimuth| acc.vote(azimuth));

        let azimuth_estimate_deg = acc.into_winner();
        println!("estimate={}", azimuth_estimate_deg);

        assert!((azimuth_estimate_deg - 15.4).abs() < 0.1);
    }

    fn make_sensor() -> (SensorParams, Sensor) {
        let params = SensorParams::default();
        (params.clone(), Sensor::from(params))
    }
}

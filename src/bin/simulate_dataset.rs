use chrono::{TimeZone, Utc};
use rayon::prelude::*;
use rumpus::{image::to_rgb, sensor::*};
use std::{path::Path, u8};

fn main() {
    let sensor_size_px = (2448, 2048);
    let time = Utc
        .with_ymd_and_hms(2024, 8, 10, 19, 33, 0)
        .single()
        .unwrap();

    let pixels: Vec<(u32, u32)> = (0..sensor_size_px.1)
        .into_iter()
        .map(|row| (0..sensor_size_px.0).into_iter().map(move |col| (col, row)))
        .flatten()
        .collect();

    let output_dir = Path::new("assets/img");
    for angle in 160..200 {
        let enu_pose_deg = (0., 0., angle as f64);
        let sensor: Sensor = SensorParams {
            pixel_size_um: (3.45, 3.45),
            sensor_size_px,
            focal_length_mm: 3.5,
            enu_pose_deg,
            // RMC Parade Square
            lat: 44.13474,
            lon: -76.28020,
            time,
        }
        .into();

        let simulated_image = sensor.par_simulate_pixels(&pixels);
        let size_bytes = (sensor_size_px.0 * sensor_size_px.1 * 3) as usize;
        let mut bytes: Vec<u8> = simulated_image
            .par_iter()
            .map(|(aop, _)| to_rgb(*aop, -90., 90.).unwrap())
            .flatten()
            .collect();

        let output_path = output_dir.join(format!("aop_{:03}.png", angle));
        let _ = image::save_buffer(
            output_path,
            &bytes,
            sensor_size_px.0,
            sensor_size_px.1,
            image::ExtendedColorType::Rgb8,
        );
        println!("Write angle {} to image", angle);
    }
}

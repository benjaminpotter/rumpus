use chrono::{TimeZone, Utc};
use rayon::prelude::*;
use rumpus::sensor::*;
use std::{path::Path, u8};

/// Map an f64 on the interval [x_min, x_max] to an RGB color.
fn to_rgb(x: f64, x_min: f64, x_max: f64) -> Option<(u8, u8, u8)> {
    if x < x_min || x > x_max {
        return None;
    }

    let interval_width = x_max - x_min;
    let x_norm = ((x - x_min) / interval_width * 255.).floor() as u8;

    let r = vec![
        255,
        x_norm
            .checked_sub(96)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
        255 - x_norm
            .checked_sub(224)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
    ]
    .into_iter()
    .min()
    .unwrap();

    let g = vec![
        255,
        x_norm
            .checked_sub(32)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
        255 - x_norm
            .checked_sub(160)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
    ]
    .into_iter()
    .min()
    .unwrap();

    let b = vec![
        255,
        x_norm
            .checked_add(127)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
        255 - x_norm
            .checked_sub(96)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
    ]
    .into_iter()
    .min()
    .unwrap();

    Some((r, g, b))
}

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
    for angle in 0..10 {
        let enu_pose_deg = (0., 0., angle as f64);
        let sensor: Sensor = SensorParams {
            pixel_size_um: (3.45, 3.45),
            sensor_size_px,
            focal_length_mm: 3.5,
            enu_pose_deg,
            lat: 44.2187,
            lon: -76.4747,
            time,
        }
        .into();

        let simulated_image = sensor.par_simulate_pixels(&pixels);
        let size_bytes = (sensor_size_px.0 * sensor_size_px.1 * 3) as usize;
        let mut bytes: Vec<u8> = vec![0; size_bytes];
        for (i, (r, g, b)) in simulated_image
            .par_iter()
            .map(|(aop, _)| to_rgb(*aop, -90., 90.).unwrap())
            .collect::<Vec<(u8, u8, u8)>>()
            .into_iter()
            .enumerate()
        {
            bytes[i * 3 + 0] = r;
            bytes[i * 3 + 1] = g;
            bytes[i * 3 + 2] = b;
        }

        let output_path = output_dir.join(format!("aop_{:03}.png", angle));
        let _ = image::save_buffer(
            output_path,
            &bytes,
            sensor_size_px.0,
            sensor_size_px.1,
            image::ExtendedColorType::Rgb8,
        );
    }
}

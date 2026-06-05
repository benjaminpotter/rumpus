//! Show different camera models.
//!

use std::f64::consts::TAU;

use rumpus::optic::{Camera, Optic, PinholeOptic, PixelCoordinate, RayDirection};
use uom::si::{
    angle::radian,
    f64::Length,
    length::{micron, millimeter},
    ratio::ratio,
};

fn main() -> anyhow::Result<()> {
    let focal_length = Length::new::<millimeter>(8.0);
    let cam_pinhole = camera_with_optic(PinholeOptic::from_focal_length(focal_length));
    let pixels = pixels_of_interest();

    let rays = pixels
        .iter()
        .map(|pixel| cam_pinhole.trace_from_pixel(pixel))
        .collect::<Vec<_>>();

    println!("model, row, col, polar, azimuth, x, y");
    for i in 0..pixels.len() {
        if let Some(ray) = rays[i] {
            let row = pixels[i].row();
            let col = pixels[i].col();
            let polar = ray.polar().get::<radian>();
            let azimuth = ray.azimuth().get::<radian>();
            let (x, y) = intersect(ray);

            println!("pinhole, {row}, {col}, {polar}, {azimuth}, {x}, {y}");
        }
    }

    Ok(())
}

fn camera_with_optic<O: Optic>(optic: O) -> Camera<O> {
    Camera::new(optic, Length::new::<micron>(6.9), 1024, 1224)
}

fn pixels_of_interest() -> Vec<PixelCoordinate> {
    let mut result = Vec::new();
    let row_off = 512.;
    let col_off = 612.;
    let samples_per_rev = 50;
    for r in vec![10., 100., 500.] {
        for sample in 0..samples_per_rev {
            let norm_sample = sample as f64 / samples_per_rev as f64;
            let angle = TAU * norm_sample;
            let row = (angle.sin() * r).round() + row_off;
            let col = (angle.cos() * r).round() + col_off;

            result.push(PixelCoordinate::new(row as usize, col as usize));
        }
    }

    result
}

fn intersect(ray: RayDirection) -> (f64, f64) {
    let x = ray.polar().sin() * ray.azimuth().cos();
    let y = ray.polar().sin() * ray.azimuth().sin();

    (x.get::<ratio>(), y.get::<ratio>())
}

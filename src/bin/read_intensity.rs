use clap::Parser;
use image::ImageReader;
use rayon::prelude::*;
use rumpus::image::{IntensityImage, StokesReferenceFrame};

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    path: String,

    #[arg(short, long, default_value_t = 0.5)]
    dop_max: f64,
}

fn main() {
    let args = Args::parse();

    let image = ImageReader::open(args.path)
        .unwrap()
        .decode()
        .unwrap()
        .into_luma8();

    let (width, height) = image.dimensions();
    let intensity_image = IntensityImage::from_bytes(width, height, &image.into_raw()).unwrap();

    let stokes_image = intensity_image
        .into_stokes_image()
        .par_transform_frame(StokesReferenceFrame::Pixel);

    let dop_bytes: Vec<u8> = stokes_image
        .par_compute_dop_image(args.dop_max)
        .par_iter()
        .map(|dop| to_rgb(*dop, 0., args.dop_max).unwrap())
        .flatten()
        .collect();

    let aop_bytes: Vec<u8> = stokes_image
        .par_compute_aop_image()
        .par_iter()
        .map(|aop| to_rgb(*aop, -90., 90.).unwrap())
        .flatten()
        .collect();

    let dims = stokes_image.dimensions();
    let _ = image::save_buffer(
        "dop.png",
        &dop_bytes,
        dims.0,
        dims.1,
        image::ExtendedColorType::Rgb8,
    );

    let _ = image::save_buffer(
        "aop.png",
        &aop_bytes,
        dims.0,
        dims.1,
        image::ExtendedColorType::Rgb8,
    );
}

/// Map an f64 on the interval [x_min, x_max] to an RGB color.
fn to_rgb(x: f64, x_min: f64, x_max: f64) -> Option<[u8; 3]> {
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

    Some([r, g, b])
}

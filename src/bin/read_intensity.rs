use clap::{Parser, ValueEnum};
use image::ImageReader;
use rayon::prelude::*;
use rumpus::image::{to_rgb, IntensityImage, StokesReferenceFrame};
use std::path::PathBuf;

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    /// Path to the input image.
    image: PathBuf,

    #[arg(long, default_value_t = 0.5)]
    dop_max: f64,

    #[arg(value_enum)]
    frame: ReferenceFrame,
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, ValueEnum)]
enum ReferenceFrame {
    /// Incident light reference frame.
    Pixel,

    /// Sensor reference frame.
    Sensor,
}

fn main() {
    let args = Args::parse();

    let image = ImageReader::open(args.image)
        .unwrap()
        .decode()
        .unwrap()
        .into_luma8();

    let (width, height) = image.dimensions();
    let frame = match args.frame {
        ReferenceFrame::Pixel => StokesReferenceFrame::Pixel,
        ReferenceFrame::Sensor => StokesReferenceFrame::Sensor,
    };
    let stokes_image = IntensityImage::from_bytes(width, height, &image.into_raw())
        .unwrap()
        .into_stokes_image()
        .par_transform_frame(frame);

    let mms = stokes_image.into_measurements();

    let aop_bytes: Vec<u8> = mms
        .as_slice()
        .par_iter()
        .map(|mm| to_rgb(mm.aop, -90., 90.).unwrap())
        .flatten()
        .collect();

    let dop_bytes: Vec<u8> = mms
        .as_slice()
        .par_iter()
        .map(|mm| mm.dop.clamp(0., args.dop_max))
        .map(|dop| to_rgb(dop, 0., args.dop_max).unwrap())
        .flatten()
        .collect();

    let (width, height) = stokes_image.dimensions();
    let _ = image::save_buffer(
        "aop.png",
        &aop_bytes,
        width,
        height,
        image::ExtendedColorType::Rgb8,
    );

    let _ = image::save_buffer(
        "dop.png",
        &dop_bytes,
        width,
        height,
        image::ExtendedColorType::Rgb8,
    );
}

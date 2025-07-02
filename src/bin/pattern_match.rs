use clap::Parser;
use image::ImageReader;
use rayon::prelude::*;
use rumpus::{
    image::{IntensityImage, StokesReferenceFrame},
    sensor::*,
};
use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::PathBuf;

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(long)]
    image: PathBuf,

    #[arg(long)]
    timestamp: u64,

    #[arg(long)]
    sequence_number: u64,

    #[arg(long)]
    root_params: PathBuf,

    #[arg(short, long)]
    output: PathBuf,

    #[arg(long, default_value_t = 0.1)]
    dop_min: f64,

    #[arg(long, default_value_t = 0.5)]
    dop_max: f64,

    #[arg(long, default_value_t = 0.05)]
    yaw_resolution: f64,
}

fn main() {
    let args = Args::parse();

    let image = ImageReader::open(&args.image)
        .unwrap()
        .decode()
        .unwrap()
        .into_luma8();

    let (width, height) = image.dimensions();
    let intensity_image = IntensityImage::from_bytes(width, height, &image.into_raw()).unwrap();

    let (aop_image, dop_image) = intensity_image
        .into_stokes_image()
        .par_transform_frame(StokesReferenceFrame::Pixel)
        .into_aop_dop_image();

    // Read sensor parameters from config file.
    let mut file = File::open(&args.root_params).unwrap();
    let mut serialized = String::new();
    file.read_to_string(&mut serialized).unwrap();
    let root_params: SensorParams = serde_json::from_str(&serialized).unwrap();

    let count = 360.0 / args.yaw_resolution;

    let yaws: Vec<f64> = (0..count as usize)
        .map(|x| x as f64 / count * 360.0)
        .collect();

    let sensors: Vec<Sensor> = yaws
        .iter()
        .map(|yaw| root_params.to_pose((0., 0., *yaw)))
        .map(|params| Sensor::from(params))
        .collect();

    let pixels = root_params.pixels();

    let pattern: Vec<((u32, u32), f64, f64)> = pixels
        .into_iter()
        .zip(aop_image.into_vec().into_iter())
        .zip(dop_image.into_vec().into_iter())
        .map(|((pixel, aop), dop)| (pixel, aop, dop))
        // Remove pixels with DoP below threshold.
        .filter(|(_, _, dop)| *dop > args.dop_min)
        .map(|(pixel, aop, dop)| (pixel, aop, 1. / dop))
        .collect();

    let scores: Vec<f64> = sensors
        .par_iter()
        .map(|sensor| {
            pattern
                .par_iter()
                .map(|(pixel, aop, weight)| {
                    let (s_aop, _) = sensor.simulate_pixel(pixel);
                    (aop - s_aop).powf(2.) * weight
                })
                .sum()
        })
        .collect();

    let (index, _) = scores
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .unwrap();

    let mut output_file = BufWriter::new(File::create(&args.output).unwrap());

    // Write header.
    let _ = writeln!(
        output_file,
        "image_file_stem,timestamp,sequence_number,yaw,score"
    );

    let image_file_stem = &args.image.file_stem().unwrap().to_str().unwrap();

    // Write data.
    let _ = writeln!(
        output_file,
        "{},{},{},{:010.2},{:020.5}",
        image_file_stem, &args.timestamp, &args.sequence_number, yaws[index], scores[index]
    );
}

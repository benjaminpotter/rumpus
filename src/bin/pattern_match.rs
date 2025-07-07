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
    angle_resolution: f64,
}

fn linspace(start: f64, stop: f64, resolution: f64) -> Vec<f64> {
    let size = stop - start;
    let count = size / resolution;
    (0..count as usize)
        .map(|x| x as f64)
        .map(|x| x * size / count + start)
        .collect()
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

    let yaws: Vec<f64> = linspace(0., 360., args.angle_resolution);
    let pitches: Vec<f64> = linspace(-15.0, 15.0, args.angle_resolution);
    let rolls: Vec<f64> = linspace(-15.0, 15.0, args.angle_resolution);

    let mut sensor_params: Vec<SensorParams> = Vec::new();
    for i in 0..yaws.len() {
        for j in 0..pitches.len() {
            for k in 0..rolls.len() {
                let pose = (rolls[k], pitches[j], yaws[i]);
                sensor_params.push(root_params.to_pose(pose));
            }
        }
    }

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

    let losses: Vec<f64> = sensor_params
        .par_iter()
        .map(|sensor_params| {
            let sensor: Sensor = sensor_params.into();
            pattern
                .par_iter()
                .map(|(pixel, aop, weight)| {
                    let (s_aop, _) = sensor.simulate_pixel(pixel);
                    let mut diff = aop - s_aop;
                    if diff < -90. {
                        diff += 180.;
                    } else if diff > 90. {
                        diff -= 180.;
                    }
                    diff.powf(2.) * weight
                })
                .sum()
        })
        .collect();

    let (index, _) = losses
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .unwrap();

    // The sensor params that are closest to the real camera.
    let pose = sensor_params[index].enu_pose_deg;

    let mut output_file = BufWriter::new(File::create(&args.output).unwrap());

    // Write header.
    let _ = writeln!(
        output_file,
        "image_file_stem,timestamp,sequence_number,roll,pitch,yaw,score"
    );

    let image_file_stem = &args.image.file_stem().unwrap().to_str().unwrap();

    // Write data.
    let _ = writeln!(
        output_file,
        "{},{},{},{:010.2},{:010.2},{:010.2},{:020.5}",
        image_file_stem,
        &args.timestamp,
        &args.sequence_number,
        pose.0,
        pose.1,
        pose.2,
        losses[index]
    );
}

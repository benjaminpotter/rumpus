use clap::Parser;
use image::ImageReader;
use rayon::prelude::*;
use rumpus::{
    image::{IntensityImage, StokesReferenceFrame},
    sensor::*,
};
use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::ops::{Bound, RangeBounds};
use std::path::PathBuf;
use tracing::{error, info, warn};

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(long)]
    image: PathBuf,

    #[arg(long)]
    timestamp: Option<u64>,

    #[arg(long)]
    sequence_number: Option<u64>,

    #[arg(long)]
    root_sensor_params: Option<PathBuf>,

    #[arg(short, long)]
    output: Option<PathBuf>,

    #[arg(long)]
    losses: Option<PathBuf>,

    #[arg(long, default_value_t = 0.02)]
    dop_min: f64,

    #[arg(long, default_value_t = 0.5)]
    dop_max: f64,

    #[arg(long, default_value_t = 45.)]
    angle_resolution: f64,
}

fn main() {
    // Register an event subscriber that prints events to STDOUT.
    let subscriber = tracing_subscriber::FmtSubscriber::new();
    tracing::subscriber::set_global_default(subscriber).unwrap();

    // Parse command line arguments.
    let args = Args::parse();

    let raw_image = ImageReader::open(&args.image)
        .unwrap()
        .decode()
        .unwrap()
        .into_luma8();

    // Read sensor parameters from config file.
    let root_sensor_params: SensorParams = match args.root_sensor_params {
        Some(path_buf) => {
            let mut file = File::open(&path_buf).unwrap();
            let mut serialized = String::new();
            file.read_to_string(&mut serialized).unwrap();
            let sensor_params = serde_json::from_str(&serialized).unwrap();

            info!("parsed sensor params from JSON");
            sensor_params
        }
        None => {
            warn!("no sensor params provided");
            SensorParams::default()
        }
    };

    let (width, height) = raw_image.dimensions();
    let losses = pattern_match(
        &raw_image.into_raw(),
        width,
        height,
        args.dop_min,
        args.dop_max,
        args.angle_resolution,
        root_sensor_params,
    );

    if let Some(path) = &args.losses {
        let mut losses_file = BufWriter::new(File::create(&path).unwrap());
        let _ = writeln!(losses_file, "roll,pitch,yaw,loss");
        for (sensor_params, loss) in losses.iter() {
            let _ = writeln!(
                losses_file,
                "{:010.2},{:010.2},{:010.2},{:020.5}",
                sensor_params.enu_pose_deg.0,
                sensor_params.enu_pose_deg.1,
                sensor_params.enu_pose_deg.2,
                loss
            );
        }

        info!(
            path = &path.display().to_string(),
            "wrote all losses to csv"
        );
    }

    let (estimated_sensor_params, estimate_loss) = losses
        .iter()
        .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .unwrap();

    let image_file_stem = &args.image.file_stem().unwrap().to_str().unwrap();
    let timestamp: String = match args.timestamp {
        Some(ts) => format!("{:010.2}", ts),
        None => {
            warn!("no timestamp provided");
            String::new()
        }
    };
    let sequence_number: String = match args.sequence_number {
        Some(sn) => format!("{:010.2}", sn),
        None => {
            warn!("no sequence_number provided");
            String::new()
        }
    };

    let output_bytes = format!(
        "image_file_stem,timestamp,sequence_number,roll,pitch,yaw,loss\n{},{},{},{:010.2},{:010.2},{:010.2},{:020.5}\n",
        image_file_stem,
        timestamp,
        sequence_number,
        estimated_sensor_params.enu_pose_deg.0,
        estimated_sensor_params.enu_pose_deg.1,
        estimated_sensor_params.enu_pose_deg.2,
        estimate_loss,
    );

    let output = args.output.unwrap_or("result.csv".into());
    let mut output_file = BufWriter::new(File::create(&output).unwrap());

    match output_file.write_all(output_bytes.as_bytes()) {
        Ok(()) => info!(path = &output.display().to_string(), "wrote result as csv"),
        Err(e) => error!(err = e.to_string(), "failed to write result as csv"),
    }
}

fn pattern_match(
    bytes: &[u8],
    width: u32,
    height: u32,
    dop_min: f64,
    dop_max: f64,
    angle_resolution: f64,
    root_sensor_params: SensorParams,
) -> Vec<(SensorParams, f64)> {
    let intensity_image = IntensityImage::from_bytes(width, height, bytes).unwrap();
    let (aop_image, dop_image) = intensity_image
        .into_stokes_image()
        .par_transform_frame(StokesReferenceFrame::Pixel)
        .into_aop_dop_image();
    let (aop_width, aop_height) = aop_image.dimensions();

    let yaws: Vec<f64> = linspace(0.0..360.0, angle_resolution);
    let pitches: Vec<f64> = linspace(-15.0..=15.0, angle_resolution);
    let rolls: Vec<f64> = linspace(-15.0..=15.0, angle_resolution);

    let mut sensor_params: Vec<SensorParams> = Vec::new();
    for i in 0..yaws.len() {
        for j in 0..pitches.len() {
            for k in 0..rolls.len() {
                let pose = (rolls[k], pitches[j], yaws[i]);
                sensor_params.push(root_sensor_params.to_pose(pose));
            }
        }
    }
    info!("created {} simulated sensors", sensor_params.len());

    // Filter AoP by DoP to create pattern.
    let pixels = root_sensor_params.pixels();
    let pattern: Vec<((u32, u32), f64, f64)> = pixels
        .into_iter()
        .zip(aop_image.into_vec().into_iter())
        .zip(
            dop_image
                .into_vec()
                .into_iter()
                // Clamp DoP to interval [dop_min, dop_max].
                .map(|dop| dop.clamp(dop_min, dop_max)),
        )
        .map(|((pixel, aop), dop)| (pixel, aop, dop))
        // Remove pixels with DoP below threshold.
        .filter(|(_, _, dop)| *dop > dop_min)
        .map(|(pixel, aop, dop)| (pixel, aop, dop_max / dop))
        .collect();

    let num_pixels = pattern.len() as f64;
    let total_pixels = (aop_width * aop_height) as f64;
    let percent_extracted = num_pixels / total_pixels;
    info!(
        "extracted {} pixels ({:0.2}%) from the measured image",
        num_pixels, percent_extracted
    );

    sensor_params
        .into_par_iter()
        .map(|sensor_params| {
            let sensor: Sensor = (&sensor_params).into();
            let loss = pattern
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
                .sum::<f64>()
                / num_pixels;

            (sensor_params, loss)
        })
        .collect()
}

/// Constructs a vector of f64 between on the Range provided.
fn linspace<T>(range: T, resolution: f64) -> Vec<f64>
where
    T: RangeBounds<f64>,
{
    let start = match range.start_bound() {
        Bound::Included(&x) => x,
        Bound::Excluded(&x) => x + resolution,
        // TODO: Should return an error.
        Bound::Unbounded => return vec![],
    };

    let end = match range.end_bound() {
        Bound::Included(&x) => x,
        Bound::Excluded(&x) => x - resolution,
        // TODO: Should return an error.
        Bound::Unbounded => return vec![],
    };

    if start >= end {
        // TODO: Should return an error.
        return vec![];
    }

    let range_size = end - start;
    if resolution > range_size {
        return vec![start + range_size / 2.];
    }

    let mut result = Vec::new();
    let mut current = start;

    while current <= end {
        result.push(current);
        current += resolution;
    }

    result
}

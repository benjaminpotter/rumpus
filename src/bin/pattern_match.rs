use clap::Parser;
use image::ImageReader;
use rayon::prelude::*;
use rumpus::{
    image::{IntensityImage, Measurement, StokesReferenceFrame},
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

    #[arg(long, default_value_t = 0.1)]
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

    let raw_image = ImageReader::open(&args.image)
        .unwrap()
        .decode()
        .unwrap()
        .into_luma8();

    let (width, height) = raw_image.dimensions();
    let mms: Vec<_> = IntensityImage::from_bytes(width, height, &raw_image.as_raw())
        .unwrap()
        .into_stokes_image()
        .par_transform_frame(StokesReferenceFrame::Pixel)
        .into_measurements()
        .into_iter()
        // Remove pixels with low DoP value.
        .filter(|mm| mm.dop > args.dop_min)
        .map(|mm| mm.with_dop_max(args.dop_max))
        .collect();

    info!("selected {} stokes vectors", mms.len());
    let (estimated_sensor_params, estimate_loss) =
        search(mms, args.angle_resolution, root_sensor_params);

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
        estimated_sensor_params.pose.roll,
        estimated_sensor_params.pose.pitch,
        estimated_sensor_params.pose.yaw,
        estimate_loss,
    );

    let output = args.output.unwrap_or("result.csv".into());
    let mut output_file = BufWriter::new(File::create(&output).unwrap());

    match output_file.write_all(output_bytes.as_bytes()) {
        Ok(()) => info!(path = &output.display().to_string(), "wrote result as csv"),
        Err(e) => error!(err = e.to_string(), "failed to write result as csv"),
    }
}

fn search(
    mms: Vec<Measurement>,
    angle_resolution: f64,
    root_sensor_params: SensorParams,
) -> (SensorParams, f64) {
    let yaws: Vec<f64> = linspace(0.0..360.0, angle_resolution);
    let pitches: Vec<f64> = linspace(-15.0..=15.0, angle_resolution);
    let rolls: Vec<f64> = linspace(-15.0..=15.0, angle_resolution);

    let mut estimate: Option<(SensorParams, f64)> = None;
    for i in 0..yaws.len() {
        for j in 0..pitches.len() {
            for k in 0..rolls.len() {
                let sensor_params = root_sensor_params
                    .clone()
                    .with_pose((rolls[k], pitches[j], yaws[i]).into());

                let sensor: Sensor = (&sensor_params).into();
                let loss = compute_loss(sensor, &mms);

                match estimate {
                    Some((_, min_loss)) => {
                        if min_loss > loss {
                            estimate = Some((sensor_params, loss));
                        }
                    }
                    None => {
                        estimate = Some((sensor_params, loss));
                    }
                }
            }
        }
    }

    estimate.unwrap()
}

fn compute_loss(sensor: Sensor, mms: &Vec<Measurement>) -> f64 {
    mms.par_iter()
        .map(|mm| {
            // TODO: I want to compare Measurements using different loss functions
            // TODO: Provide implementation on Measurement structure
            // TODO: Make Sensor return Measurements rather than (f64, f64)s
            let (sim, _) = sensor.simulate_pixel(&mm.pixel_location);
            let mut diff = mm.aop - sim;
            if diff < -90. {
                diff += 180.;
            } else if diff > 90. {
                diff -= 180.;
            }
            diff.powf(2.) / mm.dop
        })
        .sum::<f64>()
        / mms.len() as f64
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

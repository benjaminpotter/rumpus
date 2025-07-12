use clap::Parser;
use image::ImageReader;
use rand::distr::StandardUniform;
use rand::prelude::*;
use rayon::prelude::*;
use rumpus::{
    image::{IntensityImage, Measurement, StokesReferenceFrame},
    sensor::*,
};
use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::ops::{Bound, Range, RangeBounds};
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

    #[arg(long, default_value_t = 15)]
    num_poses: usize,
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

    let ps = poses(args.num_poses, -15.0..15.0, -15.0..15.0, 0.0..360.0);
    info!("generated poses");

    let mut estimate =
        search(&mms, ps, root_sensor_params).expect("requested a zero-sized search space");
    info!("completed search");

    let epochs = 5usize;
    for epoch in 1..epochs {
        info!("epoch {}", epoch);

        let scale = (epochs - epoch) as f64;
        let roll_bound = (estimate.params.pose.roll - scale)..(estimate.params.pose.roll + scale);
        let pitch_bound =
            (estimate.params.pose.pitch - scale)..(estimate.params.pose.pitch + scale);
        let yaw_bound = (estimate.params.pose.yaw - scale)..(estimate.params.pose.yaw + scale);
        let ps = poses(args.num_poses, roll_bound, pitch_bound, yaw_bound);
        info!("generated poses");

        estimate = estimate.min(
            search(&mms, ps, root_sensor_params).expect("requested a zero-sized search space"),
        );
        info!("completed search");
    }

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
        estimate.params.pose.roll,
        estimate.params.pose.pitch,
        estimate.params.pose.yaw,
        estimate.loss,
    );

    let output = args.output.unwrap_or("result.csv".into());
    let mut output_file = BufWriter::new(File::create(&output).unwrap());

    match output_file.write_all(output_bytes.as_bytes()) {
        Ok(()) => info!(path = &output.display().to_string(), "wrote result as csv"),
        Err(e) => error!(err = e.to_string(), "failed to write result as csv"),
    }
}

fn poses(
    num_poses: usize,
    roll_bound: Range<f64>,
    pitch_bound: Range<f64>,
    yaw_bound: Range<f64>,
) -> Box<dyn Iterator<Item = Pose>> {
    Box::new(
        rand::rng()
            .sample_iter(StandardUniform)
            .scan(0usize, move |count, pose: Pose| {
                if *count == num_poses {
                    // Stop generating random poses.
                    return None;
                }

                // If pose is inside bounds, include it.
                if roll_bound.contains(&pose.roll)
                    && pitch_bound.contains(&pose.pitch)
                    && yaw_bound.contains(&pose.yaw)
                {
                    *count += 1;
                    return Some(Some(pose));
                }

                // Otherwise, pose outside of bound, discard it.
                Some(None)
            })
            .filter_map(|pose| pose),
    )
}

struct Estimate {
    params: SensorParams,
    loss: f64,
}

impl Estimate {
    fn min(self, other: Estimate) -> Estimate {
        match self.loss < other.loss {
            true => self,
            false => other,
        }
    }
}

fn search(
    mms: &Vec<Measurement>,
    poses: Box<dyn Iterator<Item = Pose>>,
    root_sensor_params: SensorParams,
) -> Option<Estimate> {
    poses
        .map(|pose| {
            let params = root_sensor_params.clone().with_pose(pose);
            Estimate {
                params,
                loss: compute_loss((&params).into(), &mms),
            }
        })
        .reduce(Estimate::min)
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

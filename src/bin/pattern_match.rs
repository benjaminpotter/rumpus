use clap::Parser;
use image::ImageReader;
use rand::distr::{Distribution, Uniform};
use rayon::prelude::*;
use rumpus::{
    image::{AopImage, IntensityImage, StokesReferenceFrame},
    mm::Measurement,
    sensor::*,
};
use std::{
    fs::File,
    io::{BufWriter, Read, Write},
    path::PathBuf,
};
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
    output_image: Option<PathBuf>,

    #[arg(short, long)]
    output: Option<PathBuf>,

    #[arg(long)]
    root_sensor_params: Option<PathBuf>,

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

    let (raw_width, raw_height) = raw_image.dimensions();
    let stokes_image = IntensityImage::from_bytes(raw_width, raw_height, &raw_image.as_raw())
        .unwrap()
        .into_stokes_image()
        .par_transform_frame(StokesReferenceFrame::Pixel);

    let (width, height) = stokes_image.dimensions();
    if root_sensor_params.sensor_size_px.0 != width || root_sensor_params.sensor_size_px.1 != height
    {
        error!(
            "sensor size mismatch from input image: sensor size was {:?} but image size was {:?}",
            root_sensor_params.sensor_size_px,
            (width, height)
        );
        std::process::exit(1);
    }

    let mms: Vec<_> = stokes_image
        .into_measurements()
        .into_iter()
        // Remove pixels with low DoP value.
        .filter(|mm| mm.dop > args.dop_min)
        .map(|mm| mm.with_dop_max(args.dop_max))
        .collect();

    info!("selected {} stokes vectors", mms.len());

    // Searching the pitch and roll axes is not going to work in the current configuration.
    // The transform into the solar principle plane assumes the optical axis is vertical.
    // For now, just allow yaw.

    let yaw_range = 0.0..360.0;
    let yaw_iter =
        Uniform::try_from(yaw_range).expect("to create random distribution from f64 range");

    let estimate = search(&mms, yaw_iter, args.num_poses, root_sensor_params)
        .expect("requested a zero-sized search space");

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

    if let Some(path_buf) = args.output_image {
        // Simulate full image with estimated sensor.
        let sensor: Sensor = (&estimate.params).into();
        let pixels = estimate.params.pixels();
        let sim_mms: Vec<Measurement> = sensor.par_simulate_pixels(&pixels);

        // Convert measurements into Rgb8 image.
        let sim = AopImage::from_sparse_mms(&sim_mms, width, height).into_inner();
        let msd = AopImage::from_sparse_mms(&mms, width, height).into_inner();

        // Join all image rows to create single double-wide image.
        let mut bytes: Vec<u8> = Vec::new();
        for row in 0..height {
            for col in 0..width {
                let i: usize = (row * width + col).try_into().unwrap();
                bytes.extend_from_slice(&msd[i]);
            }
            for col in 0..width {
                let i: usize = (row * width + col).try_into().unwrap();
                bytes.extend_from_slice(&sim[i]);
            }
        }

        // Write image buffer to disk as PNG.
        let _ = image::save_buffer(
            path_buf,
            &bytes,
            width * 2,
            height,
            image::ExtendedColorType::Rgb8,
        );
    }

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
    yaw_distr: Uniform<f64>,
    num_poses: usize,
    root_sensor_params: SensorParams,
) -> Option<Estimate> {
    let mut rng = rand::rng();
    yaw_distr
        .sample_iter(&mut rng)
        .take(num_poses)
        .map(|yaw: f64| Pose {
            roll: 0.0,
            pitch: 0.0,
            yaw,
        })
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
            let sim_mm = sensor.simulate_pixel(&mm.pixel_location);
            let mut diff = mm.aop - sim_mm.aop;
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

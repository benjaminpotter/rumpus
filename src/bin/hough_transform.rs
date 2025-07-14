use clap::Parser;
use image::ImageReader;
use rayon::prelude::*;
use rumpus::image::{IntensityImage, StokesReferenceFrame};
use std::{
    fs::File,
    io::{BufWriter, Write},
    ops::RangeInclusive,
    path::PathBuf,
};
use tracing::{error, info, warn};

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    /// Path to the input image.
    #[arg(long)]
    image: PathBuf,

    /// Optional path to write output to.
    #[arg(short, long)]
    output: Option<PathBuf>,

    #[arg(long)]
    timestamp: Option<u64>,

    #[arg(long)]
    sequence_number: Option<u64>,

    #[arg(long, default_value_t = 0.2)]
    threshold: f64,

    #[arg(long, default_value_t = 0.1)]
    resolution: f64,

    #[arg(long, default_value_t = 0.1)]
    dop_min: f64,

    #[arg(long, default_value_t = 0.5)]
    dop_max: f64,
}

fn main() {
    // Register an event subscriber that prints events to STDOUT.
    let subscriber = tracing_subscriber::FmtSubscriber::new();
    tracing::subscriber::set_global_default(subscriber).unwrap();

    let args = Args::parse();

    let raw_image = ImageReader::open(&args.image)
        .expect("failed to read input image")
        .decode()
        .expect("failed to decode input image")
        .into_luma8();

    info!("decoded input image");

    let (width, height) = raw_image.dimensions();
    let stokes_image = IntensityImage::from_bytes(width, height, &raw_image.as_raw())
        .expect("failed to parse input image as intensity")
        .into_stokes_image()
        .par_transform_frame(StokesReferenceFrame::Pixel);

    let (width, height) = stokes_image.dimensions();
    let image_center_px = (width as f64 / 2., height as f64 / 2.);
    let votes: Vec<(usize, f64)> = stokes_image
        .into_measurements()
        .into_par_iter()
        .enumerate()
        // Remove pixels with low DoP value.
        .filter(|(_, mm)| mm.dop > args.dop_min)
        .map(|(i, mm)| (i, mm.with_dop_max(args.dop_max)))
        // Apply binary threshold to select pixels close to +/- 90 deg.
        .filter(|(_, mm)| (mm.aop.abs() - 90.).abs() < args.threshold)
        // Map pixel locations to have origin at optical center.
        .map(|(i, mm)| (i, mm.pixel_location))
        .map(|(i, (col, row))| {
            (
                i,
                (
                    col as f64 - image_center_px.0,
                    (row as f64 - image_center_px.1) * -1.,
                ),
            )
        })
        // Map pixel location to an azimuth angle.
        // On the range [-90.0, 90.0].
        .map(|(i, (x, y))| (i, (y / x).atan().to_degrees()))
        .collect();

    let size = (width * height).try_into().unwrap();
    let mut bytes: Vec<u8> = vec![0u8; size];
    votes.iter().for_each(|(i, _)| bytes[*i] = 255u8);
    let _ = image::save_buffer(
        "votes.png",
        &bytes,
        width,
        height,
        image::ExtendedColorType::L8,
    );

    // Record votes in accumulator.
    let mut acc = Accumulator::new(args.resolution, -90.0..=90.0);
    votes.into_iter().for_each(|(_, azimuth)| acc.vote(azimuth));
    let estimate = acc.into_winner();

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
        "image_file_stem,timestamp,sequence_number,roll,pitch,yaw\n{},{},{},{:010.2},{:010.2},{:010.2}\n",
        image_file_stem,
        timestamp,
        sequence_number,
        0.0,
        0.0,
        estimate,
    );

    let mut writer: Box<dyn Write> = match args.output {
        Some(path_buf) => Box::new(BufWriter::new(
            File::create(path_buf).expect("unable to create output file"),
        )),
        None => Box::new(BufWriter::new(std::io::stdout())),
    };

    match writer.write_all(output_bytes.as_bytes()) {
        Ok(()) => info!("wrote result as csv"),
        Err(e) => error!(err = e.to_string(), "failed to write result as csv"),
    }
}

struct Accumulator {
    resolution: f64,
    range: RangeInclusive<f64>,
    buffer: Vec<u32>,
}

impl Accumulator {
    fn new(resolution: f64, range: RangeInclusive<f64>) -> Self {
        let len = Accumulator::len_from_params(resolution, &range);
        let buffer = vec![0; len];

        Self {
            resolution,
            range,
            buffer,
        }
    }

    fn len_from_params(resolution: f64, range: &RangeInclusive<f64>) -> usize {
        ((range.end() - range.start()) / resolution) as usize
    }

    fn value_to_index(&self, value: f64) -> usize {
        ((value - self.range.start()) / self.resolution).floor() as usize
    }

    fn index_to_value(&self, index: usize) -> f64 {
        index as f64 * self.resolution + self.range.start()
    }

    fn vote(&mut self, value: f64) {
        let index = self.value_to_index(value);
        self.buffer[index] += 1;
    }

    fn into_winner(self) -> f64 {
        let (index, _) = self
            .buffer
            .iter()
            .enumerate()
            .max_by_key(|&(_, count)| count)
            .unwrap();

        self.index_to_value(index)
    }
}

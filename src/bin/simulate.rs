use clap::Parser;
use rayon::prelude::*;
use rumpus::{image::AopImage, mm::Measurement, sensor::*};
use std::{
    ffi::OsStr,
    fs::File,
    io::{BufWriter, Read, Write},
    path::PathBuf,
};

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    /// Optional path to JSON formatted SensorParams
    ///
    /// If not provided, the SensorParams::default() will be used instead.
    #[arg(short, long)]
    params: Option<PathBuf>,

    /// File path for the simulated output.
    ///
    /// The output format is inferred from the file extension.
    #[arg(short, long)]
    output: PathBuf,
}

fn main() {
    let args = Args::parse();

    // Construct sensor from parameters.
    let params = match args.params {
        Some(path) => {
            // Read sensor parameters from config file.
            let mut file = File::open(path).unwrap();
            let mut serialized = String::new();
            file.read_to_string(&mut serialized).unwrap();
            serde_json::from_str(&serialized).unwrap()
        }
        None => SensorParams::default(),
    };

    // Simulate AoP and DoP information.
    let sensor = Sensor::from(&params);
    let mms: Vec<Measurement> = params
        .pixels()
        .into_par_iter()
        .map(|pixel_location| {
            let (aop, dop) = sensor.simulate_pixel(&pixel_location);
            Measurement {
                pixel_location,
                aop,
                dop,
            }
        })
        .collect();

    match args
        .output
        .as_path()
        .extension()
        .map(|os_str: &OsStr| os_str.to_str())
        .unwrap()
    {
        Some("png") => {
            println!("writing to PNG image at {}", &args.output.display());

            // Convert to an image buffer.
            let aop_image =
                AopImage::from_sparse_mms(&mms, params.sensor_size_px.0, params.sensor_size_px.1)
                    .into_raw();

            // Write to a PNG file.
            let _ = image::save_buffer(
                &args.output,
                &aop_image,
                params.sensor_size_px.0,
                params.sensor_size_px.1,
                image::ExtendedColorType::Rgb8,
            );
        }
        Some("dat") => {
            println!("writing to DAT file at {}", &args.output.display());

            // Write simulated output to file.
            let mut output_file = BufWriter::new(File::create(&args.output).unwrap());
            for row in 0..params.sensor_size_px.1 {
                for col in 0..params.sensor_size_px.0 {
                    let i: usize = (row * params.sensor_size_px.0 + col).try_into().unwrap();
                    let _ = write!(output_file, "{:5} ", mms[i].aop);
                }
                let _ = write!(output_file, "\n");
            }
        }
        _ => unimplemented!(),
    }
}

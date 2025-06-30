use chrono::Utc;
use clap::Parser;
use rayon::prelude::*;
use rumpus::{image::to_rgb, sensor::*};
use std::{
    ffi::OsStr,
    fs::File,
    io::{BufWriter, Read, Write},
    path::PathBuf,
};

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    params: String,

    #[arg(short, long)]
    output: PathBuf,
}

fn main() {
    let args = Args::parse();

    // Read sensor parameters from config file.
    let mut file = File::open(args.params).unwrap();
    let mut serialized = String::new();
    file.read_to_string(&mut serialized).unwrap();

    // Construct sensor from parameters.
    let params: SensorParams = serde_json::from_str(&serialized).unwrap();
    let sensor = Sensor::from(params);

    // Define pixel range as full sensor.
    let pixels: Vec<(u32, u32)> = (0..params.sensor_size_px.1)
        .into_iter()
        .map(|row| {
            (0..params.sensor_size_px.0)
                .into_iter()
                .map(move |col| (col, row))
        })
        .flatten()
        .collect();

    // Simulate AoP and DoP information.
    let simulated_image = sensor.par_simulate_pixels(&pixels);

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
            let bytes: Vec<u8> = simulated_image
                .par_iter()
                .map(|(aop, _)| to_rgb(*aop, -90., 90.).unwrap())
                .flatten()
                .collect();

            // Write to a PNG file.
            let _ = image::save_buffer(
                &args.output,
                &bytes,
                params.sensor_size_px.0,
                params.sensor_size_px.1,
                image::ExtendedColorType::Rgb8,
            );
        }
        Some("dat") => {
            println!("writing to DAT file at {}", &args.output.display());

            // Write simulated output to file.
            // Each call to write! on the raw file makes a system call.
            // Using BufWriter drastically speeds up the file dump.
            let mut output_file = BufWriter::new(File::create(&args.output).unwrap());

            // Write metadata as a comment.
            let _ = writeln!(output_file, "# Angle of Polarization Datafile");
            let _ = writeln!(output_file, "# generated_at={}", Utc::now().to_rfc3339());
            let _ = writeln!(output_file, "");

            for row in 0..params.sensor_size_px.1 {
                for col in 0..params.sensor_size_px.0 {
                    let idx = (row * params.sensor_size_px.0 + col) as usize;
                    let (aop, _) = simulated_image[idx];
                    let _ = write!(output_file, "{:5} ", aop);
                }
                let _ = write!(output_file, "\n");
            }
        }
        _ => unimplemented!(),
    }
}

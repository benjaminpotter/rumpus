use clap::Parser;
use rumpus::sensor::*;
use std::{
    fs::File,
    io::{BufWriter, Read, Write},
};

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    params: String,

    #[arg(short, long, default_value = "aop.dat")]
    output: String,
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

    // Write simulated output to file.
    // Each call to write! on the raw file makes a system call.
    // Using BufWriter drastically speeds up the file dump.
    let mut output_file = BufWriter::new(File::create(args.output).unwrap());
    for row in 0..params.sensor_size_px.1 {
        for col in 0..params.sensor_size_px.0 {
            let idx = (row * params.sensor_size_px.0 + col) as usize;
            let (aop, _) = simulated_image[idx];
            let _ = write!(output_file, "{:5} ", aop);
        }
        let _ = write!(output_file, "\n");
    }
}

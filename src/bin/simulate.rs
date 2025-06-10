// Serde the sensor parameters.
// Allow passing TOML file via clap to deserialize the sensor parameters.
// Simulate images using loaded sensor parameters
// Save images as csv files
// Use gnuplot to turn the csv files into nice heatmaps.
// For later: Save images as pngs with heatmapped stuff??

use clap::Parser;
use rumpus::{Sensor, SensorParams};
use std::{fs::File, io::Read};

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    params_path: String,
}

fn main() {
    let args = Args::parse();

    // Read sensor parameters from config file.
    let mut file = File::open(args.params_path).unwrap();
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
    let simulated_image = sensor.par_simulate_pixels(pixels);

    // Dump simulation to STDOUT.
    for row in 0..params.sensor_size_px.1 {
        for col in 0..params.sensor_size_px.0 {
            let idx = (row * params.sensor_size_px.0 + col) as usize;
            let (aop, _) = simulated_image[idx];
            print!("{:5} ", aop);
        }
        println!("");
    }
}

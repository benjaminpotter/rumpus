use clap::Parser;
use image::{GrayImage, ImageReader};
use rayon::prelude::*;

/// Pattern of linear polarizers on the CCD.
///
/// +-----+-----+-----
/// | 090 | 135 | 090
/// +-----+-----+-----
/// | 045 | 000 | 045
/// +-----+-----+-----
/// | 090 | 135 | ...

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    path: String,
}

struct Surface {
    width: u32,
    height: u32,
    samples: Vec<f64>,
}

impl Surface {
    fn from_samples(width: u32, height: u32, samples: Vec<f64>) -> Self {
        Self {
            width,
            height,
            samples,
        }
    }

    fn sample(&self, mut x: u32, mut y: u32) -> f64 {
        if x >= self.width {
            x = self.width - 1;
        }

        if y >= self.height {
            y = self.height - 1;
        }

        let i = (y * self.width + x) as usize;
        self.samples[i]
    }

    fn interp(&self, x: f64, y: f64) -> f64 {
        let x1 = x.floor();
        let x2 = x.ceil();
        let y1 = y.floor();
        let y2 = y.ceil();

        let w11 = (x2 - x) * (y2 - y);
        let w12 = (x2 - x) * (y - y1);
        let w21 = (x - x1) * (y2 - y);
        let w22 = (x - x1) * (y - y1);

        let z11 = self.sample(x1 as u32, y1 as u32);
        let z12 = self.sample(x1 as u32, y2 as u32);
        let z21 = self.sample(x2 as u32, y1 as u32);
        let z22 = self.sample(x2 as u32, y2 as u32);

        w11 * z11 + w12 * z12 + w21 * z21 + w22 * z22
    }
}

fn main() {
    let args = Args::parse();

    let image = ImageReader::open(args.path)
        .unwrap()
        .decode()
        .unwrap()
        .into_luma8();

    // Verify image dimensions.
    if image.dimensions() != (2448, 2048) {
        panic!("Incorrect image dimensions!");
    }

    // NOTE: Could create 4 images by splitting channels and then linearly interpolate based on
    // pixel position.

    let image_0 = GrayImage::from_par_fn(1224, 1024, |x, y| *image.get_pixel(x * 2 + 1, y * 2 + 1));

    let i0_samples: Vec<f64> = (0..1024)
        .into_iter()
        .map(|y| (0..1224).into_iter().map(move |x| (x, y)))
        .flatten()
        .map(|(x, y)| image.get_pixel(x * 2 + 1, y * 2 + 1).0[0] as f64 / 255.)
        .collect();

    let i45_samples: Vec<f64> = (0..1024)
        .into_iter()
        .map(|y| (0..1224).into_iter().map(move |x| (x, y)))
        .flatten()
        .map(|(x, y)| image.get_pixel(x * 2 + 0, y * 2 + 1).0[0] as f64 / 255.)
        .collect();

    let i90_samples: Vec<f64> = (0..1024)
        .into_iter()
        .map(|y| (0..1224).into_iter().map(move |x| (x, y)))
        .flatten()
        .map(|(x, y)| image.get_pixel(x * 2 + 0, y * 2 + 0).0[0] as f64 / 255.)
        .collect();

    let i135_samples: Vec<f64> = (0..1024)
        .into_iter()
        .map(|y| (0..1224).into_iter().map(move |x| (x, y)))
        .flatten()
        .map(|(x, y)| image.get_pixel(x * 2 + 1, y * 2 + 0).0[0] as f64 / 255.)
        .collect();

    let surf_i0 = Surface::from_samples(1224, 1024, i0_samples);
    let surf_i45 = Surface::from_samples(1224, 1024, i45_samples);
    let surf_i90 = Surface::from_samples(1224, 1024, i90_samples);
    let surf_i135 = Surface::from_samples(1224, 1024, i135_samples);

    let normalized_pixel_locations: Vec<(f64, f64)> = (0..2048)
        .into_iter()
        .map(|y| (0..2448).into_iter().map(move |x| (x, y)))
        .flatten()
        .map(|(x, y)| (x as f64, y as f64))
        .map(|(x, y)| (x / 2448. * 1224., y / 2048. * 1224.))
        .collect();

    let mut stokes: Vec<(f64, f64, f64)> = Vec::with_capacity(2448 * 2048);
    for (x, y) in normalized_pixel_locations.into_iter() {
        stokes.push((surf_i0.interp(x, y) + surf_i90.interp(x, y), 0., 0.));
    }
}

use clap::Parser;
use image::{ImageBuffer, ImageReader, Rgb};
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
        let x2 = match x1 == x {
            true => x1 + 1.,
            false => x.ceil(),
        };

        let y1 = y.floor();
        let y2 = match y1 == y {
            true => y1 + 1.,
            false => y.ceil(),
        };

        let w11 = (x2 - x) * (y2 - y);
        let w12 = (x2 - x) * (y - y1);
        let w21 = (x - x1) * (y2 - y);
        let w22 = (x - x1) * (y - y1);

        assert_eq!(w11 + w12 + w21 + w22, 1.);

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

    let i0_samples: Vec<f64> = (0..1024)
        .into_iter()
        .map(|y| (0..1224).into_iter().map(move |x| (x, y)))
        .flatten()
        .map(|(x, y)| image.get_pixel(x * 2 + 1, y * 2 + 1).0[0] as f64)
        .collect();

    let i45_samples: Vec<f64> = (0..1024)
        .into_iter()
        .map(|y| (0..1224).into_iter().map(move |x| (x, y)))
        .flatten()
        .map(|(x, y)| image.get_pixel(x * 2 + 0, y * 2 + 1).0[0] as f64)
        .collect();

    let i90_samples: Vec<f64> = (0..1024)
        .into_iter()
        .map(|y| (0..1224).into_iter().map(move |x| (x, y)))
        .flatten()
        .map(|(x, y)| image.get_pixel(x * 2 + 0, y * 2 + 0).0[0] as f64)
        .collect();

    let i135_samples: Vec<f64> = (0..1024)
        .into_iter()
        .map(|y| (0..1224).into_iter().map(move |x| (x, y)))
        .flatten()
        .map(|(x, y)| image.get_pixel(x * 2 + 1, y * 2 + 0).0[0] as f64)
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
        .map(|(x, y)| (x * 1224. / 2448., y * 1024. / 2048.))
        .collect();

    let mut stokes: Vec<(f64, f64, f64)> = Vec::with_capacity(2448 * 2048);
    for (x, y) in normalized_pixel_locations.into_iter() {
        let i0 = surf_i0.interp(x, y);
        let i45 = surf_i45.interp(x, y);
        let i90 = surf_i90.interp(x, y);
        let i135 = surf_i135.interp(x, y);

        let beta = (y - 512.).atan2(x - 612.) * 2.;
        let s1 = i0 - i90;
        let s2 = i45 - i135;

        stokes.push((
            (i0 + i90 + i45 + i135) / 2.,
            s1 * beta.cos() + s2 * beta.sin(),
            s2 * beta.cos() - s1 * beta.sin(),
        ));
    }

    let dop_max = 0.4;
    let dop: Vec<f64> = stokes
        .par_iter()
        .map(|(s0, s1, s2)| (s1.powf(2.) + s2.powf(2.)).sqrt() / s0)
        .map(|dop| dop.clamp(0., dop_max))
        .collect();

    let aop: Vec<f64> = stokes
        .par_iter()
        .map(|(_, s1, s2)| s2.atan2(*s1) / 2.)
        .map(|aop| aop.to_degrees())
        .collect();

    let mut dop_image = ImageBuffer::new(2448, 2048);
    dop_image
        .par_enumerate_pixels_mut()
        .for_each(|(x, y, pixel)| {
            let index = (y * 2448 + x) as usize;
            *pixel = Rgb(to_rgb(dop[index], 0., dop_max).unwrap());
        });

    let mut aop_image = ImageBuffer::new(2448, 2048);
    aop_image
        .par_enumerate_pixels_mut()
        .for_each(|(x, y, pixel)| {
            let index = (y * 2448 + x) as usize;
            *pixel = Rgb(to_rgb(aop[index], -90., 90.).unwrap());
        });

    let _ = dop_image.save("dop.png");
    let _ = aop_image.save("aop.png");
}

/// Map an f64 on the interval [x_min, x_max] to an RGB color.
fn to_rgb(x: f64, x_min: f64, x_max: f64) -> Option<[u8; 3]> {
    if x < x_min || x > x_max {
        return None;
    }

    let interval_width = x_max - x_min;
    let x_norm = ((x - x_min) / interval_width * 255.).floor() as u8;

    let r = vec![
        255,
        x_norm
            .checked_sub(96)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
        255 - x_norm
            .checked_sub(224)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
    ]
    .into_iter()
    .min()
    .unwrap();

    let g = vec![
        255,
        x_norm
            .checked_sub(32)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
        255 - x_norm
            .checked_sub(160)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
    ]
    .into_iter()
    .min()
    .unwrap();

    let b = vec![
        255,
        x_norm
            .checked_add(127)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
        255 - x_norm
            .checked_sub(96)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
    ]
    .into_iter()
    .min()
    .unwrap();

    Some([r, g, b])
}

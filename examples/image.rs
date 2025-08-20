use rumpus::prelude::*;
use uom::si::{angle::degree, f64::Length, length::micron};

fn main() {
    // Define required parameters.
    let input_path = "testing/intensity.png";
    let output_path = "aop_image.png";
    let pixel_size = Length::new::<micron>(3.45 * 2.);
    let min_dop = 0.5;

    // Open a new image and ensure it is in single channel greyscale format.
    let raw_image = image::ImageReader::open(&input_path)
        .unwrap()
        .decode()
        .unwrap()
        .into_luma8();

    // Create a new IntensityImage from the input image.
    let (width, height) = raw_image.dimensions();
    let intensity_image =
        IntensityImage::from_bytes(width as u16, height as u16, &raw_image.into_raw())
            .expect("image dimensions are even");

    // Filter the rays from the intensity image by DoP.
    // Convert the sparse RayIterator into a dense RayImage using the specs of
    // the image sensor as a RaySensor.
    let ray_image: RayImage<SensorFrame> = intensity_image
        .rays(pixel_size, pixel_size)
        .ray_filter(DopFilter::new(min_dop))
        .ray_image(&ImageSensor::new(
            pixel_size,
            pixel_size,
            intensity_image.height(),
            intensity_image.width(),
        ))
        .expect("no ray hits the same pixel");

    // Map the AoP values in the RayImage to RGB colours.
    // Draw missing pixels as white.
    let aop_image: Vec<u8> = ray_image
        .ray_pixels()
        .flat_map(|pixel| match pixel {
            Some(ray) => to_rgb(ray.aop().angle().get::<degree>(), -90.0, 90.0)
                .expect("aop in between -90 and 90"),
            None => [255, 255, 255],
        })
        .collect();

    // Save the buffer of RGB pixels as a PNG.
    image::save_buffer(
        &output_path,
        &aop_image,
        intensity_image.width().into(),
        intensity_image.height().into(),
        image::ExtendedColorType::Rgb8,
    )
    .expect("valid image and path");
}

// Map an f64 on the interval [x_min, x_max] to an RGB color.
pub fn to_rgb(x: f64, x_min: f64, x_max: f64) -> Option<[u8; 3]> {
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

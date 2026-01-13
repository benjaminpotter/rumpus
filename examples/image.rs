use rumpus::{image::Jet, prelude::*};
use uom::si::{f64::Length, length::micron};

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
    let rays = intensity_image
        .rays(pixel_size, pixel_size)
        .ray_filter(DopFilter::new(min_dop));

    let ray_image = RayImage::from_rays_with_sensor(
        rays,
        &ImageSensor::new(
            pixel_size,
            pixel_size,
            intensity_image.height(),
            intensity_image.width(),
        ),
    )
    .expect("no ray hits the same pixel");

    // Save the buffer of RGB pixels as a PNG.
    image::save_buffer(
        &output_path,
        &ray_image.aop_bytes(&Jet),
        intensity_image.width().into(),
        intensity_image.height().into(),
        image::ExtendedColorType::Rgb8,
    )
    .expect("valid image and path");
}

use rumpus::{image::Jet, prelude::*};

fn main() {
    // Define required parameters.
    let input_path = "testing/intensity.png";
    let output_path = "aop_image.png";

    // Open a new image and ensure it is in single channel greyscale format.
    let raw_image = image::ImageReader::open(&input_path)
        .unwrap()
        .decode()
        .unwrap()
        .into_luma8();

    // Create a new IntensityImage from the input image.
    let (width, height) = raw_image.dimensions();
    let intensity_image =
        IntensityImage::from_bytes(width as usize, height as usize, &raw_image.into_raw())
            .expect("image dimensions are even");

    // Filter the rays from the intensity image by DoP.
    // Convert the sparse RayIterator into a dense RayImage using the specs of
    // the image sensor as a RaySensor.
    let rays: Vec<_> = intensity_image.rays().map(|ray| Some(ray)).collect();
    let ray_image =
        RayImage::from_rays(rays, intensity_image.height(), intensity_image.width()).unwrap();

    // Save the buffer of RGB pixels as a PNG.
    image::save_buffer(
        &output_path,
        &ray_image.aop_bytes(&Jet),
        ray_image.cols() as u32,
        ray_image.rows() as u32,
        image::ExtendedColorType::Rgb8,
    )
    .expect("valid image and path");
}

use rumpus::{image::Jet, prelude::*};
use std::{
    io::Cursor,
    path::{Path, PathBuf},
};
use uom::si::{f64::Length, length::micron};

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join(name)
}

fn ray_image<P: AsRef<Path>>(path: P) -> RayImage<SensorFrame> {
    let pixel_size = Length::new::<micron>(3.45 * 2.);

    let raw_image = image::ImageReader::open(&path)
        .unwrap()
        .decode()
        .unwrap()
        .into_luma8();

    let (width, height) = raw_image.dimensions();
    let intensity_image =
        IntensityImage::from_bytes(width as u16, height as u16, &raw_image.into_raw())
            .expect("image dimensions are even");

    let ray_image = RayImage::from_rays_with_sensor(
        intensity_image.rays(pixel_size, pixel_size),
        &ImageSensor::new(
            pixel_size,
            pixel_size,
            intensity_image.height(),
            intensity_image.width(),
        ),
    )
    .expect("no ray hits the same pixel");

    ray_image
}

#[test]
fn aop_works() {
    let input_path = fixture_path("intensity.png");
    let ray_image = ray_image(&input_path);

    let mut png_bytes: Vec<u8> = Vec::new();
    image::write_buffer_with_format(
        &mut Cursor::new(&mut png_bytes),
        &ray_image.aop_bytes(&Jet),
        ray_image.cols() as u32,
        ray_image.rows() as u32,
        image::ExtendedColorType::Rgb8,
        image::ImageFormat::Png,
    )
    .unwrap();
    insta::assert_binary_snapshot!(".png", png_bytes);
}

#[test]
fn dop_works() {
    let input_path = fixture_path("intensity.png");
    let ray_image = ray_image(&input_path);

    let mut png_bytes: Vec<u8> = Vec::new();
    image::write_buffer_with_format(
        &mut Cursor::new(&mut png_bytes),
        &ray_image.dop_bytes(&Jet),
        ray_image.cols() as u32,
        ray_image.rows() as u32,
        image::ExtendedColorType::Rgb8,
        image::ImageFormat::Png,
    )
    .unwrap();
    insta::assert_binary_snapshot!(".png", png_bytes);
}

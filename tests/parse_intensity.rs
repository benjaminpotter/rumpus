use rumpus::{
    image::{Binary, Gray, Jet},
    prelude::*,
};
use std::{
    io::Cursor,
    path::{Path, PathBuf},
};

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join(name)
}

fn ray_image<P: AsRef<Path>>(path: P) -> RayImage<SensorFrame> {
    let raw_image = image::ImageReader::open(&path)
        .unwrap()
        .decode()
        .unwrap()
        .into_luma8();

    let (width, height) = raw_image.dimensions();
    let intensity_image =
        IntensityImage::from_bytes(width as usize, height as usize, &raw_image.into_raw())
            .expect("image dimensions are even");

    let rays: Vec<_> = intensity_image.rays().map(|ray| Some(ray)).collect();
    RayImage::from_rays(rays, intensity_image.height(), intensity_image.width()).unwrap()
}

#[test]
fn binary_works() {
    let input_path = fixture_path("intensity.png");
    let ray_image = ray_image(&input_path);

    let bytes = ray_image.aop_bytes(&Binary);
    insta::assert_binary_snapshot!("binary_aop.bin", bytes);

    let bytes = ray_image.dop_bytes(&Binary);
    insta::assert_binary_snapshot!("binary_dop.bin", bytes);
}

#[test]
#[allow(clippy::cast_possible_truncation)]
fn jet_works() {
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

    insta::assert_binary_snapshot!("jet_aop.png", png_bytes);

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

    insta::assert_binary_snapshot!("jet_dop.png", png_bytes);
}

#[test]
#[allow(clippy::cast_possible_truncation)]
fn gray_works() {
    let input_path = fixture_path("intensity.png");
    let ray_image = ray_image(&input_path);

    let mut png_bytes: Vec<u8> = Vec::new();
    image::write_buffer_with_format(
        &mut Cursor::new(&mut png_bytes),
        &ray_image.aop_bytes(&Gray),
        ray_image.cols() as u32,
        ray_image.rows() as u32,
        image::ExtendedColorType::L8,
        image::ImageFormat::Png,
    )
    .unwrap();

    insta::assert_binary_snapshot!("gray_aop.png", png_bytes);

    let mut png_bytes: Vec<u8> = Vec::new();
    image::write_buffer_with_format(
        &mut Cursor::new(&mut png_bytes),
        &ray_image.dop_bytes(&Gray),
        ray_image.cols() as u32,
        ray_image.rows() as u32,
        image::ExtendedColorType::L8,
        image::ImageFormat::Png,
    )
    .unwrap();

    insta::assert_binary_snapshot!("gray_dop.png", png_bytes);
}

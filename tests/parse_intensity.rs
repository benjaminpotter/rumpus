use rumpus::{
    image::{Binary, Gray, IntensityImage, Jet, RayImage},
    ray::SensorFrame,
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

fn load_intensity_image<P: AsRef<Path>>(path: P) -> IntensityImage {
    let raw_image = image::ImageReader::open(&path)
        .unwrap()
        .decode()
        .unwrap()
        .into_luma8();

    let (width, height) = raw_image.dimensions();
    IntensityImage::from_bytes(width as usize, height as usize, &raw_image.into_raw())
        .expect("image dimensions are even")
}

fn load_ray_image<P: AsRef<Path>>(path: P) -> RayImage<SensorFrame> {
    let raw_image = image::ImageReader::open(&path)
        .unwrap()
        .decode()
        .unwrap()
        .into_luma8();

    let (width, height) = raw_image.dimensions();
    let intensity_image =
        IntensityImage::from_bytes(width as usize, height as usize, &raw_image.into_raw())
            .expect("image dimensions are even");

    RayImage::from_metapixels(
        intensity_image.metapixels(),
        intensity_image.rows(),
        intensity_image.cols(),
    )
    .unwrap()
}

#[test]
fn binary_works() {
    let input_path = fixture_path("intensity.png");
    let ray_image = load_ray_image(&input_path);

    let bytes = ray_image.aop_bytes(&Binary);
    insta::assert_binary_snapshot!("binary_aop.bin", bytes);

    let bytes = ray_image.dop_bytes(&Binary);
    insta::assert_binary_snapshot!("binary_dop.bin", bytes);

    let bytes = ray_image.intensity_bytes(&Binary);
    insta::assert_binary_snapshot!("binary_intensity.bin", bytes);
}

#[test]
#[allow(clippy::cast_possible_truncation)]
fn jet_works() {
    let input_path = fixture_path("intensity.png");
    let ray_image = load_ray_image(&input_path);

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
    let ray_image = load_ray_image(&input_path);

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

    let mut png_bytes: Vec<u8> = Vec::new();
    image::write_buffer_with_format(
        &mut Cursor::new(&mut png_bytes),
        &ray_image.intensity_bytes(&Gray),
        ray_image.cols() as u32,
        ray_image.rows() as u32,
        image::ExtendedColorType::L8,
        image::ImageFormat::Png,
    )
    .unwrap();

    insta::assert_binary_snapshot!("gray_intensity.png", png_bytes);
}

#[test]
#[allow(clippy::cast_possible_truncation)]
fn s0_works() {
    let input_path = fixture_path("intensity.png");
    let intensity_image = load_intensity_image(&input_path);
    let bytes: Vec<_> = intensity_image
        .metapixels()
        .map(|mpx| mpx.stokes().s0())
        .flat_map(|s0| s0.to_le_bytes())
        .collect();

    insta::assert_binary_snapshot!("s0.bin", bytes);
}

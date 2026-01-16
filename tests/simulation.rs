use std::io::Cursor;

use chrono::prelude::*;
use rayon::prelude::*;
use rumpus::image::Jet;
use rumpus::prelude::*;
use sguaba::Coordinate;
use sguaba::engineering::Orientation;
use sguaba::systems::Wgs84;
use uom::ConstZero;
use uom::si::f64::Angle;
use uom::si::f64::Length;
use uom::si::{
    angle::degree,
    length::{micron, millimeter},
};

fn ray_image() -> RayImage<GlobalFrame> {
    let pixel_size = Length::new::<micron>(3.45 * 2.);
    let image_rows = 1024;
    let image_cols = 1224;
    // Use a small focal length to see more of the sky.
    let focal_length = Length::new::<millimeter>(3.0);
    let latitude = Angle::new::<degree>(44.2187);
    let longitude = Angle::new::<degree>(-76.4747);
    let time = "2025-06-13T16:26:47+00:00";
    let orientation = Orientation::<CameraEnu>::tait_bryan_builder()
        .yaw(Angle::new::<degree>(0.0))
        .pitch(Angle::new::<degree>(0.0))
        .roll(Angle::new::<degree>(0.0))
        .build();

    let lens = Lens::from_focal_length(focal_length).expect("focal length is greater than zero");
    let image_sensor = ImageSensor::new(pixel_size, pixel_size, image_rows, image_cols);
    let coords: Vec<Coordinate<CameraFrd>> = (0..image_rows)
        .flat_map(|row| (0..image_cols).map(move |col| (row, col)))
        .map(|(row, col)| image_sensor.at_pixel(row, col).unwrap())
        .collect();

    let sky_model = SkyModel::from_wgs84_and_time(
        Wgs84::builder()
            .latitude(latitude)
            .expect("latitude is between -90 and 90")
            .longitude(longitude)
            .altitude(Length::ZERO)
            .build(),
        time.parse::<DateTime<Utc>>()
            .expect("valid datetime string"),
    );

    let camera = Camera::new(lens.clone(), orientation);
    let rays: Vec<Ray<_>> = coords
        .par_iter()
        .filter_map(|coord| {
            let bearing_cam_enu = camera
                .trace_from_sensor(*coord)
                .expect("coord on sensor plane");
            let aop = sky_model.aop(bearing_cam_enu)?;
            let dop = sky_model.dop(bearing_cam_enu)?;

            Some(Ray::new(*coord, aop, dop))
        })
        .collect();

    let ray_image =
        RayImage::from_rays_with_sensor(rays, &image_sensor).expect("no ray hits the same pixel");

    ray_image
}

#[test]
fn aop_works() {
    let ray_image = ray_image();

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
    let ray_image = ray_image();

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

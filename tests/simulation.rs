use std::io::Cursor;

use chrono::prelude::*;
use rumpus::image::Jet;
use rumpus::image::RayImage;
use rumpus::optic::Camera;
use rumpus::optic::PinholeOptic;
use rumpus::ray::GlobalFrame;
use rumpus::simulation::Simulation;
use sguaba::Coordinate;
use sguaba::engineering::Orientation;
use sguaba::engineering::Pose;
use sguaba::math::RigidBodyTransform;
use sguaba::system;
use sguaba::systems::Wgs84;
use uom::ConstZero;
use uom::si::f64::Angle;
use uom::si::f64::Length;
use uom::si::{
    angle::degree,
    length::{micron, millimeter},
};

system!(struct CameraBody using right-handed XYZ);
system!(struct CameraEnu using ENU);

fn ray_image() -> RayImage<GlobalFrame> {
    let pixel_size = Length::new::<micron>(3.45 * 2.);
    let image_rows = 1024;
    let image_cols = 1224;
    // Use a small focal length to see more of the sky.
    let focal_length = Length::new::<millimeter>(3.0);
    let latitude = Angle::new::<degree>(44.2187);
    let longitude = Angle::new::<degree>(-76.4747);
    let position = Wgs84::builder()
        .latitude(latitude)
        .expect("latitude is between -90 and 90")
        .longitude(longitude)
        .altitude(Length::ZERO)
        .build();
    let time = "2025-06-13T16:26:47+00:00";

    let camera_pose_enu = Pose::new(
        Coordinate::origin(),
        Orientation::<CameraEnu>::tait_bryan_builder()
            .yaw(Angle::new::<degree>(0.0))
            .pitch(Angle::new::<degree>(0.0))
            // .pitch(Angle::new::<degree>(180.0))
            .roll(Angle::new::<degree>(180.0))
            .build(),
    );

    // SAFETY: CameraBody and CameraEnu have coincident origins.
    let camera_enu_to_ecef = unsafe { RigidBodyTransform::ecef_to_enu_at(&position) }.inverse();
    let camera_pose_ecef = camera_enu_to_ecef.transform(camera_pose_enu);

    Simulation::new(
        Camera::new(
            PinholeOptic::from_focal_length(focal_length),
            pixel_size,
            image_rows,
            image_cols,
        ),
        camera_pose_ecef,
        time.parse::<DateTime<Utc>>()
            .expect("valid datetime string"),
    )
    .par_ray_image()
}

#[test]
fn aop_works() {
    let ray_image = ray_image();

    for pixel in ray_image.rays() {
        assert!(pixel.is_some());
    }

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

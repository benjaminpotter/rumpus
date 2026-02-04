use chrono::{DateTime, Utc};
use rumpus::{
    estimate::pattern_match::Matcher,
    image::Jet,
    optic::{Camera, PinholeOptic},
    prelude::*,
};
use sguaba::{
    engineering::{Orientation, Pose},
    system,
    systems::Wgs84,
};
use std::{
    io::Cursor,
    path::{Path, PathBuf},
};
use uom::{
    ConstZero,
    si::{
        angle::degree,
        f64::{Angle, Length},
        length::{micron, millimeter},
    },
};

#[test]
fn match_works() {
    let image_path = fixture_path("intensity.png");
    let ray_image = ray_image(&image_path);

    matcher()
        .orientation_of::<CameraEnu>(ray_image, position(), time())
        .unwrap();
}

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

fn position() -> Wgs84 {
    let latitude = Angle::new::<degree>(44.2187);
    let longitude = Angle::new::<degree>(-76.4747);
    Wgs84::builder()
        .latitude(latitude)
        .expect("latitude is between -90 and 90")
        .longitude(longitude)
        .altitude(Length::ZERO)
        .build()
}

system!(struct CameraEnu using ENU);

fn orientation() -> Orientation<CameraEnu> {
    Orientation::<CameraEnu>::tait_bryan_builder()
        .yaw(Angle::new::<degree>(0.0))
        .pitch(Angle::new::<degree>(0.0))
        // .pitch(Angle::new::<degree>(180.0))
        .roll(Angle::new::<degree>(180.0))
        .build()
}

fn time() -> DateTime<Utc> {
    let time = "2025-06-13T16:26:47+00:00";
    time.parse::<DateTime<Utc>>()
        .expect("valid datetime string")
}

fn matcher() -> Matcher<PinholeOptic> {
    let pixel_size = Length::new::<micron>(3.45 * 2.);
    let image_rows = 1024;
    let image_cols = 1224;
    // Use a small focal length to see more of the sky.
    let focal_length = Length::new::<millimeter>(8.0);

    Matcher::new(
        Camera::new(
            PinholeOptic::from_focal_length(focal_length),
            pixel_size,
            image_rows,
            image_cols,
        ),
        0.1,
        0.1,
        10,
    )
}

use chrono::prelude::*;
use image::ImageReader;
use rumpus::prelude::*;
use sguaba::{engineering::Orientation, systems::Wgs84};
use uom::{
    ConstZero,
    si::{
        angle::degree,
        f64::{Angle, Length},
        length::{micron, millimeter},
    },
};

fn main() {
    // let image_path = "testing/intensity.png";
    // let pixel_size = Length::new::<micron>(3.45 * 2.);
    // let focal_length = Length::new::<millimeter>(8.0);
    // let latitude = Angle::new::<degree>(44.2187);
    // let longitude = Angle::new::<degree>(-76.4747);
    // let time = "2025-06-13T16:26:47+00:00";
    // let orientations = vec![
    //     Orientation::<CameraEnu>::tait_bryan_builder()
    //         .yaw(Angle::new::<degree>(0.0))
    //         .pitch(Angle::new::<degree>(0.0))
    //         .roll(Angle::new::<degree>(0.0))
    //         .build(),
    //     Orientation::<CameraEnu>::tait_bryan_builder()
    //         .yaw(Angle::new::<degree>(90.0))
    //         .pitch(Angle::new::<degree>(0.0))
    //         .roll(Angle::new::<degree>(0.0))
    //         .build(),
    // ];
    // let min_dop = 0.5;
    // let max_iterations = 10;
    //
    // let raw_image = ImageReader::open(&image_path)
    //     .unwrap()
    //     .decode()
    //     .unwrap()
    //     .into_luma8();
    //
    // let (width, height) = raw_image.dimensions();
    // let estimate = IntensityImage::from_bytes(width as u16, height as u16, &raw_image.into_raw())
    //     .expect("image dimensions are even")
    //     .rays(pixel_size, pixel_size)
    //     .ray_filter(DopFilter::new(min_dop))
    //     .estimate(PatternMatch::new(
    //         Lens::from_focal_length(focal_length).expect("focal length is greater than zero"),
    //         SkyModel::from_wgs84_and_time(
    //             Wgs84::builder()
    //                 .latitude(latitude)
    //                 .expect("latitude is between -90 and 90")
    //                 .longitude(longitude)
    //                 .altitude(Length::ZERO)
    //                 .build(),
    //             time.parse::<DateTime<Utc>>()
    //                 .expect("valid datetime string"),
    //         ),
    //         VecSearch::new(orientations),
    //         max_iterations,
    //     ));
    //
    // println!("{:#?}", estimate.to_tait_bryan_angles());
}

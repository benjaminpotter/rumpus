use chrono::prelude::*;
use rayon::prelude::*;
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

fn main() {
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

    // Map the AoP values in the RayImage to RGB colours.
    // Draw missing pixels as white.
    let aop_image: Vec<u8> = ray_image
        .pixels()
        .flat_map(|pixel| match pixel {
            Some(ray) => to_rgb(Into::<Angle>::into(*ray.aop()).get::<degree>(), -90.0, 90.0)
                .expect("aop in between -90 and 90"),
            None => [255, 255, 255],
        })
        .collect();

    // Map the DoP values in the RayImage to RGB colours.
    // Draw missing pixels as white.
    let dop_image: Vec<u8> = ray_image
        .pixels()
        .flat_map(|pixel| match pixel {
            Some(ray) => to_rgb(*ray.dop(), 0.0, 1.0).expect("dop in between 0 and 1"),
            None => [255, 255, 255],
        })
        .collect();

    // Save the buffer of RGB pixels as a PNG.
    let _ = image::save_buffer(
        "aop.png",
        &aop_image,
        image_cols.into(),
        image_rows.into(),
        image::ExtendedColorType::Rgb8,
    );

    // Save the buffer of RGB pixels as a PNG.
    let _ = image::save_buffer(
        "dop.png",
        &dop_image,
        image_cols.into(),
        image_rows.into(),
        image::ExtendedColorType::Rgb8,
    );
}

// Map an f64 on the interval [x_min, x_max] to an RGB color.
pub fn to_rgb(x: impl Into<f64>, x_min: f64, x_max: f64) -> Option<[u8; 3]> {
    let x = x.into();
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

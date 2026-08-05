use chrono::{DateTime, Utc};
use rumpus::{
    optic::{Camera, PinholeOptic},
    simulation::Simulation,
};
use sguaba::{
    Coordinate,
    engineering::{Orientation, Pose},
    math::RigidBodyTransform,
    system,
    systems::Wgs84,
};
use uom::{
    ConstZero,
    si::{
        angle::{Angle, degree},
        f64::Length,
        length::{micron, millimeter},
    },
};

system!(struct CameraBody using right-handed XYZ);
system!(struct CameraEnu using ENU);

fn main() {
    // Define the parameters for the sensor.
    let pixel_size = Length::new::<micron>(3.45 * 2.);
    let image_rows = 1024;
    let image_cols = 1224;

    // Define the parameters for the optical model.
    let focal_length = Length::new::<millimeter>(3.0);

    // Define the state of the virtual camera.
    let time = "2025-06-13T16:26:47+00:00";
    let latitude = Angle::new::<degree>(44.2187);
    let longitude = Angle::new::<degree>(-76.4747);
    let position = Wgs84::builder()
        .latitude(latitude)
        .expect("latitude is between -90 and 90")
        .longitude(longitude)
        .altitude(Length::ZERO)
        .build();

    // Construct a pose for the virtual camera from the provided state.
    // Pose is initially defined in the ENU frame with the origin at the virtual camera.
    let camera_pose_enu = Pose::new(
        Coordinate::origin(),
        Orientation::<CameraEnu>::tait_bryan_builder()
            .yaw(Angle::new::<degree>(0.0))
            .pitch(Angle::new::<degree>(0.0))
            .roll(Angle::new::<degree>(180.0))
            .build(),
    );

    // Pose is transformed into the ECEF frame.
    // SAFETY: CameraBody and CameraEnu have coincident origins.
    let camera_enu_to_ecef = unsafe { RigidBodyTransform::ecef_to_enu_at(&position) }.inverse();
    let camera_pose_ecef = camera_enu_to_ecef.transform(camera_pose_enu);

    // Build the simulation from the camera model and the virtual camera state.
    // Use .par_ray_image() to process each pixel in parallel.
    let _ray_image = Simulation::new(
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
    // Request an image from the simulation.
    .par_ray_image();
}

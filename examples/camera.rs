use rumpus::prelude::*;
use sguaba::{coordinate, engineering::Orientation, system, Bearing, Coordinate, Vector};
use uom::{
    si::{
        angle::degree,
        f64::{Angle, Length},
        length::{micron, millimeter},
    },
    ConstZero,
};

fn main() {
    let x_seed: i16 = 0;
    let y_seed: i16 = 0;
    let roll_seed: i8 = 0;
    let pitch_seed: i8 = 0;
    let yaw_seed: u16 = 0;

    // letruct pixel coordinates from the random seed passed by quickcheck.
    // Aim to have pixel coordinates on range -5000 to 5000 microns.
    let x = Length::new::<micron>(x_seed as f64 * 5000. / i16::MAX as f64);
    let y = Length::new::<micron>(y_seed as f64 * 5000. / i16::MAX as f64);
    let px = Coordinate::<CameraFrd>::builder()
        .frd_front(x)
        .frd_right(y)
        .frd_down(Length::ZERO)
        .build();
    dbg!(px);

    // letruct orientation from the random seed passed by quickcheck.
    // Aim to have roll and pitch on the range -30 to 30.
    // Aim to have yaw on the range 0 to 360.
    let roll = Angle::new::<degree>(roll_seed as f64 * 30. / i8::MAX as f64);
    let pitch = Angle::new::<degree>(pitch_seed as f64 * 30. / i8::MAX as f64);
    let yaw = Angle::new::<degree>(yaw_seed as f64 * 360. / u16::MAX as f64);
    let ort = Orientation::<CameraEnu>::tait_bryan_builder()
        .yaw(yaw)
        .pitch(pitch)
        .roll(roll)
        .build();

    let focal_length = Length::new::<millimeter>(8.0);
    let lens = Lens::from_focal_length(focal_length).expect("focal_length is greater than zero");
    let cam = Camera::new(lens, ort);

    let bearing_enu = cam
        .trace_from_sensor(px.clone())
        .expect("px has a z value of zero");
    dbg!(bearing_enu);
    let result = cam.trace_from_sky(bearing_enu);
    dbg!(result);
}

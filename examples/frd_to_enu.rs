use sguaba::{engineering::Orientation, system, Bearing, Coordinate};
use uom::{
    si::{
        angle::degree,
        f64::{Angle, Length},
        length::{micron, millimeter},
    },
    ConstZero,
};

system!(struct SensorEnu using ENU);
system!(struct SensorFrd using FRD);

fn main() {
    // Define the orientation of the sensor in the ENU frame.
    let ort = Orientation::<SensorEnu>::tait_bryan_builder()
        .yaw(Angle::new::<degree>(0.0))
        .pitch(Angle::new::<degree>(0.0))
        .roll(Angle::new::<degree>(0.0))
        .build();

    // Define a coordinate relative to the sensor's FRD frame.
    // Point of interest is above the sensor, hence a negative Z value.
    let poi_frd = Coordinate::<SensorFrd>::builder()
        .frd_front(Length::new::<millimeter>(0.0))
        .frd_right(Length::new::<millimeter>(0.0))
        .frd_down(Length::new::<millimeter>(-1.0))
        .build();
    dbg!(poi_frd);

    // Take a bearing on the point of interest in the sensor's FRD frame.
    let poi_bearing_frd = poi_frd
        .bearing_from_origin()
        .expect("poi is not at the origin");
    dbg!(poi_bearing_frd);

    // Construct a transform between the sensor's FRD frame and the sensor's ENU frame.
    // Both frames have the same origin - the center of the sensor - but the FRD
    // frame's orientation is with respect to the sensor's axes whereas the ENU
    // frame is with respect to the East, North, and Up axes.
    // In other words, if the ENU axes are rotated to `ort` then they coincide
    // with the orientation of the FRD axes.
    let frd_to_enu = unsafe { ort.map_as_zero_in::<SensorFrd>() }.inverse();

    // We want to determine the point of interest's coordinate in the ENU frame.
    let poi_enu = frd_to_enu.transform(poi_frd);
    dbg!(poi_enu);

    // Take a bearing on the point of interest in the sensor's ENU frame.
    let poi_bearing_enu = poi_enu
        .bearing_from_origin()
        .expect("poi is not at the origin");
    dbg!(poi_bearing_enu);

    // Transform our bearing from sensor's FRD frame to sensor's ENU frame.
    assert_eq!(poi_bearing_enu, frd_to_enu.transform(poi_bearing_frd));
}

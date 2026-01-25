use crate::{
    image::RayImage,
    model::SkyModel,
    optic::{Camera, Optic, PixelCoordinate},
    ray::{GlobalFrame, Ray},
};
use chrono::{DateTime, Utc};
use sguaba::{
    Bearing,
    engineering::Pose,
    math::{RigidBodyTransform, Rotation},
    system,
    systems::{BearingDefined, Ecef},
};
use uom::si::f64::Angle;

system!(struct SimulationEnu using ENU);
system!(struct CameraXyz using right-handed XYZ);

pub struct Simulation<O> {
    camera: Camera<O>,
    camera_pose: Pose<SimulationEnu>,
    model: SkyModel<SimulationEnu>,
}

impl<O> Simulation<O> {
    pub fn new(camera: Camera<O>, camera_pose: Pose<Ecef>, time: DateTime<Utc>) -> Self {
        // SAFETY: The origin of SimulationEnu is coincident with the camera's position.
        let model = unsafe { SkyModel::from_position_and_time(camera_pose.position(), time) };
        let camera_pose =
            unsafe { RigidBodyTransform::ecef_to_enu_at(&camera_pose.position().into()) }
                .transform(camera_pose);
        Self {
            camera,
            camera_pose,
            model,
        }
    }

    pub fn ray(&self, pixel: impl AsRef<PixelCoordinate>) -> Option<Ray<GlobalFrame>>
    where
        O: Optic,
    {
        let ray_bearing = self.camera.trace_from_pixel(pixel)?;

        let bearing_cam =
            CameraXyz::spherical_to_bearing(ray_bearing.polar(), ray_bearing.azimuth()).unwrap();

        // SAFETY: The position of camera_pose lies at the origin of CameraXyz.
        let cam_to_sim: Rotation<CameraXyz, SimulationEnu> =
            unsafe { self.camera_pose.orientation().map_as_zero_in::<CameraXyz>() }.inverse();
        let bearing_sim = cam_to_sim.transform(bearing_cam);

        Some(Ray::new(
            self.model.aop(bearing_sim)?,
            self.model.dop(bearing_sim)?,
        ))
    }

    pub fn ray_image(&self) -> RayImage<GlobalFrame>
    where
        O: Optic,
    {
        RayImage::from_pixels(
            self.camera.pixels().map(|px| self.ray(px)).collect(),
            self.camera.rows(),
            self.camera.cols(),
        )
        .unwrap()
    }
}

impl BearingDefined for CameraXyz {
    fn bearing_to_spherical(bearing: Bearing<Self>) -> (Angle, Angle) {
        let polar = Angle::HALF_TURN / 2.0 - bearing.elevation();
        let azimuth = bearing.azimuth();
        (polar, azimuth)
    }

    fn spherical_to_bearing(
        polar: impl Into<Angle>,
        azimuth: impl Into<Angle>,
    ) -> Option<Bearing<Self>> {
        let elevation = Angle::HALF_TURN / 2.0 - polar.into();
        let azimuth = azimuth.into();

        Some(
            Bearing::builder()
                .azimuth(azimuth)
                .elevation(elevation)?
                .build(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;
    use uom::ConstZero;

    #[rstest]
    #[case(Angle::HALF_TURN/2.0)]
    #[case(Angle::HALF_TURN/4.0)]
    fn bearing_cam_xyz_roundtrip(#[case] elevation: Angle) {
        let bearing = Bearing::<CameraXyz>::builder()
            .azimuth(Angle::ZERO)
            .elevation(elevation)
            .unwrap()
            .build();

        let (polar, azimuth) = CameraXyz::bearing_to_spherical(bearing);
        let result = CameraXyz::spherical_to_bearing(polar, azimuth);

        assert_eq!(result, Some(bearing));
    }
}

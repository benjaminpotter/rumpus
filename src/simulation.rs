use crate::{
    image::RayImage,
    model::SkyModel,
    optic::{Camera, Optic, PixelCoordinate},
    ray::{GlobalFrame, Ray},
};
use chrono::{DateTime, Utc};
use sguaba::{
    Bearing,
    engineering::{Orientation, Pose},
    math::{RigidBodyTransform, Rotation},
    system,
    systems::{Ecef, Wgs84},
};
use uom::si::f64::Length;

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
            unsafe { RigidBodyTransform::ecef_to_enu_at(camera_pose.position().into()) }
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
        // convert pixel to sensor coordinate
        // let ray_bearing = self.camera.trace_backward(pixel);

        // let bearing = Bearing::builder().azimuth(pixel_bearing.azimuth()).elevation(pixel_bearing.elevation())
        //
        // Ray::new(self.model.aop(pixel_bearing), self.model.dop(pixel_bearing))
        // let rays: Vec<Ray<_>> = coords
        //     .par_iter()
        //     .filter_map(|coord| {
        //         let bearing_cam_enu = camera
        //             .trace_from_sensor(*coord)
        //             .expect("coord on sensor plane");
        //         let aop = sky_model.aop(bearing_cam_enu)?;
        //         let dop = sky_model.dop(bearing_cam_enu)?;
        //
        //         Some(Ray::new(*coord, aop, dop))
        //     })
        //     .collect();

        let ray_bearing = self.camera.trace_from_pixel(pixel)?;
        let bearing_cam = Bearing::<CameraXyz>::builder()
            .azimuth(ray_bearing.azimuth())
            .elevation(ray_bearing.elevation())
            .unwrap()
            .build();

        let cam_to_sim: Rotation<CameraXyz, SimulationEnu> =
            unsafe { self.camera_pose.orientation().map_as_zero_in::<CameraXyz>() }.inverse();
        let bearing_sim = cam_to_sim.transform(bearing_cam);

        todo!()
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

#[cfg(test)]
mod tests {}

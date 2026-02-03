use crate::{
    image::RayImage,
    model::SkyModel,
    optic::{Camera, Optic, PixelCoordinate},
    ray::{GlobalFrame, Ray},
};
use chrono::{DateTime, Utc};
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use sguaba::{
    Bearing,
    engineering::Pose,
    math::{RigidBodyTransform, Rotation},
    system,
    systems::{BearingDefined, Ecef},
};
use uom::si::f64::Angle;

// Global frame of the simulation.
// Axes are aligned with east, north, and up.
// Orientation of the camera is defined in this frame.
system!(struct SimulationEnu using ENU);

// Body frame of the camera.
// X points towards the right of the image.
// Y points towards the top of the image.
// Z points towards the viewer (away from the sky).
system!(struct CameraXyz using right-handed XYZ);

/// This type describes a [`Camera`] with a [`Pose`] viewing a [`SkyModel`].
/// It is responsible for mapping [`PixelCoordinate`]s from the [`Camera`] onto [`Ray`]s from
/// incident skylight.
/// [`Ray`]s encode the polarization state (i.e., the angle and degree of polarization) for
/// different regions of the sky.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Simulation<O> {
    camera: Camera<O>,
    camera_pose: Pose<SimulationEnu>,
    model: SkyModel<SimulationEnu>,
}

impl<O> Simulation<O> {
    /// Construct a simulation from a [`Camera`] with a [`Pose`] in [`Ecef`] and at a
    /// [`DateTime<Utc>`].
    ///
    /// The time is used to construct a [`SkyModel`] which requires knowing the position of the sun
    /// in the sky.
    /// This is determined with the time provided and the position of the camera taken from its pose.
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

    /// # Panics
    /// Panics if the [`crate::optic::RayDirection`] returned by the [`Camera`] points behind the
    /// plane of the sensor.
    /// This would represent a field of view larger than 180 degrees.
    pub fn ray(&self, pixel: impl AsRef<PixelCoordinate>) -> Option<Ray<GlobalFrame>>
    where
        O: Optic,
    {
        // Defined in the body frame of the camera.
        let ray_direction = self.camera.trace_from_pixel(pixel)?;

        // FIXME: This should probably be allowed... what happens if a lens has a FOV > 180?
        let bearing_cam =
            CameraXyz::spherical_to_bearing(ray_direction.polar(), ray_direction.azimuth())
                .unwrap();

        // SAFETY: The position of camera_pose lies at the origin of CameraXyz.
        let cam_to_sim: Rotation<CameraXyz, SimulationEnu> =
            unsafe { self.camera_pose.orientation().map_as_zero_in::<CameraXyz>() }.inverse();
        let bearing_sim = cam_to_sim.transform(bearing_cam);

        Some(Ray::new(
            self.model.aop(bearing_sim)?,
            self.model.dop(bearing_sim)?,
        ))
    }

    /// # Panics
    /// Panics if the dimensions of the [`Camera`]'s image sensor do not match the results returned
    /// by [`Camera::pixels`].
    /// This should never occur.
    pub fn ray_image(&self) -> RayImage<GlobalFrame>
    where
        O: Optic,
    {
        RayImage::from_rays(
            self.camera.pixels().map(|px| self.ray(px)),
            self.camera.rows(),
            self.camera.cols(),
        )
        .unwrap()
    }

    /// # Panics
    /// Panics if the dimensions of the [`Camera`]'s image sensor do not match the results returned
    /// by [`Camera::pixels`].
    /// This should never occur.
    pub fn par_ray_image(&self) -> RayImage<GlobalFrame>
    where
        O: Optic + Send + Sync,
    {
        let pixels: Vec<_> = self.camera.pixels().collect();
        let rays: Vec<_> = pixels.into_par_iter().map(|px| self.ray(px)).collect();
        RayImage::from_rays(rays, self.camera.rows(), self.camera.cols()).unwrap()
    }
}

// Used to convert from the polar angle convention to the elevation angle convention.
// The elevation angle is taken from the horizontal plane positive towards Z.
// Bearings from the camera should have a negative elevation angle.
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

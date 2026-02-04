use std::marker::PhantomData;

use chrono::{DateTime, Utc};
use sguaba::{
    engineering::Orientation,
    systems::{Ecef, Wgs84},
};
use thiserror::Error;
use uom::si::{angle::degree, f64::Angle};

use crate::{
    image::RayImage,
    optic::{Camera, Optic},
    ray::{GlobalFrame, SensorFrame},
    simulation::Simulation,
};

#[derive(Debug, Error)]
pub enum EstimatorError {
    #[error(
        "estimator failed to converge after {iterations} iterations: {convergence_parameter} > {convergence_threshold}"
    )]
    FailedToConverge {
        iterations: usize,
        convergence_threshold: f64,
        convergence_parameter: f64,
    },
}

pub struct Matcher<O> {
    camera: Camera<O>,
    learning_rate: f64,
    convergence_threshold: f64,
    max_iterations: usize,
}

impl<O> Matcher<O> {
    pub fn new(
        camera: Camera<O>,
        learning_rate: f64,
        convergence_threshold: f64,
        max_iterations: usize,
    ) -> Self {
        Self {
            camera,
            learning_rate,
            convergence_threshold,
            max_iterations,
        }
    }

    fn simulation<In>(
        &self,
        position: impl Into<Wgs84>,
        orientation: Orientation<In>,
        time: impl Into<DateTime<Utc>>,
    ) -> Simulation<O> {
        todo!()
    }

    fn weighted_rmse(measured: &RayImage<GlobalFrame>, simulated: &RayImage<GlobalFrame>) -> f64 {
        let mut sum_weighted_errors = 0.0f64;
        let mut sum_weights = 0.0f64;
        let mut samples = 0.;

        for rpx in measured.pixels() {
            if let Some(measured_ray) = rpx.ray()
                && let Some(simulated_ray) = simulated.ray(rpx.row(), rpx.col())
            {
                let weight = measured_ray.dop();
                let error = Angle::from(simulated_ray.aop() - measured_ray.aop())
                    .get::<degree>()
                    .powf(2.);

                sum_weights += weight;
                sum_weighted_errors += weight * error;
                samples += 1.;
            }
        }

        (sum_weighted_errors / sum_weights / samples).sqrt()
    }

    // NOTE: What did I just realize?
    // - The gradient calculation is based on the partial derivative of AoP wrt yaw.
    // - It could also be defined wrt pitch and roll, but I have not done this yet.
    // - The partial derivative of AoP wrt yaw requires the solar position (i.e., SkyModel).
    // - I think the SkyModel should implement this functionality.
    // - However, it's not clear how the Matcher interacts with this part of the SkyModel.
    // - The Matcher might have to stop using the Simulation as its too abstracted.
    //
    // Implemented by SkyModel:
    //
    // if bearing.elevation() < Angle::ZERO {
    //     return None;
    // }
    //
    // let solar_azimuth = model.solar_bearing().azimuth();
    // let solar_zenith = Angle::HALF_TURN / 2. - model.solar_bearing().elevation();
    // let azimuth = bearing.azimuth();
    // let zenith = Angle::HALF_TURN / 2. - bearing.elevation();
    //
    // let a = (zenith.sin() * solar_zenith.cos()).get::<ratio>();
    // let b = zenith.cos().get::<ratio>();
    // let c = solar_zenith.sin().get::<ratio>();
    // let d = (azimuth - solar_azimuth).get::<radian>();
    //
    // let angle = c * (b * c * d.sin().powf(2.) - d.cos() * (-1. * b * c * d.cos() + a))
    //     / (-1. * b * c * d.cos() + a).powf(2.)
    //     + c.powf(2.) * d.sin().powf(2.);
    //
    // Some(Aop::from_angle_wrapped(Angle::new::<radian>(angle)))
    //
    // Then the weighted_rmse_gradient can do:
    //
    // let gradient = ray_info
    //     .par_iter()
    //     .map(|ri| {
    //         let weighted_delta = ri.weight
    //             * ri.difference.into_inner().get::<radian>()
    //             * ri.partial_derivative.into_inner().get::<radian>();
    //
    //         weighted_delta
    //     })
    //     .sum::<f64>()
    //     / rays.len() as f64
    //     * 2.;

    fn weighted_rmse_gradient<In>(
        &self,
        measured: &RayImage<GlobalFrame>,
        simulated: &RayImage<GlobalFrame>,
    ) -> OrientationGradient<In> {
        todo!()
    }

    fn candidate<In>(
        &self,
        measured: &RayImage<GlobalFrame>,
        simulated: &RayImage<GlobalFrame>,
    ) -> Candidate<In> {
        todo!()
    }

    fn sensor_to_global<In>(
        image: RayImage<SensorFrame>,
        orientation: &Orientation<In>,
    ) -> RayImage<GlobalFrame> {
        todo!()
    }

    pub fn orientation_of<In>(
        &self,
        // If this is in the global frame, then we already have the orientation.
        image: RayImage<SensorFrame>,
        position: impl Into<Wgs84> + Clone,
        time: impl Into<DateTime<Utc>> + Clone,
    ) -> Result<Orientation<In>, EstimatorError>
    where
        O: Optic + Send + Sync,
    {
        let mut orientation = Orientation::aligned();
        let mut last_weighted_rmse_gradient_norm = 0.;
        for iteration in 0..self.max_iterations {
            let measured = Self::sensor_to_global(image.clone(), &orientation);
            let simulation = self.simulation(position.clone(), orientation, time.clone());
            let simulated = simulation.par_ray_image();

            let candidate = self.candidate(&measured, &simulated);
            last_weighted_rmse_gradient_norm = candidate.weighted_rmse_gradient_norm();

            println!(
                "[{iteration:05}/{:05}] weighted_rmse: {}, weighed_rmse_gradient_norm: {}",
                self.max_iterations, candidate.weighted_rmse, last_weighted_rmse_gradient_norm
            );

            if last_weighted_rmse_gradient_norm <= self.convergence_threshold {
                return Ok(orientation);
            }

            orientation = candidate.minimize_weighted_rmse();
        }

        Err(EstimatorError::FailedToConverge {
            iterations: self.max_iterations,
            convergence_threshold: self.convergence_threshold,
            convergence_parameter: last_weighted_rmse_gradient_norm,
        })
    }
}

struct OrientationGradient<In> {
    inner: Orientation<In>,
}

struct Candidate<In> {
    orientation: Orientation<In>,
    weighted_rmse: f64,
    weighted_rmse_gradient: OrientationGradient<In>,
}

impl<In> Candidate<In> {
    fn weighted_rmse_gradient_norm(&self) -> f64 {
        todo!()
    }

    fn minimize_weighted_rmse(&self) -> Orientation<In> {
        todo!()
    }
}

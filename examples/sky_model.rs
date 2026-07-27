//! Use a [`SkyModel`] to predict the [`Aop`] and [`Dop`] for a series of sun positions and sky
//! points.

use rumpus::model::SkyModel;
use sguaba::{Bearing, bearing, system};
use uom::si::{angle::degree, f64::Angle};

system!(struct Enu using ENU);

fn main() -> anyhow::Result<()> {
    let sample_bearings = iter_sample_bearings();
    let solar_bearings = iter_solar_bearings();

    println!(
        "solar_azimuth_deg,solar_elevation_deg,sample_azimuth_deg,sample_elevation_deg,aop_deg,dop"
    );
    for solar_bearing in solar_bearings {
        // Construct a SkyModel from the position of the sun in the sky.
        // Since the position of the sun can be inferred, this can also be done using a Wgs84 position and UTC timestamp.
        let sky_model = SkyModel::from_solar_bearing(solar_bearing);

        for sample_bearing in &sample_bearings {
            let aop = sky_model.aop(*sample_bearing);
            let dop = sky_model.dop(*sample_bearing);

            let solar_azimuth = solar_bearing.azimuth().get::<degree>();
            let solar_elevation = solar_bearing.elevation().get::<degree>();
            let sample_azimuth = sample_bearing.azimuth().get::<degree>();
            let sample_elevation = sample_bearing.elevation().get::<degree>();
            let aop = aop
                .map(|aop| Into::<Angle>::into(aop).get::<degree>())
                .unwrap_or(f64::NAN);
            let dop = dop.map(|dop| Into::<f64>::into(dop)).unwrap_or(f64::NAN);

            println!(
                "{solar_azimuth},{solar_elevation},{sample_azimuth},{sample_elevation},{aop},{dop}"
            );
        }
    }

    Ok(())
}

fn iter_solar_bearings() -> Vec<Bearing<Enu>> {
    vec![
        bearing!(azimuth = deg(45.0), elevation = deg(00.0); in Enu),
        bearing!(azimuth = deg(45.0), elevation = deg(30.0); in Enu),
        bearing!(azimuth = deg(45.0), elevation = deg(60.0); in Enu),
    ]
}

fn iter_sample_bearings() -> Vec<Bearing<Enu>> {
    let azimuths = (0..360)
        .map(|i| Angle::new::<degree>(i as f64))
        .collect::<Vec<_>>();
    let elevations = (0..90)
        .map(|i| Angle::new::<degree>(i as f64))
        .collect::<Vec<_>>();

    let mut result = Vec::with_capacity(azimuths.len() * elevations.len());
    for azimuth in azimuths {
        for elevation in elevations.clone() {
            result.push(
                Bearing::builder()
                    .azimuth(azimuth)
                    .elevation(elevation)
                    .expect("elevation is in [-90°, 90°]")
                    .build(),
            );
        }
    }

    result
}

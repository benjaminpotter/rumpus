pub struct StokesParams {
    px: (u32, u32),
    inner: [f64; 3],
}

impl StokesParams {
    pub fn new(px: (u32, u32), inner: [f64; 3]) -> Self {
        Self { px, inner }
    }
}

impl Into<Measurement> for StokesParams {
    fn into(self) -> Measurement {
        Measurement::new(
            self.px,
            (self.inner[2].atan2(self.inner[1]) / 2.).to_degrees(),
            (self.inner[1].powf(2.) + self.inner[2].powf(2.)).sqrt() / self.inner[0],
        )
    }
}

#[derive(Debug, PartialEq)]
pub struct Measurement {
    pixel_location: (u32, u32),
    aop: f64,
    dop: f64,
}

impl Measurement {
    pub fn new(pixel_location: (u32, u32), aop: f64, dop: f64) -> Self {
        Self {
            pixel_location,
            aop,
            dop,
        }
    }

    pub fn with_dop_max(mut self, max: f64) -> Self {
        self.dop = self.dop.clamp(0.0, max);
        self
    }

    pub fn get_pixel_location(&self) -> &(u32, u32) {
        &self.pixel_location
    }

    pub fn get_aop(&self) -> &f64 {
        &self.aop
    }

    pub fn get_dop(&self) -> &f64 {
        &self.dop
    }
}

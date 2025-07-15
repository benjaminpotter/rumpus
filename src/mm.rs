pub struct Measurement {
    pub pixel_location: (u32, u32),
    pub aop: f64,
    pub dop: f64,
}

impl Measurement {
    pub fn with_dop_max(mut self, max: f64) -> Self {
        self.dop = self.dop.clamp(0.0, max);
        self
    }
}

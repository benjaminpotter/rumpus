use rand::{distr::StandardUniform, Rng};
use rumpus::sensor::Pose;
use std::{
    fs::File,
    io::{BufWriter, Write},
};

fn main() {
    const COUNT: usize = 10;
    let file = File::create("poses.dat").expect("failed to open file");
    let mut writer = BufWriter::new(file);
    for pose in rand::rng()
        .sample_iter::<Pose, _>(StandardUniform)
        .take(COUNT)
    {
        let _ = writeln!(writer, "{} {} {}", pose.roll, pose.pitch, pose.yaw);
    }
}

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use rumpus::image::{Jet, RayMap};
use std::hint::black_box;

fn ray_map_jet(c: &mut Criterion) {
    static JET: Jet = Jet;
    static MIN: f64 = 0.0;
    static MAX: f64 = 1.0;

    let mut group = c.benchmark_group("jet");
    for value in [0.0, 0.5, 1.0].iter() {
        group.throughput(Throughput::Bytes(8));
        group.bench_with_input(BenchmarkId::new("map", value), value, |b, &value| {
            b.iter(|| JET.map(value, black_box(MIN), black_box(MAX)));
        });
    }
    group.finish();
}

criterion_group!(ray_map_benches, ray_map_jet);
criterion_main!(ray_map_benches);

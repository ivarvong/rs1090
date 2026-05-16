//! Magnitude-stage microbenchmarks.
//!
//! Sized for one millisecond of capture at 2 MS/s: 2000 samples. That's a
//! realistic chunk for the streaming pipeline; per-call overhead dominates
//! anything much smaller.

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use rs1090::test_utils::{batch_amam, batch_lut};
use rs1090::Iq;

fn sample_chunk(n: usize) -> Vec<Iq> {
    // Deterministic pseudo-random spread covering the full (i8, i8) range.
    let mut out = Vec::with_capacity(n);
    let mut x: u32 = 0x00C0_FFEE;
    for _ in 0..n {
        x = x.wrapping_mul(1_103_515_245).wrapping_add(12345);
        let i = ((x >> 16) & 0xFF) as i8;
        x = x.wrapping_mul(1_103_515_245).wrapping_add(12345);
        let q = ((x >> 16) & 0xFF) as i8;
        out.push(Iq::new(i, q));
    }
    out
}

fn bench_magnitude(c: &mut Criterion) {
    const N: usize = 2000;
    let samples = sample_chunk(N);
    let mut out = vec![0u16; N];

    let mut g = c.benchmark_group("magnitude");
    g.throughput(Throughput::Elements(N as u64));

    g.bench_function("alpha_max_beta_min", |b| {
        b.iter(|| {
            batch_amam(black_box(&samples), black_box(&mut out));
        });
    });

    g.bench_function("lut", |b| {
        b.iter(|| {
            batch_lut(black_box(&samples), black_box(&mut out));
        });
    });

    g.finish();
}

criterion_group!(benches, bench_magnitude);
criterion_main!(benches);

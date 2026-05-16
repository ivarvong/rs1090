//! CRC-24 microbenchmarks.
//!
//! The byte-at-a-time table-driven implementation is on every CRC-clean
//! frame and on every address-XOR recovery attempt against the active
//! aircraft set, so its per-call cost is multiplied by both the frame
//! rate and the active-set size on every ingest.
//!
//! Two sizes match the on-the-wire Mode S frame lengths: 7 bytes
//! (DF 0/4/5/11) and 14 bytes (DF 16/17/18/20/21).

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use rs1090::crc::{crc24, crc24_bitwise};

fn bench_crc(c: &mut Criterion) {
    // Real DF17 payload bytes (synth_df17_payload-shaped) so the table
    // probes a realistic distribution, not all-zeros.
    let short_frame: [u8; 7] = [0x5D, 0xA0, 0xBA, 0x4E, 0x12, 0x34, 0x56];
    let long_frame: [u8; 14] = [
        0x8D, 0xA0, 0xBA, 0x4E, 0x59, 0x09, 0xC3, 0x27, 0x9F, 0x7E, 0x82, 0xF7, 0x90, 0x6D,
    ];

    let mut g = c.benchmark_group("crc24");
    g.throughput(Throughput::Bytes(7));
    g.bench_function("table_short_7B", |b| {
        b.iter(|| crc24(black_box(&short_frame)));
    });
    g.throughput(Throughput::Bytes(14));
    g.bench_function("table_long_14B", |b| {
        b.iter(|| crc24(black_box(&long_frame)));
    });

    // Bitwise is the fallback for memory-starved targets where the 1 KiB
    // table would cause thrashing. ~8× slower per byte but no table
    // footprint.
    g.throughput(Throughput::Bytes(7));
    g.bench_function("bitwise_short_7B", |b| {
        b.iter(|| crc24_bitwise(black_box(&short_frame)));
    });
    g.throughput(Throughput::Bytes(14));
    g.bench_function("bitwise_long_14B", |b| {
        b.iter(|| crc24_bitwise(black_box(&long_frame)));
    });
    g.finish();
}

criterion_group!(benches, bench_crc);
criterion_main!(benches);

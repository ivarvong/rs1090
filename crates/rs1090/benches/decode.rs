//! `message::decode` microbenchmark.
//!
//! Every detected frame runs through `message::decode` once. The path
//! is pure CPU (no I/O, no allocation in the decoder); the cost is
//! dominated by the DF/TC dispatch and the bit-field extraction via
//! `BitReader`. Numbers here translate directly to per-frame overhead
//! at the message layer.

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use rs1090::frame::Frame;
use rs1090::message::decode;

fn bench_decode(c: &mut Criterion) {
    let mut g = c.benchmark_group("decode");

    // DF17 airborne position (TC 11). Real bytes from a captured frame
    // over NYC.
    let pos: [u8; 14] = [
        0x8F, 0xA4, 0xCA, 0xF6, 0x59, 0x09, 0xC3, 0x27, 0x9F, 0x7E, 0x82, 0xF7, 0x90, 0x6D,
    ];
    let pos_frame = Frame::from_bytes(&pos);
    g.bench_function("df17_airborne_position", |b| {
        b.iter(|| decode(black_box(&pos_frame)));
    });

    // DF17 airborne velocity (TC 19, subtype 1, ground velocity).
    let vel: [u8; 14] = [
        0x8D, 0xA0, 0xBA, 0x4E, 0x99, 0x88, 0x76, 0x0D, 0xC8, 0x0C, 0x84, 0xD6, 0x74, 0x45,
    ];
    let vel_frame = Frame::from_bytes(&vel);
    g.bench_function("df17_velocity", |b| {
        b.iter(|| decode(black_box(&vel_frame)));
    });

    // DF11 all-call reply. Short frame; the cheap path.
    let allcall: [u8; 7] = [0x5D, 0xA0, 0xBA, 0x4E, 0x12, 0x34, 0x56];
    let allcall_frame = Frame::from_bytes(&allcall);
    g.bench_function("df11_allcall", |b| {
        b.iter(|| decode(black_box(&allcall_frame)));
    });

    g.finish();
}

criterion_group!(benches, bench_decode);
criterion_main!(benches);

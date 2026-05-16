//! Fuzz target: feed arbitrary bytes as `Iq` samples into a fresh
//! `FrameDetector` and verify the entire IQ → magnitude → preamble →
//! slice → CRC pipeline never panics. Each pair of input bytes becomes
//! one `(i, q)` sample, so libFuzzer can explore the full signal-space
//! the live SDR pipeline ever sees.
//!
//! First-time setup (from `crates/rs1090/fuzz`):
//!
//!     mkdir -p corpus/process_frame
//!     cp -n seeds/process_frame/* corpus/process_frame/
//!
//! Then on every run:
//!
//!     cargo +nightly fuzz run process_frame

#![no_main]

use libfuzzer_sys::fuzz_target;
use rs1090::frame::FrameDetector;
use rs1090::Iq;

fuzz_target!(|data: &[u8]| {
    // Need at least one full sample pair to do anything interesting.
    if data.len() < 2 {
        return;
    }
    // Cap the per-iteration work so a malicious input can't make a
    // single execution take seconds; the fuzzer wants high throughput.
    let cap = data.len().min(8 * 1024);
    let samples: Vec<Iq> = data[..cap]
        .chunks_exact(2)
        .map(|c| Iq::new(c[0] as i8, c[1] as i8))
        .collect();
    let mut det = FrameDetector::with_chunk_capacity(samples.len());
    det.process(&samples, |_| {});
});

//! Fuzz target: build a `Frame` from arbitrary 7- or 14-byte input and
//! call [`rs1090::message::decode`]. The decoder dispatches on DF and
//! TypeCode through many manual bit-shift paths; the goal is to surface
//! any input that causes a panic — integer overflow, slice OOB, unwrap
//! on `None`, debug-assert, etc.
//!
//! First-time setup (from `crates/rs1090/fuzz`):
//!
//!     mkdir -p corpus/decode_message
//!     cp -n seeds/decode_message/* corpus/decode_message/
//!
//! Then on every run:
//!
//!     cargo +nightly fuzz run decode_message
//!
//! `seeds/decode_message/` is the committed read-only seed corpus
//! (real frames from a live capture); `corpus/decode_message/` is
//! libFuzzer's writable working directory, gitignored. The one-shot
//! seed copy keeps the canonical seed set immutable while letting
//! libFuzzer accumulate coverage-discovered inputs across runs.
//!
//! Each input is a flat byte buffer; we accept both short (7-byte) and
//! long (14-byte) frame shapes, mapping any other length to "skip" so
//! libFuzzer converges quickly on valid sizes.

#![no_main]

use libfuzzer_sys::fuzz_target;
use rs1090::frame::Frame;
use rs1090::message::decode;

fuzz_target!(|data: &[u8]| {
    if data.len() != 7 && data.len() != 14 {
        return;
    }
    let frame = Frame::from_bytes(data);
    // We don't care about the decode outcome — only that it terminates
    // without panicking on any input. Errors are expected for malformed
    // payloads; that's the whole point.
    let _ = decode(&frame);
});

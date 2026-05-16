//! Fuzz target: build a `Frame` from arbitrary 7- or 14-byte input and
//! call [`rs1090::message::decode`]. The decoder dispatches on DF and
//! TypeCode through many manual bit-shift paths; the goal is to surface
//! any input that causes a panic — integer overflow, slice OOB, unwrap
//! on `None`, debug-assert, etc.
//!
//! Run via
//!
//!     cargo +nightly fuzz run decode_message seeds/decode_message
//!
//! from `crates/rs1090/fuzz`. The committed `seeds/decode_message/`
//! directory contains real frames extracted from a live capture; on
//! first invocation libFuzzer copies these into its working `corpus/`
//! (gitignored) and grows it from there.
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

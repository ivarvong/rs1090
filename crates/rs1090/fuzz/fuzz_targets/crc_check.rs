//! Fuzz target: feed arbitrary 7- or 14-byte buffers to `crc::check`,
//! which validates the syndrome and, on a 1-bit error, *mutates the
//! buffer in place* by flipping the recovered bit. The combination of
//! input dependence and side-effects is a classic place for off-by-one
//! and OOB bugs to hide.
//!
//! First-time setup (from `crates/rs1090/fuzz`):
//!
//!     mkdir -p corpus/crc_check
//!     cp -n seeds/crc_check/* corpus/crc_check/
//!
//! Then on every run:
//!
//!     cargo +nightly fuzz run crc_check

#![no_main]

use libfuzzer_sys::fuzz_target;
use rs1090::crc;

fuzz_target!(|data: &[u8]| {
    let mut buf = match data.len() {
        7 | 14 => data.to_vec(),
        _ => return,
    };
    let _ = crc::check(&mut buf);
});

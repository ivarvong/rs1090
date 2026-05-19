//! `rs1090` — a Mode S / ADS-B decoder.
//!
//! The library is structured as a stack of layers, each independently testable:
//!
//! ```text
//! Sample Source  →  Magnitude  →  Demodulator  →  Frame Detector  →  Message Decoder  →  State Tracker
//! ```
//!
//! Each layer exposes a small, typed interface. The top-level decoder wires
//! the default stack; advanced users can swap any layer.
//!
//! See `DESIGN.md` in the repository for the architecture and rationale.

#![cfg_attr(not(feature = "std"), no_std)]
#![doc(html_root_url = "https://docs.rs/rs1090")]

pub mod cpr;
pub mod crc;
pub mod frame;
pub mod message;

// Implementation detail modules. The signal-processing primitives
// (`demod`, `magnitude`) are deliberately *not* part of the public
// API — their shape is load-bearing for the frame-detection pipeline
// and must remain free to evolve with the implementation. Consumers
// that need access for tests or benchmarks should enable the
// `test-utils` feature and use [`test_utils`].
pub(crate) mod demod;
pub(crate) mod demod_2400;
pub(crate) mod magnitude;

#[cfg(feature = "std")]
pub mod source;
#[cfg(feature = "std")]
pub mod state;

/// Helpers re-exported for integration tests, fuzz harnesses, and
/// benchmarks. Enabled by the `test-utils` feature. **Not** part of the
/// stable API — items here may change shape in any release.
#[cfg(feature = "test-utils")]
pub mod test_utils {
    pub use crate::demod::{
        synth_bits_as_magnitude, synth_preamble, PREAMBLE_HIGH_IDX, PREAMBLE_SAMPLES,
        SAMPLES_PER_BIT,
    };
    pub use crate::magnitude::{batch_amam, batch_lut};
}

/// A complex I/Q sample with signed 8-bit components.
///
/// This is the canonical sample type throughout the library. SDR backends
/// convert their native format (RTL-SDR's unsigned 8-bit biased at 127, for
/// example) into this signed representation before handing samples downstream.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(C)]
pub struct Iq {
    pub i: i8,
    pub q: i8,
}

impl Iq {
    #[inline]
    pub const fn new(i: i8, q: i8) -> Self {
        Self { i, q }
    }
}

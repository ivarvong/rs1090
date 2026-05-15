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

pub mod magnitude;

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

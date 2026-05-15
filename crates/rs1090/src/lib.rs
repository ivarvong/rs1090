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

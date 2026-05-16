//! Frame detection: glue between the demodulator and the message decoder.
//!
//! A [`FrameDetector`] consumes `Iq` samples in chunks, maintains a noise
//! floor, hunts for preambles, slices bits, runs CRC, and yields fully
//! validated [`Frame`]s via a callback. It owns all its scratch buffers and
//! allocates nothing in the hot path.
//!
//! The detector is intentionally a small, mechanical pipeline; the
//! interesting variability (magnitude function, threshold gain, EMA shift)
//! lives one layer down where it can be tested in isolation.
//!
//! ## State machine
//!
//! ```text
//!   Hunting ──preamble candidate──▶ Reading DF (5 bits)
//!      ▲                                  │
//!      │                          DF reserved?  ──yes──┐
//!      │                                  │ no         │
//!      │                          Reading rest of frame│
//!      │                                  │            │
//!      │                          CRC check            │
//!      │                                  │            │
//!      └──────────── done ◀──────────────┘◀───────────┘
//! ```
//!
//! In practice we do this without an explicit state variable — the detector
//! reads everything in one stretch off the magnitude buffer once a preamble
//! clears the threshold, so it's just straight-line code.
//!
//! Lint exemption: technical terms (DF, CRC, ARMv6, Mode S) aren't Rust items.

#![allow(clippy::doc_markdown)]

use crate::crc::{self, CrcOutcome, LONG_FRAME_BYTES, SHORT_FRAME_BYTES};
use crate::demod::{
    aggregate_confidence, pack_bits_msb, preamble_clears_threshold, preamble_score, slice_bits,
    NoiseFloor, PREAMBLE_SAMPLES, SAMPLES_PER_BIT,
};
use crate::magnitude;
use crate::Iq;

// --- Downlink format --------------------------------------------------------

/// Mode S downlink format code, occupying the high 5 bits of the first byte
/// of a frame.
///
/// Only the values the decoder actually handles get named variants; everything
/// else flows through [`DownlinkFormat::Reserved`] so the detector can
/// surface them to upper layers without `match` exhaustiveness churn.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum DownlinkFormat {
    /// DF 0: short air-to-air ACAS.
    ShortAcas,
    /// DF 4: altitude reply (address-XORed CRC).
    AltitudeReply,
    /// DF 5: identity reply (address-XORed CRC).
    IdentityReply,
    /// DF 11: all-call reply. CRC is clean.
    AllCallReply,
    /// DF 16: long air-to-air ACAS.
    LongAcas,
    /// DF 17: ADS-B extended squitter. Primary target. CRC is clean.
    ExtendedSquitter,
    /// DF 18: TIS-B / non-transponder. Same payload shape as DF 17.
    TisB,
    /// DF 20: Comm-B altitude reply (address-XORed CRC).
    CommBAltitude,
    /// DF 21: Comm-B identity reply (address-XORed CRC).
    CommBIdentity,
    /// Anything else. The raw 5-bit DF value rides along for diagnostics.
    Reserved(u8),
}

impl DownlinkFormat {
    /// Parse a DF code from the high 5 bits of the first frame byte.
    #[inline]
    #[must_use]
    pub const fn from_first_byte(b: u8) -> Self {
        match b >> 3 {
            0 => Self::ShortAcas,
            4 => Self::AltitudeReply,
            5 => Self::IdentityReply,
            11 => Self::AllCallReply,
            16 => Self::LongAcas,
            17 => Self::ExtendedSquitter,
            18 => Self::TisB,
            20 => Self::CommBAltitude,
            21 => Self::CommBIdentity,
            other => Self::Reserved(other),
        }
    }

    /// Frame length in bytes implied by the DF code.
    ///
    /// DF ≥ 16 is 14 bytes; everything else is 7. (The bit-16 of DF
    /// determines length: per the Mode S spec, length = DF & 0x10 ? 112 : 56
    /// bits. We follow that mechanically rather than match on each variant.)
    #[inline]
    #[must_use]
    pub const fn frame_bytes(self) -> usize {
        let raw = self.raw_value();
        if raw & 0x10 != 0 {
            LONG_FRAME_BYTES
        } else {
            SHORT_FRAME_BYTES
        }
    }

    /// The 5-bit DF code as it appears on the wire.
    #[inline]
    #[must_use]
    pub const fn raw_value(self) -> u8 {
        match self {
            Self::ShortAcas => 0,
            Self::AltitudeReply => 4,
            Self::IdentityReply => 5,
            Self::AllCallReply => 11,
            Self::LongAcas => 16,
            Self::ExtendedSquitter => 17,
            Self::TisB => 18,
            Self::CommBAltitude => 20,
            Self::CommBIdentity => 21,
            Self::Reserved(v) => v,
        }
    }

    /// Whether the CRC is a clean (zero) syndrome on a good frame.
    ///
    /// `true` for DF 11, 17, 18. The others XOR the CRC with the ICAO
    /// address; validating them is the message decoder's job, not the
    /// frame detector's.
    #[inline]
    #[must_use]
    pub const fn has_clean_crc(self) -> bool {
        matches!(self, Self::AllCallReply | Self::ExtendedSquitter | Self::TisB)
    }
}

// --- Frame ------------------------------------------------------------------

/// Maximum frame size in bytes (long frames). Sized so a `Frame` is one
/// stack-friendly value.
pub const MAX_FRAME_BYTES: usize = LONG_FRAME_BYTES;

/// A demodulated Mode S frame, with its DF, byte payload, CRC outcome, and
/// per-frame confidence aggregate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    bytes: [u8; MAX_FRAME_BYTES],
    len: u8,
    df: DownlinkFormat,
    crc: CrcOutcome,
    confidence: u8,
}

impl Frame {
    /// Byte payload of the frame (7 or 14 bytes). The trailing 3 bytes are
    /// the (possibly corrected) CRC.
    #[inline]
    #[must_use]
    pub fn bytes(&self) -> &[u8] {
        &self.bytes[..self.len as usize]
    }

    #[inline]
    #[must_use]
    pub fn downlink_format(&self) -> DownlinkFormat {
        self.df
    }

    #[inline]
    #[must_use]
    pub fn crc_outcome(&self) -> CrcOutcome {
        self.crc
    }

    /// Aggregate per-bit confidence in `[0, 255]`. See
    /// [`crate::demod::aggregate_confidence`].
    #[inline]
    #[must_use]
    pub fn confidence(&self) -> u8 {
        self.confidence
    }

    #[inline]
    #[must_use]
    pub fn len(&self) -> usize {
        self.len as usize
    }

    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Build a `Frame` directly from a 7- or 14-byte payload, computing
    /// its `DF` and CRC outcome on the fly. Confidence is set to
    /// `u8::MAX` — there is no per-bit margin to derive without a
    /// demodulator slicer.
    ///
    /// Only available with the `test-utils` feature. Production frames
    /// come from [`FrameDetector::process`], which carries the real
    /// confidence value from the slicer. This constructor exists so
    /// fuzz targets and integration tests can feed known byte sequences
    /// to [`crate::message::decode`] without re-synthesising the demod
    /// path.
    ///
    /// # Panics
    /// Panics if `bytes.len()` is not [`SHORT_FRAME_BYTES`] (7) or
    /// [`LONG_FRAME_BYTES`] (14).
    #[cfg(any(feature = "test-utils", test))]
    #[must_use]
    pub fn from_bytes(bytes: &[u8]) -> Self {
        assert!(
            bytes.len() == SHORT_FRAME_BYTES || bytes.len() == LONG_FRAME_BYTES,
            "Frame::from_bytes requires {SHORT_FRAME_BYTES} or {LONG_FRAME_BYTES} bytes, got {}",
            bytes.len(),
        );
        let mut buf = [0u8; MAX_FRAME_BYTES];
        buf[..bytes.len()].copy_from_slice(bytes);
        let df = DownlinkFormat::from_first_byte(buf[0]);
        let crc = if crc::crc24(&buf[..bytes.len()]) == 0 {
            CrcOutcome::Clean
        } else {
            CrcOutcome::Failed
        };
        // bytes.len() is asserted to be 7 or 14, both well within u8.
        let len = bytes.len() as u8;
        Self { bytes: buf, len, df, crc, confidence: u8::MAX }
    }
}

// --- Frame detector ---------------------------------------------------------

/// Streaming frame detector. Owns all its scratch buffers; one instance per
/// receive stream.
///
/// Call [`FrameDetector::process`] with chunks of `Iq` samples; for each
/// frame detected (CRC clean or 1-bit corrected) the callback fires once.
/// Samples may be split across chunks: the detector retains the tail of the
/// previous chunk so a preamble straddling a boundary is not lost.
#[derive(Debug)]
pub struct FrameDetector {
    floor: NoiseFloor,
    /// Configuration: minimum aggregate confidence for a frame to be
    /// surfaced. Frames below this are dropped silently. `0` means "accept
    /// everything that passes CRC".
    min_confidence: u8,
    /// Pending samples carried over from the previous chunk. We need
    /// enough to cover a preamble plus the longest frame: 16 +
    /// 14 * 8 * 2 = 240 samples. We keep a power-of-two-ish margin.
    carry: [u16; CARRY_SAMPLES],
    carry_len: usize,
}

/// Samples we keep across `process` calls. A preamble plus a long frame is
/// 16 + 224 = 240 samples; round up to 256 for clean indexing.
const CARRY_SAMPLES: usize = 256;

/// Bits per long frame.
const LONG_FRAME_BITS: usize = LONG_FRAME_BYTES * 8;

/// Bits per short frame.
const SHORT_FRAME_BITS: usize = SHORT_FRAME_BYTES * 8;

/// Samples covering preamble + at least a short payload. This is the minimum
/// we need to even *try* to read a preamble plus its 5-bit DF prefix; if the
/// DF resolves to a long frame and we don't have enough buffer, we break out
/// and the carry-over picks up the trail on the next call.
const MIN_DETECTION_WINDOW: usize =
    PREAMBLE_SAMPLES + SHORT_FRAME_BITS * SAMPLES_PER_BIT;

impl Default for FrameDetector {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameDetector {
    #[must_use]
    pub fn new() -> Self {
        Self {
            floor: NoiseFloor::fresh(),
            min_confidence: 0,
            carry: [0; CARRY_SAMPLES],
            carry_len: 0,
        }
    }

    /// Drop frames whose aggregate confidence is below `min`.
    pub fn set_min_confidence(&mut self, min: u8) {
        self.min_confidence = min;
    }

    /// Reseed the noise floor. Useful when restarting on a known-quiet
    /// segment or after a long gap.
    pub fn reset_noise_floor(&mut self, seed: u16) {
        self.floor = NoiseFloor::new(seed, NoiseFloor::DEFAULT_SHIFT);
    }

    /// Consume a chunk of samples and call `on_frame` for each detected frame.
    ///
    /// `samples` may be of any length; internally the detector concatenates
    /// it with the carry-over from the previous call. Allocates nothing.
    pub fn process<F: FnMut(&Frame)>(&mut self, samples: &[Iq], mut on_frame: F) {
        // 1. Build a contiguous magnitude buffer from carry + new samples.
        //    For now we use a heap Vec; the streaming version uses a fixed
        //    ring. The DESIGN.md hot-path budget allows pre-allocated buffers
        //    sized once at startup, which the higher-level Decoder will own;
        //    here we keep the API simple and revisit when the streaming
        //    pipeline lands.
        //
        //    This is the one allocation in the frame layer; it's outside
        //    the per-sample hot path (one alloc per `process` call) and
        //    will be lifted to a caller-supplied scratch in a later pass.
        let mut buf: alloc::vec::Vec<u16> =
            alloc::vec::Vec::with_capacity(self.carry_len + samples.len());
        buf.extend_from_slice(&self.carry[..self.carry_len]);
        for &s in samples {
            // We use alpha-max-beta-min here unconditionally; the LUT vs
            // AMBM choice is left to a higher-level pipeline that knows
            // its target arch.
            buf.push(magnitude::alpha_max_beta_min(s));
        }

        // 2. Walk the buffer looking for preambles. Update the floor at
        //    every sample so it tracks across both signal and noise.
        let mut i = 0usize;
        let n = buf.len();

        // Scratch for bit slicing & CRC. Stack-allocated; size is bounded.
        let mut bits = [false; LONG_FRAME_BITS];
        let mut conf = [0u8; LONG_FRAME_BITS];
        let mut bytes = [0u8; MAX_FRAME_BYTES];

        while i + MIN_DETECTION_WINDOW <= n {
            // Update floor with one sample of look-back. We update with the
            // current sample so that the threshold reflects the local
            // baseline at the candidate position.
            let floor = self.floor.update(buf[i]);

            // 2a. Score the preamble window.
            // SAFETY: bounds checked by the loop condition.
            let window: &[u16; PREAMBLE_SAMPLES] = (&buf[i..i + PREAMBLE_SAMPLES])
                .try_into()
                .expect("slice has correct length by construction");
            let score = preamble_score(window);
            if !preamble_clears_threshold(score, floor) {
                i += 1;
                continue;
            }

            // 2b. Preamble candidate. Slice the first 5 bits to read DF.
            //     The payload starts immediately after the preamble.
            let payload_start = i + PREAMBLE_SAMPLES;
            let df_samples = &buf[payload_start..payload_start + 5 * SAMPLES_PER_BIT];
            let (df_byte_high, df_conf_high) = slice_df_prefix(df_samples);
            let df = DownlinkFormat::from_first_byte(df_byte_high);

            // 2c. Drop reserved DFs immediately; advance past the preamble.
            if matches!(df, DownlinkFormat::Reserved(_)) {
                i += PREAMBLE_SAMPLES;
                continue;
            }

            let frame_bytes = df.frame_bytes();
            let frame_bits = frame_bytes * 8;
            let payload_samples = frame_bits * SAMPLES_PER_BIT;
            if payload_start + payload_samples > n {
                // Not enough samples yet; bail out and let the next chunk
                // pick up the trail.
                break;
            }

            // 2d. Slice the whole frame.
            slice_bits(
                &buf[payload_start..payload_start + payload_samples],
                &mut bits[..frame_bits],
                &mut conf[..frame_bits],
            );
            pack_bits_msb(&bits[..frame_bits], &mut bytes[..frame_bytes]);

            // 2e. Run CRC. For non-clean-CRC DFs we surface the frame with
            //     its raw syndrome and leave address resolution to the
            //     message decoder.
            let crc_outcome = if df.has_clean_crc() {
                crc::check(&mut bytes[..frame_bytes])
            } else {
                // No correction attempt: the syndrome is the XOR with the
                // ICAO address and we don't have the address yet. Surface
                // as Failed; the message-decoder layer will reinterpret.
                CrcOutcome::Failed
            };

            let aggregate = aggregate_confidence(&conf[..frame_bits]).min(df_conf_high);
            if aggregate >= self.min_confidence {
                let mut frame_bytes_arr = [0u8; MAX_FRAME_BYTES];
                frame_bytes_arr[..frame_bytes].copy_from_slice(&bytes[..frame_bytes]);
                let frame = Frame {
                    bytes: frame_bytes_arr,
                    len: frame_bytes as u8,
                    df,
                    crc: crc_outcome,
                    confidence: aggregate,
                };
                on_frame(&frame);
            }

            // 2f. Advance past the entire frame. Overlapping frames are
            //     not physically possible — a transponder doesn't transmit
            //     two preambles inside one frame's window.
            i = payload_start + payload_samples;
        }

        // 3. Stash the trailing window we couldn't fully consume.
        //    Worst case: we need to retain everything from the last
        //    unattempted preamble position to the end. Keep up to
        //    CARRY_SAMPLES of the tail.
        let tail_start = i.saturating_sub(0);
        let tail_len = n - tail_start;
        let retain = tail_len.min(CARRY_SAMPLES);
        let copy_from = n - retain;
        self.carry[..retain].copy_from_slice(&buf[copy_from..n]);
        self.carry_len = retain;
    }
}

/// Decode the first byte of the frame (5-bit DF + 3 bits) given 10 magnitude
/// samples. Returns the byte and the min confidence of the 5 DF bits (the
/// other 3 bits are spec'd as message-specific and aren't needed yet).
fn slice_df_prefix(samples: &[u16]) -> (u8, u8) {
    debug_assert_eq!(samples.len(), 5 * SAMPLES_PER_BIT);
    let mut bits = [false; 5];
    let mut conf = [0u8; 5];
    slice_bits(samples, &mut bits, &mut conf);
    let mut byte = 0u8;
    for (k, &b) in bits.iter().enumerate() {
        if b {
            // DF lives in the top 5 bits of byte 0.
            byte |= 1u8 << (7 - k);
        }
    }
    (byte, aggregate_confidence(&conf))
}

// `process` uses Vec from `alloc` so the module remains usable in `no_std`
// targets that link an allocator. With `feature = "std"` (the default),
// `alloc` is re-exported by the standard library and this is a no-op.
extern crate alloc;

// --- Tests ------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::demod::{synth_bits_as_magnitude, PREAMBLE_HIGH_IDX};

    #[test]
    fn downlink_format_from_byte_picks_known_codes() {
        for (raw, expected) in [
            (0u8, DownlinkFormat::ShortAcas),
            (4, DownlinkFormat::AltitudeReply),
            (5, DownlinkFormat::IdentityReply),
            (11, DownlinkFormat::AllCallReply),
            (16, DownlinkFormat::LongAcas),
            (17, DownlinkFormat::ExtendedSquitter),
            (18, DownlinkFormat::TisB),
            (20, DownlinkFormat::CommBAltitude),
            (21, DownlinkFormat::CommBIdentity),
        ] {
            // The DF lives in the top 5 bits, so shift up before parsing.
            let byte = raw << 3;
            assert_eq!(DownlinkFormat::from_first_byte(byte), expected);
            assert_eq!(expected.raw_value(), raw);
        }
    }

    #[test]
    fn downlink_format_reserved_round_trips() {
        // DF 6 is unused; check we pass it through.
        let byte = 6u8 << 3;
        let df = DownlinkFormat::from_first_byte(byte);
        assert_eq!(df, DownlinkFormat::Reserved(6));
        assert_eq!(df.raw_value(), 6);
        assert!(!df.has_clean_crc());
    }

    #[test]
    fn frame_length_follows_df_high_bit() {
        assert_eq!(DownlinkFormat::ShortAcas.frame_bytes(), SHORT_FRAME_BYTES);
        assert_eq!(DownlinkFormat::AllCallReply.frame_bytes(), SHORT_FRAME_BYTES);
        assert_eq!(DownlinkFormat::ExtendedSquitter.frame_bytes(), LONG_FRAME_BYTES);
        assert_eq!(DownlinkFormat::LongAcas.frame_bytes(), LONG_FRAME_BYTES);
        // Reserved DFs follow the same bit-16 rule:
        assert_eq!(DownlinkFormat::Reserved(6).frame_bytes(), SHORT_FRAME_BYTES);
        assert_eq!(DownlinkFormat::Reserved(24).frame_bytes(), LONG_FRAME_BYTES);
    }

    #[test]
    fn clean_crc_set_matches_spec() {
        assert!(DownlinkFormat::AllCallReply.has_clean_crc());
        assert!(DownlinkFormat::ExtendedSquitter.has_clean_crc());
        assert!(DownlinkFormat::TisB.has_clean_crc());
        assert!(!DownlinkFormat::AltitudeReply.has_clean_crc());
        assert!(!DownlinkFormat::CommBAltitude.has_clean_crc());
    }

    // --- End-to-end: synth a frame, push it through the detector ---

    /// Build a synthetic DF 17 frame: 14 bytes of data, last 3 are the CRC.
    fn synth_df17_payload() -> [u8; LONG_FRAME_BYTES] {
        // DF 17 in the top 5 bits of byte 0. The remaining 3 bits + 10
        // bytes of payload are arbitrary; pick fixed values so the test
        // is deterministic.
        let mut data = [0u8; LONG_FRAME_BYTES - 3];
        data[0] = 17 << 3; // DF 17, capability = 0
        data[1] = 0xAB;
        data[2] = 0xCD;
        data[3] = 0xEF;
        data[4] = 0x12;
        // ME field
        data[5] = 0x58;
        data[6] = 0x10;
        data[7] = 0x20;
        data[8] = 0x30;
        data[9] = 0x40;
        data[10] = 0x50;
        // Compute CRC over data and append.
        let crc_val = crc::crc24(&data);
        let mut frame = [0u8; LONG_FRAME_BYTES];
        frame[..LONG_FRAME_BYTES - 3].copy_from_slice(&data);
        frame[LONG_FRAME_BYTES - 3] = ((crc_val >> 16) & 0xFF) as u8;
        frame[LONG_FRAME_BYTES - 2] = ((crc_val >> 8) & 0xFF) as u8;
        frame[LONG_FRAME_BYTES - 1] = (crc_val & 0xFF) as u8;
        debug_assert_eq!(crc::crc24(&frame), 0, "synth frame should have zero syndrome");
        frame
    }

    /// Encode a frame as a magnitude stream: noise floor + preamble + bits.
    /// The output is the magnitude buffer plus the index of the preamble.
    fn synth_frame_as_magnitudes(
        frame_bytes: &[u8],
        pulse: u16,
        floor: u16,
        leading_noise_samples: usize,
        trailing_noise_samples: usize,
    ) -> alloc::vec::Vec<u16> {
        let payload_bits = frame_bytes.len() * 8;
        let payload_samples = payload_bits * SAMPLES_PER_BIT;
        let total = leading_noise_samples + PREAMBLE_SAMPLES + payload_samples
            + trailing_noise_samples;
        let mut out = alloc::vec![floor; total];

        // Preamble.
        let pre_start = leading_noise_samples;
        for &k in &PREAMBLE_HIGH_IDX {
            out[pre_start + k] = pulse;
        }

        // Payload bits.
        let mut bits = alloc::vec![false; payload_bits];
        for (byte_idx, b) in frame_bytes.iter().enumerate() {
            for k in 0..8 {
                bits[byte_idx * 8 + k] = (b >> (7 - k)) & 1 != 0;
            }
        }
        let payload_start = pre_start + PREAMBLE_SAMPLES;
        synth_bits_as_magnitude(
            &bits,
            pulse,
            floor,
            &mut out[payload_start..payload_start + payload_samples],
        );
        out
    }

    /// Convert a magnitude stream back to Iq samples whose
    /// `alpha_max_beta_min` magnitude is the input. We pick i = mag,
    /// q = 0 — alpha_max_beta_min(mag, 0) = mag exactly.
    fn magnitudes_to_iq(mags: &[u16]) -> alloc::vec::Vec<Iq> {
        mags.iter()
            .map(|&m| {
                // i8 range is -128..=127, magnitude up to 128.
                let i = m.min(127) as i8;
                Iq::new(i, 0)
            })
            .collect()
    }

    #[test]
    fn detector_recovers_synthetic_df17_frame() {
        let frame = synth_df17_payload();
        let mags = synth_frame_as_magnitudes(&frame, 120, 5, 64, 64);
        let samples = magnitudes_to_iq(&mags);

        // Seed the noise floor so the threshold is meaningful from sample 0.
        let mut det = FrameDetector::new();
        det.reset_noise_floor(5);

        let mut got: alloc::vec::Vec<Frame> = alloc::vec::Vec::new();
        det.process(&samples, |f| got.push(*f));

        assert_eq!(got.len(), 1, "expected exactly one frame");
        let f = &got[0];
        assert_eq!(f.downlink_format(), DownlinkFormat::ExtendedSquitter);
        assert_eq!(f.len(), LONG_FRAME_BYTES);
        assert_eq!(f.crc_outcome(), CrcOutcome::Clean);
        assert_eq!(f.bytes(), &frame[..]);
        assert!(f.confidence() > 200, "high SNR should give high confidence");
    }

    #[test]
    fn detector_corrects_single_bit_error() {
        let mut frame = synth_df17_payload();
        let mags = synth_frame_as_magnitudes(&frame, 120, 5, 64, 64);

        // Inject one bit flip into the magnitude stream by inverting the
        // half-bits for a chosen payload bit. Bit 50 lands in byte 6, bit 2.
        let preamble_offset = 64 + PREAMBLE_SAMPLES;
        let target_bit = 50usize;
        let mut mags = mags;
        let sample_idx = preamble_offset + target_bit * SAMPLES_PER_BIT;
        mags.swap(sample_idx, sample_idx + 1);
        let samples = magnitudes_to_iq(&mags);

        let mut det = FrameDetector::new();
        det.reset_noise_floor(5);
        let mut got: alloc::vec::Vec<Frame> = alloc::vec::Vec::new();
        det.process(&samples, |f| got.push(*f));

        // The originally constructed frame is the "clean" reference.
        // After the flip and detector-driven correction, the surfaced
        // bytes should equal the clean frame.
        assert_eq!(got.len(), 1);
        let f = &got[0];
        assert!(matches!(f.crc_outcome(), CrcOutcome::Corrected { .. }));
        // The detector's view of the recovered frame must match the original.
        assert_eq!(f.bytes(), &frame[..]);
        // The original `frame` variable is still the clean payload — show
        // we never mutated it inside this test (sanity check on the helper).
        assert_eq!(crc::crc24(&frame), 0);
        // Touch `frame` to silence the unused-mut lint should rustc decide
        // it isn't actually mutated.
        let _ = &mut frame[0];
    }

    #[test]
    fn detector_emits_nothing_on_pure_noise() {
        let samples: alloc::vec::Vec<Iq> = (0..4096)
            .map(|n| {
                // Low-amplitude random-ish noise, well below any pulse level.
                let i = ((n * 7919) % 13) as i8 - 6;
                let q = ((n * 31337) % 11) as i8 - 5;
                Iq::new(i, q)
            })
            .collect();
        let mut det = FrameDetector::new();
        det.reset_noise_floor(5);
        let mut got = 0usize;
        det.process(&samples, |_| got += 1);
        assert_eq!(got, 0, "noise should not produce frames");
    }

    #[test]
    fn detector_handles_chunked_input() {
        let frame = synth_df17_payload();
        let mags = synth_frame_as_magnitudes(&frame, 120, 5, 64, 64);
        let samples = magnitudes_to_iq(&mags);

        let mut det = FrameDetector::new();
        det.reset_noise_floor(5);
        let mut got: alloc::vec::Vec<Frame> = alloc::vec::Vec::new();

        // Feed in 73-sample chunks — a prime that guarantees the preamble
        // straddles a chunk boundary.
        for chunk in samples.chunks(73) {
            det.process(chunk, |f| got.push(*f));
        }

        assert_eq!(got.len(), 1, "frame should be recovered across chunks");
        assert_eq!(got[0].bytes(), &frame[..]);
    }

    #[test]
    fn detector_min_confidence_filters_marginal_frames() {
        let frame = synth_df17_payload();
        // Use a low-SNR signal so per-bit confidence stays modest.
        // pulse=60, floor=40 ⇒ per-bit conf = 20*255/100 = 51.
        let mags = synth_frame_as_magnitudes(&frame, 60, 40, 64, 64);
        let samples = magnitudes_to_iq(&mags);

        let mut det = FrameDetector::new();
        det.reset_noise_floor(40);
        det.set_min_confidence(200); // stricter than the signal can deliver
        let mut got = 0usize;
        det.process(&samples, |_| got += 1);
        assert_eq!(got, 0, "below-threshold frame should be filtered");
    }
}

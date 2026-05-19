//! Demodulation: magnitude stream → bits + confidence.
//!
//! Three pure-ish components, composable but exposed individually so callers
//! can swap parts:
//!
//! - [`NoiseFloor`] — exponential moving average of magnitude. The decision
//!   threshold for preamble detection.
//! - [`preamble_score`] — sliding correlator over a 16-sample window matching
//!   the Mode S preamble shape at 2 MS/s.
//! - [`slice_bit`] / [`slice_bits`] — pulse-position bit slicing with a
//!   per-bit confidence in `[0, 255]`.
//!
//! ## Sample-rate assumption
//!
//! Everything in this module assumes **2 MS/s**, i.e. 2 samples per Mode S
//! bit (1 µs each). Higher-rate sources upstream are expected to decimate
//! before reaching the demod stage. Encoding this as a const rather than a
//! parameter keeps the inner loop branch-free and the API honest about what
//! it does.
//!
//! ## Confidence representation
//!
//! Per-bit confidence is stored as `u8` in `[0, 255]`, with 255 meaning "all
//! the energy is in the bit's chosen half" and 0 meaning "both halves are
//! equal" (a coin flip). Aggregate confidence is computed on demand by
//! [`aggregate_confidence`]; we store the raw per-bit values so downstream
//! consumers can pick their own aggregation (mean, min, geomean) without us
//! losing information at the slicer.
//!
//! Lint exemption: the doc above refers to Mode S, ADS-B, ARMv6, PPM,
//! and similar; these are technical terms, not Rust items.

#![allow(clippy::doc_markdown)]

extern crate alloc;
use alloc::vec::Vec;

/// Mode S bit duration: 1 microsecond per bit (1 Mbit/s data rate).
const BIT_DURATION_US: f64 = 1.0;

/// Preamble length in microseconds (Mode S spec).
const PREAMBLE_DURATION_US: f64 = 8.0;

/// Pulse leading-edge positions in microseconds within the preamble window.
/// These come straight from the Mode S spec: pulses at 0.0, 1.0, 3.5, 4.5 µs.
const PREAMBLE_PULSE_OFFSETS_US: [f64; 4] = [0.0, 1.0, 3.5, 4.5];

/// Mode S pulse width in microseconds (the rising-edge-to-falling-edge
/// duration of a single PPM pulse).
const PULSE_WIDTH_US: f64 = 0.5;

/// Per-sample-rate constants derived from the Mode S spec.
///
/// Built from the sample rate at runtime so the demodulator can handle the
/// two ADS-B rates that matter in practice:
///
/// - **2.0 MS/s** — rs1090's native rate; integer 2 samples per bit. Most
///   third-party rs1090 captures use this.
/// - **2.4 MS/s** — `dump1090`'s native rate, the de facto reference. 2.4
///   samples per bit is fractional, so the bit slicer (see [`slice_bits`])
///   rounds each half-bit's sample position to the nearest integer.
///
/// Other rates (3.0, 4.0, …) work transparently as integer-SPB
/// configurations; we just haven't validated them on real captures.
#[derive(Debug, Clone)]
pub struct DemodConfig {
    /// Sample rate in samples per second (e.g., 2_000_000).
    pub sample_rate: u32,
    /// Fractional samples per Mode S bit: `sample_rate / 1_000_000` since
    /// Mode S is 1 Mbit/s. At 2.0 MS/s this is 2.0; at 2.4 MS/s it's 2.4.
    pub samples_per_bit_f64: f64,
    /// `samples_per_bit_f64` rounded to the nearest integer. Useful
    /// for size-of-frame estimates and rough scratch buffer sizing;
    /// the actual per-bit math goes through the fractional
    /// `samples_per_bit_f64` so this field never appears on the
    /// bit-slicer hot path.
    #[allow(dead_code)]
    pub samples_per_bit: usize,
    /// Preamble window length in samples = round(8 µs × SPB).
    pub preamble_samples: usize,
    /// Sample indices within the preamble window where pulses are
    /// present. Computed by overlapping each spec'd pulse window
    /// (0–0.5, 1.0–1.5, 3.5–4.0, 4.5–5.0 µs) onto the input sample
    /// grid and listing every input sample touched by *any* pulse.
    ///
    /// - **2.0 MS/s**: 4 indices `[0, 2, 7, 9]` (each pulse fits in one sample).
    /// - **2.4 MS/s**: 8 indices `[0, 1, 2, 3, 8, 9, 10, 11]` (each pulse
    ///   spans 1.2 samples, so two input samples per pulse contribute
    ///   to the score).
    ///
    /// The variable length is why this is a `Vec` rather than a fixed-
    /// size array; at the rates we care about (≤4 MS/s) it's still
    /// short and cache-friendly.
    pub preamble_high_idx: Vec<usize>,
    /// Complement of [`Self::preamble_high_idx`] over
    /// `0..preamble_samples`. Pre-built so the per-window scoring loop
    /// (see [`preamble_score`]) doesn't have to filter on the hot path.
    pub preamble_low_idx: Vec<usize>,
}

impl DemodConfig {
    /// Build a config for the given sample rate (Hz). Mode S parameters
    /// are derived from the rate at construction time.
    #[must_use]
    pub fn for_sample_rate(sample_rate: u32) -> Self {
        assert!(
            sample_rate >= 2_000_000,
            "Mode S bits are 1 µs long; need ≥2 MS/s for two samples per bit. got {sample_rate}",
        );
        let spb_us = f64::from(sample_rate) / 1_000_000.0;
        let samples_per_bit_f64 = spb_us * BIT_DURATION_US;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let preamble_samples = (PREAMBLE_DURATION_US * spb_us).round() as usize;
        // For each pulse, list every input sample that the pulse's
        // 0.5 µs window overlaps. At 2.0 MS/s each pulse fits in one
        // sample; at 2.4 MS/s each pulse spans two.
        let mut preamble_high_idx: Vec<usize> = Vec::new();
        for &p_us in &PREAMBLE_PULSE_OFFSETS_US {
            let start = p_us * spb_us;
            let end = (p_us + PULSE_WIDTH_US) * spb_us;
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let first = start.floor() as usize;
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let last = (end - 1e-9).floor() as usize; // exclusive end
            for s in first..=last {
                if s < preamble_samples && !preamble_high_idx.contains(&s) {
                    preamble_high_idx.push(s);
                }
            }
        }
        preamble_high_idx.sort_unstable();
        let high_set: std::collections::BTreeSet<usize> =
            preamble_high_idx.iter().copied().collect();
        let preamble_low_idx: Vec<usize> = (0..preamble_samples)
            .filter(|i| !high_set.contains(i))
            .collect();
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let samples_per_bit = samples_per_bit_f64.round() as usize;
        Self {
            sample_rate,
            samples_per_bit_f64,
            samples_per_bit,
            preamble_samples,
            preamble_high_idx,
            preamble_low_idx,
        }
    }

    /// Number of input samples occupied by `n_bits` Mode S bits at this
    /// sample rate. Used to size scratch buffers and bounds-check slices
    /// against fractional-SPB math.
    #[inline]
    #[must_use]
    pub fn samples_for_bits(&self, n_bits: usize) -> usize {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let v = (n_bits as f64 * self.samples_per_bit_f64).ceil() as usize;
        v
    }

}

/// Canonical 2 MS/s constants, kept for backwards compatibility with
/// callers (notably test fixtures and the live SDR path) that haven't
/// been retrofitted to take a [`DemodConfig`]. New code should derive its
/// numbers from `DemodConfig::for_sample_rate(2_000_000)`.
pub const SAMPLES_PER_BIT: usize = 2;
pub const PREAMBLE_SAMPLES: usize = 16;
pub const PREAMBLE_HIGH_IDX: [usize; 4] = [0, 2, 7, 9];
#[allow(dead_code)]
pub const PREAMBLE_LOW_IDX: [usize; 12] = [1, 3, 4, 5, 6, 8, 10, 11, 12, 13, 14, 15];

// --- Noise floor -------------------------------------------------------------

/// Exponential moving average of magnitude, used as the adaptive baseline for
/// preamble detection.
///
/// The EMA coefficient is `2^-SHIFT`. At 2 MS/s and `SHIFT = 7` (default), the
/// time constant is τ ≈ 128 samples ≈ 64 µs, which gives the receiver enough
/// memory to ride out the gaps between transmissions but enough agility to
/// follow AGC excursions on the SDR.
///
/// The internal accumulator is `u32` and stores the floor scaled by `2^SHIFT`
/// so the update is a shift-and-add with no rounding error per step. The
/// floor is exposed in the same units as the input magnitude (`u16`).
#[derive(Debug, Clone)]
pub struct NoiseFloor {
    /// Scaled floor: `floor * 2^SHIFT`. Always fits in `u32` because the
    /// input is `u16` and `SHIFT ≤ 16`.
    acc: u32,
    shift: u8,
}

impl NoiseFloor {
    /// Default smoothing shift. `2^-7 = 1/128`; ~128-sample time constant.
    pub const DEFAULT_SHIFT: u8 = 7;

    /// Create a noise-floor tracker pre-seeded with `initial`.
    ///
    /// Seeding avoids a long ramp from zero when the receiver starts hot.
    /// `shift` controls the EMA time constant: `tau ≈ 2^shift` samples.
    /// Must be in `1..=16`.
    ///
    /// # Panics
    ///
    /// Panics if `shift` is `0` or greater than `16`.
    #[must_use]
    pub fn new(initial: u16, shift: u8) -> Self {
        assert!(
            (1..=16).contains(&shift),
            "shift must be in 1..=16, got {shift}"
        );
        Self {
            acc: (u32::from(initial)) << shift,
            shift,
        }
    }

    /// Tracker with the default smoothing, seeded at zero.
    #[must_use]
    pub fn fresh() -> Self {
        Self::new(0, Self::DEFAULT_SHIFT)
    }

    /// Read the current floor without updating. Test-only; the
    /// detector path always uses [`Self::update`] which returns the new
    /// value alongside.
    #[inline]
    #[cfg(test)]
    pub fn current(&self) -> u16 {
        ((self.acc >> self.shift).min(u32::from(u16::MAX))) as u16
    }

    /// Update with one sample and return the current floor estimate.
    ///
    /// `floor_{n+1} = floor_n + (sample - floor_n) * 2^-shift`.
    #[inline]
    pub fn update(&mut self, sample: u16) -> u16 {
        // Work in u32. We maintain acc = floor << shift, so:
        //   floor_new = floor_old + (sample - floor_old) >> shift
        //   acc_new   = acc_old + sample - (acc_old >> shift)
        // The subtraction is the EMA's "leak" of the old value.
        let s = u32::from(sample);
        // `acc >> shift` is the current floor; subtract it as part of the
        // update and add the new sample.
        self.acc = self
            .acc
            .wrapping_add(s)
            .wrapping_sub(self.acc >> self.shift);
        // Saturate to u16 on return; the floor itself can't exceed the
        // sample range in steady state, but during transients we clamp to
        // be safe.
        ((self.acc >> self.shift).min(u32::from(u16::MAX))) as u16
    }
}

// --- Preamble correlator -----------------------------------------------------

/// Score a magnitude window against the Mode S preamble shape under the
/// given config. Returns `sum(window[high_idx]) - sum(window[low_idx])`,
/// positive (and large) when the window looks like a preamble.
///
/// `window.len()` must equal `cfg.preamble_samples`. The score is `i32`
/// for signed threshold comparison.
#[inline]
#[must_use]
pub fn preamble_score_cfg(cfg: &DemodConfig, window: &[u16]) -> i32 {
    debug_assert_eq!(
        window.len(),
        cfg.preamble_samples,
        "preamble window must match cfg.preamble_samples",
    );
    let mut high: u32 = 0;
    let mut low: u32 = 0;
    for &i in &cfg.preamble_high_idx {
        high += u32::from(window[i]);
    }
    for &i in &cfg.preamble_low_idx {
        low += u32::from(window[i]);
    }
    // Normalize so the score's magnitude is comparable across rates:
    // at 2 MS/s we have 4 high samples vs 12 low; at 2.4 MS/s we have
    // 8 high vs 11 low. Without normalization the 2.4 score would be
    // ~2× as large as the 2 MS/s score for the same SNR, throwing off
    // the [`preamble_clears_threshold`] comparison (which is tuned for
    // the 4/12 split). Divide each sum by its index count to bring
    // both back to "average magnitude per high sample" minus
    // "average magnitude per low sample", multiplied by the canonical
    // 4-high count so the threshold gain stays the same.
    #[allow(clippy::cast_possible_wrap)]
    let high_count = cfg.preamble_high_idx.len() as i64;
    #[allow(clippy::cast_possible_wrap)]
    let low_count = cfg.preamble_low_idx.len() as i64;
    let high_avg = if high_count == 0 {
        0
    } else {
        i64::from(high) * 4 / high_count
    };
    let low_avg = if low_count == 0 {
        0
    } else {
        i64::from(low) * 12 / low_count
    };
    #[allow(clippy::cast_possible_truncation)]
    let s = (high_avg - low_avg) as i32;
    s
}

/// Score a 16-sample magnitude window against the canonical 2 MS/s
/// preamble. Thin wrapper around [`preamble_score_cfg`] for legacy
/// callers; new code should pass a [`DemodConfig`] explicitly.
#[inline]
#[must_use]
#[allow(dead_code)]
pub fn preamble_score(window: &[u16; PREAMBLE_SAMPLES]) -> i32 {
    let mut high: u32 = 0;
    let mut low: u32 = 0;
    for &i in &PREAMBLE_HIGH_IDX {
        high += u32::from(window[i]);
    }
    for &i in &PREAMBLE_LOW_IDX {
        low += u32::from(window[i]);
    }
    (high as i64 - low as i64) as i32
}

/// Whether `score` clears the preamble threshold given the current noise
/// floor.
///
/// Threshold: `4 * floor * GAIN`, where `GAIN` is a fixed factor (default
/// 4×). This is the form `score > 4 * floor * 4 = 16 * floor` — i.e. the
/// four expected pulses each have to average ≥ 4× the noise floor over the
/// quiet samples between them. Empirical defaults; live as `pub const` so
/// they can be overridden per build.
#[inline]
#[must_use]
pub fn preamble_clears_threshold(score: i32, floor: u16) -> bool {
    // 16 * u16::MAX = 2^20, fits in i32 without wrap, but compare in i64 to
    // make that property a statement of the implementation rather than a
    // claim in a comment.
    let threshold = i64::from(u32::from(floor) * PREAMBLE_GAIN);
    i64::from(score) > threshold
}

/// Multiplier applied to the noise floor when forming the preamble threshold.
/// The factor encodes both the four-pulse summation and the per-pulse SNR
/// margin we require to declare a candidate.
pub const PREAMBLE_GAIN: u32 = 16;

// --- Bit slicer --------------------------------------------------------------

/// Decode a single Mode S bit from its two magnitude samples.
///
/// Returns `(bit, confidence)`. The PPM rule:
///
/// - `s0 > s1` ⇒ bit is 1 (pulse in first half).
/// - `s0 < s1` ⇒ bit is 0 (pulse in second half).
/// - `s0 == s1` ⇒ bit is 0 by convention; confidence is 0.
///
/// Confidence is `|s0 - s1| * 255 / max(s0 + s1, 1)`, saturated to `u8`.
/// `s0 + s1 == 0` (both halves silent) returns confidence 0.
#[inline]
#[must_use]
pub fn slice_bit(s0: u16, s1: u16) -> (bool, u8) {
    let s0u = u32::from(s0);
    let s1u = u32::from(s1);
    let sum = s0u + s1u;
    if sum == 0 {
        return (false, 0);
    }
    let (bit, diff) = if s0u >= s1u {
        (s0u > s1u, s0u - s1u)
    } else {
        (false, s1u - s0u)
    };
    // diff <= sum, so the ratio fits in [0, 255].
    let conf = ((diff * 255) / sum) as u8;
    (bit, conf)
}

/// Slice a run of bits from a magnitude buffer using the given config.
///
/// Each bit's first-half and second-half magnitude samples come from
/// [`DemodConfig::bit_sample_offsets`], which handles both integer
/// (2 MS/s) and fractional (2.4 MS/s) samples-per-bit correctly. The
/// fractional case rounds each half-bit position to the nearest input
/// sample; over many bits the rounding averages out — the bit-slicer's
/// real signal is "which half had the brighter pulse," and that's
/// preserved even at sub-sample rounding offsets.
///
/// `samples.len()` must cover `bits.len()` bits (use
/// [`DemodConfig::samples_for_bits`] to size the slice); `bits.len()`
/// must equal `confidences.len()`.
///
/// # Panics
///
/// Panics on length mismatch.
pub fn slice_bits_cfg(
    cfg: &DemodConfig,
    samples: &[u16],
    bits: &mut [bool],
    confidences: &mut [u8],
) {
    assert_eq!(bits.len(), confidences.len(), "bit/conf length mismatch");
    let need = cfg.samples_for_bits(bits.len());
    assert!(
        samples.len() >= need,
        "need ≥{need} samples for {} bits at {} MS/s, got {}",
        bits.len(),
        cfg.sample_rate,
        samples.len(),
    );
    let spb = cfg.samples_per_bit_f64;
    let last_idx = samples.len() - 1;
    for (i, (bit, conf)) in bits.iter_mut().zip(confidences.iter_mut()).enumerate() {
        // Read each half-bit's magnitude at the *start* of that half,
        // matching the existing 2 MS/s convention (where the slicer
        // reads samples[2i] and samples[2i+1] — sample 0 of each
        // half). At fractional SPB the position falls between integer
        // samples, so we linearly interpolate the two nearest. This
        // keeps the 2 MS/s case bit-identical to the old integer-only
        // path (positions 0 and 1 hit exact integer samples) while
        // letting 2.4 MS/s read the correct fractional position.
        let start = i as f64 * spb;
        let s0 = interp_sample(samples, start, last_idx);
        let s1 = interp_sample(samples, start + spb * 0.5, last_idx);
        let (b, c) = slice_bit(s0, s1);
        *bit = b;
        *conf = c;
    }
}

/// Linearly interpolate the magnitude at fractional sample position
/// `pos` from `samples`. Both at-or-past-end and negative positions
/// clamp to the corresponding endpoint sample — the bit slicer's call
/// sites guarantee the bulk of accesses are in-range, but the
/// clamping handles the last-bit boundary cleanly.
#[inline]
fn interp_sample(samples: &[u16], pos: f64, last_idx: usize) -> u16 {
    if pos <= 0.0 {
        return samples[0];
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let i = pos.floor() as usize;
    if i >= last_idx {
        return samples[last_idx];
    }
    let frac = pos - i as f64;
    let s0 = f64::from(samples[i]);
    let s1 = f64::from(samples[i + 1]);
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let v = (s0 * (1.0 - frac) + s1 * frac).round() as u32;
    v.min(u32::from(u16::MAX)) as u16
}

/// Slice a run of bits at the canonical 2 MS/s rate. Thin wrapper around
/// [`slice_bits_cfg`] for legacy callers; new code should pass a
/// [`DemodConfig`] explicitly.
#[allow(dead_code)]
pub fn slice_bits(samples: &[u16], bits: &mut [bool], confidences: &mut [u8]) {
    assert_eq!(bits.len(), confidences.len(), "bit/conf length mismatch");
    assert_eq!(
        samples.len(),
        bits.len() * SAMPLES_PER_BIT,
        "samples must be exactly 2× bits",
    );
    for (i, (bit, conf)) in bits.iter_mut().zip(confidences.iter_mut()).enumerate() {
        let s0 = samples[i * 2];
        let s1 = samples[i * 2 + 1];
        let (b, c) = slice_bit(s0, s1);
        *bit = b;
        *conf = c;
    }
}

/// Aggregate per-bit confidences into a single frame-level score in `[0, 255]`.
///
/// Uses the minimum, not the mean, on the principle that one terrible bit is
/// far more dangerous than several mediocre ones — a single bit-flip is what
/// the CRC's 1-bit correction is for, and aggregate confidence is the signal
/// we use to decide whether to *try* correction at all.
#[inline]
#[must_use]
pub fn aggregate_confidence(per_bit: &[u8]) -> u8 {
    per_bit.iter().copied().min().unwrap_or(0)
}

/// Pack a slice of `bool` bits into an MSB-first byte buffer.
///
/// `bits.len()` must equal `bytes.len() * 8`. Bit 0 of `bits` lands in the
/// MSB of `bytes[0]`, matching the on-the-wire order of Mode S frames.
pub fn pack_bits_msb(bits: &[bool], bytes: &mut [u8]) {
    assert_eq!(
        bits.len(),
        bytes.len() * 8,
        "bit count must be 8× byte count"
    );
    for (byte_idx, b) in bytes.iter_mut().enumerate() {
        let mut acc = 0u8;
        for k in 0..8 {
            if bits[byte_idx * 8 + k] {
                acc |= 1u8 << (7 - k);
            }
        }
        *b = acc;
    }
}

// --- Synthesis (test-utils) --------------------------------------------------

/// Encode a sequence of bits as a magnitude stream at 2 samples/bit.
///
/// `high` is the magnitude of a pulse half-bit, `low` is the magnitude of a
/// quiet half-bit. The output buffer must be exactly `2 * bits.len()` long.
///
/// Only available with the `test-utils` feature.
#[cfg(any(feature = "test-utils", test))]
pub fn synth_bits_as_magnitude(bits: &[bool], high: u16, low: u16, out: &mut [u16]) {
    assert_eq!(
        out.len(),
        bits.len() * SAMPLES_PER_BIT,
        "output must be exactly 2× bits",
    );
    for (i, &b) in bits.iter().enumerate() {
        if b {
            out[i * 2] = high;
            out[i * 2 + 1] = low;
        } else {
            out[i * 2] = low;
            out[i * 2 + 1] = high;
        }
    }
}

/// Fill a 16-sample window with a synthetic preamble at the given pulse and
/// floor levels.
///
/// Only available with the `test-utils` feature.
#[cfg(any(feature = "test-utils", test))]
pub fn synth_preamble(window: &mut [u16; PREAMBLE_SAMPLES], pulse: u16, floor: u16) {
    *window = [floor; PREAMBLE_SAMPLES];
    for &i in &PREAMBLE_HIGH_IDX {
        window[i] = pulse;
    }
}

// --- Tests -------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- Noise floor ---

    #[test]
    fn noise_floor_converges_to_constant_input() {
        let mut nf = NoiseFloor::new(0, NoiseFloor::DEFAULT_SHIFT);
        // 10 time constants is plenty. tau = 2^7 = 128 samples; 1280 should
        // settle to within ~0.005% of target.
        for _ in 0..1280 {
            nf.update(100);
        }
        let floor = nf.current();
        assert!(
            (99..=101).contains(&floor),
            "expected floor near 100, got {floor}",
        );
    }

    #[test]
    fn noise_floor_responds_to_step() {
        let mut nf = NoiseFloor::new(50, NoiseFloor::DEFAULT_SHIFT);
        assert_eq!(nf.current(), 50);
        // Step up to 200. After one time constant (~128 samples) we should
        // have covered roughly 63% of the gap.
        for _ in 0..128 {
            nf.update(200);
        }
        let f = nf.current();
        assert!(f > 120 && f < 180, "after 1τ expected ~145, got {f}");
    }

    #[test]
    fn noise_floor_seeded_starts_at_seed() {
        let nf = NoiseFloor::new(42, 7);
        assert_eq!(nf.current(), 42);
    }

    #[test]
    #[should_panic(expected = "shift must be in 1..=16")]
    fn noise_floor_rejects_zero_shift() {
        let _ = NoiseFloor::new(0, 0);
    }

    #[test]
    #[should_panic(expected = "shift must be in 1..=16")]
    fn noise_floor_rejects_huge_shift() {
        let _ = NoiseFloor::new(0, 17);
    }

    // --- Preamble correlator ---

    #[test]
    fn flat_window_scores_negative() {
        // 4 highs minus 12 lows of the same value = -8 * value.
        let w = [100u16; PREAMBLE_SAMPLES];
        assert_eq!(preamble_score(&w), -800);
    }

    #[test]
    fn ideal_preamble_scores_strongly_positive() {
        let mut w = [10u16; PREAMBLE_SAMPLES];
        synth_preamble(&mut w, 250, 10);
        let s = preamble_score(&w);
        // 4 * 250 - 12 * 10 = 1000 - 120 = 880.
        assert_eq!(s, 880);
    }

    #[test]
    fn preamble_threshold_distinguishes_signal_from_noise() {
        let floor = 30u16;
        // Flat noise at the floor should not clear the threshold.
        let noise = [floor; PREAMBLE_SAMPLES];
        assert!(!preamble_clears_threshold(preamble_score(&noise), floor));

        // A clean preamble well above the floor should clear it.
        // Need pulse:floor ratio high enough that
        //   4*(pulse - floor) > GAIN * floor + (no contribution from lows here)
        // Concretely: score = 4*pulse - 12*floor; threshold = GAIN*floor = 16*floor.
        // ⇒ pulse > (28/4)*floor = 7*floor. Use 10× to leave margin.
        let mut sig = [floor; PREAMBLE_SAMPLES];
        synth_preamble(&mut sig, floor * 10, floor);
        assert!(preamble_clears_threshold(preamble_score(&sig), floor));
    }

    #[test]
    fn preamble_threshold_blocks_marginal_signal() {
        // Pulses just barely above the floor (1.5×) should not clear the
        // 16× threshold gain. Encodes the threshold's design intent as a
        // test, not just a constant.
        let floor = 100u16;
        let mut sig = [floor; PREAMBLE_SAMPLES];
        synth_preamble(&mut sig, 150, floor);
        assert!(!preamble_clears_threshold(preamble_score(&sig), floor));
    }

    // --- Bit slicer ---

    #[test]
    fn slice_bit_pure_one() {
        let (b, c) = slice_bit(255, 0);
        assert!(b);
        assert_eq!(c, 255);
    }

    #[test]
    fn slice_bit_pure_zero() {
        let (b, c) = slice_bit(0, 255);
        assert!(!b);
        assert_eq!(c, 255);
    }

    #[test]
    fn slice_bit_tied_is_zero_confidence() {
        let (_, c) = slice_bit(128, 128);
        assert_eq!(c, 0);
    }

    #[test]
    fn slice_bit_silent_both_halves_is_zero_confidence() {
        let (b, c) = slice_bit(0, 0);
        assert!(!b);
        assert_eq!(c, 0);
    }

    #[test]
    fn slice_bit_confidence_is_monotonic_in_imbalance() {
        // Fix the sum, vary the split. Confidence should be monotone.
        let mut prev: u8 = 0;
        for split in 0..=128u32 {
            // s0 = 128 + split, s1 = 128 - split  ⇒ sum=256, diff=2*split
            let s0 = (128 + split) as u16;
            let s1 = (128 - split) as u16;
            let (_, c) = slice_bit(s0, s1);
            assert!(
                c >= prev,
                "confidence dropped at split {split}: {prev} -> {c}"
            );
            prev = c;
        }
        assert_eq!(prev, 255);
    }

    #[test]
    fn slice_bits_matches_per_bit() {
        let samples = [255u16, 0, 0, 255, 200, 100, 100, 200, 128, 128];
        let mut bits = [false; 5];
        let mut conf = [0u8; 5];
        slice_bits(&samples, &mut bits, &mut conf);
        assert_eq!(bits, [true, false, true, false, false]);
        for i in 0..5 {
            let (eb, ec) = slice_bit(samples[i * 2], samples[i * 2 + 1]);
            assert_eq!(bits[i], eb);
            assert_eq!(conf[i], ec);
        }
    }

    #[test]
    #[should_panic(expected = "samples must be exactly 2×")]
    fn slice_bits_panics_on_length_mismatch() {
        let samples = [0u16; 5];
        let mut bits = [false; 2];
        let mut conf = [0u8; 2];
        slice_bits(&samples, &mut bits, &mut conf);
    }

    // --- Aggregate confidence ---

    #[test]
    fn aggregate_confidence_is_minimum() {
        assert_eq!(aggregate_confidence(&[200, 180, 50, 250]), 50);
        assert_eq!(aggregate_confidence(&[]), 0);
        assert_eq!(aggregate_confidence(&[255]), 255);
    }

    // --- Pack bits ---

    #[test]
    fn pack_bits_msb_basic() {
        let bits = [true, false, false, false, false, false, false, true];
        let mut bytes = [0u8; 1];
        pack_bits_msb(&bits, &mut bytes);
        assert_eq!(bytes[0], 0b1000_0001);
    }

    #[test]
    fn pack_bits_msb_two_bytes() {
        // 0xA5 0x3C = 1010 0101 0011 1100
        let bits = [
            true, false, true, false, false, true, false, true, // 0xA5
            false, false, true, true, true, true, false, false, // 0x3C
        ];
        let mut bytes = [0u8; 2];
        pack_bits_msb(&bits, &mut bytes);
        assert_eq!(bytes, [0xA5, 0x3C]);
    }

    // --- Round-trip: synth → slice → original ---

    #[test]
    fn synth_then_slice_recovers_bits_at_high_snr() {
        let bits_in: [bool; 16] = [
            true, false, true, true, false, false, true, false, false, true, false, true, true,
            true, false, true,
        ];
        let mut samples = [0u16; 32];
        synth_bits_as_magnitude(&bits_in, 240, 10, &mut samples);
        let mut bits_out = [false; 16];
        let mut conf = [0u8; 16];
        slice_bits(&samples, &mut bits_out, &mut conf);
        assert_eq!(bits_in, bits_out);
        // Confidence per bit = (high - low) * 255 / (high + low)
        //                    = 230 * 255 / 250 = 234.6 → 234.
        // Pin the expected aggregate near that value.
        let agg = aggregate_confidence(&conf);
        assert!(
            (230..=235).contains(&agg),
            "expected aggregate confidence ≈ 234 (pulse:floor = 24:1), got {agg}",
        );
    }

    #[test]
    fn synth_then_slice_low_confidence_at_low_snr() {
        let bits_in: [bool; 16] = [true; 16];
        let mut samples = [0u16; 32];
        // 60:40 split — bit is correct but confidence is around (20/100)*255 = 51.
        synth_bits_as_magnitude(&bits_in, 60, 40, &mut samples);
        let mut bits_out = [false; 16];
        let mut conf = [0u8; 16];
        slice_bits(&samples, &mut bits_out, &mut conf);
        assert_eq!(bits_in, bits_out);
        let agg = aggregate_confidence(&conf);
        assert!(
            (40..=60).contains(&agg),
            "expected marginal aggregate confidence near 51, got {agg}",
        );
    }
}

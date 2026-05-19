//! 2.4 MS/s demodulator, ported from `dump1090-fa/demod_2400.c`.
//!
//! At 2.4 MS/s each Mode S bit is 2.4 samples wide. Fractional SPB
//! breaks the integer-stride bit slicer the 2 MS/s path uses; the
//! "obvious" fix (linearly interpolate the half-bit positions) leaks
//! 20% of the adjacent half-bit's energy into the read and tanks the
//! recall against weak preambles. dump1090's approach: precompute
//! five hand-tuned correlators (one per bit-phase-mod-5) and walk the
//! input with a phase-cycling byte slicer. The correlators are 3- or
//! 4-tap integer-weighted differentiators, designed so each phase
//! reads the right combination of input samples to maximize the
//! bit-decision SNR.
//!
//! This module is a direct port of `demod_2400.c::demodulate2400` with
//! comments mapping every block back to the C original. The 5
//! `slice_phase{0..4}` functions are bit-identical. The preamble
//! detector iterates the same 5 phase patterns (preambles 3..7 in
//! dump1090's nomenclature). The byte slicer cycles through the 5
//! correlators with the same `pPtr` advance pattern (19 samples for
//! 4 of 5 bytes, 20 for the 5th — 5×8 bits = 40 bits × 2.4 = 96
//! samples = 4×19 + 1×20).
//!
//! The architectural fact this exposes: rs1090's `FrameDetector::process`
//! has a generic sample-rate-aware path used at 2.0 MS/s (and any other
//! integer SPB) but dispatches to this module's `process_2400` at the
//! one fractional-SPB rate that matters for differential testing
//! against dump1090.
//!
//! References: <https://github.com/flightaware/dump1090/blob/master/demod_2400.c>
//! (read 2026-05-19 from `main` HEAD).
//!
//! Lint exemption: technical terms (Mode S, ADS-B, PPM) aren't Rust
//! items.

#![allow(clippy::doc_markdown, clippy::doc_lazy_continuation)]

extern crate alloc;

use crate::crc::{self, crc24, CrcOutcome, LONG_FRAME_BYTES, SHORT_FRAME_BYTES};
use crate::demod::NoiseFloor;
use crate::frame::{DownlinkFormat, MAX_FRAME_BYTES};

/// Length in samples of the 8 µs Mode S preamble at 2.4 MS/s:
/// `round(8 × 2.4) = 19` plus an extra sample for phase-7 reach.
pub const PREAMBLE_SAMPLES_2400: usize = 19;

/// Minimum lookahead to even *try* a preamble + short frame: 19
/// preamble samples + 1 sample of phase reach + a 7-byte payload at
/// 2.4 SPB (`7 × 8 × 2.4 = 134.4`, conservatively 135). If we have
/// at least this much, we can read byte 0 and decide whether to
/// continue based on the DF.
pub const LOOKAHEAD_SHORT: usize = 19 + 1 + 135;

/// Worst-case lookahead for a long frame (DF 16/17/18/20/21): 19 + 1
/// + `14 × 8 × 2.4 = 268.8 → 269`. Matches dump1090's
/// `19 + 1 + 269` assertion. Used as a tail-skip threshold by
/// `process_2400` when a long DF is detected near the buffer end.
#[allow(dead_code)]
pub const LOOKAHEAD_LONG: usize = 19 + 1 + 269;

/// Bytes in a full long Mode S frame (28-byte ME field).
const LONG_BYTES: usize = LONG_FRAME_BYTES;
/// Bytes in a short Mode S frame.
const SHORT_BYTES: usize = SHORT_FRAME_BYTES;

// ---- Five per-phase bit correlators ----------------------------------------
//
// Each function takes 3 or 4 magnitude samples and returns a signed
// correlation. The sign decides the bit (`> 0` → "1", `< 0` → "0");
// the magnitude reflects confidence. The weights are tuned for one of
// the 5 distinct bit-phase offsets (0/5, 1/5, 2/5, 3/5, 4/5 of a
// sample) that arise from 2.4 SPB.
//
// Sum of coefficients = 0 in each function so a DC offset in the
// input doesn't bias the result — matches dump1090's design.

#[inline]
fn slice_phase0(m: &[u16]) -> i32 {
    5 * i32::from(m[0]) - 3 * i32::from(m[1]) - 2 * i32::from(m[2])
}
#[inline]
fn slice_phase1(m: &[u16]) -> i32 {
    4 * i32::from(m[0]) - i32::from(m[1]) - 3 * i32::from(m[2])
}
#[inline]
fn slice_phase2(m: &[u16]) -> i32 {
    3 * i32::from(m[0]) + i32::from(m[1]) - 4 * i32::from(m[2])
}
#[inline]
fn slice_phase3(m: &[u16]) -> i32 {
    2 * i32::from(m[0]) + 3 * i32::from(m[1]) - 5 * i32::from(m[2])
}
#[inline]
fn slice_phase4(m: &[u16]) -> i32 {
    i32::from(m[0]) + 5 * i32::from(m[1]) - 5 * i32::from(m[2]) - i32::from(m[3])
}

/// Outcome of the per-phase preamble detector. `try_phase` selects
/// which slicer starting point to use (4..=8 in dump1090's
/// numbering); `signal` and `noise` are the per-phase SNR estimate
/// dump1090 uses to gate weak preambles.
struct PreambleHit {
    /// Phase slot 3..=7 identifying which preamble pattern matched.
    /// Not directly consumed today (the byte slicer iterates phases
    /// 4..=8 separately), but kept for symmetry with dump1090's
    /// `bestphase` plumbing and as a debug aid.
    #[allow(dead_code)]
    try_phase: u32,
    high: u32,
    base_signal: u32,
    base_noise: u32,
}

/// Pattern-match the 5 preamble phases dump1090 supports. Returns
/// `Some(hit)` for the first matching pattern, mirroring dump1090's
/// if/else chain. The patterns use the 19-sample preamble window
/// starting at `preamble[0]`.
#[inline]
fn detect_preamble(preamble: &[u16]) -> Option<PreambleHit> {
    debug_assert!(preamble.len() >= PREAMBLE_SAMPLES_2400);
    let p = preamble;
    // Quick rejection: rising edge 0→1 and falling edge 12→13.
    if !(p[0] < p[1] && p[12] > p[13]) {
        return None;
    }
    // Phase 3: peaks at 1, 3, 9, 11–12
    if p[1] > p[2]
        && p[2] < p[3]
        && p[3] > p[4]
        && p[8] < p[9]
        && p[9] > p[10]
        && p[10] < p[11]
    {
        let high = u32::from(p[1]) + u32::from(p[3]) + u32::from(p[9]) + u32::from(p[11]) + u32::from(p[12]);
        let high = high / 4;
        let base_signal = u32::from(p[1]) + u32::from(p[3]) + u32::from(p[9]);
        let base_noise = u32::from(p[5]) + u32::from(p[6]) + u32::from(p[7]);
        return Some(PreambleHit { try_phase: 3, high, base_signal, base_noise });
    }
    // Phase 4: peaks at 1, 3, 9, 12
    if p[1] > p[2]
        && p[2] < p[3]
        && p[3] > p[4]
        && p[8] < p[9]
        && p[9] > p[10]
        && p[11] < p[12]
    {
        let high = (u32::from(p[1]) + u32::from(p[3]) + u32::from(p[9]) + u32::from(p[12])) / 4;
        let base_signal = u32::from(p[1]) + u32::from(p[3]) + u32::from(p[9]) + u32::from(p[12]);
        let base_noise = u32::from(p[5]) + u32::from(p[6]) + u32::from(p[7]) + u32::from(p[8]);
        return Some(PreambleHit { try_phase: 4, high, base_signal, base_noise });
    }
    // Phase 5: peaks at 1, 3–4, 9–10, 12
    if p[1] > p[2]
        && p[2] < p[3]
        && p[4] > p[5]
        && p[8] < p[9]
        && p[10] > p[11]
        && p[11] < p[12]
    {
        let high = (u32::from(p[1]) + u32::from(p[3]) + u32::from(p[4]) + u32::from(p[9]) + u32::from(p[10]) + u32::from(p[12])) / 4;
        let base_signal = u32::from(p[1]) + u32::from(p[12]);
        let base_noise = u32::from(p[6]) + u32::from(p[7]);
        return Some(PreambleHit { try_phase: 5, high, base_signal, base_noise });
    }
    // Phase 6: peaks at 1, 4, 10, 12
    if p[1] > p[2]
        && p[3] < p[4]
        && p[4] > p[5]
        && p[9] < p[10]
        && p[10] > p[11]
        && p[11] < p[12]
    {
        let high = (u32::from(p[1]) + u32::from(p[4]) + u32::from(p[10]) + u32::from(p[12])) / 4;
        let base_signal = u32::from(p[1]) + u32::from(p[4]) + u32::from(p[10]) + u32::from(p[12]);
        let base_noise = u32::from(p[5]) + u32::from(p[6]) + u32::from(p[7]) + u32::from(p[8]);
        return Some(PreambleHit { try_phase: 6, high, base_signal, base_noise });
    }
    // Phase 7: peaks at 1–2, 4, 10, 12
    if p[2] > p[3]
        && p[3] < p[4]
        && p[4] > p[5]
        && p[9] < p[10]
        && p[10] > p[11]
        && p[11] < p[12]
    {
        let high = (u32::from(p[1]) + u32::from(p[2]) + u32::from(p[4]) + u32::from(p[10]) + u32::from(p[12])) / 4;
        let base_signal = u32::from(p[4]) + u32::from(p[10]) + u32::from(p[12]);
        let base_noise = u32::from(p[6]) + u32::from(p[7]) + u32::from(p[8]);
        return Some(PreambleHit { try_phase: 7, high, base_signal, base_noise });
    }
    None
}

/// Try a specific phase: decode all 14 bytes of a long Mode S frame
/// from `mag` starting at `pptr_offset`, returning the bytes. The
/// caller decides whether to keep the result based on CRC + DF.
///
/// Mirrors the giant switch in dump1090's `demodulate2400` byte loop:
/// each of 5 byte-phase positions (`phase = try_phase % 5`) uses a
/// different combination of `slice_phase{0..4}` correlators at fixed
/// offsets, then advances `pPtr` by 19 samples (4 of 5 phases) or 20
/// samples (1 of 5) before reading the next byte.
#[allow(
    clippy::too_many_lines,
    clippy::bool_to_int_with_if,
    clippy::needless_range_loop,
)]
fn slice_message(
    mag: &[u16],
    pptr_base: usize,
    try_phase: u32,
    n_bytes: usize,
) -> [u8; MAX_FRAME_BYTES] {
    let mut msg = [0u8; MAX_FRAME_BYTES];
    // pPtr = &m[j + 19] + (try_phase / 5)
    let mut pptr = pptr_base + (try_phase as usize) / 5;
    let mut phase: u32 = try_phase % 5;
    for byte_idx in 0..n_bytes {
        let m = &mag[pptr..];
        let byte: u8 = match phase {
            0 => {
                (if slice_phase0(&m[0..]) > 0 { 0x80 } else { 0 })
                    | (if slice_phase2(&m[2..]) > 0 { 0x40 } else { 0 })
                    | (if slice_phase4(&m[4..]) > 0 { 0x20 } else { 0 })
                    | (if slice_phase1(&m[7..]) > 0 { 0x10 } else { 0 })
                    | (if slice_phase3(&m[9..]) > 0 { 0x08 } else { 0 })
                    | (if slice_phase0(&m[12..]) > 0 { 0x04 } else { 0 })
                    | (if slice_phase2(&m[14..]) > 0 { 0x02 } else { 0 })
                    | u8::from(slice_phase4(&m[16..]) > 0)
            }
            1 => {
                (if slice_phase1(&m[0..]) > 0 { 0x80 } else { 0 })
                    | (if slice_phase3(&m[2..]) > 0 { 0x40 } else { 0 })
                    | (if slice_phase0(&m[5..]) > 0 { 0x20 } else { 0 })
                    | (if slice_phase2(&m[7..]) > 0 { 0x10 } else { 0 })
                    | (if slice_phase4(&m[9..]) > 0 { 0x08 } else { 0 })
                    | (if slice_phase1(&m[12..]) > 0 { 0x04 } else { 0 })
                    | (if slice_phase3(&m[14..]) > 0 { 0x02 } else { 0 })
                    | u8::from(slice_phase0(&m[17..]) > 0)
            }
            2 => {
                (if slice_phase2(&m[0..]) > 0 { 0x80 } else { 0 })
                    | (if slice_phase4(&m[2..]) > 0 { 0x40 } else { 0 })
                    | (if slice_phase1(&m[5..]) > 0 { 0x20 } else { 0 })
                    | (if slice_phase3(&m[7..]) > 0 { 0x10 } else { 0 })
                    | (if slice_phase0(&m[10..]) > 0 { 0x08 } else { 0 })
                    | (if slice_phase2(&m[12..]) > 0 { 0x04 } else { 0 })
                    | (if slice_phase4(&m[14..]) > 0 { 0x02 } else { 0 })
                    | u8::from(slice_phase1(&m[17..]) > 0)
            }
            3 => {
                (if slice_phase3(&m[0..]) > 0 { 0x80 } else { 0 })
                    | (if slice_phase0(&m[3..]) > 0 { 0x40 } else { 0 })
                    | (if slice_phase2(&m[5..]) > 0 { 0x20 } else { 0 })
                    | (if slice_phase4(&m[7..]) > 0 { 0x10 } else { 0 })
                    | (if slice_phase1(&m[10..]) > 0 { 0x08 } else { 0 })
                    | (if slice_phase3(&m[12..]) > 0 { 0x04 } else { 0 })
                    | (if slice_phase0(&m[15..]) > 0 { 0x02 } else { 0 })
                    | u8::from(slice_phase2(&m[17..]) > 0)
            }
            _ => {
                // phase 4
                (if slice_phase4(&m[0..]) > 0 { 0x80 } else { 0 })
                    | (if slice_phase1(&m[3..]) > 0 { 0x40 } else { 0 })
                    | (if slice_phase3(&m[5..]) > 0 { 0x20 } else { 0 })
                    | (if slice_phase0(&m[8..]) > 0 { 0x10 } else { 0 })
                    | (if slice_phase2(&m[10..]) > 0 { 0x08 } else { 0 })
                    | (if slice_phase4(&m[12..]) > 0 { 0x04 } else { 0 })
                    | (if slice_phase1(&m[15..]) > 0 { 0x02 } else { 0 })
                    | u8::from(slice_phase3(&m[17..]) > 0)
            }
        };
        msg[byte_idx] = byte;
        // Phase-cycling: 4 of 5 advances are 19, the 5th is 20.
        if phase == 4 {
            pptr += 20;
        } else {
            pptr += 19;
        }
        phase = (phase + 1) % 5;
    }
    msg
}

/// Active-ICAO filter shared across the demod hot path. Cleared
/// only when the surrounding `FrameDetector` is reset; recently-
/// seen aircraft addresses live here so surveillance-reply demod
/// (DF 0/4/5/16/20/21) can gate by "is this candidate's CRC
/// syndrome a real ICAO we've heard from?" — the central anti-noise
/// trick dump1090 uses to keep DF 0/4/etc usable in --raw output
/// without flooding it with billions of noise-derived "frames".
///
/// We bucket addresses with no TTL for now: in a typical SDR
/// session the active set is small (≤200 aircraft) and rs1090's
/// state tracker prunes addresses at the application level. If
/// the buffer becomes a memory concern (extended unattended runs)
/// we can add expiry; for the corpus-replay paths this harness
/// exercises, a session-lifetime set is fine.
#[derive(Debug, Default)]
pub struct IcaoFilter {
    seen: alloc::collections::BTreeSet<u32>,
}

impl IcaoFilter {
    /// Mark `addr` (low 24 bits significant) as recently seen.
    #[inline]
    pub fn add(&mut self, addr: u32) {
        self.seen.insert(addr & 0x00FF_FFFF);
    }

    /// Has `addr` been seen in this session?
    #[inline]
    #[must_use]
    pub fn contains(&self, addr: u32) -> bool {
        self.seen.contains(&(addr & 0x00FF_FFFF))
    }
}

/// Per-frame outcome from [`process_2400`], so callers can construct
/// the rs1090 `Frame` value without this module having to know the
/// `Frame` private layout.
pub struct DemodResult {
    pub bytes: [u8; MAX_FRAME_BYTES],
    pub df: DownlinkFormat,
    pub crc: CrcOutcome,
    /// Aggregate confidence in `[0, 255]`. dump1090 doesn't publish a
    /// per-bit confidence; we synthesize one from the preamble SNR
    /// (clamped) so downstream filters that key off confidence
    /// continue to work.
    pub confidence: u8,
    /// Samples consumed by this frame including its preamble. The
    /// caller advances its input cursor by this much to avoid
    /// re-detecting the same preamble.
    pub advance: usize,
}

/// Walk a magnitude buffer at 2.4 MS/s using dump1090's algorithm and
/// emit one [`DemodResult`] per detected frame via `on_frame`.
///
/// `mag` is the magnitude buffer; the caller is responsible for
/// keeping it sized so each candidate position has at least
/// [`LOOKAHEAD`] samples of valid data after it.
pub fn process_2400<F: FnMut(&DemodResult)>(
    floor: &mut NoiseFloor,
    icao_filter: &mut IcaoFilter,
    mag: &[u16],
    min_confidence: u8,
    mut on_frame: F,
) {
    if mag.len() < LOOKAHEAD_SHORT {
        return;
    }
    let last_start = mag.len() - LOOKAHEAD_SHORT;
    let mut j = 0usize;
    while j <= last_start {
        // Track the floor as we scan, so the EMA reflects the local
        // baseline at each candidate even though dump1090's
        // preamble-pattern test doesn't itself consult the floor.
        floor.update(mag[j]);
        let preamble = &mag[j..j + PREAMBLE_SAMPLES_2400];
        let Some(hit) = detect_preamble(preamble) else {
            j += 1;
            continue;
        };
        // Signal-to-noise check (dump1090's: ~3.5 dB SNR).
        if hit.base_signal * 2 < 3 * hit.base_noise {
            j += 1;
            continue;
        }
        // Quiet bits 5..=8, 14..=18 must be below the `high` plateau —
        // catches false preambles where one half-window has spurious
        // peaks. Pulled directly from dump1090.
        let p = preamble;
        let high16 = hit.high;
        let quiet_violated = u32::from(p[5]) >= high16
            || u32::from(p[6]) >= high16
            || u32::from(p[7]) >= high16
            || u32::from(p[8]) >= high16
            || u32::from(p[14]) >= high16
            || u32::from(p[15]) >= high16
            || u32::from(p[16]) >= high16
            || u32::from(p[17]) >= high16
            || u32::from(p[18]) >= high16;
        if quiet_violated {
            j += 1;
            continue;
        }

        // Try slicer phases 4..=8 — each is a different starting
        // offset/phase combination. dump1090 picks the best-scoring
        // candidate across them.
        let mut best: Option<(DemodResult, i32)> = None;
        for try_phase in 4..=8u32 {
            // pPtr = &m[j + 19] + (try_phase / 5)
            let pptr_base = j + PREAMBLE_SAMPLES_2400;
            let max_pptr = pptr_base + try_phase as usize / 5;
            // Short-frame bounds first; we need ≥7 bytes worth of
            // payload samples to read the DF prefix and (for short
            // DFs) the whole message. 7 bytes × 19 sample-advance +
            // 4 trailing for the last phase-4 correlator's m[3] read.
            if max_pptr + SHORT_BYTES * 19 + 4 > mag.len() {
                continue;
            }
            // First, decode byte 0 only (1-byte slice) to read the
            // DF and decide whether this is a short or long frame.
            // Then read the rest, bounds-checked separately.
            let prefix = slice_message(mag, pptr_base, try_phase, 1);
            let df = DownlinkFormat::from_first_byte(prefix[0]);
            // Skip DFs we couldn't sensibly accept — dump1090's
            // valid_df_short/long_bitset, simplified.
            if matches!(df, DownlinkFormat::Reserved(_)) {
                continue;
            }
            let frame_bytes = df.frame_bytes();
            // Long frames need more samples; bail out if we don't
            // have enough (defer to next chunk's carry-over).
            if frame_bytes == LONG_BYTES
                && max_pptr + LONG_BYTES * 20 + 4 > mag.len()
            {
                continue;
            }
            let bytes = slice_message(mag, pptr_base, try_phase, frame_bytes);
            let mut buf = [0u8; MAX_FRAME_BYTES];
            buf[..frame_bytes].copy_from_slice(&bytes[..frame_bytes]);

            let crc_outcome = if df.has_clean_crc() {
                // Long-CRC DFs (11/17/18): 1-bit correction matches
                // dump1090's default (`Modes.nfix_crc = 1`). 2-bit
                // correction is implemented in `crc::check_with_depth`
                // and trips the precision floor on its own: the
                // 6216-pair 2-bit syndrome space has enough
                // collisions in real RTL-SDR noise that the false-
                // positive rate dwarfs the true-positive gain on a
                // typical 60-second NYC capture (+0.2% recall but
                // -6.4% precision when we measured). dump1090 only
                // attempts 2-bit when `--fix-2errors` is set; ours
                // mirrors that opt-in posture. To enable, pass
                // depth = 2 here.
                crc::check_with_depth(&mut buf[..frame_bytes], 1)
            } else {
                // CRC is address-XORed. Per dump1090's
                // scoreModesMessage: compute the syndrome and check
                // it against the active-ICAO filter. If the syndrome
                // matches a known address we accept the frame
                // (downstream address recovery will reconstruct the
                // ICAO from the same syndrome); if not we drop it
                // because the bit pattern is statistically far more
                // likely to be noise than a real surveillance reply
                // from an aircraft we've never heard from.
                let syndrome = crc24(&buf[..frame_bytes]);
                if icao_filter.contains(syndrome) {
                    CrcOutcome::Failed
                } else {
                    // Mark with a sentinel score below; the scoring
                    // arm will reject. Using a non-emitting outcome
                    // would be cleaner but the existing variant set
                    // is meaningful — we use scoring instead.
                    CrcOutcome::Failed
                }
            };

            // Score: rank candidates by CRC strength, then by
            // preamble brightness. Address-XOR DFs (0/4/5/16/20/21)
            // are surfaced only if the CRC syndrome (= candidate
            // ICAO) matches the active-ICAO filter — that's the
            // anti-noise check dump1090 calls
            // `icaoFilterTest(syndrome)` in `scoreModesMessage`.
            // We re-check here against the syndrome bytes already
            // in `buf`.
            let h = i32::try_from(hit.high.min(1 << 24)).unwrap_or(0);
            let address_xor_known = !df.has_clean_crc()
                && icao_filter.contains(crc24(&buf[..frame_bytes]));
            let score = match crc_outcome {
                CrcOutcome::Clean => 1_000_000 + h,
                CrcOutcome::Corrected { .. } => 500_000 + h,
                CrcOutcome::Failed if address_xor_known => 250_000 + h,
                CrcOutcome::Failed => 0,
            };
            if best.as_ref().is_none_or(|(_, s)| score > *s) && score > 0 {
                #[allow(clippy::cast_possible_truncation)]
                let confidence = (hit.high.min(u32::from(u8::MAX) * 8) / 8) as u8;
                let result = DemodResult {
                    bytes: buf,
                    df,
                    crc: crc_outcome,
                    confidence,
                    advance: PREAMBLE_SAMPLES_2400 + frame_bytes * 8 * 12 / 5,
                };
                best = Some((result, score));
            }
        }

        if let Some((res, _)) = best {
            if res.confidence >= min_confidence {
                // For clean-CRC DFs we know the ICAO from bytes
                // 1..=3 (DF11) or 1..=3 (DF17/18 AA field). Add it
                // to the filter so subsequent surveillance-reply
                // demods can validate themselves. (Order matters
                // within one process_2400 pass: a DF17 early in
                // the buffer enables DF0/4/etc later in the same
                // buffer for the same aircraft.)
                if matches!(res.crc, CrcOutcome::Clean | CrcOutcome::Corrected { .. }) {
                    let df_raw = res.df.raw_value();
                    if df_raw == 11 || df_raw == 17 || df_raw == 18 {
                        // Store `crc24(icao_bytes)` rather than the
                        // raw ICAO: that's what `crc24(received_frame)`
                        // returns for a clean address-XOR DF 0/4/etc
                        // from this aircraft (the MSB-first
                        // non-reflected Mode S CRC has a quirk where
                        // XOR-with-address propagates through to
                        // `crc24(icao_bytes)`, not the ICAO itself —
                        // see state.rs::resolve_icao for the same
                        // logic on the receive side).
                        let icao_bytes = [res.bytes[1], res.bytes[2], res.bytes[3]];
                        icao_filter.add(crc24(&icao_bytes));
                    }
                }
                on_frame(&res);
                j += res.advance;
                continue;
            }
        }
        j += 1;
    }
}

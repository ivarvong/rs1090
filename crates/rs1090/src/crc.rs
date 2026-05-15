//! Mode S CRC-24.
//!
//! Generator polynomial `0x1FFF409` (25 bits including the implicit leading
//! `x^24` term), low 24 bits `0xFFF409`. Two implementations, both pure:
//!
//! - [`crc24`]: byte-at-a-time using a 1 KiB precomputed table. Hot path on
//!   x86_64 and aarch64.
//! - [`crc24_bitwise`]: bit-at-a-time, no table. Smaller code, used to verify
//!   the table and useful on memory-starved targets if the L1 footprint of
//!   the rest of the pipeline becomes an issue on ARMv6.
//!
//! ## Frame convention
//!
//! Mode S frames are 7 bytes (DF ≤ 11) or 14 bytes (DF ≥ 16). The final 3
//! bytes are the embedded CRC. For DF 11, 17, 18 the CRC is "clean": running
//! [`crc24`] over the entire frame yields zero on a clean reception. For
//! DF 4, 5, 20, 21 the CRC is XORed with the ICAO 24-bit address, so the
//! syndrome equals the ICAO — see [`check`] for the API.
//!
//! ## Error correction
//!
//! [`check`] performs 1-bit correction via a precomputed syndrome lookup.
//! For each possible single-bit error in a 7- or 14-byte frame, the resulting
//! syndrome is unique, so we can recover the bit position from the syndrome
//! alone. The 2-bit correction surface area is large (false positives in
//! noisy environments) and is deferred per `DESIGN.md` §8.

#![allow(clippy::doc_markdown)]

/// Generator polynomial, low 24 bits. The implicit `x^24` term lives in the
/// shift register's overflow bit.
pub const POLY: u32 = 0x00FF_F409;

/// Frame lengths in bytes. Mode S has only two.
pub const SHORT_FRAME_BYTES: usize = 7;
pub const LONG_FRAME_BYTES: usize = 14;

/// Outcome of CRC validation for a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrcOutcome {
    /// Syndrome is zero — frame is intact.
    Clean,
    /// One bit was flipped in transit; the bit position has been corrected in
    /// the caller's buffer. Position is a bit index `0..n*8`, MSB-first.
    Corrected { bit: u16 },
    /// Syndrome is non-zero and does not correspond to any single-bit error.
    Failed,
}

/// Compute the Mode S CRC-24 of `bytes`, MSB-first, no reflection, no final
/// XOR. The result occupies the low 24 bits of the returned `u32`.
///
/// A "clean" Mode S frame (DF 11/17/18) yields a return of zero when this
/// function is run over the whole frame including the embedded CRC bytes.
#[must_use]
pub fn crc24(bytes: &[u8]) -> u32 {
    let mut rem: u32 = 0;
    for &b in bytes {
        let idx = ((rem >> 16) as u8) ^ b;
        rem = ((rem << 8) ^ TABLE[idx as usize]) & 0x00FF_FFFF;
    }
    rem
}

/// Bit-at-a-time reference implementation. Identical output to [`crc24`].
///
/// Used in tests to verify [`TABLE`] is correct and available as a fallback
/// where the 1 KiB table cost is unacceptable.
#[must_use]
pub fn crc24_bitwise(bytes: &[u8]) -> u32 {
    let mut rem: u32 = 0;
    for &b in bytes {
        rem ^= (b as u32) << 16;
        for _ in 0..8 {
            if rem & 0x0080_0000 != 0 {
                rem = ((rem << 1) ^ POLY) & 0x00FF_FFFF;
            } else {
                rem = (rem << 1) & 0x00FF_FFFF;
            }
        }
    }
    rem
}

/// Validate a frame's CRC, attempting 1-bit correction on failure.
///
/// `bytes` must be exactly [`SHORT_FRAME_BYTES`] or [`LONG_FRAME_BYTES`] long;
/// the last 3 bytes are the embedded CRC. On `Corrected`, the offending bit
/// in `bytes` has been flipped in place to match the corrected frame.
///
/// This routine assumes a "clean" CRC (DF 11/17/18). For DF 4/5/20/21 the
/// caller must XOR the syndrome with each candidate ICAO; that lives in the
/// message-decoder layer, not here.
///
/// # Panics
///
/// Panics if `bytes.len()` is neither [`SHORT_FRAME_BYTES`] nor
/// [`LONG_FRAME_BYTES`]. Caller is responsible for length-gating; this is a
/// programmer error, not runtime input.
pub fn check(bytes: &mut [u8]) -> CrcOutcome {
    let n = bytes.len();
    assert!(
        n == SHORT_FRAME_BYTES || n == LONG_FRAME_BYTES,
        "Mode S frames are 7 or 14 bytes, got {n}",
    );
    let syndrome = crc24(bytes);
    if syndrome == 0 {
        return CrcOutcome::Clean;
    }
    if let Some(bit) = single_bit_correction(syndrome, n) {
        // Flip the bit in place.
        let byte = (bit / 8) as usize;
        let mask = 1u8 << (7 - (bit % 8));
        bytes[byte] ^= mask;
        debug_assert_eq!(crc24(bytes), 0, "correction did not yield zero syndrome");
        return CrcOutcome::Corrected { bit };
    }
    CrcOutcome::Failed
}

/// Find a single-bit-error position whose syndrome matches `syndrome`, or
/// `None`. `n` is the frame length in bytes.
fn single_bit_correction(syndrome: u32, n: usize) -> Option<u16> {
    let table = match n {
        SHORT_FRAME_BYTES => &SYNDROME_SHORT[..],
        LONG_FRAME_BYTES => &SYNDROME_LONG[..],
        _ => return None,
    };
    // Linear scan. Frames are 56 or 112 bits; this is a few microseconds at
    // worst and is only invoked on CRC failure, not the hot path. A sorted
    // table with binary search would be 2-3x faster but the gain is in the
    // noise next to the rest of the decode budget.
    table
        .iter()
        .position(|&s| s == syndrome)
        .map(|i| i as u16)
}

// --- Tables ------------------------------------------------------------------

/// Byte-at-a-time CRC table: `TABLE[b]` is `crc24_bitwise(&[b])`, i.e. the
/// remainder when feeding byte `b` into an all-zero register.
static TABLE: [u32; 256] = build_table();

const fn build_table() -> [u32; 256] {
    let mut t = [0u32; 256];
    let mut b = 0u32;
    while b < 256 {
        let mut rem = b << 16;
        let mut i = 0;
        while i < 8 {
            if rem & 0x0080_0000 != 0 {
                rem = ((rem << 1) ^ POLY) & 0x00FF_FFFF;
            } else {
                rem = (rem << 1) & 0x00FF_FFFF;
            }
            i += 1;
        }
        t[b as usize] = rem;
        b += 1;
    }
    t
}

/// `SYNDROME_SHORT[i]` is the CRC-24 of a 7-byte frame that is all zero
/// except for bit `i` (MSB-first). Used for 1-bit error correction on short
/// frames. Index `i` ranges over `0..56`.
static SYNDROME_SHORT: [u32; SHORT_FRAME_BYTES * 8] =
    build_syndrome_table::<{ SHORT_FRAME_BYTES * 8 }>(SHORT_FRAME_BYTES);

/// As [`SYNDROME_SHORT`] but for 14-byte frames; index `i` ranges over `0..112`.
static SYNDROME_LONG: [u32; LONG_FRAME_BYTES * 8] =
    build_syndrome_table::<{ LONG_FRAME_BYTES * 8 }>(LONG_FRAME_BYTES);

const fn build_syndrome_table<const N: usize>(bytes: usize) -> [u32; N] {
    let mut out = [0u32; N];
    let mut bit = 0;
    while bit < N {
        // Compute crc24 of a buffer that is all zero except bit `bit`.
        // We can do this without allocating: byte-at-a-time, with the target
        // byte XORed at the right position.
        let target_byte = bit / 8;
        let target_mask = 1u8 << (7 - (bit % 8));
        let mut rem: u32 = 0;
        let mut i = 0;
        while i < bytes {
            let b: u8 = if i == target_byte { target_mask } else { 0 };
            let idx = ((rem >> 16) as u8) ^ b;
            // Inline the table lookup. We can't use TABLE here because
            // `static` initializers can't reference other `static`s of the
            // same crate at const-eval time in stable Rust as of 1.85, so
            // we recompute the byte step.
            let mut step = (idx as u32) << 16;
            let mut k = 0;
            while k < 8 {
                if step & 0x0080_0000 != 0 {
                    step = ((step << 1) ^ POLY) & 0x00FF_FFFF;
                } else {
                    step = (step << 1) & 0x00FF_FFFF;
                }
                k += 1;
            }
            rem = ((rem << 8) ^ step) & 0x00FF_FFFF;
            i += 1;
        }
        out[bit] = rem;
        bit += 1;
    }
    out
}

// --- Tests -------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_matches_bitwise() {
        for b in 0u8..=255 {
            assert_eq!(crc24(&[b]), crc24_bitwise(&[b]));
        }
    }

    #[test]
    fn empty_input_is_zero() {
        assert_eq!(crc24(&[]), 0);
        assert_eq!(crc24_bitwise(&[]), 0);
    }

    #[test]
    fn known_vector_all_zero_short_frame() {
        // An all-zero 7-byte frame trivially has CRC zero, since the
        // generator is zero plus the remainder of zero. Pinned as a
        // base-case sanity check.
        let buf = [0u8; 7];
        assert_eq!(crc24(&buf), 0);
        assert_eq!(crc24_bitwise(&buf), 0);
    }

    /// Helper: build an n-byte frame whose first `n-3` bytes are `data` and
    /// whose last 3 bytes are the CRC computed over those `n-3` bytes,
    /// big-endian. Equivalent to what a Mode S transponder transmits.
    ///
    /// For this MSB-first, non-reflected CRC with no final XOR, embedding
    /// `crc24(data)` directly into the trailing three bytes yields a zero
    /// syndrome when the whole frame is run through `crc24`. (Verified
    /// experimentally; the appended-zeros variant does *not* round-trip.)
    fn build_frame(data: &[u8]) -> Vec<u8> {
        let crc = crc24(data);
        let mut frame = data.to_vec();
        frame.push(((crc >> 16) & 0xFF) as u8);
        frame.push(((crc >> 8) & 0xFF) as u8);
        frame.push((crc & 0xFF) as u8);
        frame
    }

    #[test]
    fn appending_crc_zeroes_syndrome_short() {
        let data = [0xA1, 0xB2, 0xC3, 0xD4];
        let frame = build_frame(&data);
        assert_eq!(frame.len(), SHORT_FRAME_BYTES);
        assert_eq!(crc24(&frame), 0, "round-trip syndrome should be zero");
    }

    #[test]
    fn appending_crc_zeroes_syndrome_long() {
        let data: Vec<u8> = (0u8..11).collect();
        let frame = build_frame(&data);
        assert_eq!(frame.len(), LONG_FRAME_BYTES);
        assert_eq!(crc24(&frame), 0);
    }

    #[test]
    fn check_clean_short_frame() {
        let data = [0x8D, 0x40, 0x62, 0x10];
        let mut frame = build_frame(&data);
        assert_eq!(check(&mut frame), CrcOutcome::Clean);
    }

    #[test]
    fn check_clean_long_frame() {
        let data: Vec<u8> = (0u8..11).rev().collect();
        let mut frame = build_frame(&data);
        assert_eq!(check(&mut frame), CrcOutcome::Clean);
    }

    #[test]
    fn check_corrects_every_single_bit_error_short() {
        let data = [0x12, 0x34, 0x56, 0x78];
        let clean = build_frame(&data);
        for flip in 0..(SHORT_FRAME_BYTES * 8) as u16 {
            let mut frame = clean.clone();
            let byte = (flip / 8) as usize;
            let mask = 1u8 << (7 - (flip % 8));
            frame[byte] ^= mask;
            match check(&mut frame) {
                CrcOutcome::Corrected { bit } => {
                    assert_eq!(bit, flip, "wrong corrected bit for flip {flip}");
                    assert_eq!(frame, clean, "corrected frame differs from clean");
                }
                other => panic!("expected Corrected, got {other:?} for flip {flip}"),
            }
        }
    }

    #[test]
    fn check_corrects_every_single_bit_error_long() {
        let data: Vec<u8> = (0u8..11).collect();
        let clean = build_frame(&data);
        for flip in 0..(LONG_FRAME_BYTES * 8) as u16 {
            let mut frame = clean.clone();
            let byte = (flip / 8) as usize;
            let mask = 1u8 << (7 - (flip % 8));
            frame[byte] ^= mask;
            match check(&mut frame) {
                CrcOutcome::Corrected { bit } => {
                    assert_eq!(bit, flip);
                    assert_eq!(frame, clean);
                }
                other => panic!("expected Corrected, got {other:?} for flip {flip}"),
            }
        }
    }

    #[test]
    fn check_fails_on_multi_bit_errors() {
        // Flip two well-separated bits. The combined syndrome should not
        // (with very high probability) coincide with any single-bit
        // syndrome. We pin one specific pair as a regression guard.
        let data = [0x12, 0x34, 0x56, 0x78];
        let clean = build_frame(&data);
        let mut frame = clean.clone();
        frame[0] ^= 0x80; // bit 0
        frame[3] ^= 0x01; // bit 31
        assert_eq!(check(&mut frame), CrcOutcome::Failed);
    }

    #[test]
    fn syndrome_tables_are_collision_free() {
        // The discriminating property of Mode S CRC-24 over 56/112 bits is
        // that all single-bit-error syndromes are distinct. If this ever
        // fires, 1-bit correction is structurally broken — escalate.
        let mut sorted: Vec<u32> = SYNDROME_SHORT.to_vec();
        sorted.sort_unstable();
        for w in sorted.windows(2) {
            assert_ne!(w[0], w[1], "duplicate syndrome in SHORT table");
        }
        let mut sorted: Vec<u32> = SYNDROME_LONG.to_vec();
        sorted.sort_unstable();
        for w in sorted.windows(2) {
            assert_ne!(w[0], w[1], "duplicate syndrome in LONG table");
        }
    }

    #[test]
    fn syndrome_tables_match_runtime_computation() {
        // Cross-check the const-evaluated tables against a runtime build
        // that uses the byte-table-driven crc24.
        for (i, &s) in SYNDROME_SHORT.iter().enumerate() {
            let mut buf = [0u8; SHORT_FRAME_BYTES];
            buf[i / 8] = 1u8 << (7 - (i % 8));
            assert_eq!(crc24(&buf), s, "SHORT syndrome mismatch at bit {i}");
        }
        for (i, &s) in SYNDROME_LONG.iter().enumerate() {
            let mut buf = [0u8; LONG_FRAME_BYTES];
            buf[i / 8] = 1u8 << (7 - (i % 8));
            assert_eq!(crc24(&buf), s, "LONG syndrome mismatch at bit {i}");
        }
    }

    #[test]
    #[should_panic(expected = "Mode S frames are 7 or 14 bytes")]
    fn check_panics_on_wrong_length() {
        let mut buf = [0u8; 8];
        let _ = check(&mut buf);
    }

    // --- Property tests -----------------------------------------------------

    use proptest::prelude::*;

    proptest! {
        #[test]
        fn prop_crc_roundtrip_short(data in proptest::array::uniform4(any::<u8>())) {
            let frame = build_frame(&data);
            prop_assert_eq!(crc24(&frame), 0);
            prop_assert_eq!(crc24_bitwise(&frame), 0);
        }

        #[test]
        fn prop_crc_roundtrip_long(data in proptest::collection::vec(any::<u8>(), 11)) {
            let frame = build_frame(&data);
            prop_assert_eq!(crc24(&frame), 0);
            prop_assert_eq!(crc24_bitwise(&frame), 0);
        }

        #[test]
        fn prop_table_matches_bitwise(data in proptest::collection::vec(any::<u8>(), 0..32)) {
            prop_assert_eq!(crc24(&data), crc24_bitwise(&data));
        }

        #[test]
        fn prop_single_bit_error_is_recoverable(
            data in proptest::array::uniform4(any::<u8>()),
            flip in 0u16..(SHORT_FRAME_BYTES * 8) as u16,
        ) {
            let clean = build_frame(&data);
            let mut frame = clean.clone();
            let byte = (flip / 8) as usize;
            let mask = 1u8 << (7 - (flip % 8));
            frame[byte] ^= mask;
            match check(&mut frame) {
                CrcOutcome::Corrected { bit } => {
                    prop_assert_eq!(bit, flip);
                    prop_assert_eq!(frame, clean);
                }
                other => prop_assert!(false, "expected Corrected, got {:?}", other),
            }
        }
    }
}

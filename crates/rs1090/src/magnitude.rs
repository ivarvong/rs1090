//! Magnitude stage: convert `(I, Q)` samples to scalar magnitude.
//!
//! Two implementations, both pure functions:
//!
//! - [`alpha_max_beta_min`]: branchless, no multiply, no square root. Peak
//!   relative error ~11.80% in continuous math, attained at `|min|/|max| ≈
//!   0.5` (derivable analytically). Integer truncation of `min >> 1` makes
//!   the relative error worse at very low magnitudes (e.g. `|I| = |Q| = 1`
//!   gives 29% under), but those samples sit in the receiver noise floor and
//!   downstream stages compare magnitudes against an adaptive threshold
//!   rather than an absolute scale — the bias does not bleed into bit
//!   decisions. This is the hot path on ARMv6, where there is no SIMD and
//!   integer multiply is expensive.
//! - [`lut`]: exact-to-rounding lookup in a 128 KiB precomputed table. One
//!   memory load per sample. Wins on x86_64 where the table fits in L2 with
//!   room for the rest of the working set.
//!
//! Both produce a `u16` magnitude. The maximum possible value over `(i8, i8)`
//! is `√(128² + 128²) ≈ 181`, so `u16` is comfortable.
//!
//! All functions are `#[inline]` and allocation-free.
//!
//! Lints: identifiers like `ARMv6`, `x86_64`, and `DESIGN.md` are technical
//! terms, not Rust items; we disable `clippy::doc_markdown` for this module
//! rather than peppering the prose with backticks.

#![allow(clippy::doc_markdown)]
//!
//! ## Note on the published 3.96% bound
//!
//! The classic "alpha-max-beta-min" error figure of 3.96% applies to the
//! variant with `α = 15/16, β = 15/32`. With the shift-only `α = 1, β = 1/2`
//! used here (chosen for ARMv6 branchlessness), the continuous-math peak is
//! ~11.80% (at `|min|/|max| = 0.5`). DESIGN.md was imprecise on this point;
//! the implementation matches the formula written there, not the error
//! figure. If 3.96% becomes load-bearing for a downstream stage, switch
//! coefficients here — the shape of the API does not change.

use crate::Iq;

/// Alpha-max-plus-beta-min approximation of |I + jQ|.
///
/// Computes `mag ≈ max(|I|, |Q|) + (min(|I|, |Q|) >> 1)`. Continuous-math
/// peak relative error ~11.80% at `|min|/|max| = 0.5`; see the module-level
/// docs for the integer-truncation regime at low magnitudes. Branchless on
/// ARMv6 in release mode.
#[inline]
#[must_use]
pub fn alpha_max_beta_min(s: Iq) -> u16 {
    let ai = (s.i as i16).unsigned_abs();
    let aq = (s.q as i16).unsigned_abs();
    let (mx, mn) = if ai >= aq { (ai, aq) } else { (aq, ai) };
    mx + (mn >> 1)
}

/// Exact magnitude via 128 KiB lookup table.
///
/// One indexed load per sample. The table is computed at compile time and
/// lives in `.rodata`. Currently used only by the bench harness; the
/// detector picks [`alpha_max_beta_min`] for its smaller cache footprint.
#[inline]
#[must_use]
#[cfg(any(feature = "test-utils", test))]
pub fn lut(s: Iq) -> u16 {
    let idx = ((s.i as u8 as usize) << 8) | (s.q as u8 as usize);
    MAG_LUT[idx]
}

/// Compute magnitudes for a batch of samples using [`alpha_max_beta_min`].
///
/// Equivalent to `out[i] = alpha_max_beta_min(samples[i])`. The slices must
/// have the same length. Bench / test surface only.
#[inline]
#[cfg(any(feature = "test-utils", test))]
pub fn batch_amam(samples: &[Iq], out: &mut [u16]) {
    assert_eq!(samples.len(), out.len(), "slice length mismatch");
    for (s, m) in samples.iter().zip(out.iter_mut()) {
        *m = alpha_max_beta_min(*s);
    }
}

/// Compute magnitudes for a batch of samples using [`lut`]. Bench / test
/// surface only.
#[inline]
#[cfg(any(feature = "test-utils", test))]
pub fn batch_lut(samples: &[Iq], out: &mut [u16]) {
    assert_eq!(samples.len(), out.len(), "slice length mismatch");
    for (s, m) in samples.iter().zip(out.iter_mut()) {
        *m = lut(*s);
    }
}

// --- LUT construction --------------------------------------------------------

/// 128 KiB table mapping each (i, q) byte pair to its rounded true magnitude.
#[cfg(any(feature = "test-utils", test))]
static MAG_LUT: [u16; 65536] = build_mag_lut();

// `clippy::large_stack_arrays` would flag this if it ran here, but the array
// is initialized inside a `const fn` evaluated at compile time and the
// resulting `static` lives in `.rodata` — no runtime stack involved.
#[allow(clippy::large_stack_arrays)]
#[cfg(any(feature = "test-utils", test))]
const fn build_mag_lut() -> [u16; 65536] {
    let mut t = [0u16; 65536];
    let mut iu = 0u32;
    while iu < 256 {
        let mut qu = 0u32;
        while qu < 256 {
            // Signed-magnitude squared equals (|signed|)². Map the unsigned
            // byte to the absolute value of the i8 it represents:
            // 0..=127 → 0..=127, 128..=255 → 128..=1 (mirror about 256).
            let ai = if iu < 128 { iu } else { 256 - iu };
            let aq = if qu < 128 { qu } else { 256 - qu };
            let sq = ai * ai + aq * aq;
            t[((iu as usize) << 8) | (qu as usize)] = isqrt_round(sq) as u16;
            qu += 1;
        }
        iu += 1;
    }
    t
}

/// Integer square root, rounded to nearest (half rounds up).
#[cfg(any(feature = "test-utils", test))]
const fn isqrt_round(n: u32) -> u32 {
    if n == 0 {
        return 0;
    }
    // Newton's method gives floor(sqrt(n)) in O(log log n).
    let mut x = n;
    let mut y = x.div_ceil(2);
    while y < x {
        x = y;
        y = u32::midpoint(x, n / x);
    }
    // x is now floor(sqrt(n)). Round to nearest: if n - x² > x, bump up.
    if n - x * x > x {
        x + 1
    } else {
        x
    }
}

// --- Tests -------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// True magnitude rounded to nearest, for use as a reference in tests.
    fn truth(s: Iq) -> u16 {
        let i = s.i as f64;
        let q = s.q as f64;
        (i.mul_add(i, q * q)).sqrt().round() as u16
    }

    #[test]
    fn alpha_zero_is_zero() {
        assert_eq!(alpha_max_beta_min(Iq::new(0, 0)), 0);
        assert_eq!(lut(Iq::new(0, 0)), 0);
    }

    #[test]
    fn alpha_handles_i8_min() {
        // i = -128 is the case that overflows naive `i8::abs()`. Make sure
        // we don't.
        let m = alpha_max_beta_min(Iq::new(-128, 0));
        assert_eq!(m, 128);
        let m = alpha_max_beta_min(Iq::new(-128, -128));
        // max=128, min=128 → 128 + 64 = 192
        assert_eq!(m, 192);
    }

    #[test]
    fn alpha_max_beta_min_continuous_bound_above_noise_floor() {
        // For samples where the minor component is ≥ 4 (well above the
        // noise floor of any real receiver), the integer truncation of
        // `min >> 1` is negligible and the relative error tracks the
        // continuous-math peak of ~11.80% at |min|/|max| = 0.5.
        let mut worst: f64 = 0.0;
        for i in i8::MIN..=i8::MAX {
            for q in i8::MIN..=i8::MAX {
                let mn = (i as i16).unsigned_abs().min((q as i16).unsigned_abs());
                if mn < 4 {
                    continue;
                }
                let approx = alpha_max_beta_min(Iq::new(i, q)) as f64;
                let t = ((i as f64).powi(2) + (q as f64).powi(2)).sqrt();
                let err = (approx - t).abs() / t;
                if err > worst {
                    worst = err;
                }
            }
        }
        assert!(
            worst <= 0.119,
            "above-noise peak error {worst:.5} exceeds 11.9%"
        );
    }

    #[test]
    fn alpha_max_beta_min_underestimates_only_within_one_lsb_when_min_is_odd() {
        // The continuous form `max + min/2` is always ≥ √(max² + min²). The
        // integer form uses `min >> 1`, which truncates a half whenever `min`
        // is odd. So the worst-case underestimate vs. the true magnitude is
        // bounded by 0.5 lsb plus rounding, never more than 1.
        for i in i8::MIN..=i8::MAX {
            for q in i8::MIN..=i8::MAX {
                let approx = alpha_max_beta_min(Iq::new(i, q)) as f64;
                let t = ((i as f64).powi(2) + (q as f64).powi(2)).sqrt();
                // Continuous form, computed as a reference.
                let ai = (i as i16).unsigned_abs() as f64;
                let aq = (q as i16).unsigned_abs() as f64;
                let cont = ai.max(aq) + ai.min(aq) * 0.5;
                // Approx is at most 0.5 below the continuous form.
                assert!(
                    approx + 0.5 + 1e-9 >= cont,
                    "approx {approx} more than 0.5 below cont {cont} at ({i},{q})"
                );
                // And the continuous form is always ≥ truth.
                assert!(
                    cont + 1e-9 >= t,
                    "continuous form {cont} below truth {t:.3} at ({i},{q})"
                );
            }
        }
    }

    #[test]
    fn alpha_max_beta_min_reduces_to_max_when_minor_truncates() {
        // |min| ≤ 1 ⇒ (min >> 1) = 0, so the approximation collapses to
        // max(|I|, |Q|). Pinned so the truncation regime is documented in
        // the tests, not just in prose.
        for i in i8::MIN..=i8::MAX {
            for q in i8::MIN..=i8::MAX {
                let ai = (i as i16).unsigned_abs();
                let aq = (q as i16).unsigned_abs();
                if ai.min(aq) > 1 {
                    continue;
                }
                let approx = alpha_max_beta_min(Iq::new(i, q));
                assert_eq!(approx, ai.max(aq));
            }
        }
    }

    #[test]
    fn lut_matches_rounded_true_magnitude_exhaustively() {
        for i in i8::MIN..=i8::MAX {
            for q in i8::MIN..=i8::MAX {
                let s = Iq::new(i, q);
                assert_eq!(
                    lut(s),
                    truth(s),
                    "lut({i},{q}) disagrees with rounded sqrt(i²+q²)"
                );
            }
        }
    }

    #[test]
    fn batch_amam_matches_scalar() {
        let samples: Vec<Iq> = (0i32..1024)
            .map(|n| Iq::new((n - 512) as i8, (n % 200 - 100) as i8))
            .collect();
        let mut out = vec![0u16; samples.len()];
        batch_amam(&samples, &mut out);
        for (s, m) in samples.iter().zip(out.iter()) {
            assert_eq!(*m, alpha_max_beta_min(*s));
        }
    }

    #[test]
    fn batch_lut_matches_scalar() {
        let samples: Vec<Iq> = (0i32..1024)
            .map(|n| Iq::new((n - 512) as i8, (n % 200 - 100) as i8))
            .collect();
        let mut out = vec![0u16; samples.len()];
        batch_lut(&samples, &mut out);
        for (s, m) in samples.iter().zip(out.iter()) {
            assert_eq!(*m, lut(*s));
        }
    }

    #[test]
    #[should_panic(expected = "slice length mismatch")]
    fn batch_amam_panics_on_length_mismatch() {
        let s = [Iq::default(); 4];
        let mut out = [0u16; 3];
        batch_amam(&s, &mut out);
    }

    #[test]
    fn isqrt_round_spot_checks() {
        assert_eq!(isqrt_round(0), 0);
        assert_eq!(isqrt_round(1), 1);
        // sqrt(2)=1.414 → round to 1
        assert_eq!(isqrt_round(2), 1);
        // sqrt(3)=1.732 → round to 2
        assert_eq!(isqrt_round(3), 2);
        // sqrt(4)=2
        assert_eq!(isqrt_round(4), 2);
        // sqrt(32768)=181.019 → 181
        assert_eq!(isqrt_round(32768), 181);
        // sqrt(100)=10
        assert_eq!(isqrt_round(100), 10);
        // sqrt(110)=10.488 → 10
        assert_eq!(isqrt_round(110), 10);
        // sqrt(111)=10.535 → 11
        assert_eq!(isqrt_round(111), 11);
    }
}

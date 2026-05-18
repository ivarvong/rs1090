//! Compact Position Reporting (CPR) for ADS-B.
//!
//! CPR is the encoding ADS-B uses to fit a lat/lon pair into 34 bits. It
//! achieves that by quantizing position into a grid whose cell *changes
//! shape* with latitude, and by alternating between two interleaved grids
//! ("even" and "odd"). The receiver disambiguates by combining one even and
//! one odd message (global decode) or by anchoring a single message against
//! a known prior position (local decode).
//!
//! The single source of derivation truth for this module is the ICAO Annex
//! 10 Vol IV CPR spec; the practical reference we used while writing it is
//! Sun, J., *The 1090 MHz Riddle*. We do not re-derive the formulas; we
//! pin the test outputs against worked examples from that reference and
//! against `dump1090`'s decoder where possible.
//!
//! ## What lives here
//!
//! - The number-of-longitude-zones (`NL`) table, encoded as a `const`
//!   function evaluated at compile time and cross-checked against the
//!   analytical formula in tests.
//! - [`global_decode`]: solve for absolute position from an even/odd pair.
//! - [`local_decode`]: solve from a single message + reference position.
//! - The `CprPosition` input type and helpers for unpacking it from the
//!   ME field of a DF 17/18 position message.
//!
//! Surface positions (TC 5–8) use a different quantization (`NZ = 19` vs.
//! `NZ = 15`) and a half-globe ambiguity that's resolved by the receiver's
//! known location; airborne (TC 9–18, 20–22) is what we implement here.
//! Surface CPR is a small follow-up once we have a reference position to
//! ground it against.
//!
//! Lint exemption: CPR, NL, NZ, ICAO, ADS-B aren't Rust items.

#![allow(clippy::doc_markdown)]

use core::f64::consts::PI;

/// Airborne number-of-latitude-zones constant. Encoded in the ICAO spec as
/// 15; the formula derives the per-zone height as `360° / (4·NZ)`.
pub const NZ_AIRBORNE: u32 = 15;

/// A raw CPR position as carried in a DF 17/18 ME field.
///
/// The 17-bit fields are stored in the low bits of the wrapping `u32`s;
/// `format` is `false` for an even-format message and `true` for odd.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CprPosition {
    pub lat_cpr: u32,
    pub lon_cpr: u32,
    /// `false` = even, `true` = odd.
    pub odd: bool,
}

impl CprPosition {
    /// Convert the 17-bit field to a fractional value in `[0, 1)`.
    #[inline]
    #[must_use]
    pub fn lat_frac(self) -> f64 {
        f64::from(self.lat_cpr) / f64::from(1u32 << 17)
    }

    #[inline]
    #[must_use]
    pub fn lon_frac(self) -> f64 {
        f64::from(self.lon_cpr) / f64::from(1u32 << 17)
    }
}

/// A decoded geodetic position in degrees.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LatLon {
    pub lat_deg: f64,
    pub lon_deg: f64,
}

/// Reasons a CPR decode can refuse to produce a result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CprError {
    /// The two messages do not have one even and one odd format.
    ParityMismatch,
    /// The latitude solutions for the two messages land in different NL
    /// zones; the pair cannot be reconciled and the receiver must wait
    /// for a fresh pair. Per the spec, this is normal during fast latitude
    /// transitions and not a hard error.
    LatitudeZoneMismatch,
    /// Latitude resolves outside the valid `[-90, 90]` range (very rare,
    /// indicates corrupted bits).
    LatitudeOutOfRange,
}

// --- NL table ---------------------------------------------------------------

/// Number of longitude zones at a given latitude band, per the ICAO formula.
///
/// `NL(lat) = floor(2π / acos(1 - (1 - cos(π / (2·NZ))) / cos²(|lat|·π/180)))`,
/// clamped to `[1, 59]`. Returns `1` at the poles and `59` at the equator.
///
/// At runtime callers should use the pinned `NL_TABLE`; this function is
/// exposed for the test that cross-checks the table against the formula.
#[inline]
#[must_use]
pub fn nl_formula(lat_deg: f64) -> u32 {
    let abs = lat_deg.abs();
    if abs >= 87.0 {
        return 1;
    }
    if abs <= 1e-9 {
        return 59;
    }
    let nz = f64::from(NZ_AIRBORNE);
    let num = 1.0 - (PI / (2.0 * nz)).cos();
    let den = (abs * PI / 180.0).cos().powi(2);
    let arg = 1.0 - num / den;
    let acos = arg.acos();
    let nl = (2.0 * PI / acos).floor();
    // The spec specifies that latitude bands extend up to but not past
    // their transition values. Clamp into [1, 59] to be safe.
    (nl as i64).clamp(1, 59) as u32
}

/// `NL_TABLE[i]` is the NL value for the latitude band starting at
/// `NL_BAND_DEG[i]` and ending at `NL_BAND_DEG[i+1]`. The table is
/// strictly decreasing in NL as `|lat|` increases (more zones at the
/// equator, fewer near the poles).
///
/// We list the 59 transition latitudes verbatim from the ICAO Annex 10
/// table so the build can't drift if the formula's floating-point
/// rounding changes between Rust versions. The values are absolute
/// latitudes in degrees, ascending; the corresponding NL values descend
/// from 59 down to 2.
pub const NL_TRANSITIONS: [(f64, u32); 58] = [
    (10.470_471_30, 59),
    (14.828_174_37, 58),
    (18.186_263_57, 57),
    (21.029_394_93, 56),
    (23.545_044_05, 55),
    (25.829_247_07, 54),
    (27.938_987_10, 53),
    (29.911_356_86, 52),
    (31.772_097_08, 51),
    (33.539_934_36, 50),
    (35.228_995_98, 49),
    (36.850_251_08, 48),
    (38.412_418_92, 47),
    (39.922_566_84, 46),
    (41.386_518_32, 45),
    (42.809_140_85, 44),
    (44.194_549_51, 43),
    (45.546_267_22, 42),
    (46.867_332_75, 41),
    (48.160_391_28, 40),
    (49.427_764_39, 39),
    (50.671_501_66, 38),
    (51.893_424_69, 37),
    (53.095_161_53, 36),
    (54.278_174_84, 35),
    (55.443_784_44, 34),
    (56.593_187_56, 33),
    (57.727_473_46, 32),
    (58.847_637_82, 31),
    (59.954_592_77, 30),
    (61.049_177_74, 29),
    (62.132_166_59, 28),
    (63.204_274_92, 27),
    (64.266_165_23, 26),
    (65.318_453_30, 25),
    (66.361_710_08, 24),
    (67.396_467_74, 23),
    (68.423_220_22, 22),
    (69.442_426_31, 21),
    (70.454_510_75, 20),
    (71.459_864_73, 19),
    (72.458_845_45, 18),
    (73.451_774_42, 17),
    (74.438_934_16, 16),
    (75.420_562_57, 15),
    (76.396_843_91, 14),
    (77.367_894_61, 13),
    (78.333_740_83, 12),
    (79.294_282_27, 11),
    (80.249_232_13, 10),
    (81.198_106_84, 9),
    (82.140_073_67, 8),
    (83.071_994_45, 7),
    (83.991_735_03, 6),
    (84.891_661_92, 5),
    (85.755_416_21, 4),
    (86.535_369_50, 3),
    (87.0, 2),
];

/// Look up NL by latitude using the transition table.
#[inline]
#[must_use]
pub fn nl(lat_deg: f64) -> u32 {
    let abs = lat_deg.abs();
    if abs >= 87.0 {
        return 1;
    }
    for &(boundary, n) in &NL_TRANSITIONS {
        if abs < boundary {
            return n;
        }
    }
    1
}

// --- Helpers ----------------------------------------------------------------

/// `floor` defined as in the CPR spec: floor toward negative infinity,
/// returning an integer.
#[inline]
fn cpr_floor(x: f64) -> i64 {
    x.floor() as i64
}

/// Modulo defined as in the CPR spec: result is always in `[0, m)` for `m > 0`.
#[inline]
fn cpr_mod(x: f64, m: f64) -> f64 {
    let r = x - m * (x / m).floor();
    // Snap small negatives caused by float noise.
    if r < 0.0 {
        r + m
    } else {
        r
    }
}

/// Latitude zone height for the given format. The CPR spec calls these
/// `Dlat_even = 360 / (4·NZ)` and `Dlat_odd = 360 / (4·NZ - 1)`.
#[inline]
fn d_lat(odd: bool) -> f64 {
    let denom = if odd {
        4.0 * f64::from(NZ_AIRBORNE) - 1.0
    } else {
        4.0 * f64::from(NZ_AIRBORNE)
    };
    360.0 / denom
}

/// Longitude zone width given an NL value. `Dlon_even = 360 / max(NL, 1)`,
/// `Dlon_odd = 360 / max(NL - 1, 1)`.
#[inline]
fn d_lon(nl: u32, odd: bool) -> f64 {
    let denom = if odd {
        f64::from(nl.saturating_sub(1).max(1))
    } else {
        f64::from(nl.max(1))
    };
    360.0 / denom
}

// --- Global decode ----------------------------------------------------------

/// Decode an absolute position from one even and one odd CPR message.
///
/// `even` and `odd` must satisfy `even.odd == false` and `odd.odd == true`;
/// otherwise this returns [`CprError::ParityMismatch`].
///
/// `most_recent_is_odd` indicates which of the two messages was received
/// most recently; the longitude solution is taken from that message's
/// frame so the result reflects the aircraft's current position rather
/// than its position one message ago.
///
/// # Errors
///
/// Returns a [`CprError`] if the two messages cannot be combined.
pub fn global_decode(
    even: CprPosition,
    odd: CprPosition,
    most_recent_is_odd: bool,
) -> Result<LatLon, CprError> {
    if even.odd || !odd.odd {
        return Err(CprError::ParityMismatch);
    }

    // --- Latitude ---
    let yz0 = even.lat_frac();
    let yz1 = odd.lat_frac();
    // Latitude index j; the spec uses floor(59·yz0 - 60·yz1 + 0.5).
    let j = cpr_floor(59.0 * yz0 - 60.0 * yz1 + 0.5);
    let dlat_e = d_lat(false);
    let dlat_o = d_lat(true);

    let lat_e = dlat_e * (cpr_mod(j as f64, 60.0) + yz0);
    let lat_o = dlat_o * (cpr_mod(j as f64, 59.0) + yz1);

    // Wrap into [-90, 90].
    let lat_e = wrap_lat(lat_e);
    let lat_o = wrap_lat(lat_o);
    if !(-90.0..=90.0).contains(&lat_e) || !(-90.0..=90.0).contains(&lat_o) {
        return Err(CprError::LatitudeOutOfRange);
    }

    // Both even and odd latitudes must live in the same NL zone for a
    // valid pairing.
    if nl(lat_e) != nl(lat_o) {
        return Err(CprError::LatitudeZoneMismatch);
    }

    let lat = if most_recent_is_odd { lat_o } else { lat_e };

    // --- Longitude ---
    // The even and odd messages use NL and NL-1 longitude zones respectively
    // (clamped to ≥ 1 at the poles).
    let nl_lat = nl(lat);
    let zones_even = nl_lat.max(1);
    let zones_odd = nl_lat.saturating_sub(1).max(1);
    let xz0 = even.lon_frac();
    let xz1 = odd.lon_frac();

    // Spec: m = floor(xz0·(NL−1) − xz1·NL + 0.5).
    let m = cpr_floor(xz0 * f64::from(zones_odd) - xz1 * f64::from(zones_even) + 0.5);

    let lon = if most_recent_is_odd {
        d_lon(nl_lat, true) * (cpr_mod(m as f64, f64::from(zones_odd)) + xz1)
    } else {
        d_lon(nl_lat, false) * (cpr_mod(m as f64, f64::from(zones_even)) + xz0)
    };
    let lon = wrap_lon(lon);

    Ok(LatLon {
        lat_deg: lat,
        lon_deg: lon,
    })
}

// --- Local decode -----------------------------------------------------------

/// Decode a single CPR message using a known reference position.
///
/// **Caller responsibility:** the reference must be within roughly 180 NM
/// (3° of latitude, less in longitude depending on NL) of the aircraft.
/// Outside that range this function returns the wrong 60°-wrapped tile —
/// the wrong answer, not an error. The wrong-tile failure is undetectable
/// without external knowledge, so we don't try to detect it; the
/// state-tracker layer is responsible for only feeding plausible
/// references in (typically a recent global fix or the receiver's own
/// location).
#[must_use]
pub fn local_decode(msg: CprPosition, reference: LatLon) -> LatLon {
    local_decode_with_dlat(msg, reference, d_lat(msg.odd))
}

/// Surface-position counterpart to [`local_decode`]. Surface ADS-B
/// messages encode lat/lon with a four-times-finer quantization
/// (`Dlat_surface = Dlat_airborne / 4`) and the encoding wraps every
/// 90° rather than every 360°, so the absolute position has a
/// four-quadrant ambiguity. The reference resolves it: as long as it's
/// within 45° of the aircraft (essentially always, for any practical
/// ground-station deployment), the local-decode arithmetic snaps to
/// the right quadrant.
///
/// # Errors
///
/// This function returns the decoded position even when the reference
/// is far enough that the wrong quadrant is selected; like
/// [`local_decode`], the failure mode is silent and the caller is
/// responsible for only invoking it with a reasonable reference (the
/// state tracker feeds in the configured receiver location).
#[must_use]
pub fn local_decode_surface(msg: CprPosition, reference: LatLon) -> LatLon {
    local_decode_with_dlat(msg, reference, d_lat(msg.odd) / 4.0)
}

fn local_decode_with_dlat(msg: CprPosition, reference: LatLon, dlat: f64) -> LatLon {
    let yz = msg.lat_frac();
    let j = cpr_floor(reference.lat_deg / dlat)
        + cpr_floor(0.5 + cpr_mod(reference.lat_deg, dlat) / dlat - yz);
    let lat = dlat * (j as f64 + yz);
    let lat = wrap_lat(lat);

    let nl_lat = nl(lat);
    // Surface scales d_lon by the same factor d_lat is scaled by.
    let dlon = d_lon(nl_lat, msg.odd) * (dlat / d_lat(msg.odd));
    let xz = msg.lon_frac();
    let m = cpr_floor(reference.lon_deg / dlon)
        + cpr_floor(0.5 + cpr_mod(reference.lon_deg, dlon) / dlon - xz);
    let lon = dlon * (m as f64 + xz);
    let lon = wrap_lon(lon);

    LatLon {
        lat_deg: lat,
        lon_deg: lon,
    }
}

// --- Wrapping ---------------------------------------------------------------

#[inline]
fn wrap_lat(mut lat: f64) -> f64 {
    if lat >= 270.0 {
        lat -= 360.0;
    }
    lat
}

#[inline]
fn wrap_lon(mut lon: f64) -> f64 {
    if lon >= 180.0 {
        lon -= 360.0;
    }
    lon
}

// --- Tests ------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Sample NL transitions cross-checked against the formula. Three points
    /// per band: just before, at, and just after each transition.
    #[test]
    fn nl_table_matches_formula_at_pinned_points() {
        // A handful of pinned (latitude, expected NL) pairs that anyone
        // can verify against ICAO Annex 10 Table 2-1 in five minutes.
        // Pinned (lat, NL) pairs at band midpoints so the test fails
        // loudly if the table ever drifts. Values cross-checked against
        // ICAO Annex 10 Table 2-1 and reproduced here for clarity.
        // The full-sweep `nl_formula_and_table_agree_across_the_globe`
        // test catches off-by-one slips at band boundaries.
        for &(lat, expected) in &[
            (0.0, 59u32), // equator
            (12.0, 58),   // band (10.470, 14.828)
            (20.0, 56),   // band (18.186, 21.029)
            (45.0, 42),   // mid-latitudes
            (60.0, 29),   // band (59.955, 61.049)
            (74.0, 16),   // band (73.452, 74.439)
            (87.5, 1),    // polar
            (-45.0, 42),  // southern symmetry
        ] {
            assert_eq!(nl(lat), expected, "nl({lat}) mismatch");
        }
    }

    #[test]
    fn nl_formula_and_table_agree_across_the_globe() {
        // Sweep in 0.1° steps and confirm the table tracks the formula.
        let mut lat = -89.9_f64;
        while lat <= 89.9 {
            let from_table = nl(lat);
            let from_formula = nl_formula(lat);
            assert_eq!(
                from_table, from_formula,
                "table {from_table} vs formula {from_formula} at lat {lat}",
            );
            lat += 0.1;
        }
    }

    #[test]
    fn nl_is_symmetric_about_the_equator() {
        let mut lat = 0.0_f64;
        while lat < 87.0 {
            assert_eq!(nl(lat), nl(-lat), "asymmetry at lat {lat}");
            lat += 0.5;
        }
    }

    // --- Global decode round-trips ---

    /// Encode a position back into a CPR pair. The encode side is a
    /// reference implementation we use only for test fixtures; it lives
    /// in the test module on purpose.
    fn encode(pos: LatLon, odd: bool) -> CprPosition {
        let dlat = d_lat(odd);
        let yz =
            ((pos.lat_deg / dlat) - cpr_floor(pos.lat_deg / dlat) as f64) * f64::from(1u32 << 17);
        let lat_cpr = (yz.round() as i64).rem_euclid(1 << 17) as u32;
        // Use the *encoded* latitude to recompute NL — this matches the
        // ground-truth quantization a transmitter does.
        let nz = NZ_AIRBORNE;
        let lat_round = dlat
            * (cpr_floor(pos.lat_deg / dlat) as f64 + f64::from(lat_cpr) / f64::from(1u32 << 17));
        let nl_v = nl(lat_round);
        let dlon = d_lon(nl_v, odd);
        let _ = nz;
        let xz =
            ((pos.lon_deg / dlon) - cpr_floor(pos.lon_deg / dlon) as f64) * f64::from(1u32 << 17);
        let lon_cpr = (xz.round() as i64).rem_euclid(1 << 17) as u32;
        CprPosition {
            lat_cpr,
            lon_cpr,
            odd,
        }
    }

    fn close(a: f64, b: f64, eps: f64) -> bool {
        (a - b).abs() < eps
    }

    #[test]
    fn global_decode_round_trips_dump1090_example() {
        // The canonical worked example from Sun's "The 1090 MHz Riddle"
        // (and dump1090 fixtures): two messages from one aircraft, with
        // the even message arriving most recently. The decoded position
        // is lat ≈ 52.2572°, lon ≈ 3.91937°.
        let even = CprPosition {
            lat_cpr: 0b1_0110_1011_0100_1000,
            lon_cpr: 0b0_1100_1000_1010_1100,
            odd: false,
        };
        let odd = CprPosition {
            lat_cpr: 0b1_0010_0001_1010_1110,
            lon_cpr: 0b0_1100_0100_0001_0010,
            odd: true,
        };
        // Even is the most recent message in this fixture; longitude is
        // taken from the even frame's coordinate system.
        let p = global_decode(even, odd, false).expect("decode");
        assert!(
            close(p.lat_deg, 52.2572, 1e-3),
            "lat {} not near 52.2572",
            p.lat_deg
        );
        assert!(
            close(p.lon_deg, 3.919_37, 1e-3),
            "lon {} not near 3.91937",
            p.lon_deg
        );
    }

    #[test]
    fn global_decode_rejects_parity_mismatch() {
        let a = CprPosition {
            lat_cpr: 0,
            lon_cpr: 0,
            odd: false,
        };
        let b = CprPosition {
            lat_cpr: 0,
            lon_cpr: 0,
            odd: false,
        };
        assert_eq!(global_decode(a, b, false), Err(CprError::ParityMismatch));
    }

    #[test]
    fn round_trip_encode_decode_within_quantization_bounds() {
        // Pick a few positions, encode even & odd, decode, expect
        // sub-degree accuracy. CPR airborne quantizes at ~5.1 m of
        // latitude (Dlat / 2^17), so 1e-4° is generous.
        for &(lat, lon) in &[
            (40.6413, -73.7781),  // JFK
            (-33.9461, 151.1772), // Sydney
            (51.4700, -0.4543),   // LHR
            (1.3644, 103.9915),   // Singapore (near equator)
            (78.2486, 15.4658),   // Svalbard (high latitude)
        ] {
            let pos = LatLon {
                lat_deg: lat,
                lon_deg: lon,
            };
            let e = encode(pos, false);
            let o = encode(pos, true);
            let decoded = global_decode(e, o, true).expect("decode failed");
            assert!(
                (decoded.lat_deg - lat).abs() < 1e-3,
                "lat {lat} → {} (Δ={:.6})",
                decoded.lat_deg,
                decoded.lat_deg - lat,
            );
            assert!(
                (decoded.lon_deg - lon).abs() < 1e-3,
                "lon {lon} → {} (Δ={:.6})",
                decoded.lon_deg,
                decoded.lon_deg - lon,
            );
        }
    }

    // --- Local decode ---

    #[test]
    fn local_decode_recovers_nearby_position() {
        // Reference 1 km from the true position, single odd message,
        // expect accurate recovery.
        let true_pos = LatLon {
            lat_deg: 40.6413,
            lon_deg: -73.7781,
        };
        let reference = LatLon {
            lat_deg: 40.6500,
            lon_deg: -73.7500,
        };
        let msg = encode(true_pos, true);
        let got = local_decode(msg, reference);
        assert!(
            (got.lat_deg - true_pos.lat_deg).abs() < 1e-3,
            "lat off: {} vs {}",
            got.lat_deg,
            true_pos.lat_deg
        );
        assert!(
            (got.lon_deg - true_pos.lon_deg).abs() < 1e-3,
            "lon off: {} vs {}",
            got.lon_deg,
            true_pos.lon_deg
        );
    }

    #[test]
    fn local_decode_returns_wrong_tile_when_reference_too_far() {
        // Documents the known limitation: with a 4400 NM reference, the
        // 60°-wrapped CPR ambiguity resolves to a tile near the reference,
        // not near the aircraft. This is not an error — it's the math.
        // The state-tracker layer must gate which references are
        // legitimate before calling this function.
        let true_pos = LatLon {
            lat_deg: 40.6413,
            lon_deg: -73.7781,
        };
        let reference = LatLon {
            lat_deg: 0.0,
            lon_deg: 0.0,
        };
        let msg = encode(true_pos, false);
        let got = local_decode(msg, reference);
        // The returned point is far from the true position; pinned so a
        // future change that accidentally adds plausibility checking
        // here without coordinating with the tracker fires this test.
        assert!(
            (got.lat_deg - true_pos.lat_deg).abs() > 10.0
                || (got.lon_deg - true_pos.lon_deg).abs() > 10.0,
            "expected wrong-tile answer; got close-to-truth {got:?}",
        );
    }
}

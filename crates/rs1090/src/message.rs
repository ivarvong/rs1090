//! Mode S / ADS-B message decoding.
//!
//! Two-level dispatch:
//!
//! - **Outer (DF):** picks the message family. DF 17/18 are the ADS-B
//!   extended squitters that carry positions, velocities, and identification.
//!   DF 11 surfaces ICAO addresses in the clear. The other DFs (4/5/16/20/21)
//!   carry surveillance replies whose CRC is XORed with the aircraft's ICAO
//!   address; validating them requires an active-address set we don't have
//!   in this milestone, so we surface them as raw bytes with the DF tag.
//!
//! - **Inner (TC, for DF 17/18):** the type code in the high 5 bits of the
//!   ME field selects the message variant: identification, position,
//!   velocity, status.
//!
//! The decoder is allocation-free. Callsigns are `arrayvec::ArrayString<8>`
//! to keep them stack-friendly while preserving the API ergonomics of a
//! string-like type.
//!
//! ## Field references
//!
//! All bit positions in this module follow the convention of *MSB-first
//! within each byte*, with bit 0 being the MSB of byte 0 of the frame. This
//! matches the on-the-wire ordering produced by [`crate::frame`].
//!
//! Lint exemption: technical terms (DF, TC, ME, ADS-B, ICAO, CPR, AC, MB)
//! aren't Rust items.

#![allow(clippy::doc_markdown)]

use arrayvec::ArrayString;

use crate::cpr::CprPosition;
use crate::frame::{DownlinkFormat, Frame};

// --- ICAO address -----------------------------------------------------------

/// The 24-bit ICAO aircraft address.
///
/// On the wire this is a big-endian 3-byte field; we store it as a `u32`
/// with the low 24 bits significant and the high 8 bits always zero. The
/// inner field is private so that invariant is enforced by construction;
/// use [`Icao::from_bytes`], [`Icao::from_u24`], or [`Icao::from_hex`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Icao(u32);

impl Icao {
    /// The all-zero address. Reserved on the wire — used as a sentinel for
    /// test fixtures and snapshot scaffolding.
    pub const ZERO: Self = Self(0);

    /// Construct from three big-endian bytes (the on-the-wire layout).
    #[inline]
    #[must_use]
    pub const fn from_bytes(b: [u8; 3]) -> Self {
        Self(((b[0] as u32) << 16) | ((b[1] as u32) << 8) | (b[2] as u32))
    }

    /// Construct from the low 24 bits of a `u32`. Returns `None` if any
    /// of the high 8 bits are set.
    #[inline]
    #[must_use]
    pub const fn from_u24(value: u32) -> Option<Self> {
        if value & 0xFF00_0000 == 0 {
            Some(Self(value))
        } else {
            None
        }
    }

    /// Parse from a six-character hex string, case-insensitive.
    /// Returns `None` if the length is not 6 or any character is non-hex.
    #[must_use]
    pub fn from_hex(s: &str) -> Option<Self> {
        if s.len() != 6 {
            return None;
        }
        let mut v: u32 = 0;
        for c in s.bytes() {
            let d = match c {
                b'0'..=b'9' => c - b'0',
                b'a'..=b'f' => c - b'a' + 10,
                b'A'..=b'F' => c - b'A' + 10,
                _ => return None,
            };
            v = (v << 4) | u32::from(d);
        }
        // 6 hex digits ≤ 24 bits, so the from_u24 check is provably
        // satisfied; the helper still does the right thing if this is
        // ever generalised.
        Self::from_u24(v)
    }

    /// The 24-bit address as a `u32`. The high 8 bits are always zero.
    #[inline]
    #[must_use]
    pub const fn as_u24(self) -> u32 {
        self.0
    }

    /// The three on-the-wire bytes, MSB first.
    #[inline]
    #[must_use]
    pub const fn to_bytes(self) -> [u8; 3] {
        [
            ((self.0 >> 16) & 0xFF) as u8,
            ((self.0 >> 8) & 0xFF) as u8,
            (self.0 & 0xFF) as u8,
        ]
    }

    /// Hex representation, six uppercase digits.
    #[must_use]
    pub fn to_hex(self) -> ArrayString<6> {
        const HEX: &[u8; 16] = b"0123456789ABCDEF";
        let mut s = ArrayString::<6>::new();
        for shift in (0..6).rev() {
            let nibble = (self.0 >> (shift * 4)) & 0xF;
            s.push(HEX[nibble as usize] as char);
        }
        s
    }
}

impl core::fmt::Display for Icao {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.to_hex().as_str())
    }
}

// --- Type code (DF 17/18 only) ---------------------------------------------

/// ADS-B type code (TC), occupying the high 5 bits of the ME field.
///
/// Variants cover what v0.1 decodes; the rest map to [`TypeCode::Other`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum TypeCode {
    /// TC 1–4: aircraft identification and category.
    Identification,
    /// TC 5–8: surface position (not decoded in v0.1; surface CPR uses a
    /// different quantization and an external reference).
    SurfacePosition,
    /// TC 9–18, 20–22: airborne position. Includes altitude (baro for 9–18,
    /// GNSS for 20–22) and the 17+17 CPR lat/lon.
    AirbornePosition,
    /// TC 19: airborne velocity.
    Velocity,
    /// TC 28: aircraft status (emergency, ACAS).
    AircraftStatus,
    /// TC 31: operational status (capability info).
    OperationalStatus,
    /// Anything else; the raw TC is preserved for diagnostics.
    Other(u8),
}

impl TypeCode {
    #[inline]
    #[must_use]
    pub const fn from_raw(tc: u8) -> Self {
        match tc {
            1..=4 => Self::Identification,
            5..=8 => Self::SurfacePosition,
            9..=18 | 20..=22 => Self::AirbornePosition,
            19 => Self::Velocity,
            28 => Self::AircraftStatus,
            31 => Self::OperationalStatus,
            other => Self::Other(other),
        }
    }

    #[inline]
    #[must_use]
    pub const fn raw_value(self) -> u8 {
        match self {
            Self::Identification => 4, // representative; the *range* is 1..=4
            Self::SurfacePosition => 8,
            Self::AirbornePosition => 11,
            Self::Velocity => 19,
            Self::AircraftStatus => 28,
            Self::OperationalStatus => 31,
            Self::Other(v) => v,
        }
    }
}

// --- Decoded payloads -------------------------------------------------------

/// Aircraft identification + emitter category (TC 1–4).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Identification {
    pub callsign: ArrayString<8>,
    /// Emitter category encoded with the TC: light, heavy, glider, etc.
    /// We expose the raw `(tc, category)` pair so callers can interpret
    /// against ICAO Annex 10 Vol IV Table 2-1 without us baking the entire
    /// table into the library.
    pub category_set: u8,
    pub category: u8,
}

/// Altitude reported in feet, tagged with its encoding/source.
///
/// Marked `#[non_exhaustive]` so we can add encodings (e.g. pressure
/// altitude variants from DF 4/20) without a breaking change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Altitude {
    /// Barometric altitude in feet, encoded with the Q-bit (25 ft resolution).
    BaroFeet(i32),
    /// Barometric altitude with the older Gillham (Gray-code) encoding,
    /// 100 ft resolution. Q-bit = 0.
    BaroGillhamFeet(i32),
    /// GNSS altitude in feet (TC 20–22).
    GnssFeet(i32),
    /// Altitude field was all zeros — the aircraft is not reporting.
    Unavailable,
}

impl Altitude {
    /// Numeric value in feet, regardless of encoding. `None` for
    /// [`Altitude::Unavailable`].
    #[inline]
    #[must_use]
    pub const fn feet(self) -> Option<i32> {
        match self {
            Self::BaroFeet(ft) | Self::BaroGillhamFeet(ft) | Self::GnssFeet(ft) => Some(ft),
            Self::Unavailable => None,
        }
    }

    /// Stable wire-format tag for the altitude source: `"baro"` (Q-bit or
    /// Gillham) or `"gnss"`. `None` for [`Altitude::Unavailable`]. The
    /// compiler enforces exhaustive matching here so adding a new
    /// variant fires a build error in exactly one place.
    #[inline]
    #[must_use]
    pub const fn source_tag(self) -> Option<&'static str> {
        match self {
            Self::BaroFeet(_) | Self::BaroGillhamFeet(_) => Some("baro"),
            Self::GnssFeet(_) => Some("gnss"),
            Self::Unavailable => None,
        }
    }
}

/// Airborne position (TC 9–18, 20–22).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AirbornePosition {
    pub altitude: Altitude,
    pub cpr: CprPosition,
}

/// Airborne velocity (TC 19), with the subtype-specific encoding flattened
/// into one struct. Heading is degrees-true clockwise from north; vertical
/// rate is feet per minute with positive = climb.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Velocity {
    pub kind: VelocityKind,
    pub vertical_rate_fpm: Option<i32>,
    /// Source of the vertical rate: baro or GNSS.
    pub vertical_rate_source: VerticalRateSource,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VelocityKind {
    /// Subtype 1 (subsonic) or 2 (supersonic): ground speed + heading.
    Ground { speed_kt: u16, heading_deg: f32 },
    /// Subtype 3 or 4: airspeed + heading.
    Airspeed {
        speed_kt: u16,
        heading_deg: Option<f32>,
        magnetic: bool,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VerticalRateSource {
    Baro,
    Gnss,
}

// --- Top-level message ------------------------------------------------------

/// A decoded Mode S / ADS-B message.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum Message {
    /// DF 17 / 18: extended squitter from a transponder or TIS-B source.
    ExtendedSquitter(ExtendedSquitter),
    /// DF 11: all-call reply. The ICAO comes from XORing the syndrome with
    /// the all-call interrogator code (0); see [`decode`].
    AllCallReply { icao: Icao },
    /// DF 0, 4, 5, 16, 20, 21: surveillance reply. The CRC is XORed with
    /// the aircraft's ICAO address; without an active-address set we
    /// cannot validate it here. We pass the original [`Frame`] through
    /// so the state tracker can attempt address-XOR recovery — encoding
    /// (hex, display, etc.) happens at the serialisation boundary, not
    /// here in the decoded type.
    SurveillanceReply { frame: Frame },
    /// Reserved/unknown DF.
    Other { df: DownlinkFormat },
}

/// An ADS-B extended squitter, dispatched by TC into a typed payload.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct ExtendedSquitter {
    pub icao: Icao,
    pub type_code: TypeCode,
    pub payload: SquitterPayload,
    /// Capability field (DF 17) or control field (DF 18); 3 bits.
    pub capability: u8,
}

#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum SquitterPayload {
    Identification(Identification),
    AirbornePosition(AirbornePosition),
    Velocity(Velocity),
    /// TC handled by dispatch but not yet decoded into a typed field.
    /// The 7-byte raw ME is preserved.
    Raw([u8; 7]),
}

// --- Errors -----------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// Frame is too short for the DF we read.
    TruncatedFrame,
    /// Velocity message has an unsupported subtype (we only decode 1–4).
    UnsupportedVelocitySubtype(u8),
}

// --- Top-level decode -------------------------------------------------------

/// Decode a [`Frame`] into a structured [`Message`].
///
/// This is a pure function over the frame's byte payload. It does **not**
/// validate the CRC — that's the frame layer's job, and the frame may
/// arrive with `CrcOutcome::Failed` (e.g. DF 4/5/20/21 whose CRC is
/// address-XORed). The decoder takes the bytes at face value.
///
/// # Errors
///
/// Returns [`DecodeError`] when the frame contents are structurally
/// impossible to interpret (e.g. truncated buffer).
pub fn decode(frame: &Frame) -> Result<Message, DecodeError> {
    let bytes = frame.bytes();
    let df = frame.downlink_format();
    match df {
        DownlinkFormat::ExtendedSquitter | DownlinkFormat::TisB => {
            if bytes.len() < 14 {
                return Err(DecodeError::TruncatedFrame);
            }
            Ok(Message::ExtendedSquitter(decode_extended_squitter(bytes)))
        }
        DownlinkFormat::AllCallReply => {
            // DF 11: bytes 1..4 carry the AA (announced address) directly
            // when the syndrome is zero. With clean CRC the frame's
            // address is at bytes 1..=3.
            if bytes.len() < 7 {
                return Err(DecodeError::TruncatedFrame);
            }
            let icao = Icao::from_bytes([bytes[1], bytes[2], bytes[3]]);
            Ok(Message::AllCallReply { icao })
        }
        DownlinkFormat::AltitudeReply
        | DownlinkFormat::IdentityReply
        | DownlinkFormat::ShortAcas
        | DownlinkFormat::LongAcas
        | DownlinkFormat::CommBAltitude
        | DownlinkFormat::CommBIdentity => {
            Ok(Message::SurveillanceReply { frame: *frame })
        }
        DownlinkFormat::Reserved(_) => Ok(Message::Other { df }),
    }
}

// --- DF 17/18 internals -----------------------------------------------------

fn decode_extended_squitter(bytes: &[u8]) -> ExtendedSquitter {
    let capability = bytes[0] & 0x07;
    let icao = Icao::from_bytes([bytes[1], bytes[2], bytes[3]]);
    // ME field: bytes 4..=10 (7 bytes, 56 bits).
    let me: [u8; 7] = bytes[4..11].try_into().expect("DF17 has 7-byte ME");
    let tc_raw = me[0] >> 3;
    let type_code = TypeCode::from_raw(tc_raw);

    let payload = match type_code {
        TypeCode::Identification => SquitterPayload::Identification(decode_identification(me)),
        TypeCode::AirbornePosition => {
            SquitterPayload::AirbornePosition(decode_airborne_position(me))
        }
        TypeCode::Velocity => decode_velocity(me).map_or(SquitterPayload::Raw(me), SquitterPayload::Velocity),
        _ => SquitterPayload::Raw(me),
    };

    ExtendedSquitter {
        icao,
        type_code,
        payload,
        capability,
    }
}

// --- Bit reader -------------------------------------------------------------

/// MSB-first reader over a byte slice using 1-indexed bit positions.
///
/// Mode S / ADS-B field layouts are documented in DO-260B / ICAO Annex 10
/// using 1-indexed positions counted from the MSB of byte 0. Manual
/// `(bits >> (N - position)) & mask` shifts get ahead of the reader fast
/// — a 56-bit ME field with a dozen fields is hard to audit against the
/// spec. This helper lets the call sites match the spec literally:
///
/// ```ignore
/// let r = BitReader::new(&me);
/// let subtype = r.bits(6, 3);   // ME bits 6..=8
/// let lat_cpr = r.bits(23, 17); // ME bits 23..=39
/// ```
struct BitReader<'a> {
    bytes: &'a [u8],
}

impl<'a> BitReader<'a> {
    #[inline]
    const fn new(bytes: &'a [u8]) -> Self {
        Self { bytes }
    }

    /// Read `len` bits starting at 1-indexed position `pos`, MSB-first.
    /// The result is right-aligned in the returned `u32`.
    ///
    /// # Panics
    /// Panics in debug builds if the request is out of range or larger
    /// than 32 bits. The decoder calls only ever request fields that
    /// fit the spec; corrupt inputs are filtered at the frame layer.
    #[inline]
    fn bits(&self, pos: usize, len: u8) -> u32 {
        debug_assert!(pos >= 1, "BitReader uses 1-indexed positions");
        debug_assert!(len > 0 && len <= 32);
        debug_assert!(pos - 1 + len as usize <= self.bytes.len() * 8);

        let start = pos - 1; // 0-indexed from MSB of byte 0
        let end = start + len as usize; // exclusive
        let first_byte = start / 8;
        let last_byte = (end - 1) / 8;

        // Load up to 5 bytes (40 bits) into a u64 accumulator, then
        // shift to right-align. 5 bytes covers any 32-bit field across
        // any byte alignment.
        let mut acc: u64 = 0;
        for &b in &self.bytes[first_byte..=last_byte] {
            acc = (acc << 8) | u64::from(b);
        }
        let trailing = (last_byte + 1) * 8 - end;
        let mask = if len == 32 {
            u64::from(u32::MAX)
        } else {
            (1u64 << len) - 1
        };
        ((acc >> trailing) & mask) as u32
    }

    /// Read a single bit at 1-indexed position `pos`.
    #[inline]
    fn bit(&self, pos: usize) -> bool {
        debug_assert!(pos >= 1);
        debug_assert!(pos <= self.bytes.len() * 8);
        let byte = (pos - 1) / 8;
        let bit = 7 - ((pos - 1) % 8);
        (self.bytes[byte] >> bit) & 1 != 0
    }
}

// --- Identification ---------------------------------------------------------

/// Map a 6-bit character code (per ICAO Annex 10 Vol IV §3.1.2.9) to a
/// printable ASCII character. Reserved codes map to `'_'`.
#[inline]
fn callsign_char(code: u8) -> char {
    // The table is laid out so codes 1..=26 are A..=Z, 32 is space, and
    // 48..=57 are '0'..='9'. Everything else is reserved.
    match code {
        1..=26 => (b'A' + (code - 1)) as char,
        32 => ' ',
        48..=57 => (b'0' + (code - 48)) as char,
        _ => '_',
    }
}

fn decode_identification(me: [u8; 7]) -> Identification {
    let category_set = me[0] >> 3; // = TC, 1..=4
    let category = me[0] & 0x07;

    // Eight 6-bit characters packed starting at bit 8 of the ME (i.e.
    // byte 1 onwards). Layout: 8 chars × 6 bits = 48 bits, occupying
    // bytes 1..7.
    let mut bits: u64 = 0;
    for &b in &me[1..7] {
        bits = (bits << 8) | u64::from(b);
    }
    // bits now has 48 bits in the low 48 positions.
    let mut callsign = ArrayString::<8>::new();
    for k in 0..8 {
        let shift = (7 - k) * 6;
        let code = ((bits >> shift) & 0x3F) as u8;
        callsign.push(callsign_char(code));
    }
    // Strip trailing spaces and underscores for ergonomics — many
    // callsigns are padded with spaces.
    while matches!(callsign.as_str().chars().last(), Some(' ' | '_')) {
        callsign.pop();
    }

    Identification {
        callsign,
        category_set,
        category,
    }
}

// --- Airborne position ------------------------------------------------------

fn decode_airborne_position(me: [u8; 7]) -> AirbornePosition {
    // ME bit layout per DO-260B (1-indexed MSB-first):
    //   bits  1..=5  : TC
    //   bits  6..=7  : surveillance status
    //   bit   8      : NIC supplement B (TC 9-18) or single-antenna flag
    //   bits  9..=20 : AC12 altitude (Q-bit at position 13)
    //   bit  21      : T flag
    //   bit  22      : F flag (CPR even/odd)
    //   bits 23..=39 : lat_cpr (17 bits)
    //   bits 40..=56 : lon_cpr (17 bits)
    let r = BitReader::new(&me);
    #[allow(clippy::cast_possible_truncation)]
    let tc = r.bits(1, 5) as u8;
    #[allow(clippy::cast_possible_truncation)]
    let ac12 = r.bits(9, 12) as u16;
    let f_flag = r.bit(22);
    let lat_cpr = r.bits(23, 17);
    let lon_cpr = r.bits(40, 17);

    AirbornePosition {
        altitude: decode_altitude_ac12(ac12, tc),
        cpr: CprPosition {
            lat_cpr,
            lon_cpr,
            odd: f_flag,
        },
    }
}

/// Decode the 12-bit AC12 altitude field.
///
/// `tc` selects baro (TC 9–18) vs GNSS (TC 20–22). The Q-bit at position 4
/// within the field distinguishes 25-ft from 100-ft Gillham encoding.
#[inline]
fn decode_altitude_ac12(ac12: u16, tc: u8) -> Altitude {
    if ac12 == 0 {
        return Altitude::Unavailable;
    }

    // GNSS altitudes (TC 20-22) are reported in meters and we convert to ft.
    if matches!(tc, 20..=22) {
        let meters = i32::from(ac12);
        let feet = (meters as f64 * 3.280_84) as i32;
        return Altitude::GnssFeet(feet);
    }

    // Baro altitude.
    let q_bit = (ac12 >> 4) & 1;
    if q_bit == 1 {
        // 25-ft encoding. The 11-bit altitude value is the AC12 with the Q
        // bit removed; the result times 25 minus 1000 gives altitude in
        // feet. Reference: dump1090 modesAltitudeM1090.c.
        let n = (i32::from(ac12 & 0x0F)) | ((i32::from(ac12 >> 5)) << 4);
        Altitude::BaroFeet(n * 25 - 1000)
    } else {
        // Gillham (Mode C) encoding. We expose the raw value × 100 ft for
        // now; full Gray-code decoding is a future enhancement.
        Altitude::BaroGillhamFeet(i32::from(ac12) * 100 - 1000)
    }
}

// --- Velocity ---------------------------------------------------------------

fn decode_velocity(me: [u8; 7]) -> Result<Velocity, DecodeError> {
    // ME bit layout for TC=19 (velocity), 1-indexed MSB-first:
    //   bits  1..=5  : TC (19)
    //   bits  6..=8  : subtype (1=GS subsonic, 2=GS supersonic,
    //                            3=AS subsonic, 4=AS supersonic)
    //   bit   9      : intent change flag
    //   bit  10      : IFR capability
    //   bits 11..=13 : NACv
    //
    //   Subtype 1/2 (ground velocity):
    //     bit  14     : EW direction (0=east, 1=west)
    //     bits 15..=24: EW velocity (raw-1; 0 = unavailable)
    //     bit  25     : NS direction (0=north, 1=south)
    //     bits 26..=35: NS velocity
    //
    //   Subtype 3/4 (airspeed):
    //     bit  14     : heading status (1=heading valid)
    //     bits 15..=24: heading (× 360/1024 = deg)
    //     bit  25     : airspeed type (0=IAS, 1=TAS)
    //     bits 26..=35: airspeed (raw-1)
    //
    //   Common to all subtypes:
    //     bit  36     : VR source (0=baro, 1=GNSS)
    //     bit  37     : VR sign (1 = descending)
    //     bits 38..=46: VR magnitude (raw-1, × 64 = ft/min)
    let r = BitReader::new(&me);
    #[allow(clippy::cast_possible_truncation)]
    let subtype = r.bits(6, 3) as u8;
    let vr_source = if r.bit(36) {
        VerticalRateSource::Gnss
    } else {
        VerticalRateSource::Baro
    };
    let vr_sign = r.bit(37);
    #[allow(clippy::cast_possible_wrap)]
    let vr_mag = r.bits(38, 9) as i32;
    let vertical_rate_fpm = if vr_mag == 0 {
        None
    } else {
        let v = (vr_mag - 1) * 64;
        Some(if vr_sign { -v } else { v })
    };

    let kind = match subtype {
        1 | 2 => {
            let ew_sign = r.bit(14); // 1 = west
            #[allow(clippy::cast_possible_wrap)]
            let ew_v = r.bits(15, 10) as i32;
            let ns_sign = r.bit(25); // 1 = south
            #[allow(clippy::cast_possible_wrap)]
            let ns_v = r.bits(26, 10) as i32;

            // Both fields encode magnitude as `raw - 1`; 0 means
            // "not available". A missing component still produces a
            // velocity at the other axis, rather than failing decode.
            let ew = if ew_v == 0 { 0 } else { ew_v - 1 };
            let ns = if ns_v == 0 { 0 } else { ns_v - 1 };
            let ew = if ew_sign { -ew } else { ew };
            let ns = if ns_sign { -ns } else { ns };

            // Supersonic subtypes (2) scale by 4.
            let scale = if subtype == 2 { 4.0 } else { 1.0 };
            let ew_f = f64::from(ew) * scale;
            let ns_f = f64::from(ns) * scale;
            // Truncate (don't round) to match pyModeS and the typical
            // `(int)sqrt(...)` idiom in C reference decoders. DO-260B
            // doesn't specify; the difference is bounded by 1 kt.
            let speed = ew_f.hypot(ns_f).trunc() as u16;
            // Heading is degrees clockwise from north: atan2(east, north).
            let heading = ew_f.atan2(ns_f).to_degrees();
            let heading = if heading < 0.0 { heading + 360.0 } else { heading };
            VelocityKind::Ground {
                speed_kt: speed,
                heading_deg: heading as f32,
            }
        }
        3 | 4 => {
            let heading_valid = r.bit(14);
            let heading_raw = r.bits(15, 10);
            let airspeed_type_mach = r.bit(25);
            let airspeed_raw = r.bits(26, 10);
            let speed = if airspeed_raw == 0 {
                0
            } else if subtype == 4 {
                (airspeed_raw - 1) * 4
            } else {
                airspeed_raw - 1
            } as u16;
            let heading_deg = if heading_valid {
                Some((heading_raw as f32) * 360.0 / 1024.0)
            } else {
                None
            };
            VelocityKind::Airspeed {
                speed_kt: speed,
                heading_deg,
                magnetic: !airspeed_type_mach,
            }
        }
        other => return Err(DecodeError::UnsupportedVelocitySubtype(other)),
    };

    Ok(Velocity {
        kind,
        vertical_rate_fpm,
        vertical_rate_source: vr_source,
    })
}

// --- Tests ------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn icao(bytes: [u8; 3]) -> u32 {
        Icao::from_bytes(bytes).as_u24()
    }

    #[test]
    fn icao_bytes_roundtrip() {
        let i = Icao::from_bytes([0xA1, 0xB2, 0xC3]);
        assert_eq!(i.as_u24(), 0x00A1_B2C3);
        assert_eq!(i.to_bytes(), [0xA1, 0xB2, 0xC3]);
        assert_eq!(icao([0x00, 0x00, 0x01]), 1);
    }

    #[test]
    fn icao_display_and_hex_are_uppercase_hex() {
        let i = Icao::from_bytes([0xa1, 0xb2, 0xc3]);
        assert_eq!(format!("{i}"), "A1B2C3");
        assert_eq!(i.to_hex().as_str(), "A1B2C3");
    }

    #[test]
    fn icao_from_u24_rejects_high_bits() {
        assert_eq!(Icao::from_u24(0x00FF_FFFF).map(Icao::as_u24), Some(0x00FF_FFFF));
        assert_eq!(Icao::from_u24(0x0100_0000), None);
        assert_eq!(Icao::from_u24(0xFFFF_FFFF), None);
    }

    #[test]
    fn icao_from_hex_accepts_both_cases_and_rejects_garbage() {
        assert_eq!(Icao::from_hex("A1B2C3"), Some(Icao::from_bytes([0xA1, 0xB2, 0xC3])));
        assert_eq!(Icao::from_hex("a1b2c3"), Some(Icao::from_bytes([0xA1, 0xB2, 0xC3])));
        assert_eq!(Icao::from_hex("000001"), Some(Icao::from_bytes([0, 0, 1])));
        assert_eq!(Icao::from_hex(""), None);
        assert_eq!(Icao::from_hex("A1B2C"), None); // too short
        assert_eq!(Icao::from_hex("A1B2C3D"), None); // too long
        assert_eq!(Icao::from_hex("ZZZZZZ"), None);
        assert_eq!(Icao::from_hex("A1B2C!"), None);
    }

    #[test]
    fn typecode_dispatch_matches_spec_ranges() {
        for tc in 1..=4 {
            assert_eq!(TypeCode::from_raw(tc), TypeCode::Identification);
        }
        for tc in 5..=8 {
            assert_eq!(TypeCode::from_raw(tc), TypeCode::SurfacePosition);
        }
        for tc in (9..=18).chain(20..=22) {
            assert_eq!(TypeCode::from_raw(tc), TypeCode::AirbornePosition);
        }
        assert_eq!(TypeCode::from_raw(19), TypeCode::Velocity);
        assert_eq!(TypeCode::from_raw(28), TypeCode::AircraftStatus);
        assert_eq!(TypeCode::from_raw(31), TypeCode::OperationalStatus);
        // Unallocated codes round-trip through Other.
        assert_eq!(TypeCode::from_raw(0), TypeCode::Other(0));
        assert_eq!(TypeCode::from_raw(23), TypeCode::Other(23));
    }

    // --- Callsign decoding ---

    #[test]
    fn callsign_char_covers_alphabet_digits_space() {
        // Letters: A=1, Z=26.
        for k in 0..26u8 {
            assert_eq!(callsign_char(1 + k), (b'A' + k) as char);
        }
        // Digits: 0=48, 9=57.
        for k in 0..10u8 {
            assert_eq!(callsign_char(48 + k), (b'0' + k) as char);
        }
        assert_eq!(callsign_char(32), ' ');
        // Reserved → underscore.
        assert_eq!(callsign_char(0), '_');
        assert_eq!(callsign_char(27), '_');
        assert_eq!(callsign_char(63), '_');
    }

    /// Pack 8 six-bit codes into a 7-byte ME field starting at byte 1.
    fn pack_identification_me(tc: u8, cat: u8, codes: [u8; 8]) -> [u8; 7] {
        let mut me = [0u8; 7];
        me[0] = (tc << 3) | (cat & 0x07);
        let mut bits: u64 = 0;
        for c in codes {
            bits = (bits << 6) | u64::from(c & 0x3F);
        }
        // bits is in the low 48; place into bytes 1..7 (big-endian).
        for i in 0..6 {
            me[1 + i] = ((bits >> (8 * (5 - i))) & 0xFF) as u8;
        }
        me
    }

    #[test]
    fn identification_decodes_classic_callsign() {
        // "KLM1023 " (trailing space) — well-known dump1090 fixture style.
        let codes = [
            11, // K
            12, // L
            13, // M
            49, // 1
            48, // 0
            50, // 2
            51, // 3
            32, // space
        ];
        let me = pack_identification_me(4, 0, codes);
        let id = decode_identification(me);
        assert_eq!(id.callsign.as_str(), "KLM1023");
        assert_eq!(id.category_set, 4);
        assert_eq!(id.category, 0);
    }

    #[test]
    fn identification_strips_trailing_padding() {
        // All spaces → empty string after stripping.
        let me = pack_identification_me(4, 0, [32; 8]);
        let id = decode_identification(me);
        assert_eq!(id.callsign.as_str(), "");
    }

    // --- Altitude ---

    #[test]
    fn altitude_zero_is_unavailable() {
        let alt = decode_altitude_ac12(0, 11);
        assert_eq!(alt, Altitude::Unavailable);
    }

    #[test]
    fn altitude_q1_decodes_baro_25ft_steps() {
        // AC12 with Q=1 and value such that altitude = 38000 ft.
        // 38000 = n*25 - 1000  →  n = 1560 = 0b110_0001_1000
        // AC12 layout: upper 7 bits of n in positions 5..=11, Q at 4, low 4 of n in 0..=3.
        let n: u16 = 1560;
        let upper = (n >> 4) & 0x7F; // 7 bits
        let lower = n & 0x0F; // 4 bits
        let ac12 = (upper << 5) | (1 << 4) | lower;
        let alt = decode_altitude_ac12(ac12, 11);
        assert_eq!(alt, Altitude::BaroFeet(38_000));
    }

    #[test]
    fn altitude_q0_is_gillham_placeholder() {
        // Q=0, raw value 100 → reported as (100 * 100 - 1000) ft. Pinned
        // so a future move to a real Gray-code decode is a visible change.
        let ac12: u16 = 100; // Q bit at position 4 is zero
        let alt = decode_altitude_ac12(ac12, 11);
        assert_eq!(alt, Altitude::BaroGillhamFeet(9000));
    }

    // --- Airborne position ---

    #[test]
    fn airborne_position_unpacks_cpr_bits() {
        // Construct an ME with TC=11, F=1 (odd), lat_cpr=0x1F1F0, lon_cpr=0x0DEAD.
        // Bit layout in ME (56 bits, MSB first):
        //   [TC:5][SS:2][NICb:1][AC12:12][T:1][F:1][lat:17][lon:17]
        let tc: u64 = 11;
        let ss: u64 = 0;
        let nicb: u64 = 0;
        let ac12: u64 = 0; // unavailable
        let t: u64 = 0;
        let f: u64 = 1;
        let lat: u64 = 0x1F1F0;
        let lon: u64 = 0x0DEAD;
        let me_bits: u64 = (tc << 51)
            | (ss << 49)
            | (nicb << 48)
            | (ac12 << 36)
            | (t << 35)
            | (f << 34)
            | (lat << 17)
            | lon;
        let mut me = [0u8; 7];
        for (i, byte) in me.iter_mut().enumerate() {
            *byte = ((me_bits >> (8 * (6 - i))) & 0xFF) as u8;
        }
        let pos = decode_airborne_position(me);
        assert!(pos.cpr.odd);
        assert_eq!(pos.cpr.lat_cpr, 0x1_F1F0);
        assert_eq!(pos.cpr.lon_cpr, 0x0_DEAD);
        assert_eq!(pos.altitude, Altitude::Unavailable);
    }

    // --- Top-level decode + DF dispatch ---

    use crate::crc;
    use crate::frame::{DownlinkFormat, Frame, FrameDetector};

    /// Synth a DF 17 long frame carrying the given ME bytes, with a valid
    /// CRC. Returns the 14-byte frame.
    fn synth_df17_frame(icao: [u8; 3], capability: u8, me: [u8; 7]) -> [u8; 14] {
        let mut data = [0u8; 11];
        data[0] = (17u8 << 3) | (capability & 0x07);
        data[1] = icao[0];
        data[2] = icao[1];
        data[3] = icao[2];
        data[4..11].copy_from_slice(&me);
        let crc_val = crc::crc24(&data);
        let mut f = [0u8; 14];
        f[..11].copy_from_slice(&data);
        f[11] = ((crc_val >> 16) & 0xFF) as u8;
        f[12] = ((crc_val >> 8) & 0xFF) as u8;
        f[13] = (crc_val & 0xFF) as u8;
        debug_assert_eq!(crc::crc24(&f), 0);
        f
    }

    /// Feed bytes-as-a-frame through the detector by synthesizing a
    /// magnitude stream + iq samples. Reuses the path the rest of the
    /// pipeline takes, so the test exercises the real assembly.
    fn frame_from_bytes(frame_bytes: [u8; 14]) -> Frame {
        use crate::demod::{
            synth_bits_as_magnitude, PREAMBLE_HIGH_IDX, PREAMBLE_SAMPLES, SAMPLES_PER_BIT,
        };
        use crate::Iq;

        let payload_bits = frame_bytes.len() * 8;
        let payload_samples = payload_bits * SAMPLES_PER_BIT;
        let total = 64 + PREAMBLE_SAMPLES + payload_samples + 64;
        let mut mags = vec![5u16; total];
        let pre = 64;
        for &k in &PREAMBLE_HIGH_IDX {
            mags[pre + k] = 120;
        }
        let mut bits = vec![false; payload_bits];
        for (i, b) in frame_bytes.iter().enumerate() {
            for k in 0..8 {
                bits[i * 8 + k] = (b >> (7 - k)) & 1 != 0;
            }
        }
        synth_bits_as_magnitude(
            &bits,
            120,
            5,
            &mut mags[pre + PREAMBLE_SAMPLES..pre + PREAMBLE_SAMPLES + payload_samples],
        );
        let samples: Vec<Iq> = mags
            .iter()
            .map(|&m| Iq::new(m.min(127) as i8, 0))
            .collect();
        let mut det = FrameDetector::new();
        det.reset_noise_floor(5);
        let mut out = Vec::new();
        det.process(&samples, |f| out.push(*f));
        assert_eq!(out.len(), 1);
        out[0]
    }

    #[test]
    fn decode_identifies_extended_squitter_callsign() {
        let codes = [11, 12, 13, 49, 48, 50, 51, 32];
        let me = pack_identification_me(4, 0, codes);
        let frame_bytes = synth_df17_frame([0xA1, 0xB2, 0xC3], 5, me);
        let frame = frame_from_bytes(frame_bytes);
        assert_eq!(frame.downlink_format(), DownlinkFormat::ExtendedSquitter);
        let msg = decode(&frame).expect("decode");
        let es = match msg {
            Message::ExtendedSquitter(es) => es,
            other => panic!("expected ExtendedSquitter, got {other:?}"),
        };
        assert_eq!(es.icao, Icao::from_bytes([0xA1, 0xB2, 0xC3]));
        assert_eq!(es.type_code, TypeCode::Identification);
        let id = match es.payload {
            SquitterPayload::Identification(id) => id,
            other => panic!("expected Identification, got {other:?}"),
        };
        assert_eq!(id.callsign.as_str(), "KLM1023");
    }

    // The all-call DF 11 case requires building a 7-byte frame via the
    // detector, which we already exercise in frame.rs tests. Here we hit
    // the dispatcher with a hand-crafted Frame built through the public
    // synthesis path used in CLI integration tests, to keep this module's
    // surface area small.

    #[test]
    fn bit_reader_reads_known_fields_msb_first() {
        // Bytes 0x8D, 0x40, 0x62, 0x10 in MSB-first bit positions:
        //   1  2  3  4  5  6  7  8  | 9 10 11 12 13 14 15 16 ...
        //   1  0  0  0  1  1  0  1  | 0  1  0  0  0  0  0  0 ...
        let r = BitReader::new(&[0x8D, 0x40, 0x62, 0x10]);
        assert_eq!(r.bits(1, 5), 0b10001); // 0x11 = 17
        assert_eq!(r.bits(6, 3), 0b101); // 5
        assert!(r.bit(1));
        assert!(!r.bit(2));
        assert!(r.bit(8));
        assert!(!r.bit(9));
        // Cross a byte boundary: bits 6..=13 should give the low 3 of
        // byte 0 concatenated with the high 5 of byte 1: 0b101 01000
        assert_eq!(r.bits(6, 8), 0b1010_1000);
        // Wide field: bits 9..=32 (24 bits) should be 0x40_6210.
        assert_eq!(r.bits(9, 24), 0x0040_6210);
    }

    #[test]
    fn bit_reader_handles_32_bit_fields() {
        let r = BitReader::new(&[0xDE, 0xAD, 0xBE, 0xEF, 0x00]);
        assert_eq!(r.bits(1, 32), 0xDEAD_BEEF);
        assert_eq!(r.bits(9, 32), 0xADBE_EF00);
    }

    #[test]
    fn ground_velocity_truncates_speed_fraction() {
        // Real ADS-B frame 8DA0BA4E9988760DC80C84D67445 from
        // corpus/live_1090mhz_2min.iq — TC=19 subtype 1 (subsonic ground
        // velocity) with EW = 117 kt and NS = 109 kt, so the true speed
        // is √(117² + 109²) = 159.906… kt. We truncate the fractional
        // kt (matching pyModeS and the conventional `(int)sqrt(...)`
        // result from C reference decoders); rounding would yield 160.
        // DO-260B doesn't specify and the difference is bounded by 1 kt.
        let me = [0x99, 0x88, 0x76, 0x0D, 0xC8, 0x0C, 0x84];
        let v = decode_velocity(me).expect("valid velocity ME");
        let VelocityKind::Ground { speed_kt, .. } = v.kind else {
            panic!("expected ground velocity, got {:?}", v.kind);
        };
        assert_eq!(speed_kt, 159);
    }
}

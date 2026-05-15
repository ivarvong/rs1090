//! Per-aircraft state tracker.
//!
//! Aggregates a stream of decoded [`Message`]s into a table keyed by ICAO
//! address, maintaining for each aircraft:
//!
//! - Latest callsign + category (from identification messages).
//! - Latest velocity (ground speed / airspeed, heading, vertical rate).
//! - Latest known position, plus a small CPR pairing buffer so that an
//!   even+odd pair within the freshness window resolves into a global
//!   decode.
//! - Per-aircraft counters: total messages, CRC clean/corrected/failed,
//!   marginal-confidence drops.
//! - `last_seen` so we can evict stale entries.
//!
//! The table is bounded by an LRU at a configurable capacity (default 4096
//! per DESIGN.md §10). On eviction the tracker emits a [`StateEvent::Lost`]
//! so downstream consumers can release any state they keep alongside.
//!
//! ## Address-XOR CRC validation
//!
//! For DF 4, 5, 20, 21 (and DF 0, 16) the CRC syndrome is XORed with the
//! aircraft's ICAO address. The frame layer can't validate those alone, so
//! it surfaces them with `CrcOutcome::Failed`. The tracker maintains the
//! active-ICAO set and, for each "failed" surveillance reply, tries the
//! XOR-with-known-ICAO trick to recover the address. If the recovered
//! syndrome matches a recently-seen aircraft (within the last 60 seconds
//! by default), we treat the frame as valid and update that aircraft's
//! counter. This is the mechanism that turns the 80% "failed" rate in our
//! live captures into useful surveillance data.
//!
//! Lint exemption: technical terms (ICAO, CRC, DF, CPR, ADS-B) aren't Rust
//! items.

#![allow(clippy::doc_markdown)]

use std::collections::HashMap;
use std::time::{Duration, Instant};

use arrayvec::ArrayString;

use crate::cpr::{self, CprPosition, LatLon};
use crate::crc::{self, CrcOutcome, LONG_FRAME_BYTES, SHORT_FRAME_BYTES};
use crate::frame::{DownlinkFormat, Frame};
use crate::message::{self, ExtendedSquitter, Icao, Message, SquitterPayload, Velocity};

// --- Public types -----------------------------------------------------------

/// Default LRU capacity per DESIGN.md §10.
pub const DEFAULT_CAPACITY: usize = 4096;

/// Maximum age of a CPR fragment before it's discarded.
pub const CPR_PAIR_WINDOW: Duration = Duration::from_secs(10);

/// Maximum age of an active ICAO when trying to recover address-XOR CRC.
pub const ACTIVE_ICAO_WINDOW: Duration = Duration::from_secs(60);

/// Maximum age before an aircraft is dropped on the next eviction pass.
pub const STALE_AFTER: Duration = Duration::from_secs(300);

/// Per-aircraft running state, exposed for read but not constructed by users.
#[derive(Debug, Clone)]
pub struct Aircraft {
    pub icao: Icao,
    pub callsign: Option<ArrayString<8>>,
    pub category: Option<(u8, u8)>,
    pub position: Option<TimedPosition>,
    pub velocity: Option<Velocity>,
    pub counters: Counters,
    pub last_seen: Instant,
    /// Pending CPR halves awaiting a complement to global-decode against.
    /// At most one of each parity; replaced if a fresher one arrives.
    cpr_even: Option<TimedCpr>,
    cpr_odd: Option<TimedCpr>,
}

#[derive(Debug, Clone, Copy)]
pub struct TimedPosition {
    pub at: Instant,
    pub pos: LatLon,
    /// How this position was derived.
    pub source: PositionSource,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PositionSource {
    /// Global CPR decode (even+odd pair).
    Global,
    /// Local CPR decode against a prior known position.
    Local,
}

#[derive(Debug, Clone, Copy)]
struct TimedCpr {
    at: Instant,
    cpr: CprPosition,
}

/// Counters for per-aircraft telemetry.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Counters {
    pub messages_total: u64,
    pub crc_clean: u64,
    pub crc_corrected: u64,
    /// CRC failed and we couldn't recover the address either.
    pub crc_failed: u64,
    /// Frame had failed CRC but we recovered its ICAO via address-XOR
    /// syndrome matching against the active set.
    pub crc_address_recovered: u64,
}

// --- Events emitted by the tracker -----------------------------------------

/// Side-effect events the tracker emits during `ingest` so callers can drive
/// downstream consumers (logging, network output) without polling the table.
#[derive(Debug, Clone)]
pub enum StateEvent {
    /// First time we've seen this aircraft in this tracker's lifetime.
    Acquired(Icao),
    /// Callsign learned or changed.
    Identification {
        icao: Icao,
        callsign: ArrayString<8>,
    },
    /// New position fix (global or local).
    Position { icao: Icao, pos: LatLon, source: PositionSource },
    /// New velocity vector.
    Velocity { icao: Icao, velocity: Velocity },
    /// Aircraft evicted from the table (LRU or staleness).
    Lost(Icao),
    /// A surveillance reply with failed CRC was matched against the active
    /// ICAO set. Useful for monitoring the recovery rate.
    AddressRecovered { icao: Icao, df: DownlinkFormat },
    /// A frame arrived but couldn't be associated with any known aircraft.
    /// Surfaced for diagnostics; not generally actionable.
    Orphan { df: DownlinkFormat },
}

// --- Tracker ----------------------------------------------------------------

/// Per-receiver state tracker. Owns the aircraft table; not `Sync`.
#[derive(Debug)]
pub struct StateTracker {
    by_icao: HashMap<Icao, Aircraft>,
    capacity: usize,
    /// Optional receiver location, used as a reference for local CPR decode
    /// when no recent global fix is available. If `None` we don't attempt
    /// local decode (it would silently produce wrong tiles).
    reference: Option<LatLon>,
}

impl StateTracker {
    #[must_use]
    pub fn new() -> Self {
        Self {
            by_icao: HashMap::with_capacity(DEFAULT_CAPACITY),
            capacity: DEFAULT_CAPACITY,
            reference: None,
        }
    }

    #[must_use]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            by_icao: HashMap::with_capacity(capacity),
            capacity,
            reference: None,
        }
    }

    /// Set the receiver's reference position, enabling local CPR decode.
    pub fn set_reference(&mut self, pos: LatLon) {
        self.reference = Some(pos);
    }

    /// Number of aircraft currently tracked.
    #[must_use]
    pub fn len(&self) -> usize {
        self.by_icao.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.by_icao.is_empty()
    }

    /// Iterate over all currently-tracked aircraft.
    pub fn aircraft(&self) -> impl Iterator<Item = &Aircraft> + '_ {
        self.by_icao.values()
    }

    /// Look up by ICAO.
    #[must_use]
    pub fn get(&self, icao: Icao) -> Option<&Aircraft> {
        self.by_icao.get(&icao)
    }

    /// Ingest one frame at the wall-clock instant it was received. Returns
    /// the events that resulted from this frame.
    ///
    /// `out` is reused across calls to avoid allocations in the hot path —
    /// callers typically keep a `Vec<StateEvent>` in scope and `clear()` it
    /// between iterations.
    pub fn ingest(&mut self, frame: &Frame, at: Instant, out: &mut Vec<StateEvent>) {
        // First, try to resolve the frame to an ICAO. For clean-CRC frames
        // (DF 11/17/18) the address is in the bytes. For failed-CRC frames
        // (DF 0/4/5/16/20/21) we try address-XOR recovery against the
        // active set.
        let resolved = self.resolve_icao(frame, at);

        let Some((icao, decoded)) = resolved else {
            out.push(StateEvent::Orphan {
                df: frame.downlink_format(),
            });
            return;
        };

        let is_new = !self.by_icao.contains_key(&icao);
        if is_new {
            self.evict_if_needed(at, out);
            self.by_icao.insert(
                icao,
                Aircraft {
                    icao,
                    callsign: None,
                    category: None,
                    position: None,
                    velocity: None,
                    counters: Counters::default(),
                    last_seen: at,
                    cpr_even: None,
                    cpr_odd: None,
                },
            );
            out.push(StateEvent::Acquired(icao));
        }

        let aircraft = self
            .by_icao
            .get_mut(&icao)
            .expect("just inserted or already present");
        aircraft.last_seen = at;
        aircraft.counters.messages_total += 1;
        match frame.crc_outcome() {
            CrcOutcome::Clean => aircraft.counters.crc_clean += 1,
            CrcOutcome::Corrected { .. } => aircraft.counters.crc_corrected += 1,
            CrcOutcome::Failed => {
                // Address-recovered frames count separately so the operator
                // can see how many "failed-but-recovered" we're handling.
                aircraft.counters.crc_address_recovered += 1;
                out.push(StateEvent::AddressRecovered {
                    icao,
                    df: frame.downlink_format(),
                });
            }
        }

        // Apply the decoded payload, if any. Frames whose CRC failed *and*
        // weren't address-recovered never reach here (they're orphans).
        if let Some(msg) = decoded {
            apply_message(aircraft, &msg, at, self.reference, out);
        }
    }

    /// Drop aircraft that haven't been heard in [`STALE_AFTER`].
    ///
    /// Normally called automatically by `ingest`; exposed so callers can run
    /// it on a wall-clock timer when no frames are arriving.
    pub fn evict_stale(&mut self, now: Instant, out: &mut Vec<StateEvent>) {
        let stale: Vec<Icao> = self
            .by_icao
            .iter()
            .filter(|(_, a)| now.saturating_duration_since(a.last_seen) > STALE_AFTER)
            .map(|(k, _)| *k)
            .collect();
        for icao in stale {
            self.by_icao.remove(&icao);
            out.push(StateEvent::Lost(icao));
        }
    }

    // --- internal helpers -----------------------------------------------

    /// Returns `(icao, decoded_message_if_available)`. The message is
    /// returned only when CRC was clean/corrected, or when we recovered
    /// the address from a failed-CRC frame *and* the DF carries decodable
    /// content. For DF 0/4/5/16/20/21 we only return the ICAO (to update
    /// the active-set/counters); the payload is not decoded into a
    /// `Message` because we don't yet implement the address-stripped CRC
    /// path's payload extraction.
    fn resolve_icao(
        &self,
        frame: &Frame,
        at: Instant,
    ) -> Option<(Icao, Option<Message>)> {
        match frame.crc_outcome() {
            CrcOutcome::Clean | CrcOutcome::Corrected { .. } => {
                let msg = message::decode(frame).ok()?;
                let icao = icao_of(&msg)?;
                Some((icao, Some(msg)))
            }
            CrcOutcome::Failed => {
                // For address-XOR DFs, the syndrome of the raw frame
                // equals `crc24(icao_bytes)` — *not* the ICAO itself,
                // because of how MSB-first non-reflected CRC handles the
                // XOR. We search the active aircraft set for one whose
                // ICAO would produce this syndrome. This is the dump1090
                // approach: dignified linear scan against a small set is
                // fast (the inner crc24 over 3 bytes is ~3 cycles), and
                // we never invent ghost aircraft from corrupted frames.
                let n = frame.len();
                if n != SHORT_FRAME_BYTES && n != LONG_FRAME_BYTES {
                    return None;
                }
                let syndrome = crc::crc24(frame.bytes());
                if syndrome == 0 {
                    return None;
                }
                for (icao, aircraft) in &self.by_icao {
                    if at.saturating_duration_since(aircraft.last_seen)
                        >= ACTIVE_ICAO_WINDOW
                    {
                        continue;
                    }
                    if crc::crc24(&icao.to_bytes()) == syndrome {
                        return Some((*icao, None));
                    }
                }
                None
            }
        }
    }

    fn evict_if_needed(&mut self, now: Instant, out: &mut Vec<StateEvent>) {
        if self.by_icao.len() < self.capacity {
            return;
        }
        // Drop the single oldest entry.
        let oldest = self
            .by_icao
            .iter()
            .min_by_key(|(_, a)| a.last_seen)
            .map(|(k, _)| *k);
        if let Some(icao) = oldest {
            self.by_icao.remove(&icao);
            out.push(StateEvent::Lost(icao));
        }
        // Also opportunistically evict the obviously stale ones.
        self.evict_stale(now, out);
    }
}

impl Default for StateTracker {
    fn default() -> Self {
        Self::new()
    }
}

// --- Per-message application -----------------------------------------------

fn icao_of(msg: &Message) -> Option<Icao> {
    match msg {
        Message::ExtendedSquitter(es) => Some(es.icao),
        Message::AllCallReply { icao } => Some(*icao),
        Message::SurveillanceReply { .. } | Message::Other { .. } => None,
    }
}

fn apply_message(
    aircraft: &mut Aircraft,
    msg: &Message,
    at: Instant,
    reference: Option<LatLon>,
    out: &mut Vec<StateEvent>,
) {
    let Message::ExtendedSquitter(es) = msg else {
        return;
    };
    apply_extended_squitter(aircraft, es, at, reference, out);
}

fn apply_extended_squitter(
    aircraft: &mut Aircraft,
    es: &ExtendedSquitter,
    at: Instant,
    reference: Option<LatLon>,
    out: &mut Vec<StateEvent>,
) {
    match &es.payload {
        SquitterPayload::Identification(id) => {
            let changed = aircraft.callsign.as_ref() != Some(&id.callsign);
            aircraft.callsign = Some(id.callsign);
            aircraft.category = Some((id.category_set, id.category));
            if changed {
                out.push(StateEvent::Identification {
                    icao: es.icao,
                    callsign: id.callsign,
                });
            }
        }
        SquitterPayload::AirbornePosition(p) => {
            // Always stash the freshest CPR fragment; replace any previous
            // fragment of the same parity.
            let fragment = TimedCpr { at, cpr: p.cpr };
            if p.cpr.odd {
                aircraft.cpr_odd = Some(fragment);
            } else {
                aircraft.cpr_even = Some(fragment);
            }
            try_resolve_position(aircraft, at, reference, out);
        }
        SquitterPayload::Velocity(v) => {
            aircraft.velocity = Some(*v);
            out.push(StateEvent::Velocity {
                icao: es.icao,
                velocity: *v,
            });
        }
        SquitterPayload::Raw(_) => {
            // TC handled by dispatch but not yet typed (e.g. status). No-op.
        }
    }
}

fn try_resolve_position(
    aircraft: &mut Aircraft,
    at: Instant,
    reference: Option<LatLon>,
    out: &mut Vec<StateEvent>,
) {
    // Prefer a global decode from a fresh even/odd pair.
    if let (Some(e), Some(o)) = (aircraft.cpr_even, aircraft.cpr_odd) {
        let gap = e.at.max(o.at).saturating_duration_since(e.at.min(o.at));
        if gap <= CPR_PAIR_WINDOW {
            let most_recent_odd = o.at >= e.at;
            match cpr::global_decode(e.cpr, o.cpr, most_recent_odd) {
                Ok(pos) => {
                    aircraft.position = Some(TimedPosition {
                        at,
                        pos,
                        source: PositionSource::Global,
                    });
                    out.push(StateEvent::Position {
                        icao: aircraft.icao,
                        pos,
                        source: PositionSource::Global,
                    });
                    return;
                }
                Err(cpr::CprError::LatitudeZoneMismatch) => {
                    // Drop the older half; keep the fresher one and wait
                    // for a fresh complement.
                    if o.at >= e.at {
                        aircraft.cpr_even = None;
                    } else {
                        aircraft.cpr_odd = None;
                    }
                }
                Err(_) => {
                    // Parity/range error: drop both halves and start over.
                    aircraft.cpr_even = None;
                    aircraft.cpr_odd = None;
                }
            }
        } else {
            // Stale pairing: discard the older half.
            if o.at >= e.at {
                aircraft.cpr_even = None;
            } else {
                aircraft.cpr_odd = None;
            }
        }
    }

    // No global fix possible; try a local decode if we have a reference
    // and a fresh fragment of either parity.
    let Some(ref_pos) = reference else { return };
    // Use the freshest CPR fragment available, regardless of parity.
    let fragment = match (aircraft.cpr_even, aircraft.cpr_odd) {
        (Some(e), Some(o)) => Some(if e.at >= o.at { e } else { o }),
        (Some(e), None) => Some(e),
        (None, Some(o)) => Some(o),
        (None, None) => None,
    };
    let Some(frag) = fragment else { return };
    let pos = cpr::local_decode(frag.cpr, ref_pos);
    aircraft.position = Some(TimedPosition {
        at,
        pos,
        source: PositionSource::Local,
    });
    out.push(StateEvent::Position {
        icao: aircraft.icao,
        pos,
        source: PositionSource::Local,
    });
}

// --- Tests ------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc;
    use crate::demod::{
        synth_bits_as_magnitude, PREAMBLE_HIGH_IDX, PREAMBLE_SAMPLES, SAMPLES_PER_BIT,
    };
    use crate::frame::FrameDetector;
    use crate::Iq;

    // --- Helpers for synthesizing frames in test ---

    /// Build a 14-byte DF 17 frame with valid CRC carrying the given ME.
    fn build_df17(icao: [u8; 3], me: [u8; 7]) -> [u8; 14] {
        let mut data = [0u8; 11];
        data[0] = (17u8 << 3) | 5;
        data[1] = icao[0];
        data[2] = icao[1];
        data[3] = icao[2];
        data[4..11].copy_from_slice(&me);
        let c = crc::crc24(&data);
        let mut f = [0u8; 14];
        f[..11].copy_from_slice(&data);
        f[11] = ((c >> 16) & 0xFF) as u8;
        f[12] = ((c >> 8) & 0xFF) as u8;
        f[13] = (c & 0xFF) as u8;
        f
    }

    /// Run frame bytes through the real detector so the resulting `Frame`
    /// has its private fields populated correctly.
    fn frame_from_bytes(frame_bytes: &[u8]) -> Frame {
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
        assert_eq!(out.len(), 1, "synth frame must produce exactly one frame");
        out[0]
    }

    /// Pack 8 six-bit codes for an identification ME.
    fn identification_me(codes: [u8; 8]) -> [u8; 7] {
        let mut me = [0u8; 7];
        me[0] = 4u8 << 3; // TC=4, category=0
        let mut bits: u64 = 0;
        for c in codes {
            bits = (bits << 6) | u64::from(c & 0x3F);
        }
        for i in 0..6 {
            me[1 + i] = ((bits >> (8 * (5 - i))) & 0xFF) as u8;
        }
        me
    }

    /// Build an airborne-position ME for TC=11 with given CPR fields and
    /// altitude unavailable.
    fn airborne_position_me(odd: bool, lat_cpr: u32, lon_cpr: u32) -> [u8; 7] {
        let tc: u64 = 11;
        let me_bits: u64 = (tc << 51)
            | (u64::from(odd) << 34)
            | (u64::from(lat_cpr) << 17)
            | u64::from(lon_cpr);
        let mut me = [0u8; 7];
        for (i, byte) in me.iter_mut().enumerate() {
            *byte = ((me_bits >> (8 * (6 - i))) & 0xFF) as u8;
        }
        me
    }

    // --- Tests ---

    #[test]
    fn acquires_new_aircraft_on_first_frame() {
        let me = identification_me([20, 5, 19, 20, 49, 50, 51, 52]); // TEST1234
        let bytes = build_df17([0xAB, 0xCD, 0xEF], me);
        let frame = frame_from_bytes(&bytes);

        let mut tracker = StateTracker::new();
        let mut events = Vec::new();
        let now = Instant::now();
        tracker.ingest(&frame, now, &mut events);

        let icao = Icao::from_bytes([0xAB, 0xCD, 0xEF]);
        assert!(events
            .iter()
            .any(|e| matches!(e, StateEvent::Acquired(i) if *i == icao)));
        assert!(events
            .iter()
            .any(|e| matches!(e, StateEvent::Identification { icao: i, callsign }
                              if *i == icao && callsign.as_str() == "TEST1234")));
        let ac = tracker.get(icao).expect("aircraft tracked");
        assert_eq!(ac.callsign.unwrap().as_str(), "TEST1234");
        assert_eq!(ac.counters.messages_total, 1);
        assert_eq!(ac.counters.crc_clean, 1);
    }

    #[test]
    fn second_frame_from_same_aircraft_does_not_re_acquire() {
        let me = identification_me([20, 5, 19, 20, 49, 50, 51, 52]);
        let bytes = build_df17([0xAB, 0xCD, 0xEF], me);
        let frame = frame_from_bytes(&bytes);

        let mut tracker = StateTracker::new();
        let mut events = Vec::new();
        let t0 = Instant::now();
        tracker.ingest(&frame, t0, &mut events);
        events.clear();
        tracker.ingest(&frame, t0 + Duration::from_millis(100), &mut events);
        assert!(!events
            .iter()
            .any(|e| matches!(e, StateEvent::Acquired(_))));
        // Identification didn't change ⇒ no second Identification event.
        assert!(!events
            .iter()
            .any(|e| matches!(e, StateEvent::Identification { .. })));
        let ac = tracker.get(Icao::from_bytes([0xAB, 0xCD, 0xEF])).unwrap();
        assert_eq!(ac.counters.messages_total, 2);
    }

    #[test]
    fn even_then_odd_within_window_produces_global_fix() {
        // Use the canonical CPR worked example: (lat=52.2572, lon=3.91937).
        let icao_bytes = [0x4B, 0x9C, 0xA2];
        let icao = Icao::from_bytes(icao_bytes);
        let even = airborne_position_me(false, 0b1_0110_1011_0100_1000, 0b0_1100_1000_1010_1100);
        let odd = airborne_position_me(true, 0b1_0010_0001_1010_1110, 0b0_1100_0100_0001_0010);
        let f_even = frame_from_bytes(&build_df17(icao_bytes, even));
        let f_odd = frame_from_bytes(&build_df17(icao_bytes, odd));

        let mut tracker = StateTracker::new();
        let mut events = Vec::new();
        let t0 = Instant::now();
        // Odd arrives first, then even 500ms later. The pair resolves
        // using the even frame's longitude (because even is most recent).
        tracker.ingest(&f_odd, t0, &mut events);
        events.clear();
        tracker.ingest(&f_even, t0 + Duration::from_millis(500), &mut events);

        let pos_evt = events
            .iter()
            .find_map(|e| match e {
                StateEvent::Position { icao: i, pos, source } if *i == icao => {
                    Some((*pos, *source))
                }
                _ => None,
            })
            .expect("position event emitted");
        assert_eq!(pos_evt.1, PositionSource::Global);
        assert!(
            (pos_evt.0.lat_deg - 52.2572).abs() < 1e-3,
            "lat {} not near 52.2572", pos_evt.0.lat_deg
        );
        assert!(
            (pos_evt.0.lon_deg - 3.919_37).abs() < 1e-3,
            "lon {} not near 3.91937", pos_evt.0.lon_deg
        );
    }

    #[test]
    fn stale_cpr_pair_does_not_resolve() {
        let icao_bytes = [0x4B, 0x9C, 0xA2];
        let even = airborne_position_me(false, 0b1_0110_1011_0100_1000, 0b0_1100_1000_1010_1100);
        let odd = airborne_position_me(true, 0b1_0010_0001_1010_1110, 0b0_1100_0100_0001_0010);
        let f_even = frame_from_bytes(&build_df17(icao_bytes, even));
        let f_odd = frame_from_bytes(&build_df17(icao_bytes, odd));

        let mut tracker = StateTracker::new();
        let mut events = Vec::new();
        let t0 = Instant::now();
        tracker.ingest(&f_even, t0, &mut events);
        events.clear();
        // 11 seconds later — outside the 10-second window.
        tracker.ingest(&f_odd, t0 + Duration::from_secs(11), &mut events);
        assert!(!events
            .iter()
            .any(|e| matches!(e, StateEvent::Position { .. })));
    }

    #[test]
    fn local_decode_used_when_global_unavailable() {
        let icao_bytes = [0xAB, 0xCD, 0xEF];
        let icao = Icao::from_bytes(icao_bytes);
        // Encode a position with reference helper from the cpr tests.
        // Use a low-latitude position so quantization is easy to reason about.
        let bytes = build_df17(
            icao_bytes,
            airborne_position_me(false, 0b1_0110_1011_0100_1000, 0b0_1100_1000_1010_1100),
        );
        let frame = frame_from_bytes(&bytes);

        let mut tracker = StateTracker::new();
        // Reference within ~1° of the true position so local decode works.
        tracker.set_reference(LatLon { lat_deg: 52.0, lon_deg: 4.0 });
        let mut events = Vec::new();
        tracker.ingest(&frame, Instant::now(), &mut events);

        let (pos, source) = events
            .iter()
            .find_map(|e| match e {
                StateEvent::Position { icao: i, pos, source } if *i == icao => {
                    Some((*pos, *source))
                }
                _ => None,
            })
            .expect("local position event emitted");
        assert_eq!(source, PositionSource::Local);
        // Should land near 52.26°N, 3.92°E.
        assert!((pos.lat_deg - 52.26).abs() < 0.2, "lat {}", pos.lat_deg);
        assert!((pos.lon_deg - 3.92).abs() < 0.2, "lon {}", pos.lon_deg);
    }

    #[test]
    fn lru_evicts_oldest_when_full() {
        let mut tracker = StateTracker::with_capacity(2);
        let mut events = Vec::new();
        let t0 = Instant::now();

        let me = identification_me([20, 5, 19, 20, 49, 50, 51, 52]);
        let f1 = frame_from_bytes(&build_df17([0x00, 0x00, 0x01], me));
        let f2 = frame_from_bytes(&build_df17([0x00, 0x00, 0x02], me));
        let f3 = frame_from_bytes(&build_df17([0x00, 0x00, 0x03], me));
        tracker.ingest(&f1, t0, &mut events);
        tracker.ingest(&f2, t0 + Duration::from_millis(100), &mut events);
        events.clear();
        tracker.ingest(&f3, t0 + Duration::from_millis(200), &mut events);

        // f1 (oldest) should have been evicted.
        assert!(events
            .iter()
            .any(|e| matches!(e, StateEvent::Lost(i) if *i == Icao::from_bytes([0,0,1]))));
        assert!(tracker.get(Icao::from_bytes([0, 0, 1])).is_none());
        assert!(tracker.get(Icao::from_bytes([0, 0, 2])).is_some());
        assert!(tracker.get(Icao::from_bytes([0, 0, 3])).is_some());
    }

    #[test]
    fn evict_stale_drops_old_entries() {
        let mut tracker = StateTracker::new();
        let mut events = Vec::new();
        let t0 = Instant::now();
        let me = identification_me([20, 5, 19, 20, 49, 50, 51, 52]);
        let f = frame_from_bytes(&build_df17([0xAB, 0xCD, 0xEF], me));
        tracker.ingest(&f, t0, &mut events);
        events.clear();

        tracker.evict_stale(t0 + STALE_AFTER + Duration::from_secs(1), &mut events);
        assert!(events
            .iter()
            .any(|e| matches!(e, StateEvent::Lost(_))));
        assert!(tracker.is_empty());
    }

    // --- Address-XOR CRC recovery (M3's headline feature on real captures) ---

    /// Build a DF 4 (short) altitude-reply frame whose CRC is XORed with
    /// the given ICAO, as the spec prescribes for surveillance replies.
    ///
    /// Key property of this MSB-first non-reflected CRC: the syndrome of
    /// the resulting frame equals `crc24(icao_bytes)`, NOT the ICAO
    /// itself. The state tracker exploits that: it computes the syndrome
    /// of each known ICAO and matches.
    fn build_df4_address_xor(icao: [u8; 3], payload: [u8; 4]) -> [u8; 7] {
        let mut data = [0u8; 4];
        data[0] = 4u8 << 3 | (payload[0] & 0x07);
        data[1..4].copy_from_slice(&payload[1..4]);
        let crc_val = crc::crc24(&data);
        let icao_val =
            (u32::from(icao[0]) << 16) | (u32::from(icao[1]) << 8) | u32::from(icao[2]);
        let xored = crc_val ^ icao_val;
        let mut f = [0u8; 7];
        f[..4].copy_from_slice(&data);
        f[4] = ((xored >> 16) & 0xFF) as u8;
        f[5] = ((xored >> 8) & 0xFF) as u8;
        f[6] = (xored & 0xFF) as u8;
        // Verify: the syndrome equals `crc24(icao_bytes)`.
        debug_assert_eq!(crc::crc24(&f), crc::crc24(&icao));
        f
    }

    #[test]
    fn address_xor_crc_recovers_known_icao() {
        let mut tracker = StateTracker::new();
        let mut events = Vec::new();
        let t0 = Instant::now();

        // First, register the aircraft via a clean DF 17.
        let me = identification_me([20, 5, 19, 20, 49, 50, 51, 52]);
        let clean = frame_from_bytes(&build_df17([0xA2, 0x4A, 0xA8], me));
        tracker.ingest(&clean, t0, &mut events);
        events.clear();

        // Now feed a DF 4 reply whose CRC is XORed with that ICAO. The
        // frame layer will report CRC::Failed; the tracker should
        // recover the address and update counters.
        let df4 = build_df4_address_xor([0xA2, 0x4A, 0xA8], [0, 0, 0, 0]);
        let frame = frame_from_bytes(&df4);
        assert_eq!(frame.crc_outcome(), CrcOutcome::Failed);

        tracker.ingest(&frame, t0 + Duration::from_secs(1), &mut events);
        let icao = Icao::from_bytes([0xA2, 0x4A, 0xA8]);
        assert!(events.iter().any(|e| matches!(
            e,
            StateEvent::AddressRecovered { icao: i, .. } if *i == icao
        )), "expected AddressRecovered event, got {events:?}");
        let ac = tracker.get(icao).expect("still tracked");
        assert_eq!(ac.counters.crc_address_recovered, 1);
        assert_eq!(ac.counters.messages_total, 2);
    }

    #[test]
    fn address_xor_crc_rejects_unknown_icao() {
        // No prior aircraft registered ⇒ syndrome doesn't match any known
        // address ⇒ the frame is orphaned, not silently invented.
        let df4 = build_df4_address_xor([0xDE, 0xAD, 0xBE], [0, 0, 0, 0]);
        let frame = frame_from_bytes(&df4);
        let mut tracker = StateTracker::new();
        let mut events = Vec::new();
        tracker.ingest(&frame, Instant::now(), &mut events);
        assert!(tracker.is_empty(), "no aircraft should be invented");
        assert!(events.iter().any(|e| matches!(e, StateEvent::Orphan { .. })));
    }

    #[test]
    fn address_xor_crc_rejects_stale_icao() {
        // Register, then wait past the active window before sending the
        // address-XOR reply. The reply should be orphaned, not matched.
        let mut tracker = StateTracker::new();
        let mut events = Vec::new();
        let t0 = Instant::now();
        let me = identification_me([20, 5, 19, 20, 49, 50, 51, 52]);
        let clean = frame_from_bytes(&build_df17([0xA2, 0x4A, 0xA8], me));
        tracker.ingest(&clean, t0, &mut events);
        events.clear();

        let df4 = build_df4_address_xor([0xA2, 0x4A, 0xA8], [0, 0, 0, 0]);
        let frame = frame_from_bytes(&df4);
        let too_late = t0 + ACTIVE_ICAO_WINDOW + Duration::from_secs(1);
        tracker.ingest(&frame, too_late, &mut events);
        assert!(events.iter().any(|e| matches!(e, StateEvent::Orphan { .. })));
    }
}

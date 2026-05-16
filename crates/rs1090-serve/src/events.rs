//! JSON event types emitted over SSE.
//!
//! These mirror `rs1090::state::StateEvent` but live here so the library
//! itself stays serde-free. Per DESIGN.md §12.2:
//!
//! - **SI units only**: meters, m/s, degrees, RFC 3339 UTC timestamps.
//! - **Versioned schema** (`"v": 1`).
//! - **Stable field names**; adding fields is non-breaking, renaming is.
//! - **Monotonic `id`** (`u64` counter; clients use `Last-Event-ID` to
//!   resume).

use serde::Serialize;

use rs1090::frame::DownlinkFormat;
use rs1090::message::{Icao, Velocity, VelocityKind};
use rs1090::state::{PositionSource, StateEvent};

/// Wire-format event envelope. Each variant becomes one SSE event with
/// `event: <tag>` and `data: <json>`.
#[derive(Debug, Clone, Serialize)]
#[serde(tag = "event", rename_all = "snake_case")]
pub enum Event {
    Acquired(AircraftAcquired),
    Identification(AircraftIdentification),
    Position(AircraftPosition),
    Velocity(AircraftVelocity),
    Lost(AircraftLost),
    AddressRecovered(AddressRecovered),
}

/// A decoded SSE event paired with the monotonic id assigned at broadcast
/// time. The `id` participates in `Last-Event-ID` reconnect.
#[derive(Debug, Clone)]
pub struct EventEnvelope {
    pub id: u64,
    pub event: Event,
}

impl Event {
    /// SSE event-tag (matches the `event:` line in the wire format).
    pub fn tag(&self) -> &'static str {
        match self {
            Event::Acquired(_) => "acquired",
            Event::Identification(_) => "identification",
            Event::Position(_) => "position",
            Event::Velocity(_) => "velocity",
            Event::Lost(_) => "lost",
            Event::AddressRecovered(_) => "address_recovered",
        }
    }

    /// ICAO address of the aircraft this event is about, if any.
    ///
    /// All current variants are aircraft-scoped; the `Option` return is
    /// kept so future non-aircraft events (e.g. receiver-health
    /// heartbeats) can use the same dispatch.
    #[allow(clippy::unnecessary_wraps)]
    pub fn icao(&self) -> Option<Icao> {
        match self {
            Event::Acquired(e) => Some(e.icao),
            Event::Identification(e) => Some(e.icao),
            Event::Position(e) => Some(e.icao),
            Event::Velocity(e) => Some(e.icao),
            Event::Lost(e) => Some(e.icao),
            Event::AddressRecovered(e) => Some(e.icao),
        }
    }
}

// --- Payload structs --------------------------------------------------------

#[derive(Debug, Clone, Serialize)]
#[non_exhaustive]
pub struct AircraftAcquired {
    pub v: u8,
    pub t: String,
    #[serde(serialize_with = "ser_icao")]
    pub icao: Icao,
}

#[derive(Debug, Clone, Serialize)]
#[non_exhaustive]
pub struct AircraftIdentification {
    pub v: u8,
    pub t: String,
    #[serde(serialize_with = "ser_icao")]
    pub icao: Icao,
    pub callsign: String,
}

#[derive(Debug, Clone, Serialize)]
#[non_exhaustive]
pub struct AircraftPosition {
    pub v: u8,
    pub t: String,
    #[serde(serialize_with = "ser_icao")]
    pub icao: Icao,
    pub lat: f64,
    pub lon: f64,
    /// Altitude in feet, regardless of encoding. Absent when the
    /// aircraft did not report an altitude.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alt_ft: Option<i32>,
    /// `"baro"` or `"gnss"`. Absent when `alt_ft` is absent.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alt_source: Option<&'static str>,
    #[serde(serialize_with = "ser_position_source")]
    pub source: PositionSource,
}

#[derive(Debug, Clone, Serialize)]
#[non_exhaustive]
pub struct AircraftVelocity {
    pub v: u8,
    pub t: String,
    #[serde(serialize_with = "ser_icao")]
    pub icao: Icao,
    /// Ground speed in m/s, when the message subtype reports ground speed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gs_mps: Option<f64>,
    /// Track (true) in degrees clockwise from north, when ground subtype.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub track_deg: Option<f64>,
    /// Airspeed in m/s, when subtype reports airspeed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ias_mps: Option<f64>,
    /// Heading in degrees, when airspeed subtype has a valid heading.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub heading_deg: Option<f64>,
    /// `true` if the heading is magnetic, `false` if true.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub heading_magnetic: Option<bool>,
    /// Vertical rate in m/s, positive = climb.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vr_mps: Option<f64>,
    /// Source of the vertical rate: `"baro"` or `"gnss"`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vr_source: Option<&'static str>,
}

#[derive(Debug, Clone, Serialize)]
#[non_exhaustive]
pub struct AircraftLost {
    pub v: u8,
    pub t: String,
    #[serde(serialize_with = "ser_icao")]
    pub icao: Icao,
}

#[derive(Debug, Clone, Serialize)]
#[non_exhaustive]
pub struct AddressRecovered {
    pub v: u8,
    pub t: String,
    #[serde(serialize_with = "ser_icao")]
    pub icao: Icao,
    /// Downlink format the recovered frame had (0/4/5/16/20/21).
    pub df: u8,
}

// --- Serde helpers ----------------------------------------------------------

#[allow(clippy::trivially_copy_pass_by_ref)]
fn ser_icao<S: serde::Serializer>(icao: &Icao, s: S) -> Result<S::Ok, S::Error> {
    s.serialize_str(icao.to_hex().as_str())
}

#[allow(clippy::trivially_copy_pass_by_ref)]
fn ser_position_source<S: serde::Serializer>(
    p: &PositionSource,
    s: S,
) -> Result<S::Ok, S::Error> {
    s.serialize_str(p.wire_tag())
}

// --- Conversion helpers -----------------------------------------------------

const KT_TO_MPS: f64 = 0.514_444;
const FPM_TO_MPS: f64 = 0.005_08;

/// Convert a state-tracker event into a wire-format JSON event.
pub fn from_state_event(ev: &StateEvent, now_iso: &str) -> Option<Event> {
    let t = || now_iso.to_string();
    Some(match ev {
        StateEvent::Acquired(icao) => Event::Acquired(AircraftAcquired {
            v: 1,
            t: t(),
            icao: *icao,
        }),
        StateEvent::Identification { icao, callsign } => {
            Event::Identification(AircraftIdentification {
                v: 1,
                t: t(),
                icao: *icao,
                callsign: callsign.to_string(),
            })
        }
        StateEvent::Position { icao, pos, altitude, source } => {
            Event::Position(AircraftPosition {
                v: 1,
                t: t(),
                icao: *icao,
                lat: pos.lat_deg,
                lon: pos.lon_deg,
                alt_ft: altitude.feet(),
                alt_source: altitude.source_tag(),
                source: *source,
            })
        }
        StateEvent::Velocity { icao, velocity } => {
            Event::Velocity(velocity_to_wire(*icao, *velocity, &t()))
        }
        StateEvent::Lost(icao) => Event::Lost(AircraftLost {
            v: 1,
            t: t(),
            icao: *icao,
        }),
        StateEvent::AddressRecovered { icao, df } => Event::AddressRecovered(AddressRecovered {
            v: 1,
            t: t(),
            icao: *icao,
            df: downlink_format_raw(*df),
        }),
        // Orphan frames don't get broadcast — they're diagnostic noise that
        // would otherwise drown out useful events on busy receivers.
        StateEvent::Orphan { .. } => return None,
    })
}

fn velocity_to_wire(icao: Icao, v: Velocity, t: &str) -> AircraftVelocity {
    let (gs_mps, track_deg, ias_mps, heading_deg, heading_magnetic) = match v.kind {
        VelocityKind::Ground {
            speed_kt,
            heading_deg,
        } => (
            Some(f64::from(speed_kt) * KT_TO_MPS),
            Some(f64::from(heading_deg)),
            None,
            None,
            None,
        ),
        VelocityKind::Airspeed {
            speed_kt,
            heading_deg,
            magnetic,
        } => (
            None,
            None,
            Some(f64::from(speed_kt) * KT_TO_MPS),
            heading_deg.map(f64::from),
            Some(magnetic),
        ),
    };
    let vr_mps = v.vertical_rate_fpm.map(|fpm| f64::from(fpm) * FPM_TO_MPS);
    let vr_source = v.vertical_rate_fpm.map(|_| match v.vertical_rate_source {
        rs1090::message::VerticalRateSource::Baro => "baro",
        rs1090::message::VerticalRateSource::Gnss => "gnss",
    });
    AircraftVelocity {
        v: 1,
        t: t.to_string(),
        icao,
        gs_mps,
        track_deg,
        ias_mps,
        heading_deg,
        heading_magnetic,
        vr_mps,
        vr_source,
    }
}

fn downlink_format_raw(df: DownlinkFormat) -> u8 {
    df.raw_value()
}

// --- Aircraft snapshot for /aircraft endpoint -------------------------------

/// JSON-friendly summary of a currently-tracked aircraft. Constructed from
/// `rs1090::state::Aircraft` at the moment of HTTP request.
#[derive(Debug, Clone, Serialize)]
#[non_exhaustive]
pub struct AircraftSnapshot {
    #[serde(serialize_with = "ser_icao")]
    pub icao: Icao,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub callsign: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub category: Option<[u8; 2]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub position: Option<SnapshotPosition>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub velocity: Option<SnapshotVelocity>,
    pub counters: SnapshotCounters,
}

#[derive(Debug, Clone, Serialize)]
#[non_exhaustive]
pub struct SnapshotPosition {
    pub lat: f64,
    pub lon: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alt_ft: Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alt_source: Option<&'static str>,
    #[serde(serialize_with = "ser_position_source")]
    pub source: PositionSource,
}



#[derive(Debug, Clone, Serialize)]
#[non_exhaustive]
pub struct SnapshotVelocity {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gs_mps: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub track_deg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ias_mps: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub heading_deg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub heading_magnetic: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vr_mps: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vr_source: Option<&'static str>,
}

impl SnapshotVelocity {
    pub fn from_velocity(v: Velocity) -> Self {
        let wire = velocity_to_wire(Icao::ZERO, v, "");
        Self {
            gs_mps: wire.gs_mps,
            track_deg: wire.track_deg,
            ias_mps: wire.ias_mps,
            heading_deg: wire.heading_deg,
            heading_magnetic: wire.heading_magnetic,
            vr_mps: wire.vr_mps,
            vr_source: wire.vr_source,
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize)]
#[non_exhaustive]
pub struct SnapshotCounters {
    pub messages_total: u64,
    pub crc_clean: u64,
    pub crc_corrected: u64,
    pub crc_address_recovered: u64,
}

// --- Tests ------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn position_event_serializes_to_expected_schema() {
        let ev = Event::Position(AircraftPosition {
            v: 1,
            t: "2026-05-15T18:42:01.123Z".into(),
            icao: Icao::from_bytes([0xA1, 0xB2, 0xC3]),
            lat: 40.6413,
            lon: -73.7781,
            alt_ft: Some(25_000),
            alt_source: Some("baro"),
            source: PositionSource::Global,
        });
        let j: serde_json::Value = serde_json::to_value(&ev).unwrap();
        assert_eq!(j["event"], "position");
        assert_eq!(j["v"], 1);
        assert_eq!(j["t"], "2026-05-15T18:42:01.123Z");
        assert_eq!(j["icao"], "A1B2C3");
        assert_eq!(j["lat"], 40.6413);
        assert_eq!(j["lon"], -73.7781);
        assert_eq!(j["alt_ft"], 25_000);
        assert_eq!(j["alt_source"], "baro");
        assert_eq!(j["source"], "global");
    }

    #[test]
    fn velocity_ground_omits_airspeed_fields() {
        let v = Velocity {
            kind: VelocityKind::Ground {
                speed_kt: 360,
                heading_deg: 168.3,
            },
            vertical_rate_fpm: Some(1728),
            vertical_rate_source: rs1090::message::VerticalRateSource::Baro,
        };
        let wire = velocity_to_wire(Icao::from_bytes([0, 0, 1]), v, "t");
        let j = serde_json::to_value(&wire).unwrap();
        assert!(j.get("gs_mps").is_some());
        assert!(j.get("track_deg").is_some());
        assert!(j.get("ias_mps").is_none());
        assert!(j.get("heading_deg").is_none());
        assert!(j.get("vr_mps").is_some());
        // 1728 fpm = ~8.78 m/s
        let vr = j["vr_mps"].as_f64().unwrap();
        assert!((vr - 8.78).abs() < 0.05, "vr={vr}");
    }

    #[test]
    fn velocity_airspeed_omits_ground_fields() {
        let v = Velocity {
            kind: VelocityKind::Airspeed {
                speed_kt: 250,
                heading_deg: Some(45.0),
                magnetic: true,
            },
            vertical_rate_fpm: None,
            vertical_rate_source: rs1090::message::VerticalRateSource::Baro,
        };
        let wire = velocity_to_wire(Icao::from_bytes([0, 0, 1]), v, "t");
        let j = serde_json::to_value(&wire).unwrap();
        assert!(j.get("ias_mps").is_some());
        assert!(j.get("heading_deg").is_some());
        assert_eq!(j["heading_magnetic"], true);
        assert!(j.get("gs_mps").is_none());
        assert!(j.get("track_deg").is_none());
        assert!(j.get("vr_mps").is_none());
    }

    #[test]
    fn orphan_events_are_not_broadcast() {
        let ev = StateEvent::Orphan {
            df: DownlinkFormat::AltitudeReply,
        };
        assert!(from_state_event(&ev, "t").is_none());
    }

    #[test]
    fn knot_to_mps_conversion_pinned() {
        let v = Velocity {
            kind: VelocityKind::Ground {
                speed_kt: 360,
                heading_deg: 0.0,
            },
            vertical_rate_fpm: None,
            vertical_rate_source: rs1090::message::VerticalRateSource::Baro,
        };
        let wire = velocity_to_wire(Icao::ZERO, v, "t");
        // 360 kt = 185.2 m/s.
        assert!(
            (wire.gs_mps.unwrap() - 185.2).abs() < 0.1,
            "gs_mps = {}",
            wire.gs_mps.unwrap()
        );
    }
}

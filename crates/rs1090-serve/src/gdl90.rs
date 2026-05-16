//! Garmin GDL90 UDP broadcast.
//!
//! GDL90 is the wire format every iPad / iPhone EFB app speaks:
//! ForeFlight, Garmin Pilot, FlyQ, FltPlan Go. Broadcast traffic
//! reports to UDP 4000 on the LAN and aircraft show up on those apps'
//! moving maps with no custom mobile code — the Stratux model.
//!
//! Spec reference: FAA "GDL 90 Data Interface Specification, Rev A"
//! (`GDL90_Public_ICD_RevA.PDF`). We implement the subset every EFB
//! cares about:
//!
//! - Message **0 Heartbeat**: sent once per second. Without this no
//!   client treats us as alive.
//! - Message **20 Traffic Report**: one per tracked aircraft per
//!   second. The payload that actually populates the map.
//!
//! Framing per spec §2.2:
//!
//! ```text
//!   0x7E | message-ID + payload | CRC-16-CCITT LSB-first | 0x7E
//! ```
//!
//! After CRC append and before the trailing flag, the byte stream is
//! escape-stuffed: `0x7E` → `0x7D 0x5E`, `0x7D` → `0x7D 0x5D`.
//!
//! Lint exemption: technical terms (GDL90, CRC, NIC, NACp, ICAO,
//! UTC) aren't Rust items. Lossy casts are deliberate at the
//! protocol-byte-packing layer.
#![allow(
    clippy::doc_markdown,
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss
)]

use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4};
use std::sync::Arc;

use tokio::net::UdpSocket;

use rs1090::state::PositionSource;

use crate::broadcaster::AppState;
use crate::events::AircraftSnapshot;

/// Default UDP target. Broadcast to the LAN; ForeFlight auto-discovers
/// any GDL90 source on this port.
pub const DEFAULT_TARGET: SocketAddr = SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::BROADCAST, 4000));

/// Run the GDL90 broadcaster until cancelled. One heartbeat per second
/// plus one traffic report per tracked aircraft per second, all
/// UDP-sent to `target`. The client (EFB app) is responsible for
/// freshness — we just snapshot the table every second and shove it
/// out the wire.
pub async fn run(state: AppState, target: SocketAddr) -> anyhow::Result<()> {
    let bind: SocketAddr = SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 0));
    let sock = UdpSocket::bind(bind).await?;
    sock.set_broadcast(true)?;
    tracing::info!(%target, "GDL90 broadcasting");

    let sock = Arc::new(sock);
    let mut ticker = tokio::time::interval(std::time::Duration::from_secs(1));

    loop {
        ticker.tick().await;
        send_tick(&sock, &state, target).await;
    }
}

async fn send_tick(sock: &UdpSocket, state: &AppState, target: SocketAddr) {
    // Heartbeat first — the EFB rejects a session that never sees one.
    let beat = encode_heartbeat();
    if let Err(e) = sock.send_to(&beat, target).await {
        tracing::warn!(error = %e, "GDL90 send heartbeat failed");
        return;
    }

    // ForeFlight identification message. Not part of the public GDL90
    // spec — it's a ForeFlight extension that gates traffic display
    // on recognising the source. Many open-source GDL90 senders
    // (Stratux, dump978) emit this once per second to stay on
    // ForeFlight's allow-list. Garmin Pilot and FlyQ ignore it
    // harmlessly.
    let ff_id = encode_foreflight_id();
    let _ = sock.send_to(&ff_id, target).await;

    let snapshot: Vec<AircraftSnapshot> = {
        let map = state.snapshot.read().expect("snapshot lock poisoned");
        map.values().cloned().collect()
    };
    for a in &snapshot {
        let Some(packet) = encode_traffic(a) else {
            continue;
        };
        if let Err(e) = sock.send_to(&packet, target).await {
            tracing::warn!(error = %e, "GDL90 send traffic failed");
            return;
        }
    }
}

// --- CRC -------------------------------------------------------------------

/// CRC-16-CCITT (poly 0x1021, init 0x0000, no reflection, no final XOR)
/// over the raw message bytes (before framing/stuffing). Output is
/// appended LSB-first per GDL90 §B.
fn crc16_ccitt(bytes: &[u8]) -> u16 {
    let mut c: u16 = 0;
    for &b in bytes {
        let idx = ((c >> 8) as u8) ^ b;
        c = c.wrapping_shl(8) ^ CRC_TABLE[idx as usize];
    }
    c
}

const CRC_TABLE: [u16; 256] = build_crc_table();

const fn build_crc_table() -> [u16; 256] {
    let mut t = [0u16; 256];
    let mut i = 0u16;
    while i < 256 {
        let mut crc: u16 = i << 8;
        let mut j = 0;
        while j < 8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
            j += 1;
        }
        t[i as usize] = crc;
        i += 1;
    }
    t
}

// --- Framing ---------------------------------------------------------------

/// Wrap a raw message in GDL90's HDLC-style framing: CRC suffix,
/// byte-stuffing of 0x7E and 0x7D, then 0x7E start/stop flags.
fn frame(payload: &[u8]) -> Vec<u8> {
    let crc = crc16_ccitt(payload);
    let mut body = Vec::with_capacity(payload.len() + 2);
    body.extend_from_slice(payload);
    body.push((crc & 0xFF) as u8);
    body.push(((crc >> 8) & 0xFF) as u8);

    let mut out = Vec::with_capacity(body.len() * 2 + 2);
    out.push(0x7E);
    for &b in &body {
        match b {
            0x7E => out.extend_from_slice(&[0x7D, 0x5E]),
            0x7D => out.extend_from_slice(&[0x7D, 0x5D]),
            _ => out.push(b),
        }
    }
    out.push(0x7E);
    out
}

// --- Heartbeat (message ID 0) ---------------------------------------------

fn encode_heartbeat() -> Vec<u8> {
    let mut buf = [0u8; 7];
    buf[0] = 0x00; // message ID
                   // Status byte 1: bit 7 = "UAT initialized + currently sending"
                   // bit 0 = "GPS position valid". We don't have GPS but ForeFlight
                   // is happy with bit 7 alone for a "stationary receiver" use case.
    buf[1] = 0x81;
    // Status byte 2: bit 0 = "UTC timing valid"; we set it because the
    // timestamp below comes from the host clock.
    buf[2] = 0x01;
    // 17-bit timestamp split: bit 7 of byte 2 holds the high bit (we
    // ignore wraparound near 23:59:59 — at one packet per second the
    // worst case is a single bad heartbeat at the day boundary).
    let secs = seconds_since_utc_midnight();
    buf[3] = (secs & 0xFF) as u8; // LSB
    buf[4] = ((secs >> 8) & 0xFF) as u8;
    // Bytes 5-6: message counts (uplink + basic). Zero is acceptable.
    frame(&buf)
}

fn seconds_since_utc_midnight() -> u32 {
    use std::time::{SystemTime, UNIX_EPOCH};
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    // Wrap into a day. 17-bit field; 86400 < 131072 so a u32 modulo is
    // fine and never overflows the field.
    u32::try_from(secs % 86_400).unwrap_or(0)
}

// --- ForeFlight ID (message ID 0x65, sub-ID 0x00) -------------------------

/// ForeFlight's source-identification extension. Not in the GDL90
/// public spec; documented in ForeFlight's "Broadcast Protocols"
/// note. Required for ForeFlight to enable traffic display from a
/// previously-unseen GDL90 source. Stratux and dump978 both emit it
/// every second; other EFB clients (Garmin Pilot, FlyQ) ignore the
/// message harmlessly.
///
/// Payload (38 bytes after message ID):
///
/// ```text
///   sub-id (1)      : 0x00
///   serial   (8 LE) : opaque device identifier
///   short name (8)  : ASCII, space-padded
///   long name  (16) : ASCII, space-padded
///   capabilities (4 BE) : bit 0 = WGS-84 geometric altitude provider,
///                         bit 1 = MSL altitude provider. We emit
///                         barometric altitudes, which ForeFlight
///                         treats as MSL — leave both bits clear.
/// ```
fn encode_foreflight_id() -> Vec<u8> {
    let mut buf = [0u8; 39];
    buf[0] = 0x65; // message ID
    buf[1] = 0x00; // sub-message ID: identification

    // Serial: any unique 64-bit token. The high bits spell "rs1090".
    let serial = 0x7273_3130_3930_0001_u64;
    buf[2..10].copy_from_slice(&serial.to_le_bytes());

    // Short name (8 bytes, space-padded).
    let short = b"rs1090  ";
    buf[10..18].copy_from_slice(short);

    // Long name (16 bytes, space-padded).
    let long = b"rs1090 ADS-B    ";
    buf[18..34].copy_from_slice(long);

    // Capabilities (big-endian u32). 0 = barometric altitudes,
    // no MSL conversion. ForeFlight treats the GDL90 altitude as MSL,
    // which is what our baro altitudes effectively are.
    buf[34..38].copy_from_slice(&0u32.to_be_bytes());

    // (Byte 38 in some references is "MSL altitude scaling" reserved
    // for future use; left zero.)

    frame(&buf)
}

// --- Traffic Report (message ID 20) ----------------------------------------

/// Encode a single aircraft as a GDL90 traffic report. Returns `None`
/// if the snapshot lacks the bare minimum (a position fix) — ForeFlight
/// silently ignores reports without a usable lat/lon anyway.
fn encode_traffic(a: &AircraftSnapshot) -> Option<Vec<u8>> {
    let position = a.position.as_ref()?;

    let mut buf = [0u8; 28];
    buf[0] = 0x14; // message ID = 20

    // Byte 1: high nibble = status (0 = no alert), low nibble = address
    // type (0 = ADS-B with ICAO address).
    buf[1] = 0x00;

    // Bytes 2-4: participant address, big-endian 24-bit.
    let icao = a.icao.to_bytes();
    buf[2] = icao[0];
    buf[3] = icao[1];
    buf[4] = icao[2];

    // Bytes 5-7: latitude as signed-24 semicircles × 2^23/180.
    let lat = encode_semicircle(position.lat);
    buf[5] = ((lat >> 16) & 0xFF) as u8;
    buf[6] = ((lat >> 8) & 0xFF) as u8;
    buf[7] = (lat & 0xFF) as u8;

    // Bytes 8-10: longitude.
    let lon = encode_semicircle(position.lon);
    buf[8] = ((lon >> 16) & 0xFF) as u8;
    buf[9] = ((lon >> 8) & 0xFF) as u8;
    buf[10] = (lon & 0xFF) as u8;

    // Bytes 11-12: altitude (12 bits) | misc (4 bits).
    let alt12 = encode_altitude(position.alt_ft);
    // Misc: i=1 (airborne) << 2 | w=0 (updated) << 1 | t=1 (true track) << 0
    //     = 0b1001 = 0x9.
    let misc: u8 = 0b1001;
    buf[11] = ((alt12 >> 4) & 0xFF) as u8;
    buf[12] = (((alt12 & 0x0F) as u8) << 4) | misc;

    // Byte 13: NIC (high nibble) | NACp (low nibble). We don't decode
    // operational-status squitters yet, so use 8/8 — "valid, reasonably
    // accurate" — which is what most ADS-B Out installations emit.
    buf[13] = (8 << 4) | 8;

    // Bytes 14-16: horizontal velocity (12-bit kt) | vertical velocity
    // (signed 12-bit, 64 fpm units).
    let h: u32 = a
        .velocity
        .as_ref()
        .and_then(|v| v.gs_mps)
        .map_or(0xFFF, |mps| {
            let kt = (mps * 1.94384).round() as i32;
            kt.clamp(0, 0xFFE) as u32
        });
    let v: u32 = a
        .velocity
        .as_ref()
        .and_then(|v| v.vr_mps)
        .map_or(0x800, |mps| {
            // 64 fpm units → divide fpm by 64 and clamp to ±510.
            let fpm = (mps * 196.85).round() as i32;
            let units = (fpm / 64).clamp(-510, 510);
            // 2's-complement low 12 bits.
            (units as u32) & 0xFFF
        });
    buf[14] = ((h >> 4) & 0xFF) as u8;
    buf[15] = (((h & 0x0F) << 4) as u8) | ((v >> 8) & 0x0F) as u8;
    buf[16] = (v & 0xFF) as u8;

    // Byte 17: track (0..255 mapping to 0..360°). Spec invalid = 0,
    // not 0xFF — ForeFlight tolerates either; we use 0 when unknown.
    buf[17] = a
        .velocity
        .as_ref()
        .and_then(|v| v.track_deg.or(v.heading_deg))
        .map_or(0u8, |t| {
            let scaled = ((t / 360.0) * 256.0).round() as i32;
            scaled.rem_euclid(256) as u8
        });

    // Byte 18: emitter category. 0 = unknown. We could derive from
    // `category` (TC 1-4 emitter set + cat) but the encoding is
    // GDL90-specific and unknown is acceptable.
    buf[18] = 0;

    // Bytes 19-26: callsign. 8 ASCII chars, space-padded. If we have
    // no callsign, fall back to the ICAO hex so something appears on
    // the EFB.
    let cs_string = a
        .callsign
        .clone()
        .unwrap_or_else(|| format!("{:02X}{:02X}{:02X}", icao[0], icao[1], icao[2]));
    let cs_bytes = cs_string.as_bytes();
    for i in 0..8 {
        buf[19 + i] = if i < cs_bytes.len() {
            // GDL90 callsigns are uppercase A-Z, 0-9 only. Map anything
            // else to space — keeps the EFB from rendering control bytes.
            let b = cs_bytes[i];
            if b.is_ascii_alphanumeric() {
                b.to_ascii_uppercase()
            } else {
                b' '
            }
        } else {
            b' '
        };
    }

    // Byte 27: emergency code (high nibble) | spare. 0 = no emergency.
    buf[27] = 0;

    // PositionSource isn't consumed here but the import would warn if
    // unused; keep it for future use (e.g. NIC scaling on local vs
    // global decode).
    let _ = std::mem::discriminant(&PositionSource::Global);

    Some(frame(&buf))
}

/// Encode a latitude or longitude as the signed 24-bit "semicircle"
/// the GDL90 traffic report expects: `value × 2^23 / 180`.
fn encode_semicircle(deg: f64) -> u32 {
    let scaled = (deg * f64::from(1u32 << 23) / 180.0).round() as i32;
    // 2's-complement low 24 bits.
    (scaled as u32) & 0x00FF_FFFF
}

/// Encode altitude as the GDL90 12-bit field. Resolution 25 ft,
/// offset −1000 ft. Sentinel 0xFFF for unknown.
fn encode_altitude(alt_ft: Option<i32>) -> u32 {
    match alt_ft {
        Some(ft) => {
            let raw = (ft + 1000) / 25;
            u32::try_from(raw.clamp(0, 0xFFE)).unwrap_or(0xFFF)
        }
        None => 0xFFF,
    }
}

// --- Tests ----------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use rs1090::message::Icao;

    #[test]
    fn crc_known_vector_empty() {
        // CRC of an empty message with init=0 is 0.
        assert_eq!(crc16_ccitt(&[]), 0);
    }

    #[test]
    fn crc_known_vector_123456789() {
        // CRC-16/XMODEM (poly 0x1021, init 0x0000, no reflection, no
        // final XOR) — the form GDL90 specifies — check value over
        // "123456789" is 0x31C3. (The more-famous 0x29B1 belongs to
        // CRC-16-CCITT-FALSE, which uses init 0xFFFF.)
        assert_eq!(crc16_ccitt(b"123456789"), 0x31C3);
    }

    #[test]
    fn frame_escapes_0x7e_and_0x7d() {
        let body = vec![0x01, 0x7E, 0x02, 0x7D, 0x03];
        let framed = frame(&body);
        // Starts and ends with 0x7E.
        assert_eq!(framed.first(), Some(&0x7E));
        assert_eq!(framed.last(), Some(&0x7E));
        // No internal 0x7E (other than flags) or unescaped 0x7D.
        let inner = &framed[1..framed.len() - 1];
        assert!(!inner.windows(1).any(|w| w == [0x7E]));
        // Each escape is followed by the right complement.
        let mut i = 0;
        while i < inner.len() {
            if inner[i] == 0x7D {
                assert!(matches!(inner[i + 1], 0x5E | 0x5D));
                i += 2;
            } else {
                i += 1;
            }
        }
    }

    #[test]
    fn encode_semicircle_north_positive() {
        // 40.7021° N → +1_896_846 ish.
        let v = encode_semicircle(40.7021);
        let signed = v as i32;
        // 2^23/180 * 40.7021 = 1896890 (give or take rounding).
        assert!((1_896_000..=1_897_000).contains(&signed), "got {signed}");
    }

    #[test]
    fn encode_semicircle_west_two_complement() {
        // -73.9826° W. 2's-complement 24-bit; the high byte should
        // have the sign bit set.
        let v = encode_semicircle(-73.9826);
        assert!(v & 0x0080_0000 != 0, "expected sign bit, got 0x{v:06X}");
    }

    #[test]
    fn encode_altitude_known_values() {
        assert_eq!(encode_altitude(None), 0xFFF);
        assert_eq!(encode_altitude(Some(-1000)), 0);
        assert_eq!(encode_altitude(Some(0)), 40);
        assert_eq!(encode_altitude(Some(30_000)), 1240);
        // Above-max clamps to 0xFFE (not 0xFFF, which is the "unknown"
        // sentinel; we deliberately don't reuse it for overflow).
        assert_eq!(encode_altitude(Some(200_000)), 0xFFE);
    }

    #[test]
    fn heartbeat_is_well_formed() {
        let h = encode_heartbeat();
        // Frame flags + 7-byte payload + 2-byte CRC = 11 bytes minimum
        // (no stuffing needed if neither CRC byte happens to be 0x7E/0x7D).
        assert_eq!(h.first(), Some(&0x7E));
        assert_eq!(h.last(), Some(&0x7E));
        // Roundtrip: strip flags, un-escape, CRC over the first 7
        // bytes should equal the trailing 2 bytes LSB-first.
        let de = unframe(&h);
        assert_eq!(de[0], 0x00);
        let crc = crc16_ccitt(&de[..7]);
        assert_eq!(de[7], (crc & 0xFF) as u8);
        assert_eq!(de[8], ((crc >> 8) & 0xFF) as u8);
    }

    #[test]
    fn traffic_report_skips_aircraft_without_position() {
        let no_pos = AircraftSnapshot {
            icao: Icao::from_bytes([0xAB, 0xCD, 0xEF]),
            callsign: None,
            category: None,
            position: None,
            velocity: None,
            counters: crate::events::SnapshotCounters {
                messages_total: 0,
                crc_clean: 0,
                crc_corrected: 0,
                crc_address_recovered: 0,
            },
        };
        assert!(encode_traffic(&no_pos).is_none());
    }

    #[test]
    fn traffic_report_layout() {
        let a = AircraftSnapshot {
            icao: Icao::from_bytes([0xA0, 0xBA, 0x4E]),
            callsign: Some("UAL123".into()),
            category: None,
            position: Some(crate::events::SnapshotPosition {
                lat: 40.7021,
                lon: -73.9826,
                alt_ft: Some(3625),
                alt_source: Some("baro"),
                source: rs1090::state::PositionSource::Global,
            }),
            velocity: Some(crate::events::SnapshotVelocity {
                gs_mps: Some(150.0),
                track_deg: Some(207.0),
                ias_mps: None,
                heading_deg: None,
                heading_magnetic: None,
                vr_mps: Some(-5.0),
                vr_source: None,
            }),
            counters: crate::events::SnapshotCounters {
                messages_total: 0,
                crc_clean: 0,
                crc_corrected: 0,
                crc_address_recovered: 0,
            },
        };
        let framed = encode_traffic(&a).expect("encoded");
        let payload = unframe(&framed);
        assert_eq!(payload.len(), 30); // 28 message + 2 CRC
        assert_eq!(payload[0], 0x14);
        assert_eq!(&payload[2..5], &[0xA0, 0xBA, 0x4E]);
        // Callsign upper-cased ASCII.
        assert_eq!(&payload[19..27], b"UAL123  ");
    }

    /// Strip 0x7E flags and un-stuff escapes, returning the inner
    /// message + its 2-byte trailing CRC.
    fn unframe(framed: &[u8]) -> Vec<u8> {
        assert_eq!(framed.first(), Some(&0x7E));
        assert_eq!(framed.last(), Some(&0x7E));
        let inner = &framed[1..framed.len() - 1];
        let mut out = Vec::with_capacity(inner.len());
        let mut i = 0;
        while i < inner.len() {
            if inner[i] == 0x7D {
                out.push(inner[i + 1] ^ 0x20);
                i += 2;
            } else {
                out.push(inner[i]);
                i += 1;
            }
        }
        out
    }
}

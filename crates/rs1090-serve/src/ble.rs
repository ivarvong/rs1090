//! BLE GATT peripheral. Advertises rs1090 as a Bluetooth Low Energy
//! peripheral so iPhone / Android debug apps (nRF Connect, LightBlue,
//! BLE Scanner) can subscribe to live aircraft data without any custom
//! mobile code — the debug app *is* the UI.
//!
//! Three GATT characteristics under one custom service:
//!
//! - **count** (`u16` little-endian) — the number of aircraft currently
//!   tracked. Read + notify on every refresh tick.
//! - **nearest** (15 bytes packed binary) — the aircraft with the lowest
//!   altitude that has a position fix. Read + notify. Format:
//!
//!   ```text
//!   bytes  0..3    ICAO (big-endian, 24-bit; high byte zero)
//!          3..5    altitude / 25 ft (u16 LE; 0xFFFF if unknown)
//!          5..9    lat × 1e6 as i32 LE
//!          9..13   lon × 1e6 as i32 LE
//!         13..15   track × 10 as u16 LE (0xFFFF if unknown)
//!   ```
//!
//! - **summary** (UTF-8) — a human-readable one-line snapshot of the
//!   top few aircraft, sized to fit comfortably in a 23-byte ATT MTU
//!   (default — clients can negotiate up). Read + notify.
//!
//! The peripheral re-pushes notifications on every event from the
//! `tokio::sync::broadcast` channel that drives the SSE side, so a
//! subscribed BLE client sees updates at the same cadence as a
//! subscribed web client. Linux-only via the `bluer` crate.
//!
//! Lint exemption: BLE / GATT / ICAO / UUID aren't Rust items.
#![allow(clippy::doc_markdown)]

use std::collections::BTreeSet;
use std::sync::Arc;

use bluer::adv::{Advertisement, Type as AdType};
use bluer::gatt::local::{
    Application, Characteristic, CharacteristicNotify, CharacteristicNotifyMethod,
    CharacteristicRead, Service,
};
use bluer::Uuid;
use futures_util::FutureExt;
use tokio::sync::RwLock;

use rs1090::message::Icao;
use rs1090::state::PositionSource;

use crate::broadcaster::AppState;
use crate::events::AircraftSnapshot;

// Custom 128-bit UUIDs. The prefix `10901090` is a deliberate eye-catcher
// so the service jumps out in BLE scanner apps full of generic 0x1800-ish
// SIG numbers. Bytes are little-endian on the wire but the constants
// here are written in their conventional big-endian form.
const SERVICE_UUID: Uuid = Uuid::from_u128(0x1090_1090_1090_1090_1090_1090_1090_1090);
const COUNT_UUID: Uuid = Uuid::from_u128(0x1090_1090_0001_4000_8000_00805f9b34fb);
const NEAREST_UUID: Uuid = Uuid::from_u128(0x1090_1090_0002_4000_8000_00805f9b34fb);
const SUMMARY_UUID: Uuid = Uuid::from_u128(0x1090_1090_0003_4000_8000_00805f9b34fb);

/// Cached current values that the notify-loop closures read. One shared
/// instance behind an `Arc<RwLock>`; the supervisor task rewrites it on
/// every event from the broadcaster, and per-subscriber notify tasks
/// snapshot it before each push.
#[derive(Default, Clone)]
struct Latest {
    count: u16,
    nearest: Vec<u8>,
    summary: String,
}

/// Run the BLE peripheral forever. Errors propagate to the caller, which
/// is the dedicated BLE task; the rest of the server is unaffected.
pub async fn run(state: AppState) -> anyhow::Result<()> {
    let session = bluer::Session::new().await?;
    let adapter = session.default_adapter().await?;
    adapter.set_powered(true).await?;
    let adapter_name = adapter.name().to_string();
    eprintln!("rs1090-serve: BLE adapter {adapter_name}, advertising rs1090 service");

    let latest: Arc<RwLock<Latest>> = Arc::new(RwLock::new(Latest::default()));

    let app = build_application(latest.clone()).await;
    let _app_handle = adapter.serve_gatt_application(app).await?;

    let advertisement = Advertisement {
        advertisement_type: AdType::Peripheral,
        service_uuids: {
            let mut s = BTreeSet::new();
            s.insert(SERVICE_UUID);
            s
        },
        local_name: Some("rs1090".into()),
        discoverable: Some(true),
        ..Default::default()
    };
    let _adv_handle = adapter.advertise(advertisement).await?;

    // Spawn a supervisor that refreshes `latest` on every broadcaster
    // event, capped at one rewrite per ~250 ms so a busy receiver
    // doesn't spam notifications faster than a BLE link can absorb.
    let mut rx = state.broadcaster.subscribe();
    let mut last_push = std::time::Instant::now() - std::time::Duration::from_secs(1);
    loop {
        // Drain whatever's pending; we only care about the trigger.
        match rx.recv().await {
            Ok(_) | Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => {}
            Err(tokio::sync::broadcast::error::RecvError::Closed) => break,
        }
        if last_push.elapsed() < std::time::Duration::from_millis(250) {
            continue;
        }
        last_push = std::time::Instant::now();
        let snapshot = {
            let map = state.snapshot.read().expect("snapshot lock poisoned");
            map.values().cloned().collect::<Vec<_>>()
        };
        let new_latest = derive_latest(&snapshot);
        *latest.write().await = new_latest;
    }
    Ok(())
}

async fn build_application(latest: Arc<RwLock<Latest>>) -> Application {
    let count_read = make_read(latest.clone(), |l| l.count.to_le_bytes().to_vec());
    let count_notify = make_notify(latest.clone(), |l| l.count.to_le_bytes().to_vec());

    let nearest_read = make_read(latest.clone(), |l| l.nearest.clone());
    let nearest_notify = make_notify(latest.clone(), |l| l.nearest.clone());

    let summary_read = make_read(latest.clone(), |l| l.summary.as_bytes().to_vec());
    let summary_notify = make_notify(latest.clone(), |l| l.summary.as_bytes().to_vec());

    Application {
        services: vec![Service {
            uuid: SERVICE_UUID,
            primary: true,
            characteristics: vec![
                Characteristic {
                    uuid: COUNT_UUID,
                    read: Some(count_read),
                    notify: Some(count_notify),
                    ..Default::default()
                },
                Characteristic {
                    uuid: NEAREST_UUID,
                    read: Some(nearest_read),
                    notify: Some(nearest_notify),
                    ..Default::default()
                },
                Characteristic {
                    uuid: SUMMARY_UUID,
                    read: Some(summary_read),
                    notify: Some(summary_notify),
                    ..Default::default()
                },
            ],
            ..Default::default()
        }],
        ..Default::default()
    }
}

/// Build a `CharacteristicRead` whose value is computed from the shared
/// `Latest` snapshot by `pick`. Cheap clone per read.
fn make_read<F>(latest: Arc<RwLock<Latest>>, pick: F) -> CharacteristicRead
where
    F: Fn(&Latest) -> Vec<u8> + Send + Sync + Clone + 'static,
{
    CharacteristicRead {
        read: true,
        fun: Box::new(move |_req| {
            let latest = latest.clone();
            let pick = pick.clone();
            async move {
                let l = latest.read().await;
                Ok(pick(&l))
            }
            .boxed()
        }),
        ..Default::default()
    }
}

/// Build a `CharacteristicNotify` whose subscribers each get one task
/// that polls `latest` at 1 Hz and pushes the current bytes. Polling
/// over edge-triggering keeps the code small; the cost is one wake per
/// subscriber per second which is negligible on a Pi at our event
/// rates. Each task exits the moment `notifier.notify` errors (the
/// `bluer` signal that the client unsubscribed or disconnected).
fn make_notify<F>(latest: Arc<RwLock<Latest>>, pick: F) -> CharacteristicNotify
where
    F: Fn(&Latest) -> Vec<u8> + Send + Sync + Clone + 'static,
{
    CharacteristicNotify {
        notify: true,
        method: CharacteristicNotifyMethod::Fun(Box::new(move |mut notifier| {
            let latest = latest.clone();
            let pick = pick.clone();
            async move {
                tokio::spawn(async move {
                    let mut last_bytes: Vec<u8> = Vec::new();
                    loop {
                        let bytes = {
                            let l = latest.read().await;
                            pick(&l)
                        };
                        if bytes != last_bytes {
                            if notifier.notify(bytes.clone()).await.is_err() {
                                return;
                            }
                            last_bytes = bytes;
                        }
                        tokio::time::sleep(std::time::Duration::from_secs(1)).await;
                    }
                });
            }
            .boxed()
        })),
        ..Default::default()
    }
}

/// Compute the three published values from a snapshot of the aircraft
/// table. The "nearest" we pick is the lowest-altitude aircraft with a
/// position fix — the most-likely-to-be-interesting on an LGA-shaped
/// receiver. Summary fits an ATT default 20-byte MTU when truncated.
fn derive_latest(aircraft: &[AircraftSnapshot]) -> Latest {
    let count = u16::try_from(aircraft.len()).unwrap_or(u16::MAX);

    // Pick the lowest-altitude aircraft with a position fix. None
    // collapses to a zeroed payload — clients see the all-zero ICAO
    // and treat it as "no nearest".
    let nearest = aircraft
        .iter()
        .filter(|a| a.position.is_some() && a.position.as_ref().and_then(|p| p.alt_ft).is_some())
        .min_by_key(|a| {
            a.position
                .as_ref()
                .and_then(|p| p.alt_ft)
                .unwrap_or(i32::MAX)
        });

    let nearest_bytes = match nearest {
        Some(a) => pack_nearest(a),
        None => vec![0u8; 15],
    };

    let summary = match nearest {
        Some(a) => format_summary(a, count),
        None => format!("{count} ac"),
    };

    Latest {
        count,
        nearest: nearest_bytes,
        summary,
    }
}

fn pack_nearest(a: &AircraftSnapshot) -> Vec<u8> {
    let mut out = Vec::with_capacity(15);
    out.extend_from_slice(&a.icao.to_bytes()); // 3
    let alt = a
        .position
        .as_ref()
        .and_then(|p| p.alt_ft)
        .map_or(0xFFFFu16, |ft| u16::try_from(ft / 25).unwrap_or(u16::MAX));
    out.extend_from_slice(&alt.to_le_bytes()); // 2

    let (lat, lon) = a
        .position
        .as_ref()
        .map_or((0, 0), |p| ((p.lat * 1e6) as i32, (p.lon * 1e6) as i32));
    out.extend_from_slice(&lat.to_le_bytes()); // 4
    out.extend_from_slice(&lon.to_le_bytes()); // 4

    let track = a
        .velocity
        .as_ref()
        .and_then(|v| v.track_deg.or(v.heading_deg))
        .map_or(0xFFFFu16, |t| {
            u16::try_from((t * 10.0).round() as i32).unwrap_or(u16::MAX)
        });
    out.extend_from_slice(&track.to_le_bytes()); // 2

    debug_assert_eq!(out.len(), 15);
    out
}

fn format_summary(a: &AircraftSnapshot, count: u16) -> String {
    let icao = Icao::from_bytes(a.icao.to_bytes());
    let callsign = a.callsign.as_deref().unwrap_or("");
    let alt = a
        .position
        .as_ref()
        .and_then(|p| p.alt_ft)
        .map_or_else(|| "?".to_string(), |ft| format!("{ft}"));
    let source_tag = a.position.as_ref().map_or("?", |p| match p.source {
        PositionSource::Global => "G",
        PositionSource::Local => "L",
        _ => "?",
    });
    // Aim for <=20 bytes: "AC123 N777ZA 750ft G  ~12"
    let head = if callsign.is_empty() {
        format!("{} {}ft{} #{}", icao, alt, source_tag, count)
    } else {
        format!("{} {}ft{} #{}", callsign, alt, source_tag, count)
    };
    head
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::events::SnapshotCounters;

    fn sample(icao: [u8; 3], alt_ft: Option<i32>, track: Option<f64>) -> AircraftSnapshot {
        AircraftSnapshot {
            icao: Icao::from_bytes(icao),
            callsign: None,
            category: None,
            position: alt_ft.map(|alt| crate::events::SnapshotPosition {
                lat: 40.7,
                lon: -74.0,
                alt_ft: Some(alt),
                alt_source: Some("baro"),
                source: PositionSource::Global,
            }),
            velocity: track.map(|t| crate::events::SnapshotVelocity {
                gs_mps: Some(150.0),
                track_deg: Some(t),
                ias_mps: None,
                heading_deg: None,
                heading_magnetic: None,
                vr_mps: None,
                vr_source: None,
            }),
            counters: SnapshotCounters {
                messages_total: 0,
                crc_clean: 0,
                crc_corrected: 0,
                crc_address_recovered: 0,
            },
        }
    }

    #[test]
    fn pack_nearest_layout_is_fifteen_bytes_msb_icao_le_rest() {
        let a = sample([0xAB, 0xCD, 0xEF], Some(1000), Some(207.5));
        let bytes = pack_nearest(&a);
        assert_eq!(bytes.len(), 15);
        // ICAO is the on-wire MSB-first triplet.
        assert_eq!(&bytes[0..3], &[0xAB, 0xCD, 0xEF]);
        // alt 1000 ft / 25 = 40, LE u16.
        assert_eq!(&bytes[3..5], &[40, 0]);
        // lat 40.7 × 1e6 = 40_700_000, LE i32.
        assert_eq!(&bytes[5..9], &40_700_000i32.to_le_bytes());
        // lon -74.0 × 1e6 = -74_000_000, LE i32.
        assert_eq!(&bytes[9..13], &(-74_000_000i32).to_le_bytes());
        // track 207.5 × 10 = 2075, LE u16.
        assert_eq!(&bytes[13..15], &2075u16.to_le_bytes());
    }

    #[test]
    fn pack_nearest_signals_unknown_with_sentinels() {
        let a = sample([0x00, 0x00, 0x01], None, None);
        let bytes = pack_nearest(&a);
        assert_eq!(bytes.len(), 15);
        // No position → alt sentinel 0xFFFF, lat/lon zero.
        assert_eq!(&bytes[3..5], &[0xFF, 0xFF]);
        assert_eq!(&bytes[5..9], &0i32.to_le_bytes());
        // No velocity → track sentinel 0xFFFF.
        assert_eq!(&bytes[13..15], &[0xFF, 0xFF]);
    }

    #[test]
    fn derive_latest_picks_lowest_altitude_with_position() {
        let fleet = vec![
            sample([0, 0, 1], Some(35_000), Some(90.0)), // high cruise
            sample([0, 0, 2], Some(800), Some(207.0)),   // on final ← should win
            sample([0, 0, 3], Some(5_000), Some(15.0)),
            sample([0, 0, 4], None, None), // no fix, ineligible
        ];
        let latest = derive_latest(&fleet);
        assert_eq!(latest.count, 4);
        // First 3 bytes are the ICAO of the chosen aircraft.
        assert_eq!(&latest.nearest[0..3], &[0, 0, 2]);
        // Summary should mention the count and contain "ft".
        assert!(
            latest.summary.contains("ft"),
            "summary={:?}",
            latest.summary
        );
    }

    #[test]
    fn derive_latest_handles_empty_fleet() {
        let latest = derive_latest(&[]);
        assert_eq!(latest.count, 0);
        assert_eq!(latest.nearest, vec![0u8; 15]);
        assert_eq!(latest.summary, "0 ac");
    }
}

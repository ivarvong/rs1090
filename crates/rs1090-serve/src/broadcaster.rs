//! Owns the live aircraft snapshot and broadcasts SSE events.
//!
//! Two pieces:
//!
//! - `snapshot: ArcSwap<AircraftMap>` (here implemented as
//!   `Arc<RwLock<...>>` to avoid pulling in `arc-swap`) for `/aircraft`
//!   reads. Updates happen on the decoder thread; reads are lock-free
//!   in the common case (single Arc clone).
//! - `tokio::sync::broadcast` channel that fans events out to every
//!   connected SSE client. Slow clients lag rather than block the decode
//!   thread — `broadcast::Sender::send` returns immediately even if
//!   subscribers are far behind, and individual receivers see
//!   `RecvError::Lagged(n)` so they know how many they missed.

use std::collections::HashMap;
use std::sync::{Arc, RwLock};

use tokio::sync::broadcast;

use rs1090::message::Icao;
use rs1090::state::{Aircraft, PositionSource};

use crate::events::{
    AircraftSnapshot, EventEnvelope, SnapshotCounters, SnapshotPosition, SnapshotVelocity,
};

/// Default broadcast channel capacity. Each SSE client maintains its own
/// receiver position; events older than this many positions before the
/// slowest receiver get dropped (and that receiver sees `RecvError::Lagged`).
pub const DEFAULT_BROADCAST_CAPACITY: usize = 4096;

/// Shared state cloned into every axum handler.
#[derive(Clone)]
pub struct AppState {
    pub broadcaster: broadcast::Sender<EventEnvelope>,
    pub snapshot: Arc<RwLock<HashMap<Icao, AircraftSnapshot>>>,
}

impl AppState {
    pub fn new() -> Self {
        let (tx, _rx) = broadcast::channel(DEFAULT_BROADCAST_CAPACITY);
        Self {
            broadcaster: tx,
            snapshot: Arc::new(RwLock::new(HashMap::new())),
        }
    }
}

impl Default for AppState {
    fn default() -> Self {
        Self::new()
    }
}

/// Build a JSON-ready snapshot from a tracked aircraft.
pub fn snapshot_from(a: &Aircraft) -> AircraftSnapshot {
    AircraftSnapshot {
        icao: a.icao,
        callsign: a.callsign.map(|cs| cs.to_string()),
        category: a.category.map(|(s, c)| [s, c]),
        position: a.position.map(|p| SnapshotPosition {
            lat: p.pos.lat_deg,
            lon: p.pos.lon_deg,
            source: match p.source {
                PositionSource::Global => "global",
                PositionSource::Local => "local",
            },
        }),
        velocity: a.velocity.map(SnapshotVelocity::from_velocity),
        counters: SnapshotCounters {
            messages_total: a.counters.messages_total,
            crc_clean: a.counters.crc_clean,
            crc_corrected: a.counters.crc_corrected,
            crc_address_recovered: a.counters.crc_address_recovered,
        },
    }
}

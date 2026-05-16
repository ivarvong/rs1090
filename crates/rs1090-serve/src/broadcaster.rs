//! Owns the live aircraft snapshot and broadcasts SSE events.
//!
//! Two pieces:
//!
//! - `snapshot: Arc<RwLock<HashMap<Icao, AircraftSnapshot>>>` for
//!   `/aircraft` reads. **`RwLock` is the right primitive here — not
//!   `arc-swap`.** The decoder thread updates the snapshot on every
//!   state event (10-100 Hz of writes); HTTP reads on `/aircraft` are
//!   sporadic (a polling browser, every few seconds). With this
//!   write-heavy, read-light pattern, an `ArcSwap<HashMap>` would force
//!   the writer to rebuild and atomically swap the whole map on every
//!   event — orders of magnitude more allocation than the per-event
//!   `HashMap::insert` we do under `RwLock`. `dashmap` was considered
//!   for fine-grained locking but with a single writer thread there's
//!   no contention to fix; an `RwLock` with one writer and one or two
//!   readers spends ~zero time blocked at this rate.
//! - `tokio::sync::broadcast` channel that fans events out to every
//!   connected SSE client. Slow clients lag rather than block the decode
//!   thread — `broadcast::Sender::send` returns immediately even if
//!   subscribers are far behind, and individual receivers see
//!   `RecvError::Lagged(n)` so they know how many they missed.

use std::collections::HashMap;
use std::sync::{Arc, RwLock};

use tokio::sync::broadcast;

use rs1090::message::Icao;
use rs1090::state::Aircraft;

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
            alt_ft: p.altitude.feet(),
            alt_source: p.altitude.source_tag(),
            source: p.source,
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

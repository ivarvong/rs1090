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

use std::collections::{HashMap, VecDeque};
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex, RwLock};

use metrics_exporter_prometheus::PrometheusHandle;
use tokio::sync::{broadcast, Notify};

use rs1090::frame::Frame;
use rs1090::message::Icao;
use rs1090::state::Aircraft;

use crate::events::{
    AircraftSnapshot, EventEnvelope, SnapshotCounters, SnapshotPosition, SnapshotVelocity,
};

/// Default broadcast channel capacity. Each SSE client maintains its own
/// receiver position; events older than this many positions before the
/// slowest receiver get dropped (and that receiver sees `RecvError::Lagged`).
pub const DEFAULT_BROADCAST_CAPACITY: usize = 4096;

/// Recent-event ring buffer capacity, in events. Per DESIGN.md §12.4:
/// when an SSE client reconnects with `Last-Event-ID: N`, the server
/// replays any events with id > N from this ring before subscribing
/// the client to the live broadcast. Sized the same as the broadcast
/// channel so a client that fully exhausts the broadcast (forcing
/// `RecvError::Lagged`) can still recover the same window from here.
///
/// 4096 events at the ~100/s live rate observed on the Pi Zero 2 W is
/// ~40 s of replay history — plenty for a transient network blip, far
/// short of "indefinite buffering" that would grow unbounded.
pub const REPLAY_CAPACITY: usize = 4096;

/// Shared state cloned into every axum handler.
#[derive(Clone)]
pub struct AppState {
    pub broadcaster: broadcast::Sender<EventEnvelope>,
    /// Per-frame fan-out — every CRC-clean or 1-bit-corrected frame
    /// the detector emits is pushed here. Consumers that need raw
    /// frame bytes (AVR-text TCP, Beast TCP, dump1090-compatible
    /// feeders) subscribe to this rather than reconstructing bytes
    /// from the decoded `EventEnvelope` stream — the envelope is
    /// post-decode, the frame is pre-decode, and they're disjoint
    /// information once you cross that boundary.
    pub frame_broadcaster: broadcast::Sender<Frame>,
    pub snapshot: Arc<RwLock<HashMap<Icao, AircraftSnapshot>>>,
    /// Set to `true` while the decoder thread is running. The thread
    /// flips this to `false` on any exit path — clean return, error
    /// return, or panic unwind — via a `Drop` guard installed at the
    /// top of its body. `/healthz` returns 503 once this goes false,
    /// so a wedged decoder is detectable by load balancers and
    /// monitoring without polling the snapshot for liveness.
    pub decoder_alive: Arc<AtomicBool>,
    /// Tokio-side counterpart to `decoder_alive`. The decoder thread's
    /// Drop guard calls `notify_one()` on any abnormal exit; the
    /// runtime's shutdown signal awaits this alongside Ctrl-C so a
    /// dead decoder takes the HTTP server down with it instead of
    /// leaving the process serving a frozen snapshot.
    pub decoder_died: Arc<Notify>,
    /// Prometheus exposition handle. Cloned cheaply; the `/metrics`
    /// HTTP handler calls `.render()` on it to produce the text.
    pub metrics: PrometheusHandle,
    /// Ring buffer of the last [`REPLAY_CAPACITY`] event envelopes.
    /// Written by the decoder loop (one lock per state event; the
    /// critical section is two `VecDeque` ops); read on SSE reconnect
    /// to serve the `Last-Event-ID` replay gap. A `std::sync::Mutex`
    /// is the right primitive: writes are bursty but trivially short
    /// and reads are rare (only on reconnect), so contention is near
    /// zero in practice.
    pub replay: Arc<Mutex<VecDeque<EventEnvelope>>>,
}

impl AppState {
    pub fn new(metrics: PrometheusHandle) -> Self {
        let (tx, _rx) = broadcast::channel(DEFAULT_BROADCAST_CAPACITY);
        let (frame_tx, _frame_rx) = broadcast::channel(DEFAULT_BROADCAST_CAPACITY);
        Self {
            broadcaster: tx,
            frame_broadcaster: frame_tx,
            snapshot: Arc::new(RwLock::new(HashMap::new())),
            decoder_alive: Arc::new(AtomicBool::new(true)),
            decoder_died: Arc::new(Notify::new()),
            metrics,
            replay: Arc::new(Mutex::new(VecDeque::with_capacity(REPLAY_CAPACITY))),
        }
    }
}

/// Append an envelope to the replay ring, evicting the oldest entry
/// when the capacity is reached. Caller must hold no other locks; the
/// critical section is two `VecDeque` ops so it returns near-instantly.
pub fn push_replay(replay: &Mutex<VecDeque<EventEnvelope>>, env: EventEnvelope) {
    let mut ring = replay.lock().expect("replay lock poisoned");
    if ring.len() >= REPLAY_CAPACITY {
        ring.pop_front();
    }
    ring.push_back(env);
}

/// Snapshot the slice of replay events with `id > last_event_id`.
/// Cloned out from under the lock so the caller can yield without
/// blocking the decoder.
pub fn replay_after(
    replay: &Mutex<VecDeque<EventEnvelope>>,
    last_event_id: u64,
) -> Vec<EventEnvelope> {
    let ring = replay.lock().expect("replay lock poisoned");
    ring.iter()
        .filter(|e| e.id > last_event_id)
        .cloned()
        .collect()
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

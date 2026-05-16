//! Prometheus `/metrics` exposition.
//!
//! Operator-visible counters and gauges for everything that matters
//! when an `rs1090-serve` is running unattended in the field. We use
//! the `metrics` crate facade so the call sites (in the decoder loop,
//! the broadcaster, the SSE handler) don't know or care about the
//! backend; the Prometheus handle lives here and is queried only by
//! the `/metrics` HTTP handler.
//!
//! ## Metric names
//!
//! | Name | Type | Description |
//! |---|---|---|
//! | `rs1090_frames_total{outcome}` | counter | every `Frame` the detector emits, labelled `clean`/`corrected`/`failed` |
//! | `rs1090_state_events_total{kind}` | counter | every `StateEvent` the tracker emits |
//! | `rs1090_aircraft_tracked` | gauge | size of the tracker's active table |
//! | `rs1090_sse_subscribers` | gauge | currently-connected SSE clients |
//! | `rs1090_decoder_alive` | gauge | 1 while the decoder thread is alive, 0 once it has died |
//!
//! Labels are kept low-cardinality on purpose. We don't tag per-ICAO
//! (would blow up the time-series count); aircraft state lives in the
//! tracker, not in metrics.

use metrics_exporter_prometheus::{PrometheusBuilder, PrometheusHandle};

/// Install the Prometheus recorder as the global `metrics` backend
/// and return the handle that renders the exposition text.
///
/// Idempotent only in the sense that the call site (`main`) calls it
/// once at startup — the underlying `metrics` recorder is install-only-once.
pub fn install() -> PrometheusHandle {
    PrometheusBuilder::new()
        .install_recorder()
        .expect("install Prometheus recorder")
}

// Metric names as constants so call sites can't typo them. The
// suffixes follow the Prometheus naming conventions (`_total` for
// counters, no suffix for gauges).
pub const FRAMES_TOTAL: &str = "rs1090_frames_total";
pub const STATE_EVENTS_TOTAL: &str = "rs1090_state_events_total";
pub const AIRCRAFT_TRACKED: &str = "rs1090_aircraft_tracked";
pub const SSE_SUBSCRIBERS: &str = "rs1090_sse_subscribers";
pub const DECODER_ALIVE: &str = "rs1090_decoder_alive";

/// Describe each metric once at startup so `/metrics` includes
/// `# HELP` and `# TYPE` lines even before the first observation.
pub fn describe_all() {
    metrics::describe_counter!(
        FRAMES_TOTAL,
        "Mode S frames emitted by the detector, labelled by CRC outcome",
    );
    metrics::describe_counter!(
        STATE_EVENTS_TOTAL,
        "State-tracker events (acquired, identification, position, velocity, lost, …)",
    );
    metrics::describe_gauge!(
        AIRCRAFT_TRACKED,
        "Number of aircraft currently in the state tracker",
    );
    metrics::describe_gauge!(
        SSE_SUBSCRIBERS,
        "Number of currently-connected SSE subscribers on /stream",
    );
    metrics::describe_gauge!(
        DECODER_ALIVE,
        "1 while the decoder thread is running, 0 once it has exited abnormally",
    );
}

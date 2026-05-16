//! Axum routes.

use std::collections::HashSet;
use std::convert::Infallible;
use std::time::Duration;

use axum::extract::{Path, Query, State};
use axum::http::HeaderMap;
use axum::response::sse::{Event as SseEvent, KeepAlive, Sse};
use axum::response::{IntoResponse, Response};
use axum::routing::get;
use axum::{Json, Router};
use futures_util::stream::Stream;
use serde::Deserialize;
use tokio_stream::wrappers::BroadcastStream;
use tokio_stream::StreamExt as _;

use rs1090::message::Icao;

use crate::broadcaster::AppState;
use crate::events::{AircraftSnapshot, EventEnvelope};

pub fn router(state: AppState) -> Router {
    Router::new()
        .route("/", get(index))
        .route("/healthz", get(healthz))
        .route("/aircraft", get(list_aircraft))
        .route("/aircraft/:icao", get(get_aircraft))
        .route("/stream", get(stream))
        .with_state(state)
}

/// Single-page live map. Static HTML/CSS/JS embedded at compile time so
/// the binary stays self-contained; Leaflet is pulled from a CDN at
/// page-load time (the only runtime dependency). Talks to `/aircraft`
/// once for a snapshot, then `/stream` for live updates.
const INDEX_HTML: &str = include_str!("../static/index.html");

async fn index() -> Response {
    ([("content-type", "text/html; charset=utf-8")], INDEX_HTML).into_response()
}

/// Returns 200 OK / `"ok"` while the decoder thread is alive, 503
/// `"decoder dead"` once it has exited (clean, error, or panic). Load
/// balancers and uptime monitors can therefore detect a wedged process
/// without needing to scrape the snapshot for staleness heuristics.
async fn healthz(State(state): State<AppState>) -> Response {
    use std::sync::atomic::Ordering;
    if state.decoder_alive.load(Ordering::Acquire) {
        (axum::http::StatusCode::OK, "ok").into_response()
    } else {
        (axum::http::StatusCode::SERVICE_UNAVAILABLE, "decoder dead").into_response()
    }
}

async fn list_aircraft(State(state): State<AppState>) -> Json<Vec<AircraftSnapshot>> {
    // Brief read under std::sync::RwLock: clone the snapshot vector and
    // release. Holding a std RwLock across .await would be a bug, so we
    // never .await while the guard is live.
    let mut out: Vec<AircraftSnapshot> = {
        let map = state.snapshot.read().expect("snapshot lock poisoned");
        map.values().cloned().collect()
    };
    out.sort_by_key(|a| a.icao);
    Json(out)
}

async fn get_aircraft(State(state): State<AppState>, Path(icao_hex): Path<String>) -> Response {
    let Some(icao) = Icao::from_hex(&icao_hex) else {
        return (
            axum::http::StatusCode::BAD_REQUEST,
            "icao must be 6 hex digits",
        )
            .into_response();
    };
    let snap = {
        let map = state.snapshot.read().expect("snapshot lock poisoned");
        map.get(&icao).cloned()
    };
    match snap {
        Some(a) => Json(a).into_response(),
        None => (axum::http::StatusCode::NOT_FOUND, "unknown ICAO").into_response(),
    }
}

#[derive(Debug, Deserialize, Default)]
struct StreamParams {
    /// Optional comma-separated event types to subscribe to.
    /// e.g. `?type=position,velocity`.
    #[serde(rename = "type")]
    r#type: Option<String>,
    /// Optional comma-separated ICAO addresses (hex, no separators).
    icao: Option<String>,
    /// Geographic bounding box: `min_lat,min_lon,max_lat,max_lon` in
    /// decimal degrees. Events are passed only when the aircraft's
    /// current known position falls inside this box. Aircraft without
    /// a resolved position are filtered out for the lifetime of this
    /// subscription, even on their non-position events (acquired,
    /// velocity, etc.) — that's the price of a meaningful bbox.
    bbox: Option<String>,
    /// Lower altitude bound in feet, inclusive. Aircraft below this
    /// altitude (or with no known altitude) are filtered out.
    alt_min: Option<i32>,
    /// Upper altitude bound in feet, inclusive.
    alt_max: Option<i32>,
}

/// Parsed bounding box in `(min_lat, min_lon, max_lat, max_lon)` order.
#[derive(Debug, Clone, Copy)]
struct Bbox {
    min_lat: f64,
    min_lon: f64,
    max_lat: f64,
    max_lon: f64,
}

impl Bbox {
    /// Parse from `"min_lat,min_lon,max_lat,max_lon"`. Returns `None`
    /// for malformed input, out-of-range coordinates, or boxes where
    /// `max <= min` on either axis. Antimeridian-wrapping boxes
    /// (e.g. spanning ±180°) are deliberately not supported in v0.1.
    fn parse(s: &str) -> Option<Self> {
        let mut parts = s.split(',');
        let min_lat: f64 = parts.next()?.trim().parse().ok()?;
        let min_lon: f64 = parts.next()?.trim().parse().ok()?;
        let max_lat: f64 = parts.next()?.trim().parse().ok()?;
        let max_lon: f64 = parts.next()?.trim().parse().ok()?;
        if parts.next().is_some() {
            return None;
        }
        if !(-90.0..=90.0).contains(&min_lat) || !(-90.0..=90.0).contains(&max_lat) {
            return None;
        }
        if !(-180.0..=180.0).contains(&min_lon) || !(-180.0..=180.0).contains(&max_lon) {
            return None;
        }
        if max_lat <= min_lat || max_lon <= min_lon {
            return None;
        }
        Some(Self {
            min_lat,
            min_lon,
            max_lat,
            max_lon,
        })
    }

    fn contains(&self, lat: f64, lon: f64) -> bool {
        lat >= self.min_lat && lat <= self.max_lat && lon >= self.min_lon && lon <= self.max_lon
    }
}

async fn stream(
    State(state): State<AppState>,
    Query(params): Query<StreamParams>,
    headers: HeaderMap,
) -> Sse<impl Stream<Item = Result<SseEvent, Infallible>>> {
    // Subscribe before doing any filtering work so we don't miss events
    // emitted between filter setup and the receiver attaching.
    let rx = state.broadcaster.subscribe();
    let stream = BroadcastStream::new(rx);

    let type_filter: Option<HashSet<String>> = params
        .r#type
        .as_deref()
        .map(|s| s.split(',').map(|t| t.trim().to_lowercase()).collect());

    let icao_filter: Option<HashSet<Icao>> = params.icao.as_deref().map(|s| {
        s.split(',')
            .filter_map(|tok| Icao::from_hex(tok.trim()))
            .collect()
    });

    let bbox_filter: Option<Bbox> = params.bbox.as_deref().and_then(Bbox::parse);
    let alt_min = params.alt_min;
    let alt_max = params.alt_max;
    // We need the snapshot inside the filter closure to resolve an
    // aircraft's current position for bbox/altitude checks on *any*
    // event type, not just `position`. Clone the Arc once.
    let snapshot = state.snapshot.clone();
    let needs_aircraft_lookup = bbox_filter.is_some() || alt_min.is_some() || alt_max.is_some();

    // Last-Event-ID: the client may include this header to resume from a
    // specific event id after reconnect. We don't yet maintain a replay
    // buffer (that lands once we observe a real-world reconnect pattern),
    // but we honour the header by advancing past `id <= last_event_id`
    // in the live stream when it shows up. Clients that don't set it
    // get all live events from the moment they connect.
    let last_event_id: Option<u64> = headers
        .get("last-event-id")
        .and_then(|v| v.to_str().ok())
        .and_then(|s| s.parse().ok());

    let filtered = stream.filter_map(move |item| {
        let env = item.ok()?;
        if let Some(min) = last_event_id {
            if env.id <= min {
                return None;
            }
        }
        if let Some(types) = &type_filter {
            if !types.contains(env.event.tag()) {
                return None;
            }
        }
        if let Some(icaos) = &icao_filter {
            match env.event.icao() {
                Some(i) if icaos.contains(&i) => {}
                _ => return None,
            }
        }
        // bbox + altitude apply to the *aircraft* the event refers to,
        // not the event itself: a velocity event for an aircraft whose
        // last position is inside the box passes, even though the
        // event payload carries no lat/lon. Single snapshot read per
        // event when any geo/alt filter is active.
        if needs_aircraft_lookup {
            let icao = env.event.icao()?;
            let snap = snapshot.read().expect("snapshot lock poisoned");
            let aircraft = snap.get(&icao)?;
            let position = aircraft.position.as_ref()?;
            if let Some(bbox) = &bbox_filter {
                if !bbox.contains(position.lat, position.lon) {
                    return None;
                }
            }
            if alt_min.is_some() || alt_max.is_some() {
                let alt = position.alt_ft?;
                if let Some(lo) = alt_min {
                    if alt < lo {
                        return None;
                    }
                }
                if let Some(hi) = alt_max {
                    if alt > hi {
                        return None;
                    }
                }
            }
        }
        Some(envelope_to_sse(env))
    });

    Sse::new(filtered).keep_alive(
        KeepAlive::new()
            .interval(Duration::from_secs(15))
            .text("heartbeat"),
    )
}

/// Convert a broadcast `EventEnvelope` to an SSE wire event.
///
/// Serialization can't fail in practice (we own the types and they're all
/// Serialize), so we collapse errors into `Infallible` by `expect`ing.
/// The `Result<_, Infallible>` return is the type axum's SSE stream
/// requires; we own it from end to end.
#[allow(clippy::unnecessary_wraps, clippy::needless_pass_by_value)]
fn envelope_to_sse(env: EventEnvelope) -> Result<SseEvent, Infallible> {
    let data = serde_json::to_string(&env.event)
        .expect("Event serialization is infallible for our own types");
    Ok(SseEvent::default()
        .id(env.id.to_string())
        .event(env.event.tag())
        .data(data))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bbox_parses_canonical_form() {
        let b = Bbox::parse("40.0,-74.0,41.0,-73.0").expect("valid");
        assert!((b.min_lat - 40.0).abs() < 1e-9);
        assert!((b.min_lon - -74.0).abs() < 1e-9);
        assert!((b.max_lat - 41.0).abs() < 1e-9);
        assert!((b.max_lon - -73.0).abs() < 1e-9);
    }

    #[test]
    fn bbox_rejects_malformed_input() {
        assert!(Bbox::parse("").is_none());
        assert!(Bbox::parse("40,-74,41").is_none()); // too few fields
        assert!(Bbox::parse("40,-74,41,-73,99").is_none()); // too many
        assert!(Bbox::parse("nope,-74,41,-73").is_none());
    }

    #[test]
    fn bbox_rejects_out_of_range_coords() {
        assert!(Bbox::parse("91.0,-74.0,92.0,-73.0").is_none()); // lat > 90
        assert!(Bbox::parse("40.0,-181.0,41.0,-180.5").is_none()); // lon < -180
        assert!(Bbox::parse("40.0,-74.0,40.0,-73.0").is_none()); // degenerate lat
        assert!(Bbox::parse("40.0,-74.0,41.0,-74.0").is_none()); // degenerate lon
        assert!(Bbox::parse("41.0,-74.0,40.0,-73.0").is_none()); // inverted lat
    }

    #[test]
    fn bbox_contains_handles_boundary() {
        let b = Bbox::parse("40.0,-74.0,41.0,-73.0").unwrap();
        // Interior.
        assert!(b.contains(40.5, -73.5));
        // Edges are inclusive.
        assert!(b.contains(40.0, -74.0));
        assert!(b.contains(41.0, -73.0));
        // Outside.
        assert!(!b.contains(39.9, -73.5));
        assert!(!b.contains(40.5, -74.1));
        assert!(!b.contains(41.1, -73.5));
        assert!(!b.contains(40.5, -72.9));
    }
}

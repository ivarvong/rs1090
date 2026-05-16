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
    (
        [("content-type", "text/html; charset=utf-8")],
        INDEX_HTML,
    )
        .into_response()
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
        (
            axum::http::StatusCode::SERVICE_UNAVAILABLE,
            "decoder dead",
        )
            .into_response()
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

async fn get_aircraft(
    State(state): State<AppState>,
    Path(icao_hex): Path<String>,
) -> Response {
    let Some(icao) = Icao::from_hex(&icao_hex) else {
        return (axum::http::StatusCode::BAD_REQUEST, "icao must be 6 hex digits").into_response();
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


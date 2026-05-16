//! `rs1090-serve`: HTTP/SSE server.
//!
//! Per DESIGN.md §12.6, this is the only place `tokio` lives. The decoder
//! runs in a sync OS thread that owns the SDR; it `try_send`s events into
//! a `tokio::sync::broadcast` channel, and never awaits anything. The tokio
//! runtime hosts axum and the SSE connections.

mod avr;
mod beast;
mod broadcaster;
mod events;
mod gdl90;
mod metrics;
mod server;

#[cfg(all(target_os = "linux", feature = "ble"))]
mod ble;

use std::fs::File;
use std::io::BufReader;
use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};

use rs1090::cpr::LatLon;
use rs1090::crc::CrcOutcome;
use rs1090::frame::FrameDetector;
use rs1090::source::{IqFileSource, SampleSource};
use rs1090::state::{StateEvent, StateTracker};
use rs1090::Iq;

use crate::broadcaster::{snapshot_from, AppState};
use crate::events::{from_state_event, EventEnvelope};

#[derive(Parser, Debug)]
#[command(name = "rs1090-serve", version, about = "HTTP/SSE for rs1090")]
// Clap's argument struct naturally accumulates booleans as we add
// optional outputs (BLE, GDL90, AVR, Beast, …). The struct-of-bools
// shape *is* the desired shape — clippy's heuristic doesn't apply.
#[allow(clippy::struct_excessive_bools)]
struct Cli {
    /// Address to bind. Defaults to loopback for safety.
    #[arg(long, default_value = "127.0.0.1:8080")]
    bind: String,

    /// Receiver reference position for local CPR fallback (`lat,lon`).
    #[arg(long, value_parser = parse_latlon)]
    reference: Option<LatLon>,

    /// Drop frames whose aggregate per-bit confidence is below this.
    #[arg(long, default_value_t = 0)]
    min_confidence: u8,

    /// Also expose a BLE GATT peripheral so iPhone / Android BLE
    /// debug apps (nRF Connect) can subscribe to live aircraft data
    /// over Bluetooth. Linux + the `ble` build feature only — silently
    /// rejected on other targets to keep cross-platform builds clean.
    #[arg(long)]
    ble: bool,

    /// Broadcast GDL90 traffic reports over UDP for EFB apps
    /// (`ForeFlight`, Garmin Pilot, `FlyQ`). Pass `--gdl90` for the
    /// LAN-broadcast default of `255.255.255.255:4000`, or
    /// `--gdl90-target IP:PORT` for unicast (e.g. an iPad over
    /// Tailscale).
    #[arg(long)]
    gdl90: bool,

    /// Override the GDL90 UDP destination. Implies `--gdl90`.
    #[arg(long, value_parser = clap::value_parser!(std::net::SocketAddr))]
    gdl90_target: Option<std::net::SocketAddr>,

    /// Serve AVR-text (dump1090 `--raw` shape) over TCP at
    /// `0.0.0.0:30002`. Override the address with `--avr-bind`.
    #[arg(long)]
    avr: bool,

    /// Override the AVR TCP bind address. Implies `--avr`.
    #[arg(long, value_parser = clap::value_parser!(std::net::SocketAddr))]
    avr_bind: Option<std::net::SocketAddr>,

    /// Serve Beast binary (dump1090-fa `--net-bo-port`) over TCP at
    /// `0.0.0.0:30005`. Override with `--beast-bind`.
    #[arg(long)]
    beast: bool,

    /// Override the Beast TCP bind address. Implies `--beast`.
    #[arg(long, value_parser = clap::value_parser!(std::net::SocketAddr))]
    beast_bind: Option<std::net::SocketAddr>,

    #[command(subcommand)]
    source: Source,
}

#[derive(Subcommand, Debug)]
enum Source {
    /// Replay an `.iq` file (signed-8-bit interleaved I/Q).
    File {
        path: PathBuf,
        #[arg(long, default_value_t = 2_000_000)]
        sample_rate: u32,
        #[arg(long, default_value_t = 1_090_000_000)]
        center_freq: u32,
        /// Decode at wall-clock speed instead of as-fast-as-possible. Useful
        /// for demos where you want the SSE stream to "play" at the real
        /// rate the samples were captured at.
        #[arg(long)]
        realtime: bool,
    },
    /// Live RTL-SDR dongle.
    #[cfg(feature = "rtl-sdr")]
    Live {
        #[arg(long, default_value_t = 0)]
        device: usize,
        #[arg(long, default_value_t = 400)]
        gain_tenth_db: i32,
        #[arg(long)]
        auto_gain: bool,
        #[arg(long)]
        bias_t: bool,
    },
}

fn parse_latlon(s: &str) -> std::result::Result<LatLon, String> {
    let (lat, lon) = s
        .split_once(',')
        .ok_or_else(|| format!("expected `lat,lon`, got `{s}`"))?;
    let lat_deg: f64 = lat
        .trim()
        .parse()
        .map_err(|e| format!("bad latitude: {e}"))?;
    let lon_deg: f64 = lon
        .trim()
        .parse()
        .map_err(|e| format!("bad longitude: {e}"))?;
    if !(-90.0..=90.0).contains(&lat_deg) || !(-180.0..=180.0).contains(&lon_deg) {
        return Err(format!("out of range: {lat_deg},{lon_deg}"));
    }
    Ok(LatLon { lat_deg, lon_deg })
}

// `main` is the wiring point — CLI parse, sub-system spawn, runtime
// setup, shutdown handling. Splitting it for the line-count lint would
// fragment the boot story across helpers and obscure it.
#[allow(clippy::too_many_lines)]
fn main() -> Result<()> {
    init_tracing();
    let metrics_handle = metrics::install();
    metrics::describe_all();
    ::metrics::gauge!(metrics::DECODER_ALIVE).set(1.0);

    let cli = Cli::parse();
    let state = AppState::new(metrics_handle);
    let bind = cli.bind.clone();
    let ble_requested = cli.ble;
    let gdl90_target = match (cli.gdl90, cli.gdl90_target) {
        (_, Some(addr)) => Some(addr),
        (true, None) => Some(gdl90::DEFAULT_TARGET),
        (false, None) => None,
    };
    let avr_bind = match (cli.avr, cli.avr_bind) {
        (_, Some(addr)) => Some(addr),
        (true, None) => Some(SocketAddr::from(([0, 0, 0, 0], avr::DEFAULT_PORT))),
        (false, None) => None,
    };
    let beast_bind = match (cli.beast, cli.beast_bind) {
        (_, Some(addr)) => Some(addr),
        (true, None) => Some(SocketAddr::from(([0, 0, 0, 0], beast::DEFAULT_PORT))),
        (false, None) => None,
    };

    // Warn loudly when bound publicly per DESIGN.md §12.8.
    if !bind.starts_with("127.0.0.1")
        && !bind.starts_with("localhost")
        && !bind.starts_with("[::1]")
    {
        tracing::warn!(
            %bind,
            "binding to a non-loopback address with no auth configured — front with a reverse proxy for TLS + auth",
        );
    }

    // Spawn the decoder in a sync OS thread (per DESIGN.md §12.6) so it
    // never awaits anything. It pushes into the broadcaster from there.
    //
    // `decoder_alive` is wired via a disarm-able Drop guard:
    //   - Decoder returns `Err(_)`  → guard fires → flag goes false.
    //   - Decoder panics            → guard fires on unwind → flag false.
    //   - Decoder returns `Ok(())`  → guard disarmed → flag stays true.
    //     File replay finishing normally is not a failure; the server
    //     keeps serving the final snapshot until Ctrl-C, and `/healthz`
    //     stays green.
    let decoder_state = state.clone();
    let decoder_args = cli;
    std::thread::Builder::new()
        .name("decoder".into())
        .spawn(move || {
            struct LivenessGuard {
                flag: Arc<std::sync::atomic::AtomicBool>,
                died: Arc<tokio::sync::Notify>,
                armed: bool,
            }
            impl Drop for LivenessGuard {
                fn drop(&mut self) {
                    if self.armed {
                        self.flag.store(false, std::sync::atomic::Ordering::Release);
                        ::metrics::gauge!(crate::metrics::DECODER_ALIVE).set(0.0);
                        // Wake the tokio shutdown task — the server should
                        // exit too so systemd's `Restart=on-failure` brings
                        // the whole process back rather than leaving us
                        // serving a frozen snapshot indefinitely.
                        self.died.notify_one();
                    }
                }
            }
            let mut guard = LivenessGuard {
                flag: decoder_state.decoder_alive.clone(),
                died: decoder_state.decoder_died.clone(),
                armed: true,
            };
            match run_decoder(decoder_args, decoder_state.clone()) {
                Ok(()) => guard.armed = false,
                Err(e) => tracing::error!(error = ?e, "decoder error"),
            }
        })
        .context("spawning decoder thread")?;

    // Tokio runtime for axum + broadcaster. 2 worker threads is plenty;
    // the SSE workload is mostly I/O.
    let rt = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()
        .context("building tokio runtime")?;

    let server_state = state.clone();
    rt.block_on(async move {
        let listener = tokio::net::TcpListener::bind(&bind)
            .await
            .with_context(|| format!("binding {bind}"))?;
        tracing::info!(%bind, "HTTP server listening");

        // Optional BLE peripheral. Runs in its own task; errors are
        // logged and the rest of the server keeps going. Linux + `ble`
        // feature only — on every other build target, the flag is
        // silently accepted and the spawn is a no-op.
        if ble_requested {
            spawn_ble(server_state.clone());
        }

        // Optional GDL90 broadcaster. Spawns a tokio task that pushes
        // a heartbeat + one traffic report per known aircraft every
        // second to the configured UDP target. Always available
        // (no feature gate; pure UDP, no platform deps).
        if let Some(target) = gdl90_target {
            let st = server_state.clone();
            tokio::spawn(async move {
                if let Err(e) = gdl90::run(st, target).await {
                    tracing::error!(error = ?e, "GDL90 broadcaster exited");
                }
            });
        }

        // Optional AVR-text / Beast TCP listeners. Each subscribes to
        // the per-frame broadcast inside its own client tasks; multiple
        // consumers per protocol are fine.
        if let Some(bind) = avr_bind {
            let st = server_state.clone();
            tokio::spawn(async move {
                if let Err(e) = avr::run(st, bind).await {
                    tracing::error!(error = ?e, "AVR listener exited");
                }
            });
        }
        if let Some(bind) = beast_bind {
            let st = server_state.clone();
            tokio::spawn(async move {
                if let Err(e) = beast::run(st, bind).await {
                    tracing::error!(error = ?e, "Beast listener exited");
                }
            });
        }

        let shutdown_state = server_state.clone();
        axum::serve(listener, server::router(server_state))
            .with_graceful_shutdown(shutdown_signal(shutdown_state))
            .await
            .context("serving")
    })?;

    // If the decoder died (not a clean file-replay exit), exit non-zero
    // so systemd's `Restart=on-failure` (or Kubernetes restartPolicy,
    // or any other supervisor) brings the process back. Clean exits
    // (Ctrl-C, file replay finishing) return zero.
    if !state
        .decoder_alive
        .load(std::sync::atomic::Ordering::Acquire)
    {
        tracing::error!("exiting non-zero due to decoder failure");
        std::process::exit(1);
    }
    Ok(())
}

async fn shutdown_signal(state: AppState) {
    tokio::select! {
        _ = tokio::signal::ctrl_c() => tracing::info!("Ctrl-C received, shutting down"),
        () = state.decoder_died.notified() => {
            tracing::error!("decoder died, shutting down HTTP server");
        }
    }
}

/// Wire up tracing-subscriber. `RUST_LOG` controls verbosity in the
/// usual way (`info`, `info,rs1090_serve=debug`, etc.). Default if
/// unset: `info` for our own crates, `warn` for everything else —
/// keeps Tokio + Hyper noise out of operator logs.
fn init_tracing() {
    use tracing_subscriber::{fmt, EnvFilter};
    let filter = EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| EnvFilter::new("warn,rs1090_serve=info,rs1090=info"));
    fmt().with_env_filter(filter).with_target(false).init();
}

/// Linux + `ble` feature: spawn the BLE peripheral task. Any other
/// target: print a notice and continue — `--ble` is intentionally
/// not gated by `#[cfg]` so cross-platform builds accept the same
/// flag set, they just can't honour it.
#[cfg(all(target_os = "linux", feature = "ble"))]
fn spawn_ble(state: AppState) {
    tokio::spawn(async move {
        if let Err(e) = ble::run(state).await {
            tracing::error!(error = ?e, "BLE peripheral exited");
        }
    });
}

#[cfg(not(all(target_os = "linux", feature = "ble")))]
fn spawn_ble(_state: AppState) {
    tracing::warn!(
        "--ble was set but this build has no BLE support (Linux + `ble` feature required)",
    );
}

/// Synchronous decoder loop, run on a dedicated OS thread.
///
/// Both args are passed by value because this is the function the worker
/// thread takes ownership of; clippy's "needless pass by value" warns on
/// `cli` and `state` but moving them is the point.
#[allow(clippy::needless_pass_by_value)]
fn run_decoder(cli: Cli, state: AppState) -> Result<()> {
    let next_id = Arc::new(AtomicU64::new(1));

    // Open the configured source.
    let mut detector = FrameDetector::new();
    detector.set_min_confidence(cli.min_confidence);
    let mut tracker = StateTracker::new();
    if let Some(r) = cli.reference {
        tracker.set_reference(r);
    }

    let mut events_buf: Vec<StateEvent> = Vec::with_capacity(32);
    let mut iq_buf = vec![Iq::default(); 65_536];

    let mut source_kind = SourceKind::open(&cli.source)?;
    let sample_rate = source_kind.sample_rate();
    let realtime = matches!(&cli.source, Source::File { realtime: true, .. });

    let t0 = Instant::now();
    let mut samples_consumed: u64 = 0;
    let mut frames: u64 = 0;

    loop {
        let n = source_kind.read(&mut iq_buf)?;
        if n == 0 {
            break;
        }
        samples_consumed += n as u64;

        // For file replay with --realtime, pace the read loop so the SSE
        // stream emits at the rate the samples were captured at.
        if realtime {
            let wall_target =
                Duration::from_nanos(samples_consumed * 1_000_000_000 / u64::from(sample_rate));
            let wall_actual = t0.elapsed();
            if let Some(sleep_for) = wall_target.checked_sub(wall_actual) {
                std::thread::sleep(sleep_for);
            }
        }

        let virtual_now =
            t0 + Duration::from_nanos(samples_consumed * 1_000_000_000 / u64::from(sample_rate));

        let tx = state.broadcaster.clone();
        let frame_tx = state.frame_broadcaster.clone();
        let snapshot = state.snapshot.clone();
        let next_id = next_id.clone();

        detector.process(&iq_buf[..n], |frame| {
            frames += 1;
            ::metrics::counter!(
                crate::metrics::FRAMES_TOTAL,
                "outcome" => crc_outcome_label(frame.crc_outcome()),
            )
            .increment(1);
            // Fan the raw Frame out to AVR / Beast / etc. before any
            // decode pass. Errors only fire when no consumer is
            // subscribed; drop silently. `Frame: Copy`, so this is
            // free.
            let _ = frame_tx.send(*frame);
            tracker.ingest(frame, virtual_now, &mut events_buf);
            for ev in events_buf.drain(..) {
                ::metrics::counter!(
                    crate::metrics::STATE_EVENTS_TOTAL,
                    "kind" => state_event_label(&ev),
                )
                .increment(1);
                // Refresh the shared snapshot keyed on the affected ICAO.
                update_snapshot(&snapshot, &tracker, &ev);
                // Convert and broadcast. Orphan events are dropped at this
                // boundary so they don't reach SSE consumers.
                let now_iso = rfc3339_now();
                if let Some(wire) = from_state_event(&ev, &now_iso) {
                    let env = EventEnvelope {
                        id: next_id.fetch_add(1, Ordering::Relaxed),
                        event: wire,
                    };
                    // `send` errors only when there are zero receivers, which
                    // means no clients are connected. Drop silently.
                    let _ = tx.send(env);
                }
            }
            #[allow(clippy::cast_precision_loss)]
            ::metrics::gauge!(crate::metrics::AIRCRAFT_TRACKED).set(tracker.len() as f64);
        });
    }

    tracing::info!(
        frames,
        aircraft = tracker.len(),
        elapsed_s = t0.elapsed().as_secs_f64(),
        "decoder exhausted",
    );
    Ok(())
}

/// Refresh the shared snapshot in response to a state-tracker event.
///
/// We always overwrite the entry for the affected ICAO from the tracker's
/// current state, except on `Lost` (where we delete) and `Orphan` (no-op).
/// The lock is held briefly — a single `HashMap` insert/remove — so HTTP
/// readers that wait on it never block for measurable time.
fn update_snapshot(
    snapshot: &Arc<
        std::sync::RwLock<
            std::collections::HashMap<rs1090::message::Icao, events::AircraftSnapshot>,
        >,
    >,
    tracker: &StateTracker,
    ev: &StateEvent,
) {
    let icao = match ev {
        StateEvent::Acquired(i) | StateEvent::Lost(i) => *i,
        StateEvent::Identification { icao, .. }
        | StateEvent::Position { icao, .. }
        | StateEvent::Velocity { icao, .. }
        | StateEvent::AddressRecovered { icao, .. } => *icao,
        StateEvent::Orphan { .. } => return,
    };
    let mut map = snapshot.write().expect("snapshot lock poisoned");
    if matches!(ev, StateEvent::Lost(_)) {
        map.remove(&icao);
    } else if let Some(a) = tracker.get(icao) {
        map.insert(icao, snapshot_from(a));
    }
}

/// Low-cardinality label for a frame's CRC outcome. We deliberately
/// fold `Corrected { bit }` down to the variant name — the per-bit
/// counter would explode the time-series count for no operational gain.
fn crc_outcome_label(outcome: CrcOutcome) -> &'static str {
    match outcome {
        CrcOutcome::Clean => "clean",
        CrcOutcome::Corrected { .. } => "corrected",
        CrcOutcome::Failed => "failed",
    }
}

/// Low-cardinality label for a tracker event. Matches the SSE wire
/// `tag()` strings so a `STATE_EVENTS_TOTAL{kind="position"}` total
/// lines up one-to-one with what SSE clients see — except for
/// `orphan`, which is dropped at the wire boundary but still counted
/// here so operators can monitor unassociated-frame rates.
fn state_event_label(ev: &StateEvent) -> &'static str {
    match ev {
        StateEvent::Acquired(_) => "acquired",
        StateEvent::Identification { .. } => "identification",
        StateEvent::Position { .. } => "position",
        StateEvent::Velocity { .. } => "velocity",
        StateEvent::Lost(_) => "lost",
        StateEvent::AddressRecovered { .. } => "address_recovered",
        StateEvent::Orphan { .. } => "orphan",
    }
}

fn rfc3339_now() -> String {
    use time::format_description::well_known::Rfc3339;
    time::OffsetDateTime::now_utc()
        .format(&Rfc3339)
        .unwrap_or_default()
}

/// Type-erased sample source: file or live.
///
/// `IqFileSource` carries an internal 4 KiB scratch buffer (clippy flags the
/// resulting enum size). Both variants are constructed exactly once at
/// startup, so the size is a fixed cost, not a hot-path concern; we box
/// neither and quiet the lint.
#[allow(clippy::large_enum_variant)]
enum SourceKind {
    File(IqFileSource<BufReader<File>>),
    #[cfg(feature = "rtl-sdr")]
    Live(rs1090::source::RtlSdrSource),
}

impl SourceKind {
    fn open(src: &Source) -> Result<Self> {
        match src {
            Source::File {
                path,
                sample_rate,
                center_freq,
                realtime: _,
            } => {
                let f = File::open(path).with_context(|| format!("opening {}", path.display()))?;
                let s = IqFileSource::new(BufReader::new(f), *sample_rate, *center_freq);
                Ok(Self::File(s))
            }
            #[cfg(feature = "rtl-sdr")]
            Source::Live {
                device,
                gain_tenth_db,
                auto_gain,
                bias_t,
            } => {
                let mut b = rs1090::source::RtlSdrSourceBuilder::new()
                    .device_index(*device)
                    .bias_t(*bias_t);
                if *auto_gain {
                    b = b.auto_gain();
                } else {
                    b = b.gain_tenth_db(*gain_tenth_db);
                }
                let s = b.open().context("opening RTL-SDR")?;
                Ok(Self::Live(s))
            }
        }
    }

    fn sample_rate(&self) -> u32 {
        match self {
            Self::File(s) => s.sample_rate(),
            #[cfg(feature = "rtl-sdr")]
            Self::Live(s) => s.sample_rate(),
        }
    }

    fn read(&mut self, out: &mut [Iq]) -> Result<usize> {
        Ok(match self {
            Self::File(s) => s.read(out)?,
            #[cfg(feature = "rtl-sdr")]
            Self::Live(s) => s.read(out)?,
        })
    }
}

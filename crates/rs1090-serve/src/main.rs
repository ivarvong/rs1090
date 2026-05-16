//! `rs1090-serve`: HTTP/SSE server.
//!
//! Per DESIGN.md §12.6, this is the only place `tokio` lives. The decoder
//! runs in a sync OS thread that owns the SDR; it `try_send`s events into
//! a `tokio::sync::broadcast` channel, and never awaits anything. The tokio
//! runtime hosts axum and the SSE connections.

mod broadcaster;
mod events;
mod server;

use std::fs::File;
use std::io::BufReader;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};

use rs1090::cpr::LatLon;
use rs1090::frame::FrameDetector;
use rs1090::source::{IqFileSource, SampleSource};
use rs1090::state::{StateEvent, StateTracker};
use rs1090::Iq;

use crate::broadcaster::{snapshot_from, AppState};
use crate::events::{from_state_event, EventEnvelope};

#[derive(Parser, Debug)]
#[command(name = "rs1090-serve", version, about = "HTTP/SSE for rs1090")]
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
    let lat_deg: f64 = lat.trim().parse().map_err(|e| format!("bad latitude: {e}"))?;
    let lon_deg: f64 = lon.trim().parse().map_err(|e| format!("bad longitude: {e}"))?;
    if !(-90.0..=90.0).contains(&lat_deg) || !(-180.0..=180.0).contains(&lon_deg) {
        return Err(format!("out of range: {lat_deg},{lon_deg}"));
    }
    Ok(LatLon { lat_deg, lon_deg })
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let state = AppState::new();
    let bind = cli.bind.clone();

    // Warn loudly when bound publicly per DESIGN.md §12.8.
    if !bind.starts_with("127.0.0.1") && !bind.starts_with("localhost") && !bind.starts_with("[::1]") {
        eprintln!(
            "rs1090-serve: WARNING — binding to a non-loopback address ({bind}). \
             No auth is configured. Front with a reverse proxy for TLS + auth."
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
                armed: bool,
            }
            impl Drop for LivenessGuard {
                fn drop(&mut self) {
                    if self.armed {
                        self.flag
                            .store(false, std::sync::atomic::Ordering::Release);
                    }
                }
            }
            let mut guard = LivenessGuard {
                flag: decoder_state.decoder_alive.clone(),
                armed: true,
            };
            match run_decoder(decoder_args, decoder_state.clone()) {
                Ok(()) => guard.armed = false,
                Err(e) => eprintln!("rs1090-serve: decoder error: {e:#}"),
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

    rt.block_on(async move {
        let listener = tokio::net::TcpListener::bind(&bind).await.with_context(|| {
            format!("binding {bind}")
        })?;
        eprintln!("rs1090-serve: listening on {bind}");
        eprintln!("    curl http://{bind}/healthz");
        eprintln!("    curl http://{bind}/aircraft");
        eprintln!("    curl -N http://{bind}/stream");
        axum::serve(listener, server::router(state))
            .with_graceful_shutdown(shutdown_signal())
            .await
            .context("serving")
    })
}

async fn shutdown_signal() {
    let _ = tokio::signal::ctrl_c().await;
    eprintln!("rs1090-serve: shutting down");
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
            let wall_target = Duration::from_nanos(
                samples_consumed * 1_000_000_000 / u64::from(sample_rate),
            );
            let wall_actual = t0.elapsed();
            if let Some(sleep_for) = wall_target.checked_sub(wall_actual) {
                std::thread::sleep(sleep_for);
            }
        }

        let virtual_now = t0
            + Duration::from_nanos(
                samples_consumed * 1_000_000_000 / u64::from(sample_rate),
            );

        let tx = state.broadcaster.clone();
        let snapshot = state.snapshot.clone();
        let next_id = next_id.clone();

        detector.process(&iq_buf[..n], |frame| {
            frames += 1;
            tracker.ingest(frame, virtual_now, &mut events_buf);
            for ev in events_buf.drain(..) {
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
        });
    }

    eprintln!(
        "rs1090-serve: decoder exhausted ({frames} frames, {} aircraft, {:.2}s)",
        tracker.len(),
        t0.elapsed().as_secs_f64()
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

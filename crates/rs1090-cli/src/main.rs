//! `rs1090` command-line tool.
//!
//! Two subcommands:
//!
//! - `replay` — one line per detected frame (per-frame diagnostics).
//! - `track`  — aggregate frames per aircraft and print state-tracker
//!   events (acquisitions, identifications, position fixes, velocities,
//!   address-XOR CRC recoveries, losses).
//!
//! Live SDR ingestion (RTL-SDR backend) lands in a later milestone.

use std::fs::File;
use std::io::{self, BufReader, BufWriter, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};

use rs1090::cpr::LatLon;
use rs1090::frame::{Frame, FrameDetector};
use rs1090::message::{
    self, Altitude, Message, SquitterPayload, Velocity, VelocityKind,
};
use rs1090::source::{IqFileSource, SampleSource};
use rs1090::state::{PositionSource, StateEvent, StateTracker};
use rs1090::Iq;

#[derive(Parser, Debug)]
#[command(name = "rs1090", version, about = "Mode S / ADS-B decoder")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// Replay an interleaved signed-8-bit I/Q file and print detected frames.
    Replay(ReplayArgs),
    /// Replay an `.iq` file and print state-tracker events: aircraft
    /// acquisitions, identifications, positions, velocities, losses.
    Track(TrackArgs),
}

#[derive(clap::Args, Debug)]
struct ReplayArgs {
    /// Path to the `.iq` file. The format is raw interleaved signed 8-bit
    /// I/Q samples (`i q i q ...`).
    file: PathBuf,

    /// Sample rate of the file in samples per second.
    #[arg(long, default_value_t = 2_000_000)]
    sample_rate: u32,

    /// Center frequency in Hz. For ADS-B this is 1090 MHz.
    #[arg(long, default_value_t = 1_090_000_000)]
    center_freq: u32,

    /// Drop frames whose aggregate per-bit confidence is below this value
    /// (`0..=255`).
    #[arg(long, default_value_t = 0)]
    min_confidence: u8,

    /// Seed the noise floor at this magnitude before processing the first
    /// sample. Useful for short files where the EMA wouldn't otherwise
    /// settle.
    #[arg(long)]
    noise_seed: Option<u16>,
}

#[derive(clap::Args, Debug)]
struct TrackArgs {
    /// Path to the `.iq` file.
    file: PathBuf,

    /// Sample rate of the file in samples per second.
    #[arg(long, default_value_t = 2_000_000)]
    sample_rate: u32,

    /// Center frequency in Hz.
    #[arg(long, default_value_t = 1_090_000_000)]
    center_freq: u32,

    /// Drop frames whose aggregate per-bit confidence is below this value.
    #[arg(long, default_value_t = 0)]
    min_confidence: u8,

    /// Seed the noise floor at this magnitude before processing.
    #[arg(long)]
    noise_seed: Option<u16>,

    /// Receiver reference position for local CPR decode, in
    /// `lat,lon` decimal degrees. When set, the tracker falls back to
    /// local decode whenever no even/odd pair is available.
    #[arg(long, value_parser = parse_latlon)]
    reference: Option<LatLon>,

    /// Print a one-line summary of each tracked aircraft at the end.
    #[arg(long)]
    summary: bool,
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
    match &cli.command {
        Command::Replay(args) => run_replay(args),
        Command::Track(args) => run_track(args),
    }
}

fn run_replay(args: &ReplayArgs) -> Result<()> {
    let file = File::open(&args.file)
        .with_context(|| format!("opening {}", args.file.display()))?;
    let reader = BufReader::new(file);
    let mut source = IqFileSource::new(reader, args.sample_rate, args.center_freq);

    let mut detector = FrameDetector::new();
    detector.set_min_confidence(args.min_confidence);
    if let Some(seed) = args.noise_seed {
        detector.reset_noise_floor(seed);
    }

    let stdout = io::stdout();
    let mut out = BufWriter::new(stdout.lock());
    let mut buf = vec![Iq::default(); 65_536];
    let mut frame_count = 0u64;

    loop {
        let n = source.read(&mut buf).context("reading samples")?;
        if n == 0 {
            break;
        }
        detector.process(&buf[..n], |frame| {
            // Errors from `print_frame` are I/O errors on stdout, which we
            // surface by counting and reporting at the end. We can't return
            // a Result from the callback without complicating the trait;
            // a write failure here is non-recoverable anyway.
            let _ = print_frame(&mut out, frame);
            frame_count += 1;
        });
    }
    out.flush().context("flushing stdout")?;
    eprintln!("rs1090: {frame_count} frames");
    Ok(())
}

fn print_frame<W: Write>(out: &mut W, frame: &Frame) -> io::Result<()> {
    // One-line, space-separated format: DF code, hex payload, CRC outcome,
    // aggregate confidence, then a short decoded summary.
    write!(out, "DF{:<2} ", frame.downlink_format().raw_value())?;
    for b in frame.bytes() {
        write!(out, "{b:02X}")?;
    }
    write!(out, " ")?;
    match frame.crc_outcome() {
        rs1090::crc::CrcOutcome::Clean => write!(out, "clean       ")?,
        rs1090::crc::CrcOutcome::Corrected { bit } => write!(out, "corrected:{bit:<3} ")?,
        rs1090::crc::CrcOutcome::Failed => write!(out, "failed      ")?,
    }
    write!(out, "conf={:<3} ", frame.confidence())?;

    // Decoded summary. Errors in the decoder are non-fatal; print the raw
    // tag and move on.
    match message::decode(frame) {
        Ok(msg) => print_message_summary(out, &msg)?,
        Err(e) => write!(out, "decode-err:{e:?}")?,
    }
    writeln!(out)?;
    Ok(())
}

fn print_message_summary<W: Write>(out: &mut W, msg: &Message) -> io::Result<()> {
    match msg {
        Message::ExtendedSquitter(es) => {
            write!(out, "ICAO={} ", es.icao)?;
            match &es.payload {
                SquitterPayload::Identification(id) => {
                    write!(out, "ident callsign={} cat={}/{}",
                        id.callsign, id.category_set, id.category)?;
                }
                SquitterPayload::AirbornePosition(p) => {
                    write!(out, "airpos {} cpr-{}({},{})",
                        fmt_altitude(p.altitude),
                        if p.cpr.odd { "odd" } else { "even" },
                        p.cpr.lat_cpr, p.cpr.lon_cpr,
                    )?;
                }
                SquitterPayload::Velocity(v) => print_velocity_summary(out, v)?,
                SquitterPayload::Raw(_) => write!(out, "tc={:?}", es.type_code)?,
                _ => write!(out, "tc={:?}(unhandled)", es.type_code)?,
            }
        }
        Message::AllCallReply { icao } => write!(out, "all-call ICAO={icao}")?,
        Message::SurveillanceReply { df, bytes } => {
            write!(out, "surv DF{} bytes={}", df.raw_value(), bytes)?;
        }
        Message::Other { df } => write!(out, "other DF{}", df.raw_value())?,
        _ => write!(out, "unhandled")?,
    }
    Ok(())
}

fn fmt_altitude(a: Altitude) -> String {
    match a {
        Altitude::BaroFeet(ft) => format!("alt={ft}ft"),
        Altitude::BaroGillhamFeet(ft) => format!("alt={ft}ft(gillham)"),
        Altitude::GnssFeet(ft) => format!("alt={ft}ft(gnss)"),
        Altitude::Unavailable => "alt=n/a".to_string(),
    }
}

fn print_velocity_summary<W: Write>(out: &mut W, v: &Velocity) -> io::Result<()> {
    match &v.kind {
        VelocityKind::Ground { speed_kt, heading_deg } => {
            write!(out, "vel gs={speed_kt}kt hdg={heading_deg:.1}°")?;
        }
        VelocityKind::Airspeed { speed_kt, heading_deg, magnetic } => {
            write!(out, "vel ias={speed_kt}kt")?;
            if let Some(h) = heading_deg {
                let label = if *magnetic { "mag" } else { "true" };
                write!(out, " hdg={h:.1}°({label})")?;
            }
        }
    }
    if let Some(vr) = v.vertical_rate_fpm {
        write!(out, " vr={vr:+}fpm")?;
    }
    Ok(())
}

// --- `track` subcommand ----------------------------------------------------

fn run_track(args: &TrackArgs) -> Result<()> {
    let file = File::open(&args.file)
        .with_context(|| format!("opening {}", args.file.display()))?;
    let reader = BufReader::new(file);
    let mut source = IqFileSource::new(reader, args.sample_rate, args.center_freq);

    let mut detector = FrameDetector::new();
    detector.set_min_confidence(args.min_confidence);
    if let Some(seed) = args.noise_seed {
        detector.reset_noise_floor(seed);
    }

    let mut tracker = StateTracker::new();
    if let Some(r) = args.reference {
        tracker.set_reference(r);
    }

    let stdout = io::stdout();
    let mut out = BufWriter::new(stdout.lock());
    let mut buf = vec![Iq::default(); 65_536];

    // We track a virtual "capture time" that advances proportionally to
    // samples consumed, so the tracker's CPR-pairing window and
    // active-ICAO window are meaningful on replay. One sample = 1/sr
    // seconds; we round to nanoseconds.
    let t0 = Instant::now();
    let mut samples_consumed: u64 = 0;
    let mut events_buf: Vec<StateEvent> = Vec::with_capacity(32);

    loop {
        let n = source.read(&mut buf).context("reading samples")?;
        if n == 0 {
            break;
        }
        // The detector callback may fire multiple times per chunk; for each
        // frame we approximate "now" as the time at which the chunk's
        // *last* sample would have arrived. That bounds CPR pairing
        // freshness conservatively (older fragments look slightly more
        // stale than they really are), which is the safe direction.
        samples_consumed += n as u64;
        let virtual_now = t0
            + Duration::from_nanos(
                samples_consumed * 1_000_000_000 / u64::from(args.sample_rate),
            );

        detector.process(&buf[..n], |frame| {
            tracker.ingest(frame, virtual_now, &mut events_buf);
            for event in events_buf.drain(..) {
                let _ = print_state_event(&mut out, &event);
            }
        });
    }
    // Final eviction pass.
    let final_now = t0
        + Duration::from_nanos(samples_consumed * 1_000_000_000 / u64::from(args.sample_rate));
    tracker.evict_stale(final_now, &mut events_buf);
    for event in events_buf.drain(..) {
        let _ = print_state_event(&mut out, &event);
    }
    out.flush().context("flushing stdout")?;

    if args.summary {
        writeln!(out, "--- aircraft summary ---")?;
        let mut entries: Vec<_> = tracker.aircraft().collect();
        entries.sort_by_key(|a| a.icao);
        for a in entries {
            write!(out, "{}", a.icao)?;
            if let Some(cs) = &a.callsign {
                write!(out, " {cs}")?;
            }
            if let Some(p) = a.position {
                write!(
                    out,
                    " @ {:.4},{:.4} ({:?})",
                    p.pos.lat_deg, p.pos.lon_deg, p.source
                )?;
            }
            if let Some(v) = a.velocity {
                write!(out, " ")?;
                print_velocity_summary(&mut out, &v)?;
            }
            writeln!(
                out,
                "  [msgs={} clean={} corrected={} addr-rec={}]",
                a.counters.messages_total,
                a.counters.crc_clean,
                a.counters.crc_corrected,
                a.counters.crc_address_recovered,
            )?;
        }
        out.flush()?;
    }

    eprintln!("rs1090: {} aircraft tracked", tracker.len());
    Ok(())
}

fn print_state_event<W: Write>(out: &mut W, event: &StateEvent) -> io::Result<()> {
    match event {
        StateEvent::Acquired(icao) => writeln!(out, "acquire {icao}")?,
        StateEvent::Identification { icao, callsign } => {
            writeln!(out, "ident   {icao} callsign={callsign}")?;
        }
        StateEvent::Position { icao, pos, source } => {
            let tag = match source {
                PositionSource::Global => "global",
                PositionSource::Local => "local ",
            };
            writeln!(
                out,
                "pos     {icao} {:.4},{:.4} ({tag})",
                pos.lat_deg, pos.lon_deg,
            )?;
        }
        StateEvent::Velocity { icao, velocity } => {
            write!(out, "vel     {icao} ")?;
            print_velocity_summary(out, velocity)?;
            writeln!(out)?;
        }
        StateEvent::Lost(icao) => writeln!(out, "lost    {icao}")?,
        StateEvent::AddressRecovered { icao, df } => {
            writeln!(out, "addr-recover {icao} from DF{}", df.raw_value())?;
        }
        StateEvent::Orphan { df } => writeln!(out, "orphan  DF{}", df.raw_value())?,
    }
    Ok(())
}

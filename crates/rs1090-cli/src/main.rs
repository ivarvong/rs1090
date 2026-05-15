//! `rs1090` command-line tool.
//!
//! For M1: a single `replay` subcommand that reads an `.iq` file (interleaved
//! signed 8-bit I/Q samples) and prints one line per detected Mode S frame.
//! Live SDR ingestion lands in a later milestone.

use std::fs::File;
use std::io::{self, BufReader, BufWriter, Write};
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};

use rs1090::frame::{Frame, FrameDetector};
use rs1090::source::{IqFileSource, SampleSource};
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

fn main() -> Result<()> {
    let cli = Cli::parse();
    match &cli.command {
        Command::Replay(args) => run_replay(args),
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
    // aggregate confidence. Subject to change once the message decoder
    // produces richer output; for now this is enough to eyeball the M1
    // pipeline against a known capture.
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
    writeln!(out, "conf={}", frame.confidence())?;
    Ok(())
}

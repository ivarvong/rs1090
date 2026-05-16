//! Beast binary TCP server (`dump1090-fa --net-bo-port 30005`).
//!
//! Beast is the de facto interchange format between the SDR-side
//! decoder and downstream consumers (tar1090, FlightAware feeders,
//! ADS-B Exchange, Virtual Radar Server). Anything that speaks ADS-B
//! over TCP accepts it natively, so emitting Beast lets rs1090 slot
//! into the existing ecosystem without translation.
//!
//! Wire format per the [Mode-S Beast firmware
//! manual](https://wiki.modesbeast.com/Mode-S_Beast:Data_Output_Formats):
//!
//! ```text
//!   0x1A  <mode>  <timestamp:6>  <signal:1>  <frame bytes>
//! ```
//!
//! Mode byte distinguishes the frame shape:
//!
//! - `0x31` = Mode-A/C (2 bytes, not produced by this decoder)
//! - `0x32` = Mode-S short  (7-byte payload)
//! - `0x33` = Mode-S long   (14-byte payload)
//!
//! Inside the bytes after the leading `0x1A`, any `0x1A` is doubled
//! (`0x1A 0x1A`) — the only escape the protocol has.
//!
//! Timestamp is a free-running 12 MHz GPS counter. Many consumers
//! ignore it; some derive multilateration timing. We emit a counter
//! derived from monotonic time since process start so it's stable and
//! monotonically increasing within a session.
//!
//! Signal level is 0-255 representing peak signal strength. We use
//! the frame's per-bit aggregate confidence as a proxy — same shape
//! (`u8`), same "higher is stronger" semantics.
//!
//! Lint exemption: tar1090 / FlightAware / Beast / MLAT aren't Rust items.
#![allow(clippy::doc_markdown, clippy::cast_possible_truncation)]

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Instant;

use rs1090::crc::CrcOutcome;
use rs1090::frame::Frame;
use tokio::io::AsyncWriteExt;
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::broadcast;

use crate::broadcaster::AppState;

/// Conventional dump1090-fa Beast-output port. The CLI default value
/// uses this in string form; the constant here is for documentation
/// and for any future consumer that needs the protocol-canonical number.
#[allow(dead_code)]
pub const DEFAULT_PORT: u16 = 30_005;

pub async fn run(state: AppState, bind: SocketAddr) -> anyhow::Result<()> {
    let listener = TcpListener::bind(bind).await?;
    tracing::info!(%bind, "Beast binary TCP listening");
    let state = Arc::new(state);
    // One shared start time so all sessions agree on the timeline of
    // the 12 MHz counter (matters if a downstream is doing MLAT-style
    // cross-correlation against multiple sources).
    let t0 = Instant::now();
    loop {
        let (sock, peer) = listener.accept().await?;
        let rx = state.frame_broadcaster.subscribe();
        tokio::spawn(async move {
            if let Err(e) = handle_client(sock, rx, t0).await {
                tracing::debug!(%peer, error = %e, "Beast client disconnected");
            }
        });
    }
}

async fn handle_client(
    mut sock: TcpStream,
    mut rx: broadcast::Receiver<Frame>,
    t0: Instant,
) -> std::io::Result<()> {
    // Long frame = 1 + 1 + 6 + 1 + 14 = 23 raw bytes, ≤ 46 after
    // worst-case doubling. Preallocate once and reuse.
    let mut buf = Vec::with_capacity(64);
    loop {
        let frame = match rx.recv().await {
            Ok(f) => f,
            Err(broadcast::error::RecvError::Lagged(_)) => continue,
            Err(broadcast::error::RecvError::Closed) => return Ok(()),
        };
        if !matches!(
            frame.crc_outcome(),
            CrcOutcome::Clean | CrcOutcome::Corrected { .. }
        ) {
            continue;
        }
        buf.clear();
        encode_beast_frame(&frame, t0, &mut buf);
        sock.write_all(&buf).await?;
    }
}

/// Append a single Beast-framed message to `out`.
fn encode_beast_frame(frame: &Frame, t0: Instant, out: &mut Vec<u8>) {
    let mode = match frame.bytes().len() {
        7 => 0x32,   // Mode-S short
        14 => 0x33,  // Mode-S long
        _ => return, // we never emit Mode-A/C
    };
    let ts = mlat_timestamp(t0);
    let mut header = [0u8; 8];
    header[0] = mode;
    header[1] = ((ts >> 40) & 0xFF) as u8;
    header[2] = ((ts >> 32) & 0xFF) as u8;
    header[3] = ((ts >> 24) & 0xFF) as u8;
    header[4] = ((ts >> 16) & 0xFF) as u8;
    header[5] = ((ts >> 8) & 0xFF) as u8;
    header[6] = (ts & 0xFF) as u8;
    header[7] = frame.confidence();

    out.push(0x1A);
    push_escaped(out, &header);
    push_escaped(out, frame.bytes());
}

/// Append `bytes` to `out`, doubling any `0x1A` per Beast escape rules.
fn push_escaped(out: &mut Vec<u8>, bytes: &[u8]) {
    for &b in bytes {
        out.push(b);
        if b == 0x1A {
            out.push(0x1A);
        }
    }
}

/// Beast timestamp is a 48-bit free-running 12 MHz counter. We
/// derive it from `Instant::now() - t0` to keep it monotonic and
/// session-stable. Rollover at (2^48 / 12e6) seconds ≈ 7.4 hours of
/// continuous run — fine for our use cases; consumers that care
/// about absolute time treat it as relative anyway.
fn mlat_timestamp(t0: Instant) -> u64 {
    let nanos = t0.elapsed().as_nanos();
    let ticks = (nanos * 12) / 1000;
    (ticks as u64) & 0x0000_FFFF_FFFF_FFFF
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_long_frame() -> Frame {
        let bytes = [
            0x8D, 0xA0, 0xBA, 0x4E, 0x59, 0x09, 0xC3, 0x27, 0x9F, 0x7E, 0x82, 0xF7, 0x90, 0x6D,
        ];
        Frame::from_bytes(&bytes)
    }

    #[test]
    fn encodes_long_frame_with_mode_0x33() {
        let frame = synthetic_long_frame();
        let t0 = Instant::now();
        let mut out = Vec::new();
        encode_beast_frame(&frame, t0, &mut out);
        // [0x1A, 0x33, 6×ts, 1×signal, 14×data] = 23 bytes minimum.
        assert_eq!(out[0], 0x1A);
        assert_eq!(out[1], 0x33);
        assert_eq!(out.len(), 23);
        // Last 14 bytes are the frame, unmodified (no 0x1A in this frame).
        assert_eq!(&out[9..], frame.bytes());
    }

    #[test]
    fn escapes_inner_0x1a_with_doubling() {
        // Synthesise a frame whose bytes contain 0x1A.
        let bytes = [
            0x8D, 0xA0, 0x1A, 0x4E, 0x59, 0x09, 0xC3, 0x27, 0x9F, 0x7E, 0x82, 0xF7, 0x90, 0x6D,
        ];
        let frame = Frame::from_bytes(&bytes);
        let mut out = Vec::new();
        encode_beast_frame(&frame, Instant::now(), &mut out);
        // Find the doubled 0x1A inside the data portion. The leading
        // 0x1A (out[0]) is the frame marker; the inner one must be
        // followed by another 0x1A.
        let mut found = false;
        let mut i = 1; // skip the leading marker
        while i < out.len() - 1 {
            if out[i] == 0x1A {
                assert_eq!(out[i + 1], 0x1A, "lone 0x1A at byte {i}");
                found = true;
                i += 2;
            } else {
                i += 1;
            }
        }
        assert!(found, "expected the inner 0x1A to be doubled");
    }

    #[test]
    fn short_frame_uses_mode_0x32() {
        let bytes = [0x5D, 0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6];
        let frame = Frame::from_bytes(&bytes);
        let mut out = Vec::new();
        encode_beast_frame(&frame, Instant::now(), &mut out);
        assert_eq!(out[1], 0x32);
        // [0x1A, 0x32, 6×ts, 1×signal, 7×data] = 16 bytes.
        assert_eq!(out.len(), 16);
    }

    #[test]
    fn timestamp_fits_in_48_bits() {
        let ts = mlat_timestamp(Instant::now());
        assert_eq!(ts & 0xFFFF_0000_0000_0000, 0);
    }
}

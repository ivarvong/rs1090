//! AVR-text TCP server (dump1090 `--raw` convention).
//!
//! Every CRC-clean or 1-bit-corrected frame the decoder produces is
//! emitted to every connected TCP client as one ASCII line:
//!
//! ```text
//! *8DA0BA4E5909C3279F7E82F7906D;
//! ```
//!
//! `*` opens, `;` closes, frame bytes in between as uppercase hex.
//! Lines are terminated with `\r\n` to match the dump1090 wire format
//! exactly — many downstream tools (BaseStation viewers, hobby
//! decoders, `pyModeS.tcpclient.TcpClient`) parse only this shape.
//!
//! Convention port is **30002**. Spawn a tokio task per connection;
//! each subscribes to `state.frame_broadcaster` independently.
//!
//! Lint exemption: ASCII / BaseStation / TCP aren't Rust items.
#![allow(clippy::doc_markdown)]

use std::fmt::Write as _;
use std::net::SocketAddr;
use std::sync::Arc;

use rs1090::crc::CrcOutcome;
use rs1090::frame::Frame;
use tokio::io::AsyncWriteExt;
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::broadcast;

use crate::broadcaster::AppState;

/// Conventional dump1090 raw-output port. The CLI default value uses
/// this in string form; the constant here is for documentation and
/// for any future consumer that needs the protocol-canonical number.
#[allow(dead_code)]
pub const DEFAULT_PORT: u16 = 30_002;

/// Run the AVR TCP listener until it errors. Accepts connections
/// forever; each connection gets its own broadcast receiver so
/// per-client backpressure stays per-client.
pub async fn run(state: AppState, bind: SocketAddr) -> anyhow::Result<()> {
    let listener = TcpListener::bind(bind).await?;
    tracing::info!(%bind, "AVR-text TCP listening");
    let state = Arc::new(state);
    loop {
        let (sock, peer) = listener.accept().await?;
        let rx = state.frame_broadcaster.subscribe();
        tokio::spawn(async move {
            if let Err(e) = handle_client(sock, rx).await {
                tracing::debug!(%peer, error = %e, "AVR client disconnected");
            }
        });
    }
}

async fn handle_client(
    mut sock: TcpStream,
    mut rx: broadcast::Receiver<Frame>,
) -> std::io::Result<()> {
    // Reuse one String across frames so we don't allocate per emit.
    // Long-frame hex = 28 chars + 4 control bytes = 32 bytes; round up.
    let mut line = String::with_capacity(40);
    loop {
        let frame = match rx.recv().await {
            Ok(f) => f,
            // Slow clients lag — clear the backlog, keep streaming.
            Err(broadcast::error::RecvError::Lagged(_)) => continue,
            Err(broadcast::error::RecvError::Closed) => return Ok(()),
        };
        // AVR is a "validated frames only" feed by convention; broadcast
        // failures (the address-XOR recovery path) aren't represented
        // because their byte content isn't authoritative.
        if !matches!(
            frame.crc_outcome(),
            CrcOutcome::Clean | CrcOutcome::Corrected { .. }
        ) {
            continue;
        }
        line.clear();
        line.push('*');
        for b in frame.bytes() {
            let _ = write!(line, "{b:02X}");
        }
        line.push_str(";\r\n");
        sock.write_all(line.as_bytes()).await?;
    }
}

#[cfg(test)]
mod tests {
    use rs1090::frame::Frame;
    use std::fmt::Write as _;

    /// The encoding done inline in `handle_client`, extracted for unit
    /// test. Keeping it inline in the hot path avoids the allocation
    /// of a return Vec; this test pins the canonical output shape.
    fn encode_line(frame: &Frame) -> String {
        let mut s = String::with_capacity(40);
        s.push('*');
        for b in frame.bytes() {
            let _ = write!(s, "{b:02X}");
        }
        s.push_str(";\r\n");
        s
    }

    #[test]
    fn avr_line_matches_dump1090_shape() {
        // 14-byte long frame, real-looking DF17 bytes.
        let bytes = [
            0x8D, 0xA0, 0xBA, 0x4E, 0x59, 0x09, 0xC3, 0x27, 0x9F, 0x7E, 0x82, 0xF7, 0x90, 0x6D,
        ];
        let f = Frame::from_bytes(&bytes);
        let line = encode_line(&f);
        assert_eq!(line, "*8DA0BA4E5909C3279F7E82F7906D;\r\n");
    }

    #[test]
    fn avr_line_handles_short_frame() {
        let bytes = [0x5D, 0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6];
        let f = Frame::from_bytes(&bytes);
        let line = encode_line(&f);
        assert_eq!(line, "*5DA1B2C3D4E5F6;\r\n");
    }
}

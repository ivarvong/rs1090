//! HTTP/SSE integration test.
//!
//! Synthesizes a known DF 17 frame, writes it as an `.iq` file in a temp
//! directory, spawns the binary against it, then hits each endpoint and
//! asserts on the responses. This is the M5 acceptance criterion in
//! miniature: a single test that takes the pipeline from samples on disk
//! through HTTP/SSE to a parsed JSON event.

use std::io::Write;
use std::process::{Child, Command, Stdio};
use std::thread;
use std::time::Duration;

use rs1090::crc;
use rs1090::test_utils::{
    synth_bits_as_magnitude, PREAMBLE_HIGH_IDX, PREAMBLE_SAMPLES, SAMPLES_PER_BIT,
};

const LONG_FRAME_BYTES: usize = 14;

/// Synth a DF 17 identification frame carrying "TEST1234" with ICAO A1B2C3.
fn synth_frame() -> [u8; LONG_FRAME_BYTES] {
    let codes: [u8; 8] = [20, 5, 19, 20, 49, 50, 51, 52]; // TEST1234
    let mut me = [0u8; 7];
    me[0] = 4u8 << 3; // TC=4
    let mut bits: u64 = 0;
    for c in codes {
        bits = (bits << 6) | u64::from(c & 0x3F);
    }
    for i in 0..6 {
        me[1 + i] = ((bits >> (8 * (5 - i))) & 0xFF) as u8;
    }

    let mut data = [0u8; 11];
    data[0] = 17u8 << 3 | 5;
    data[1] = 0xA1;
    data[2] = 0xB2;
    data[3] = 0xC3;
    data[4..11].copy_from_slice(&me);
    let crc_val = crc::crc24(&data);
    let mut frame = [0u8; LONG_FRAME_BYTES];
    frame[..11].copy_from_slice(&data);
    frame[11] = ((crc_val >> 16) & 0xFF) as u8;
    frame[12] = ((crc_val >> 8) & 0xFF) as u8;
    frame[13] = (crc_val & 0xFF) as u8;
    frame
}

fn make_iq_file(frame: &[u8], path: &std::path::Path) {
    let payload_bits = frame.len() * 8;
    let payload_samples = payload_bits * SAMPLES_PER_BIT;
    // 2048 leading samples ≈ several time constants of the default
    // noise-floor EMA, so the floor settles to the steady-state value
    // before the preamble arrives. The CLI's `replay` test uses
    // `--noise-seed 5` to skip the warmup; the server binary takes its
    // defaults, so we have to give it room.
    let lead = 2048;
    let total = lead + PREAMBLE_SAMPLES + payload_samples + 64;
    let mut mags = vec![5u16; total];
    let pre = lead;
    for &k in &PREAMBLE_HIGH_IDX {
        mags[pre + k] = 120;
    }
    let mut bits = vec![false; payload_bits];
    for (i, b) in frame.iter().enumerate() {
        for k in 0..8 {
            bits[i * 8 + k] = (b >> (7 - k)) & 1 != 0;
        }
    }
    synth_bits_as_magnitude(
        &bits,
        120,
        5,
        &mut mags[pre + PREAMBLE_SAMPLES..pre + PREAMBLE_SAMPLES + payload_samples],
    );
    let mut bytes = Vec::with_capacity(mags.len() * 2);
    for m in mags {
        let i = i8::try_from(m.min(127)).unwrap();
        bytes.extend_from_slice(&i.to_ne_bytes());
        bytes.push(0);
    }
    std::fs::write(path, bytes).expect("write iq");
}

/// Spawn the binary and wait until it answers `/healthz`. Returns the child
/// handle so the test can kill it on completion. Uses a high port (38xxx)
/// to avoid clashes with anything else on the machine.
///
/// The returned Child must be wrapped in `KillOnDrop` by the caller —
/// clippy's `zombie_processes` lint sees this fn return without `wait()`
/// and doesn't realize ownership transfers to the caller.
#[allow(clippy::zombie_processes)]
fn spawn_serve(iq_path: &std::path::Path, port: u16) -> Child {
    let bin = env!("CARGO_BIN_EXE_rs1090-serve");
    let bind = format!("127.0.0.1:{port}");
    let mut child = Command::new(bin)
        .args(["--bind", &bind, "file", iq_path.to_str().unwrap()])
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn rs1090-serve");
    // Poll /healthz for up to 5 seconds.
    for _ in 0..50 {
        thread::sleep(Duration::from_millis(100));
        let url = format!("http://127.0.0.1:{port}/healthz");
        if let Ok(resp) = ureq_get(&url) {
            if resp == "ok" {
                return child;
            }
        }
    }
    let _ = child.kill();
    panic!("rs1090-serve did not come up on port {port}");
}

/// Minimal blocking HTTP GET using std (no extra dep).
///
/// Reads the entire response (Connection: close) then locates the
/// body after the `\r\n\r\n` header terminator. Naively splits on the
/// terminator string rather than trying to parse Content-Length /
/// Transfer-Encoding, which is plenty for our axum responses that
/// always send the body in one shot.
fn ureq_get(url: &str) -> Result<String, std::io::Error> {
    use std::io::Read;
    use std::net::TcpStream;
    let url = url.trim_start_matches("http://");
    let (host_port, path) = url.split_once('/').unwrap_or((url, ""));
    let path = format!("/{path}");
    let mut stream = TcpStream::connect(host_port)?;
    stream.set_read_timeout(Some(Duration::from_secs(2)))?;
    write!(
        stream,
        "GET {path} HTTP/1.1\r\nHost: {host_port}\r\nConnection: close\r\n\r\n"
    )?;
    let mut raw = Vec::new();
    stream.read_to_end(&mut raw)?;
    // Find header/body separator.
    let sep = b"\r\n\r\n";
    let body_start = raw
        .windows(sep.len())
        .position(|w| w == sep)
        .map_or(raw.len(), |p| p + sep.len());
    Ok(String::from_utf8_lossy(&raw[body_start..]).to_string())
}

#[test]
fn http_endpoints_serve_decoded_state() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let iq_path = tmp.path().join("synth.iq");
    let frame = synth_frame();
    make_iq_file(&frame, &iq_path);

    // Pick a per-test port to avoid races between concurrent integration
    // tests if anyone adds more later.
    let port = 38_421u16;
    let mut child = spawn_serve(&iq_path, port);
    let _kill_on_drop = scopeguard_kill(&mut child);

    // Poll /aircraft until the decoder has produced our frame. The synth
    // file is ~500 samples (0.25 ms at 2 MS/s), so the decoder finishes
    // essentially immediately in non-realtime mode. With --realtime the
    // server paces by sample-rate which would mean ~0 wall time anyway.
    // Allow up to 5 seconds for the JSON to show our ICAO.
    let mut body = String::new();
    for _ in 0..50 {
        thread::sleep(Duration::from_millis(100));
        body = ureq_get(&format!("http://127.0.0.1:{port}/aircraft")).expect("aircraft");
        if body.contains("A1B2C3") {
            break;
        }
    }

    // /aircraft — the polling loop above should have populated this.
    assert!(
        body.contains("A1B2C3"),
        "aircraft body missing ICAO: {body}"
    );
    assert!(
        body.contains("TEST1234"),
        "aircraft body missing callsign: {body}"
    );

    // /healthz (asserted after /aircraft so a shadowing bug like the
    // one this test caught during development can't recur).
    let health = ureq_get(&format!("http://127.0.0.1:{port}/healthz")).expect("healthz");
    assert_eq!(health.trim(), "ok");

    // /aircraft/A1B2C3
    let body = ureq_get(&format!("http://127.0.0.1:{port}/aircraft/A1B2C3")).expect("by-icao");
    assert!(body.contains("TEST1234"), "got: {body}");

    // /aircraft/ZZZZZZ → 400
    // (our minimal client doesn't surface status codes, so just check the
    // body looks like an error string)
    let body = ureq_get(&format!("http://127.0.0.1:{port}/aircraft/ZZZZZZ")).unwrap_or_default();
    assert!(
        body.contains("icao") || body.is_empty(),
        "expected error body, got: {body}",
    );

    // /metrics — Prometheus exposition. After decoding our DF 17 frame,
    // we expect both the per-frame counter (with `outcome="clean"`) and
    // the identification state-event counter to be non-zero, plus the
    // aircraft-tracked gauge to register our one ICAO.
    let body = ureq_get(&format!("http://127.0.0.1:{port}/metrics")).expect("metrics");
    assert!(
        body.contains("# TYPE rs1090_frames_total counter"),
        "metrics body missing frames counter type: {body}"
    );
    assert!(
        body.contains("rs1090_frames_total{outcome=\"clean\"} 1"),
        "metrics body missing clean-frame counter: {body}"
    );
    assert!(
        body.contains("rs1090_state_events_total{kind=\"identification\"} 1"),
        "metrics body missing identification event counter: {body}"
    );
    assert!(
        body.contains("rs1090_aircraft_tracked 1"),
        "metrics body missing aircraft gauge: {body}"
    );
    assert!(
        body.contains("rs1090_decoder_alive 1"),
        "metrics body missing decoder_alive gauge: {body}"
    );
}

/// RAII helper: ensure the spawned child is killed when the test exits.
fn scopeguard_kill(child: &mut Child) -> KillOnDrop<'_> {
    KillOnDrop(child)
}

struct KillOnDrop<'a>(&'a mut Child);
impl Drop for KillOnDrop<'_> {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// `--config <toml>` should provide the `[source]` section in place
/// of the subcommand, and any non-CLI fields should come from the
/// file. Asserts the round-trip works end-to-end by writing a TOML
/// pointing at a synthesised IQ file, running `rs1090-serve --config
/// FILE` with no subcommand, and confirming `/aircraft` shows the
/// expected frame.
#[test]
#[allow(clippy::zombie_processes)]
fn config_file_provides_source_and_bind() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let iq_path = tmp.path().join("synth.iq");
    let frame = synth_frame();
    make_iq_file(&frame, &iq_path);

    let port = 38_422u16;
    let config_path = tmp.path().join("serve.toml");
    let toml_body = format!(
        r#"
bind = "127.0.0.1:{port}"
min_confidence = 0

[source]
kind = "file"
path = "{path}"
sample_rate = 2000000
center_freq = 1090000000
"#,
        path = iq_path.to_str().unwrap().replace('\\', "\\\\"),
    );
    std::fs::write(&config_path, toml_body).expect("write toml");

    let bin = env!("CARGO_BIN_EXE_rs1090-serve");
    let mut child = Command::new(bin)
        .args(["--config", config_path.to_str().unwrap()])
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn rs1090-serve");
    let _kill_on_drop = scopeguard_kill(&mut child);

    // Wait for /healthz.
    let mut up = false;
    for _ in 0..50 {
        thread::sleep(Duration::from_millis(100));
        if let Ok(resp) = ureq_get(&format!("http://127.0.0.1:{port}/healthz")) {
            if resp == "ok" {
                up = true;
                break;
            }
        }
    }
    assert!(up, "rs1090-serve --config never came up on port {port}");

    // Poll /aircraft until the decoded frame is visible.
    let mut body = String::new();
    for _ in 0..50 {
        thread::sleep(Duration::from_millis(100));
        body = ureq_get(&format!("http://127.0.0.1:{port}/aircraft")).expect("aircraft");
        if body.contains("A1B2C3") {
            break;
        }
    }
    assert!(
        body.contains("A1B2C3"),
        "config-driven run missed ICAO: {body}"
    );
}

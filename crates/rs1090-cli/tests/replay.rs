//! End-to-end test for the `replay` subcommand.
//!
//! Synthesizes a known DF 17 frame, writes it as an `.iq` file in the
//! canonical signed-8-bit interleaved format, runs the CLI, and asserts
//! the printed output matches the synthesized frame.
//!
//! This is the M1 acceptance test in miniature: every layer from sample
//! source through CRC has to work together to produce the expected line.

use std::fmt::Write as _;
use std::io::Write;
use std::process::Command;

use rs1090::crc;
use rs1090::test_utils::{
    synth_bits_as_magnitude, PREAMBLE_HIGH_IDX, PREAMBLE_SAMPLES, SAMPLES_PER_BIT,
};

/// Long-frame size in bytes. Mirrored locally so this integration test
/// doesn't depend on whether `crc::LONG_FRAME_BYTES` is part of the public
/// API surface.
const LONG_FRAME_BYTES: usize = 14;

/// Build a synthetic DF 17 identification frame carrying callsign
/// "TEST1234". Lets us assert end-to-end that the CLI invokes the
/// message decoder and prints the callsign, not just the raw bytes.
fn synth_df17_frame() -> [u8; LONG_FRAME_BYTES] {
    // Identification: TC=4, category=0, eight 6-bit codes for "TEST1234".
    let codes: [u8; 8] = [
        20, // T
        5,  // E
        19, // S
        20, // T
        49, // 1
        50, // 2
        51, // 3
        52, // 4
    ];
    let mut me = [0u8; 7];
    me[0] = 4u8 << 3; // TC=4, category=0
    let mut bits: u64 = 0;
    for c in codes {
        bits = (bits << 6) | u64::from(c & 0x3F);
    }
    for i in 0..6 {
        me[1 + i] = ((bits >> (8 * (5 - i))) & 0xFF) as u8;
    }

    let mut data = [0u8; LONG_FRAME_BYTES - 3];
    data[0] = (17u8 << 3) | 5; // DF 17, capability = 5
    data[1] = 0x4B;
    data[2] = 0x9C;
    data[3] = 0xA2;
    data[4..11].copy_from_slice(&me);
    let crc_val = crc::crc24(&data);
    let mut frame = [0u8; LONG_FRAME_BYTES];
    frame[..LONG_FRAME_BYTES - 3].copy_from_slice(&data);
    frame[LONG_FRAME_BYTES - 3] = ((crc_val >> 16) & 0xFF) as u8;
    frame[LONG_FRAME_BYTES - 2] = ((crc_val >> 8) & 0xFF) as u8;
    frame[LONG_FRAME_BYTES - 1] = (crc_val & 0xFF) as u8;
    debug_assert_eq!(crc::crc24(&frame), 0);
    frame
}

/// Encode a frame as a magnitude stream plus leading/trailing noise floor.
fn synth_magnitudes(frame: &[u8], pulse: u16, floor: u16, lead: usize, trail: usize) -> Vec<u16> {
    let payload_bits = frame.len() * 8;
    let payload_samples = payload_bits * SAMPLES_PER_BIT;
    let total = lead + PREAMBLE_SAMPLES + payload_samples + trail;
    let mut out = vec![floor; total];
    // Preamble.
    let pre = lead;
    for &k in &PREAMBLE_HIGH_IDX {
        out[pre + k] = pulse;
    }
    // Payload.
    let mut bits = vec![false; payload_bits];
    for (i, b) in frame.iter().enumerate() {
        for k in 0..8 {
            bits[i * 8 + k] = (b >> (7 - k)) & 1 != 0;
        }
    }
    synth_bits_as_magnitude(
        &bits,
        pulse,
        floor,
        &mut out[pre + PREAMBLE_SAMPLES..pre + PREAMBLE_SAMPLES + payload_samples],
    );
    out
}

/// Convert magnitudes to interleaved signed-8-bit I/Q bytes such that
/// `alpha_max_beta_min(i, 0) == mag`. We clamp to the i8 positive range
/// and emit the byte via `to_ne_bytes` so the i8→u8 reinterpretation is
/// explicit rather than relying on a lint-suppressed `as` cast.
fn magnitudes_to_iq_bytes(mags: &[u16]) -> Vec<u8> {
    let mut out = Vec::with_capacity(mags.len() * 2);
    for &m in mags {
        let i = i8::try_from(m.min(127)).expect("clamped to i8::MAX");
        out.extend_from_slice(&i.to_ne_bytes());
        out.push(0);
    }
    out
}

#[test]
fn replay_decodes_synthetic_df17_frame() {
    let frame = synth_df17_frame();
    let mags = synth_magnitudes(&frame, 120, 5, 64, 64);
    let bytes = magnitudes_to_iq_bytes(&mags);

    let tmp = tempfile::NamedTempFile::new().expect("tempfile");
    tmp.as_file().write_all(&bytes).expect("write iq");
    tmp.as_file().sync_all().ok();

    let bin = env!("CARGO_BIN_EXE_rs1090");
    let output = Command::new(bin)
        .args([
            "replay",
            tmp.path().to_str().unwrap(),
            "--sample-rate",
            "2000000",
            "--noise-seed",
            "5",
        ])
        .output()
        .expect("run rs1090");
    assert!(
        output.status.success(),
        "rs1090 replay failed: stderr={}",
        String::from_utf8_lossy(&output.stderr)
    );

    let stdout = String::from_utf8(output.stdout).expect("utf-8 stdout");
    let stderr = String::from_utf8(output.stderr).expect("utf-8 stderr");

    // Expected payload as hex.
    let mut hex = String::with_capacity(frame.len() * 2);
    for b in frame {
        write!(hex, "{b:02X}").expect("write to String never fails");
    }

    assert!(
        stdout.contains("DF17"),
        "expected DF17 line in stdout, got:\n{stdout}"
    );
    assert!(
        stdout.contains(&hex),
        "expected hex payload {hex} in stdout, got:\n{stdout}"
    );
    assert!(
        stdout.contains("clean"),
        "expected clean CRC outcome, got:\n{stdout}"
    );
    assert!(
        stdout.contains("ICAO=4B9CA2"),
        "expected ICAO=4B9CA2 in decoded line, got:\n{stdout}"
    );
    assert!(
        stdout.contains("callsign=TEST1234"),
        "expected callsign=TEST1234 in decoded line, got:\n{stdout}"
    );
    assert!(
        stderr.contains("1 frames"),
        "expected exactly one frame in stderr summary, got:\n{stderr}"
    );
}

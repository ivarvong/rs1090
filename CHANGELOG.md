# Changelog

All notable changes to rs1090 will be documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Initial workspace scaffold: `rs1090` (library) and `rs1090-cli` (binary) crates.
- `magnitude` module with two implementations of |I + jQ|: branchless
  alpha-max-plus-beta-min (`α = 1, β = 1/2`) for ARMv6, and a 128 KiB
  compile-time LUT exact to rounding for x86_64. Exhaustive tests over the full
  `(i8, i8)` input space; criterion benchmark covering a 1 ms / 2 MS/s chunk.
- `crc` module: Mode S CRC-24 with generator `0xFFF409`. Byte-at-a-time
  (1 KiB table) and bit-at-a-time implementations cross-check each other.
  `check()` performs 1-bit error correction via compile-time syndrome tables
  for both short (56-bit) and long (112-bit) frames; tables are verified
  collision-free, which is the structural prerequisite for unambiguous
  correction. Proptest sweeps the roundtrip and single-bit recovery.
- `demod` module: noise-floor EMA (`NoiseFloor`, shift-based), preamble
  correlator (`preamble_score`, `preamble_clears_threshold`), and bit slicer
  with per-bit confidence in `[0, 255]`. Aggregate confidence uses the
  minimum — one bad bit dominates a frame's correctability. Synthetic-PPM
  round-trip tests under `test-utils` (also enabled in `#[cfg(test)]`).
- `frame` module: `DownlinkFormat` enum (DFs 0/4/5/11/16/17/18/20/21 plus a
  `Reserved` catch-all), `Frame` value type, and `FrameDetector` — a
  streaming detector that takes chunked `Iq` input via a callback. Handles
  chunk-straddling preambles with a 256-sample carry buffer. End-to-end tests
  recover synthetic DF 17 frames at high SNR and exercise 1-bit correction
  through the detector → CRC path.
- `source` module (std-only): `SampleSource` trait and `IqFileSource` backend
  reading raw interleaved signed 8-bit I/Q files. Handles short reads and
  odd-byte EOFs.
- `rs1090-cli`: `replay` subcommand reads an `.iq` file and prints one line
  per detected frame (DF code, hex payload, CRC outcome, aggregate
  confidence). Integration test synthesizes a DF 17 frame on disk and
  verifies the binary's output end-to-end.
- `cpr` module: CPR airborne position decoder. Global decode from
  even/odd pair, local decode from a reference position. NL transition
  table pinned from ICAO Annex 10 Table 2-1 with three cross-checks: a
  formula sweep across the globe at 0.1° resolution, equator symmetry,
  and pinned band midpoints. Tests the canonical Sun "1090 MHz Riddle"
  worked example and round-trips at five geographically diverse points.
  Local decode's wrong-tile failure mode for distant references is
  documented as a known limitation (test pinned).
- `message` module: top-level `decode(&Frame)`. ICAO address type, TC
  dispatch covering identification (TC 1–4), airborne position (TC 9–18,
  20–22), and velocity (TC 19, subtypes 1–4). Callsigns decoded into
  `ArrayString<8>` from the 6-bit ICAO charset. Altitude decoding
  supports the 25-ft Q-bit baro encoding; Gillham (Mode C) is exposed as
  a distinct variant for future Gray-code work. Surveillance replies
  (DF 4/5/16/20/21) surface raw bytes because their address-XOR CRC
  cannot be validated without an active-aircraft set.
- CLI now prints a decoded summary per frame: `ICAO=...`, callsign,
  altitude, CPR fields, velocity in knots/heading.

- `state` module: per-aircraft state tracker keyed by ICAO. Maintains
  callsign, category, latest position (with even/odd CPR pairing within
  a 10s window), latest velocity, and per-aircraft CRC counters. LRU
  eviction at default capacity 4096; stale entries are dropped after
  5 minutes idle.
- Address-XOR CRC recovery for DF 0/4/5/16/20/21 surveillance replies:
  the tracker maintains the active-ICAO set (last 60s by default) and
  matches each failed-CRC frame's syndrome against `crc24(icao_bytes)`
  for known aircraft. In our 2-minute live capture this recovers 1272
  of 1522 address-XOR'd frames (84%), turning ~80% of "failed" frames
  into useful surveillance data per active aircraft.
- `FrameDetector` bug fix: the inner sliding-window loop required room
  for a long (14-byte) frame, missing short DF 0/4/5/11 frames near the
  tail of any sample buffer. Fixed to require only the minimum
  short-frame window; long-frame buffer length is still checked once
  the DF resolves.
- `rs1090 track` subcommand: replay an `.iq` file through the full
  pipeline (source → detector → message → tracker) and print one line
  per state event: `acquire`, `ident`, `pos` (with global/local source
  tag), `vel`, `lost`, `addr-recover`, `orphan`. `--summary` prints a
  per-aircraft roll-up at end of file. `--reference lat,lon` enables
  local CPR decode when no even/odd pair is available.

### M3 known limitations

- CPR latitude-zone-mismatch path is implemented but didn't fire in our
  2-min capture; will be exercised once a fast-moving target near an NL
  boundary appears.
- Address-XOR CRC recovery uses linear scan over the active set on each
  failed frame. With ~10 active aircraft this is sub-microsecond; with
  thousands (e.g. an airport-camera-grade receiver) it becomes O(n) per
  frame and would want a syndrome→ICAO precomputed map. Optimization
  deferred until we have a use case for it.
- Surface position (TC 5–8), aircraft status (TC 28), and operational
  status (TC 31) are still surfaced as `Raw` payloads; decoding lands
  in a future milestone alongside Beast/SBS network output formats.

## M4: live RTL-SDR backend

### Added

- `rtl-sdr` feature on the `rs1090` library: a `SampleSource`
  implementation backed by the `rs-rtl` crate (pure-Rust nusb driver,
  no libusb dependency). `RtlSdrSourceBuilder` exposes device index,
  sample rate, center freq, manual/auto gain, and bias-T.
- `rs1090 live` subcommand: open the dongle, stream samples directly
  through the detector and tracker, print state events as they happen.
  `--record` simultaneously saves the bias-subtracted IQ stream to disk
  for later replay; `--duration-secs` bounds the session; Ctrl-C
  shuts down cleanly. First-class real-time decoder, no python
  bias-subtraction pipeline required.

## M5: HTTP / Server-Sent Events server

### Added

- New crate `rs1090-serve`: an HTTP server that exposes the decoded
  stream over Server-Sent Events. Sync decoder thread owns the SDR;
  tokio runtime hosts `axum` and the SSE fan-out via
  `tokio::sync::broadcast`. The library proper stays tokio-free.
- Endpoints (per DESIGN.md §12):
  - `GET /healthz`
  - `GET /aircraft` — JSON snapshot of all currently-tracked aircraft.
  - `GET /aircraft/:icao` — JSON for a single ICAO.
  - `GET /stream` — `text/event-stream` of live events with
    `event:`/`id:`/`data:` lines. Heartbeats every 15s.
  - Stream filters: `?type=position,velocity` and `?icao=AB9B13,A1D49E`
    compose. Multiple values are comma-separated.
- JSON schema per DESIGN.md §12.2: SI units throughout (m/s, degrees),
  RFC 3339 UTC timestamps, versioned (`"v": 1`), `id` field on every
  event so clients can resume via `Last-Event-ID` (honoured by the
  server; full ring-buffer replay is deferred until a real reconnect
  pattern emerges).
- Two source modes: `file` (replay an `.iq` capture, with optional
  `--realtime` pacing for demos) and `live` (open the RTL-SDR dongle,
  behind the `rtl-sdr` feature flag).
- Integration test that synthesizes a DF 17 frame on disk, spawns the
  binary, polls `/aircraft` until the decoder catches up, and asserts
  the ICAO and callsign appear in the JSON. Uses a hand-rolled
  blocking HTTP client to avoid pulling in an HTTP-client dep.
- Sanity: 99 tests passing across the workspace (90 lib + 1 CLI
  integration + 7 serve unit + 1 serve integration); clippy clean
  under `-D warnings` with the pedantic group enabled.

### M5 deferred work

- Full Last-Event-ID replay (ring buffer of recent events). The
  protocol is honoured — clients sending `Last-Event-ID` skip past
  earlier ids in the live stream — but we don't yet maintain a
  bounded history for true gap-filling reconnects. The atomic id
  counter is already in place; adding the ring buffer is a self-
  contained change.
- Backpressure: `tokio::sync::broadcast` lags rather than drops on
  slow consumers (subscribers see `RecvError::Lagged`); DESIGN.md
  calls for explicit `event: dropped` notifications and per-client
  queue metrics, which are not yet wired up.
- Auth, Prometheus `/metrics`, bbox filter, CORS configuration. All
  trivial individually; deferred until the use case is concrete.

### Fixed

- DESIGN.md previously cited the 3.96% error bound, which applies to the
  multiplicative variant `α = 15/16, β = 15/32`. The shift-only form we
  actually use peaks at ~11.8% in continuous math; corrected in both the
  design doc and module-level docs.

# Changelog

All notable changes to rs1090 will be documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Structured logging via `tracing`** in `rs1090-serve` — every prior
  `eprintln!` is now a `tracing` event with the right level and
  structured fields. `RUST_LOG=warn,rs1090_serve=info,rs1090=info` is
  the default filter; override via `RUST_LOG` like any other
  tracing app.
- **Prometheus `/metrics` endpoint** on `rs1090-serve`:
  - `rs1090_frames_total{outcome=clean|corrected|failed}` (counter)
  - `rs1090_state_events_total{kind=acquired|identification|position|velocity|address_recovered|orphan|lost}` (counter)
  - `rs1090_aircraft_tracked` (gauge)
  - `rs1090_sse_subscribers` (gauge, RAII-decremented when a client
    drops)
  - `rs1090_decoder_alive` (gauge, mirrors `/healthz`)
  Labels are deliberately low-cardinality (no per-ICAO tags). See
  [`docs/deploy.md`](docs/deploy.md#prometheus-metrics) for the
  Grafana / alerting playbook.
- **Shippable systemd unit** at
  [`dist/systemd/rs1090-serve.service`](dist/systemd/rs1090-serve.service)
  with `install.sh` next to it. Hardening over the previous
  inline-in-docs unit: `StartLimitIntervalSec=0` so flap-protection
  never gives up on a chronically-flaky USB cable;
  `ProtectSystem=strict`, `NoNewPrivileges`, `PrivateTmp` and the
  rest of the sandboxing knobs that match what the service actually
  needs (USB + a listen socket).
- **One-command remote deploy** via `dist/deploy.sh` driven by
  `dist/.env` (gitignored). Cross-compiles with `cargo-zigbuild`,
  scps the binary, renders the unit's `ExecStart` with per-site
  values, installs + enables + restarts in one go, polls `/healthz`
  to verify. Idempotent. Full runbook at
  [`docs/deploy.md`](docs/deploy.md).
- **TOML config via `--config /etc/rs1090/serve.toml`** —
  alternative to the `.env`-rendered `ExecStart` path. Every CLI
  flag has a typed home in the TOML schema, including a `[source]`
  section that stands in for the `live` / `file` subcommand.
  Explicit CLI flags still win (precedence resolved via clap's
  `ValueSource`). Example file ships at
  `dist/etc/rs1090/serve.toml.example`. Closes #33.

### Changed

- **Decoder death exits the process.** The `LivenessGuard` in
  `rs1090-serve` (added M6) used to flip `/healthz` to 503 and leave
  the HTTP server running on a frozen snapshot. Now it also notifies
  the tokio runtime, which closes the listener; `main` returns
  `exit(1)` so systemd's `Restart=on-failure` brings the process
  back. The old behaviour is what most operators thought was a bug.
- **Live-source `Ok(0)` reads are fatal.** When the `rs-rtl`
  streaming thread gives up after five consecutive bulk-transfer
  errors (dongle yanked, USB reset), the `SampleSource::read`
  returns `Ok(0)`. The decoder loop previously treated that as
  clean file-EOF and stayed running; now it `bail!`s, the
  LivenessGuard fires, and systemd brings us back. File replay
  still EOFs cleanly so `… file capture.iq` keeps serving the
  final snapshot until Ctrl-C.

## [0.1.0] — 2026-05-16

First tagged release. The full pipeline — `Iq` samples → demod → CRC →
message decode → state tracker — works end-to-end on real captures and
on a live RTL-SDR. HTTP/SSE serving, a live web map, GDL90 / AVR /
Beast ecosystem outputs, and an unvalidated-on-hardware BLE GATT
peripheral all ship in `rs1090-serve`. 123 tests passing across the
workspace; clippy clean under `-D warnings` pedantic; libFuzzer
crash-free across three targets at tens of millions of executions;
the pyModeS differential harness reports zero field-level
disagreements on the 2-minute corpus.

**Known scope/quality calls explicitly made for v0.1.0**:

- The BLE peripheral code ships and unit-tests pass, but live
  hardware verification (nRF Connect on iPhone seeing the
  advertisement) is deferred — the on-Pi `--features ble` build was
  in flight when v0.1.0 was tagged. Code is dormant unless the user
  enables both the `ble` feature and `--ble` at runtime.
- ForeFlight GDL90 reception requires a Pro Plus subscription; we
  emit the ForeFlight ID extension correctly, but free/Basic tiers
  don't bind UDP 4000 and so won't show traffic regardless.
- The Pi Zero W (original ARMv6) target named in DESIGN.md is still
  unvalidated; only the Pi Zero 2 W (aarch64) has been measured
  end-to-end.
- The `live` subcommand requires a real RTL-SDR; there is no mock
  backend for CI-free hardware-less testing of the SDR path.

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
- Auth and CORS configuration — still deferred until the use case
  is concrete. (Prometheus `/metrics` and the bbox + altitude
  filters subsequently landed; see `[Unreleased]` and the README's
  "Stream filters" table.)

### Fixed

- DESIGN.md previously cited the 3.96% error bound, which applies to the
  multiplicative variant `α = 15/16, β = 15/32`. The shift-only form we
  actually use peaks at ~11.8% in continuous math; corrected in both the
  design doc and module-level docs.

## M6: validation, hardening, and a staff+ quality pass

### Added

- **Differential testing harness** (`scripts/diff_pymodes.py`) — runs
  rs1090 `replay` over an `.iq` corpus and cross-checks every CRC-clean
  DF 11 / DF 17 / DF 18 frame's decoded fields against pyModeS, the
  de facto Python reference for ADS-B decoding. ICAO, altitude, CPR
  even/odd, CPR lat/lon, callsign, ground speed, and vertical rate
  all compared per-frame; reports agreement counts plus sample
  disagreements. See [`docs/differential-testing.md`](docs/differential-testing.md).
- **libFuzzer harness** at `crates/rs1090/fuzz/` with three targets:
  - `decode_message` — `Frame::from_bytes` → `message::decode`.
  - `process_frame` — full IQ → magnitude → preamble → CRC pipeline.
  - `crc_check` — `crc::check` on arbitrary 7- or 14-byte buffers.
  Each target ships with a seed corpus of real-frame inputs from a
  live capture. Crash-free across tens of millions of executions to
  date. See [`docs/fuzzing.md`](docs/fuzzing.md).
- **Live aircraft map UI** served at `GET /` from `rs1090-serve`.
  Single static page (HTML/CSS/JS embedded via `include_str!`,
  Leaflet from a CDN), seeds from `/aircraft` on load and subscribes
  to `/stream` for live updates. Aircraft icons rotate to track
  angle; popups show callsign, lat/lon, altitude (ft + baro/gnss
  tag), track, ground speed, and vertical rate.
- **Altitude in the wire format**: `AircraftPosition` and
  `SnapshotPosition` now carry `alt_ft: Option<i32>` and
  `alt_source: Option<"baro"|"gnss">` — the decoder was already
  extracting altitude; the wire path was dropping it.
- **`Frame::from_bytes` constructor** under the `test-utils` feature,
  so fuzz targets and integration tests can build a `Frame` from raw
  bytes without re-synthesising the demod pipeline.
- **`BitReader` helper** in `message.rs` — 1-indexed MSB-first reader
  that makes `decode_velocity` and `decode_airborne_position`
  line-by-line auditable against DO-260B. Replaces a dozen ad-hoc
  `(me_bits >> (56 - N)) & MASK` expressions.
- **`/healthz` returns 503 when the decoder thread has died** —
  disarm-able Drop guard distinguishes clean file-replay completion
  (still healthy) from error / panic exit (unhealthy).
- **`PositionSource::wire_tag()`** consolidates the three
  duplicated `match source { Global => "global", Local => "local" }`
  blocks in CLI, broadcaster, and events into one method.
- **GitHub Actions CI** at `.github/workflows/ci.yml` — `cargo test`,
  `cargo clippy -- -D warnings`, `cargo fmt --check`, and an MSRV
  matrix row pinned at the workspace `rust-version` (1.85 at M6,
  bumped to 1.88 in v0.1.0 when `time = 0.3.47` required it).
- **Raspberry Pi deployment guide** at [`docs/raspberry-pi.md`](docs/raspberry-pi.md),
  validated end-to-end on a Pi Zero 2 W with an RTL-SDR dongle.
  Cross-compile via `cargo-zigbuild` targeting
  `aarch64-unknown-linux-gnu`; measured CPU 13–17 % of one core,
  resident memory ~5 MB. The original Pi Zero W (ARMv6) is still
  the design target and remains unvalidated; status is documented.
- **Development runbook** at [`docs/development.md`](docs/development.md) —
  local build, test, format, lint workflow plus the pre-push
  checklist that CI mirrors.

### Changed

- **`Icao` field is private.** Construct via `from_bytes`, `from_u24`
  (validates), `from_hex`, or `Icao::ZERO`. Access the raw 24-bit
  value via `as_u24()`. The "high byte always zero" invariant is now
  enforced by construction instead of by comment.
- **Frame layer is now allocation-free in the steady state.**
  `FrameDetector` owns its magnitude scratch buffer (sized in
  `new()` / `with_chunk_capacity`); `process` clears and reuses it.
  The `process_is_zero_allocation_in_steady_state` test pins the
  invariant.
- **Tracker is `LinkedHashMap<Icao, Aircraft, FxBuildHasher>`** —
  O(1) LRU touch on every ingest, O(1) eviction at capacity, O(k)
  stale-prune from the LRU end, and the address-XOR active-set scan
  iterates MRU → LRU so it exits the moment it crosses the
  active-ICAO window instead of scanning the whole table.
- **`SurveillanceReply` variant** carries the original `Frame`
  instead of a pre-rendered hex string. Encoding decisions now live
  at the serialisation boundary.
- **Public API surface narrowed**: `demod` and `magnitude` are
  `pub(crate)`. A new `pub mod test_utils` (gated on the `test-utils`
  feature) re-exports the small set of helpers benches, fuzz targets,
  and integration tests need.
- **Read-only public types are `#[non_exhaustive]`** — `Aircraft`,
  `Counters`, `TimedPosition`, `PositionSource`, `Altitude`, and the
  10 wire-format / snapshot types in `rs1090-serve`. Adding fields no
  longer requires a semver-major bump.
- **`SnapshotPosition.source` and `AircraftPosition.source`** are now
  `PositionSource` enums with a custom serializer, replacing the
  stringly-typed `&'static str`. The wire format is unchanged.

### Fixed

- **Ground speed truncates instead of rounds.** `decode_velocity`
  previously did `(ew_f.hypot(ns_f)).round() as u16`; the differential
  test harness's first run flagged 40 disagreements with pyModeS,
  every one off by exactly +1 kt. Changed to `.trunc()` to match
  pyModeS and the `(int)sqrt(...)` C convention. DO-260B doesn't
  specify; the practical difference is bounded by 1 kt. Pinned with
  `ground_velocity_truncates_speed_fraction` using ME bytes from a
  real frame in the corpus.

### Removed

- `parse_icao` function in `rs1090-serve` (replaced by
  `Icao::from_hex` on the type itself).
- `NoiseFloor::current` from production builds (only the test
  module called it; gated on `#[cfg(test)]`).

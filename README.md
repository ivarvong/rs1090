# rs1090

[![ci](https://github.com/ivarvong/rs1090/actions/workflows/ci.yml/badge.svg)](https://github.com/ivarvong/rs1090/actions/workflows/ci.yml)
[![license: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![rust 1.85+](https://img.shields.io/badge/rust-1.85%2B-orange.svg)](https://www.rust-lang.org)

A Mode S / ADS-B decoder in Rust. Library-first, real-time, runs on
anything from a Raspberry Pi to a beefy x86_64 server. `unsafe` is
forbidden workspace-wide.

**Status:** pre-alpha. Pipeline is end-to-end working: library decodes
real signals; CLI captures and replays; HTTP/SSE server fans the
decoded stream out to a live web map. 104 tests passing, clippy clean
under `-D warnings` pedantic, libFuzzer crash-free across three
targets, differential-tested against pyModeS, validated on a Pi Zero 2 W.

The design rationale lives in [DESIGN.md](./DESIGN.md); the runbooks
below cover every reproducible workflow.

## Layout

- `crates/rs1090` — the decoder library: sample sources, magnitude,
  demod, CRC, frame detector, message decoder, CPR, state tracker.
  `no_std`-friendly behind the default `std` feature; the `rtl-sdr`
  feature gates a live USB backend via pure-Rust `nusb`.
- `crates/rs1090-cli` — `replay`, `track`, and `live` subcommands.
- `crates/rs1090-serve` — HTTP server with `/aircraft`,
  `/aircraft/:icao`, `/healthz`, `/stream` (Server-Sent Events), and
  `/` (live Leaflet map).
- `scripts/` — `diff_pymodes.py` differential-test harness,
  `s8_to_uc8` Cargo example for UC8 conversion.

## Quick start

```sh
# Replay a capture and print one line per detected frame.
cargo run --release -p rs1090-cli -- replay path/to/capture.iq

# Same input, aggregated per-aircraft into state-tracker events.
cargo run --release -p rs1090-cli -- track path/to/capture.iq --summary

# Live from an RTL-SDR dongle (plug it in first).
cargo run --release -p rs1090-cli -- live --duration-secs 30

# HTTP + SSE server with a live web map at http://127.0.0.1:8080
cargo run --release -p rs1090-serve -- live
```

The `--record capture.iq` flag on `live` tees the bias-subtracted
USB stream to disk for deterministic later replay. The `--reference
LAT,LON` global flag on `rs1090-serve` enables local CPR decode when
no even/odd pair is available — faster first fix from a cold start.

## Wire format

Each SSE event is a JSON object with versioned schema and SI units:

```
event: position
id: 88
data: {"event":"position","v":1,"t":"2026-05-15T23:02:30.036384Z",
       "icao":"A0BA4E","lat":40.708648681640625,"lon":-73.92547607421875,
       "alt_ft":3625,"alt_source":"baro","source":"global"}
```

See DESIGN.md §12 for the full conventions.

## Stream filters

`GET /stream` accepts query parameters that compose. Bbox and altitude
filters apply to the *aircraft* the event refers to (looked up via the
current snapshot), so they work uniformly across `position`, `velocity`,
`identification`, and `lost` events.

| Param | Example | Effect |
|-------|---------|--------|
| `type` | `?type=position,velocity` | comma-separated event tags |
| `icao` | `?icao=A0BA4E,AC92E1` | comma-separated 6-hex ICAO addresses |
| `bbox` | `?bbox=40.0,-74.4,41.0,-73.5` | `min_lat,min_lon,max_lat,max_lon`. Aircraft without a resolved position are filtered out. |
| `alt_min` | `?alt_min=20000` | minimum altitude in feet (inclusive). Aircraft with no known altitude are filtered out. |
| `alt_max` | `?alt_max=5000` | maximum altitude in feet (inclusive) |

Compose freely:

```sh
# Only position events for aircraft in NYC metro below 5,000 ft
curl -sN 'http://<host>:8080/stream?type=position&bbox=40,-74.4,41,-73.5&alt_max=5000'

# Anything from one specific airframe
curl -sN 'http://<host>:8080/stream?icao=A0BA4E'

# High-altitude cruise traffic only
curl -sN 'http://<host>:8080/stream?alt_min=30000'
```

## Talking to the rest of the ecosystem

`rs1090-serve` can broadcast its decoded traffic in formats other
aviation/SDR tools understand, alongside the HTTP/SSE feed:

- **GDL90 over UDP** — `--gdl90` broadcasts to `255.255.255.255:4000`,
  or `--gdl90-target IP:PORT` unicasts. iPad / iPhone EFB apps
  (ForeFlight, Garmin Pilot, FlyQ) auto-discover and render the
  traffic on their moving map. No custom mobile code required —
  the EFB *is* the UI.
- **BLE GATT peripheral** — `--ble` (Linux + `ble` feature) advertises
  rs1090 as a Bluetooth peripheral with a custom service exposing
  aircraft count, nearest-aircraft summary, and a UTF-8 one-liner.
  iPhone running nRF Connect connects and subscribes, no app needed.
  See [`docs/raspberry-pi.md`](docs/raspberry-pi.md#ble-peripheral-optional).

## Documentation

| Topic | Doc |
|-------|-----|
| Architecture and design rationale | [`DESIGN.md`](./DESIGN.md) |
| Local development workflow | [`docs/development.md`](docs/development.md) |
| Differential testing against pyModeS | [`docs/differential-testing.md`](docs/differential-testing.md) |
| Fuzzing with cargo-fuzz | [`docs/fuzzing.md`](docs/fuzzing.md) |
| Cross-compile + deploy to a Raspberry Pi | [`docs/raspberry-pi.md`](docs/raspberry-pi.md) |
| Release history | [`CHANGELOG.md`](./CHANGELOG.md) |

## License

MIT. See [LICENSE](./LICENSE).

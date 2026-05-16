# rs1090

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

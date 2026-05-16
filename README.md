# rs1090

A Mode S / ADS-B decoder in Rust. Library-first, real-time, runs on anything
from a Raspberry Pi Zero W to a beefy x86_64 server.

**Status:** pre-alpha, but the pipeline is end-to-end working. Library
decodes real signals; CLI captures and replays; HTTP/SSE server fans the
decoded stream out to consumers. 99 tests passing, clippy clean, unsafe
forbidden. See [DESIGN.md](./DESIGN.md) for the architecture.

## Layout

- `crates/rs1090` — the decoder library: sample sources, magnitude, demod,
  CRC, frame detector, message decoder, CPR, state tracker. `no_std`-friendly
  behind the default `std` feature; `rtl-sdr` feature gates a live USB
  backend via pure-Rust `nusb`.
- `crates/rs1090-cli` — `replay`, `track`, and `live` subcommands.
- `crates/rs1090-serve` — HTTP server with `/aircraft`, `/aircraft/:icao`,
  `/healthz`, and `/stream` (Server-Sent Events). Filters by ICAO and
  event type.

## Quick start

```sh
# Decode an .iq file (interleaved signed 8-bit I/Q) and print per-frame lines.
cargo run --release -p rs1090-cli -- replay path/to/capture.iq

# Aggregate by aircraft and print state-tracker events.
cargo run --release -p rs1090-cli -- track path/to/capture.iq --summary

# Live capture from an RTL-SDR (requires the dongle plugged in).
cargo run --release -p rs1090-cli -- live --duration-secs 30

# HTTP server. SSE stream at /stream, JSON snapshot at /aircraft.
cargo run --release -p rs1090-serve -- live
curl -sN http://127.0.0.1:8080/stream
curl -s  http://127.0.0.1:8080/aircraft | jq
```

The `rs1090 live --record capture.iq` flag tees the live USB stream into a
file so you can replay it deterministically later.

## Wire format

Each SSE event is a JSON object with versioned schema and SI units:

```
event: position
id: 88
data: {"event":"position","v":1,"t":"2026-05-15T23:02:30.036384Z",
       "icao":"A0BA4E","lat":40.708648681640625,"lon":-73.92547607421875,
       "source":"global"}
```

See DESIGN.md §12 for the full conventions.

## Testing

```sh
cargo test --workspace        # 100 unit + integration tests
cargo clippy --workspace --all-targets -- -D warnings

# Differential test of the decoder against pyModeS (requires Python).
scripts/diff_pymodes.py corpus/<some>.iq

# libFuzzer harness for the message decoder (requires nightly +
# cargo-fuzz). Seeded with real frames; runs at ~1M exec/s on M-series.
cd crates/rs1090/fuzz
mkdir -p corpus/decode_message
cp -n seeds/decode_message/* corpus/decode_message/   # first run only
cargo +nightly fuzz run decode_message
```

## License

MIT. See [LICENSE](./LICENSE).

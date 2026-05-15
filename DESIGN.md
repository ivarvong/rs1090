# rs1090 — A Mode S / ADS-B Decoder in Rust

**Status:** Draft v0.1 · **Owner:** TBD · **License (target):** MIT

---

## 0. TL;DR

We are building a from-scratch Rust library and CLI that ingests 1090 MHz I/Q samples from a software-defined radio, demodulates Mode S frames, decodes ADS-B messages, and emits a structured stream of aircraft state. The binding constraint is the **Raspberry Pi Zero W** (BCM2835, ARMv6, single core, 1 GHz, no NEON, 512 MB RAM). If it runs there with headroom, it runs anywhere we care about.

The deliverable is one repo, one library crate (`rs1090`), one CLI crate (`rs1090-cli`), and a reproducible CI matrix that proves the claims on the README. The success bar is: a stranger reads this doc and the tests and feels they understand the system and trust the numbers.

---

## 1. Goals and Non-Goals

### Goals

1. **Correctness first.** Decoded messages must match `dump1090-fa` on the same I/Q recording to within marginal-SNR noise, with a documented diff methodology.
2. **Library-first design.** A `no_std`-friendly demodulator and decoder, with `std`-gated SDR sources. Downstream tools (web UI, network exporters, custom dashboards) compose against the library.
3. **Cross-platform from day one.** macOS (aarch64, x86_64), Linux (x86_64, aarch64, armv6, armv7). CI matrix proves it.
4. **Real-time on Pi Zero W** at 2.0 MS/s with ≥30% CPU headroom on a single core.
5. **Deterministic and reproducible.** Every bug reducible to a recorded I/Q clip plus a seed. No nondeterminism in tests.
6. **Minimal external surface.** `tokio`-free in the library. The hot path uses sync I/O and a bounded ring buffer. The CLI can wrap async if it wants.
7. **Network output is a first-class feature.** Decoded messages stream over **SSE** (Server-Sent Events) with versioned JSON, server-side filters, and `Last-Event-ID` resume. Designed in §12.
8. **Open source, MIT.** Public benchmarks, replay corpora, methodology docs.

### Non-Goals (v0.x)

- Mode A/C decoding, TIS-B/ADS-R synthesis, uplink (UF) handling.
- Beast binary, AVR, and SBS-1 protocols. Deferred to sibling crates (`rs1090-beast` etc.) for ecosystem interop with existing feeders and tools like `tar1090`. SSE is the primary network output.
- GUI / map / web UI.
- Multi-receiver fusion or MLAT.
- Replacing FlightAware's commercial stack. We are building the *core* well.

---

## 2. Background (just enough)

**ADS-B** is a cooperative surveillance system. Aircraft transmit position, velocity, and identification at 1090 MHz roughly twice per second. The physical layer is **Mode S**: pulse-position modulation (PPM) at 1 Mbit/s with an 8 µs preamble.

A Mode S downlink frame is 56 or 112 bits. The first 5 bits are the **downlink format (DF)**. The last 24 bits are a CRC. For DF 17/18 (ADS-B extended squitter), the CRC is clean — the syndrome should be zero. For DF 4/5/11/20/21, the CRC is XORed with the ICAO 24-bit address, which means CRC validation depends on prior knowledge of the address (chicken-and-egg on cold start; handled by DF 17/18 messages, which carry the address in the clear).

ADS-B position uses **Compact Position Reporting (CPR)**: a clever encoding that halves the bits at the cost of needing two consecutive frames (even and odd) for an unambiguous global decode, or one frame plus a prior known position for a local decode. CPR has known gotchas around the NL (number-of-longitude-zones) table boundaries.

References at the bottom of the doc. The single best background source is Junzi Sun's *The 1090 MHz Riddle* (free online); we will not re-derive the spec.

---

## 3. Meta-Design: How We Think About This

This is the part that earns the doc its keep. Features are easy; *engineering posture* is not. Five principles, in priority order:

**P1 — Layers with hard interfaces.** Every stage in the pipeline (SDR → samples → bits → frames → messages → state) is a pure function or an `Iterator`/`Stream` with a small, typed interface. Each layer is independently testable from a file source. Bugs are localized by construction.

**P2 — The hot path allocates nothing.** The demodulator processes ~2M samples/sec on a Pi Zero W. We verify zero allocation with `dhat` in a steady-state bench and treat allocation in the hot path as a release-blocking bug. Buffers are owned, reused, and sized once at startup.

**P3 — Every bug is a recording.** When something misbehaves in the field, the answer is "send me the I/Q capture and the seed." We will provide a `record` subcommand that dumps raw samples plus metadata, and a `replay` mode that produces bit-identical output. CI runs against a corpus of recordings; no live SDR required to reproduce.

**P4 — Confidence is a first-class output.** A real receiver lives in marginal SNR. We do not present "we decoded this frame" as a boolean. Each message carries a confidence score (per-bit slicer ratio aggregated) and a CRC-corrected flag. Downstream consumers decide their own thresholds.

**P5 — The spec is the tests.** Property tests for CRC roundtrips, CPR encode/decode invariants, message field bounds. Vector tests for every DF/TC combination we claim to support. Comparison tests against `dump1090-fa` on a shared corpus. If a behavior isn't tested, we don't claim it.

A non-principle worth noting: we are not chasing peak throughput. The Pi Zero W is the constraint, and "fast enough on a Pi Zero W with headroom" is a stronger result than "fastest on a Threadripper."

---

## 4. Performance Budget

This is the most important section. We do this *before* writing code.

Pi Zero W (BCM2835, ARMv6, 1 GHz):

| Quantity                    | Value                                  |
|-----------------------------|----------------------------------------|
| Sample rate (RTL-SDR)       | 2.0 MS/s I/Q, u8 pairs                 |
| Raw bandwidth               | 4 MB/s into userspace                  |
| Cycles per second           | ~1.0 × 10⁹                             |
| Cycles per sample (budget)  | **~500 cycles** worst case             |
| L1 D-cache                  | 16 KB                                  |
| L2 cache                    | 128 KB shared                          |
| SIMD                        | None (no NEON on ARMv6)                |

Five-hundred cycles per sample is tight but feasible. The magnitude step is the dominant cost. We use the **alpha-max-plus-beta-min** approximation: `mag ≈ max(|I|,|Q|) + 0.5·min(|I|,|Q|)`. No multiply, no sqrt, branchless on ARMv6. Continuous-math peak error ~11.8% at `|min|/|max| = 0.5`; the classic 3.96% figure applies to the multiplicative variant `α = 15/16, β = 15/32`, which costs a multiply we'd rather avoid on ARMv6. The approximation is used only to feed an adaptive threshold downstream, so the relative bias doesn't bleed into bit decisions. We *also* maintain a 128KB precomputed magnitude lookup table keyed on the `(I, Q)` byte pair (since RTL-SDR samples are 8-bit) — this fits in L2 on x86_64 and turns the magnitude step into a single load. We benchmark both and pick per platform.

The preamble correlator and bit slicer operate on the magnitude stream at 2 samples/bit. Their per-sample cost is well under the magnitude step's.

**Allocation budget:** zero in the hot path. All buffers preallocated. Verified by `dhat` in CI.

**Memory budget:** under 16 MB resident for the library, under 32 MB for the CLI including the aircraft state table (LRU-bounded at 4096 entries by default).

If we cannot hit these numbers on Pi Zero W, the fallback is **Pi Zero 2 W** (ARMv7-A with NEON), which is a much easier target. We document this honestly rather than pretend.

---

## 5. Architecture

```
┌─────────────────────────────┐
│ Network Outputs             │  SSE server (rs1090-serve); future: Beast
├─────────────────────────────┤
│ State Tracker               │  aircraft table, CPR pairing, dedup, LRU
├─────────────────────────────┤
│ Message Decoder             │  DF dispatch, TC dispatch, typed messages
├─────────────────────────────┤
│ Frame Detector              │  56/112-bit frames, CRC, 1-bit correction
├─────────────────────────────┤
│ Demodulator                 │  preamble detect, bit slice, confidence
├─────────────────────────────┤
│ Magnitude Stage             │  I/Q (u8,u8) → mag (u16)
├─────────────────────────────┤
│ Sample Source               │  trait; backends: RTL-SDR, file, SoapySDR
└─────────────────────────────┘
```

Each layer:

- Has a single typed input and a single typed output.
- Owns its buffers; no shared mutable state between layers.
- Has a `no_std`-compatible variant where feasible (the I/O layers are `std`).
- Has its own benchmark and its own fuzz target.

The library exposes each layer publicly so advanced users can swap one (e.g., bring their own demodulator for an SDRplay at 8 MS/s). The top-level `Decoder` is a thin convenience that wires the default stack.

---

## 6. SDR Abstraction

```rust
pub trait SampleSource {
    fn sample_rate(&self) -> u32;
    fn center_freq(&self) -> u32;
    /// Read I/Q samples into the buffer. Returns the number of samples written.
    /// Blocks until at least one sample is available or returns an error.
    fn read(&mut self, out: &mut [Complex<i8>]) -> io::Result<usize>;
}
```

Notes:

- We use `Complex<i8>` not `(u8, u8)`. RTL-SDR delivers unsigned 8-bit samples centered at 127; the backend subtracts the bias once and we work in signed space everywhere downstream.
- No async in the trait. The RTL-SDR backend uses its own internal USB transfer thread (the underlying C library is callback-based) and feeds a single-producer single-consumer ring buffer. Reads pop from that ring.
- The trait is sync because Pi Zero W has no useful async story for this workload, and we don't want to color the library with `async fn` for nothing.

**Backends:**

| Backend     | Feature flag | Default | Notes                                  |
|-------------|--------------|---------|----------------------------------------|
| `RtlSdr`    | `rtl-sdr`    | yes     | Primary. C dep, but well-supported.    |
| `File`      | none         | yes     | Replay `.iq` files, deterministic.     |
| `SoapySdr`  | `soapy`      | no      | Off by default. C deps balloon.        |
| `Null`      | `test-utils` | no      | Generates synthetic frames for tests.  |

Rationale for default-off SoapySDR: it pulls a significant C toolchain that hurts the cross-compile-to-Pi experience for users who only have an RTL-SDR dongle. Users with HackRF/Airspy turn it on explicitly.

---

## 7. Demodulation Pipeline

### 7.1 Magnitude

Two implementations, benchmarked, pick winner per platform at build time:

1. **Alpha-max-beta-min**: `mag = max + (min >> 1)`. Branchless on ARMv6. ~4 cycles/sample.
2. **256×256 LUT**: `mag = LUT[i_byte][q_byte]`. One load. ~3 cycles/sample but 128 KB cache pressure (fits L2 alone, contends with everything else).

Expectation: alpha-max-beta-min wins on Pi Zero W (cache-bound), LUT wins on x86_64. Both produce `u16` magnitudes.

### 7.2 Preamble Detection

The Mode S preamble has pulses at 0.0, 1.0, 3.5, 4.5 µs with low between. At 2 MS/s that's a specific 16-sample pattern. We use a sliding correlator: at each sample, compute the difference between the expected "high" sample sums and "low" sample sums. A preamble candidate is declared when the score exceeds an adaptive threshold (moving noise floor × constant).

Adaptive noise floor: an exponential moving average of magnitude with τ ≈ 100 µs. Keeps the receiver useful across SNR variations.

### 7.3 Bit Slicing

For each bit (1 µs = 2 samples at 2 MS/s), the PPM rule is: pulse-in-first-half = 1, pulse-in-second-half = 0. We compute the difference of the two half-sums and:

- The sign gives the bit.
- The absolute value normalized by the bit's energy gives a per-bit confidence in `[0, 1]`.

Aggregated confidence per frame is the geometric mean of per-bit confidences. This drives the message confidence field.

### 7.4 Frame Length Determination

The DF field is bits 0–4 of the frame. DF determines length (56 or 112 bits). We always read 5 bits, look up the length, and read the rest. If the DF is reserved/unused, we drop the frame early.

---

## 8. CRC and Error Correction

Mode S CRC-24, polynomial `0xFFF409`. Implementation:

- Table-driven byte-at-a-time on x86_64 and aarch64.
- Bit-at-a-time on armv6 (table thrashes L1; benchmark to confirm).

**Error correction:** we support 1-bit correction by precomputed syndrome table (length-112 frames have a 112-entry syndrome lookup; same for 56). 2-bit correction is implemented but **off by default** — it produces too many false positives in noisy environments. Users opt in per layer.

For DF 11 and DF 17/18 the CRC is clean. For DF 4/5/20/21 the CRC is XORed with the ICAO address; we validate by trying known-active ICAOs from the state tracker (bounded set, fast). Unmatched frames are dropped or surfaced as "unverified" depending on a config flag.

We surface three CRC outcomes: `Clean`, `Corrected(bits: u8)`, `Failed`.

---

## 9. Message Decoding

Two-level dispatch. Outer: DF. Inner (for DF 17/18): Type Code (TC).

We support, in v0.1:

| DF | Name                       | Notes                              |
|----|----------------------------|------------------------------------|
| 0  | Short ACAS                 | Decoded, not validated (no addr)   |
| 4  | Altitude reply             | Address-XOR CRC                    |
| 5  | Identity reply             | Address-XOR CRC                    |
| 11 | All-Call reply             | Discovers ICAO address             |
| 16 | Long ACAS                  | Decoded, not validated             |
| 17 | ADS-B Extended Squitter    | **Primary**                        |
| 18 | TIS-B / non-transponder    | Same payload as 17                 |
| 20 | Comm-B altitude reply      | Address-XOR CRC, MB field          |
| 21 | Comm-B identity reply      | Address-XOR CRC, MB field          |

For DF 17/18, TC dispatch covers: aircraft identification (TC 1-4), surface position (TC 5-8), airborne position (TC 9-18, 20-22), airborne velocity (TC 19), operational status (TC 31). Less common TCs are decoded into a raw payload struct for users to handle.

Decoded messages are `#[non_exhaustive]` enums with typed variants. No `String` allocation in the decoder — callsigns are `arrayvec::ArrayString<8>`.

---

## 10. State Tracker

Keyed on ICAO 24-bit address. Per-aircraft state:

- Latest position (with CPR pairing state: `Even(t, lat, lon)` / `Odd(t, lat, lon)` / `Resolved(t, lat, lon)`)
- Latest velocity
- Identification (callsign, category)
- Squawk
- Counters: messages, CRC clean/corrected/failed, marginal-confidence
- `last_seen: Instant`

Bounded by LRU at a configurable capacity (default 4096). On eviction we emit a `Lost` event so downstream consumers can clean up.

**CPR:** global decode requires an even/odd pair within 10 seconds and within reasonable distance (the NL table is the trap). Local decode requires a prior reference position within 180 NM. We implement both, prefer global, fall back to local. The NL table is a `const` array, not computed at runtime.

---

## 11. Public API Sketch

```rust
use rs1090::{Decoder, source::RtlSdr, Message};

fn main() -> anyhow::Result<()> {
    let source = RtlSdr::open(0)?
        .with_sample_rate(2_000_000)?
        .with_gain(rs1090::Gain::Auto)?;

    let mut decoder = Decoder::new(source);

    while let Some(event) = decoder.next()? {
        match event.message {
            Message::AirbornePosition(p) => println!(
                "{:06X} {:.4},{:.4} alt={}ft conf={:.2}",
                event.icao, p.lat, p.lon, p.altitude_ft, event.confidence
            ),
            Message::Velocity(v) => { /* ... */ }
            _ => {}
        }
    }
    Ok(())
}
```

Library users who want lower-level access can construct each stage manually.

---

## 12. Network Output (Server-Sent Events)

A separate crate `rs1090-serve` exposes the decoded stream over HTTP. The library proper stays sync and `tokio`-free; the server is where async lives.

### 12.1 Why SSE

The dump1090 ecosystem standardized on the **Beast binary protocol** — efficient, but a 1990s artifact that needs custom parsers in every language. Anything modern (a browser, a Grafana dashboard, a Python notebook, a Go service, an LLM agent) wants HTTP + JSON.

SSE is the right primitive for this workload:

- Unidirectional server→client streaming. We have one writer, many readers.
- Plain HTTP/1.1. Traverses every proxy and firewall on the planet.
- Built-in reconnection via `Last-Event-ID`. We don't reinvent it.
- Debuggable with `curl localhost:8080/stream`. No client library required.
- No framing protocol, no schema compiler, no `.proto` files.

Beast support is **not dropped** — it moves to a sibling crate `rs1090-beast` for compatibility with existing feeders (FlightAware, tar1090, etc.). SSE is the front door for everyone who isn't already in that ecosystem.

### 12.2 Wire Format

Endpoint: `GET /stream` returns `text/event-stream`. Each decoded message becomes one SSE event. Event names map to message types:

```
event: position
id: 12847
data: {"v":1,"t":"2026-05-15T18:42:01.123Z","icao":"A1B2C3","lat":40.6413,"lon":-73.7781,"alt_m":3048,"src":"baro","conf":0.94,"crc":"clean"}

event: velocity
id: 12848
data: {"v":1,"t":"2026-05-15T18:42:01.456Z","icao":"A1B2C3","gs_mps":231.5,"track_deg":87.2,"vr_mps":-5.1,"conf":0.91,"crc":"clean"}

event: identification
id: 12849
data: {"v":1,"t":"2026-05-15T18:42:02.001Z","icao":"A1B2C3","callsign":"DAL2104","category":"A3"}

event: lost
id: 12850
data: {"v":1,"t":"2026-05-15T18:42:30.000Z","icao":"A1B2C3","reason":"timeout","last_seen":"2026-05-15T18:42:00.000Z"}
```

Conventions, enforced by golden tests:

- **SI units only.** Meters, m/s, degrees, UTC timestamps in RFC 3339. No feet, no knots, no local time. Consumers convert at the edge.
- **Versioned schema** (`"v": 1`). Breaking changes bump the version and live in parallel for one release before the old version is removed.
- **Stable field names.** Adding fields is non-breaking; renaming or removing is breaking and requires a version bump.
- **Monotonic `id`.** A `u64` counter, used by clients to resume via `Last-Event-ID`.

### 12.3 Endpoints

| Method | Path                                | Description                                       |
|--------|-------------------------------------|---------------------------------------------------|
| GET    | `/stream`                           | SSE stream of all events                          |
| GET    | `/stream?icao=A1B2C3`               | Filtered: single aircraft (repeatable)            |
| GET    | `/stream?bbox=lat1,lon1,lat2,lon2`  | Filtered: bounding box (position events only)     |
| GET    | `/stream?type=position,velocity`    | Filtered: event types                             |
| GET    | `/aircraft`                         | JSON snapshot of current state                    |
| GET    | `/aircraft/A1B2C3`                  | JSON snapshot for one aircraft                    |
| GET    | `/healthz`                          | Liveness                                          |
| GET    | `/metrics`                          | Prometheus exposition                             |

Standard client pattern: `GET /aircraft` once for the current world, then `GET /stream` for deltas. Filters compose (`?bbox=...&type=position,velocity`). Server-side filtering means low-bandwidth clients don't have to receive everything.

### 12.4 Reconnection and Replay

The server maintains a bounded ring buffer of recent events (default: last 60 seconds *or* 10,000 events, whichever fires first). On reconnect with `Last-Event-ID: N`:

- If `N` is in the buffer, replay events `N+1..` then continue live.
- If `N` is older than the buffer, emit one `event: gap` with `{"missed": count}`, then resume live. The client can re-fetch `/aircraft` if it needs state continuity.

This is good enough for the realistic failure modes (transient network blips, mobile handoffs) without unbounded server memory.

### 12.5 Backpressure

Each SSE connection has a bounded queue (default 1,024 events). When full:

- **Drop oldest.** Recent state is more valuable than ancient state.
- Increment `client_drops_total{client_id}` and `client_queue_full_total`.
- Send one `event: dropped` with `{"count": n}` so the client knows it's behind.

Slow clients **cannot** block the decode pipeline. The decode-to-broadcaster handoff is a single non-blocking `try_send` into each subscriber's queue. The decode thread never awaits anything.

### 12.6 Threading Model

```
[decode thread] ─sync─▶ [crossbeam channel] ─▶ [tokio runtime: axum + broadcaster]
                                                       │
                                                       ├─▶ SSE conn 1
                                                       ├─▶ SSE conn 2
                                                       └─▶ SSE conn N
```

The decode thread is sync, runs at OS-default priority, owns the SDR. It does `try_send` into a `crossbeam-channel` and never blocks. The `tokio` runtime hosts `axum`, the broadcaster, and all SSE connections, on a small thread pool (2 threads on Pi Zero W, default on bigger hosts). This is the **only** place `tokio` enters the project — the library stays sync, as designed.

The broadcaster owns:

- Current state snapshot (copy-on-write `Arc<AircraftMap>`, swapped atomically on each update; `/aircraft` requests clone the `Arc`).
- List of subscribers, each with its own bounded queue and filter predicate.
- The replay ring buffer.

### 12.7 Configuration

```toml
[server]
bind = "127.0.0.1:8080"          # default localhost; explicit opt-in for public
auth_token = ""                   # if set, required as Bearer; empty = no auth
replay_buffer_seconds = 60
replay_buffer_max_events = 10_000
client_queue_size = 1024
heartbeat_seconds = 15
cors_origins = []                 # empty = no CORS headers; "*" or specific origins allowed
```

Heartbeats are SSE comments (`: heartbeat\n\n`) — invisible to consumers but keep proxies and load balancers from killing idle connections.

### 12.8 Security Posture

- **Default bind is `127.0.0.1`.** The user must explicitly choose a public bind address. We log a `WARN` on startup when bound to anything other than loopback.
- **Bearer token auth is optional and off by default.** Documented reverse-proxy patterns (caddy, nginx) handle TLS; we don't terminate TLS ourselves.
- **No write endpoints exist.** There is nothing for an attacker to modify, only read.
- **CORS defaults closed.** Set `cors_origins` explicitly. We never echo arbitrary `Origin` headers.
- **No PII.** ICAO addresses and callsigns are public broadcast data, not personal information, but we document this clearly so users in stricter jurisdictions can make informed choices.

### 12.9 Performance Cost

Network output is cheap. Decode produces 10–50 messages/sec in a busy airspace at roughly 400 bytes/event of JSON. Total output well under 50 KB/s per client. The `tokio` runtime + `axum` adds ~3–5 MB resident. On Pi Zero W we pin the network thread(s) to nice +5 so they cannot preempt decode. CI asserts decode CPU is unchanged with the server running and 10 connected clients.

### 12.10 Testing

- **Integration tests** spin up the server with a synthetic source, connect SSE clients, assert event sequences match expected.
- **Reconnection test:** kill the client mid-stream, reconnect with `Last-Event-ID`, assert no events lost within the replay window.
- **Backpressure test:** slow consumer (sleep between reads), assert decode-side throughput is unaffected and `dropped` event is emitted with the right count.
- **Schema golden test:** pin the JSON for representative events. Breaking the schema requires a version bump and an explicit golden update — caught in code review.
- **Filter tests:** every filter parameter, alone and in combination, with both included and excluded events asserted.

---

## 13. Testing Strategy

This section is the most direct evidence of engineering quality, and we put real effort into it.

### 13.1 Unit Tests

Standard. Every module. Run on `cargo test` with no features beyond `default`.

### 13.2 Property Tests (`proptest`)

- CRC: for any random 88-bit payload, CRC-then-verify yields zero.
- CRC: flipping any single bit causes failure or successful 1-bit correction.
- CPR: encode-then-decode round-trips within latitude/longitude quantization bounds.
- Magnitude: alpha-max-beta-min within ~11.8% of true magnitude where the minor component is above the integer-truncation regime (|min| ≥ 4); LUT exact to rounding.
- Bit slicer: synthetic PPM with known bits decodes back to those bits at SNR ≥ 6 dB.

### 13.3 Vector Tests

A YAML file of hand-curated Mode S frames from the literature (DO-260B Appendix, ICAO 9871) with expected decode. Each entry is a hex frame and the expected typed message. Easy to add to.

### 13.4 Replay Tests

A corpus of recorded `.iq` files (~30 seconds each) captured at known locations and times, with corresponding golden-output files generated by `dump1090-fa`. CI runs our decoder over the corpus and diffs.

Tolerance: an exact match on clean-CRC frames. For marginal-CRC frames we allow disagreement but require *symmetry* (we shouldn't systematically lose frames `dump1090-fa` catches at the same SNR).

The corpus lives in a separate repo with Git LFS so the main repo stays small.

### 13.5 Fuzz Tests (`cargo-fuzz`)

Two targets:

1. Frame decoder fed random bytes.
2. Full decoder fed random magnitude streams.

Goal: no panics, no UB, no allocations beyond a bounded budget. Run nightly in CI.

### 13.6 Differential Tests vs `dump1090-fa`

Beyond the replay corpus: a property-based test that generates synthetic I/Q (random ADS-B messages convolved with a known channel model) and runs both decoders. Disagreements are inspected manually and become either a bug fix or a known-divergence note.

### 13.7 Performance Tests

`criterion` benchmarks for each layer. A CI job on a self-hosted Pi Zero W runner asserts:

- 2.0 MS/s sustained for 60 seconds with CPU < 70%.
- No allocation in the hot path (verified by `dhat`).
- Median message-to-output latency < 5 ms.

Regression > 10% fails the build.

---

## 14. Observability

- `tracing` crate, with `tracing-subscriber` opt-in. The library *emits* but does not configure.
- Metrics counters exposed via a `metrics` trait the user implements (default: no-op). Optional `prometheus` exporter in a sibling crate.
- Counter set: `frames_total{result=clean|corrected|failed}`, `messages_total{df, tc}`, `bits_confidence_histogram`, `sample_buffer_fullness`, `aircraft_active`.

---

## 15. Build, Distribution, Reproducibility

- **MSRV** pinned and tested in CI. Bumps are SemVer-minor and noted in CHANGELOG.
- **`cross`** for the Pi Zero W (`arm-unknown-linux-gnueabihf` with `+v6`). The exact toolchain is in `Cross.toml` and pinned by digest.
- **CI matrix:** `{macOS-14 (arm64), macOS-13 (x86_64), ubuntu-22.04 (x86_64), ubuntu-22.04 (aarch64), ubuntu-22.04 (armv6)}`. The armv6 job runs on a self-hosted Pi Zero W, gated on a PR label to keep PR latency reasonable.
- **Releases:** GitHub Releases with pre-built binaries for each target. Checksums signed. `cargo install rs1090-cli` for source builds.
- **Reproducible:** `--locked` everywhere. `Cargo.lock` checked in for both crates.

---

## 16. Milestones

| M  | Window  | Deliverable                                                                 |
|----|---------|-----------------------------------------------------------------------------|
| M1 | wk 1-2  | File source + magnitude + demod + frame detector. Vector tests green.       |
| M2 | wk 3    | RTL-SDR backend. Real-time on Linux x86_64. Basic CLI.                      |
| M3 | wk 4    | Pi Zero W performance pass. Profile, optimize, document numbers.            |
| M4 | wk 5-6  | DF dispatch + ADS-B TC decoders. State tracker with CPR. Replay tests.      |
| M5 | wk 7    | `rs1090-serve`: SSE server, filters, reconnect, snapshot endpoint. Integration tests. |
| M6 | wk 8    | Differential tests vs `dump1090-fa`. Fuzz harness. Schema golden tests.     |
| M7 | wk 9    | Release `0.1.0`. README, benchmarks, blog post.                             |

Total: ~9 weeks at full focus. Realistic at half-time: ~4.5 months.

---

## 17. Risks and How We Retire Them

| Risk                                                                   | Likelihood | Retirement                                                |
|------------------------------------------------------------------------|------------|-----------------------------------------------------------|
| Pi Zero W can't hit 2 MS/s sustained without inline ASM or `unsafe`    | Medium     | M3 spike with profile-driven optimization; fallback to Pi Zero 2 W in claims if unrecoverable |
| CPR NL-table edge cases produce wrong positions                        | Medium     | Property tests against published reference implementation |
| RTL-SDR USB jitter on Pi causes sample drops                           | Medium     | Larger ring buffer, document the failure mode in metrics  |
| `dump1090-fa` differential test surfaces our bugs *and* theirs         | High       | Documented "known divergences" file with rationale        |
| Cross-compiling for armv6 breaks with toolchain updates                | Medium     | Pin Docker image by digest; nightly canary build          |
| `tokio` + `axum` on Pi Zero W contends with decode CPU                 | Low-Med    | Nice-level pinning; CI asserts decode throughput with N clients connected |
| SSE schema needs breaking changes more often than expected             | Medium     | Version field is mandatory; v1 and v2 live side-by-side for one release   |
| Replay buffer + N clients drives memory growth on small hosts          | Low        | Hard caps in config; metrics on memory; LRU eviction documented           |

---

## 18. Open Questions

1. **Async or sync at the library boundary?** *Resolved:* sync. Pi Zero W has no benefit from async here, and we don't want to color the library. The `tokio` runtime lives entirely in `rs1090-serve`, bridged from decode via a `crossbeam-channel`.
2. **Expose pre-CRC frames?** Current answer: yes, behind a `raw` flag in the iterator. Some users want their own error correction or to train ML models on raw bits.
3. **SoapySDR by default?** Current answer: no. Reconsider once RTL-SDR is rock-solid.
4. **`no_std` library?** Aspirationally yes, for the demodulator and decoder. The state tracker needs an allocator. Gate `std` behind a feature and see how much surface compiles without it.
5. **License audit of dependencies.** Must all be MIT/Apache-2.0/BSD compatible. No GPL anywhere in the dependency closure. (Note: `axum`, `tokio`, `hyper`, `serde` are all MIT/Apache-2.0 — clean.)
6. **WebSocket as an alternative to SSE?** Current answer: no. SSE is one-way and simpler; we'd only add WebSocket if a real consumer needs bidirectional, and we don't have one.

---

## 19. Why This Is Worth Building

Three reasons, in priority order:

1. **The existing implementations have aged.** `dump1090` and its forks are excellent C, but they fuse demodulation, decoding, state, networking, and a web UI into one binary with shared globals. A clean library-first Rust implementation gives the community a building block, not a monolith.
2. **It is an unusually good demonstration of systems engineering.** Real-time DSP, tight performance budget, hardware in the loop, a public spec to validate against, a known-good reference to diff against. Few projects offer all of that in one repo.
3. **The constraint set forces good taste.** Pi Zero W + zero-allocation + cross-platform + reproducible tests is a forcing function for the kind of layered, honest engineering that doesn't always survive contact with deadlines.

---

## 20. References

- RTCA DO-260B — *Minimum Operational Performance Standards for 1090 MHz Extended Squitter ADS-B*.
- ICAO Annex 10, Vol IV — *Surveillance and Collision Avoidance Systems*.
- ICAO Doc 9871 — *Technical Provisions for Mode S Services and Extended Squitter*.
- Sun, J. *The 1090 MHz Riddle: An Open-Access Book about Decoding Mode S and ADS-B Data*. https://mode-s.org/decode/.
- Sanfilippo, S. *dump1090*. The progenitor.
- FlightAware *dump1090-fa*. The current de-facto reference.
- Lyons, R. *Understanding Digital Signal Processing*, 3rd ed. — for the magnitude approximation and PPM background.

---

## Appendix A: Repo Layout

```
rs1090/
├── Cargo.toml                   workspace
├── crates/
│   ├── rs1090/                  library (sync, no tokio)
│   │   ├── src/
│   │   │   ├── lib.rs
│   │   │   ├── source/          SDR backends
│   │   │   ├── magnitude.rs
│   │   │   ├── demod.rs
│   │   │   ├── frame.rs         CRC, length
│   │   │   ├── decode/          DF/TC dispatch
│   │   │   ├── state.rs         tracker, CPR
│   │   │   └── confidence.rs
│   │   ├── benches/
│   │   ├── fuzz/
│   │   └── tests/
│   ├── rs1090-cli/              CLI (decode, record, replay)
│   └── rs1090-serve/            SSE server (tokio + axum)
│       ├── src/
│       │   ├── main.rs
│       │   ├── broadcaster.rs   subscriber list, replay buffer
│       │   ├── sse.rs           SSE encoding, heartbeats
│       │   ├── filters.rs       icao/bbox/type parsing
│       │   └── schema.rs        versioned JSON shapes
│       └── tests/               integration: synthetic source → SSE clients
├── corpus/                       (git submodule, LFS) recorded I/Q + golden output
├── DESIGN.md                     this doc
├── README.md
└── CHANGELOG.md
```

## Appendix B: What "done" looks like for v0.1

- All CI green on the full matrix including Pi Zero W.
- README shows real numbers: messages/sec on each platform, CPU usage, memory.
- Replay test corpus of ≥10 recordings, diff against `dump1090-fa` documented.
- `cargo doc` is publishable; every public item has at least one example.
- A 1500-word blog post explains the design with the same honesty as this doc.
- One external user has tried it on their own SDR and filed an issue or a star.

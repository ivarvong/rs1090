# Benchmarks

Microbenchmarks for the hot paths plus end-to-end CPU measurements
from real hardware. Numbers below are *headline values*; full
distributions and confidence intervals are in `criterion`'s HTML
output under `target/criterion/`.

## How to run

```sh
# All benches (release profile, test-utils for the bench-only helpers).
cargo bench -p rs1090 --features test-utils

# A specific bench, with criterion's progress output.
cargo bench -p rs1090 --features test-utils --bench magnitude
cargo bench -p rs1090 --features test-utils --bench crc
cargo bench -p rs1090 --features test-utils --bench decode

# Filter to a single benchmark by name substring.
cargo bench -p rs1090 --features test-utils -- alpha_max
```

## Microbenchmarks — Apple M-series (aarch64)

Hardware: Apple Silicon M-series, macOS 24, release profile
(`lto = "thin"`, `codegen-units = 1`).

### Magnitude (per 2,000-sample chunk = 1 ms of capture at 2 MS/s)

| Implementation | Time | Throughput | Per-sample |
|---|---|---|---|
| `alpha_max_beta_min` | 76.5 ns | **26.0 Gelem/s** | 38 ps |
| `lut` (128 KiB table) | 575 ns | 3.48 Gelem/s | 287 ps |

The branchless alpha-max-beta-min wins by ~7.5× on M-series — the
LUT lookup is dominated by cache traffic against the 128 KiB table
while the arithmetic form vectorises cleanly. The detector uses
alpha-max-beta-min unconditionally; the LUT remains in tree as a
benchmark reference and a future option for in-cache wins on
specific x86_64 SKUs.

At 2 MS/s the magnitude step alone costs **76 µs of CPU per second**
of capture (≈ 0.008 % of one M-series core).

### CRC-24 (Mode S generator `0xFFF409`, no reflection)

| Implementation | Frame size | Time | Throughput |
|---|---|---|---|
| Table-driven (1 KiB lookup) | 7 B (short) | 4.7 ns | **1.39 GiB/s** |
| Table-driven | 14 B (long) | 13.4 ns | 993 MiB/s |
| Bitwise (no table) | 7 B | 33.7 ns | 198 MiB/s |
| Bitwise | 14 B | 83.0 ns | 161 MiB/s |

The table-driven version is the hot path for every CRC-clean frame
and every address-XOR recovery attempt against the active set. The
bitwise variant exists for memory-starved targets where the 1 KiB
table would thrash L1; ~6× slower per byte. We cross-check the two
implementations against each other in tests.

### Message decode (`message::decode` on real captured frames)

| DF / TC | Time |
|---|---|
| DF 11 (all-call reply, 7 B) | 3.3 ns |
| DF 17 TC 11 (airborne position) | 4.6 ns |
| DF 17 TC 19 (airborne velocity, ground subtype) | 13.5 ns |

Pure CPU, no allocation. The `BitReader` helper accounts for the
velocity decode taking ~3× the position decode (more fields,
including the hypot for ground speed).

### Summary table

| Stage | Per-call cost | Throughput |
|---|---|---|
| Magnitude (1 ms @ 2 MS/s) | 76.5 ns | 26 Gelem/s |
| CRC long-frame | 13.4 ns | 993 MiB/s |
| Decode DF17 position | 4.6 ns | — |
| Decode DF17 velocity | 13.5 ns | — |

A busy receiver sees ~10–50 messages/sec; the per-frame decode +
CRC layers together cost a handful of microseconds per second of
wall-clock time. The whole pipeline is dominated by the magnitude
stage upstream, which is itself well under 0.01 % of one core.

## End-to-end on a Raspberry Pi Zero 2 W

Same binary, cross-compiled `aarch64-unknown-linux-gnu` and deployed
per [`raspberry-pi.md`](raspberry-pi.md). Live RTL-SDR (Realtek
RTL2832U + R820T tuner), antenna near a window in NYC.

| Metric | Value |
|---|---|
| CPU | **13–17 % of one core** (~3–4 % of the quad-core total) |
| Resident memory | ~5 MB |
| Virtual memory | ~344 MB (mostly mmap, not resident) |
| Aircraft tracked | 9 in 80 s, 4 with resolved positions |
| Frame throughput | ~100–200 Mode S messages/sec |
| SoC temperature | 43.5 °C under load, indoor ambient |

The Cortex-A53 in the Zero 2 W is ~5-10× slower per-clock than the
M-series core that benches above. The pipeline still has 80%+
headroom on a single A53 core.

## Original Pi Zero W (ARMv6) — unmeasured

`DESIGN.md` calls out the original Pi Zero W (BCM2835, ARMv6,
single-core, no NEON, 1 GHz) as the design target. It has not yet
been measured end-to-end. The most likely lever if we hit a CPU
ceiling there is swapping `magnitude::alpha_max_beta_min` for
`magnitude::lut`: the LUT loses by 7.5× on M-series because the
arithmetic vectorises, but on ARMv6 with no SIMD the table's "one
load per sample" pattern should win — and the 128 KiB table fits
comfortably in the Zero W's 128 KiB L2.

## Pinning numbers in CI

Criterion supports baseline comparisons (`--save-baseline`,
`--baseline`), but we don't gate CI on perf regressions today. The
benchmarks are reproducible — anyone can run them on a machine they
trust and compare. If we add a self-hosted ARM runner later, this
section gets a "regression > X% fails the build" line.

## What the benches don't yet cover

- **`FrameDetector::process`** end-to-end (magnitude → preamble →
  slicer → CRC over a synthetic chunk). Stateful — needs care to
  bench cleanly. Tracked under "future work".
- **`StateTracker::ingest`** including CPR pairing and address-XOR
  recovery against a populated active set. Same reason.
- **SSE serialization** under load. Pure I/O; not particularly
  illuminating microbench-wise.
- **GDL90 / Beast frame encoding**. Trivially fast (small `Vec`
  pushes); below the noise floor we'd measure.

Adding any of these is small; we'll do it the first time we touch a
hot path that crosses one of those layers.

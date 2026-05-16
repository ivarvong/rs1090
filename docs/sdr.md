# SDR pipeline

How the radio side of rs1090 actually works — from antenna, through
USB, to the bytes the decoder consumes. Covers the choices made in
`crates/rs1090/src/source.rs`, the file capture format, and the
parts of the Mode S physical layer that the rest of the codebase
silently relies on.

## The 1090 MHz physical layer

Mode S transponders transmit on **1090 MHz**, at peak powers
typically 70-500 W. Frames are **pulse-position modulation (PPM)
at 1 Mbit/s** — each 1 µs bit time has a pulse in either its first
or second half:

```
bit = 1 :  ▇▇▏        bit = 0 :     ▏▇▇
           0   1                    0   1   (µs)
```

A frame opens with an **8 µs preamble** with pulses at
`0.0, 1.0, 3.5, 4.5 µs`, the rest low:

```
       ▇   ▇         ▇   ▇
       0 1 2 3 4 5 6 7 8 9   (sample index at 2 MS/s)
```

After the preamble, the data field is either **56 bits** (DF ≤ 11,
"short Mode S") or **112 bits** (DF ≥ 16, "long Mode S" — ADS-B
extended squitter lives here). The high 5 bits of the data field
are the **DF (Downlink Format)** code, which also implies the
length.

The last 24 bits are a CRC. For DF 11/17/18 the CRC is clean (zero
syndrome). For DF 0/4/5/16/20/21 the CRC is XORed with the
aircraft's 24-bit ICAO address — you cannot validate the frame
without already knowing who sent it (see `state.rs`'s active-set
recovery).

At 2 MS/s sampling, one bit is 2 samples and one frame is
**112 + 16 = 128 sample positions for a long Mode S frame**. The
detector buffers 256 samples of carry between chunks so a frame
straddling a chunk boundary isn't lost.

## RTL2832U + R820T — what we use, and why

The cheap-and-cheerful SDR hardware on the open market is the
**Realtek RTL2832U** demodulator chip paired with the **Rafael
Micro R820T** silicon tuner (USB device ID `0bda:2838`).
Originally sold as DVB-T receiver dongles for European digital TV,
they expose a "raw I/Q" mode that's perfect for our use case.

| Capability | What we use |
|---|---|
| Tuning range | 24-1766 MHz (we sit at 1090 MHz) |
| Sample rates | 0.25-3.2 MS/s (we use **2 MS/s**) |
| ADC | 8-bit I and 8-bit Q, separately, ~50 dB dynamic range |
| Gain | R820T programmable, 29 discrete steps from 0.0 to 49.6 dB |
| Bandwidth | Filter rolloff well outside 2 MS/s |
| Bias-T | 5 V on the antenna port for active LNAs (off by default) |

We pick **2 MS/s** because it's the standard ADS-B sample rate
(2 samples per bit at 1 Mbit/s) and the slowest rate that still
gives the slicer enough timing margin against R820T's PLL jitter.
2.4 MS/s is also common and would work; 8 MS/s would let the
slicer recover better at marginal SNR but burns 4× the CPU.

**Gain matters more than antenna for short-range work.** At 1090 MHz
the wavelength is 27.5 cm, so a 6.8 cm quarter-wave whip is enough
for line-of-sight to anything in your local airspace. Outdoor
mounting with line-of-sight to the horizon is the biggest win,
followed by raising the gain to 40 dB (R820T step 16-17 of 29 —
the "sweet spot" before the front-end saturates on close-by
transmitters).

## Pure-Rust USB via `nusb` (not `librtlsdr`)

The textbook way to talk to an RTL-SDR is `librtlsdr` (a C library
that wraps `libusb`). rs1090 uses [`rs-rtl`](https://crates.io/crates/rs-rtl)
instead, which is built on [`nusb`](https://crates.io/crates/nusb)
— a **pure-Rust USB stack** that talks directly to the host's
USB APIs (IOKit on macOS, kernel uapi on Linux, WinUSB on Windows).

Concretely, the dep tree looks like:

```text
rs1090 (rtl-sdr feature)
  └─ rs-rtl       — RTL2832U register protocol + R820T tuner driver
       └─ nusb    — pure-Rust USB stack (no libusb)
            └─ OS USB APIs (IOKit / Linux uapi / WinUSB)
```

This buys us three things:

1. **Trivial cross-compile.** No C compiler or `*-sys` build script
   to fight. `cargo-zigbuild aarch64-unknown-linux-gnu` produces a
   binary that just works on a Pi.
2. **No system dependency at run-time.** The deployment artifact is
   a single ELF — no `libusb.so` to install.
3. **`unsafe`-free at the application level.** All the FFI risk is
   contained to `nusb`, which audits itself. rs1090 itself runs
   under `unsafe_code = "forbid"`.

The cost: smaller community than libusb-backed stacks. We accepted
it on the bet that compiles-anywhere is worth more than
battle-tested-for-a-decade.

### USB transfer pattern

RTL-SDR USB endpoints stream bulk-IN data at ~4 MB/s (2 MS/s × 2
bytes per sample). `rs-rtl` keeps **15 in-flight bulk transfers of
32 KiB each** by default — about 7.5 ms of buffered samples in the
USB pipeline. As soon as one completes, the next is submitted, so
the device never starves between submissions.

Completed chunks land on a bounded channel inside `rs-rtl`. Our
[`RtlSdrSource::read`](https://github.com/ivarvong/rs1090/blob/main/crates/rs1090/src/source.rs)
blocks on `recv()` and converts each unsigned byte to signed via
`wrapping_sub(128)`. That's the only conversion — no DC removal,
no AGC, no decimation. We work in `Iq { i: i8, q: i8 }` for the
rest of the pipeline.

## Sample format conventions

There are two competing conventions for 8-bit I/Q on RTL-SDR:

| Format | Encoding | Files use |
|---|---|---|
| **UC8** | unsigned 8-bit, biased at 127.5 | `dump1090 --iformat UC8` |
| **SC8** (we call it `i8`) | signed 8-bit, centered at 0 | rs1090's `.iq` format |

RTL2832U delivers UC8 to userspace. We subtract 128 (`wrapping_sub`)
on read and work in SC8 internally. **Files written by
`rs1090 live --record` are in SC8.**

To go back to UC8 for a third-party tool that wants it (e.g.
`dump1090`), use the converter example:

```sh
cargo build --release -p rs1090-cli --example s8_to_uc8
./target/release/examples/s8_to_uc8 < capture.iq > capture.uc8
```

It's an XOR of the high bit (`b ^= 0x80`) — equivalent to
`(b + 128) mod 256` — applied byte-by-byte. About 200 MB/s on a
laptop.

### Capture file layout

`.iq` files are **raw interleaved signed-8-bit I/Q with no header**:

```
byte 0: i₀   byte 1: q₀
byte 2: i₁   byte 3: q₁
…
```

At 2 MS/s this is 4 MB/s, or ≈ 240 MB/min. Sample rate and centre
frequency are not encoded in the file — they're constants the
toolchain assumes (`--sample-rate` / `--center-freq` are CLI
overrides on `replay` and `track`). One file format, two
parameters out-of-band; the convention is "everything is ADS-B
at 1090 MHz / 2 MS/s unless you say otherwise."

## Frame detection at the IF level

The detector runs three operations per sample:

1. **Magnitude:** alpha-max-beta-min, `mag = max(|I|, |Q|) +
   min(|I|, |Q|) / 2`. Branchless, 38 ps/sample on M-series. See
   [`benchmarks.md`](benchmarks.md).
2. **Noise-floor EMA:** exponential moving average of magnitude
   with τ ≈ 100 µs (`shift = 9`). Drives the adaptive preamble
   threshold; keeps the receiver useful as SNR varies.
3. **Preamble correlator:** sliding 16-sample correlator scoring
   `Σhigh − Σlow` against the canonical preamble pattern.

A preamble candidate clears the threshold → the next 56 or 112 bits
are sliced at 2 samples/bit (rule: pulse in first half = 1, second
half = 0; absolute value of the slicer's difference becomes the
per-bit confidence in `[0, 255]`). Aggregate per-frame confidence
is the **minimum** of the per-bit values — one bad bit dominates
how correctable a frame is.

The slicer's output goes to the CRC layer. CRC-clean or 1-bit
corrected frames bubble up via the `process` callback; everything
else is dropped unless the state tracker's address-XOR recovery
later resurrects it (see `state.rs`).

## Practical antenna notes

For "validate on real signals" rather than "build a feeder station":

- **Indoor near a window** works for major-metro airspace. NYC
  rooftop windows pull JFK/LGA/EWR approach traffic at altitudes
  down to ~800 ft with the stock dongle whip.
- **Outdoor with line-of-sight** is the dominant win. Range scales
  with the square root of antenna height above local obstacles —
  6 m of mast is worth more than any amplifier you can buy.
- **Bias-T off by default.** rs1090 ships with bias-T disabled
  because feeding 5 V into an antenna that doesn't expect it can
  damage things. Pass `--bias-t` on `rs1090-cli live` only if your
  antenna explicitly supports it.
- **Gain at 40 dB** is the practical sweet spot. Higher saturates
  on close-by transmitters; lower loses marginal-SNR frames at
  range. Auto-gain (`--auto-gain`) is reasonable as a default but
  rarely wins over a hand-tuned static gain in stable RF
  environments.

## Live capture, replay, and the differential test

The capture/replay loop is the workhorse of correctness work:

```sh
# Live capture, also writing the bias-subtracted IQ to disk.
rs1090-cli live --duration-secs 120 --record corpus/$(date +%Y%m%d-%H%M%S).iq

# Replay the captured file — same decoder, deterministic output.
rs1090-cli replay corpus/<file>.iq

# Differential test the replay output against pyModeS.
scripts/diff_pymodes.py corpus/<file>.iq
```

The same `SampleSource` trait drives both `IqFileSource` (the
replay path) and `RtlSdrSource` (the live path). Anything that
works on a captured file is byte-for-byte reproducible on every
machine that has the file — `dump1090` files included, once
converted to SC8 with `s8_to_uc8` (in reverse, via the same XOR
operation). See [`differential-testing.md`](differential-testing.md)
for the full runbook.

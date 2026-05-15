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

### Known limitations carried into M3

- DF 11 address recovery from a non-zero CRC syndrome is not yet wired
  into the frame layer. Clean DF 11 frames work; corrupted ones whose
  syndrome equals an unknown ICAO are surfaced as `Failed` rather than
  having their address recovered.
- Surface position (TC 5–8), aircraft status (TC 28), and operational
  status (TC 31) are accepted by the dispatcher but emitted as `Raw`;
  decoding lands in M3 once we have a state tracker to feed reference
  positions to.

### Fixed

- DESIGN.md previously cited the 3.96% error bound, which applies to the
  multiplicative variant `α = 15/16, β = 15/32`. The shift-only form we
  actually use peaks at ~11.8% in continuous math; corrected in both the
  design doc and module-level docs.

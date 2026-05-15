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

### Fixed

- DESIGN.md previously cited the 3.96% error bound, which applies to the
  multiplicative variant `α = 15/16, β = 15/32`. The shift-only form we
  actually use peaks at ~11.8% in continuous math; corrected in both the
  design doc and module-level docs.

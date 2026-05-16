# Development

Local workflow for the rs1090 workspace — build, test, format, lint,
and what to run before opening a PR.

## Prerequisites

- **Rust 1.88+** via `rustup`. The MSRV is pinned in
  `Cargo.toml` (`rust-version = "1.88"`); CI rejects PRs that need a
  newer compiler unless `rust-version` is bumped in the same PR with
  rationale.
- **No system C dependencies** for the default build. The RTL-SDR
  backend uses pure-Rust `nusb`; cross-compiles are linker-only
  (see [`raspberry-pi.md`](raspberry-pi.md)).
- For the **differential test**, Python 3.10+ and pyModeS. See
  [`differential-testing.md`](differential-testing.md).
- For **fuzzing**, nightly Rust and `cargo-fuzz`. See
  [`fuzzing.md`](fuzzing.md).

## Layout

```text
crates/
  rs1090/         # the decoder library (#![no_std]-friendly under default features)
    src/
      crc.rs            CRC-24 with single-bit correction and syndrome tables
      cpr.rs            Compact Position Reporting (airborne global + local)
      demod.rs          (pub(crate)) preamble + slicer + noise floor
      magnitude.rs      (pub(crate)) alpha-max-beta-min and LUT
      frame.rs          FrameDetector and the Frame value type
      message.rs        DF/TC dispatch, decoded payload structs, BitReader
      source.rs         SampleSource trait + IqFileSource + RtlSdrSource
      state.rs          StateTracker — LRU table, CPR pairing, address-XOR recovery
      lib.rs            module roots + Iq + test_utils re-exports
    benches/      criterion microbenchmarks
    fuzz/         libFuzzer targets + seed corpora
  rs1090-cli/     replay, track, live subcommands
  rs1090-serve/   HTTP + SSE server, static map UI
scripts/          diff_pymodes.py (differential test harness)
docs/             this directory
DESIGN.md         the architecture rationale; read this before non-trivial changes
CHANGELOG.md      Keep-a-Changelog format; update on every shipped milestone
```

## Build

Every command runs from the workspace root. The release profile is
configured with `lto = "thin"`, `codegen-units = 1`, and
line-tables-only debuginfo — about a 30 s clean release build on
M-series hardware.

```sh
cargo build --workspace                  # debug — fast incremental
cargo build --workspace --release        # release — ship and bench
cargo build --release -p rs1090-serve    # one crate at a time
```

The RTL-SDR live backend is gated on the `rtl-sdr` feature, which is
**on by default** for `rs1090-cli` and `rs1090-serve`. For machines
without an SDR present, the library and the `file`/`replay`
subcommands still build and run unchanged.

## Test

```sh
cargo test --workspace --release          # ~5 s, 104 tests at last count
```

The `--release` flag matters for the demod/magnitude tests — they
exercise the alpha-max-beta-min path enough that debug builds take
30× longer. Same `release` is used by CI.

Targeted runs while iterating:

```sh
cargo test -p rs1090 frame::             # only the frame module's tests
cargo test --release -p rs1090 message:: # all message tests
cargo test --release ground_velocity     # one specific test by substring
```

## Format

```sh
cargo fmt --all                  # apply
cargo fmt --all --check          # CI runs this — must be green
```

Editor-on-save is the path of least friction. The repo deliberately
has no `rustfmt.toml` — vanilla rustfmt is the canon.

## Lint

```sh
cargo clippy --workspace --all-targets --release -- -D warnings
```

Pedantic-level lints are enabled in workspace `[lints.clippy]`.
The `--all-targets` flag includes integration tests, benches, and
examples — drift in any of them fails locally before CI.

## Before pushing

Three commands, in this order, must come up green:

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets --release -- -D warnings
cargo test --workspace --release
```

If any of them fails locally, CI will fail the same way. The CI
workflow file at `.github/workflows/ci.yml` runs exactly these.

## Running the binaries locally

```sh
# Replay a recorded capture and print per-frame lines
cargo run --release -p rs1090-cli -- replay corpus/<file>.iq

# Aggregate into per-aircraft state-tracker events
cargo run --release -p rs1090-cli -- track corpus/<file>.iq --summary

# Live from RTL-SDR (requires the dongle on USB)
cargo run --release -p rs1090-cli -- live --duration-secs 30

# HTTP + SSE server (file replay)
cargo run --release -p rs1090-serve -- file corpus/<file>.iq --realtime

# HTTP + SSE server (live SDR) — open http://127.0.0.1:8080 for the map
cargo run --release -p rs1090-serve -- live
```

See [`raspberry-pi.md`](raspberry-pi.md) for the cross-compile +
deploy story.

## Adding tests

Prefer **structural** tests over coverage tests. Examples that hit
the bar:

- Cross-implementation verification (`crc::tests::table_matches_bitwise`).
- Invariant tests (`crc::tests::syndrome_tables_are_collision_free` —
  if it fires, 1-bit correction is structurally broken).
- Pinned regression for real-world inputs
  (`message::tests::ground_velocity_truncates_speed_fraction` —
  exact bytes from a real frame).
- Round-trip with synthesised inputs (`frame::tests::detector_recovers_synthetic_df17_frame`).

Avoid asserting only on side-effects of internal state without
proving the assertion is non-trivial. If a test would still pass
when the code does nothing, it isn't a test.

## Commit messages

The repository's history uses a tight, descriptive style:

```
<scope>: <one-line summary, no trailing punctuation>

Paragraph explaining *why* — the constraint, decision, or bug being
fixed. Quote line numbers and identifiers; readers grep history.

Follow-up paragraph for tradeoffs, alternative approaches considered
and rejected, or load-bearing test names.
```

Recent commits in `git log --oneline -20` are the canon. Subject
line under 70 chars; body wrapped at 72.

## Updating the CHANGELOG

`CHANGELOG.md` follows the [Keep a Changelog](https://keepachangelog.com)
format. Substantive work — new features, fixes a user could feel, API
or wire-format changes — goes under the `[Unreleased]` section in one
of `Added` / `Changed` / `Fixed` / `Removed` / `Deprecated` /
`Security`. Pure internal refactors and test additions don't need an
entry. When a release tag goes out, move `[Unreleased]` to the new
version's section.

# Fuzzing

libFuzzer (via `cargo-fuzz`) is the primary tool for finding panics,
integer overflows, slice OOB, and unwrap-on-None bugs in the parser
layers. Three targets cover the parser-shaped surfaces end to end:

| Target | What it fuzzes | Seed corpus |
|--------|----------------|-------------|
| `decode_message` | `Frame::from_bytes` → `message::decode` | 120 real frames from a live capture |
| `process_frame` | `FrameDetector::process` (IQ → magnitude → preamble → CRC) | 32 KiB of real captured IQ |
| `crc_check` | `crc::check` (validation + in-place 1-bit correction) | Real-frame bytes |

Each target's source is `crates/rs1090/fuzz/fuzz_targets/<name>.rs`;
seeds live at `crates/rs1090/fuzz/seeds/<name>/`. The working corpus
at `crates/rs1090/fuzz/corpus/<name>/` is gitignored.

## Prerequisites

```sh
rustup install nightly
cargo install cargo-fuzz
```

One-time setup, ~3 minutes total.

## Running a target

```sh
cd crates/rs1090/fuzz

# First time only — copy seeds into the working corpus.
mkdir -p corpus/<target>
cp -n seeds/<target>/* corpus/<target>/

# Then every run:
cargo +nightly fuzz run <target>
```

Targets:

```sh
cargo +nightly fuzz run decode_message
cargo +nightly fuzz run process_frame
cargo +nightly fuzz run crc_check
```

Time-bounded runs:

```sh
cargo +nightly fuzz run decode_message -- -max_total_time=300   # 5 min
```

Crash files (if any) land under `crates/rs1090/fuzz/artifacts/<target>/`
with names like `crash-<sha>`. The corpus directory grows as libFuzzer
discovers new coverage-extending inputs.

## Throughput we currently see

On an M-series Mac, against the committed seed corpora:

- `decode_message`: ~925 K exec/s, plateaus at 166 edges / 219 features
  in the first second. Seed corpus already exercises every reachable
  decode path.
- `process_frame`: ~20 K exec/s (full pipeline per input — slower
  because each IQ slab actually runs demod + CRC).
- `crc_check`: ~947 K exec/s, plateaus at 66 edges / 80 features
  immediately. The CRC machinery is tight.

To date all three are crash-free across tens of millions of executions.

## Triaging a crash

If a target crashes, libFuzzer drops the offending input to
`artifacts/<target>/crash-<sha>` and prints the bytes. To reproduce
locally:

```sh
cargo +nightly fuzz run <target> artifacts/<target>/crash-<sha>
```

Single input, immediate panic, full stack trace via `RUST_BACKTRACE=1`.

For the fix:

1. Read the panic message — most are `unreachable!`, debug-assert
   failures, or arithmetic overflow. The path is usually narrow.
2. Write a unit test that takes the crash bytes verbatim and exercises
   the affected entry point. Name the test
   `<function>_does_not_panic_on_<short_description>`.
3. Fix the code. The test should go red → green.
4. Add the crash file to `seeds/<target>/` (rename to
   `seed_<short_description>.bin` for clarity) so future fuzz runs
   start from this anchor.
5. Re-run the target to confirm no new crashes are found.

## Adding a new target

`cargo fuzz add <name>` creates a stub at
`crates/rs1090/fuzz/fuzz_targets/<name>.rs` and adds a `[[bin]]`
entry to `fuzz/Cargo.toml`. Write the body:

```rust
#![no_main]
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    // Coerce `data` into the input shape your target consumes.
    // Bail early on inputs the function can't handle so libFuzzer
    // converges on valid sizes quickly.
    if data.len() != EXPECTED_LEN { return; }
    let _ = your_function(data);
});
```

Then:

1. Add a `seeds/<name>/` directory with at least one realistic input
   (helps libFuzzer discover the meaningful coverage edges fast).
2. Document the target in the table at the top of this file.
3. Re-build the target: `cargo +nightly fuzz build`.
4. Smoke-run for 10–60 seconds to confirm it actually exercises new
   coverage edges relative to the seed corpus.

## Corpus hygiene

The committed `seeds/` is the canonical reproducible set. The working
`corpus/` may grow indefinitely as libFuzzer discovers new inputs;
it's gitignored.

If `corpus/<target>/` grows beyond a few thousand files and slows down
each run's startup, minimise it:

```sh
cargo +nightly fuzz cmin <target>
```

`cmin` keeps only the smallest set of inputs that reproduces the
current coverage. After a `cmin`, optionally promote any newly-
interesting inputs to `seeds/` with a meaningful `seed_*.bin` name.

## How this fits with the rest

Fuzzing complements but doesn't replace:

- **Unit tests** — pin known good behaviour.
- **Property-based tests** (proptest) — sweep continuous-input
  spaces (sample distributions, CRC roundtrips).
- **Differential testing** — cross-check decoded fields against
  pyModeS on real captures (see
  [`differential-testing.md`](differential-testing.md)).

Each catches a different kind of bug. The fuzz harness specifically
catches "what about *this* sequence of bytes we never thought to
write a test for?" — and is the only one that finds panics on
adversarial input.

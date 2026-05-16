# Differential testing against pyModeS

For every `.iq` capture we run, we cross-check rs1090's decoded
fields against [pyModeS] — the de facto Python reference for ADS-B
field decoding. Catching a field-level disagreement on a real frame
is the cheapest way to find decoder bugs that pass unit tests
because no one wrote a unit test for that exact byte pattern.

The harness lives at `scripts/diff_pymodes.py`.

## What's checked

Every CRC-clean DF 11 / DF 17 / DF 18 frame in the capture has its
hex payload decoded by both rs1090 (via `cargo run -p rs1090-cli
replay`) and pyModeS (`pyModeS.decode`). We compare overlapping
fields one by one and report per-field tallies plus a sample of any
disagreements.

Currently compared fields:

| Field | Source |
|-------|--------|
| ICAO | DF 11 (announced address); DF 17/18 AA |
| altitude | TC 9–18, 20–22 |
| cpr_format (even/odd) | TC 9–18, 20–22 |
| cpr_lat, cpr_lon | TC 9–18, 20–22 |
| callsign | TC 1–4 |
| groundspeed (kt) | TC 19 subtype 1–2 |
| vertical_rate (fpm) | TC 19, all subtypes |

Heading and airspeed are not yet cross-checked because pyModeS reports
them under slightly different field names by subtype. Adding them is a
small follow-up — see "Adding more fields" below.

## Prerequisites

A Python venv with pyModeS v3+ installed:

```sh
python3 -m venv .venv
.venv/bin/pip install pyModeS
```

The harness invokes whichever `python3` is on PATH. To use the venv
explicitly, run via its python directly (see commands below).

## Running it

From the workspace root:

```sh
.venv/bin/python3 scripts/diff_pymodes.py corpus/<file>.iq
```

The harness builds rs1090-cli (or reuses an existing release binary)
and runs `replay` on the file under the hood, so the same release
profile and flags are exercised. Typical output:

```text
# decoding corpus/live_1090mhz_2min.iq via rs1090 replay…
# 2032 replay lines

======================================================================
rs1090 ↔ pyModeS differential test
======================================================================
corpus:                corpus/live_1090mhz_2min.iq

CRC outcomes from rs1090:
       clean: 491
   corrected: 19
      failed: 1522

CRC-clean DF distribution: {11: 206, 17: 285}

pyModeS decode failures: 0
frames compared:         491

field              agree   disagree  rs1090-only  pms-only
icao                 491          0            0         0
altitude             108          0            0         0
cpr_format           108          0            0         0
cpr_lat              108          0            0         0
cpr_lon              108          0            0         0
callsign              15          0            0         0
groundspeed           89          0            0         0
vertical_rate         89          0            0         0

✓ no field-level disagreements
```

Exit code 0 if every comparable field agrees; 1 if any disagree.

Flags:

```sh
--show-disagreements N   # cap example disagreements per field (default 5)
```

## When it finds a real bug

The harness has already caught one bug this way (commit `4d5f2a0`).
The shape:

1. The harness reports `groundspeed: 49 agree, 40 disagree`.
2. Sample disagreements all show `rs1090 = X, pms = X-1` — exactly +1 kt.
3. Investigate: `decode_velocity` was rounding `hypot(ew, ns)` while
   pyModeS truncates.
4. Fix: `.round() as u16` → `.trunc() as u16` at `message.rs:494`.
5. Pin: add a unit test using a real ME byte slice from the disagreeing
   frame, asserting the truncated value.
6. Re-run harness: 0 disagreements.

The general pattern: **disagreement → identify rule → pick rs1090's
behaviour → fix one or the other → pin with a unit test
referencing real bytes**.

## When to trust pyModeS vs rs1090

pyModeS is the closer-to-spec implementation by construction (its
author wrote *The 1090 MHz Riddle* alongside the library) and is
widely used. **Default to pyModeS being right** when there's a
genuine disagreement, unless we have a documented reason to deviate
(e.g. the truncation choice for groundspeed — DO-260B doesn't
specify, both are defensible).

If we deviate intentionally, the unit test that pins the choice
should *say so in a comment*, and the harness still has to come up
green — the diff harness pins the choice as much as the unit test
does.

## Adding more fields

The harness extracts fields from rs1090's text output via
regexes (see `LINE_RE` and the per-field regexes near the top of
`diff_pymodes.py`). To add a new field:

1. Confirm rs1090's `replay` output already includes it. If not,
   extend `print_message_summary` in `crates/rs1090-cli/src/main.rs`.
2. Add a regex in `diff_pymodes.py` and pull the field into
   `parse_rs1090`'s returned dict.
3. Add the field name to `COMPARABLE_FIELDS`.
4. If pyModeS reports it under a different key, add a key-translation
   step in the main loop where `pms_val = pms_result.get(pms_key)`.
5. If the comparison needs tolerance (floats, etc.), extend the
   `field_equal` helper.
6. Re-run on the existing corpus; expect zero disagreements for an
   already-correct field, or a real bug to surface.

## Where the corpus lives

The captures themselves are gitignored (`corpus/` at the repo root —
see `.gitignore`). They're big and non-deterministic; we keep them
local but don't commit them. To share a corpus across machines,
SCP/rsync; to share a *known-good* slice, extract a minimal fixture
and commit it under `crates/rs1090/tests/fixtures/`.

If you want a fresh corpus, the easiest path is to record live:

```sh
cargo run --release -p rs1090-cli -- live --duration-secs 120 \
    --record corpus/$(date +%Y%m%d-%H%M%S).iq
```

Pi Zero 2 W with the antenna near a window in a major-metro
airspace is plenty — see [`raspberry-pi.md`](raspberry-pi.md).

[pyModeS]: https://github.com/junzis/pyModeS

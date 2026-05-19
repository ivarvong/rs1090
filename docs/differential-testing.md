# Differential testing against pyModeS

For every `.iq` capture we run, we cross-check rs1090's decoded
fields against [pyModeS] — the de facto Python reference for ADS-B
field decoding. Catching a field-level disagreement on a real frame
is the cheapest way to find decoder bugs that pass unit tests
because no one wrote a unit test for that exact byte pattern.

Two harnesses, two layers:

| Script | Layer | What it catches |
|---|---|---|
| `scripts/diff_pymodes.py` | per-frame fields | Mis-parsed bits, wrong altitude, wrong CPR even/odd, wrong callsign decoding |
| `scripts/diff_positions.py` | per-aircraft trajectories | Bugs in the state tracker's CPR pairing, in the local-decode fallback, or in surface-position quadrant resolution — anything where the bits parse the same but the resulting *position* differs |

The two are complementary: bit-level agreement is necessary but not
sufficient; identical bits can still resolve to different positions
if the tracker pairs them differently.

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

A Python venv with **pyModeS v2** installed. pyModeS v3 removed the
per-field function API (`pyModeS.adsb.position`, `…altitude`, …)
that both harnesses depend on; until they're rewritten against v3's
single-`decode()` shape, pin v2:

```sh
python3 -m venv .venv
.venv/bin/pip install "pyModeS<3"
```

Run via the venv's python directly to make sure the right interpreter
sees the install (the scripts' `#!/usr/bin/env python3` shebang picks
up the system python by default):

```sh
.venv/bin/python3 scripts/diff_pymodes.py corpus/<file>.iq
.venv/bin/python3 scripts/diff_positions.py corpus/<file>.iq
```

## Running diff_pymodes (field-level)

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
4. Fix: `.round() as u16` → `.trunc() as u16` in `decode_velocity`.
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

## Running diff_positions (position-level)

```sh
.venv/bin/python3 scripts/diff_positions.py corpus/<file>.iq --reference 40.70214,-73.98262
```

The reference is what the operator set on the live receiver — the
state tracker's local-decode fallback uses it, so we use the same
value to compare apples to apples. Typical output:

```text
ICAO     rs1090 last (lat,lon,src)              pyModeS last (lat,lon)         Δ nm
-----------------------------------------------------------------------------------
A1C57E   40.8261,-73.9033 (global)              40.8182,-73.9009              0.48
A310CF   40.8251,-73.9030 (global)              40.8211,-73.9016              0.25

per-ICAO trajectory inspection (rs1090):
  ICAO     fixes  extent nm  closest to ref nm  sources
  -----------------------------------------------------
  A1C57E       8      0.74              7.70  global,local
  A310CF       4      0.28              8.01  global,local
```

What to look for:

- **Per-fix delta > 1 nm**: a `⚠` flag means rs1090 and pyModeS
  disagree on where the aircraft is. Most of the time small (<1 nm)
  deltas are timing-of-last-pair, not decoder error — but anything
  in the multi-nm range is real divergence.
- **`stuck near reference` pattern**: ≥3 fixes for one aircraft, all
  within 2 nm of each other AND within 2 nm of the receiver, with
  no `global` source. Hallmark of local-decode-wrong-tile: the
  aircraft is really somewhere far away but the local decode keeps
  snapping it to the reference tile because no global pair forms.
  Frequently a low-altitude (≤2000 ft) match here is a real
  helicopter near the receiver, not a bug.
- **rs1090-only aircraft with `local` source**: not a disagreement
  per se — pyModeS doesn't have a local-decode tracker, so it
  produces no fix for an aircraft that never gives us a fresh
  even/odd pair. Still worth eyeballing the trajectory for the
  `stuck near reference` pattern.

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

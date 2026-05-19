#!/usr/bin/env python3
"""
Position-level differential test: rs1090's state-tracker output vs. an
independent CPR resolver built on pyModeS, run over the same frame
sequence.

Why this exists on top of `diff_pymodes.py`:

  - `diff_pymodes.py` compares per-frame *fields* (CPR even/odd flag,
    17-bit CPR lat/lon, altitude, callsign, …). Agreement at that
    level only proves we *parsed* the same bits.
  - It does NOT check whether the CPR bits decode to the same actual
    lat/lon. The decode goes through a state tracker that pairs even
    and odd messages, falls back to local-decode against a reference
    when no fresh pair is available, and (for TC 5–8) navigates the
    four-quadrant surface ambiguity.
  - Every one of those tracker decisions is a place a bug can hide
    that bit-level diff misses.

This script closes that gap. For each CRC-clean DF 17/18 airborne
position frame in the input, it asks two questions:

  1. **What did rs1090's tracker emit?** From `rs1090 track <iq>`,
     filter the per-aircraft `pos` events.
  2. **What would pyModeS emit given the same frames in order?**
     Re-implement just the pairing + global-decode logic on top of
     `pms.adsb.position()`, walking the same ordered hex frames.

Then per ICAO, compare the trajectories. The headline outputs:

  - **last-fix delta**: how far apart the two decoders' last position
    for each aircraft was, in nautical miles. Anything >1 nm is
    suspicious; anything that lands near the receiver's reference
    when pyModeS doesn't agree is the classic local-decode-wrong-tile
    failure mode.
  - **trajectory drift**: per-fix delta over time. A bug that
    sometimes picks the wrong pair will show up as a sequence of
    correct fixes interrupted by one or two outliers, then back to
    correct.

Usage:
    scripts/diff_positions.py [path/to/capture.iq] [--reference LAT,LON]

The reference is what rs1090 would have been configured with on the
live receiver; pyModeS's local fallback uses the same value. Default
is the project's example reference (Brooklyn antenna site).
"""

import argparse
import math
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

try:
    import pyModeS as pms
    from pyModeS import adsb
except ImportError:
    sys.exit("error: pyModeS not installed. Try `pip install pyModeS`.")


# rs1090-cli's `replay --timestamps` output (one line per detected frame):
#   [T+SECONDS ]DFnn HEX clean|corrected:N|failed conf=N ICAO=ABC123 ... cpr-even/odd(LAT,LON) ...
# The leading `T+SECONDS ` is optional — older captures without it
# still parse, just without wall-clock pair-window enforcement.
REPLAY_LINE = re.compile(
    r"^(?:T\+(?P<t>[\d.]+)\s+)?"
    r"DF(?P<df>\d+)\s+(?P<hex>[0-9A-F]+)\s+(?P<crc>clean|corrected:\d+|failed)\s+"
    r"conf=(?P<conf>\d+)\s*(?P<rest>.*)$"
)
REPLAY_ICAO = re.compile(r"ICAO=([0-9A-F]{6})")
REPLAY_TC = re.compile(r"\bTC=(\d+)")  # if present in the summary
REPLAY_CPR = re.compile(r"cpr-(even|odd)\((\d+),(\d+)\)")
REPLAY_ALT = re.compile(r"alt=(-?\d+)ft")

# rs1090-cli's `track` output (one line per state event):
#   pos     ABC123 40.7000,-74.0000 25000ft (global)
TRACK_POS = re.compile(
    r"^pos\s+(?P<icao>[0-9A-F]{6})\s+(?P<lat>-?[\d.]+),(?P<lon>-?[\d.]+)\s+"
    r"(?P<alt>\S+)\s+\((?P<source>\w+)\s*\)"
)


def run(cmd: list[str]) -> str:
    """Run a subprocess and return stdout. Aborts on non-zero exit."""
    proc = subprocess.run(cmd, capture_output=True, check=True, text=True)
    return proc.stdout


def parse_replay(raw: str):
    """Yield dicts for every recognised replay line. Order preserved.
    Each dict includes a `t` field if the replay was run with
    `--timestamps`, otherwise `t` is `None`.
    """
    for line in raw.splitlines():
        m = REPLAY_LINE.match(line)
        if not m:
            continue
        d = {
            "t": float(m["t"]) if m["t"] else None,
            "df": int(m["df"]),
            "hex": m["hex"],
            "crc": m["crc"],
        }
        rest = m["rest"]
        if (mm := REPLAY_ICAO.search(rest)):
            d["icao"] = mm.group(1)
        if (mm := REPLAY_CPR.search(rest)):
            d["cpr_odd"] = mm.group(1) == "odd"
            d["cpr_lat"] = int(mm.group(2))
            d["cpr_lon"] = int(mm.group(3))
        if (mm := REPLAY_ALT.search(rest)):
            d["alt_ft"] = int(mm.group(1))
        yield d


def parse_track(raw: str):
    """Yield position events from the track output, in order."""
    for line in raw.splitlines():
        m = TRACK_POS.match(line)
        if not m:
            continue
        yield {
            "icao": m["icao"],
            "lat": float(m["lat"]),
            "lon": float(m["lon"]),
            "alt": m["alt"],
            "source": m["source"],
        }


def haversine_nm(lat1, lon1, lat2, lon2):
    """Great-circle distance in nautical miles."""
    R_NM = 3440.065  # mean Earth radius
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R_NM * math.asin(min(1.0, math.sqrt(a)))


def simulate_pymodes_tracker(
    frames,
    reference,
    pair_window_secs=10.0,
    fallback_pair_window_frames=10,
):
    """
    Walk the ordered frames and emit per-aircraft positions exactly
    where pyModeS's global decode would produce one.

    Pair window is enforced in **wall-clock seconds** when the
    replay was run with `--timestamps` (each frame carries `t`).
    Without timestamps we fall back to a per-ICAO frame distance,
    which is loose for sparsely-seen aircraft — see commit history
    for why this matters (10-min Mac capture had 3 aircraft where
    a frame-distance window happily paired even+odd halves several
    minutes apart in real time, getting pyModeS to "decode" them
    to physically impossible positions).

    Returns a dict[icao] -> list of (global_frame_index, lat, lon,
    source).
    """
    state = {}
    positions = defaultdict(list)
    seen_count = defaultdict(int)
    for i, f in enumerate(frames):
        if f["crc"] != "clean" or f["df"] not in (17, 18):
            continue
        if "cpr_odd" not in f or "icao" not in f:
            continue
        icao = f["icao"]
        seen_count[icao] += 1
        local_idx = seen_count[icao]
        slot = "odd" if f["cpr_odd"] else "even"
        other = "even" if f["cpr_odd"] else "odd"
        st = state.setdefault(icao, {})
        # (per_icao_idx, t_seconds_or_None, global_idx, hex)
        st[slot] = (local_idx, f["t"], i, f["hex"])

        if other not in st:
            continue
        # Decide whether the most recent even+odd pair is fresh
        # enough to be a valid global-decode candidate.
        t_a = st[other][1]
        t_b = st[slot][1]
        if t_a is not None and t_b is not None:
            in_window = abs(t_b - t_a) <= pair_window_secs
        else:
            in_window = (
                abs(st[other][0] - local_idx) <= fallback_pair_window_frames
            )
        if not in_window:
            continue

        even_hex = st["even"][3]
        odd_hex = st["odd"][3]
        t_even = st["even"][1] if st["even"][1] is not None else float(st["even"][0])
        t_odd = st["odd"][1] if st["odd"][1] is not None else float(st["odd"][0])
        try:
            pos = adsb.position(even_hex, odd_hex, t_even, t_odd)
        except Exception:
            pos = None
        if pos and pos[0] is not None and pos[1] is not None:
            positions[icao].append((i, pos[0], pos[1], "global"))
    return positions


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("iq", nargs="?", default="corpus/capture_5min.iq")
    ap.add_argument(
        "--reference",
        default="40.70214,-73.98262",
        help="Receiver reference for rs1090's local-decode fallback "
        "(default: Brooklyn antenna site)",
    )
    ap.add_argument(
        "--threshold-nm",
        type=float,
        default=1.0,
        help="Flag last-fix deltas larger than this (default: 1 nm)",
    )
    ap.add_argument(
        "--pair-window",
        type=float,
        default=10.0,
        help="pyModeS pair window in **wall-clock seconds** when the "
        "replay carries timestamps (run with `rs1090 replay --timestamps`); "
        "falls back to per-ICAO frame distance otherwise. Default 10.0 "
        "matches rs1090's internal CPR_PAIR_WINDOW.",
    )
    args = ap.parse_args()

    iq_path = Path(args.iq)
    if not iq_path.exists():
        sys.exit(f"error: {iq_path} not found")

    ref_lat, ref_lon = (float(x) for x in args.reference.split(","))

    print(f"# replay  {iq_path}…", file=sys.stderr)
    replay = run(
        [
            "cargo", "run", "--release", "--quiet", "-p", "rs1090-cli", "--",
            "replay", "--timestamps", str(iq_path),
        ]
    )
    frames = list(parse_replay(replay))
    have_ts = any(f["t"] is not None for f in frames)
    print(
        f"#   {len(frames)} frames "
        f"({'with' if have_ts else 'without'} wall-clock timestamps)",
        file=sys.stderr,
    )

    print(f"# track   {iq_path}…", file=sys.stderr)
    track = run(
        [
            "cargo", "run", "--release", "--quiet", "-p", "rs1090-cli", "--",
            "track", "--reference", args.reference, str(iq_path),
        ]
    )
    track_events = list(parse_track(track))
    print(f"#   {len(track_events)} position events from rs1090", file=sys.stderr)

    # rs1090 positions per ICAO, in emission order.
    rs_positions = defaultdict(list)
    for ev in track_events:
        rs_positions[ev["icao"]].append(
            (ev["lat"], ev["lon"], ev["source"])
        )

    # pyModeS positions per ICAO, in emission order. We pin the
    # `pair_window` from the CLI so we can shift it if needed (rs1090
    # uses 10s of wall time; we use frame distance as a proxy).
    pms_positions = simulate_pymodes_tracker(
        frames, (ref_lat, ref_lon), pair_window_secs=args.pair_window
    )

    # ----- Report -----
    print()
    print("=" * 78)
    print("rs1090-tracker ↔ pyModeS-tracker  position differential")
    print("=" * 78)
    print(f"corpus:     {iq_path}")
    print(f"reference:  {ref_lat:.4f}, {ref_lon:.4f}")
    print(f"frames:     {len(frames)} total, "
          f"{sum(1 for f in frames if f['crc'] == 'clean')} CRC-clean")
    print()

    icaos = sorted(set(rs_positions) | set(pms_positions))
    print(f"aircraft seen by either decoder: {len(icaos)}")
    print()

    header = f"{'ICAO':<8} {'rs1090 last (lat,lon,src)':<38} {'pyModeS last (lat,lon)':<26} {'Δ nm':>8}"
    print(header)
    print("-" * len(header))
    flagged = 0
    for icao in icaos:
        rs_list = rs_positions.get(icao, [])
        pms_list = pms_positions.get(icao, [])
        rs_last = rs_list[-1] if rs_list else None
        pms_last = pms_list[-1] if pms_list else None

        rs_str = (
            f"{rs_last[0]:.4f},{rs_last[1]:.4f} ({rs_last[2]})"
            if rs_last else "—"
        )
        pms_str = (
            f"{pms_last[1]:.4f},{pms_last[2]:.4f}"
            if pms_last else "—"
        )
        if rs_last and pms_last:
            delta = haversine_nm(
                rs_last[0], rs_last[1], pms_last[1], pms_last[2]
            )
            # Only fire ⚠ on a *like-for-like* comparison: both
            # decoders' last fix is from a fresh global pair. If
            # rs1090's last fix is `local`, it's a single-message
            # fallback against the receiver reference, possibly
            # snapshotted at a different moment than pyModeS's
            # most recent pair; the resulting delta then mostly
            # reflects how far the aircraft moved between those
            # two moments, not decoder error.
            comparable = rs_last[2] == "global"
            mark = ""
            if delta > args.threshold_nm and comparable:
                mark = "  ⚠"
                flagged += 1
            elif delta > args.threshold_nm:
                mark = "  (rs:local, timing-mismatch likely)"
            print(f"{icao:<8} {rs_str:<38} {pms_str:<26} {delta:>7.2f}{mark}")
        else:
            print(f"{icao:<8} {rs_str:<38} {pms_str:<26} {'—':>8}")

    # Also flag aircraft where rs1090 emitted a fix but pyModeS
    # couldn't form a pair within the window. That's not necessarily
    # a bug — rs1090's local-decode fallback can produce a fix when
    # only one parity is available — but it deserves a heads-up so
    # the reader knows the comparison was one-sided.
    rs_only = [i for i in icaos if rs_positions.get(i) and not pms_positions.get(i)]
    pms_only = [i for i in icaos if pms_positions.get(i) and not rs_positions.get(i)]

    print()
    print(f"rs1090-only positions (no pyModeS pair):  {len(rs_only)}  {rs_only[:10]}")
    print(f"pyModeS-only positions (no rs1090 fix):   {len(pms_only)}  {pms_only[:10]}")
    print()

    # Per-trajectory dump: catches "label cluster near receiver"
    # bugs that the last-fix delta misses. For each ICAO, the
    # trajectory length (number of fixes) and the spatial extent
    # (max distance between any two fixes) together identify the
    # most common state-tracker failure mode — multiple "fixes" that
    # all land near the reference because the local-decode tile
    # ambiguity wasn't resolved.
    print("per-ICAO trajectory inspection (rs1090):")
    header = f"  {'ICAO':<8} {'fixes':>5} {'extent nm':>10} {'closest to ref nm':>18}  {'sources'}"
    print(header)
    print("  " + "-" * (len(header) - 2))
    suspicious = []
    for icao in sorted(rs_positions):
        fixes = rs_positions[icao]
        sources = ",".join(sorted(set(f[2] for f in fixes)))
        if len(fixes) >= 2:
            extent = max(
                haversine_nm(a[0], a[1], b[0], b[1])
                for i, a in enumerate(fixes)
                for b in fixes[i + 1:]
            )
        else:
            extent = 0.0
        closest_to_ref = min(
            haversine_nm(f[0], f[1], ref_lat, ref_lon) for f in fixes
        )
        marker = ""
        # An aircraft sitting <2 nm from the receiver in *every*
        # fix, with at least two fixes, and never re-decoded via
        # `global` source, is the canonical local-decode-wrong-tile
        # symptom.
        if (
            len(fixes) >= 3
            and extent < 2.0
            and closest_to_ref < 2.0
            and "global" not in sources
        ):
            marker = "  ⚠ stuck near reference"
            suspicious.append(icao)
        print(
            f"  {icao:<8} {len(fixes):>5} {extent:>9.2f} {closest_to_ref:>17.2f}  {sources}{marker}"
        )
    if suspicious:
        print()
        print(f"⚠ {len(suspicious)} aircraft with the local-decode-stuck pattern: {suspicious}")
    print()
    print(f"⚠  {flagged} aircraft with last-fix delta > {args.threshold_nm:.2f} nm")
    return 1 if flagged or suspicious else 0


if __name__ == "__main__":
    sys.exit(main())

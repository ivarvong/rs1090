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


# rs1090-cli's `replay` output (one line per detected frame):
#   DFnn HEX clean|corrected:N|failed conf=N ICAO=ABC123 ... cpr-even/odd(LAT,LON) ...
REPLAY_LINE = re.compile(
    r"^DF(?P<df>\d+)\s+(?P<hex>[0-9A-F]+)\s+(?P<crc>clean|corrected:\d+|failed)\s+"
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
    """Yield dicts for every recognised replay line. Order preserved."""
    for line in raw.splitlines():
        m = REPLAY_LINE.match(line)
        if not m:
            continue
        d = {
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


def simulate_pymodes_tracker(frames, reference, pair_window_frames=10):
    """
    Walk the ordered frames and emit per-aircraft positions exactly
    where pyModeS's global decode would produce one, given:

      - even/odd pair within `pair_window_frames` of the same ICAO
        (counted *per ICAO*, not over the global stream — aircraft
        interleave heavily on a busy receiver, so a global window
        spuriously rejects valid pairs as the active-aircraft count
        rises)
      - clean CRC, DF 17/18, has CPR fields
      - both members of the pair pass pms.adsb.position()

    Returns a dict[icao] -> list of (global_frame_index, lat, lon, source).
    """
    # Per-ICAO state: (per_icao_index, global_index, hex) for the most
    # recent even and odd, plus a counter of how many position frames
    # we've seen for this ICAO so far.
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
        st[slot] = (local_idx, i, f["hex"])
        if other in st and abs(st[other][0] - local_idx) <= pair_window_frames:
            even_hex = st["even"][2]
            odd_hex = st["odd"][2]
            # Use the per-ICAO local index as the pseudo-timestamp;
            # pyModeS only uses the *relative* ordering of the two,
            # so any consistent ordering works.
            t_even = st["even"][0] * 1.0
            t_odd = st["odd"][0] * 1.0
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
        type=int,
        default=10,
        help="pyModeS pair window, measured per-ICAO in frames "
        "(default: 10). At ~1 position frame per second per active "
        "aircraft, 10 ≈ the 10 s wall-clock window rs1090's tracker "
        "uses.",
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
            "replay", str(iq_path),
        ]
    )
    frames = list(parse_replay(replay))
    print(f"#   {len(frames)} frames", file=sys.stderr)

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
        frames, (ref_lat, ref_lon), pair_window_frames=args.pair_window
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
            mark = ""
            if delta > args.threshold_nm:
                mark = "  ⚠"
                flagged += 1
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

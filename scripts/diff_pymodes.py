#!/usr/bin/env python3
"""
Differential test: compare rs1090's decoded fields against pyModeS for every
CRC-clean DF11/DF17/DF18 frame produced from a captured .iq corpus.

We can't easily diff against dump1090 (its file-input mode rejects valid
messages on this machine for opaque reasons), but pyModeS is the
canonical ADS-B reference library and gives us a far stronger semantic
check than "do the bit patterns match" — it compares decoded *fields*
against an independent implementation of the spec.

Usage:
    scripts/diff_pymodes.py [path/to/capture.iq]

The script invokes `cargo run --release -p rs1090-cli -- replay <iq>`,
parses the per-frame replay output, runs each CRC-clean frame through
pyModeS.decode(), and reports field-level agreement/disagreement counts
plus a sample of any disagreements.

Requirements: pyModeS>=3 (`pip install pyModeS`), Cargo on PATH.
"""

import argparse
import json
import re
import subprocess
import sys
from collections import Counter, defaultdict
from pathlib import Path

try:
    import pyModeS as pms
except ImportError:
    sys.exit("error: pyModeS not installed. Try `pip install pyModeS`.")


# rs1090 replay line format (from print_frame in rs1090-cli/src/main.rs):
#   DF{nn} {HEX} {clean|corrected:N|failed} conf={N} {decoded summary}
LINE_RE = re.compile(
    r"^DF(?P<df>\d+)\s+"
    r"(?P<hex>[0-9A-F]+)\s+"
    r"(?P<crc>clean|corrected:\d+|failed)\s+"
    r"conf=\d+\s*"
    r"(?P<rest>.*)$"
)

ICAO_RE = re.compile(r"ICAO=([0-9A-F]{6})")
ALT_RE = re.compile(r"alt=(-?\d+)ft")
CPR_RE = re.compile(r"cpr-(even|odd)\((\d+),(\d+)\)")
CALLSIGN_RE = re.compile(r"callsign=(\S+)")
VEL_GS_RE = re.compile(r"gs=(\d+)kt")
VEL_HDG_RE = re.compile(r"hdg=(-?[\d.]+)°")
VEL_VR_RE = re.compile(r"vr=([-+]?\d+)fpm")


def parse_rs1090(line: str) -> dict | None:
    m = LINE_RE.match(line.rstrip())
    if not m:
        return None
    out: dict = {
        "df": int(m["df"]),
        "hex": m["hex"],
        "crc": m["crc"],
    }
    rest = m["rest"]
    if (mm := ICAO_RE.search(rest)):
        out["icao"] = mm.group(1)
    if (mm := ALT_RE.search(rest)):
        out["altitude"] = int(mm.group(1))
    if (mm := CPR_RE.search(rest)):
        out["cpr_format"] = 0 if mm.group(1) == "even" else 1
        out["cpr_lat"] = int(mm.group(2))
        out["cpr_lon"] = int(mm.group(3))
    if (mm := CALLSIGN_RE.search(rest)):
        out["callsign"] = mm.group(1)
    if (mm := VEL_GS_RE.search(rest)):
        out["groundspeed"] = int(mm.group(1))
    if (mm := VEL_HDG_RE.search(rest)):
        out["heading"] = float(mm.group(1))
    if (mm := VEL_VR_RE.search(rest)):
        out["vertical_rate"] = int(mm.group(1))
    return out


# Fields where rs1090 and pyModeS should agree exactly.
COMPARABLE_FIELDS = (
    "icao",
    "altitude",
    "cpr_format",
    "cpr_lat",
    "cpr_lon",
    "callsign",
    "groundspeed",
    "vertical_rate",
)


def field_equal(name: str, a, b) -> bool:
    if a is None or b is None:
        return a == b
    if name == "callsign":
        # pyModeS may include trailing '_' (ICAO padding); strip both.
        return a.rstrip("_").strip() == b.rstrip("_").strip()
    if name == "heading":
        return abs(float(a) - float(b)) < 0.5
    return a == b


def run_replay(iq_path: Path) -> list[str]:
    proc = subprocess.run(
        [
            "cargo", "run", "--release", "--quiet",
            "-p", "rs1090-cli", "--",
            "replay", str(iq_path),
        ],
        capture_output=True,
        check=True,
        text=True,
    )
    return proc.stdout.splitlines()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "iq",
        nargs="?",
        default="corpus/live_1090mhz_2min.iq",
        help="Path to .iq file (default: corpus/live_1090mhz_2min.iq)",
    )
    ap.add_argument(
        "--show-disagreements", type=int, default=5,
        help="Print up to N example disagreements per field (default: 5)",
    )
    args = ap.parse_args()

    iq_path = Path(args.iq)
    if not iq_path.exists():
        sys.exit(f"error: {iq_path} not found")

    print(f"# decoding {iq_path} via rs1090 replay…", file=sys.stderr)
    lines = run_replay(iq_path)
    print(f"# {len(lines)} replay lines", file=sys.stderr)

    # Tallies.
    crc_outcomes: Counter[str] = Counter()
    df_clean: Counter[int] = Counter()
    pms_decode_failed = 0
    frames_compared = 0
    field_agree: Counter[str] = Counter()
    field_disagree: Counter[str] = Counter()
    field_only_rs1090: Counter[str] = Counter()
    field_only_pms: Counter[str] = Counter()
    disagreement_samples: defaultdict[str, list] = defaultdict(list)

    for line in lines:
        parsed = parse_rs1090(line)
        if parsed is None:
            continue
        crc_outcomes[parsed["crc"]] += 1
        if parsed["crc"] != "clean":
            continue
        df_clean[parsed["df"]] += 1
        if parsed["df"] not in (11, 17, 18):
            continue

        try:
            pms_result = pms.decode(parsed["hex"])
        except Exception as e:
            pms_decode_failed += 1
            if pms_decode_failed <= 5:
                print(
                    f"# pyModeS decode error on {parsed['hex']}: {e}",
                    file=sys.stderr,
                )
            continue

        if not pms_result.get("crc_valid", True):
            # rs1090 marked this as clean but pyModeS doesn't. That's a
            # disagreement we want to flag at the CRC level itself.
            field_disagree["crc_valid"] += 1
            disagreement_samples["crc_valid"].append(
                {"hex": parsed["hex"], "rs1090": "clean", "pms": pms_result.get("crc_valid")}
            )
            continue

        frames_compared += 1

        for fname in COMPARABLE_FIELDS:
            rs_val = parsed.get(fname)
            pms_val = pms_result.get(fname)
            if rs_val is None and pms_val is None:
                continue
            if rs_val is not None and pms_val is None:
                field_only_rs1090[fname] += 1
                continue
            if rs_val is None and pms_val is not None:
                field_only_pms[fname] += 1
                continue
            if field_equal(fname, rs_val, pms_val):
                field_agree[fname] += 1
            else:
                field_disagree[fname] += 1
                if len(disagreement_samples[fname]) < args.show_disagreements:
                    disagreement_samples[fname].append(
                        {"hex": parsed["hex"], "rs1090": rs_val, "pms": pms_val}
                    )

    # ----- Report -----
    print()
    print("=" * 70)
    print("rs1090 ↔ pyModeS differential test")
    print("=" * 70)
    print(f"corpus:                {iq_path}")
    print()
    print("CRC outcomes from rs1090:")
    for k in ("clean", "corrected", "failed"):
        n = sum(v for kk, v in crc_outcomes.items() if kk.startswith(k))
        print(f"  {k:>10}: {n}")
    print()
    print(f"CRC-clean DF distribution: {dict(sorted(df_clean.items()))}")
    print()
    print(f"pyModeS decode failures: {pms_decode_failed}")
    print(f"frames compared:         {frames_compared}")
    print()
    print(f"{'field':<16}  {'agree':>6}  {'disagree':>9}  {'rs1090-only':>11}  {'pms-only':>8}")
    for fname in COMPARABLE_FIELDS:
        a = field_agree[fname]
        d = field_disagree[fname]
        r = field_only_rs1090[fname]
        p = field_only_pms[fname]
        if a + d + r + p == 0:
            continue
        print(f"{fname:<16}  {a:>6}  {d:>9}  {r:>11}  {p:>8}")

    if any(disagreement_samples.values()):
        print()
        print("--- disagreement samples ---")
        for fname, samples in disagreement_samples.items():
            print(f"  {fname}:")
            for s in samples:
                print(f"    {json.dumps(s)}")

    if sum(field_disagree.values()) == 0:
        print()
        print("✓ no field-level disagreements")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())

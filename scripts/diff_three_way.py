#!/usr/bin/env python3
"""
Three-way consensus verification: run dump1090-fa, readsb, and rs1090
on the same UC8 capture, then score each decoder against the 2-of-3
consensus.

The capture must come from a tool *outside* the rs1090 pipeline so a
bug in rs1090's IQ-write path can't pre-corrupt the input. The
standard `rtl_sdr` CLI is what we use:

    rtl_sdr -f 1090000000 -s 2400000 -g 0 -n N capture.uc8

(2.4 MS/s is dump1090's and readsb's native rate; UC8 is what
`rtl_sdr` writes natively. rs1090 reads UC8 too as of the
multi-rate refactor.)

Why three decoders, not two: dump1090-fa and readsb are independent
demodulator implementations (different forks, different DSP), and
when they disagree on a frame's existence there's no single "ground
truth" to defer to. The 2-of-3 consensus approximates ground truth
by treating "any frame at least two of the three decoders found" as
real. The diff harness can then measure each decoder's:

  - **recall** — how many consensus frames did this decoder find?
  - **precision** — how many of this decoder's frames are in the
    consensus set? (frames only this decoder found are "ghosts" —
    either real frames the others missed, or false positives.)

Once the verification metric exists, any future demod work has a
falsifiable target: "does this change move recall up without dropping
precision?"

Usage:
    scripts/diff_three_way.py corpus/<file>.uc8
"""

import argparse
import re
import subprocess
import sys
from pathlib import Path

# Path to the dump1090-fa binary; the Homebrew formula keeps it
# under its `opt` keg with the generic name `dump1090`, so we have
# to spell out the full path to avoid colliding with the
# `dump1090-mutability` binary that the `dump1090` shim on PATH
# points to.
DUMP1090_FA = "/opt/homebrew/opt/dump1090-fa/bin/dump1090"

# Wire format note: dump1090 / readsb emit `*HEX;` per line; rs1090
# emits `[T+seconds ]DFnn HEX clean|... conf=N ...`. We normalise both
# to a set of uppercase hex strings.
DUMP_RE = re.compile(r"^\*([0-9A-Fa-f]+);\s*$")
RS_RE = re.compile(
    r"^(?:T\+[\d.]+\s+)?DF\d+\s+(?P<hex>[0-9A-F]+)\s+(?P<crc>clean|corrected:\d+|failed)\b"
)


def run_dump1090_fa(uc8_path: Path) -> set[str]:
    proc = subprocess.run(
        [
            DUMP1090_FA, "--ifile", str(uc8_path), "--iformat", "UC8",
            "--raw", "--no-fix",
        ],
        capture_output=True, text=True, check=False,
    )
    frames: set[str] = set()
    for line in proc.stdout.splitlines():
        m = DUMP_RE.match(line)
        if m:
            frames.add(m.group(1).upper())
    return frames


def run_readsb(uc8_path: Path) -> set[str]:
    proc = subprocess.run(
        [
            "readsb",
            "--device-type=ifile",
            f"--ifile={uc8_path}",
            "--iformat=UC8",
            "--raw",
            "--no-interactive",
            "--no-fix",
        ],
        capture_output=True, text=True, check=False, timeout=600,
    )
    frames: set[str] = set()
    for line in proc.stdout.splitlines():
        m = DUMP_RE.match(line)
        if m:
            frames.add(m.group(1).upper())
    return frames


def run_rs1090(uc8_path: Path, sample_rate: int) -> set[str]:
    proc = subprocess.run(
        [
            "cargo", "run", "--release", "--quiet", "-p", "rs1090-cli", "--",
            "replay", "--format", "uc8", "--sample-rate", str(sample_rate),
            str(uc8_path),
        ],
        capture_output=True, text=True, check=True,
    )
    frames: set[str] = set()
    for line in proc.stdout.splitlines():
        m = RS_RE.match(line)
        if not m:
            continue
        # Match dump1090/readsb's --no-fix: include only CRC-clean
        # frames so the comparison is at the same level.
        if m.group("crc") != "clean":
            continue
        frames.add(m.group("hex").upper())
    return frames


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("uc8")
    ap.add_argument("--sample-rate", type=int, default=2_400_000)
    args = ap.parse_args()
    uc8 = Path(args.uc8)
    if not uc8.exists():
        sys.exit(f"error: {uc8} not found")

    print(f"# corpus: {uc8}  ({uc8.stat().st_size / 1e6:.1f} MB)", file=sys.stderr)

    print("# dump1090-fa…", file=sys.stderr)
    fa = run_dump1090_fa(uc8)
    print(f"#   {len(fa)} unique CRC-clean frames", file=sys.stderr)

    print("# readsb…", file=sys.stderr)
    rs = run_readsb(uc8)
    print(f"#   {len(rs)} unique CRC-clean frames", file=sys.stderr)

    print(f"# rs1090 (replay --format uc8 --sample-rate {args.sample_rate})…", file=sys.stderr)
    ours = run_rs1090(uc8, args.sample_rate)
    print(f"#   {len(ours)} unique CRC-clean frames", file=sys.stderr)

    # Consensus: any frame at least two of the three decoders found.
    union = fa | rs | ours
    consensus = {f for f in union if (f in fa) + (f in rs) + (f in ours) >= 2}

    # Per-decoder recall/precision against the consensus.
    def score(found: set[str], label: str) -> dict:
        tp = len(found & consensus)
        fp = len(found - consensus)
        fn = len(consensus - found)
        recall = tp / len(consensus) if consensus else 0.0
        precision = tp / len(found) if found else 0.0
        return {
            "label": label, "total": len(found),
            "tp": tp, "fp": fp, "fn": fn,
            "recall": recall, "precision": precision,
        }

    rows = [score(fa, "dump1090-fa"), score(rs, "readsb"), score(ours, "rs1090")]

    print()
    print("=" * 78)
    print("Three-way consensus verification")
    print("=" * 78)
    print(f"union of all three:  {len(union):>6} unique frames")
    print(f"2-of-3 consensus:    {len(consensus):>6} unique frames (treated as 'real')")
    print()
    print(
        f"{'decoder':<14} {'total':>7} {'tp':>7} {'fp':>7} {'fn':>7} "
        f"{'recall':>9} {'prec':>9}"
    )
    print("-" * 78)
    for r in rows:
        print(
            f"{r['label']:<14} {r['total']:>7} {r['tp']:>7} {r['fp']:>7} {r['fn']:>7} "
            f"{r['recall']*100:>8.1f}% {r['precision']*100:>8.1f}%"
        )
    print()
    # Pairwise frame agreement on the union — useful sanity check.
    print("pairwise overlap (|A ∩ B|):")
    print(f"  fa ∩ readsb:  {len(fa & rs):>5}")
    print(f"  fa ∩ rs1090:  {len(fa & ours):>5}")
    print(f"  readsb ∩ rs1090:  {len(rs & ours):>5}")
    print()
    if rows[2]["recall"] >= 0.9:
        print("✓ rs1090 recall ≥ 90% of consensus")
        return 0
    print(f"⚠ rs1090 recall is {rows[2]['recall']*100:.1f}% — work to do")
    return 1


if __name__ == "__main__":
    sys.exit(main())

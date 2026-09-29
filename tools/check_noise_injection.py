#!/usr/bin/env python3
"""
check_noise_injection.py -- did any released session run with firmware noise injection on?

Background (C-02b). Several branches of Houserz/Shield were built with
`#define NOISE_INJECTION true` (main, setup, sparsh-changes, the B4 scaffold 216572e):
30 minutes into a session every driver multiplies its value by (1 + g), where g is a
Gaussian with s.d. 0.4 clamped to [-1, 1] (gaussian.cpp), i.e. about 40 % multiplicative
noise on every sample of every channel. Builds on shaker-table-tests and later branches
have it off. This tool looks for the signature in recorded sessions, without needing to know
which firmware was flashed:

  * `noise_data.bin` present in the RUN folder (sparsh-changes writes one per session)
  * a step in sample-to-sample scatter of the slow-tier channels (temperature, pressure)
    between the 30 minutes before and after the injection start. Injection makes the
    median absolute successive difference jump by orders of magnitude (22 C x 0.4 = ~9 C);
    a cold-bath ramp does not, because the median ignores slow trends.

Usage:  python check_noise_injection.py <folder-with-RUN_xxx-folders> [more folders] [--split-min 30]
Exit code 0 if no session is flagged and every session could be read, 1 if any session is flagged,
3 if none is flagged but some slow_data.bin files could not be read (cloud-only OneDrive files: pin the
folder and rerun; unreadable is not the same as clean). A flag means "look at this session", not proof;
an unflagged session shorter than split + 5 min cannot be judged and is listed as such.

slow_data.bin layout (unchanged): 12 B records <u32 timestamp_ms, u8 sensor_id, 3 pad, f32>;
sensor ids 3 = pressure, 4 = temperature. Timestamps are tick x 10 ms (see C-02).
"""
import argparse
import os
import struct
import sys
from collections import defaultdict

REC = "<IB3xf"
SZ = struct.calcsize(REC)
NAMES = {3: "pressure", 4: "temperature"}
RATIO_FLAG = 5.0


def read_slow(path):
    series = defaultdict(list)
    if not os.path.isfile(path):
        return series
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError:
        # cloud-only OneDrive placeholder (or locked file): cannot be judged, not the same as clean
        return None
    for i in range(len(data) // SZ):
        t, sid, v = struct.unpack_from(REC, data, i * SZ)
        if sid in NAMES:
            series[sid].append((t / 60000.0, v))
    return series


def med_abs_diff(pairs):
    d = sorted(abs(pairs[i + 1][1] - pairs[i][1]) for i in range(len(pairs) - 1))
    if not d:
        return None
    m = len(d) // 2
    return d[m] if len(d) % 2 else 0.5 * (d[m - 1] + d[m])


def check_run(run_dir, split_min):
    out = {"run": os.path.basename(run_dir), "path": run_dir, "noise_file_bytes": None, "duration_min": 0.0, "ratios": {}, "flag": False, "note": ""}
    nf = os.path.join(run_dir, "noise_data.bin")
    if os.path.isfile(nf):
        out["noise_file_bytes"] = os.path.getsize(nf)
        if out["noise_file_bytes"] > 0:
            out["flag"] = True
            out["note"] += "non-empty noise_data.bin; "
    series = read_slow(os.path.join(run_dir, "slow_data.bin"))
    if series is None:
        out["unreadable"] = True
        out["note"] += "slow_data.bin unreadable (cloud-only OneDrive file? pin the folder with attrib +P -U and rerun); "
        return out
    if not series:
        out["note"] += "no slow-tier data; "
        return out
    # Times are made relative to the first slow sample of the session: the sequencer builds (A1 batch, B4)
    # keep one running clock across sessions, so a session can start at minute 82 of the board's uptime.
    t0 = min(p[0][0] for p in series.values())
    for sid in list(series):
        series[sid] = [(t - t0, v) for t, v in series[sid]]
    out["duration_min"] = max(p[-1][0] for p in series.values())
    if out["duration_min"] < split_min + 5:
        out["note"] += f"shorter than {split_min + 5:.0f} min: injection would not have started or cannot be judged; "
        return out
    for sid, pairs in series.items():
        early = [p for p in pairs if 2.0 <= p[0] < split_min]
        late = [p for p in pairs if split_min + 1.0 <= p[0] <= split_min + 15.0]
        a, b = med_abs_diff(early), med_abs_diff(late)
        if a is None or b is None:
            continue
        ratio = b / a if a > 0 else (float("inf") if b > 0 else 1.0)
        out["ratios"][NAMES[sid]] = ratio
        if ratio > RATIO_FLAG:
            out["flag"] = True
            out["note"] += f"{NAMES[sid]} scatter x{ratio:.0f} after minute {split_min}; "
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("roots", nargs="+")
    ap.add_argument("--split-min", type=float, default=30.0, help="NOISE_INJECTION_DELAY in minutes (30 on all affected builds)")
    args = ap.parse_args()
    runs = []
    for root in args.roots:
        for dirpath, dirnames, filenames in os.walk(root):
            if "slow_data.bin" in filenames or "noise_data.bin" in filenames:
                runs.append(dirpath)
    if not runs:
        print("no session folders (containing slow_data.bin) found")
        sys.exit(2)
    results = [check_run(r, args.split_min) for r in sorted(runs)]
    print(f"{'run folder (last 4 path parts)':<58} {'min':>6} {'noise_bytes':>11} {'temp x':>8} {'press x':>8}  verdict")
    for r in results:
        rt, rp = r["ratios"].get("temperature"), r["ratios"].get("pressure")
        parts = r["path"].replace("\\", "/").rstrip("/").split("/")
        name = "/".join(parts[-4:])
        name = name if len(name) <= 58 else "…" + name[-57:]
        print(f"{name:<58} {r['duration_min']:>6.0f} {str(r['noise_file_bytes']):>11} "
              f"{('%.1f' % rt) if rt is not None else '-':>8} {('%.1f' % rp) if rp is not None else '-':>8}  "
              f"{'FLAG: ' if r['flag'] else ('UNREADABLE: ' if r.get('unreadable') else 'ok: ')}{r['note']}")
    flagged = [r for r in results if r["flag"]]
    judged = [r for r in results if r["ratios"]]
    unreadable = [r for r in results if r.get("unreadable")]
    print(f"\n{len(results)} sessions, {len(judged)} long enough to judge, {len(flagged)} flagged, {len(unreadable)} unreadable")
    if flagged:
        sys.exit(1)
    if unreadable:
        print("Not a clean result: the unreadable sessions above were not checked.")
        sys.exit(3)
    sys.exit(0)


if __name__ == "__main__":
    main()

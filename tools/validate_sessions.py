#!/usr/bin/env python3
"""
validate_sessions.py -- pass/fail check for the A2 platform validation (C-12)
and a per-session sanity report for the review-1 campaign.

Reads every RUN_xxx folder under one or more roots (an SD card, or copied
folders) and reports, per session:

  * tier-file sizes read from disk (fast/medium/slow) and whether any is empty
  * meta.json close_report (close_ok, sizes the firmware read back) and
    statistics (samples, queue_overruns, sd_write_errors)
  * board_mac, firmware_git, freertos_tick_hz, rtc_set, BNO085 calibration
    config as written by the firmware
  * G0.11 channel-identity check on medium_data.bin: fraction of consecutive
    (current, photodiode) pairs with identical stored values. The released
    Tier 2 corpus fails this with ~1.0 (photodiode never converted); a
    passing session is far below 0.05.
  * achieved rates per channel from timestamp_ms (median spacing)
  * number of MARK lines in events.log

Exit code 0 when every session passes, 1 otherwise. The A2 pass criterion is
10 of 10 sessions passing (5 per board, each from a power cycle, >= 20 min).

Record layouts (unchanged from the released firmware):
  fast_data.bin   16 B: u32 timestamp_ms, u8 sensor_id, 3 pad, 3 x f32
  medium_data.bin 12 B: u32 timestamp_ms, u8 sensor_id, 3 pad, f32
  slow_data.bin   12 B: u32 timestamp_ms, u8 sensor_id, 3 pad, f32
"""
import argparse
import json
import os
import struct
import sys
from collections import defaultdict

FAST_FMT = "<IB3x3f"
MED_FMT = "<IB3xf"
FAST_SZ = struct.calcsize(FAST_FMT)   # 16
MED_SZ = struct.calcsize(MED_FMT)     # 12

CHANNEL_NAMES = {1: "vibration", 2: "current", 3: "pressure", 4: "temperature",
                 5: "microphone", 6: "photodiode", 7: "magnetometer", 8: "gyroscope", 9: "accelerometer"}

MIN_DURATION_S = 20 * 60
G011_MAX_IDENTICAL_FRACTION = 0.05


def read_records(path, fmt, size):
    out = []
    if not os.path.isfile(path):
        return out
    with open(path, "rb") as f:
        data = f.read()
    n = len(data) // size
    for i in range(n):
        out.append(struct.unpack_from(fmt, data, i * size))
    return out


def median(xs):
    xs = sorted(xs)
    if not xs:
        return None
    m = len(xs) // 2
    return xs[m] if len(xs) % 2 else 0.5 * (xs[m - 1] + xs[m])


def achieved_rates(records):
    """median timestamp spacing per sensor id -> Hz (timestamps are 10 ms ticks on the 100 Hz build)"""
    ts = defaultdict(list)
    for r in records:
        ts[r[1]].append(r[0])
    rates = {}
    for sid, t in ts.items():
        if len(t) < 3:
            continue
        # mean rate over the span is robust to 10 ms quantisation
        span_s = (t[-1] - t[0]) / 1000.0
        rates[sid] = (len(t) - 1) / span_s if span_s > 0 else None
    return rates


def g011_identity(medium_records):
    """fraction of consecutive current/photodiode pairs whose stored floats are identical"""
    cur = [r for r in medium_records if r[1] == 2]
    pho = [r for r in medium_records if r[1] == 6]
    n = min(len(cur), len(pho))
    if n == 0:
        return None, 0
    same = sum(1 for i in range(n) if cur[i][2] == pho[i][2])
    return same / n, n


def check_session(run_dir):
    res = {"run": os.path.basename(run_dir), "pass": True, "reasons": []}
    sizes = {}
    for tier in ("fast", "medium", "slow"):
        p = os.path.join(run_dir, f"{tier}_data.bin")
        sizes[tier] = os.path.getsize(p) if os.path.isfile(p) else -1
    res["sizes"] = sizes
    for tier, sz in sizes.items():
        if sz <= 0:
            res["pass"] = False
            res["reasons"].append(f"{tier}_data.bin missing or empty")

    meta_path = os.path.join(run_dir, "meta.json")
    meta = {}
    if os.path.isfile(meta_path):
        try:
            with open(meta_path) as f:
                meta = json.load(f)
        except Exception as e:  # noqa
            res["reasons"].append(f"meta.json unreadable: {e}")
            res["pass"] = False
    else:
        res["reasons"].append("meta.json missing")
        res["pass"] = False

    dev = meta.get("device_info", {})
    res["board_mac"] = dev.get("board_mac")
    res["firmware_git"] = dev.get("firmware_git")
    res["tick_hz"] = dev.get("freertos_tick_hz")
    res["rtc_set"] = meta.get("rtc_set")
    res["cal_config"] = meta.get("bno085", {}).get("dynamic_calibration_config")
    stats = meta.get("statistics", {})
    res["samples"] = stats.get("total_samples", {})
    res["queue_overruns"] = stats.get("queue_overruns")
    res["sd_write_errors"] = stats.get("sd_write_errors")
    res["duration_s"] = (stats.get("duration_ms") or 0) / 1000.0
    cr = meta.get("close_report", {})
    res["close_ok"] = cr.get("close_ok")
    res["marker_count"] = cr.get("marker_count")

    if res["close_ok"] is False:
        res["pass"] = False
        res["reasons"].append("firmware close_report.close_ok = false")
    if res["sd_write_errors"]:
        res["pass"] = False
        res["reasons"].append(f"sd_write_errors = {res['sd_write_errors']}")
    if res["duration_s"] < MIN_DURATION_S:
        res["pass"] = False
        res["reasons"].append(f"duration {res['duration_s']:.0f} s < {MIN_DURATION_S} s")
    if res["board_mac"] in (None, "not-read", "read-failed"):
        res["reasons"].append("board_mac not recorded")
        res["pass"] = False

    # G0.11 on the medium tier
    med = read_records(os.path.join(run_dir, "medium_data.bin"), MED_FMT, MED_SZ)
    frac, npairs = g011_identity(med)
    res["g011_identical_fraction"] = frac
    res["g011_pairs"] = npairs
    if frac is None:
        res["pass"] = False
        res["reasons"].append("no current/photodiode pairs")
    elif frac > G011_MAX_IDENTICAL_FRACTION:
        res["pass"] = False
        res["reasons"].append(f"G0.11 fails: {frac:.3f} of pairs identical (photodiode not converting)")

    # rates
    fast = read_records(os.path.join(run_dir, "fast_data.bin"), FAST_FMT, FAST_SZ)
    slow = read_records(os.path.join(run_dir, "slow_data.bin"), MED_FMT, MED_SZ)
    rates = {}
    for recs in (fast, med, slow):
        for sid, hz in achieved_rates(recs).items():
            rates[CHANNEL_NAMES.get(sid, str(sid))] = round(hz, 1) if hz else None
    res["rates_hz"] = rates
    missing = [CHANNEL_NAMES[s] for s in CHANNEL_NAMES if CHANNEL_NAMES[s] not in rates]
    if missing:
        res["pass"] = False
        res["reasons"].append(f"channels absent: {', '.join(missing)}")

    # markers
    ev = os.path.join(run_dir, "events.log")
    marks = 0
    if os.path.isfile(ev):
        with open(ev, errors="replace") as f:
            marks = sum(1 for line in f if "] MARK:" in line)
    res["markers_in_log"] = marks
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("roots", nargs="+", help="SD card root(s) or folders containing RUN_xxx")
    ap.add_argument("--expect", type=int, default=10, help="sessions expected to pass (A2: 10)")
    ap.add_argument("--json", help="write the full report here")
    args = ap.parse_args()

    runs = []
    for root in args.roots:
        for name in sorted(os.listdir(root)):
            if name.startswith("RUN_") and os.path.isdir(os.path.join(root, name)):
                runs.append(os.path.join(root, name))
    if not runs:
        print("no RUN_xxx folders found")
        sys.exit(1)

    results = [check_session(r) for r in runs]
    passed = sum(1 for r in results if r["pass"])

    print(f"{'run':<9} {'pass':<5} {'fast_B':>10} {'med_B':>9} {'slow_B':>8} {'dur_s':>6} {'ovr':>6} {'g011':>6} {'mac':<18} {'git':<14} reasons")
    for r in results:
        s = r["sizes"]
        g011 = r["g011_identical_fraction"]
        g011_txt = f"{g011:.3f}" if g011 is not None else "n/a"
        reasons = "; ".join(r["reasons"])
        print(f"{r['run']:<9} {'OK' if r['pass'] else 'FAIL':<5} {s['fast']:>10} {s['medium']:>9} {s['slow']:>8} "
              f"{r['duration_s']:>6.0f} {str(r['queue_overruns']):>6} {g011_txt:>6} "
              f"{str(r['board_mac']):<18} {str(r['firmware_git'])[:14]:<14} {reasons}")
        print(f"          rates: {r['rates_hz']}  markers: {r['markers_in_log']}  cal_config: {r['cal_config']}  rtc_set: {r['rtc_set']}")

    macs = sorted({r["board_mac"] for r in results if r["board_mac"]})
    print(f"\n{passed} of {len(results)} sessions pass; boards seen: {macs}")
    if args.json:
        with open(args.json, "w") as f:
            json.dump(results, f, indent=2)
    verdict = passed >= args.expect and passed == len(results)
    print("A2 VALIDATION:", "PASS" if verdict else "FAIL")
    sys.exit(0 if verdict else 1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
campaign_finish.py -- after the card is pulled: validate the session, fill its row
in session_log.csv from the files, copy the RUN folder, and run the log checker.

  python tools\\campaign_finish.py A1-U1-S-low --card D:\\ --outdir campaign_runs ^
      --operator SD --equipment-id "shaker S/N ..., controller COM7" ^
      --room-temp-start 21.3 --room-temp-end 21.6 --room-pressure-start 1008.2 --room-pressure-end 1008.0 ^
      --ref-thermometer "model/id" --ref-barometer "model/id" --copy-to D_backup

What is read from the files (nothing typed twice): board MAC, firmware hash, RUN folder,
rtc_set (meta.json on the card); power-on wall-clock, marks, close line, overruns, ADS
timeouts, commanded shaker setting (the session meta written by campaign_session.py);
validate_result (validate_sessions.py on that RUN folder). What only you can supply is
passed as an option; anything you leave out is written `not recorded` (never estimated).
Deviations are written `none` unless you pass --deviation or the script found one
(an unacknowledged mark, an unconfirmed event, a manual mark, CLOSE not OK).

A row that already has a date is not overwritten unless you pass --overwrite.
Use --exclude "reason" to mark a failed session excluded (the checker stops the day on a
FAIL that is not excluded).
"""
import argparse
import csv
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
CAMPAIGN_DIR = (r"C:\Users\Metal\OneDrive - UW\Team Sentinel - Documents\General"
                r"\SHIBench Paper Collaboration\review1\campaign_logs")
NR = "not recorded"
# Build of record for the campaign: the session-loop build (branch review1-campaign).
# 0cbcfa446713 was the C-12 build; the session loop and the console fix moved the hash.
FW_VALIDATED = "168eb16c4aef"

# Which physical board is which unit, from C-10 (UNIT_0001 = SHIELD 1, UNIT_0002 = SHIELD 2)
# and the esptool MACs read at flash time. The session log gets board_unit from the schedule
# and board_mac from the card, so a swapped pair of console ports would relabel one board's
# data as the other's -- consistently, which means check_session_log.py's one-mac-per-unit
# rule would NOT catch it. This table is the thing that catches it.
UNIT_MAC = {
    "UNIT_0001": "1c:db:d4:ae:74:64",
    "UNIT_0002": "1c:db:d4:ae:d5:28",
}


def find_run(card, run_id):
    for root, dirs, files in os.walk(card):
        if os.path.basename(root) == run_id and "meta.json" in files:
            return root
    return None


def setting_planned(row):
    c, p = row["condition"], row["parameter"]
    if c == "A1":
        return {"low": "80 RPM", "medium": "140 RPM", "high": "200 RPM"}.get(p, "0 RPM (handled-healthy control)")
    if c == "A2":
        return f"{p} cold exposure" if row["arm"] == "S" else f"{p} room-temperature bag (handled-healthy)"
    return f"{c} {p} ({row['arm']})"


def setting_actual(sm):
    parts = []
    t0 = None
    for e in sm.get("events", []):
        if e["event"] == "settle_start":
            t0 = e["elapsed_real_s"]
    for e in sm.get("events", []):
        if e["event"] in ("settle_start",) or e["event"].startswith("break_cue"):
            continue
        s = e.get("elapsed_real_s")
        sc = sm.get("time_scale") or 1.0
        stamp = "" if s is None else f"@{int(s * sc // 60)}:{int(s * sc % 60):02d}"
        flag = "" if e.get("confirmed", True) else "(unconfirmed)"
        parts.append(f"{e['event']}{stamp}{flag}")
    if sm.get("condition") == "A1" and sm.get("target_rpm") is not None:
        parts.insert(0, f"commanded {sm['target_rpm']} RPM (accel log {sm['session_id']}_shaker_accel.csv)")
    return "; ".join(parts) if parts else NR


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_id")
    ap.add_argument("--card", required=True, help="the mounted card (e.g. D:\\) or a folder holding the RUN folder")
    ap.add_argument("--outdir", default="campaign_runs", help="where campaign_session.py wrote the session meta")
    ap.add_argument("--run", help="RUN folder name if the meta did not capture it")
    ap.add_argument("--log", default="session_log.csv")
    ap.add_argument("--schedule", default=os.path.join(CAMPAIGN_DIR, "campaign_schedule_planned.csv"))
    ap.add_argument("--template", default=os.path.join(CAMPAIGN_DIR, "session_log_template.csv"))
    ap.add_argument("--checker", default=os.path.join(CAMPAIGN_DIR, "check_session_log.py"))
    ap.add_argument("--fw", default=FW_VALIDATED, help="firmware hash that passed C-12")
    ap.add_argument("--operator", default=NR)
    ap.add_argument("--equipment-id", default=NR)
    ap.add_argument("--room-temp-start", default=NR)
    ap.add_argument("--room-temp-end", default=NR)
    ap.add_argument("--room-pressure-start", default=NR)
    ap.add_argument("--room-pressure-end", default=NR)
    ap.add_argument("--ref-thermometer", default=NR)
    ap.add_argument("--ref-barometer", default=NR)
    ap.add_argument("--deviation", default=None)
    ap.add_argument("--exclude", default=None, help="exclusion reason; marks the row excluded = Y")
    ap.add_argument("--notes", default="")
    ap.add_argument("--copy-to", help="copy the RUN folder to <dir>/<session_id>/<RUN_xxx>")
    ap.add_argument("--min-duration-min", type=float, default=None,
                    help="override the duration the validator requires (default: planned end - 1 min)")
    ap.add_argument("--allow-mac-mismatch", action="store_true",
                    help="write the row even if the card's MAC is not the one UNIT_MAC expects "
                         "for this session's board (use only if the table is wrong, not the run)")
    ap.add_argument("--overwrite", action="store_true")
    args = ap.parse_args()

    import validate_sessions as vs  # noqa: E402  (same folder)

    mpath = os.path.join(args.outdir, f"{args.session_id}_session_meta.json")
    if not os.path.isfile(mpath):
        sys.exit(f"{mpath} not found: was this session run with campaign_session.py in this --outdir?")
    with open(mpath, encoding="utf-8") as f:
        sm = json.load(f)
    run_id = args.run or sm.get("run_id")
    if not run_id:
        sys.exit("the session meta has no RUN id; pass --run RUN_xxx")
    run_dir = find_run(args.card, run_id)
    if not run_dir:
        sys.exit(f"{run_id}/meta.json not found under {args.card}")

    end_s = (sm.get("planned_s") or {}).get("end") or 0
    vs.MIN_DURATION_S = (args.min_duration_min * 60.0) if args.min_duration_min else max(60.0, end_s - 60.0)
    r = vs.check_session(run_dir)
    verdict = "PASS" if r["pass"] else "FAIL"
    print(f"validate {run_id}: {verdict}" + ("" if r["pass"] else "  <- " + "; ".join(r["reasons"])), flush=True)

    # deviations found automatically
    dev = []
    n_marks = len(sm.get("board_marks", []))
    n_ack = sum(1 for m in sm.get("board_marks", []) if m.get("reply"))
    if n_ack != n_marks:
        dev.append(f"{n_marks - n_ack} of {n_marks} marks not acknowledged by the board")
    if sm.get("unconfirmed_events"):
        dev.append("events marked without operator confirmation: " + ",".join(sm["unconfirmed_events"]))
    if sm.get("manual_marks"):
        dev.append(f"{sm['manual_marks']} manual mark(s) sent")
    if sm.get("close") != "OK":
        dev.append(f"firmware close line: {sm.get('close')}")
    if sm.get("abort_reason"):
        dev.append(f"runner aborted: {sm['abort_reason']}")
    if sm.get("relabel_note"):
        # the session was launched under the wrong id; relabel_session.py recorded why.
        dev.append(sm["relabel_note"])
    if sm.get("dry_run"):
        dev.append("TEST MODE (time-scale) -- not a real session")
    if args.deviation:
        dev.append(args.deviation)
    banner_local = (sm.get("banner_local") or "")
    with open(args.schedule, newline="", encoding="utf-8") as f:
        sched = {x["session_id"]: x for x in csv.DictReader(f)}
    srow = sched[args.session_id]

    # Board identity: the card's MAC must be the one that belongs to this session's unit.
    want = UNIT_MAC.get(srow["board"])
    got = (r.get("board_mac") or "").strip().lower()
    if want and got and got != want.lower() and not args.allow_mac_mismatch:
        other = [u for u, m in UNIT_MAC.items() if m.lower() == got]
        sys.exit(f"BOARD IDENTITY MISMATCH: {args.session_id} is a {srow['board']} session, "
                 f"whose board is {want}, but {run_id} on this card was recorded by {got}"
                 + (f" -- that is {other[0]}. The two console ports were almost certainly given "
                    f"in the wrong order, so this run's board labels are swapped." if other else ".")
                 + "\nNothing was written to the session log. Fix the mapping (or pass "
                   "--allow-mac-mismatch if you are certain the table is what is wrong).")

    row = {
        "session_id": args.session_id,
        "date": banner_local[:10] or NR,
        "operator": args.operator,
        "board_unit": srow["board"],
        "board_mac": r.get("board_mac") or NR,
        "firmware_git": str(r.get("firmware_git") or NR),
        "run_folder": run_id,
        "poweron_wallclock_local": banner_local[11:19] or NR,
        "rtc_set": "yes" if r.get("rtc_set") else "no",
        "marks_source": "console MARK sent by campaign_session.py",
        "stress_setting_planned": setting_planned(srow),
        "stress_setting_actual": setting_actual(sm),
        "equipment_id": args.equipment_id,
        "room_temp_start_C": args.room_temp_start, "room_temp_end_C": args.room_temp_end,
        "room_pressure_start_hPa": args.room_pressure_start, "room_pressure_end_hPa": args.room_pressure_end,
        "ref_thermometer_id": args.ref_thermometer, "ref_barometer_id": args.ref_barometer,
        "validate_result": verdict,
        "excluded": "Y" if args.exclude else "N",
        "exclusion_reason": args.exclude or "",
        "deviations": "; ".join(dev) if dev else "none",
        "notes": args.notes or (f"overruns {sm.get('overruns')}, ads_timeouts {sm.get('ads_timeouts')}, "
                                f"ads_i2c_err {sm.get('ads_i2c_err')}, self-test attempts {sm.get('selftest_attempts')}"),
    }

    if not os.path.isfile(args.log):
        shutil.copyfile(args.template, args.log)
    with open(args.log, newline="", encoding="utf-8") as f:
        rd = csv.DictReader(f)
        fields = rd.fieldnames
        rows = list(rd)
    hit = [x for x in rows if x["session_id"] == args.session_id]
    if not hit:
        sys.exit(f"{args.session_id} is not a row of {args.log}")
    if (hit[0].get("date") or "").strip() and not args.overwrite:
        sys.exit(f"{args.session_id} already has a row (date {hit[0]['date']}); use --overwrite to replace it")
    for x in rows:
        if x["session_id"] == args.session_id:
            x.update({k: v for k, v in row.items() if k in fields})
    tmp = args.log + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    os.replace(tmp, args.log)
    print(f"row written to {args.log}", flush=True)

    if args.copy_to:
        dst = os.path.join(args.copy_to, args.session_id, run_id)
        if os.path.exists(dst):
            print(f"copy skipped: {dst} exists")
        else:
            shutil.copytree(run_dir, dst)
            print(f"RUN folder copied to {dst}")

    if os.path.isfile(args.checker):
        print("\n--- check_session_log.py ---", flush=True)
        subprocess.run([sys.executable, args.checker, args.log, "--schedule", args.schedule,
                        "--cards", args.card, "--fw", args.fw])
    else:
        print(f"(checker not found at {args.checker}; run it by hand)")
    sys.exit(0 if verdict == "PASS" or args.exclude else 1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
bench_validate_run.py -- drive the C-12 cold-start validation (or any run of
identical sessions) from one command.

Start it ONCE with both boards' console ports. For each session it waits for
the board's boot banner (you power-cycle the boards: unplug, wait 10 s, plug in),
then it does what the paper sheet asks of the operator:

  * sends  SETTIME <epoch>  inside the firmware's 5 s window
  * sends  MARK <label>     at each requested time (default 5:00 and 15:00)
  * sends  STOP             after --minutes (default 21, so duration >= 20 min)
  * waits for the firmware's  CLOSE OK / CLOSE FAIL  line and records it
  * appends a row to the CSV (port, session, firmware hash, RUN id if seen,
    SD self-test attempts, close line, ADS timeouts/I2C errors, markers)

Both ports are handled in parallel, so each power cycle is one cold-start
session for each board. After the last session it prints the exact
validate_sessions.py command to run on the pulled cards.

Why the ports are opened first: opening a USB-UART port can toggle DTR/RTS and
reset an ESP32. Opening them before power-on means any such reset happens
while the board is unpowered. DTR/RTS are forced low before the open.

Usage (PowerShell, from the repo root):
  python tools\\bench_validate_run.py COM8 COM6 --sessions 5
  python tools\\bench_validate_run.py COM8 COM6 --sessions 5 --minutes 21 --marks 5:00,15:00 --out validation_2026-10-08

Add --dry-seconds 20 to test the whole flow with a 20 s session (the sessions
will then fail the 20 min duration check; do not use that data).
"""
import argparse
import csv
import datetime
import os
import re
import sys
import threading
import time

import serial

BAUD = 115200
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
BANNER_WAIT = "Waiting 5 s for optional"
RE_STARTING = re.compile(r"Project SHIELD starting.*build\s+(\S+?)\)?\s*=")
RE_SELFTEST = re.compile(r"self-test passed on attempt (\d+)")
RE_RUN = re.compile(r"(RUN_\d{3})")
RE_CLOSE = re.compile(r"CLOSE (OK|FAIL):")
RE_ADS = re.compile(r"ads_timeouts=(\d+)\s+ads_i2c_err=(\d+)\s+markers=(\d+)")
RE_OVR = re.compile(r"overruns=(\d+)\s+sd_errors=(\d+)")
BAD_BOOT = ("SD self-test FAILED", "Run session creation FAILED", "mkdir failed")

FIELDS = ["wall_start", "port", "session", "firmware_hash", "run_id", "selftest_attempts",
          "close", "close_line", "overruns", "sd_errors", "ads_timeouts", "ads_i2c_err",
          "markers_sent", "markers_acked", "elapsed_s", "boot_failure", "note"]

print_lock = threading.Lock()
csv_lock = threading.Lock()


def say(port, msg):
    with print_lock:
        print(f"{datetime.datetime.now():%H:%M:%S} [{port}] {msg}", flush=True)


def parse_marks(text):
    out = []
    for tok in [t for t in text.split(",") if t.strip()]:
        m, _, s = tok.strip().partition(":")
        out.append(int(m) * 60 + int(s or 0))
    return sorted(out)


def open_port(port):
    s = serial.Serial()
    s.port = port
    s.baudrate = BAUD
    s.timeout = 0.2
    s.dtr = False
    s.rts = False
    s.open()
    return s


def read_line(ser, logf):
    try:
        raw = ser.readline()
    except (serial.SerialException, OSError):
        # port vanished (USB-powered board unplugged for the power cycle): reopen when it returns
        try:
            ser.close()
        except Exception:  # noqa
            pass
        time.sleep(0.5)
        try:
            ser.open()
        except Exception:  # noqa
            pass
        return None
    if not raw:
        return None
    line = ANSI.sub("", raw.decode("utf-8", errors="replace")).rstrip()
    if line:
        logf.write(f"{datetime.datetime.now():%H:%M:%S.%f} {line}\n")
        logf.flush()
    return line


def send(ser, text):
    ser.write((text + "\n").encode())
    ser.flush()


def one_session(port, ser, logf, idx, args, stop_all):
    row = {k: "" for k in FIELDS}
    row.update(port=port, session=idx, markers_sent=0, markers_acked=0, boot_failure="no")
    say(port, f"session {idx}/{args.sessions}: waiting for power-on (unplug, 10 s, plug in)")
    # ---- wait for the boot banner ---------------------------------------
    fw = ""
    while not stop_all.is_set():
        line = read_line(ser, logf)
        if line is None:
            continue
        m = RE_STARTING.search(line)
        if m:
            fw = m.group(1)
        m = RE_SELFTEST.search(line)       # printed during boot, before the 5 s SETTIME window
        if m:
            row["selftest_attempts"] = m.group(1)
        if BANNER_WAIT in line:
            break
        if any(b in line for b in BAD_BOOT):
            # The firmware aborts here and never opens a session. This also happens when the board
            # boots on USB alone with the card unpowered, which is not a session, so by default
            # it is only reported and the script keeps waiting for the real power-on.
            if args.boot_fail_counts:
                row.update(wall_start=datetime.datetime.now().isoformat(timespec="seconds"),
                           firmware_hash=fw, boot_failure="yes", close="BOOT-FAIL", note=line[:120],
                           elapsed_s=0)
                say(port, f"BOOT FAILURE counted as a failed session: {line}")
                return row
            say(port, f"boot without a session ({line[:70]}); waiting for the next power-on "
                      f"(use --boot-fail-counts to count such boots as failed sessions)")
    else:
        return None
    t_banner = time.monotonic()
    row["wall_start"] = datetime.datetime.now().isoformat(timespec="seconds")
    row["firmware_hash"] = fw
    send(ser, f"SETTIME {int(time.time())}")
    say(port, f"banner seen (fw {fw or '?'}), SETTIME sent; recording for {args.minutes} min")
    t0 = t_banner + 5.0                      # the firmware opens the session after its 5 s window
    marks = [t0 + s for s in args.marks_s]
    stop_at = t0 + args.minutes * 60.0
    stop_sent = False
    next_beat = t0 + 60
    close_deadline = None
    while not stop_all.is_set():
        now = time.monotonic()
        if marks and now >= marks[0]:
            marks.pop(0)
            row["markers_sent"] += 1
            send(ser, f"MARK validation_s{idx}_m{row['markers_sent']}")
        if not stop_sent and now >= stop_at:
            send(ser, "STOP")
            stop_sent = True
            close_deadline = now + 180
            say(port, "STOP sent; waiting for CLOSE")
        if now >= next_beat and not stop_sent:
            say(port, f"recording {int((now - t0) // 60)} / {args.minutes} min")
            next_beat += 300
        if close_deadline and now > close_deadline:
            row.update(close="NO-CLOSE-LINE", note="no CLOSE line within 180 s of STOP")
            break
        line = read_line(ser, logf)
        if line is None:
            continue
        if "MARK #" in line:
            row["markers_acked"] += 1
        m = RE_SELFTEST.search(line)
        if m:
            row["selftest_attempts"] = m.group(1)
        m = RE_RUN.search(line)
        if m and not row["run_id"]:
            row["run_id"] = m.group(1)
        if any(b in line for b in BAD_BOOT):
            row.update(boot_failure="yes", close="BOOT-FAIL", note=line[:120])
            say(port, f"BOOT FAILURE: {line}")
            break
        m = RE_CLOSE.search(line)
        if m:
            row["close"] = m.group(1)
            row["close_line"] = line[line.index("CLOSE"):][:300]
            mo = RE_OVR.search(line)
            if mo:
                row["overruns"], row["sd_errors"] = mo.group(1), mo.group(2)
            ma = RE_ADS.search(line)
            if ma:
                row["ads_timeouts"], row["ads_i2c_err"] = ma.group(1), ma.group(2)
            break
    row["elapsed_s"] = int(time.monotonic() - t_banner)
    return row


def worker(port, args, results, stop_all):
    try:
        ser = open_port(port)
    except Exception as e:  # noqa
        say(port, f"cannot open port: {e}")
        stop_all.set()
        return
    safe_port = re.sub(r"\W", "_", port).strip("_")
    logname = os.path.join(args.outdir, f"{args.tag}_{safe_port}_console.log")
    with open(logname, "a", encoding="utf-8") as logf:
        for i in range(1, args.sessions + 1):
            row = one_session(port, ser, logf, i, args, stop_all)
            if row is None:
                break
            with csv_lock:
                new = not os.path.isfile(args.csv)
                with open(args.csv, "a", newline="", encoding="utf-8") as f:
                    w = csv.DictWriter(f, fieldnames=FIELDS)
                    if new:
                        w.writeheader()
                    w.writerow(row)
            results.append(row)
            say(port, f"session {i}: CLOSE {row['close'] or '?'}  run={row['run_id'] or '?'}  "
                      f"self-test attempts={row['selftest_attempts'] or '?'}  "
                      f"markers {row['markers_acked']}/{row['markers_sent']}")
            if row["close"] != "OK":
                say(port, "NOT OK: stop and look at the red LED and the console log before continuing")
    ser.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ports", nargs="+", help="console ports, e.g. COM8 COM6")
    ap.add_argument("--sessions", type=int, default=5)
    ap.add_argument("--minutes", type=float, default=21.0, help="session length before STOP (>= 20 for validation)")
    ap.add_argument("--marks", default="5:00,15:00", help="mm:ss list, from session start")
    ap.add_argument("--boot-fail-counts", action="store_true",
                    help="count an SD boot failure as a failed session (default: only report it)")
    ap.add_argument("--out", default=None, help="tag for output files (default validation_<date>)")
    ap.add_argument("--outdir", default=".")
    ap.add_argument("--dry-seconds", type=float, default=None, help="test mode: session length in seconds")
    args = ap.parse_args()
    if args.dry_seconds is not None:
        args.minutes = args.dry_seconds / 60.0
    args.marks_s = [s for s in parse_marks(args.marks) if s < args.minutes * 60 - 1]
    args.tag = args.out or f"validation_{datetime.date.today().isoformat()}"
    os.makedirs(args.outdir, exist_ok=True)
    args.csv = os.path.join(args.outdir, f"{args.tag}_sessions.csv")

    stop_all = threading.Event()
    results = []
    threads = [threading.Thread(target=worker, args=(p, args, results, stop_all), daemon=True) for p in args.ports]
    for t in threads:
        t.start()
    try:
        while any(t.is_alive() for t in threads):
            time.sleep(0.5)
    except KeyboardInterrupt:
        stop_all.set()
        print("\ninterrupted: the boards keep recording; send STOP or long-press to close them")
        for t in threads:
            t.join(timeout=2)

    ok = [r for r in results if r["close"] == "OK"]
    print(f"\n{len(ok)} of {len(results)} board-sessions closed OK (expected {args.sessions * len(args.ports)})")
    print(f"rows: {args.csv}")
    runs = sorted({r["run_id"] for r in results if r["run_id"]})
    runs_arg = f" --runs {','.join(runs)}" if runs else ""
    print("\nNext: pull both cards, then run (replace the drive letters):")
    print(f"  python tools\\validate_sessions.py D:\\ E:\\ --expect {args.sessions * len(args.ports)}{runs_arg} "
          f"--json {args.tag}.json")
    sys.exit(0 if len(ok) == args.sessions * len(args.ports) else 1)


if __name__ == "__main__":
    main()

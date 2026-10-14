#!/usr/bin/env python3
"""
campaign_session.py -- run ONE review-1 campaign session, on one or both boards
at once, from one command.

Both boards are normally recorded together (PI, 8 Oct: "recorded with both boards
together"), so a session takes two session ids and two ports, one per board. The
timeline comes from the session id, looked up in campaign_schedule_planned.csv,
so nothing is typed per session except the ids and the ports.

  python tools\\campaign_session.py A1-U1-S-low A1-U2-S-low ^
      --board-port COM8 --board-port COM6 --shaker-port COM7
  python tools\\campaign_session.py A2-U1-S-30min A2-U2-S-30min --board-port COM8 --board-port COM6
  python tools\\campaign_session.py B1P-U1-S-R-1 B1P-U2-S-R-1 ^
      --board-port COM8 --board-port COM6 --values 160,820,3900
  python tools\\campaign_session.py B1-U1-T-vib --board-port COM8          (one board alone)

The two ids must be the same condition, arm and parameter on different boards;
anything else is refused, because one timeline drives both.

What it does:
  1. Opens every board port FIRST (and the shaker for A1), before anything is powered.
  2. You power-cycle BOTH boards (unplug, wait 10 s, plug in; cards in).
  3. As each board's boot banner appears it gets SETTIME inside the firmware's 5 s
     window. t = 0 of the session is when the LAST board's window closes, so every
     mark lands inside every board's recording.
  4. Events fire from the schedule. 'auto' events (A1 shaker on/off, the end) happen
     by themselves; 'confirm' events (a pull, a lift-out, a resistor in) beep at the
     planned time and are marked on BOTH boards WHEN YOU PRESS ENTER, i.e. at the
     moment you did it. A bare Enter with nothing pending is ignored; type
     "m <label>" + Enter to add a manual mark (it is recorded as a deviation).
  5. At the end it marks `end`, sends STOP to both and waits for both CLOSE lines.
  6. Pull the cards and run campaign_finish.py once per board.

Timelines (seconds from the end of the 5 s boot window):
  A1      settle 6 min -> stress_on (shaker to the level's RPM; control = 0 RPM) ->
          60 min -> stress_off -> 5 min recovery -> end.   Total 71 min.
          Level from the id: low 80, medium 140, high 200 RPM; none1/2/3 = 0 RPM.
          The 6 min settle is the PI-approved figure: 5 min gives 199 windows,
          under the ~200 the Mahalanobis reference needs; 6 min gives 239.
  A2      settle_start at power-on on the bed; lift_out at the exposure (15/30/45
          min, from the id); end at 60 min.
  B1      5 min settle; pull_k / reseat_k (H arm: touch_k / release_k) at 5,10,15,20,
          25 min; end at 30 min. Seven-target sessions (arm T) use the same pattern.
  B1P R   5 min settle; three holds: insert at 5, 15, 25 min, remove 5 min later;
          5 min recovery; end at 35 min. --values names the resistors in order.
  B1I     5 min settle; window_start at 5:00, window_end at 10:00 with a beep every
          30 s for the break; end at 15:00.

Outputs in --outdir, per board: <id>_session_meta.json and <id>_console.log; for A1
one <first id>_shaker_accel.csv for the platform.

Dry run (no real waiting): add --time-scale 60. Never use that data.
"""
import argparse
import csv
import json
import os
import platform
import queue
import re
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone

try:
    import serial
except ImportError:
    sys.exit("pyserial is missing: python -m pip install pyserial")

RUNNER_VERSION = "campaign-2026-10-09"
CAMPAIGN_DIR = (r"C:\Users\Metal\OneDrive - UW\Team Sentinel - Documents\General"
                r"\SHIBench Paper Collaboration\review1\campaign_logs")
_HERE = os.path.dirname(os.path.abspath(__file__))
# Prefer a copy of shaker_control.py sitting next to this script: an overnight run must not
# depend on a OneDrive folder that may be cloud-only or mid-sync. Fall back to the team drive.
SHAKER_DIR = _HERE if os.path.isfile(os.path.join(_HERE, "shaker_control.py")) else (
    r"C:\Users\Metal\OneDrive - UW\Team Sentinel - Documents\General"
    r"\Shield drive\Shaker Table Test")

BAUD = 115200
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
BANNER_WAIT = "Waiting 5 s for optional"
BOOT_WINDOW_S = 5.0
RE_STARTING = re.compile(r"Project SHIELD starting.*build\s+(\S+?)\)?\s*=")
RE_SELFTEST = re.compile(r"self-test passed on attempt (\d+)")
RE_RUN = re.compile(r"(RUN_\d{3})")
RE_CLOSE = re.compile(r"CLOSE (OK|FAIL):")
RE_ADS = re.compile(r"ads_timeouts=(\d+)\s+ads_i2c_err=(\d+)\s+markers=(\d+)")
RE_OVR = re.compile(r"overruns=(\d+)\s+sd_errors=(\d+)")
RE_MARK = re.compile(r"MARK #(\d+) '(.*)' at t_ms=(\d+)")
BAD_BOOT = ("SD self-test FAILED", "Run session creation FAILED", "mkdir failed")

LEVEL_RPM = {"low": 80, "medium": 140, "high": 200}
SETTLE_A1_S = 6 * 60
STRESS_S = 60 * 60
RECOVERY_S = 5 * 60
SETTLE_S = 5 * 60
CONFIRM_TIMEOUT_S = 90

# Audible cues. The terminal bell (\a) is silent in a lot of PowerShell setups, so on
# Windows this plays a real tone through winsound and only falls back to the bell
# elsewhere. Each confirm event gets a 5 s countdown so two operators can act together
# on "NOW"; an event that follows within the countdown (a re-seat 2 s after its pull)
# gets its own distinct tone instead of a countdown that would talk over the first.
COUNTDOWN_S = 5
TONE_TICK = (700, 120)      # one per second, 5 4 3 2 1
TONE_ACT = (1200, 450)      # the un-mate / pull moment
TONE_BACK = (500, 350)      # the re-seat / release moment
BACK_LABELS = ("reseat", "release", "remove", "hold_out", "window_end", "lift_out")


def _play(freq, ms):
    try:
        import winsound
        winsound.Beep(freq, ms)
    except Exception:  # noqa: BLE001  not Windows, or no sound device
        sys.stdout.write("\a")
        sys.stdout.flush()


def tone(spec, enabled=True):
    """Non-blocking: winsound.Beep blocks for its duration and the console must keep
    being drained while it sounds."""
    if not enabled:
        return
    threading.Thread(target=_play, args=spec, daemon=True).start()


# ----------------------------------------------------------------------------- timeline
def build_timeline(row, values=None):
    """Return (events, end_s). event = dict(label, due, mode, rpm?)."""
    cond, arm, param = row["condition"], row["arm"], row["parameter"]
    ev = []
    if cond == "A1":
        level = "none" if param.startswith("none") else param
        rpm = 0 if level == "none" else LEVEL_RPM[level]
        ev = [dict(label="stress_on", due=SETTLE_A1_S, mode="auto", rpm=rpm),
              dict(label="stress_off", due=SETTLE_A1_S + STRESS_S, mode="auto", rpm=0)]
        end = SETTLE_A1_S + STRESS_S + RECOVERY_S
    elif cond == "A2":
        minutes = int(re.match(r"(\d+)", param).group(1))
        ev = [dict(label="lift_out", due=minutes * 60, mode="confirm")]
        end = 60 * 60
    elif cond == "B1":
        pull, back = ("touch", "release") if arm == "H" else ("pull", "reseat")
        for k in range(1, 6):
            ev.append(dict(label=f"{pull}_{k}", due=SETTLE_S + (k - 1) * 300, mode="confirm"))
            ev.append(dict(label=f"{back}_{k}", due=SETTLE_S + (k - 1) * 300 + 2, mode="confirm"))
        end = 30 * 60
    elif cond == "B1P":
        names = [v.strip() for v in values.split(",")] if values else ["1", "2", "3"]
        if len(names) != 3:
            raise SystemExit("--values needs exactly three entries (the three resistors, in order)")
        for i, nm in enumerate(names):
            tag = f"R{nm}" if values else nm
            a, b = ("hold_in", "hold_out") if arm == "H" else ("insert", "remove")
            ev.append(dict(label=f"{a}_{tag}", due=SETTLE_S + i * 600, mode="confirm"))
            ev.append(dict(label=f"{b}_{tag}", due=SETTLE_S + i * 600 + 300, mode="confirm"))
        end = 35 * 60
    elif cond == "B1I":
        ev.append(dict(label="window_start", due=SETTLE_S, mode="confirm"))
        for k in range(10):
            ev.append(dict(label=f"break_cue_{k + 1}", due=SETTLE_S + 30 * k + 15, mode="cue"))
        ev.append(dict(label="window_end", due=SETTLE_S + 300, mode="confirm"))
        end = 15 * 60
    else:
        raise SystemExit(f"no timeline defined for condition {cond!r}")
    ev.sort(key=lambda e: e["due"])
    return ev, end


# ----------------------------------------------------------------------------- helpers
def utc_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def beep(msg):
    print(f"\a\n{'=' * 64}\n{msg}\n{'=' * 64}\n", flush=True)


def hms(seconds):
    seconds = max(0, int(round(seconds)))
    return f"{seconds // 3600}:{seconds % 3600 // 60:02d}:{seconds % 60:02d}"


def load_rows(schedule, session_ids):
    with open(schedule, newline="", encoding="utf-8") as f:
        table = {r["session_id"]: r for r in csv.DictReader(f)}
    out = []
    for sid in session_ids:
        if sid not in table:
            raise SystemExit(f"{sid!r} is not in {schedule}")
        out.append(table[sid])
    first = out[0]
    for r in out[1:]:
        for k in ("condition", "arm", "parameter"):
            if r[k] != first[k]:
                raise SystemExit(
                    f"the sessions must share condition, arm and parameter: "
                    f"{first['session_id']} has {k}={first[k]!r}, {r['session_id']} has {k}={r[k]!r}")
    boards = [r["board"] for r in out]
    if len(set(boards)) != len(boards):
        raise SystemExit(f"two sessions on the same board: {boards}")
    return out


class BoardSession:
    """One board: its console port, its session id, its meta file. DTR/RTS forced low
    before opening, because a toggle can reset the S3."""

    def __init__(self, sid, port, row, outdir, timeout):
        self.sid, self.port, self.row = sid, port, row
        self.meta_path = os.path.join(outdir, f"{sid}_session_meta.json")
        self.logf = open(os.path.join(outdir, f"{sid}_console.log"), "a", encoding="utf-8")
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, BAUD, timeout
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        self.t_banner = None
        self.meta = {}

    def read_line(self):
        try:
            raw = self.ser.readline()
        except (serial.SerialException, OSError):
            try:
                self.ser.close()
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.2)
            try:
                self.ser.open()
            except Exception:  # noqa: BLE001
                pass
            return None
        if not raw:
            return None
        line = ANSI.sub("", raw.decode("utf-8", errors="replace")).rstrip()
        if line:
            self.logf.write(f"{datetime.now():%H:%M:%S.%f} {line}\n")
            self.logf.flush()
        return line or None

    def send(self, text):
        self.logf.write(f"{datetime.now():%H:%M:%S.%f} >>> {text}\n")
        self.logf.flush()
        self.ser.write((text + "\n").encode("ascii", "replace"))
        self.ser.flush()

    def handle(self, line):
        m = RE_RUN.search(line)
        if m and not self.meta.get("run_id"):
            self.meta["run_id"] = m.group(1)
        m = RE_STARTING.search(line)
        if m:
            self.meta["firmware_hash_console"] = m.group(1)
        m = RE_SELFTEST.search(line)
        if m:
            self.meta["selftest_attempts"] = int(m.group(1))
        m = RE_CLOSE.search(line)
        if m:
            self.meta["close"] = m.group(1)
            self.meta["close_line"] = line[line.index("CLOSE"):][:300]
            mo = RE_OVR.search(line)
            if mo:
                self.meta["overruns"], self.meta["sd_errors"] = int(mo.group(1)), int(mo.group(2))
            ma = RE_ADS.search(line)
            if ma:
                self.meta["ads_timeouts"], self.meta["ads_i2c_err"] = int(ma.group(1)), int(ma.group(2))

    def pump(self):
        line = self.read_line()
        if line:
            self.handle(line)
        return line

    def reset(self):
        """Pulse RTS to pull EN low: on the S3's native USB-Serial/JTAG that resets the chip.
        DTR is held low throughout so IO0 stays high and it boots the application, not the
        serial loader. The USB device may re-enumerate, so the port can drop and come back;
        read_line() reopens it. This is a WARM restart, not the cold start the paper sheet
        asks for -- campaign_finish.py records it as a deviation."""
        try:
            self.ser.dtr = False
            self.ser.rts = True
            time.sleep(0.25)
            self.ser.rts = False
            self.ser.dtr = False
            return True
        except Exception as e:  # noqa: BLE001
            print(f"  {self.port}: could not pulse the reset line ({e})")
            return False

    def reopen(self, timeout=25.0):
        """Wait for the port to come back (the native USB device re-enumerates on reset)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                if not self.ser.is_open:
                    self.ser.dtr = False
                    self.ser.rts = False
                    self.ser.open()
                return True
            except Exception:  # noqa: BLE001
                time.sleep(0.2)
        return False

    def esptool_reset(self, chip="esp32s3"):
        """Reset with esptool, the same mechanism `idf.py flash` uses on these boards
        (connect, then --after hard_reset boots the application). Slower than an RTS
        pulse, about 5 s, but it is the proven path, so it is the fallback when the
        pulse produces no banner. esptool needs the port to itself, so ours closes
        first and is reopened immediately: the board boots as esptool exits and then
        holds its 5 s SETTIME window, which is the margin this relies on."""
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass
        cmd = [sys.executable, "-m", "esptool", "--chip", chip, "-p", self.port,
               "--before", "default_reset", "--after", "hard_reset", "read_mac"]
        ok = False
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=90)
            ok = r.returncode == 0
            if not ok:
                tail = (r.stderr or r.stdout or "").strip().splitlines()[-1:] or [""]
                print(f"  {self.port}: esptool reset failed ({tail[0][:110]})")
        except FileNotFoundError:
            print(f"  {self.port}: esptool is not installed in this Python "
                  f"(python -m pip install esptool)")
        except subprocess.TimeoutExpired:
            print(f"  {self.port}: esptool reset timed out")
        if not self.reopen():
            print(f"  {self.port}: the port did not come back after the esptool reset")
            return False
        return ok

    def write_meta(self):
        tmp = self.meta_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.meta, f, indent=2)
        os.replace(tmp, self.meta_path)

    def close(self):
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.logf.close()
        except Exception:  # noqa: BLE001
            pass


# ----------------------------------------------------------------------------- session
def run(args):
    rows = load_rows(args.schedule, args.session_ids)
    if len(args.board_port) != len(args.session_ids):
        raise SystemExit(f"{len(args.session_ids)} session id(s) but {len(args.board_port)} --board-port; "
                         f"give one port per session id, in the same order")
    events, end_s = build_timeline(rows[0], args.values)
    cond = rows[0]["condition"]
    is_a1 = cond == "A1"
    if is_a1 and not args.shaker_port:
        raise SystemExit("A1 needs --shaker-port")
    scale = args.time_scale
    os.makedirs(args.outdir, exist_ok=True)
    accel_path = os.path.join(args.outdir, f"{args.session_ids[0]}_shaker_accel.csv")

    clash = [os.path.join(args.outdir, f"{s}_session_meta.json") for s in args.session_ids]
    clash = [p for p in clash if os.path.exists(p)] + ([accel_path] if is_a1 and os.path.exists(accel_path) else [])
    if clash and not args.overwrite:
        raise SystemExit("Refusing to overwrite:\n  " + "\n  ".join(clash) +
                         "\nUse --overwrite for a deliberate re-run.")

    target_rpm = next((e["rpm"] for e in events if e["label"] == "stress_on"), None)
    kind = (f"{target_rpm} RPM" if target_rpm else "control, shaker held at 0 RPM") if is_a1 else cond
    print(f"Session: {cond} arm {rows[0]['arm']} parameter {rows[0]['parameter']} ({kind})")
    for sid, port, row in zip(args.session_ids, args.board_port, rows):
        print(f"   {row['board']}  {sid}  on {port}")
    print(f"   planned length {hms(end_s)} after the boot window")
    if scale != 1.0:
        print(f"*** TEST MODE: time compressed {scale}x. Not a real session. ***")
    for e in events:
        if e["mode"] != "cue":
            print(f"   {hms(e['due']):>9}  {e['label']:<16} ({e['mode']})")
    print(f"   {hms(end_s):>9}  {'end':<16} (auto, then STOP)")

    per_port_timeout = 0.1 if len(args.board_port) == 1 else 0.05
    boards = []
    table = None
    logging_started = False
    stdin_q = queue.Queue()
    threading.Thread(target=lambda: [stdin_q.put(l) for l in iter(sys.stdin.readline, "")], daemon=True).start()

    try:
        for sid, port, row in zip(args.session_ids, args.board_port, rows):
            b = BoardSession(sid, port, row, args.outdir, per_port_timeout)
            b.meta = {
                "runner": os.path.basename(__file__), "runner_version": RUNNER_VERSION,
                "session_tag": sid, "session_id": sid, "unit": row["board"], "condition": cond,
                "arm": row["arm"], "parameter": row["parameter"],
                "level": row["parameter"] if is_a1 else None,
                "target_rpm": target_rpm, "control_handled_healthy": row["arm"] == "H",
                "paired_with": [s for s in args.session_ids if s != sid],
                "values": args.values, "label": args.label, "run_id": None,
                "shaker_port": args.shaker_port, "board_port": port,
                "time_scale": scale, "dry_run": scale != 1.0, "host": platform.node(),
                "python": sys.version.split()[0],
                "planned_s": {"end": end_s, "settle": SETTLE_A1_S if is_a1 else SETTLE_S,
                              "stress": STRESS_S if is_a1 else None,
                              "recovery": RECOVERY_S if is_a1 else None},
                "planned_events": [dict(e) for e in events],
                "events": [], "board_marks": [], "completed": False, "abort_reason": None,
                "close": None, "close_line": None, "firmware_hash_console": None,
                "selftest_attempts": None, "overruns": None, "sd_errors": None,
                "ads_timeouts": None, "ads_i2c_err": None, "settime_sent": False,
                "manual_marks": 0, "unconfirmed_events": [],
            }
            boards.append(b)

        if is_a1:
            sys.path.insert(0, args.shaker_dir)
            from shaker_control import ShakerTable  # noqa: E402
            table = ShakerTable(args.shaker_port, args.shaker_baud)
            table.set_rpm(0)

        if args.reset_method == "manual":
            for b in boards:
                b.meta["start_mode"] = "cold start (operator power-cycle)"
            print(f"\n{len(boards)} board console(s) open"
                  + (f", shaker {args.shaker_port} at 0 RPM" if is_a1 else "") +
                  f".\nNOW power-cycle {'both boards' if len(boards) > 1 else 'the board'}: "
                  f"unplug, wait 10 s, plug in (cards in). Waiting for the banner(s)...")
        else:
            print(f"\n{len(boards)} board console(s) open"
                  + (f", shaker {args.shaker_port} at 0 RPM" if is_a1 else "") +
                  f".\nRestarting the board(s) from the host "
                  f"({'esptool' if args.reset_method == 'esptool' else 'RTS pulse, esptool if that is quiet'}). "
                  f"This is a WARM restart and is recorded as a deviation.")

        # ---- wait for every board's banner; SETTIME each as it appears -------------
        # Unattended running depends entirely on this block, so a restart that does not
        # produce a banner is retried with a stronger method instead of waiting forever:
        #   RTS pulse (fast)  ->  esptool hard reset (proven, ~5 s)  ->  esptool again.
        # Only if every method has failed repeatedly does it give up and say so.
        t_wait = time.monotonic()
        next_note = t_wait + 30
        waiting = list(boards)
        plan = {b.sid: {"tries": 0, "due": time.monotonic()} for b in boards}
        PATIENCE = {"rts": 25.0, "esptool": 30.0}
        while waiting:
            for b in list(waiting):
                line = b.pump()
                if line and BANNER_WAIT in line:
                    b.t_banner = time.monotonic()
                    b.meta["banner_local"] = datetime.now().isoformat(timespec="seconds")
                    b.meta["banner_utc"] = utc_iso()
                    b.meta["start_utc"] = b.meta["banner_utc"]
                    b.send(f"SETTIME {int(time.time())}")
                    b.meta["settime_sent"] = True
                    waiting.remove(b)
                    print(f"  {b.row['board']} ({b.port}): banner seen "
                          f"(firmware {b.meta.get('firmware_hash_console') or '?'}), SETTIME sent"
                          + (f"; still waiting for {', '.join(x.port for x in waiting)}" if waiting else ""))
                elif line and any(bad in line for bad in BAD_BOOT):
                    print(f"  {b.port}: boot without a session ({line[:70]})")

            if args.reset_method != "manual":
                for b in list(waiting):
                    st = plan[b.sid]
                    if time.monotonic() < st["due"]:
                        continue
                    n = st["tries"]
                    st["tries"] += 1
                    if args.reset_method == "esptool" or n >= 1:
                        method = "esptool"
                        if n >= 1:
                            print(f"  {b.port}: no banner after the previous attempt; "
                                  f"resetting with esptool (attempt {n + 1})")
                        b.esptool_reset(args.chip)
                    else:
                        method = "rts"
                        b.reset()
                    b.meta["start_mode"] = f"warm restart ({method})"
                    b.meta["reset_attempts"] = st["tries"]
                    st["due"] = time.monotonic() + PATIENCE[method]
                    if st["tries"] >= args.reset_tries:
                        raise SystemExit(
                            f"{b.port}: no boot banner after {st['tries']} reset attempts "
                            f"(last method: {method}). Nothing usable was recorded for this session. "
                            f"Check the board's power and the cable.")

            if time.monotonic() > next_note:
                print(f"  still waiting for {', '.join(x.port for x in waiting)} "
                      f"({int(time.monotonic() - t_wait)} s)...")
                next_note += 30
            if time.monotonic() - t_wait > args.boot_timeout:
                raise SystemExit("not every board reached the banner within --boot-timeout; "
                                 "nothing usable recorded")

        # t = 0 is the LAST board's window closing, so every mark lands in every recording
        t0 = max(b.t_banner for b in boards) + BOOT_WINDOW_S
        spread = max(b.t_banner for b in boards) - min(b.t_banner for b in boards)
        for b in boards:
            b.meta["power_on_spread_s"] = round(spread, 2)
        if spread > 30:
            print(f"  NOTE: {spread:.0f} s between the power-ons; the marks are common to both "
                  f"but the recordings start that far apart.")
        if table is not None:
            table.start_logging(accel_path)
            logging_started = True
        for b in boards:
            b.write_meta()

        state = {"end_fired": False, "stop_sent": False, "t_stop": None}

        def pump_all():
            for b in boards:
                b.pump()

        def send_mark(name):
            """MARK <name> to every board; wait up to 1.5 s for each acknowledgement."""
            lab = re.sub(r"\s+", "_", name.strip())
            recs = {}
            for b in boards:
                b.send(f"MARK {lab}")
                recs[b.sid] = {"event": name, "t_utc": utc_iso(), "label": lab, "sent": True,
                               "reply": None, "board_t_ms": None, "mark_no": None}
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline and not all(recs[b.sid]["reply"] for b in boards):
                for b in boards:
                    line = b.pump()
                    if not line:
                        continue
                    m = RE_MARK.search(line)
                    if m and m.group(2) == lab and recs[b.sid]["reply"] is None:
                        recs[b.sid].update(reply=line, mark_no=int(m.group(1)),
                                           board_t_ms=int(m.group(3)))
            for b in boards:
                if recs[b.sid]["reply"] is None:
                    print(f"  WARNING: {b.row['board']} did not acknowledge MARK {lab} within 1.5 s "
                          f"(check it is recording; the paper log is the backup)")
                b.meta["board_marks"].append(recs[b.sid])

        def transition(name, rpm=None, msg=None, extra=None):
            """Event on the host clock: shaker command first (the commanded time), then the marks."""
            stamp = {"event": name, "t_utc": utc_iso(),
                     "elapsed_real_s": round(time.monotonic() - t0, 3)}
            if extra:
                stamp.update(extra)
            refs = []
            for b in boards:
                rec = dict(stamp)
                b.meta["events"].append(rec)
                refs.append(rec)
            if rpm is not None and table is not None:
                table.set_rpm(rpm)
            if msg:
                beep(msg)
            send_mark(name)
            for b in boards:
                b.write_meta()
            return refs

        while time.monotonic() < t0 + 2.0:      # the tier files must exist before the first mark
            pump_all()
        transition("settle_start",
                   msg=f"{'/'.join(args.session_ids)}: recording. settle_start marked on "
                       f"{len(boards)} board(s).")

        while not stdin_q.empty():              # drop anything typed before the session opened
            stdin_q.get_nowait()
        pending = list(events)
        # Countdown cues, built here rather than in build_timeline so they stay out of
        # planned_events and the session meta. An event whose predecessor is less than a
        # countdown away is skipped: the re-seat 2 s after a pull must not start a second
        # countdown over the top of the first.
        cues = []
        prev_due = None
        for e in events:
            if e["mode"] == "confirm" and (prev_due is None or e["due"] - prev_due > COUNTDOWN_S):
                for k in range(COUNTDOWN_S, 0, -1):
                    cues.append((e["due"] - k, k, e["label"]))
            prev_due = e["due"]
        cues.sort()
        awaiting = []
        tick_end = None
        last_hb = time.monotonic()

        while True:
            now = time.monotonic()
            while cues and now >= t0 + cues[0][0] / scale:
                _, k, label = cues.pop(0)
                tone(TONE_TICK, args.sound)
                print(f"   {label} in {k}...", flush=True)
            while pending and now >= t0 + pending[0]["due"] / scale:
                e = pending.pop(0)
                if e["mode"] == "auto":
                    rpm = e.get("rpm")
                    transition(e["label"], rpm=rpm,
                               msg=f"{hms(e['due'])}  {e['label']}"
                                   + (f"  (shaker to {rpm} RPM)" if rpm is not None else ""))
                elif e["mode"] == "cue":
                    tone(TONE_ACT, args.sound)
                    beep(f"{hms(e['due'])}  BREAK NOW ({e['label']})")
                else:
                    back = any(e["label"].startswith(x) for x in BACK_LABELS)
                    tone(TONE_BACK if back else TONE_ACT, args.sound)
                    if args.mark_on == "tone":
                        # The operators act ON the tone, so the tone is the onset and the mark
                        # goes out now. The key press that follows is only a confirmation that
                        # it happened, and its lag is recorded as confirm_lag_s, not as the
                        # event time. Pressing first and acting afterwards would put every
                        # onset seconds late, which C-90 would read as detector latency.
                        refs = transition(e["label"],
                                          msg=f"{hms(e['due'])}  {'RE-SEAT' if back else 'NOW'} on "
                                              f"{'both boards' if len(boards) > 1 else 'the board'}: "
                                              f"{e['label']}  -- MARKED; press ENTER to confirm",
                                          extra={"confirmed": None, "marked_at": "tone"})
                        awaiting.append({"ev": e, "t_beep": now, "refs": refs})
                    else:
                        beep(f"{hms(e['due'])}  {'RE-SEAT' if back else 'NOW'} on "
                             f"{'both boards' if len(boards) > 1 else 'the board'}: {e['label']}"
                             f"  -- press ENTER as you do it")
                        awaiting.append({"ev": e, "t_beep": now, "refs": None})
            if not state["end_fired"] and not pending and now >= t0 + end_s / scale:
                for a in awaiting:
                    for b in boards:
                        b.meta["unconfirmed_events"].append(a["ev"]["label"])
                    if a["refs"] is not None:
                        for rec in a["refs"]:
                            rec["confirmed"] = False
                    else:
                        transition(a["ev"]["label"], extra={"confirmed": False})
                awaiting.clear()
                transition("end", rpm=0 if table is not None else None,
                           msg=f"{hms(end_s)}  end -- protocol complete")
                for b in boards:
                    b.meta["completed"] = True
                state["end_fired"] = True
                tick_end = time.monotonic()
            if state["end_fired"] and not state["stop_sent"] and time.monotonic() >= tick_end + 3.0:
                for b in boards:
                    # QUIT, not STOP: since the session-loop firmware (3e829ae) the board opens
                    # the NEXT RUN folder a couple of seconds after a STOP and keeps recording.
                    # This tool runs exactly one session per invocation -- A2 and B1 both need the
                    # board unpowered or handled between sessions -- so it must end the loop, or
                    # every session would leave an orphan folder recording silence behind it.
                    b.send("QUIT")
                state["stop_sent"] = True
                state["t_stop"] = time.monotonic()
                print("QUIT sent to every board; waiting for CLOSE ...")
            if state["stop_sent"] and time.monotonic() > state["t_stop"] + 180:
                for b in boards:
                    b.meta["close"] = b.meta.get("close") or "NO-CLOSE-LINE"
                break
            if state["stop_sent"] and all(b.meta.get("close") for b in boards):
                break

            try:
                while True:
                    typed = stdin_q.get_nowait().strip()
                    if awaiting and not typed:
                        a = awaiting.pop(0)
                        lag = round((time.monotonic() - a["t_beep"]) * scale, 2)
                        if a["refs"] is not None:
                            for rec in a["refs"]:
                                rec["confirmed"] = True
                                rec["confirm_lag_s"] = lag
                            for b in boards:
                                b.write_meta()
                            print(f"  {a['ev']['label']} confirmed ({lag:.1f} s after the tone)")
                        else:
                            transition(a["ev"]["label"],
                                       extra={"confirmed": True, "late_s": lag})
                            print(f"  marked {a['ev']['label']} on {len(boards)} board(s)")
                    elif typed[:2].lower() == "m " and typed[2:].strip() and not state["end_fired"]:
                        for b in boards:
                            b.meta["manual_marks"] += 1
                        transition(f"manual_{typed[2:].strip()}", extra={"confirmed": True, "manual": True})
                        print(f"  manual mark sent: {typed[2:].strip()}")
                    elif not typed:
                        print("  (nothing is waiting for a key press; to add a mark type  m <label>  then Enter)")
            except queue.Empty:
                pass

            for a in list(awaiting):
                if (time.monotonic() - a["t_beep"]) > CONFIRM_TIMEOUT_S / scale:
                    awaiting.remove(a)
                    for b in boards:
                        b.meta["unconfirmed_events"].append(a["ev"]["label"])
                    if a["refs"] is not None:
                        for rec in a["refs"]:
                            rec["confirmed"] = False
                        for b in boards:
                            b.write_meta()
                    else:
                        transition(a["ev"]["label"], extra={"confirmed": False})
                    print(f"  {a['ev']['label']} not confirmed within {CONFIRM_TIMEOUT_S} s: "
                          f"marked anyway, flagged unconfirmed")

            pump_all()
            if now - last_hb > 300 and not state["end_fired"]:
                last_hb = now
                print(f"  [{hms((now - t0) * scale)} / {hms(end_s)}] recording")

        ok = True
        for b in boards:
            b.meta["end_utc"] = utc_iso()
            b.meta["duration_s"] = round((time.monotonic() - b.t_banner) * scale, 1)
            n = len(b.meta["board_marks"])
            acked = sum(1 for m in b.meta["board_marks"] if m["reply"])
            b.write_meta()
            print(f"{b.row['board']} {b.sid}: CLOSE {b.meta['close']}  run={b.meta['run_id'] or '?'}  "
                  f"overruns={b.meta['overruns']}  ads_timeouts={b.meta['ads_timeouts']}  "
                  f"marks acked {acked}/{n}")
            if b.meta["close"] != "OK" or acked != n:
                ok = False
        if not ok:
            print("NOT CLEAN: look at the red LED and the console logs before the next session.")
        if table is not None:
            table.stop_logging()
            logging_started = False
            table.set_rpm(0)
            table.close()
            table = None
        print("\nNext: pull the cards, then once per board:")
        for b in boards:
            print(f"  python tools\\campaign_finish.py {b.sid} --card <that board's card> --outdir {args.outdir}")
        return 0 if ok else 2
    except BaseException as e:  # noqa: BLE001  KeyboardInterrupt included: never leave the shaker running
        for b in boards:
            b.meta["abort_reason"] = f"{type(e).__name__}: {e}"
        print(f"\nSession aborted ({type(e).__name__}: {e}).")
        for b in boards:
            if b.meta.get("close"):
                continue
            try:
                b.send("QUIT")
            except Exception:  # noqa: BLE001
                pass
        print("QUIT sent so the card files close cleanly; these are NOT usable as complete sessions.")
        t_end = time.monotonic() + 20
        while time.monotonic() < t_end and not all(b.meta.get("close") for b in boards):
            for b in boards:
                b.pump()
        raise
    finally:
        if table is not None:
            for step in ("set_rpm", "stop_logging", "close"):
                try:
                    if step == "set_rpm":
                        table.set_rpm(0)
                    elif step == "stop_logging" and logging_started:
                        table.stop_logging()
                    elif step == "close":
                        table.close()
                except Exception as e2:  # noqa: BLE001
                    print(f"  cleanup {step} failed: {e2}")
        for b in boards:
            try:
                b.write_meta()
            except Exception:  # noqa: BLE001
                pass
            b.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_ids", nargs="+",
                    help="one id per board, e.g. A1-U1-S-low A1-U2-S-low (same condition/arm/parameter)")
    ap.add_argument("--board-port", action="append", required=True,
                    help="repeat once per session id, in the same order: --board-port COM8 --board-port COM6")
    ap.add_argument("--shaker-port", help="A1 only: shaker controller port")
    ap.add_argument("--shaker-baud", type=int, default=115200)
    ap.add_argument("--shaker-dir", default=SHAKER_DIR, help="folder holding shaker_control.py")
    ap.add_argument("--schedule", default=os.path.join(CAMPAIGN_DIR, "campaign_schedule_planned.csv"))
    ap.add_argument("--outdir", default="campaign_runs")
    ap.add_argument("--values", help="B1P resistance: the three resistor values in order, e.g. 160,820,3900")
    ap.add_argument("--label", default=None, help="free text kept in the meta files")
    ap.add_argument("--auto-reset", action="store_true",
                    help="deprecated spelling of --reset-method auto")
    ap.add_argument("--reset-method", choices=("auto", "rts", "esptool", "manual"), default=None,
                    help="how to start each session. auto (the default with --auto-reset): RTS pulse, "
                         "falling back to an esptool hard reset if no banner appears. esptool: always use "
                         "esptool, the same reset idf.py flash performs. manual: wait for an operator "
                         "power-cycle (a true cold start). auto/rts/esptool are WARM restarts and are "
                         "recorded as deviations.")
    ap.add_argument("--reset-tries", type=int, default=4,
                    help="give up on a board after this many reset attempts (default 4)")
    ap.add_argument("--chip", default="esp32s3", help="chip name passed to esptool")
    ap.add_argument("--boot-timeout", type=float, default=900, help="seconds to wait for the power-on banners")
    ap.add_argument("--time-scale", type=float, default=1.0, help="dry run only: divide all waits")
    ap.add_argument("--mark-on", choices=("tone", "keypress"), default="tone",
                    help="tone (default): the mark is written when the tone sounds, because that "
                         "is when the operators act; the key press only confirms it happened and "
                         "its lag is recorded as confirm_lag_s. keypress: the mark is written when "
                         "ENTER is pressed -- only correct if you act at the instant of the press.")
    ap.add_argument("--no-sound", dest="sound", action="store_false",
                    help="suppress the tones (the on-screen cues stay)")
    ap.add_argument("--overwrite", action="store_true")
    args = ap.parse_args()
    if args.reset_method is None:
        args.reset_method = "auto" if args.auto_reset else "manual"
    try:
        sys.exit(run(args))
    except KeyboardInterrupt:
        sys.exit(130)


if __name__ == "__main__":
    main()

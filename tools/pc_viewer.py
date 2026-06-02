#!/usr/bin/env python3
"""
PC-side live viewer for Project SHIELD.

Layout
------
A QTabWidget with one tab per sensor group. Each tab contains 1..3 panels.
Every panel shows:
    - clean samples (gray dotted)
    - noisy samples (semi-transparent orange)
    - denoised samples (solid, axis-colored)
    - denoised rolling mean over the visible window (black, thin)
    - +/- 1 sigma band around the denoised mean (semi-transparent fill)
    - title: "<channel>   Hz=...   Bias=...   Std=...   RMS=..."

Two view modes:
    - "live"  : last LIVE_WINDOW_SEC seconds at full rate
    - "full"  : entire run shown as a min/max envelope per HISTORY_BIN_SEC,
                with mean line and +/- sigma band

Transports
----------
    Primary : USB Serial (ESP32-S3 USB Serial/JTAG, virtual COM port)
    Backup  : TCP socket to ESP32 SoftAP (default 192.168.4.1:3333)

Frame format (20 bytes LE)
--------------------------
    H  uint16 magic = 0xAA55      (bytes on the wire: 0x55 0xAA)
    B  uint8  sensor_id           (1..9)
    B  uint8  axis                (0=scalar, 1=x, 2=y, 3=z)
    B  uint8  kind                (0=clean, 1=noisy, 2=denoised)
    B  uint8  flags               DATA_FLAG_* bits
    H  uint16 reserved
    I  uint32 seq                 (monotonic)
    I  uint32 timestamp_ms        (ESP32 uptime, ms)
    f  float32 value
"""
from __future__ import annotations

import argparse
import collections
import math
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("ERROR: pyserial not installed. pip install -r tools/requirements.txt")
    sys.exit(1)

try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
except ImportError:
    print("ERROR: pyqtgraph/PyQt6 not installed. pip install -r tools/requirements.txt")
    sys.exit(1)


# ====================================================================
# Protocol
# ====================================================================
FRAME_MAGIC = 0xAA55
DATA_KIND_CLEAN = 0
DATA_KIND_NOISY = 1
DATA_KIND_DENOISED = 2
VALID_DATA_KINDS = (DATA_KIND_CLEAN, DATA_KIND_NOISY, DATA_KIND_DENOISED)
FRAME_FMT = "<HBBBBHIIf"
FRAME_SIZE = struct.calcsize(FRAME_FMT)
assert FRAME_SIZE == 20

# (sensor_id, name, axis_names, unit, group_name, axis_color_rgb)
SENSORS = {
    1: ("vibration",   ["v"],            "binary", "Other",  [(255, 90, 90)]),
    2: ("current",     ["I"],            "A",      "Other",  [(140, 70, 220)]),
    3: ("pressure",    ["P"],            "Pa",     "Other",  [(120, 220, 120)]),
    4: ("temperature", ["T"],            "C",      "Other",  [(220, 220, 60)]),
    5: ("microphone",  ["mic"],          "rms",    "Other",  [(180, 120, 220)]),
    6: ("photodiode",  ["pd"],           "V",      "Other",  [(60, 200, 200)]),
    7: ("mag",         ["mx", "my", "mz"], "uT",   "Mag",
        [(220, 80, 80), (80, 200, 80), (80, 140, 240)]),
    8: ("gyro",        ["gx", "gy", "gz"], "rad/s","Gyro",
        [(220, 80, 80), (80, 200, 80), (80, 140, 240)]),
    9: ("accel",       ["ax", "ay", "az"], "m/s^2","Accel",
        [(220, 80, 80), (80, 200, 80), (80, 140, 240)]),
}

LIVE_WINDOW_SEC  = 30.0
HISTORY_BIN_SEC  = 1.0
PLOT_UPDATE_HZ   = 8
HISTORY_MAX_BINS = 60_000
LIVE_RING_MAX    = 200_000
RECONNECT_DELAY_SEC = 1.0
MAX_LIVE_PLOT_POINTS = 2400
MAX_HISTORY_PLOT_POINTS = 6000
REFRESH_DEBOUNCE_MS = 25


# ====================================================================
# Data structures
# ====================================================================
@dataclass
class ChannelBuf:
    """Per-channel (sensor_id, axis) buffers + running stats."""
    name: str
    unit: str
    # live ring at full rate
    live_t: collections.deque = field(default_factory=lambda: collections.deque(maxlen=LIVE_RING_MAX))
    live_v: collections.deque = field(default_factory=lambda: collections.deque(maxlen=LIVE_RING_MAX))
    # per-bin history
    hist_t:    list = field(default_factory=list)
    hist_min:  list = field(default_factory=list)
    hist_max:  list = field(default_factory=list)
    hist_mean: list = field(default_factory=list)
    hist_std:  list = field(default_factory=list)
    # in-progress bin
    cur_bin_t0: Optional[float] = None
    cur_bin_min:  float = float("inf")
    cur_bin_max:  float = float("-inf")
    cur_bin_sum:  float = 0.0
    cur_bin_sumsq: float = 0.0
    cur_bin_n:    int   = 0
    # all-time running stats (Welford for numerical stability)
    n_total:   int   = 0
    mean_total: float = 0.0
    m2_total:   float = 0.0  # sum of (x - mean)^2
    last_v: Optional[float] = None
    lock: threading.Lock = field(default_factory=threading.Lock)

    def push(self, t_sec: float, v: float):
        with self.lock:
            self.live_t.append(t_sec)
            self.live_v.append(v)
            self.last_v = v

            # roll live window
            cutoff = t_sec - LIVE_WINDOW_SEC
            while self.live_t and self.live_t[0] < cutoff:
                self.live_t.popleft()
                self.live_v.popleft()

            # all-time Welford
            self.n_total += 1
            delta = v - self.mean_total
            self.mean_total += delta / self.n_total
            delta2 = v - self.mean_total
            self.m2_total += delta * delta2

            # history bin accumulation
            if self.cur_bin_t0 is None:
                self.cur_bin_t0 = t_sec
            if t_sec - self.cur_bin_t0 >= HISTORY_BIN_SEC and self.cur_bin_n > 0:
                self._close_bin()

            if v < self.cur_bin_min: self.cur_bin_min = v
            if v > self.cur_bin_max: self.cur_bin_max = v
            self.cur_bin_sum   += v
            self.cur_bin_sumsq += v * v
            self.cur_bin_n     += 1

    def _close_bin(self):
        n = self.cur_bin_n
        mean = self.cur_bin_sum / n
        var  = max(0.0, self.cur_bin_sumsq / n - mean * mean)
        std  = math.sqrt(var)
        self.hist_t.append(self.cur_bin_t0 + HISTORY_BIN_SEC * 0.5)
        self.hist_min.append(self.cur_bin_min)
        self.hist_max.append(self.cur_bin_max)
        self.hist_mean.append(mean)
        self.hist_std.append(std)
        if len(self.hist_t) > HISTORY_MAX_BINS:
            drop = len(self.hist_t) - HISTORY_MAX_BINS
            del self.hist_t[:drop]
            del self.hist_min[:drop]
            del self.hist_max[:drop]
            del self.hist_mean[:drop]
            del self.hist_std[:drop]
        self.cur_bin_t0 = None
        self.cur_bin_min = float("inf")
        self.cur_bin_max = float("-inf")
        self.cur_bin_sum = 0.0
        self.cur_bin_sumsq = 0.0
        self.cur_bin_n = 0

    def stats_alltime(self) -> tuple[float, float, float]:
        """Return (bias, std, rms) over all-time samples received."""
        with self.lock:
            if self.n_total == 0:
                return (0.0, 0.0, 0.0)
            mean = self.mean_total
            var  = self.m2_total / self.n_total
            std  = math.sqrt(max(0.0, var))
            rms  = math.sqrt(mean * mean + var)
            return (mean, std, rms)

    def rate_recent_hz(self) -> float:
        """Estimate channel rate from the visible live window."""
        with self.lock:
            n = len(self.live_t)
            if n < 2:
                return 0.0
            dt = self.live_t[-1] - self.live_t[0]
            if dt <= 0.0:
                return 0.0
            return (n - 1) / dt


# ====================================================================
# Sources
# ====================================================================
class Source:
    def read(self, n: int) -> bytes: raise NotImplementedError
    def close(self): pass
    def name(self) -> str: return self.__class__.__name__


class SerialSource(Source):
    def __init__(self, port: str, baud: int = 1500000):
        self.port = port
        self.ser = serial.Serial(port, baud, timeout=0.5)
    def read(self, n: int) -> bytes:
        return self.ser.read(n)
    def close(self):
        try: self.ser.close()
        except Exception: pass
    def name(self) -> str: return f"serial:{self.port}"


class TcpSource(Source):
    def __init__(self, host: str, port: int = 3333):
        self.host = host
        self.port = port
        self.sock = socket.create_connection((host, port), timeout=2.0)
        self.sock.settimeout(1.0)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    def read(self, n: int) -> bytes:
        try:
            data = self.sock.recv(n)
            if data == b"":
                raise ConnectionError("TCP peer closed")
            return data
        except socket.timeout: return b""
    def close(self):
        try: self.sock.close()
        except Exception: pass
    def name(self) -> str: return f"tcp:{self.host}:{self.port}"


def autodetect_serial() -> Optional[str]:
    keep_keys = ("USB JTAG", "USB Serial", "Espressif",
                 "CP210", "CH340", "wchusb", "usbmodem")
    for p in list_ports.comports():
        s = " ".join(filter(None, [p.description, p.manufacturer, p.product, p.device]))
        if any(k.lower() in s.lower() for k in keep_keys):
            return p.device
    return None


def target_name(args: argparse.Namespace) -> str:
    if args.tcp:
        host, _, port = args.tcp.partition(":")
        return f"tcp:{host}:{port or 3333}"
    if args.serial:
        return f"serial:{args.serial}"
    return "serial:auto"


def open_source(args: argparse.Namespace) -> Source:
    if args.tcp:
        host, _, port = args.tcp.partition(":")
        port = int(port) if port else 3333
        return TcpSource(host, port)
    port = args.serial or autodetect_serial()
    if not port:
        raise ConnectionError("no matching serial port found")
    return SerialSource(port, args.baud)


# ====================================================================
# Reader thread
# ====================================================================
class Reader(threading.Thread):
    def __init__(self, args: argparse.Namespace, channels: dict, dump_path: Path):
        super().__init__(daemon=True)
        self.args = args
        self.src: Optional[Source] = None
        self.channels = channels
        self.dump_path = dump_path
        self.stop_flag = False
        self.frames = 0
        self.dropped_seq = 0
        self.last_seq: Optional[int] = None
        self.t0_wall = time.time()
        self.t0_ms: Optional[int] = None
        self.latest_t_sec = 0.0
        self.state_lock = threading.Lock()
        self.src_label = target_name(args)
        self.connection_status = "waiting for device"

    def _set_connection_status(self, src_label: str, status: str):
        with self.state_lock:
            self.src_label = src_label
            self.connection_status = status

    def connection_snapshot(self) -> tuple[str, str]:
        with self.state_lock:
            return self.src_label, self.connection_status

    def latest_time_sec(self) -> float:
        return self.latest_t_sec

    def _connect_once(self) -> bool:
        label = target_name(self.args)
        self._set_connection_status(label, "connecting")
        try:
            self.src = open_source(self.args)
        except Exception as exc:
            self.src = None
            self._set_connection_status(label, f"waiting: {exc}")
            return False

        self.last_seq = None
        self._set_connection_status(self.src.name(), "connected")
        print(f"[reader] connected -> {self.src.name()}")
        return True

    def _close_source(self, reason: str):
        if self.src is None:
            return
        src_name = self.src.name()
        self.src.close()
        self.src = None
        self.last_seq = None
        self._set_connection_status(target_name(self.args), f"reconnecting: {reason}")
        print(f"[reader] disconnected from {src_name}: {reason}")

    def parse_buffer(self, buf: bytearray, fout):
        i = 0
        n = len(buf)
        while i + FRAME_SIZE <= n:
            if buf[i] != 0x55 or buf[i + 1] != 0xAA:
                i += 1
                continue
            try:
                magic, sid, axis, kind, flags, _reserved, seq, ts_ms, val = struct.unpack_from(FRAME_FMT, buf, i)
            except struct.error:
                i += 1
                continue
            if (magic != FRAME_MAGIC or sid not in SENSORS or axis > 3 or
                    kind not in VALID_DATA_KINDS):
                i += 1
                continue

            fout.write(buf[i : i + FRAME_SIZE])
            self.frames += 1

            if self.last_seq is not None:
                gap = (seq - self.last_seq - 1) & 0xFFFFFFFF
                if 0 < gap < 1_000_000:
                    self.dropped_seq += gap
            self.last_seq = seq

            if self.t0_ms is None or ts_ms < self.t0_ms:
                self.t0_ms = ts_ms
            t_sec = (ts_ms - self.t0_ms) / 1000.0
            self.latest_t_sec = t_sec

            ch = self.channels.get((sid, axis, kind))
            if ch is not None:
                ch.push(t_sec, val)

            i += FRAME_SIZE
        del buf[:i]

    def run(self):
        buf = bytearray()
        with open(self.dump_path, "ab", buffering=64 * 1024) as fout:
            print(f"[reader] writing -> {self.dump_path}")
            while not self.stop_flag:
                if self.src is None:
                    if not self._connect_once():
                        time.sleep(RECONNECT_DELAY_SEC)
                    continue

                try:
                    chunk = self.src.read(4096)
                except Exception as exc:
                    buf.clear()
                    self._close_source(str(exc))
                    time.sleep(RECONNECT_DELAY_SEC)
                    continue

                if not chunk:
                    continue
                buf.extend(chunk)
                if len(buf) > 64 * 1024:
                    del buf[: len(buf) - FRAME_SIZE]
                self.parse_buffer(buf, fout)
        if self.src is not None:
            self.src.close()


# ====================================================================
# Plot panel
# ====================================================================
class SensorPanel:
    """One channel = clean/noisy overlays + denoised trace + denoised stats."""

    def __init__(self, plot: pg.PlotItem, label: str, unit: str, color_rgb):
        self.plot = plot
        self.label = label
        self.unit = unit
        col = QtGui.QColor(*color_rgb)
        col_band = QtGui.QColor(*color_rgb, 28)
        col_clean = QtGui.QColor(70, 70, 70, 95)
        col_noisy = QtGui.QColor(230, 120, 35, 115)

        plot.showGrid(x=True, y=True, alpha=0.2)
        plot.setLabel("left", f"{label} ({unit})")
        plot.setLabel("bottom", "time (s)")
        plot.enableAutoRange(axis="y", enable=False)
        plot.setAutoVisible(y=False)

        self.band_lo = plot.plot(pen=pg.mkPen(col_band, width=0))
        self.band_hi = plot.plot(pen=pg.mkPen(col_band, width=0))
        self.band_fill = pg.FillBetweenItem(self.band_lo, self.band_hi,
                                            brush=pg.mkBrush(col_band))
        plot.addItem(self.band_fill)

        self.clean_curve = plot.plot(pen=pg.mkPen(col_clean, width=0.8, style=QtCore.Qt.PenStyle.DotLine))
        self.noisy_curve = plot.plot(pen=pg.mkPen(col_noisy, width=0.9))
        self.denoised_curve = plot.plot(pen=pg.mkPen(col, width=1.8))
        self.mean_curve = plot.plot(pen=pg.mkPen(QtGui.QColor(0, 0, 0), width=1.5))

    @staticmethod
    def _finite_xy(t: np.ndarray, v: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if t.size == 0 or v.size == 0:
            empty = np.empty(0, dtype=np.float64)
            return empty, empty
        n = min(t.size, v.size)
        tt = t[:n]
        vv = v[:n]
        mask = np.isfinite(tt) & np.isfinite(vv)
        if mask.all():
            return tt, vv
        return tt[mask], vv[mask]

    @classmethod
    def _downsample_xy(cls, t: np.ndarray, v: np.ndarray, max_points: int) -> tuple[np.ndarray, np.ndarray]:
        t, v = cls._finite_xy(t, v)
        n = t.size
        if max_points <= 0 or n <= max_points:
            return t, v
        if max_points < 4:
            idx = np.linspace(0, n - 1, max(1, max_points), dtype=np.int64)
            return t[idx], v[idx]

        bucket_count = max(1, (max_points - 4) // 2)
        bucket_size = max(1, int(math.ceil(n / bucket_count)))
        bucket_count = n // bucket_size
        if bucket_count <= 0:
            return t, v

        trim = bucket_count * bucket_size
        vv = v[:trim].reshape(bucket_count, bucket_size)
        base = np.arange(bucket_count, dtype=np.int64) * bucket_size
        min_idx = base + np.argmin(vv, axis=1)
        max_idx = base + np.argmax(vv, axis=1)
        pairs = np.column_stack((min_idx, max_idx))
        pairs.sort(axis=1)

        idx_parts = [np.array([0], dtype=np.int64), pairs.ravel()]
        if trim < n:
            tail = v[trim:]
            idx_parts.append(np.array([
                trim + int(np.argmin(tail)),
                trim + int(np.argmax(tail)),
            ], dtype=np.int64))
        idx_parts.append(np.array([n - 1], dtype=np.int64))

        indices = np.unique(np.concatenate(idx_parts))
        if indices.size > max_points:
            keep = np.linspace(0, indices.size - 1, max_points, dtype=np.int64)
            indices = indices[keep]
        return t[indices], v[indices]

    @staticmethod
    def _downsample_band(t: np.ndarray,
                         lo: np.ndarray,
                         hi: np.ndarray,
                         max_points: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if t.size == 0 or lo.size == 0 or hi.size == 0:
            empty = np.empty(0, dtype=np.float64)
            return empty, empty, empty

        n = min(t.size, lo.size, hi.size)
        tt = t[:n]
        yy_lo = lo[:n]
        yy_hi = hi[:n]
        mask = np.isfinite(tt) & np.isfinite(yy_lo) & np.isfinite(yy_hi)
        if not mask.all():
            tt = tt[mask]
            yy_lo = yy_lo[mask]
            yy_hi = yy_hi[mask]
            n = tt.size
        if max_points <= 0 or n <= max_points:
            return tt, yy_lo, yy_hi
        if max_points < 2:
            idx = np.array([0], dtype=np.int64)
            return tt[idx], yy_lo[idx], yy_hi[idx]

        bucket_target = max(1, max_points - 1)
        bucket_size = max(1, int(math.ceil(n / bucket_target)))
        bucket_count = n // bucket_size
        if bucket_count <= 0:
            return tt, yy_lo, yy_hi

        trim = bucket_count * bucket_size
        t_mat = tt[:trim].reshape(bucket_count, bucket_size)
        lo_mat = yy_lo[:trim].reshape(bucket_count, bucket_size)
        hi_mat = yy_hi[:trim].reshape(bucket_count, bucket_size)

        out_t = [np.mean(t_mat, axis=1)]
        out_lo = [np.min(lo_mat, axis=1)]
        out_hi = [np.max(hi_mat, axis=1)]
        if trim < n:
            out_t.append(np.array([float(np.mean(tt[trim:]))], dtype=np.float64))
            out_lo.append(np.array([float(np.min(yy_lo[trim:]))], dtype=np.float64))
            out_hi.append(np.array([float(np.max(yy_hi[trim:]))], dtype=np.float64))

        return np.concatenate(out_t), np.concatenate(out_lo), np.concatenate(out_hi)

    @staticmethod
    def _set_curve_data(curve, visible: bool, t: np.ndarray, v: np.ndarray):
        curve.setVisible(visible)
        if visible:
            curve.setData(t, v)

    @staticmethod
    def _set_band_visible(band_lo, band_hi, band_fill, visible: bool):
        band_lo.setVisible(visible)
        band_hi.setVisible(visible)
        band_fill.setVisible(visible)

    @staticmethod
    def _visible_values(t: np.ndarray, v: np.ndarray, x_lo: float, x_hi: float) -> np.ndarray:
        tt, vv = SensorPanel._finite_xy(t, v)
        if tt.size == 0:
            return np.empty(0, dtype=np.float64)
        mask = (tt >= x_lo) & (tt <= x_hi)
        return vv[mask]

    def _set_robust_y_range(self, series: list[tuple[np.ndarray, np.ndarray]], x_lo: float, x_hi: float):
        chunks = [self._visible_values(t, v, x_lo, x_hi) for t, v in series]
        chunks = [c for c in chunks if c.size]
        if not chunks:
            return

        values = np.concatenate(chunks)
        values = values[np.isfinite(values)]
        if values.size == 0:
            return

        if values.size >= 40:
            y_lo, y_hi = np.nanpercentile(values, [0.5, 99.5])
        else:
            y_lo = float(np.nanmin(values))
            y_hi = float(np.nanmax(values))

        if not np.isfinite(y_lo) or not np.isfinite(y_hi):
            return
        if y_hi <= y_lo:
            center = float(y_hi)
            pad = max(abs(center) * 0.02, 1e-3)
            y_lo = center - pad
            y_hi = center + pad
        else:
            span = y_hi - y_lo
            pad = max(span * 0.15, 1e-4)
            y_lo -= pad
            y_hi += pad

        self.plot.setYRange(float(y_lo), float(y_hi), padding=0.0)

    def update(self,
               clean_live_t: np.ndarray, clean_live_v: np.ndarray,
               clean_hist_t: np.ndarray, clean_hist_mean: np.ndarray,
               noisy_live_t: np.ndarray, noisy_live_v: np.ndarray,
               noisy_hist_t: np.ndarray, noisy_hist_mean: np.ndarray,
               denoised_live_t: np.ndarray, denoised_live_v: np.ndarray,
               denoised_hist_t: np.ndarray, denoised_hist_mean: np.ndarray,
               denoised_hist_min: np.ndarray, denoised_hist_max: np.ndarray, denoised_hist_std: np.ndarray,
               x_lo: float, x_hi: float, view_mode: str,
               stats: tuple[float, float, float], rate_hz: float,
               show_clean: bool, show_noisy: bool, show_denoised: bool, show_band: bool,
               max_live_points: int, max_history_points: int):

        if view_mode == "live":
            clean_live_t, clean_live_v = self._downsample_xy(clean_live_t, clean_live_v, max_live_points)
            noisy_live_t, noisy_live_v = self._downsample_xy(noisy_live_t, noisy_live_v, max_live_points)
            denoised_live_t, denoised_live_v = self._downsample_xy(denoised_live_t, denoised_live_v, max_live_points)
            mean_t, mean_v = self._downsample_xy(denoised_hist_t, denoised_hist_mean, max_history_points)

            self._set_curve_data(self.clean_curve, show_clean, clean_live_t, clean_live_v)
            self._set_curve_data(self.noisy_curve, show_noisy, noisy_live_t, noisy_live_v)
            self._set_curve_data(self.denoised_curve, show_denoised, denoised_live_t, denoised_live_v)
            self.mean_curve.setData(mean_t, mean_v)
            y_series = []
            if show_clean:
                y_series.append((clean_live_t, clean_live_v))
            if show_noisy:
                y_series.append((noisy_live_t, noisy_live_v))
            if show_denoised:
                y_series.append((denoised_live_t, denoised_live_v))
            y_series.append((mean_t, mean_v))
            if show_band:
                n = min(denoised_hist_t.size, denoised_hist_mean.size, denoised_hist_std.size)
                band_t, band_lo, band_hi = self._downsample_band(
                    denoised_hist_t[:n],
                    denoised_hist_mean[:n] - denoised_hist_std[:n],
                    denoised_hist_mean[:n] + denoised_hist_std[:n],
                    max_history_points,
                )
                self._set_band_visible(self.band_lo, self.band_hi, self.band_fill, True)
                self.band_lo.setData(band_t, band_lo)
                self.band_hi.setData(band_t, band_hi)
                y_series.extend([(band_t, band_lo), (band_t, band_hi)])
            else:
                self._set_band_visible(self.band_lo, self.band_hi, self.band_fill, False)
        else:
            denoised_band_t = denoised_hist_t
            clean_hist_t, clean_hist_mean = self._downsample_xy(clean_hist_t, clean_hist_mean, max_history_points)
            noisy_hist_t, noisy_hist_mean = self._downsample_xy(noisy_hist_t, noisy_hist_mean, max_history_points)
            denoised_hist_t, denoised_hist_mean = self._downsample_xy(
                denoised_hist_t, denoised_hist_mean, max_history_points
            )

            self._set_curve_data(self.clean_curve, show_clean, clean_hist_t, clean_hist_mean)
            self._set_curve_data(self.noisy_curve, show_noisy, noisy_hist_t, noisy_hist_mean)
            self._set_curve_data(self.denoised_curve, show_denoised, denoised_hist_t, denoised_hist_mean)
            self.mean_curve.setData(denoised_hist_t, denoised_hist_mean)
            y_series = []
            if show_clean:
                y_series.append((clean_hist_t, clean_hist_mean))
            if show_noisy:
                y_series.append((noisy_hist_t, noisy_hist_mean))
            if show_denoised:
                y_series.append((denoised_hist_t, denoised_hist_mean))
            y_series.append((denoised_hist_t, denoised_hist_mean))
            if show_band:
                band_t, band_lo, band_hi = self._downsample_band(
                    denoised_band_t, denoised_hist_min, denoised_hist_max, max_history_points
                )
                self._set_band_visible(self.band_lo, self.band_hi, self.band_fill, True)
                self.band_lo.setData(band_t, band_lo)
                self.band_hi.setData(band_t, band_hi)
                y_series.extend([(band_t, band_lo), (band_t, band_hi)])
            else:
                self._set_band_visible(self.band_lo, self.band_hi, self.band_fill, False)

        self.plot.setXRange(x_lo, x_hi, padding=0.0)
        self._set_robust_y_range(y_series, x_lo, x_hi)

        bias, std, rms = stats
        self.plot.setTitle(
            f"{self.label}   Hz={rate_hz:5.1f}   Bias={bias:+.4g}   Std={std:.4g}   RMS={rms:.4g}"
        )


# ====================================================================
# Viewer
# ====================================================================
def channels_for_sensor(sid: int):
    name, axes, _, _, _ = SENSORS[sid]
    if len(axes) == 1:
        return [(sid, 0)]
    return [(sid, a) for a in (1, 2, 3)]


class Viewer(QtWidgets.QMainWindow):
    def __init__(self, channels: dict, reader: Reader, args: argparse.Namespace):
        super().__init__()
        pg.setConfigOptions(antialias=False, useOpenGL=False, background="w", foreground="k")
        self.channels = channels
        self.reader = reader
        self.plot_hz = max(1.0, float(args.plot_hz))
        self.max_live_points = max(8, int(args.max_live_points))
        self.max_history_points = max(8, int(args.max_history_points))
        self.setWindowTitle("Project SHIELD - Live Viewer")
        self.resize(1400, 900)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        v = QtWidgets.QVBoxLayout(central)

        # status bar
        self.lbl_status = QtWidgets.QLabel("connecting...")
        self.lbl_status.setStyleSheet("font-family: monospace; padding: 4px;")
        v.addWidget(self.lbl_status)

        # tabs
        self.tabs = QtWidgets.QTabWidget()
        v.addWidget(self.tabs, stretch=1)

        # Build tabs grouped by SENSORS[sid].group
        groups: dict[str, list[int]] = {}
        for sid, (_, _, _, group, _) in SENSORS.items():
            groups.setdefault(group, []).append(sid)
        # Order: Accel, Gyro, Mag, Other
        order = ["Accel", "Gyro", "Mag", "Other"]
        self._tab_panel_keys: list[list[tuple[int, int]]] = []
        for g in order:
            if g not in groups:
                continue
            sids = groups[g]
            gl = pg.GraphicsLayoutWidget()
            gl.setBackground("w")
            panel_keys = self._fill_group_tab(gl, sids)
            self.tabs.addTab(gl, g)
            self._tab_panel_keys.append(panel_keys)

        self.panels: dict[tuple[int, int], SensorPanel] = self._panels_index

        # controls
        controls = QtWidgets.QHBoxLayout()
        v.addLayout(controls)
        self.btn_full = QtWidgets.QPushButton("View: full history")
        self.btn_live = QtWidgets.QPushButton(f"View: last {int(LIVE_WINDOW_SEC)}s")
        self.btn_full.setCheckable(True)
        self.btn_live.setCheckable(True)
        self.btn_live.setChecked(True)
        self.btn_full.clicked.connect(lambda: self._set_view("full"))
        self.btn_live.clicked.connect(lambda: self._set_view("live"))
        controls.addWidget(self.btn_full)
        controls.addWidget(self.btn_live)
        self.chk_clean = QtWidgets.QCheckBox("raw")
        self.chk_noisy = QtWidgets.QCheckBox("noisy")
        self.chk_denoised = QtWidgets.QCheckBox("denoised")
        self.chk_band = QtWidgets.QCheckBox("stats band")
        self.chk_clean.setChecked(True)
        self.chk_noisy.setChecked(True)
        self.chk_denoised.setChecked(True)
        self.chk_band.setChecked(False)
        controls.addWidget(self.chk_clean)
        controls.addWidget(self.chk_noisy)
        controls.addWidget(self.chk_denoised)
        controls.addWidget(self.chk_band)
        controls.addStretch(1)

        self._view_mode = "live"

        self._refresh_debounce_timer = QtCore.QTimer(self)
        self._refresh_debounce_timer.setSingleShot(True)
        self._refresh_debounce_timer.timeout.connect(self.refresh)
        self.tabs.currentChanged.connect(lambda _idx: self.request_refresh())
        for chk in (self.chk_clean, self.chk_noisy, self.chk_denoised, self.chk_band):
            chk.stateChanged.connect(lambda _state: self.request_refresh())

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(max(16, int(1000 / self.plot_hz)))

    def _set_view(self, mode: str):
        self._view_mode = mode
        self.btn_full.setChecked(mode == "full")
        self.btn_live.setChecked(mode == "live")
        self.request_refresh()

    def request_refresh(self):
        self._refresh_debounce_timer.start(REFRESH_DEBOUNCE_MS)

    def _fill_group_tab(self, gl: pg.GraphicsLayoutWidget, sids: list[int]) -> list[tuple[int, int]]:
        if not hasattr(self, "_panels_index"):
            self._panels_index = {}
        panel_keys: list[tuple[int, int]] = []

        # If group has a single 3-axis sensor (e.g. Accel), do 3 rows.
        # If group has multiple scalar sensors (Other), do a 2-col grid.
        if len(sids) == 1:
            sid = sids[0]
            name, axes, unit, _, colors = SENSORS[sid]
            if len(axes) == 1:
                p = gl.addPlot(row=0, col=0)
                panel = SensorPanel(p, f"{name}.{axes[0]}", unit, colors[0])
                key = (sid, 0)
                self._panels_index[key] = panel
                panel_keys.append(key)
            else:
                for i, ax in enumerate(axes):
                    p = gl.addPlot(row=i, col=0)
                    panel = SensorPanel(p, f"{name}.{ax}", unit, colors[i])
                    key = (sid, i + 1)
                    self._panels_index[key] = panel
                    panel_keys.append(key)
        else:
            # Other tab: lay scalars in a 2-column grid
            for idx, sid in enumerate(sids):
                name, axes, unit, _, colors = SENSORS[sid]
                r, c = divmod(idx, 2)
                p = gl.addPlot(row=r, col=c)
                if len(axes) == 1:
                    panel = SensorPanel(p, f"{name}.{axes[0]}", unit, colors[0])
                    key = (sid, 0)
                    self._panels_index[key] = panel
                    panel_keys.append(key)
                else:
                    # unlikely, but support
                    for i, ax in enumerate(axes):
                        p2 = gl.addPlot(row=r * 3 + i, col=c)
                        panel = SensorPanel(p2, f"{name}.{ax}", unit, colors[i])
                        key = (sid, i + 1)
                        self._panels_index[key] = panel
                        panel_keys.append(key)
        return panel_keys

    def _visible_panel_keys(self) -> list[tuple[int, int]]:
        idx = self.tabs.currentIndex()
        if 0 <= idx < len(self._tab_panel_keys):
            return self._tab_panel_keys[idx]
        return list(self.panels.keys())

    @staticmethod
    def _empty_snapshot() -> dict:
        empty = np.empty(0, dtype=np.float64)
        return dict(
            lt=empty, lv=empty, ht=empty,
            hmean=empty, hmin=empty, hmax=empty, hstd=empty,
            stats=(0.0, 0.0, 0.0), rate_hz=0.0,
        )

    @staticmethod
    def _filter_arrays(t: np.ndarray,
                       arrays: list[np.ndarray],
                       x_lo: Optional[float],
                       x_hi: Optional[float]) -> tuple[np.ndarray, ...]:
        sizes = [t.size] + [arr.size for arr in arrays]
        n = min(sizes) if sizes else 0
        if n == 0:
            empty = np.empty(0, dtype=np.float64)
            return (empty, *[empty for _ in arrays])

        tt = t[:n]
        clipped = [arr[:n] for arr in arrays]
        mask = np.isfinite(tt)
        for arr in clipped:
            mask &= np.isfinite(arr)
        if x_lo is not None:
            mask &= tt >= x_lo
        if x_hi is not None:
            mask &= tt <= x_hi

        if mask.all():
            return (tt, *clipped)
        return (tt[mask], *[arr[mask] for arr in clipped])

    def _snapshot_channel(self, ch: ChannelBuf, x_lo: float, x_hi: float) -> dict:
        empty = np.empty(0, dtype=np.float64)
        with ch.lock:
            if self._view_mode == "live":
                lt = np.asarray(ch.live_t, dtype=np.float64)
                lv = np.asarray(ch.live_v, dtype=np.float64)
            else:
                lt = empty
                lv = empty
            ht = np.asarray(ch.hist_t, dtype=np.float64)
            hmean = np.asarray(ch.hist_mean, dtype=np.float64)
            hmin = np.asarray(ch.hist_min, dtype=np.float64)
            hmax = np.asarray(ch.hist_max, dtype=np.float64)
            hstd = np.asarray(ch.hist_std, dtype=np.float64)

            if ch.n_total == 0:
                stats = (0.0, 0.0, 0.0)
            else:
                mean = ch.mean_total
                var = max(0.0, ch.m2_total / ch.n_total)
                std = math.sqrt(var)
                stats = (mean, std, math.sqrt(mean * mean + var))

            n_live = len(ch.live_t)
            if n_live >= 2:
                dt = ch.live_t[-1] - ch.live_t[0]
                rate_hz = (n_live - 1) / dt if dt > 0.0 else 0.0
            else:
                rate_hz = 0.0

        if self._view_mode == "live":
            lt, lv = self._filter_arrays(lt, [lv], x_lo, x_hi)
            ht, hmean, hmin, hmax, hstd = self._filter_arrays(ht, [hmean, hmin, hmax, hstd], x_lo, x_hi)
        else:
            ht, hmean, hmin, hmax, hstd = self._filter_arrays(ht, [hmean, hmin, hmax, hstd], None, None)

        return dict(lt=lt, lv=lv, ht=ht, hmean=hmean,
                    hmin=hmin, hmax=hmax, hstd=hstd,
                    stats=stats, rate_hz=rate_hz)

    def refresh(self):
        latest_t = self.reader.latest_time_sec()
        if self._view_mode == "live":
            x_lo = max(0.0, latest_t - LIVE_WINDOW_SEC)
            x_hi = max(LIVE_WINDOW_SEC, latest_t)
        else:
            x_lo = 0.0
            x_hi = latest_t if latest_t > 0 else 1.0

        visible_panel_keys = self._visible_panel_keys()
        channel_keys: set[tuple[int, int, int]] = set()
        for sid, axis in visible_panel_keys:
            channel_keys.add((sid, axis, DATA_KIND_CLEAN))
            channel_keys.add((sid, axis, DATA_KIND_NOISY))
            channel_keys.add((sid, axis, DATA_KIND_DENOISED))

        snapshots: dict[tuple[int, int, int], dict] = {}
        for key in channel_keys:
            ch = self.channels.get(key)
            if ch is not None:
                snapshots[key] = self._snapshot_channel(ch, x_lo, x_hi)

        empty = self._empty_snapshot()
        show_clean = self.chk_clean.isChecked()
        show_noisy = self.chk_noisy.isChecked()
        show_denoised = self.chk_denoised.isChecked()
        show_band = self.chk_band.isChecked()

        for key in visible_panel_keys:
            panel = self.panels[key]
            sid, axis = key
            clean_key = (sid, axis, DATA_KIND_CLEAN)
            noisy_key = (sid, axis, DATA_KIND_NOISY)
            denoised_key = (sid, axis, DATA_KIND_DENOISED)
            clean_s = snapshots.get(clean_key, empty)
            noisy_s = snapshots.get(noisy_key, empty)
            denoised_s = snapshots.get(denoised_key, empty)
            panel.update(clean_s["lt"], clean_s["lv"], clean_s["ht"], clean_s["hmean"],
                         noisy_s["lt"], noisy_s["lv"], noisy_s["ht"], noisy_s["hmean"],
                         denoised_s["lt"], denoised_s["lv"], denoised_s["ht"], denoised_s["hmean"],
                         denoised_s["hmin"], denoised_s["hmax"], denoised_s["hstd"],
                         x_lo, x_hi, self._view_mode, denoised_s["stats"], denoised_s["rate_hz"],
                         show_clean, show_noisy, show_denoised, show_band,
                         self.max_live_points, self.max_history_points)

        # status line
        elapsed = time.time() - self.reader.t0_wall
        rate = self.reader.frames / elapsed if elapsed > 0 else 0.0
        src_name, conn_status = self.reader.connection_snapshot()
        self.lbl_status.setText(
            f"src={src_name}  status={conn_status}  frames={self.reader.frames}  "
            f"dropped_seq={self.reader.dropped_seq}  rate={rate:7.1f} pkt/s  "
            f"t={latest_t:8.2f}s  view={self._view_mode}"
        )


# ====================================================================
# Main
# ====================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", help="Serial port path (auto-detect if absent)")
    ap.add_argument("--baud", type=int, default=1500000,
                    help="Serial baudrate (USB CDC ignores this)")
    ap.add_argument("--tcp", help="TCP host[:port] (default port 3333)")
    ap.add_argument("--out", default=None, help="Output dir (default: ./pc_runs/RUN_<ts>/)")
    ap.add_argument("--plot-hz", type=float, default=PLOT_UPDATE_HZ,
                    help=f"Plot refresh rate in Hz (default: {PLOT_UPDATE_HZ})")
    ap.add_argument("--max-live-points", type=int, default=MAX_LIVE_PLOT_POINTS,
                    help=f"Max rendered points per live curve (default: {MAX_LIVE_PLOT_POINTS})")
    ap.add_argument("--max-history-points", type=int, default=MAX_HISTORY_PLOT_POINTS,
                    help=f"Max rendered points per full-history curve (default: {MAX_HISTORY_PLOT_POINTS})")
    args = ap.parse_args()
    if args.plot_hz <= 0:
        ap.error("--plot-hz must be > 0")
    if args.max_live_points <= 0:
        ap.error("--max-live-points must be > 0")
    if args.max_history_points <= 0:
        ap.error("--max-history-points must be > 0")

    out_root = Path(args.out) if args.out else (Path.cwd() / "pc_runs")
    out_root.mkdir(parents=True, exist_ok=True)
    run_dir = out_root / time.strftime("RUN_%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    dump_path = run_dir / "stream.bin"

    # Build channel buffer table mirroring SENSORS spec.
    channels: dict[tuple[int, int, int], ChannelBuf] = {}
    for sid, (name, axes, unit, _, _) in SENSORS.items():
        if len(axes) == 1:
            channels[(sid, 0, DATA_KIND_CLEAN)] = ChannelBuf(name=f"{name}.clean", unit=unit)
            channels[(sid, 0, DATA_KIND_NOISY)] = ChannelBuf(name=f"{name}.noisy", unit=unit)
            channels[(sid, 0, DATA_KIND_DENOISED)] = ChannelBuf(name=f"{name}.denoised", unit=unit)
        else:
            for ax_i, ax_name in enumerate(axes, start=1):
                channels[(sid, ax_i, DATA_KIND_CLEAN)] = ChannelBuf(name=f"{name}.{ax_name}.clean", unit=unit)
                channels[(sid, ax_i, DATA_KIND_NOISY)] = ChannelBuf(name=f"{name}.{ax_name}.noisy", unit=unit)
                channels[(sid, ax_i, DATA_KIND_DENOISED)] = ChannelBuf(name=f"{name}.{ax_name}.denoised", unit=unit)

    reader = Reader(args, channels, dump_path)
    reader.start()

    app = QtWidgets.QApplication(sys.argv)
    win = Viewer(channels, reader, args)
    win.show()
    try:
        rc = app.exec()
    finally:
        reader.stop_flag = True
        reader.join(timeout=2.0)
    sys.exit(rc)


if __name__ == "__main__":
    main()

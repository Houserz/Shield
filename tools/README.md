# SHIELD PC Viewer

Real-time viewer for the on-device streaming pipeline.

## Setup

```bash
python -m venv .venv
source .venv/bin/activate           # Windows: .venv\Scripts\activate
pip install -r tools/requirements.txt
```

## Run

### USB (primary)

Plug the ESP32-S3 USB cable into your laptop. Then:

```bash
python tools/pc_viewer.py
```

If auto-detection picks the wrong port, override:

```bash
python tools/pc_viewer.py --serial /dev/cu.usbmodem1101    # macOS
python tools/pc_viewer.py --serial COM7                    # Windows
python tools/pc_viewer.py --serial /dev/ttyACM0            # Linux
```

### Wi-Fi (backup, no USB cable)

1. On your laptop, join Wi-Fi `SHIELD_DAQ` (password `shield1234`).
2. Run:

```bash
python tools/pc_viewer.py --tcp 192.168.4.1
```

> Joining the SoftAP disconnects you from school Wi-Fi for the duration of
> the session. The viewer doesn't need internet, so this is fine.

## What it does

- Two transports, same 16-byte binary frame format (see `streamer.h`).
- Saves every received frame to `pc_runs/RUN_<timestamp>/stream.bin`.
  This is the PC-side mirror of what the ESP32 writes to SD.
- Per-channel **live ring** (last 30 s @ full rate, blue line) +
  **history bins** (1 Hz min/max envelope + mean line, gray) so you can see
  both detail and full duration without exhausting RAM.
- Toggle `View: full history` / `View: last 30s` to switch x-range.

## Replaying / analyzing later

`stream.bin` is just the same `stream_pkt_t` packed back-to-back. To load
in NumPy:

```python
import numpy as np
dt = np.dtype([
    ('magic',  '<u2'),
    ('sid',    'u1'),
    ('axis',   'u1'),
    ('seq',    '<u4'),
    ('ts_ms',  '<u4'),
    ('value',  '<f4'),
])
arr = np.fromfile('pc_runs/RUN_xxx/stream.bin', dtype=dt)
mask = (arr['sid'] == 9) & (arr['axis'] == 1)   # accel.x
t = (arr['ts_ms'][mask] - arr['ts_ms'][mask][0]) / 1000.0
v = arr['value'][mask]
```

## Notes

- The ESP32 console (ESP_LOG) shares the USB Serial/JTAG with the data
  stream. The viewer hunts for the magic header, so log lines just appear
  as a few discarded bytes; if you see drops, lower log verbosity in
  `idf.py menuconfig` → *Component config* → *Log output* → set default
  level to `Warn` or `Error`.
- During Wi-Fi mode, ESP32 stops Wi-Fi when the laptop disconnects from
  the SoftAP and resumes when it reconnects; no power-cycle needed.

## Toggling USB / Wi-Fi at build time

Defaults: USB on, Wi-Fi off (Wi-Fi radio adds ~100 mA average).

```bash
# default build:                  USB on,  Wi-Fi off
idf.py build

# both transports on:
idf.py -DSTREAMER_USB=1 -DSTREAMER_WIFI=1 build

# Wi-Fi only (no USB streaming):
idf.py -DSTREAMER_USB=0 -DSTREAMER_WIFI=1 build

# both off (acquisition + SD only, no streaming):
idf.py -DSTREAMER_USB=0 -DSTREAMER_WIFI=0 build
```

If you change these flags after a previous build, run `idf.py fullclean`
once so CMake re-evaluates `REQUIRES`.

You can also edit the defaults directly in
`components/streamer/include/streamer.h` (`STREAMER_ENABLE_USB` /
`STREAMER_ENABLE_WIFI`).

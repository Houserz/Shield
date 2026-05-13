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

You can start the viewer before the ESP32-S3 is plugged in or streaming:

```bash
python tools/pc_viewer.py
```

The window opens immediately, creates the output `stream.bin`, and waits
until a matching ESP32 serial port appears. If the cable is unplugged during
a run, the reader keeps the window open and reconnects when the port returns.

If auto-detection picks the wrong port, override:

```bash
python tools/pc_viewer.py --serial /dev/cu.usbmodem1101    # macOS
python tools/pc_viewer.py --serial COM7                    # Windows
python tools/pc_viewer.py --serial /dev/ttyACM0            # Linux
```

### Wi-Fi (backup, no USB cable)

Start the viewer in TCP mode before or after joining the ESP32 SoftAP:

```bash
python tools/pc_viewer.py --tcp 192.168.4.1
```

The window opens immediately and retries the TCP connection until the ESP32
SoftAP/server is reachable. If the connection drops, it returns to reconnect
mode without closing the recording window.

To connect over Wi-Fi, join `SHIELD_DAQ` (password `shield1234`) on your
laptop when the ESP32 SoftAP is available.

> Joining the SoftAP disconnects you from school Wi-Fi for the duration of
> the session. The viewer doesn't need internet, so this is fine.

## What it does

- Two transports, same 20-byte binary frame format (see `streamer.h`).
- Saves every received frame to `pc_runs/RUN_<timestamp>/stream.bin`.
  This is the PC-side live mirror of the same raw/processed values written to SD;
  vector SD records are expanded to one stream packet per axis.
- Per-channel **live ring** overlays raw and processed samples, plus
  **history bins** (1 Hz min/max envelope + mean line) for long runs.
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
    ('kind',   'u1'),   # 0=raw, 1=processed
    ('flags',  'u1'),
    ('reserved', '<u2'),
    ('seq',    '<u4'),
    ('ts_ms',  '<u4'),
    ('value',  '<f4'),
])
arr = np.fromfile('pc_runs/RUN_xxx/stream.bin', dtype=dt)
mask = (arr['sid'] == 9) & (arr['axis'] == 1) & (arr['kind'] == 1)  # processed accel.x
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

## Toggling USB / Wi-Fi

Edit `components/streamer/include/streamer_config.h`, then run a normal
build:

```bash
idf.py build flash monitor
```

Common modes:

```c
// Wi-Fi only
#define STREAMER_ENABLE_USB   0
#define STREAMER_ENABLE_WIFI  1

// USB data cable only
#define STREAMER_ENABLE_USB   1
#define STREAMER_ENABLE_WIFI  0

// Both transports
#define STREAMER_ENABLE_USB   1
#define STREAMER_ENABLE_WIFI  1

// SD only, no live stream
#define STREAMER_ENABLE_USB   0
#define STREAMER_ENABLE_WIFI  0
```

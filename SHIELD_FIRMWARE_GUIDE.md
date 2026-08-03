# Project SHIELD — Setup, Flash & Recording Guide

How to power, build, flash, and run the SHIELD ESP32-S3 DAQ firmware. Follow the
sections in order.

---

## 1. Power the board (stable 9 V supply)

The board is powered by a **9 V input** that an onboard 9 V→5 V regulator (LM7805)
converts to a stable 5 V rail. The MicroSD card and the analog sensors need that 5 V
to be clean — a weak or sagging 9 V source is a common cause of SD-card init failures
and drifting current readings.

- Use a **stable 9 V source**: a **fresh 9 V battery**, or a bench supply set to **9 V**.
- Do **not** rely on a nearly-dead 9 V battery for a real run — if the input droops, the
  5 V rail becomes unstable and the SD card may fail to initialize.
- During bench flashing you can power over the USB-C cable, but for an actual recording
  run use the stable 9 V supply so the SD writes reliably.
- Confirm the board is getting stable power before starting a session (LED/behaviour
  steady, no brown-out resets in the log).

---

## 2. One-time setup

### 2.1 Install ESP-IDF v5.5.x (NOT v6.x)
ESP-IDF v6.0 reorganized components (GPIO moved out of `driver` into `esp_driver_gpio`),
which breaks this project with `driver/gpio.h: No such file`. Use the VS Code
**Espressif IDF** extension:
1. Command Palette → **ESP-IDF: Open ESP-IDF Installation Manager** → select **5.5.x**
   explicitly (don't accept a 6.x default) → install.
2. Command Palette → **ESP-IDF: Select Current ESP-IDF Version** → pick the 5.5.x you
   installed. The status bar should read `v5.5.x`.

### 2.2 Put the project on a space-free path
ESP-IDF's build tools dislike spaces / non-ASCII characters in the project path. Prefer
a path like `C:\esp\Shield` over one containing spaces (e.g. `...\Shield drive\Shield`).
If you ever get strange path/build errors, this is the first thing to fix.

### 2.3 Set the target
In an ESP-IDF terminal (VS Code: **ESP-IDF: Open ESP-IDF Terminal**):
```
idf.py set-target esp32s3
```

### 2.4 REQUIRED: enable FATFS long filenames
The DAQ writes files like `fast_data.bin` (9-char base name), which exceed the FAT
"8.3" short-name limit. A freshly generated `sdkconfig` defaults to **no long
filenames**, so file creation fails at runtime with `fopen ... errno 22`. Make sure a
file named **`sdkconfig.defaults`** exists at the repo root containing:
```
CONFIG_FATFS_LFN_HEAP=y
CONFIG_FATFS_MAX_LFN=255
```
Then regenerate the config so it takes effect:
```
del sdkconfig        # (use "rm sdkconfig" on macOS/Linux)
idf.py set-target esp32s3
```
**Commit `sdkconfig.defaults` to the repo** so anyone who clones it gets this
automatically — otherwise every fresh checkout hits the `errno 22` file-write failure.
Verify it's enabled with: `grep FATFS_LFN sdkconfig` → should show `CONFIG_FATFS_LFN_HEAP=y`.

---

## 3. Build, flash, record

Connect the board's **UART** USB port (shows up as a COM port, e.g. `COM6`). From an
ESP-IDF terminal:
```
idf.py -p COM6 flash monitor      # replace COM6 with your port
```
`flash` builds first, writes over UART, then `monitor` shows the boot log.

**Flash over UART — not JTAG.** The extension's flash button may be set to JTAG
(OpenOCD), which fails on a UART-connected board (`esp_usb_jtag: LIBUSB_ERROR_NOT_FOUND
/ could not find or open device`). JTAG is only for step-debugging and is never needed
here. If needed, switch it: Command Palette → **ESP-IDF: Select Flash Method → UART**.

**Use Flash, not Monitor.** The VS Code **Monitor** button only opens a serial viewer —
it never writes firmware. If the board keeps booting the same old build, that's why.
Use the **⚡ Flash** or **🔥 Build-Flash-Monitor** button, or the command above.

**Confirm the flash actually landed:**
- esptool prints `Writing at 0x…`, `Hash of data verified`, `Hard resetting`.
- In the boot log, the `Compile time` is new (not the previous build's).
- A `Checksum mismatch between flashed and built applications` warning means the chip is
  running a *different* binary than you just built — usually because you monitored
  instead of flashing.

**If the build directory is locked** (`ninja: error: failed recompaction: Permission
denied` on `build.ninja`), another process (VS Code) is holding `build/`. Flash the
already-built binary directly, bypassing ninja — run from the `build/` folder:
```
python -m esptool --chip esp32s3 --port COM6 -b 460800 --before default_reset --after hard_reset write_flash "@flash_args"
```
If that reports the port is busy, close any open monitor first (only one process can
own the serial port at a time).

**A successful boot log looks like:**
```
SD CARD INITIALIZED.
... Sensor [0..8] ... initialized OK
Run session created: RUN_xxx at /sdcard/RUN_xxx
Metadata file created OK
System state -> RUNNING, launching tasks...
Acquisition START ...
```
…and the **GPIO 4 status LED turns on**. Then the board is recording.

---

## 4. Recording a run

The firmware is a **one-shot recorder** — there is no start command:
- **Power on** → it scans the SD card for the highest existing `RUN_xxx`, creates the
  next one, and **starts recording immediately**. **GPIO 4 LED on = recording.**
- **Press the GPIO 46 button** to stop: it flushes buffers, finalizes `meta.json`, closes
  the files, and the LED turns off. Wait for the LED to go off before unplugging.
- It also auto-stops after the configured run duration if you don't press the button.
- One session per power cycle; power-cycle to start a new run.

### Shaker experiment (A1) recipe
15-minute session, three stress levels (≈ **60 / 150 / 250 RPM**), three trials each.
Mount the board flat and rigidly (< 1 mm play) and leave ~30 cm of cable slack so cable
tension doesn't add vibration. Within each session:

| Time | Action |
|---|---|
| 0:00 | Power on (recording starts). Keep the shaker **OFF** — settle period. |
| 5:00 | Turn the shaker **ON** at the target RPM. Note the actual RPM. |
| 10:00 | Turn the shaker **OFF**. |
| 10:00–15:00 | Keep recording (recovery). |
| 15:00 | Press the **GPIO 46** button to stop. |

Keep the run under 30 minutes so it stays a clean capture (short runs never trigger
noise injection). After the session, note the new `RUN_xxx` number and copy
`/sdcard/RUN_xxx/` off the card, labeled with the unit and condition.

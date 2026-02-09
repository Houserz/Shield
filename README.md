# Shield Project

## Features

- ✅ **Dual-core Architecture**: Core 0 handles data acquisition, Core 1 handles SD card storage
- ✅ **Multiple Sampling Rates**: Supports 1kHz / 200Hz / 50Hz sampling rates
- ✅ **Hybrid Interfaces**: SPI, I2C, ADC, and GPIO hardware interfaces
- ✅ **OOP Design**: Sensor abstraction implemented in C
- ✅ **High Performance**: Uses FreeRTOS queues and buffering mechanism
- ✅ **Scalable**: Modular design, easy to add new sensors

## Hardware

**MCU**: ESP32-S3 (Dual-Core Xtensa LX7 @ 240MHz)

**Sensors**:
- BNO085 (9-DOF IMU, SPI, 1kHz, **not working yet**)
- SW-420 (Vibration, GPIO, 1kHz, **individually tested OK**)
- ACS723 (Current, ADC, 200Hz, **driver implemented, not tested**)
- MPL3115A2 (Pressure, I2C, 50Hz, **driver implemented, not tested**)
- MCP9808 (Temperature, I2C, 50Hz, **individually tested OK**)
- INMP441 (Microphone, I2S, effective 1kHz, **individually tested OK**)
- Photodiode (751-1015-ND, ADC, 200Hz, **driver implemented, not tested**)

**Storage**: SD Card (SDMMC 4-bit, 40MHz)

## Project Structure

```
├── main/
│   └── main_dual_core.c          # Main program (dual-core)
│
└── components/
    ├── sensor_hal/               # Hardware abstraction layer
    ├── drivers/                  # Sensor drivers (7 sensors)
    ├── data_types/               # Data structures & metadata
    └── sd_storage/               # SD card storage module
```

## Data Flow

```
Sensors → Drivers → FreeRTOS Queues (Core 0 → Core 1) → SD Card
         1kHz/200Hz/50Hz         Buffered Write          Binary Files
```

## Output Format

Each run session creates a directory with:

```
/sdcard/RUN_XXX/
├── fast_data.bin       # 1kHz data (IMU + Vibration + Microphone)
├── medium_data.bin     # 200Hz data (Current + Photodiode)
├── slow_data.bin       # 50Hz data (Pressure + Temperature)
├── meta.json           # Session metadata
└── events.log          # Event log
```

## Quick Start

### 1. Prerequisites

- ESP-IDF v5.0+
- ESP32-S3 development board
- SD card (Class 10 or higher recommended)

### 2. Configure Hardware

Edit `main/main_dual_core.c` to set actual pin configurations:

```c
// Update these based on your PCB design
static spi_config_t bno085_spi_cfg = {
    .spi_host = 2,
    .cs_pin = 5,      // Set actual pins
    .sclk_pin = 18,
    .mosi_pin = 23,
    .miso_pin = 19
};
// ... configure other sensors
```

### 3. Build and Flash

```bash
# Set target
idf.py set-target esp32s3

# Build
idf.py build

# Flash
idf.py -p /dev/ttyUSB0 flash

# Monitor
idf.py -p /dev/ttyUSB0 monitor
```

## Key Concepts

### OOP Polymorphism in C

Sensors are abstracted using function pointers for polymorphism:

```c
typedef struct SensorContext {
    int id;
    sensor_type_t type;
    int sampling_rate_hz;
    void *hw_config;                    // Polymorphic config
    bool (*init)(struct SensorContext *ctx);      // Virtual init
    bool (*read_sample)(struct SensorContext *ctx, float *data_out);
} SensorContext_t;
```

### Task Pinning

Tasks are pinned to specific cores for optimal performance:

- **Core 0** (PRO_CPU): Fast (P=10), Medium (P=8), Slow (P=6) acquisition tasks
- **Core 1** (APP_CPU): SD Writer (P=5) storage task

### Buffered Writing

4KB buffers with periodic flushing (every 100ms) ensure efficient SD card writes.

## API Overview

### Sensor HAL (`components/sensor_hal/`)

```c
bool xxx_init(SensorContext_t *ctx);
bool xxx_read_sample(SensorContext_t *ctx, float *data_out);
```

### Data Types (`components/data_types/`)

```c
void data_types_init(void);
uint32_t get_timestamp_ms(void);
bool metadata_create(const char *filepath, const char *run_id);
bool metadata_update_statistics(const char *filepath, const daq_statistics_t *stats);
bool metadata_finalize(const char *filepath);
```

### SD Storage (`components/sd_storage/`)

```c
bool sd_storage_init(void);
bool sd_create_run_session(void);
bool sd_write_fast_data(const fast_data_record_t *record);
bool sd_write_medium_data(const medium_data_record_t *record);
bool sd_write_slow_data(const slow_data_record_t *record);
bool sd_close_run_session(void);
void sd_storage_deinit(void);
```

## Development Status

**Current State**: Core framework and SD storage are complete. All 7 sensors are integrated into the HAL.

- **Vibration (SW-420)**: individually tested, works
- **Microphone (INMP441)**: individually tested, works
- **Temperature (MCP9808)**: individually tested, works
- **IMU (BNO085)**: under debugging, not working yet
- **Current (ACS723)**: driver implemented, not hardware-tested
- **Pressure (MPL3115A2)**: driver implemented, not hardware-tested
- **Photodiode (751-1015-ND)**: driver implemented, not hardware-tested

## Performance

| Metric | Value |
|--------|-------|
| Max Sample Rate | 1kHz |
| Total Data Rate | ~35 KB/s |
| Queue Latency | < 100ms |
| SD Flush Interval | 100ms or buffer full |

## Configuration

### Sampling Duration

Modify in `main/main_dual_core.c`:

```c
vTaskDelay(pdMS_TO_TICKS(30000));  // Current: 30 seconds
```

### Queue Sizes

Adjust in `components/data_types/include/data_types.h`:

```c
#define FAST_QUEUE_SIZE     200
#define MEDIUM_QUEUE_SIZE   20
#define SLOW_QUEUE_SIZE     10
```

### Buffer Size

Change in `components/sd_storage/include/sd_storage.h`:

```c
#define WRITE_BUFFER_SIZE   4096  // 4KB
```

## Contributing

All sensor drivers are implemented. Current priorities:

1. **Hardware testing**: Test remaining sensors (Current, Pressure, Photodiode) with real hardware
2. **Multi-sensor integration**: Verify concurrent operation of multiple sensors
3. **SD card data loss**: Investigate and fix packet loss during SD card writes
4. **Error handling**: Add recovery mechanisms for sensor failures and SD write errors

---

**Status**: Framework complete; vibration, microphone, and temperature drivers verified individually; IMU not working yet; other drivers pending hardware test.


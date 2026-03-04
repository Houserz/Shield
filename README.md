# Shield Project

## Features

- ✅ **Dual-core Architecture**: Core 0 handles data acquisition, Core 1 handles SD card storage
- ✅ **Multiple Sampling Rates**: Supports 1kHz / 200Hz / 50Hz sampling rates
- ✅ **Hybrid Interfaces**: SPI, I2C, I2S, ADC, and GPIO hardware interfaces
- ✅ **OOP Design**: Sensor abstraction implemented in C with C++ integration
- ✅ **High Performance**: Uses FreeRTOS queues and buffering mechanism
- ✅ **Scalable**: Modular design, easy to add new sensors

## Hardware

**MCU**: ESP32-S3 (Dual-Core Xtensa LX7 @ 240MHz)

**Sensors** (9 sensor channels):
- BNO085 Accelerometer (3-axis, SPI, 1kHz)
- BNO085 Gyroscope (3-axis, SPI, 1kHz)
- BNO085 Magnetometer (3-axis, SPI, 1kHz)
- SW-420 (Vibration, GPIO, 1kHz)
- ACS723 (Current, ADC via ADS1115, 200Hz)
- MPL3115A2 (Pressure, I2C, 50Hz)
- MCP9808 (Temperature, I2C, 50Hz)
- INMP441 (Microphone, I2S, 1kHz effective)
- 751-1015-ND (Photodiode, ADC via ADS1115, 200Hz)

**Storage**: SD Card (SDMMC 4-bit, 40MHz)

## Project Structure

```
├── main/
│   └── main_dual_core.cpp        # Main program (dual-core, C++)
│
└── components/
    ├── sensor_hal/               # Hardware abstraction layer
    ├── drivers/                  # Sensor drivers (11 driver files)
    ├── esp32_BNO08x/             # BNO085 library (C++)
    ├── data_types/               # Data structures & metadata
    └── sd_storage/               # SD card storage module
```

## Data Flow

```
Sensors → Drivers → FreeRTOS Queues (Core 0 → Core 1) → SD Card
         1kHz/200Hz/50Hz         Buffered Write          Binary Files

Fast (1kHz):    Accelerometer, Gyroscope, Magnetometer, Vibration, Microphone
Medium (200Hz): Current, Photodiode
Slow (50Hz):    Pressure, Temperature
```

## Output Format

Each run session creates a directory with:

```
/sdcard/RUN_XXX/
├── fast_data.bin       # 1kHz data (Accel + Gyro + Mag + Vibration + Mic)
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

Edit `main/main_dual_core.cpp` to set actual pin configurations:

```cpp
// BNO085 - SPI Configuration
// CS: GPIO37, SCLK: GPIO38, MOSI: GPIO40, MISO: GPIO39, INT: GPIO5, RST: GPIO6
static bno08x_config_t bno085_spi_cfg = {
    .io_mosi = GPIO_NUM_40,
    .io_miso = GPIO_NUM_39,
    .io_sclk = GPIO_NUM_38,
    .io_cs   = GPIO_NUM_37,
    .io_int  = GPIO_NUM_5,
    .io_rst  = GPIO_NUM_6,
    .spi_peripheral = SPI3_HOST
};

// SW-420 Vibration - GPIO Configuration
static vibration_gpio_config_t vibration_gpio_cfg = {
    .gpio_pin = 16
};

// I2C Bus (shared by MPL3115A2 and MCP9808)
// SDA: GPIO17, SCL: GPIO18
static hal_i2c_config_t mpl3115_i2c_cfg = {
    .i2c_port = 0,
    .sda_pin = 17,
    .scl_pin = 18,
    .device_addr = 0x60
};

// INMP441 Microphone - I2S Configuration
static inmp441_i2s_config_t inmp441_i2s_cfg = {
    .i2s_port = 0,
    .bck_pin = 45,
    .ws_pin = 47,
    .data_in_pin = 48,
    .sample_rate_hz = 16000
};
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
    bool enabled;                   // true = active, false = skip
    void *hw_config;                // Polymorphic config
    bool (*init)(struct SensorContext *ctx);      // Virtual init
    bool (*read_sample)(struct SensorContext *ctx, float *data_out);
} SensorContext_t;
```

Example sensor configuration:

```cpp
static SensorContext_t my_sensors[NUM_SENSORS] = {
    {
        .id = 9,
        .type = SENSOR_TYPE_ACCELEROMETER,
        .sampling_rate_hz = 1000,
        .enabled = true,
        .hw_config = &bno085_imu,
        .init = accel_init,
        .read_sample = accel_read_sample
    },
    // ... more sensors
};
```

### Task Pinning

Tasks are pinned to specific cores for optimal performance:

- **Core 0** (PRO_CPU): Fast (P=10), Medium (P=8), Slow (P=6) acquisition tasks
- **Core 1** (APP_CPU): SD Writer (P=5) storage task

### Buffered Writing

4KB buffers with periodic flushing (every 100ms) ensure efficient SD card writes.

### BNO085 Integration

The BNO085 IMU is integrated via the `esp32_BNO08x` C++ library, providing three separate sensor channels (accelerometer, gyroscope, magnetometer) that share the same SPI interface but are treated as independent sensors in the data pipeline.

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

## Performance

| Metric | Value |
|--------|-------|
| Max Sample Rate | 1kHz |
| Sensor Channels | 9 (3 IMU + 6 others) |
| Total Data Rate | ~45 KB/s |
| Queue Latency | < 100ms |
| SD Flush Interval | 100ms or buffer full |

## Configuration

### Sampling Duration

Modify in `main/main_dual_core.cpp`:

```cpp
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

## Development Status

**Current State**: Core framework and SD storage are complete. All 9 sensor channels are integrated into the HAL. All sensors have been tested and worked well together.

**Known Issues**:
- **BNO085 sampling frequncy**


## Contributing

All sensor drivers are implemented.


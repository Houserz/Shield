#ifndef SENSOR_HAL_H
#define SENSOR_HAL_H

#include <stdbool.h>
#include <stdint.h>

// Sensor type enumeration
typedef enum {
    SENSOR_TYPE_IMU,
    SENSOR_TYPE_TEMP,
    SENSOR_TYPE_PRESSURE,
    SENSOR_TYPE_VIBRATION,
    SENSOR_TYPE_CURRENT,
    SENSOR_TYPE_MICROPHONE,
    SENSOR_TYPE_PHOTODIODE
} sensor_type_t;

// Sensor context structure (implements OOP polymorphism)
typedef struct SensorContext {
    int id;
    sensor_type_t type;
    int sampling_rate_hz; // 1000, 200, or 50
    bool enabled;         // true = active, false = skip init and acquisition
    void *hw_config;      // Pointer to specific SPI/I2C/ADC config structure
    
    // Virtual function: initialize
    bool (*init)(struct SensorContext *ctx);
    // Virtual function: read sample data
    bool (*read_sample)(struct SensorContext *ctx, float *data_out);
} SensorContext_t;

// ==================== Hardware Configuration Structures ====================

// SPI configuration (for BNO085 IMU)
// BNO085 SPI requires INT and RST pins per Adafruit datasheet
typedef struct {
    int spi_host;
    int cs_pin;
    int sclk_pin;
    int mosi_pin;
    int miso_pin;
    int int_pin;   // INT - Data ready, active low (required for SPI)
    int rst_pin;   // RST - Reset, active low (required for SPI)
} spi_config_t;

// I2C configuration (for MPL3115A2 and MCP9808)
// 注意：名称使用 hal_i2c_config_t 以避免与 ESP-IDF 自带的 i2c_config_t 冲突
typedef struct {
    int i2c_port;
    int sda_pin;
    int scl_pin;
    uint8_t device_addr;
} hal_i2c_config_t;

// ADC configuration (for ACS723 current sensor)
typedef struct {
    int adc_channel;
    int gpio_pin;
} adc_config_t;

// GPIO configuration (for SW-420 vibration sensor)
// Note: Renamed to avoid conflict with ESP-IDF's gpio_config_t
typedef struct {
    int gpio_pin;
} vibration_gpio_config_t;

// I2S configuration (for INMP441 microphone)
// Named inmp441_i2s_config_t to avoid clash with ESP-IDF i2s_config_t
typedef struct {
    int i2s_port;       // I2S port (e.g. I2S_NUM_0)
    int bck_pin;        // Bit clock (BCK) GPIO
    int ws_pin;         // Word select (WS/LRCLK) GPIO
    int data_in_pin;    // Data input (SD) GPIO
    int sample_rate_hz; // I2S sample rate (e.g. 16000 for INMP441)
} inmp441_i2s_config_t;

// ==================== Driver Function Declarations ====================

// BNO085 IMU (SPI, 1kHz)
bool bno085_init(SensorContext_t *ctx);
bool bno085_read_sample(SensorContext_t *ctx, float *data_out);

// SW-420 Vibration Sensor (GPIO, 1kHz)
bool vibration_init(SensorContext_t *ctx);
bool vibration_read_sample(SensorContext_t *ctx, float *data_out);

// ACS723 Current Sensor (ADC, 200Hz)
bool current_init(SensorContext_t *ctx);
bool current_read_sample(SensorContext_t *ctx, float *data_out);

// MPL3115A2 Pressure Sensor (I2C, 50Hz)
bool mpl3115_init(SensorContext_t *ctx);
bool mpl3115_read_sample(SensorContext_t *ctx, float *data_out);

// MCP9808 Temperature Sensor (I2C, 50Hz)
bool mcp9808_init(SensorContext_t *ctx);
bool mcp9808_read_sample(SensorContext_t *ctx, float *data_out);

// INMP441 Microphone (I2S, 1kHz high-rate)
bool inmp441_init(SensorContext_t *ctx);
bool inmp441_read_sample(SensorContext_t *ctx, float *data_out);

// BPW34 Photodiode (Vishay PIN, ADC, 200 Hz medium-rate)
bool photodiode_init(SensorContext_t *ctx);
bool photodiode_read_sample(SensorContext_t *ctx, float *data_out);

#endif // SENSOR_HAL_H


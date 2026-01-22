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
    SENSOR_TYPE_CURRENT
} sensor_type_t;

// Sensor context structure (implements OOP polymorphism)
typedef struct SensorContext {
    int id;
    sensor_type_t type;
    int sampling_rate_hz; // 1000, 200, or 50
    void *hw_config;      // Pointer to specific SPI/I2C/ADC config structure
    
    // Virtual function: initialize
    bool (*init)(struct SensorContext *ctx);
    // Virtual function: read sample data
    bool (*read_sample)(struct SensorContext *ctx, float *data_out);
} SensorContext_t;

// ==================== Hardware Configuration Structures ====================

// SPI configuration (for BNO085 IMU)
typedef struct {
    int spi_host;
    int cs_pin;
    int sclk_pin;
    int mosi_pin;
    int miso_pin;
} spi_config_t;

// I2C configuration (for MPL3115A2 and MCP9808)
typedef struct {
    int i2c_port;
    int sda_pin;
    int scl_pin;
    uint8_t device_addr;
} i2c_config_t;

// ADC configuration (for ACS723 current sensor)
typedef struct {
    int adc_channel;
    int gpio_pin;
} adc_config_t;

// GPIO configuration (for SW-420 vibration sensor)
typedef struct {
    int gpio_pin;
} gpio_config_t;

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

#endif // SENSOR_HAL_H


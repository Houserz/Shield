#ifndef SENSOR_HAL_H
#define SENSOR_HAL_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c.h"

// Sensor type enumeration
typedef enum {
    SENSOR_TYPE_IMU,
    SENSOR_TYPE_TEMP,
    SENSOR_TYPE_PRESSURE,
    SENSOR_TYPE_VIBRATION,
    SENSOR_TYPE_CURRENT,
    SENSOR_TYPE_MICROPHONE,
    SENSOR_TYPE_PHOTODIODE,
    SENSOR_TYPE_MAGNETOMETER,
    SENSOR_TYPE_GYROSCOPE,
    SENSOR_TYPE_ACCELEROMETER
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
typedef struct {
    int spi_host;
    int cs_pin;
    int sclk_pin;
    int mosi_pin;
    int miso_pin;
    int int_pin;
    int rst_pin;
} spi_config_t;

// I2C configuration (for MPL3115A2 and MCP9808)
typedef struct {
    int i2c_port;
    int sda_pin;
    int scl_pin;
    uint8_t device_addr;
} hal_i2c_config_t;

// ADC configuration (for internal ESP32 ADC)
typedef struct {
    int adc_channel;
    int gpio_pin;
} adc_config_t;

// GPIO configuration (for SW-420 vibration sensor)
typedef struct {
    int gpio_pin;
} vibration_gpio_config_t;

// I2S configuration (for INMP441 microphone)
typedef struct {
    int i2s_port;
    int bck_pin;
    int ws_pin;
    int data_in_pin;
    int sample_rate_hz;
} inmp441_i2s_config_t;

// ==================== ADS1115 ====================

#define ADS1115_I2C_ADDR    0x48
#define ADS1115_CONV_REG    0x00
#define ADS1115_CONFIG_REG  0x01

#define ADS1115_PGA_6144MV  0x0000
#define ADS1115_PGA_4096MV  0x0200
#define ADS1115_PGA_2048MV  0x0400
#define ADS1115_PGA_1024MV  0x0600
#define ADS1115_PGA_512MV   0x0800
#define ADS1115_PGA_256MV   0x0A00

#define ADS1115_DR_128SPS   0x0080
#define ADS1115_DR_250SPS   0x00A0
#define ADS1115_DR_860SPS   0x00E0

#define ADS1115_CH0         0x4000
#define ADS1115_CH1         0x5000
#define ADS1115_CH2         0x6000
#define ADS1115_CH3         0x7000

#define ADS1115_OS_SINGLE   0x8000
#define ADS1115_MODE_SINGLE 0x0100

typedef struct {
    i2c_port_t i2c_port;
    uint8_t    i2c_addr;
    uint16_t   pga;
    uint16_t   data_rate;
} ads1115_config_t;

bool ads1115_read_voltage(const ads1115_config_t *cfg, uint16_t channel, float *voltage_out);

// ==================== Driver Function Declarations ====================

// BNO085 IMU (SPI, 1kHz)
bool bno085_init(SensorContext_t *ctx);
bool bno085_read_sample(SensorContext_t *ctx, float *data_out);

// SW-420 Vibration Sensor (GPIO, 1kHz)
bool vibration_init(SensorContext_t *ctx);
bool vibration_read_sample(SensorContext_t *ctx, float *data_out);

// ACS723 Current Sensor (ADS1115 A1, 200Hz)
bool current_init(SensorContext_t *ctx);
bool current_read_sample(SensorContext_t *ctx, float *data_out);

// MPL3115A2 Pressure Sensor (I2C, 50Hz)
bool mpl3115_init(SensorContext_t *ctx);
bool mpl3115_read_sample(SensorContext_t *ctx, float *data_out);

// MCP9808 Temperature Sensor (I2C, 50Hz)
bool mcp9808_init(SensorContext_t *ctx);
bool mcp9808_read_sample(SensorContext_t *ctx, float *data_out);

// INMP441 Microphone (I2S, 1kHz)
bool inmp441_init(SensorContext_t *ctx);
bool inmp441_read_sample(SensorContext_t *ctx, float *data_out);

// BPW34 Photodiode (ADS1115 A0, 200Hz)
bool photodiode_init(SensorContext_t *ctx);
bool photodiode_read_sample(SensorContext_t *ctx, float *data_out);

#endif // SENSOR_HAL_H
/**
 * @file driver_mcp9808.c
 * @brief MCP9808 Temperature Sensor Driver (I2C interface, 50Hz sampling rate)
 *
 * Uses the MCP9808 official datasheet and Adafruit MCP9808 usage guide:
 *  - I2C address: default 0x18, configurable 0x18–0x1F (determined by A0/A1/A2)
 *  - Temperature register TA: 0x05, 13-bit signed temperature, resolution 0.0625°C
 *  - Manufacturer ID: register 0x06, value should be 0x0054
 *  - Device ID/version: register 0x07, value should be 0x0400
 *  - Resolution register: 0x08, recommended value 0x03 (0.0625°C, Adafruit default)
 *
 * Hardware interface: I2C (shares I2C_NUM_0 bus with MPL3115A2)
 * Datasheet: https://www.microchip.com/en-us/product/MCP9808
 */

#include "sensor_hal.h"

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include <math.h>
#include <string.h>

// ==================== MCP9808 Registers and Constants ====================

#define MCP9808_REG_CONFIG        0x01
#define MCP9808_REG_UPPER_TEMP    0x02
#define MCP9808_REG_LOWER_TEMP    0x03
#define MCP9808_REG_CRIT_TEMP     0x04
#define MCP9808_REG_AMBIENT_TEMP  0x05
#define MCP9808_REG_MANUF_ID      0x06
#define MCP9808_REG_DEVICE_ID     0x07
#define MCP9808_REG_RESOLUTION    0x08

// Resolution setting (lowest 2 bits), 0x03 -> 0.0625°C, Adafruit example default config
#define MCP9808_RESOLUTION_0_0625C 0x03

// Expected Manufacturer ID / Device ID (per datasheet & Adafruit library)
#define MCP9808_EXPECTED_MANUF_ID  0x0054
#define MCP9808_EXPECTED_DEVICE_ID 0x0400

// Sample resolution: each LSB = 0.0625°C
#define MCP9808_TEMP_LSB_C        0.0625f

static const char *TAG = "mcp9808";

// To share I2C bus with MPL3115, only configure and install I2C driver on first initialization
// Support for up to I2C_NUM_0 / I2C_NUM_1, indexed by i2c_port
static bool s_i2c_initialized[2] = {false, false};

// ==================== I2C Utility Functions ====================

// Write 16-bit register (big endian: high byte first)
static esp_err_t mcp9808_write_u16(i2c_port_t port, uint8_t dev_addr,
                                   uint8_t reg, uint16_t value) {
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (uint8_t)((value >> 8) & 0xFF);
    buf[2] = (uint8_t)(value & 0xFF);

    return i2c_master_write_to_device(port, dev_addr, buf, sizeof(buf),
                                      pdMS_TO_TICKS(100));
}

// Read 16-bit register (big endian)
static esp_err_t mcp9808_read_u16(i2c_port_t port, uint8_t dev_addr,
                                  uint8_t reg, uint16_t *out_value) {
    uint8_t data[2] = {0};
    esp_err_t err = i2c_master_write_read_device(port, dev_addr,
                                                 &reg, 1,
                                                 data, sizeof(data),
                                                 pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        return err;
    }

    *out_value = ((uint16_t)data[0] << 8) | data[1];
    return ESP_OK;
}

// Read register of arbitrary length (used for 2-byte TA register)
static esp_err_t mcp9808_read_bytes(i2c_port_t port, uint8_t dev_addr,
                                    uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_write_read_device(port, dev_addr,
                                        &reg, 1,
                                        buf, len,
                                        pdMS_TO_TICKS(100));
}

// ==================== Public API: Initialization ====================

/**
 * @brief Initialize the MCP9808 temperature sensor
 *
 * Steps:
 * 1. Configure I2C bus and pins according to ctx->hw_config
 * 2. Initialize I2C master mode (shares I2C_NUM_0 with MPL3115)
 * 3. Read manufacturer ID and device ID to confirm MCP9808 presence
 * 4. Set resolution register to 0x03 (0.0625°C)
 * 5. Ensure operating in continuous conversion mode (CONFIG register default 0x0000 means continuous)
 *
 * @param ctx Sensor context
 * @return true if initialization succeeded, false otherwise
 */
bool mcp9808_init(SensorContext_t *ctx) {
    if (ctx == NULL || ctx->hw_config == NULL) {
        return false;
    }

    hal_i2c_config_t *cfg = (hal_i2c_config_t *)ctx->hw_config;
    i2c_port_t port = (i2c_port_t)cfg->i2c_port;
    uint8_t addr = cfg->device_addr;  // Should default to 0x18, user can reconfigure

    if (port < I2C_NUM_0 || port > I2C_NUM_1) {
        ESP_LOGE(TAG, "Invalid I2C port: %d", (int)port);
        return false;
    }

    if (!s_i2c_initialized[port]) {
        i2c_config_t idf_i2c_cfg = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = cfg->sda_pin,
            .scl_io_num = cfg->scl_pin,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 400000,  // 400kHz, supported by MCP9808 & Adafruit example
            .clk_flags = 0,
        };

        esp_err_t err = i2c_param_config(port, &idf_i2c_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2c_param_config failed: %d", err);
            return false;
        }

        err = i2c_driver_install(port, idf_i2c_cfg.mode, 0, 0, 0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            // If already installed, returns ESP_ERR_INVALID_STATE, can ignore
            ESP_LOGE(TAG, "i2c_driver_install failed: %d", err);
            return false;
        }

        s_i2c_initialized[port] = true;
        ESP_LOGI(TAG, "I2C port %d initialized for MCP9808 (SDA=%d SCL=%d addr=0x%02X)",
                 (int)port, cfg->sda_pin, cfg->scl_pin, addr);
    }

    // ========== 3. Detect MCP9808 device ==========
    uint16_t manuf_id = 0;
    uint16_t device_id = 0;

    esp_err_t err = mcp9808_read_u16(port, addr, MCP9808_REG_MANUF_ID, &manuf_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MCP9808 MANUF_ID (err=%d)", err);
        return false;
    }

    err = mcp9808_read_u16(port, addr, MCP9808_REG_DEVICE_ID, &device_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MCP9808 DEVICE_ID (err=%d)", err);
        return false;
    }

    if (manuf_id != MCP9808_EXPECTED_MANUF_ID || device_id != MCP9808_EXPECTED_DEVICE_ID) {
        ESP_LOGE(TAG, "MCP9808 ID mismatch: manuf=0x%04X (exp 0x%04X), dev=0x%04X (exp 0x%04X)",
                 manuf_id, MCP9808_EXPECTED_MANUF_ID,
                 device_id, MCP9808_EXPECTED_DEVICE_ID);
        return false;
    }

    ESP_LOGI(TAG, "MCP9808 detected OK (manuf=0x%04X dev=0x%04X)", manuf_id, device_id);

    // ========== 4. Set resolution register (0.0625°C) ==========
    uint8_t res_value = MCP9808_RESOLUTION_0_0625C;
    err = i2c_master_write_to_device(port, addr, (uint8_t[]){MCP9808_REG_RESOLUTION, res_value},
                                     2, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write RESOLUTION register (err=%d)", err);
        return false;
    }

    // ========== 5. Set to continuous conversion mode ==========
    // CONFIG register bit8 (SHDN) = 0 means continuous conversion. Default on power-up is 0x0000,
    // explicitly write it here for consistency.
    err = mcp9808_write_u16(port, addr, MCP9808_REG_CONFIG, 0x0000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CONFIG register (err=%d)", err);
        return false;
    }

    ESP_LOGI(TAG, "MCP9808 init OK (resolution=0.0625C, continuous mode)");
    return true;
}

// ==================== Public API: Read Temperature ====================

/**
 * @brief Read temperature data
 *
 * Steps:
 * 1. Use I2C to read register TA (0x05, 2 bytes)
 * 2. According to the MCP9808 datasheet, parse 13-bit temperature:
 *    - Bit[12] is the sign bit, 1 = negative temperature
 *    - Bit[11:0] is absolute value, unit is 0.0625°C
 *    - If sign bit is 1: temp_raw = (value & 0x0FFF) - 8192
 *      else: temp_raw = (value & 0x0FFF)
 *    - The final Celsius temperature: temp_C = temp_raw * 0.0625
 *
 * @param ctx       Sensor context
 * @param data_out  Output temperature value (unit: degrees Celsius °C)
 * @return true on successful read, false on failure
 */
bool mcp9808_read_sample(SensorContext_t *ctx, float *data_out) {
    if (ctx == NULL || ctx->hw_config == NULL || data_out == NULL) {
        return false;
    }

    hal_i2c_config_t *cfg = (hal_i2c_config_t *)ctx->hw_config;
    i2c_port_t port = (i2c_port_t)cfg->i2c_port;
    uint8_t addr = cfg->device_addr;

    uint8_t buf[2] = {0};
    esp_err_t err = mcp9808_read_bytes(port, addr, MCP9808_REG_AMBIENT_TEMP, buf, sizeof(buf));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature (err=%d)", err);
        return false;
    }

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];

    // Decode 13-bit temperature per MCP9808 datasheet
    int16_t temp_raw;
    if (raw & 0x1000) {
        // Negative temperature: first get 13 bits, then subtract 2^13
        temp_raw = (int16_t)((raw & 0x1FFF) - 8192);
    } else {
        temp_raw = (int16_t)(raw & 0x0FFF);
    }

    *data_out = (float)temp_raw * MCP9808_TEMP_LSB_C;
    return true;
}

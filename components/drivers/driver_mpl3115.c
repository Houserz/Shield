/**
 * @file driver_mpl3115.c
 * @brief MPL3115A2 Pressure Sensor Driver (I2C, 50 Hz sampling)
 *
 * Implementation per SparkFun MPL3115A2 Hookup Guide and NXP MPL3115A2 datasheet:
 *  - I2C address: 0x60 (7-bit)
 *  - WHO_AM_I register 0x0C, value 0xC4
 *  - Barometer mode: CTRL_REG1 (0x26) ALT=0; setModeStandby() before changing CTRL1, then setModeActive()
 *  - setOversampleRate(0..7) → 1..128 samples; datasheet recommends 128 (OSR=7), 512 ms per reading
 *  - enableEventFlags(): PT_DATA_CFG (0x13) = 0x07 (required during setup)
 *  - Pressure: read OUT_P_MSB (0x01), OUT_P_CSB (0x02), OUT_P_LSB (0x03); 20-bit, resolution 0.25 Pa
 *  - readPressure() returns barometric pressure in Pa (output converted to kPa in this driver)
 *  - Uses I2C repeated start; poll STATUS (0x00) for data ready (bit 0x08) then read OUT_P.
 *
 * Hardware: I2C (shared bus with MCP9808). Typical accuracy ±0.05 kPa, altitude ±0.3 m, temp ±3 °C.
 */

#include "sensor_hal.h"

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

// ==================== MPL3115A2 Register Map (per NXP datasheet) ====================

#define MPL3115_REG_STATUS       0x00
#define MPL3115_REG_OUT_P_MSB    0x01
#define MPL3115_REG_OUT_P_CSB    0x02
#define MPL3115_REG_OUT_P_LSB    0x03
#define MPL3115_REG_OUT_T_MSB    0x04
#define MPL3115_REG_OUT_T_LSB    0x05
#define MPL3115_REG_PT_DATA_CFG  0x13
#define MPL3115_REG_CTRL_REG1    0x26
#define MPL3115_REG_WHO_AM_I     0x0C

// STATUS: Pressure data ready (poll this before reading OUT_P)
#define MPL3115_STATUS_PDR       (1u << 3)  // 0x08

// CTRL_REG1: SBYB=0 standby, SBYB=1 active; ALT=0 barometer, ALT=1 altimeter; OSR[2:0] oversample
#define MPL3115_CTRL_SBYB        (1u << 0)
#define MPL3115_CTRL_OST         (1u << 1)
#define MPL3115_CTRL_ALT         (1u << 7)
#define MPL3115_CTRL_OSR_MASK    0x38
#define MPL3115_CTRL_OSR_SHIFT   3
// OSR 0..7 → 2^0 .. 2^7 = 1, 2, 4, 8, 16, 32, 64, 128
#define MPL3115_OSR_128          7

// PT_DATA_CFG: enable event flags (required during setup per hookup guide)
#define MPL3115_PT_DATA_CFG_TDEFE  (1u << 0)
#define MPL3115_PT_DATA_CFG_PDEFE  (1u << 1)
#define MPL3115_PT_DATA_CFG_DREM   (1u << 2)
#define MPL3115_PT_DATA_CFG_ENABLE 0x07

#define MPL3115_WHO_AM_I_EXPECTED  0xC4

// Pressure: 20-bit, 1 LSB = 0.25 Pa (datasheet Section 9.3, barometer resolution 0.25 Pa)
#define MPL3115_PA_PER_COUNT  0.25f

static const char *TAG = "mpl3115";

// Share I2C bus with MCP9808: only install driver once per port
static bool s_i2c_initialized[2] = {false, false};

// ==================== I2C Helpers ====================

static esp_err_t mpl3115_write_reg(i2c_port_t port, uint8_t dev_addr, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    return i2c_master_write_to_device(port, dev_addr, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

static esp_err_t mpl3115_read_reg(i2c_port_t port, uint8_t dev_addr, uint8_t reg, uint8_t *value) {
    return i2c_master_write_read_device(port, dev_addr, &reg, 1, value, 1, pdMS_TO_TICKS(100));
}

static esp_err_t mpl3115_read_bytes(i2c_port_t port, uint8_t dev_addr, uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_write_read_device(port, dev_addr, &reg, 1, buf, len, pdMS_TO_TICKS(100));
}

// ==================== Public API: Initialization ====================

/**
 * @brief Initialize MPL3115A2 pressure sensor
 *
 * Per Hookup Guide and datasheet:
 * 1. Configure I2C and install master (shared with MCP9808).
 * 2. Read WHO_AM_I (0x0C), expect 0xC4.
 * 3. setModeStandby() then set CTRL_REG1: barometer (ALT=0), OSR=128.
 * 4. enableEventFlags() → PT_DATA_CFG = 0x07.
 * 5. setModeActive() → start measurements.
 */
bool mpl3115_init(SensorContext_t *ctx) {
    if (ctx == NULL || ctx->hw_config == NULL) {
        return false;
    }

    hal_i2c_config_t *cfg = (hal_i2c_config_t *)ctx->hw_config;
    i2c_port_t port = (i2c_port_t)cfg->i2c_port;
    uint8_t addr = cfg->device_addr;

    if (port < I2C_NUM_0 || port > I2C_NUM_1) {
        ESP_LOGE(TAG, "Invalid I2C port: %d", (int)port);
        return false;
    }

    if (!s_i2c_initialized[port]) {
        i2c_config_t idf_cfg = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = cfg->sda_pin,
            .scl_io_num = cfg->scl_pin,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 400000,
            .clk_flags = 0,
        };

        esp_err_t err = i2c_param_config(port, &idf_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2c_param_config failed: %d", err);
            return false;
        }

        err = i2c_driver_install(port, idf_cfg.mode, 0, 0, 0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "i2c_driver_install failed: %d", err);
            return false;
        }
        s_i2c_initialized[port] = true;
        ESP_LOGI(TAG, "I2C port %d initialized (SDA=%d SCL=%d addr=0x%02X)",
                 (int)port, cfg->sda_pin, cfg->scl_pin, addr);
    }

    // 2. Detect device: WHO_AM_I = 0xC4
    uint8_t whoami = 0;
    esp_err_t err = mpl3115_read_reg(port, addr, MPL3115_REG_WHO_AM_I, &whoami);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I (err=%d)", err);
        return false;
    }
    if (whoami != MPL3115_WHO_AM_I_EXPECTED) {
        ESP_LOGE(TAG, "WHO_AM_I=0x%02X, expected 0xC4", whoami);
        return false;
    }
    ESP_LOGI(TAG, "MPL3115A2 detected (WHO_AM_I=0x%02X)", whoami);

    // 3. setModeStandby() then configure CTRL_REG1 (barometer, OSR=128)
    uint8_t ctrl = (MPL3115_OSR_128 << MPL3115_CTRL_OSR_SHIFT);  // ALT=0, SBYB=0
    err = mpl3115_write_reg(port, addr, MPL3115_REG_CTRL_REG1, ctrl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set CTRL_REG1 standby (err=%d)", err);
        return false;
    }

    // 4. enableEventFlags() — PT_DATA_CFG = 0x07 (required during setup)
    err = mpl3115_write_reg(port, addr, MPL3115_REG_PT_DATA_CFG, MPL3115_PT_DATA_CFG_ENABLE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PT_DATA_CFG (err=%d)", err);
        return false;
    }

    // 5. setModeActive() — SBYB=1
    ctrl |= MPL3115_CTRL_SBYB;
    err = mpl3115_write_reg(port, addr, MPL3115_REG_CTRL_REG1, ctrl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set active mode (err=%d)", err);
        return false;
    }

    ESP_LOGI(TAG, "MPL3115A2 init OK (barometer, OSR=128)");
    return true;
}

// ==================== Public API: Read Pressure ====================

/**
 * @brief Read pressure sample
 *
 * Per Hookup Guide: poll STATUS for data ready (PDR), then read OUT_P_MSB, OUT_P_CSB, OUT_P_LSB.
 * 20-bit value, resolution 0.25 Pa (datasheet). Output in kPa.
 * Hookup guide: -999 indicates I2C timeout (512 ms max); we return false on I2C error.
 */
bool mpl3115_read_sample(SensorContext_t *ctx, float *data_out) {
    if (ctx == NULL || ctx->hw_config == NULL || data_out == NULL) {
        return false;
    }

    hal_i2c_config_t *cfg = (hal_i2c_config_t *)ctx->hw_config;
    i2c_port_t port = (i2c_port_t)cfg->i2c_port;
    uint8_t addr = cfg->device_addr;

    // Wait for pressure data ready (PDR). Datasheet Table 6: first conversion from STANDBY->ACTIVE
    // can take up to 1000 ms at OSR=128; subsequent conversions ~512 ms. Use 1500 ms timeout.
    const int max_wait_ms = 1500;
    int waited = 0;
    uint8_t status = 0;
    while (waited < max_wait_ms) {
        esp_err_t err = mpl3115_read_reg(port, addr, MPL3115_REG_STATUS, &status);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read STATUS (err=%d)", err);
            return false;
        }
        if (status & MPL3115_STATUS_PDR) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        waited += 10;
    }
    if (!(status & MPL3115_STATUS_PDR)) {
        ESP_LOGE(TAG, "Pressure data ready timeout");
        return false;
    }

    uint8_t p_buf[3];
    esp_err_t err = mpl3115_read_bytes(port, addr, MPL3115_REG_OUT_P_MSB, p_buf, sizeof(p_buf));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read OUT_P (err=%d)", err);
        return false;
    }

    // 20-bit pressure: MSB.CSB.LSB top 4 bits of LSB are fractional (datasheet 0.25 Pa per LSB)
    uint32_t raw = ((uint32_t)p_buf[0] << 12) | ((uint32_t)p_buf[1] << 4) | (p_buf[2] >> 4);
    float pa = (float)raw * MPL3115_PA_PER_COUNT;
    *data_out = pa / 1000.0f;  // kPa

    // Per datasheet Figure 7 (polling): after read, "Set Active" again to trigger next conversion.
    uint8_t ctrl = (MPL3115_OSR_128 << MPL3115_CTRL_OSR_SHIFT) | MPL3115_CTRL_SBYB;
    (void)mpl3115_write_reg(port, addr, MPL3115_REG_CTRL_REG1, ctrl);

    return true;
}

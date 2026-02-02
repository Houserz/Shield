/**
 * @file driver_mcp9808.c
 * @brief MCP9808 Temperature Sensor Driver (I2C interface, 50Hz sampling rate)
 * 
 * TODO: Implement real MCP9808 hardware driver
 * Hardware interface: I2C
 * Datasheet: https://www.microchip.com/en-us/product/MCP9808
 */

#include "sensor_hal.h"
#include <math.h>

/**
 * @brief Initialize MCP9808 temperature sensor
 * 
 * TODO: Implement the following functions
 * 1. Configure I2C bus and pins (use configuration in ctx->hw_config)
 * 2. Initialize I2C Master mode (shares same I2C bus with MPL3115)
 * 3. Detect MCP9808 device (read Manufacturer ID: 0x0054, Device ID: 0x0400)
 * 4. Configure measurement resolution (RESOLUTION register, recommended 0.0625°C)
 * 5. Set to continuous conversion mode
 * 
 * @param ctx Sensor context
 * @return true=initialization successful, false=initialization failed
 */
bool mcp9808_init(SensorContext_t *ctx) {
    // TODO: Implement I2C initialization and MCP9808 configuration
    // i2c_config_t *i2c_cfg = (i2c_config_t *)ctx->hw_config;
    // Note: This sensor shares I2C_NUM_0 bus with MPL3115
    // RESOLUTION register: 0x08
    
    (void)ctx;
    return true;
}

/**
 * @brief Read temperature sensor data
 * 
 * TODO: Implement the following functions
 * 1. Read temperature register via I2C (TA register: 0x05)
 * 2. Parse 13-bit temperature data (sign bit + 12-bit temperature)
 * 3. Convert to Celsius temperature value
 *    - Bit 12: Sign bit (1=negative temperature)
 *    - Bit 11-0: Temperature value (each bit = 0.0625°C)
 *    - Formula: temp = (raw & 0x0FFF) / 16.0 (positive temperature)
 * 
 * @param ctx Sensor context
 * @param data_out Output temperature value (unit: °C)
 * @return true=read successful, false=read failed
 */
bool mcp9808_read_sample(SensorContext_t *ctx, float *data_out) {
    // TODO: Implement I2C read and temperature calculation
    // TA register address: 0x05 (2 bytes)
    
    (void)ctx;
    *data_out = 0.0f;
    return true;
}

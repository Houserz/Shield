/**
 * @file driver_bno085.c
 * @brief BNO085 IMU Sensor Driver (SPI interface, 1kHz sampling rate)
 *
 * Implements SHTP (Sensor Hub Transport Protocol) per Adafruit BNO085 datasheet.
 * Hardware interface: SPI (requires CS, INT, RST pins)
 * Reference: Adafruit 9-DOF Orientation IMU Fusion Breakout - BNO085
 */

#include "sensor_hal.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <string.h>
#include <math.h>

static const char *TAG = "bno085";

// ==================== SHTP Protocol Constants ====================
#define SHTP_HEADER_LEN        4
#define SHTP_CHANNEL_EXE       1
#define SHTP_CHANNEL_CONTROL   2
#define SHTP_CHANNEL_REPORTS   3

#define SHTP_REPORT_PRODUCT_ID_REQUEST  0xF9
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SET_FEATURE_COMMAND            0xFD
#define GET_FEATURE_RESPONSE           0xFC

#define SH2_REPORTID_ACCELEROMETER      0x01
#define SH2_REPORTID_GAME_ROTATION_VECTOR 0x08

#define BNO085_ACCEL_QPOINT     100.0f   // Q8.2: 1/100 for m/s²
#define BNO085_REPORT_INTERVAL_US  1000  // 1ms for 1kHz sampling

// ==================== Driver State ====================
typedef struct {
    spi_device_handle_t spi_handle;
    spi_config_t *cfg;
    bool initialized;
    uint8_t seq_control;
    uint8_t seq_exe;
    float last_accel_magnitude;  // Last valid reading
} bno085_state_t;

static bno085_state_t s_bno085 = {0};

// ==================== Helper: Build SHTP packet ====================
static void shtp_build_packet(uint8_t channel, uint8_t seq, const uint8_t *payload,
                               uint16_t payload_len, uint8_t *out_buf) {
    uint16_t total_len = payload_len + SHTP_HEADER_LEN;
    out_buf[0] = (uint8_t)(total_len & 0xFF);
    out_buf[1] = (uint8_t)((total_len >> 8) & 0x7F);  // bit 15 = 0 (no continuation)
    out_buf[2] = channel;
    out_buf[3] = seq;
    if (payload_len > 0 && payload != NULL) {
        memcpy(out_buf + SHTP_HEADER_LEN, payload, payload_len);
    }
}

// ==================== SPI Transfer ====================
static bool bno085_spi_transfer(const uint8_t *tx_buf, uint8_t *rx_buf, size_t len) {
    if (!s_bno085.spi_handle || len == 0) return false;

    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    esp_err_t ret = spi_device_polling_transmit(s_bno085.spi_handle, &t);
    return (ret == ESP_OK);
}

// ==================== Read SHTP packet (caller provides buffer) ====================
static int bno085_read_packet(uint8_t *buf, size_t buf_size) {
    if (buf_size < SHTP_HEADER_LEN) return -1;

    uint8_t tx_header[SHTP_HEADER_LEN];
    memset(tx_header, 0xFF, sizeof(tx_header));

    if (!bno085_spi_transfer(tx_header, buf, SHTP_HEADER_LEN)) {
        return -1;
    }

    uint16_t pkt_len = (uint16_t)buf[0] | ((uint16_t)(buf[1] & 0x7F) << 8);
    if (pkt_len < SHTP_HEADER_LEN || pkt_len > buf_size) {
        return -1;
    }

    uint16_t payload_len = pkt_len - SHTP_HEADER_LEN;
    if (payload_len > 0) {
        #define MAX_PAYLOAD 256
        uint8_t tx_payload[MAX_PAYLOAD];
        size_t to_read = (payload_len < MAX_PAYLOAD) ? (size_t)payload_len : MAX_PAYLOAD;
        memset(tx_payload, 0xFF, to_read);
        if (!bno085_spi_transfer(tx_payload, buf + SHTP_HEADER_LEN, to_read)) {
            return -1;
        }
    }

    return (int)pkt_len;
}

// ==================== Send SHTP packet ====================
static bool bno085_send_packet(uint8_t channel, uint8_t *seq, const uint8_t *payload,
                               uint16_t payload_len) {
    uint8_t pkt[64];
    if (payload_len + SHTP_HEADER_LEN > sizeof(pkt)) return false;

    shtp_build_packet(channel, *seq, payload, payload_len, pkt);
    *seq = (*seq + 1) & 0xFF;

    uint8_t rx_dummy[64];
    size_t total = payload_len + SHTP_HEADER_LEN;
    return bno085_spi_transfer(pkt, rx_dummy, total);
}

// ==================== Hardware reset ====================
static void bno085_hw_reset(int rst_pin) {
    if (rst_pin < 0) {
        ESP_LOGW(TAG, "BNO085: RST pin not configured, skipping hw reset");
        vTaskDelay(pdMS_TO_TICKS(350));  // Allow boot time anyway
        return;
    }
    gpio_set_level(rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(350));  // Datasheet: ~300ms boot time
}

// ==================== Wait for INT (data ready) with timeout ====================
static bool bno085_wait_int(int int_pin, uint32_t timeout_ms) {
    if (int_pin < 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
        return true;  // No INT pin: assume data available after short delay
    }
    uint32_t start = xTaskGetTickCount() * portTICK_PERIOD_MS;
    while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start) < timeout_ms) {
        if (gpio_get_level(int_pin) == 0) return true;  // Active low
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

// ==================== Parse accelerometer report ====================
// Format: ReportID(1) + Seq(1) + Status(1) + Delay(1) + X(2) + Y(2) + Z(2) = 10 bytes
// X,Y,Z are int16, Q8.2 -> value/100 = m/s²
static bool parse_accelerometer(const uint8_t *payload, uint16_t len, float *mag_out) {
    if (len < 10 || payload[0] != SH2_REPORTID_ACCELEROMETER) return false;

    int16_t x = (int16_t)(payload[4] | (payload[5] << 8));
    int16_t y = (int16_t)(payload[6] | (payload[7] << 8));
    int16_t z = (int16_t)(payload[8] | (payload[9] << 8));

    float fx = (float)x / BNO085_ACCEL_QPOINT;
    float fy = (float)y / BNO085_ACCEL_QPOINT;
    float fz = (float)z / BNO085_ACCEL_QPOINT;

    *mag_out = sqrtf(fx * fx + fy * fy + fz * fz);
    return true;
}

// ==================== Public API ====================

bool bno085_init(SensorContext_t *ctx) {
    if (ctx == NULL || ctx->hw_config == NULL) return false;

    spi_config_t *cfg = (spi_config_t *)ctx->hw_config;

    // Validate required pins (use 0 as placeholder for "not yet configured")
    if (cfg->cs_pin < 0 || cfg->sclk_pin < 0 || cfg->mosi_pin < 0 || cfg->miso_pin < 0) {
        ESP_LOGW(TAG, "BNO085: Pin config incomplete (cs=%d sclk=%d mosi=%d miso=%d)",
                 cfg->cs_pin, cfg->sclk_pin, cfg->mosi_pin, cfg->miso_pin);
        return false;
    }

    // Configure RST and INT GPIO
    if (cfg->rst_pin >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << cfg->rst_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        gpio_set_level(cfg->rst_pin, 1);
    }

    if (cfg->int_pin >= 0) {
        gpio_config_t io_int = {
            .pin_bit_mask = (1ULL << cfg->int_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_int);
    }

    // Hardware reset
    bno085_hw_reset(cfg->rst_pin);

    // SPI bus init
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = cfg->mosi_pin,
        .miso_io_num = cfg->miso_pin,
        .sclk_io_num = cfg->sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 512,
    };

    spi_host_device_t host = (spi_host_device_t)cfg->spi_host;
    if (spi_bus_initialize(host, &bus_cfg, SPI_DMA_DISABLED) != ESP_OK) {
        ESP_LOGE(TAG, "BNO085: SPI bus init failed");
        return false;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 3 * 1000 * 1000,  // 3 MHz typical for BNO085
        .mode = 0,
        .spics_io_num = cfg->cs_pin,
        .queue_size = 1,
    };

    if (spi_bus_add_device(host, &dev_cfg, &s_bno085.spi_handle) != ESP_OK) {
        ESP_LOGE(TAG, "BNO085: SPI device add failed");
        spi_bus_free(host);
        return false;
    }

    s_bno085.cfg = cfg;
    s_bno085.seq_control = 0;
    s_bno085.seq_exe = 0;
    s_bno085.last_accel_magnitude = 0.0f;

    // Soft reset via EXE channel
    uint8_t reset_cmd = 1;
    bno085_send_packet(SHTP_CHANNEL_EXE, &s_bno085.seq_exe, &reset_cmd, 1);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Request product ID (verify communication)
    uint8_t prod_req[2] = { SHTP_REPORT_PRODUCT_ID_REQUEST, 0 };
    bno085_send_packet(SHTP_CHANNEL_CONTROL, &s_bno085.seq_control, prod_req, 2);

    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t rx_buf[128];
    int n = bno085_read_packet(rx_buf, sizeof(rx_buf));
    if (n < (int)(SHTP_HEADER_LEN + 16)) {
        ESP_LOGW(TAG, "BNO085: No product ID response (n=%d), continuing anyway", n);
    } else if (rx_buf[SHTP_HEADER_LEN] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
        uint32_t part_no = (uint32_t)rx_buf[SHTP_HEADER_LEN + 4] |
                           ((uint32_t)rx_buf[SHTP_HEADER_LEN + 5] << 8) |
                           ((uint32_t)rx_buf[SHTP_HEADER_LEN + 6] << 16) |
                           ((uint32_t)rx_buf[SHTP_HEADER_LEN + 7] << 24);
        ESP_LOGI(TAG, "BNO085: Product ID 0x%08"PRIx32, part_no);
    }

    // Enable accelerometer report @ 1kHz
    uint8_t set_feature[17] = {0};
    set_feature[0] = SET_FEATURE_COMMAND;
    set_feature[1] = SH2_REPORTID_ACCELEROMETER;
    *(uint32_t *)(set_feature + 5) = BNO085_REPORT_INTERVAL_US;
    *(uint32_t *)(set_feature + 13) = 0;

    bno085_send_packet(SHTP_CHANNEL_CONTROL, &s_bno085.seq_control, set_feature, 17);
    vTaskDelay(pdMS_TO_TICKS(50));

    s_bno085.initialized = true;
    ESP_LOGI(TAG, "BNO085: Init OK");
    return true;
}

bool bno085_read_sample(SensorContext_t *ctx, float *data_out) {
    if (ctx == NULL || data_out == NULL || !s_bno085.initialized) return false;

    spi_config_t *cfg = s_bno085.cfg;
    if (cfg == NULL) return false;

    // Wait for data ready (INT pin or short delay)
    bno085_wait_int(cfg->int_pin, 20);

    uint8_t rx_buf[128];
    int n = bno085_read_packet(rx_buf, sizeof(rx_buf));
    if (n < SHTP_HEADER_LEN) {
        *data_out = s_bno085.last_accel_magnitude;  // Return last valid
        return true;  // No new packet, non-fatal
    }

    uint8_t channel = rx_buf[2];
    uint16_t payload_len = n - SHTP_HEADER_LEN;
    uint8_t *payload = rx_buf + SHTP_HEADER_LEN;

    if (channel != SHTP_CHANNEL_REPORTS || payload_len < 10) {
        *data_out = s_bno085.last_accel_magnitude;
        return true;
    }

    // Parse reports (may contain multiple in one packet)
    uint16_t offset = 0;
    while (offset + 10 <= payload_len) {
        uint8_t report_id = payload[offset];
        uint8_t report_len = 10;  // Accelerometer fixed length

        if (report_id == SH2_REPORTID_ACCELEROMETER) {
            float mag;
            if (parse_accelerometer(payload + offset, payload_len - offset, &mag)) {
                s_bno085.last_accel_magnitude = mag;
                *data_out = mag;
                return true;
            }
        }
        offset += report_len;
    }

    *data_out = s_bno085.last_accel_magnitude;
    return true;
}

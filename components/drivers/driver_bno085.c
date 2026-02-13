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
#define SHTP_MAX_PACKET        512   /* BNO085 SHTP packets can exceed 256 bytes */
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
// FIX: Read entire packet in a SINGLE SPI transaction. The old code split header/payload
// into two SPI transactions, causing CS to toggle mid-packet. BNO085 interprets each CS
// assertion as a separate SHTP exchange, so the split read consumed TWO packets per call
// (evidence: sequence numbers incremented by 2 instead of 1 in debug logs).
static int bno085_read_packet(uint8_t *buf, size_t buf_size) {
    if (buf_size < SHTP_HEADER_LEN) return -1;

    // Read the full max packet in ONE SPI transaction (CS stays LOW throughout)
    // TX header strategy depends on init vs runtime phase:
    //   Init phase  (initialized=false): send [04 00 00 00] valid SHTP heartbeat.
    //     BNO085 needs valid host headers to advance its SHTP state machine during
    //     initialization. Sending all-zeros stalls the state machine (confirmed by logs).
    //   Runtime phase (initialized=true): send [00 00 00 00] (no host data).
    //     We only read when INT=LOW (data ready), so BNO085 is already active.
    //     Sending [04 00 00 00] at runtime triggers ch=0 acknowledgment responses,
    //     which re-assert INT and create an infinite read loop (confirmed by logs).
    size_t read_len = (buf_size < SHTP_MAX_PACKET) ? buf_size : SHTP_MAX_PACKET;
    static uint8_t tx_dummy[SHTP_MAX_PACKET];
    memset(tx_dummy, 0x00, read_len);
    if (!s_bno085.initialized) {
        tx_dummy[0] = 0x04;  // Init phase: valid empty SHTP header to drive state machine
    }

    if (!bno085_spi_transfer(tx_dummy, buf, read_len)) {
        return -1;
    }

    uint16_t pkt_len = (uint16_t)buf[0] | ((uint16_t)(buf[1] & 0x7F) << 8);
    if (pkt_len < SHTP_HEADER_LEN || pkt_len > buf_size) {
        return -1;
    }

    return (int)pkt_len;
}

// ==================== Send SHTP packet ====================
// Use actual packet size for the SPI transaction. Since we always drain pending
// data before sending (INT is HIGH), there's no simultaneous BNO085 data to cover.
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

    // SPI bus init BEFORE hardware reset: BNO085's SPI interface needs properly
    // driven SPI lines (CS, SCLK, MOSI) during boot. If these pins are floating
    // (uninitialized) when BNO085 boots, its SPI slave may enter an undefined state,
    // causing all subsequent SPI reads to return invalid data.
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = cfg->mosi_pin,
        .miso_io_num = cfg->miso_pin,
        .sclk_io_num = cfg->sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SHTP_MAX_PACKET,
    };

    spi_host_device_t host = (spi_host_device_t)cfg->spi_host;
    if (spi_bus_initialize(host, &bus_cfg, SPI_DMA_CH_AUTO) != ESP_OK) {
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

    // Retry loop: BNO085 boot timing is variable; production libraries retry up to 3 times.
    // Each attempt does a hardware reset + drain + product ID request.
    // No soft reset needed - hardware reset is sufficient and more reliable.
    uint8_t rx_buf[SHTP_MAX_PACKET];
    uint8_t saved_pid_buf[SHTP_MAX_PACKET];
    int n = -1;
    bool found_product_id = false;
    const int MAX_INIT_ATTEMPTS = 3;

    for (int attempt = 0; attempt < MAX_INIT_ATTEMPTS; attempt++) {
        found_product_id = false;
        n = -1;
        s_bno085.seq_control = 0;
        s_bno085.seq_exe = 0;

        // Hardware reset on every attempt (SPI bus already configured, so BNO085
        // boots with properly driven SPI lines)
        if (attempt > 0) {
            ESP_LOGW(TAG, "BNO085: Init retry %d/%d", attempt + 1, MAX_INIT_ATTEMPTS);
        }
        bno085_hw_reset(cfg->rst_pin);

        // Soft reset via EXE channel. This triggers a clean re-initialization that
        // produces a limited set of init packets (advertisement, reset, init response).
        // Without the soft reset, hardware-reset-only produces a massive FRS data dump
        // (80+ packets) that buries the Product ID response (confirmed by logs).
        uint8_t reset_cmd = 1;
        bno085_send_packet(SHTP_CHANNEL_EXE, &s_bno085.seq_exe, &reset_cmd, 1);

        // Wait for BNO085 to complete soft reset and assert INT (data ready).
        // Boot time is typically ~300ms but can vary. Wait up to 800ms.
        bool int_ready = bno085_wait_int(cfg->int_pin, 800);

        if (!int_ready) {
            continue;
        }

        // Simplified drain + Product ID request loop.
        // Key insight: BNO085 needs CONTINUOUS SPI clock activity to advance its
        // state machine. We must keep doing SPI reads even when INT is HIGH.
        // The [04 00 00 00] heartbeat in read_packet (init phase) provides this.
        //
        // Strategy:
        //   1. Always do a read on every iteration (provides clock cycles)
        //   2. After first valid packet, embed Product ID request in next read's TX buffer
        //      (full 512-byte transaction ensures reliable delivery even when BNO085
        //      has pending data - a separate 6-byte send fails intermittently)
        //   3. Keep reading for response (don't stop when INT goes HIGH)
        //   4. Exit only when: response found, or max iterations reached
        int drain_count = 0;
        bool pid_request_sent = false;
        int idle_count = 0;

        for (int drain_i = 0; drain_i < 80; drain_i++) {
            int pkt_n;

            // Embed Product ID request in the next read transaction (after first valid pkt)
            if (!pid_request_sent && drain_count >= 1) {
                // Build a 512-byte TX buffer with the Product ID request as SHTP packet
                // + zero padding. BNO085 receives our request AND outputs its pending
                // data in the same 512-byte SPI transaction. This is reliable because
                // the full transaction gives both parties enough clock cycles.
                static uint8_t tx_cmd[SHTP_MAX_PACKET];
                memset(tx_cmd, 0x00, SHTP_MAX_PACKET);
                uint8_t prod_payload[2] = { SHTP_REPORT_PRODUCT_ID_REQUEST, 0 };
                shtp_build_packet(SHTP_CHANNEL_CONTROL, s_bno085.seq_control,
                                  prod_payload, 2, tx_cmd);
                s_bno085.seq_control = (s_bno085.seq_control + 1) & 0xFF;

                bno085_spi_transfer(tx_cmd, rx_buf, SHTP_MAX_PACKET);
                pid_request_sent = true;

                // Parse the simultaneously received BNO085 data
                uint16_t rx_len = (uint16_t)rx_buf[0] | ((uint16_t)(rx_buf[1] & 0x7F) << 8);
                pkt_n = (rx_len >= SHTP_HEADER_LEN && rx_len <= SHTP_MAX_PACKET)
                        ? (int)rx_len : -1;

            } else {
                // Normal read with heartbeat (init phase sends [04 00 00 00])
                pkt_n = bno085_read_packet(rx_buf, sizeof(rx_buf));
            }

            if (pkt_n > 0) {
                idle_count = 0;
                drain_count++;
                uint8_t pkt_ch = rx_buf[2];

                // Check for Product ID Response
                if (pkt_ch == SHTP_CHANNEL_CONTROL && pkt_n >= (int)(SHTP_HEADER_LEN + 2)
                    && rx_buf[SHTP_HEADER_LEN] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
                    found_product_id = true;
                    n = pkt_n;
                    memcpy(saved_pid_buf, rx_buf, pkt_n);
                }
            } else {
                idle_count++;
            }

            // Exit conditions
            if (found_product_id && idle_count >= 3) {
                break;
            }
            if (pid_request_sent && idle_count >= 15) {
                break;
            }
            if (!pid_request_sent && idle_count >= 30) {
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(5));
        }

        if (found_product_id && n >= (int)(SHTP_HEADER_LEN + 2)) {
            break;  // Success!
        }
    }

    if (!found_product_id || n < (int)(SHTP_HEADER_LEN + 2)) {
        ESP_LOGE(TAG, "BNO085: No product ID after %d attempts - init FAILED",
                 MAX_INIT_ATTEMPTS);
        spi_bus_remove_device(s_bno085.spi_handle);
        spi_bus_free(host);
        return false;
    }

    // Parse and log product ID (use saved_pid_buf since rx_buf may have been overwritten by drain)
    if (n >= (int)(SHTP_HEADER_LEN + 8)) {
        uint32_t part_no = (uint32_t)saved_pid_buf[SHTP_HEADER_LEN + 4] |
                           ((uint32_t)saved_pid_buf[SHTP_HEADER_LEN + 5] << 8) |
                           ((uint32_t)saved_pid_buf[SHTP_HEADER_LEN + 6] << 16) |
                           ((uint32_t)saved_pid_buf[SHTP_HEADER_LEN + 7] << 24);
        ESP_LOGI(TAG, "BNO085: Product ID 0x%08"PRIx32, part_no);
    } else {
        ESP_LOGI(TAG, "BNO085: Product ID response received (short, n=%d)", n);
    }

    // Drain remaining FRS/initialization data before enabling reports.
    // The BNO085 sends many ch=0 FRS records after reset; they must be consumed first.
    int frs_count = 0;
    for (int frs_i = 0; frs_i < 200; frs_i++) {
        if (cfg->int_pin >= 0 && gpio_get_level(cfg->int_pin) != 0) {
            // INT HIGH for this check - wait a bit and re-check to confirm truly idle
            vTaskDelay(pdMS_TO_TICKS(50));
            if (cfg->int_pin >= 0 && gpio_get_level(cfg->int_pin) != 0) {
                break;  // INT stayed HIGH for 50ms, BNO085 is truly idle
            }
        }
        int pkt_n = bno085_read_packet(rx_buf, sizeof(rx_buf));
        if (pkt_n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        frs_count++;
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

    uint8_t rx_buf[SHTP_MAX_PACKET];
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

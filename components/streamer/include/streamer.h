/**
 * @file streamer.h
 * @brief Real-time data streamer for PC-side live visualization.
 *
 * Two transport backends share one fan-out path:
 *   - USB Serial/JTAG (primary): always-on, low-latency, low-loss
 *   - Wi-Fi SoftAP + TCP (secondary): single client, drop-on-overflow
 *
 * Architecture:
 *   acquisition tasks (Core 0) ──► streamer_publish_*()  (non-blocking)
 *                                          │
 *                                          ├──► usb_queue ──► usb task (Core 1)
 *                                          └──► wifi_queue ─► wifi task (Core 1)
 *
 * Frame format on the wire (16 bytes, little-endian):
 *
 *   offset  size  field
 *   ----------------------------------------
 *      0     2    magic           = 0xAA55
 *      2     1    sensor_id       (matches SensorContext_t.id)
 *      3     1    axis            (0=scalar, 1=x, 2=y, 3=z)
 *      4     4    seq             (monotonic counter per packet)
 *      8     4    timestamp_ms    (ESP32 uptime, ms)
 *     12     4    value           (float32, processed value)
 */
#ifndef STREAMER_H
#define STREAMER_H

#include <stdint.h>
#include <stdbool.h>
#include "data_types.h"

// ==================== Compile-time switches ====================
// Override either of these via:
//   1) Editing this header
//   2) idf.py build -DSTREAMER_USB=1 -DSTREAMER_WIFI=0
//   3) Adding -DSTREAMER_ENABLE_USB=0 to CFLAGS
//
// Defaults: USB on (cheap), Wi-Fi off (Wi-Fi radio adds ~100 mA).
#ifndef STREAMER_ENABLE_USB
#define STREAMER_ENABLE_USB  1
#endif
#ifndef STREAMER_ENABLE_WIFI
#define STREAMER_ENABLE_WIFI 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define STREAM_MAGIC_LE 0xAA55u  // bytes on the wire: 0x55 then 0xAA

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t  sensor_id;
    uint8_t  axis;
    uint32_t seq;
    uint32_t timestamp_ms;
    float    value;
} stream_pkt_t;

/**
 * @brief Initialize streamer (USB CDC + Wi-Fi SoftAP).
 * Safe to call once after data_types_init(); failures of the Wi-Fi side
 * do not block USB streaming.
 */
void streamer_init(void);

/**
 * @brief Non-blocking publish APIs called from acquisition tasks.
 * fast: 3-axis (emits 3 packets), medium/slow: scalar (emits 1).
 * If both backend queues are full, packet is dropped.
 */
void streamer_publish_fast(const fast_data_record_t *rec);
void streamer_publish_medium(const medium_data_record_t *rec);
void streamer_publish_slow(const slow_data_record_t *rec);

uint32_t streamer_get_drops(void);
uint32_t streamer_get_sent_usb(void);
uint32_t streamer_get_sent_wifi(void);

#ifdef __cplusplus
}
#endif

#endif // STREAMER_H

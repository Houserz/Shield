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
 * Frame format on the wire (20 bytes, little-endian):
 *
 *   offset  size  field
 *   ----------------------------------------
 *      0     2    magic           = 0xAA55
 *      2     1    sensor_id       (matches SensorContext_t.id)
 *      3     1    axis            (0=scalar, 1=x, 2=y, 3=z)
 *      4     1    kind            (0=raw, 1=processed)
 *      5     1    flags           DATA_FLAG_* bits from data_types.h
 *      6     2    reserved
 *      8     4    seq             (monotonic counter per packet)
 *     12     4    timestamp_ms    (ESP32 uptime, ms)
 *     16     4    value           (float32)
 */
#ifndef STREAMER_H
#define STREAMER_H

#include <stdint.h>
#include <stdbool.h>
#include "data_types.h"
#include "streamer_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STREAM_MAGIC_LE 0xAA55u  // bytes on the wire: 0x55 then 0xAA

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t  sensor_id;
    uint8_t  axis;
    uint8_t  kind;
    uint8_t  flags;
    uint16_t reserved;
    uint32_t seq;
    uint32_t timestamp_ms;
    float    value;
} stream_pkt_t;

#ifdef __cplusplus
static_assert(sizeof(stream_pkt_t) == 20, "stream_pkt_t must stay 20 bytes");
#else
_Static_assert(sizeof(stream_pkt_t) == 20, "stream_pkt_t must stay 20 bytes");
#endif

/**
 * @brief Initialize streamer (USB CDC + Wi-Fi SoftAP).
 * Safe to call once after data_types_init(); failures of the Wi-Fi side
 * do not block USB streaming.
 */
void streamer_init(void);

/**
 * @brief Non-blocking publish APIs called from acquisition tasks.
 * A vector record emits one packet per axis; scalar emits one packet.
 * If both backend queues are full, packet is dropped.
 */
void streamer_publish_record(const sensor_data_record_v2_t *rec);
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

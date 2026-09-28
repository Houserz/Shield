/**
 * @file data_types.h
 * @brief Data type definitions for the data acquisition system
 *
 * Defines core data structures including sensor data packets, queue messages, etc.
 *
 * REVIEW-1 CHANGES (C-11 items 4 and 7): meta.json gains a board identifier
 * (factory MAC), the firmware git hash, the FreeRTOS tick rate, the BNO085
 * report configuration and calibration-config read-back, whether the RTC
 * was set, and at close the tier-file sizes and close status. Record layouts
 * are unchanged, so the released decoder still reads the new data.
 */

#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>   /* size_t */

// ==================== Queue Configuration ====================
#define FAST_QUEUE_SIZE     300     // 1kHz × 3 sensors (IMU, Vibration, Microphone) × 100ms buffer
#define MEDIUM_QUEUE_SIZE   40      // 200Hz × 2 sensors (Current, Photodiode) × 100ms buffer
#define SLOW_QUEUE_SIZE     10      // 50Hz × 2 sensors × 100ms buffer

// ==================== Binary Data Packet Formats (UNCHANGED) ====================

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;    // Timestamp (milliseconds)
    uint8_t sensor_id;        // Sensor ID
    uint8_t reserved[3];      // Padding for alignment
    float data[3];            // Sensor data (scalar: data[0] only; vector: x,y,z)
} fast_data_record_t;

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;    // Timestamp (milliseconds)
    uint8_t sensor_id;        // Sensor ID (2=Current, 6=Photodiode)
    uint8_t reserved[3];      // Padding for alignment
    float data;               // Sensor data (current in A, or photodiode voltage/level)
} medium_data_record_t;

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;    // Timestamp (milliseconds)
    uint8_t sensor_id;        // Sensor ID (3=Pressure, 4=Temperature)
    uint8_t reserved[3];      // Padding for alignment
    float data;               // Sensor data
} slow_data_record_t;

// ==================== Queue Message Types ====================

typedef enum {
    QUEUE_MSG_DATA,
    QUEUE_MSG_FLUSH,
    QUEUE_MSG_STOP
} queue_msg_type_t;

typedef struct { queue_msg_type_t type; fast_data_record_t data; }   fast_queue_msg_t;
typedef struct { queue_msg_type_t type; medium_data_record_t data; } medium_queue_msg_t;
typedef struct { queue_msg_type_t type; slow_data_record_t data; }   slow_queue_msg_t;

// ==================== System State ====================

typedef enum {
    DAQ_STATE_IDLE,
    DAQ_STATE_INITIALIZING,
    DAQ_STATE_RUNNING,
    DAQ_STATE_STOPPING,
    DAQ_STATE_ERROR
} daq_state_t;

typedef struct {
    uint32_t fast_samples;
    uint32_t medium_samples;
    uint32_t slow_samples;
    uint32_t queue_overruns;
    uint32_t sd_errors;
    uint32_t duration_ms;
} daq_statistics_t;

// ==================== Review-1: provenance written into meta.json ====================

#ifndef FW_GIT_HASH
#define FW_GIT_HASH "unknown"   /* set by main/CMakeLists.txt from `git describe --always --dirty` */
#endif

typedef struct {
    char     board_mac[18];          // "aa:bb:cc:dd:ee:ff" from esp_read_mac(ESP_MAC_BASE)
    char     firmware_git[48];       // FW_GIT_HASH
    uint32_t tick_hz;                // CONFIG_FREERTOS_HZ
    uint32_t accel_interval_us;      // BNO085 report intervals as requested by the drivers
    uint32_t gyro_interval_us;
    uint32_t mag_interval_us;
    const char *accel_report;        // e.g. "SH2_ACCELEROMETER (0x01, calibrated, gravity included)"
    const char *gyro_report;         // e.g. "SH2_GYROSCOPE_UNCALIBRATED (0x07)"
    const char *mag_report;          // e.g. "SH2_MAGNETIC_FIELD_UNCALIBRATED (0x0F)"
    int      bno085_cal_config;      // sh2_getCalConfig() bitmask, or -1 if not read back
    int      sd_selftest_attempts;   // mount attempts used at boot
    bool     rtc_set;                // true if time(NULL) was set by SETTIME before the session
} meta_provenance_t;

// ==================== Helper Functions ====================

uint32_t get_timestamp_ms(void);
void data_types_init(void);

/**
 * @brief Fill the provenance block that metadata_create() writes (call once before the first session)
 */
void metadata_set_provenance(const meta_provenance_t *prov);

/**
 * @brief Fill board_mac from the chip's factory MAC
 */
void metadata_read_board_mac(char *out, size_t out_len);

// ==================== Metadata Management ====================

bool metadata_create(const char *filepath, const char *run_id);
bool metadata_update_statistics(const char *filepath, const daq_statistics_t *stats);

/**
 * @brief Mark run as finished: end time, tier-file sizes read back from the card, close status
 */
bool metadata_finalize(const char *filepath, uint32_t fast_bytes, uint32_t medium_bytes,
                       uint32_t slow_bytes, bool close_ok, uint32_t marker_count);

#endif // DATA_TYPES_H
